/*
 *	mqtt_utk.c
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	Runs entirely on the single processor-1-owned task that already owns
 *	the CYW43439 and the rest of lwIP (docs/RELEASE.md safety rule 6):
 *	cyw43_utk_task() (cyw43_utk.c) calls mqtt_utk_start() once the link
 *	is up.  Every timer below rides the same sys_check_timeouts() polling
 *	that task already does for DHCP and TCP.
 *
 *	IS11 changes (TEAM.md IF-01/IF-03, agreed with the team, design 6.1):
 *	  - telemetry: every MQTT_UTK_PUMP_MS the encoded frames that
 *	    comms_task left in its outbox are published on MQTT_TOPIC_TLM,
 *	    QoS 0 (comms_bridge_take_frame, TMO_POL inside)
 *	  - commands: MQTT_TOPIC_CMD is subscribed at QoS 1; each complete
 *	    message is handed raw to comms_bridge_on_command(), which
 *	    forwards STOP at once to ctrl_task
 *	  - status: "online" (retained) on MQTT_TOPIC_STATUS after connecting,
 *	    "offline" as the Last Will, so the dashboard sees the car drop
 *	  - reconnect with back-off 1, 2, 4 s (resume within 10 s, NFR4)
 *	The bridge functions are weak here, so the library still builds and
 *	runs (publishing nothing) without the IS11 application.
 *
 *	This file does not include tk/tkernel.h or any pico-sdk/cyw43 header,
 *	so it isn't subject to the size_t workaround of cyw43_utk.c.
 */

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include "../../../../app_program/subsystem1/comms_bridge.h"
#include <tm/tmonitor.h>

/* Topics: override in config/wifi_secrets.h if needed */
#ifndef MQTT_TOPIC_TLM
#define MQTT_TOPIC_TLM		"is11/car/tlm"
#endif
#ifndef MQTT_TOPIC_CMD
#define MQTT_TOPIC_CMD		"is11/car/cmd"
#endif
#ifndef MQTT_TOPIC_STATUS
#define MQTT_TOPIC_STATUS	"is11/car/status"
#endif
#ifndef MQTT_TOPIC_LOG
#define MQTT_TOPIC_LOG		"is11/car/log"
#endif
#ifndef MQTT_TOPIC_IR
#define MQTT_TOPIC_IR		"is11/car/ir"
#endif
#ifndef MQTT_TOPIC_US
#define MQTT_TOPIC_US		"is11/car/us"
#endif
#ifndef MQTT_TOPIC_SWEEP
#define MQTT_TOPIC_SWEEP	"is11/car/sweep"
#endif
#ifndef MQTT_TOPIC_BC
#define MQTT_TOPIC_BC		"is11/car/bc"	/* last barcode read widths */
#endif

#define MQTT_UTK_PUMP_MS	100	/* outbox check; comms fills it at 5 Hz */
#define MQTT_UTK_PUMP_MAX	4	/* frames per pump: bounded poll time   */
#define MQTT_UTK_KEEPALIVE_S	30	/* tolerant of a few lost packets */
#define MQTT_UTK_RETRY_MIN_MS	1000
#define MQTT_UTK_RETRY_MAX_MS	4000
#define MQTT_UTK_RX_TOPIC_MAX	64
#define MQTT_UTK_RX_DATA_MAX	32
#define MQTT_UTK_FRAME_MAX	64
#define MQTT_UTK_LOG_MAX	200	/* matches APP_LOG_LEN */
#define MQTT_UTK_LOG_PER_PUMP	6

/* Weak defaults: replaced by app_program/subsystem1/comms_bridge.c */
__attribute__((weak)) int comms_bridge_take_frame(unsigned char *buf, int max)
{
	(void)buf; (void)max;
	return 0;
}
__attribute__((weak)) void comms_bridge_on_command(const unsigned char *data, int len)
{
	(void)data; (void)len;
}
__attribute__((weak)) void comms_bridge_set_connected(int connected)
{
	(void)connected;
}
__attribute__((weak)) int comms_bridge_peek_log(unsigned char *buf, int max)
{
	(void)buf; (void)max;
	return 0;
}
__attribute__((weak)) void comms_bridge_pop_log(void)
{
}
__attribute__((weak)) int comms_bridge_ir_text(unsigned char *buf, int max)
{
	(void)buf; (void)max;
	return 0;
}
__attribute__((weak)) int comms_bridge_us_text(unsigned char *buf, int max)
{
	(void)buf; (void)max;
	return 0;
}
__attribute__((weak)) int comms_bridge_sweep_text(unsigned char *buf, int max)
{
	(void)buf; (void)max;
	return 0;
}
__attribute__((weak)) int comms_bridge_bc_text(unsigned char *buf, int max)
{
	(void)buf; (void)max;
	return 0;
}

static mqtt_client_t	*client;
static ip_addr_t	broker_addr;
static u8_t		connecting;
static u32_t		retry_ms = MQTT_UTK_RETRY_MIN_MS;

static u8_t		rx_is_cmd;
static unsigned char	rx_data[MQTT_UTK_RX_DATA_MAX];
static u16_t		rx_len;
static u8_t		rx_truncated;

static u32_t		tx_frames;
static u32_t		tx_failed;

static int topic_equals(const char *a, const char *b)
{
	while ((*a != '\0') && (*a == *b)) {
		a++;
		b++;
	}
	return (*a == *b) ? 1 : 0;
}

static void incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
	(void)arg;
	(void)tot_len;

	rx_is_cmd = (u8_t)topic_equals(topic, MQTT_TOPIC_CMD);
	rx_len = 0;
	rx_truncated = 0;
}

static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	u16_t	i;

	(void)arg;

	for (i = 0; i < len; i++) {
		if (rx_len < MQTT_UTK_RX_DATA_MAX) {
			rx_data[rx_len++] = data[i];	/* raw binary frame */
		} else {
			rx_truncated = 1;
		}
	}
	if ((flags & MQTT_DATA_FLAG_LAST) != 0) {
		if (rx_is_cmd) {
			/* A truncated frame is passed with a bad length: rejected */
			comms_bridge_on_command(rx_data, rx_truncated ? -1 : (int)rx_len);
		}
	}
}

static void subscribe_done_cb(void *arg, err_t result)
{
	(void)arg;
	if (result == ERR_OK) {
		tm_printf((UB*)"mqtt: subscribed to " MQTT_TOPIC_CMD "\n");
	} else {
		tm_printf((UB*)"mqtt: subscribe failed, err=%d\n", (int)result);
	}
}

static void publish_done_cb(void *arg, err_t result)
{
	(void)arg;
	if (result != ERR_OK) {
		tx_failed++;
	}
}

/* Publish what comms_task has encoded.  Bounded per call, so the radio
   poll loop never spends long here. */
static void pump_timeout(void *arg)
{
	unsigned char	frame[MQTT_UTK_FRAME_MAX];
	int		n;
	int		count;
	err_t		err;

	(void)arg;

	if (mqtt_client_is_connected(client)) {
		for (count = 0; count < MQTT_UTK_PUMP_MAX; count++) {
			n = comms_bridge_take_frame(frame, (int)sizeof(frame));
			if (n <= 0) {
				break;
			}
			err = mqtt_publish(client, MQTT_TOPIC_TLM, frame, (u16_t)n,
					   0 /* qos */, 0 /* retain */,
					   publish_done_cb, NULL);
			if (err == ERR_OK) {
				tx_frames++;
			} else {
				tx_failed++;	/* request queue full: frame dropped */
				break;
			}
		}
		/* Live IR values every pump (10 Hz), dropped if the buffer is full */
		{
			unsigned char	ir[80];

			n = comms_bridge_ir_text(ir, (int)sizeof(ir));
			if (n > 0) {
				(void)mqtt_publish(client, MQTT_TOPIC_IR, ir, (u16_t)n,
						   0 /* qos */, 0 /* retain */, NULL, NULL);
			}
			n = comms_bridge_us_text(ir, (int)sizeof(ir));
			if (n > 0) {
				(void)mqtt_publish(client, MQTT_TOPIC_US, ir, (u16_t)n,
						   0 /* qos */, 0 /* retain */, NULL, NULL);
			}
		}
		{
			static unsigned char	sweep[300];	/* sweep text, then barcode widths */

			n = comms_bridge_sweep_text(sweep, (int)sizeof(sweep));
			if (n > 0) {
				(void)mqtt_publish(client, MQTT_TOPIC_SWEEP, sweep, (u16_t)n,
						   0 /* qos */, 1 /* retain: last sweep */, NULL, NULL);
			}
			n = comms_bridge_bc_text(sweep, (int)sizeof(sweep));
			if (n > 0) {
				(void)mqtt_publish(client, MQTT_TOPIC_BC, sweep, (u16_t)n,
						   0 /* qos */, 1 /* retain: last read */, NULL, NULL);
			}
		}
		/* Log lines: kept in the app ring until MQTT accepts them */
		for (count = 0; count < MQTT_UTK_LOG_PER_PUMP; count++) {
			unsigned char	line[MQTT_UTK_LOG_MAX];

			n = comms_bridge_peek_log(line, (int)sizeof(line));
			if (n <= 0) {
				break;
			}
			err = mqtt_publish(client, MQTT_TOPIC_LOG, line, (u16_t)n,
					   0 /* qos */, 0 /* retain */, NULL, NULL);
			if (err != ERR_OK) {
				break;		/* output buffer full: retry next pump */
			}
			comms_bridge_pop_log();
		}
	}
	sys_timeout(MQTT_UTK_PUMP_MS, pump_timeout, NULL);
}

static void retry_timeout(void *arg)
{
	(void)arg;
	connecting = 0;
	mqtt_utk_start();
}

static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
	(void)c;
	(void)arg;

	if (status == MQTT_CONNECT_ACCEPTED) {
		err_t	err;

		connecting = 0;
		retry_ms = MQTT_UTK_RETRY_MIN_MS;
		comms_bridge_set_connected(1);
		tm_printf((UB*)"mqtt: connected to " MQTT_BROKER_IP "\n");

		mqtt_set_inpub_callback(client, incoming_publish_cb,
					incoming_data_cb, NULL);
		err = mqtt_subscribe(client, MQTT_TOPIC_CMD, 1 /* qos */,
				     subscribe_done_cb, NULL);
		if (err != ERR_OK) {
			tm_printf((UB*)"mqtt: mqtt_subscribe() failed, err=%d\n", (int)err);
		}
		(void)mqtt_publish(client, MQTT_TOPIC_STATUS, "online", 6,
				   1 /* qos */, 1 /* retain */, publish_done_cb, NULL);

		/* Exactly one pump chain alive at a time */
		sys_untimeout(pump_timeout, NULL);
		sys_timeout(MQTT_UTK_PUMP_MS, pump_timeout, NULL);
	} else {
		comms_bridge_set_connected(0);
		sys_untimeout(pump_timeout, NULL);
		tm_printf((UB*)"mqtt: connection status %d, retrying in %d ms\n",
			  (int)status, (int)retry_ms);
		sys_timeout(retry_ms, retry_timeout, NULL);
		retry_ms = (retry_ms * 2U > MQTT_UTK_RETRY_MAX_MS) ?
			   MQTT_UTK_RETRY_MAX_MS : retry_ms * 2U;
	}
}

void mqtt_utk_start(void)
{
	struct mqtt_connect_client_info_t	client_info = {0};
	err_t					err;

	if (connecting || (client != NULL && mqtt_client_is_connected(client))) {
		return;
	}

	if (client == NULL) {
		client = mqtt_client_new();
		if (client == NULL) {
			tm_printf((UB*)"mqtt: mqtt_client_new() failed\n");
			return;
		}
	}

	if (!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr))) {
		tm_printf((UB*)"mqtt: bad MQTT_BROKER_IP\n");
		return;
	}
	IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);

	client_info.client_id = MQTT_CLIENT_ID;
	client_info.keep_alive = MQTT_UTK_KEEPALIVE_S;
	client_info.will_topic = MQTT_TOPIC_STATUS;	/* dashboard sees the drop */
	client_info.will_msg = "offline";
	client_info.will_qos = 1;
	client_info.will_retain = 1;

	connecting = 1;
	err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
				  connection_cb, NULL, &client_info);
	if (err != ERR_OK) {
		tm_printf((UB*)"mqtt: mqtt_client_connect() failed, err=%d\n", (int)err);
		connecting = 0;
	}
}
