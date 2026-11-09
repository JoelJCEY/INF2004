/*
 *	comms_bridge.c
 *	Subsystem 1 -- code that runs inside the kernel's Wi-Fi service task
 *	(called from mqtt_utk.c).  TMO_POL only; no mutex is ever taken.
 */

#include "subsystem1.h"
#include "comms_bridge.h"
#include "../subsystem5/subsystem5.h"	/* IF-03 mbf_remote, flg_sys */
#include "../subsystem3/subsystem3.h"	/* live IR values */
#include "../subsystem5/scan_profile.h"	/* SCAN_POSITIONS */

EXPORT volatile UW comms_mqtt_connected;	/* writer: Wi-Fi task (bridge) */
EXPORT volatile UW comms_rx_rejected;
/* IF-15 barcode map (app.h).  Only writer: comms_bridge_on_command() */
EXPORT volatile UW app_barcode_map = APP_BARCODE_MAP_DEFAULT;
/* Barcode test mode (app.h).  Only writer: comms_bridge_on_command() */
EXPORT volatile UW app_barcode_test = 0U;

LOCAL void on_map_frame(const unsigned char *data, int len)
{
	UW	map;

	if (comms_decode_map((const UB*)data, (INT)len, &map) != E_OK) {
		comms_rx_rejected++;
		app_log((UB*)"comms: barcode map REJECTED: need 4 different letters A-Z or 0-9\n");
		return;
	}
	app_barcode_map = map;		/* one aligned 32-bit store */
	app_log((UB*)"comms: barcode map set: forward '%c', u-turn '%c', left '%c', right '%c'\n",
		  (INT)(map & 0xFFU), (INT)((map >> 8) & 0xFFU),
		  (INT)((map >> 16) & 0xFFU), (INT)((map >> 24) & 0xFFU));
}

int comms_bridge_take_frame(unsigned char *buf, int max)
{
	UB	frame[WIRE_TLM_LEN];
	INT	n;
	INT	i;

	if ((mbf_outbox <= 0) || (max < WIRE_TLM_LEN)) {
		return 0;
	}
	n = tk_rcv_mbf(mbf_outbox, frame, TMO_POL);
	if (n <= 0) {
		return 0;
	}
	for (i = 0; i < n; i++) {
		buf[i] = frame[i];
	}
	return (int)n;
}

void comms_bridge_on_command(const unsigned char *data, int len)
{
	REMOTE_MSG	m;

	/* Barcode map (7 bytes) shares the command topic; START/STOP/RESET
	   (3 bytes) below are handled exactly as before */
	if ((len > 0) && comms_is_map_frame((const UB*)data, (INT)len)) {
		on_map_frame(data, len);
		return;
	}
	if ((len == WIRE_TEST_LEN) && (data[1] == (UB)WIRE_OP_BC_TEST)) {
		UW	on;

		if (comms_decode_bc_test((const UB*)data, (INT)len, &on) == E_OK) {
			app_barcode_test = on;		/* one aligned store */
			app_log((UB*)"comms: barcode TEST mode %s\n",
				  on ? "ON: the car will not drive, slide a barcode under the left sensor"
				     : "OFF: normal driving");
		} else {
			comms_rx_rejected++;
		}
		return;
	}
	if ((mbf_remote <= 0) || (comms_decode_cmd(data, (INT)len, &m) != E_OK)) {
		comms_rx_rejected++;
		app_log((UB*)"comms: command rejected (len %d)\n", len);
		return;
	}
	/* STOP is the latency-critical message: forwarded at once, from
	   here, without waiting for comms_task's next cycle (6.1) */
	if (tk_snd_mbf(mbf_remote, &m, sizeof(m), TMO_POL) < E_OK) {
		comms_rx_rejected++;
		return;
	}
	(void)tk_set_flg(flg_sys, EVF_REMOTE);
}

void comms_bridge_set_connected(int connected)
{
	comms_mqtt_connected = (connected != 0) ? 1U : 0U;
}

/* Dashboard log (is11/car/log): oldest line first, removed only after
   MQTT accepted it, so nothing is lost while the link is down. */
int comms_bridge_peek_log(unsigned char *buf, int max)
{
	return (int)app_log_peek((UB*)buf, (INT)max);
}

void comms_bridge_pop_log(void)
{
	app_log_pop();
}

/* Live IR snapshot for the dashboard (is11/car/ir, 10 Hz):
   "rawL rawC rawR normL normC normR err barsTotal barsPass barsLastPass readerN recross encRejects"
   (the last six: lines scanned by IR_L, see subsystem3.h) */
int comms_bridge_ir_text(unsigned char *buf, int max)
{
	UW	bp = line_bar_pass;

	if (max < 80) {
		return 0;
	}
	return (int)tm_sprintf((UB*)buf, (const UB*)"%d %d %d %d %d %d %d %d %d %d %d %d %d",
			       (INT)line_raw_avg[0], (INT)line_raw_avg[1], (INT)line_raw_avg[2],
			       (INT)line_norm[0], (INT)line_norm[1], (INT)line_norm[2],
			       (INT)line_unpack_err(line_latest),
			       (INT)(line_bar_total & 0xFFFFU), (INT)(bp & 0xFFFFU),
			       (INT)(bp >> 16), (INT)line_reader_n,
			       (INT)(line_bar_recross & 0xFFFFU),
			       (INT)(line_enc_rejects & 0xFFFFU));
}

/* Live ultrasonic for the dashboard (is11/car/us, 10 Hz):
   "state servo_deg mm"  state: 0 idle 1 watching 2 paused on hump
   3 coarse sweep 4 fine sweep 5 guard; mm 0 = no echo, 1000 = far */
int comms_bridge_us_text(unsigned char *buf, int max)
{
	UW	w = scan_status_word;

	if (max < 32) {
		return 0;
	}
	return (int)tm_sprintf((UB*)buf, (const UB*)"%d %d %d",
			       (INT)((w >> 5) & 0x7U),
			       (INT)SERVO_IDX_TO_DEG(w & 0x1FU),
			       (INT)((w >> 16) & 0x1FFFU));
}

/* Last sweep, only when a new one is ready (is11/car/sweep):
   "seq d30 d35 ... d150" */
/* Last barcode read, only when a new one finished (is11/car/bc, retained):
   "seq elements letter reason w1 w2 ..."  widths in 0.1 mm, bar first;
   letter 0 = rejected.  Copied under line_task's sequence (seqlock). */
int comms_bridge_bc_text(unsigned char *buf, int max)
{
	static UW	sent = 0;
	UH		w[LINE_BC_SHOW_MAX];
	UW		s1 = line_bc_seq;
	UW		s2;
	UW		info;
	INT		n, k, i;

	if ((s1 == sent) || ((s1 & 1U) != 0U) || (max < (LINE_BC_SHOW_MAX * 6 + 24))) {
		return 0;
	}
	info = line_bc_info;
	k = (INT)(info & 0xFFU);
	if (k > LINE_BC_SHOW_MAX) {
		k = LINE_BC_SHOW_MAX;
	}
	for (i = 0; i < k; i++) {
		w[i] = line_bc_w01mm[i];
	}
	s2 = line_bc_seq;
	if (s2 != s1) {
		return 0;		/* written meanwhile: next pump */
	}
	sent = s1;
	n = tm_sprintf((UB*)buf, (const UB*)"%d %d %d %d", (INT)(s1 / 2U),
		       (INT)(info & 0xFFU), (INT)((info >> 8) & 0xFFU),
		       (INT)((info >> 16) & 0xFFU));
	for (i = 0; i < k; i++) {
		n += tm_sprintf((UB*)&buf[n], (const UB*)" %d", (INT)w[i]);
	}
	return (int)n;
}

int comms_bridge_sweep_text(unsigned char *buf, int max)
{
	static UW	sent = 0;
	UW		seq = scan_sweep_seq;
	INT		n;
	INT		i;

	if ((seq == sent) || (max < (SCAN_POSITIONS * 6 + 12))) {
		return 0;
	}
	sent = seq;
	n = tm_sprintf((UB*)buf, (const UB*)"%d", (INT)seq);
	for (i = 0; i < SCAN_POSITIONS; i++) {
		n += tm_sprintf((UB*)&buf[n], (const UB*)" %d", (INT)scan_last_sweep[i]);
	}
	return (int)n;
}
