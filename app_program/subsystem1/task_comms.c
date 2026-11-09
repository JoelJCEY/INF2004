/*
 *	task_comms.c
 *	Subsystem 1 -- comms_task (statechart 6.1), lowest priority.
 *
 *	Every 200 ms:
 *	  - take the newest finished snapshot from the Telemetry Queue
 *	    (ctrl_task is its only writer), encode it (wire format v1) and
 *	    put the frame in the outbox; the MQTT client inside the Wi-Fi
 *	    task publishes the outbox at QoS 0
 *	  - while offline the outbox keeps the 16 NEWEST frames: when full,
 *	    the oldest is dropped and counted (~576 bytes instead of ~3 KB
 *	    of text, design 10.5)
 *	  - follow the link state; losing Wi-Fi is reported (FLT_WIFI) but
 *	    never stops the car (NFR4, T9)
 *	comms_task never takes a mutex and every kernel call is TMO_POL or a
 *	plain delay, so Wi-Fi can never hold up control tasks (NFR3).
 */

#include "subsystem1.h"
#include "../subsystem5/subsystem5.h"	/* app_report_fault */
#include "cyw43_utk.h"			/* read-only radio status */

#define CYW43_LINK_UP_VALUE	3	/* CYW43_LINK_UP in cyw43.h */

EXPORT volatile UW comms_state_word;
EXPORT volatile UW comms_outbox_dropped;

LOCAL COMMS_STATE link_state(void)
{
	T_CYW43_UTK_STATUS	st;

	cyw43_utk_get_status(&st);
	if (!st.ready) {
		return CS_DISCONNECTED;
	}
#if TM_NET_LWIP
	if (st.link_status < 0) {
		return CS_WAITING_TO_RETRY;		/* join failed: driver retries */
	}
	if (st.link_status != CYW43_LINK_UP_VALUE) {
		return CS_WIFI_CONNECTING;
	}
	return (comms_mqtt_connected != 0U) ? CS_ONLINE : CS_BROKER_CONNECTING;
#else
	return CS_DISCONNECTED;			/* no TCP/IP in this build */
#endif
}

LOCAL void outbox_put(const UB *frame, INT len)
{
	UB	old[WIRE_TLM_LEN];

	if (tk_snd_mbf(mbf_outbox, frame, len, TMO_POL) == E_OK) {
		return;
	}
	/* Full: drop the oldest, keep the newest */
	(void)tk_rcv_mbf(mbf_outbox, old, TMO_POL);
	comms_outbox_dropped++;
	(void)tk_snd_mbf(mbf_outbox, frame, len, TMO_POL);
}

EXPORT void comms_task(INT stacd, void *exinf)
{
	TLM_SNAPSHOT	snap;
	TLM_SNAPSHOT	newest;
	BOOL		have;
	UB		frame[WIRE_TLM_LEN];
	UB		seq = 0;
	INT		len;
	COMMS_STATE	st;
	COMMS_STATE	prev = CS_DISCONNECTED;

	(void)stacd; (void)exinf;

	while (1) {
		tk_dly_tsk(S1_PERIOD_MS);

		/* Newest finished snapshot (older ones are superseded) */
		have = FALSE;
		while (tk_rcv_mbf(mbf_telemetry, &snap, TMO_POL) == (INT)sizeof(snap)) {
			newest = snap;
			have = TRUE;
		}
		if (have) {
			len = comms_encode_tlm(&newest, seq++, frame);
			outbox_put(frame, len);
		}

		st = link_state();
		comms_state_word = (UW)st;
		if (st != prev) {
			app_log((UB*)"comms: link state %d -> %d\n", prev, st);
			if (prev == CS_ONLINE) {
				app_report_fault(FLT_WIFI);	/* reported only */
			}
			prev = st;
		}
	}
}
