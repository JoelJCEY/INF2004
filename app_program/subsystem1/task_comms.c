/*
 *	task_comms.c
 *	Subsystem 1 -- comms_task (COMPLETE VERSION), lowest application
 *	priority.  Packs each snapshot and keeps the 16 most recent packed
 *	snapshots in the outbox while the broker is unreachable.
 */

#include "subsystem1.h"
#include "../subsystem5/subsystem5.h"

#if TM_WIFI_CYW43
#include "cyw43_utk.h"
#endif

EXPORT volatile UW	s1_rx_count = 0U;
EXPORT volatile UW	s1_dropped = 0U;
EXPORT volatile BOOL	s1_wifi_up = FALSE;

LOCAL BOOL s1_read_link(void)
{
#if TM_WIFI_CYW43
	T_CYW43_UTK_STATUS	st;

	cyw43_utk_get_status(&st);
#if defined(TM_NET_LWIP) && TM_NET_LWIP
	return (st.ip_addr != 0U) ? TRUE : FALSE;
#else
	return ((st.init_complete != 0U) && (st.init_result == 0)) ? TRUE : FALSE;
#endif
#else
	return FALSE;
#endif
}

/* Put one packed snapshot in the outbox; if full, drop the oldest. */
LOCAL void s1_outbox_put(const UB *p)
{
	UB	old[S1_PACKED_BYTES];

	if (tk_snd_mbf(mbf_outbox, (void *)p, S1_PACKED_BYTES, TMO_POL) != E_OK) {
		(void)tk_rcv_mbf(mbf_outbox, old, TMO_POL);
		s1_dropped++;
		(void)tk_snd_mbf(mbf_outbox, (void *)p, S1_PACKED_BYTES, TMO_POL);
	}
}

EXPORT void comms_task(INT stacd, void *exinf)
{
	S1_TELEMETRY	t;
	UB		packed[S1_PACKED_BYTES];
	UW		last_link_ms = 0U;
	BOOL		was_up = FALSE;

	(void)stacd;
	(void)exinf;

	while (1) {
		INT	sz = tk_rcv_mbf(mbf_telemetry, &t, 1000);
		UW	now = s5_now_ms();

		if ((now - last_link_ms) >= 1000U) {
			BOOL	up = s1_read_link();
			last_link_ms = now;
			if (up != was_up) {
				tm_printf((UB *)"[S1] wifi link %s\n", (up == TRUE) ? "UP" : "DOWN");
#if defined(TM_NET_LWIP) && TM_NET_LWIP
				if ((up == FALSE) && (was_up == TRUE)) {
					s5_raise_fault(FAULT_WIFI_LOSS);	/* reported only */
				}
#endif
			}
			was_up = up;
			s1_wifi_up = up;
		}
		if (sz != (INT)sizeof(t)) {
			continue;
		}
		s1_rx_count++;
		t.wifi_up = (s1_wifi_up == TRUE) ? 1 : 0;
		(void)s1_pack(&t, packed);
		s1_outbox_put(packed);

		if (S1_PRINT_CSV != 0) {
			tm_printf((UB *)"T,%u,%d,%d,%c,0x%02x,%d,%d,%d,%d,%d,%d,%d\n",
				  (UINT)t.t_ms, (INT)t.state, (INT)t.last_barcode,
				  (t.last_letter != 0) ? (INT)t.last_letter : (INT)'-',
				  (UINT)t.fault_mask, (INT)t.speed_l_mm_s, (INT)t.speed_r_mm_s,
				  (INT)t.distance_mm, (INT)t.line_err, (INT)t.hump_peak_mm,
				  (INT)t.obstacle_mm, (INT)t.motion_event);
		}
	}
}
