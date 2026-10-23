/*
 *	task_comms.c
 *	Subsystem 1 -- comms_task (BASIC VERSION).
 *
 *	Waits for telemetry snapshots (1 s timeout) and prints each as a CSV
 *	line, so a laptop can log the run over USB.  Header line:
 *	  T,t_ms,seq,state,barcode,letter,faults,vL,vR,encL,encR,dist,lineErr,
 *	  lineFlags,hump,humpPeak,obstacle,wifi
 *	The Wi-Fi link state is read once a second (read-only status call).
 */

#include "subsystem1.h"

#if TM_WIFI_CYW43
#include "cyw43_utk.h"
#endif

EXPORT volatile UW	s1_rx_count = 0U;
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

EXPORT void comms_task(INT stacd, void *exinf)
{
	S1_TELEMETRY	t;
	UW		last_link_ms = 0U;
	BOOL		header_done = FALSE;
	SYSTIM		now;

	(void)stacd;
	(void)exinf;

	while (1) {
		INT	sz = tk_rcv_mbf(mbf_telemetry, &t, 1000);

		(void)tk_get_otm(&now);
		if ((UW)(now.lo - last_link_ms) >= 1000U) {
			BOOL	up = s1_read_link();
			if (up != s1_wifi_up) {
				tm_printf((UB *)"[S1] wifi link %s\n", (up == TRUE) ? "UP" : "DOWN");
			}
			s1_wifi_up = up;
			last_link_ms = now.lo;
		}

		if (sz != (INT)sizeof(t)) {
			continue;	/* timeout: nothing to print */
		}
		s1_rx_count++;
		if (S1_PRINT_CSV == 0) {
			continue;
		}
		if (header_done == FALSE) {
			tm_printf((UB *)"T,t_ms,seq,state,barcode,letter,faults,vL,vR,encL,encR,"
				  "dist,lineErr,lineFlags,hump,humpPeak,obstacle,wifi\n");
			header_done = TRUE;
		}
		tm_printf((UB *)"T,%u,%u,%d,%d,%c,0x%02x,%d,%d,%u,%u,%d,%d,0x%02x,%d,%d,%d,%d\n",
			  (UINT)t.t_ms, (UINT)t.seq, (INT)t.state, (INT)t.last_barcode,
			  (t.last_letter != 0) ? (INT)t.last_letter : (INT)'-',
			  (UINT)t.fault_mask, (INT)t.speed_l_mm_s, (INT)t.speed_r_mm_s,
			  (UINT)t.enc_l, (UINT)t.enc_r, (INT)t.distance_mm, (INT)t.line_err,
			  (UINT)t.line_flags, (INT)t.hump_active, (INT)t.hump_peak_mm,
			  (INT)t.obstacle_mm, (INT)t.wifi_up);
	}
}
