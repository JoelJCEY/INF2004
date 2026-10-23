/*
 *	subsystem5_test.c
 *	On-target tests for subsystem 5.  Built only with: make TEST=1
 *	Leave the buttons alone while the tests run.
 */

#include "subsystem5.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem5_test(void)
{
	UW	before;
	T_RTSK	rtsk;

	TEST_INIT();

	tk_dly_tsk(500);
	TEST_CHECK("S5-U01", s5_scan_ready == TRUE, "servo + ultrasonic ready");
	before = s5_ping_count;
	tk_dly_tsk(1200);
	TEST_CHECK("S5-U02", ((s5_ping_count - before) >= 10U) && ((s5_ping_count - before) <= 21U),
		   "one ping every 60-90 ms (60 ms + echo wait)");
	TEST_CHECK("S5-U05", (s5_ping_count == 0U) || (s5_echo_isr_count > 0U) || (s5_last_mm < 0),
		   "ECHO interrupt fires when an echo returns");
	TEST_CHECK("S5-U03", (s5_vehicle_state == VS_READY) ||
		   (s5_vehicle_state == VS_FAULT_STOP),
		   "ctrl_task left INITIALISING");
	TEST_CHECK("S5-U04", (tk_ref_tsk(tid_ctrl, &rtsk) == E_OK) &&
		   ((rtsk.tskstat & TTS_WAI) != 0U), "ctrl_task sleeps between events");
	tm_printf((UB *)"[TEST] state = %s, faults = 0x%02x\n",
		  s5_state_name[s5_vehicle_state], (INT)s5_fault_mask);

	TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
