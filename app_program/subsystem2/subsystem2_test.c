/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2.  Built only with: make TEST=1
 *	The motors are NOT driven by these tests.
 */

#include "subsystem2.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem2_test(void)
{
	T_RCYC	rcyc;

	TEST_INIT();
	tk_dly_tsk(300);

	TEST_CHECK("S2-U01", s2_hw_ready == TRUE, "motor + encoder init done");
	TEST_CHECK("S2-U02", (tk_ref_cyc(cyc_motion, &rcyc) == E_OK) &&
		   ((rcyc.cycstat & TCYC_STA) != 0U), "20 ms cyclic handler running");
	TEST_CHECK("S2-U03", (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) != 0U,
		   "PWM block out of reset");
	TEST_CHECK("S2-U04", s2_faulted == FALSE, "no stall while idle");

	TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
