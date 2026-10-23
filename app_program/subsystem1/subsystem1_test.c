/*
 *	subsystem1_test.c
 *	On-target tests for subsystem 1.  Built only with: make TEST=1
 */

#include "subsystem1.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem1_test(void)
{
	UW	before;

	TEST_INIT();

	/* S1-U01: ctrl_task -> comms_task telemetry at about 5 Hz */
	before = s1_rx_count;
	tk_dly_tsk(2000);
	TEST_CHECK("S1-U01", ((s1_rx_count - before) >= 8U) && ((s1_rx_count - before) <= 12U),
		   "about 10 telemetry snapshots in 2 s");

	TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
