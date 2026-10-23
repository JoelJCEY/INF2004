/*
 *	subsystem4_test.c
 *	On-target tests for subsystem 4.  Built only with: make TEST=1
 *	Run with the car still and level.
 */

#include "subsystem4.h"
#include "imu_math.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem4_test(void)
{
	UW	before;

	TEST_INIT();

	TEST_CHECK("S4-U01", imu_isqrt(1000000U) == 1000U, "isqrt(1e6) = 1000");
	TEST_CHECK("S4-U02", (imu_atan2_deg10(1000, 1000) >= 447) &&
		   (imu_atan2_deg10(1000, 1000) <= 453), "atan2 45 deg within 0.3");

	tk_dly_tsk(1500);	/* baseline needs 1 s */
	TEST_CHECK("S4-U03", s4_ok == TRUE, "IMU found and configured");
	before = s4_samples;
	tk_dly_tsk(1000);
	TEST_CHECK("S4-U04", (s4_samples - before) >= 80U, "~100 samples/s");
	TEST_CHECK("S4-U05", (s4_last_mag_mg > 900) && (s4_last_mag_mg < 1100),
		   "|a| at rest within 900-1100 mg");
	TEST_CHECK("S4-U06", (s4_sin_x1000 < 30) && (s4_sin_x1000 > -30),
		   "level after baseline (|sin| < 0.03)");

	TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
