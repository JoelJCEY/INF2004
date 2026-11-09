/*
 *	subsystem4_test.c
 *	On-target tests for subsystem 4 (Terrain).  make TEST=1
 *	Record results in testing/subsystem4/TEST_LOG.md.
 *
 *	S4-U01..U04 drive the hump detector with synthetic tilt.  Runs over
 *	a hump of known height, flat-track noise and impacts (A4, NFR5) are
 *	manual tests, see TEST_LOG.md.
 */

#include "subsystem4.h"
#include "hump.h"
#include "../subsystem5/subsystem5.h"
#include "../test_report.h"

#if APP_TEST

LOCAL HUMP_DET	t_hump;

/* Front-wheel height (µm) of a hump: triangle, height hmax, length len */
LOCAL W hump_shape(W s_mm, W hmax_um, W len_mm)
{
	W	half = len_mm / 2;

	if ((s_mm <= 0) || (s_mm >= len_mm)) {
		return 0;
	}
	return (s_mm < half) ? (hmax_um * s_mm) / half
			     : (hmax_um * (len_mm - s_mm)) / half;
}

/* Drive over a hump, 6 mm per tick (0.3 m/s): returns the reported
   peak in µm, or -1 if none, and counts the events */
LOCAL W drive_over(W hmax_um, W len_mm, W total_mm, INT *starts, INT *discards)
{
	W	s;
	W	dh;
	W	peak = -1;
	W	p;
	HUMP_EVENT e;

	hump_reset(&t_hump);
	*starts = 0;
	*discards = 0;
	for (s = 0; s < total_mm; s += 6) {
		dh = hump_shape(s - 100, hmax_um, len_mm) -
		     hump_shape(s - 100 - HUMP_WHEELBASE_MM, hmax_um, len_mm);
		e = hump_update(&t_hump, dh, (UW)(s * 10), &p);
		if (e == HE_START) (*starts)++;
		if (e == HE_DISCARD) (*discards)++;
		if (e == HE_END) peak = p;
	}
	return peak;
}

EXPORT void subsystem4_test(void)
{
	W	peak;
	INT	starts, discards;
	T_RFLG	rflg;

	TEST_INIT();

	/* S4-U01: integer square root */
	TEST_CHECK("S4-U01", (hump_isqrt(1000000U) == 1000U) && (hump_isqrt(99U) == 9U),
		   "integer sqrt");

	/* S4-U02: 20 mm hump, 200 mm long: peak within ±5 mm or 15% (NFR5) */
	peak = drive_over(20000, 200, 800, &starts, &discards);
	TEST_CHECK("S4-U02", (starts == 1) && (APP_ABS(peak - 20000) <= 3000),
		   "20 mm hump reconstructed within 15%");
	tm_printf((UB*)"[TEST] S4 info: reconstructed peak %d um\n", peak);

	/* S4-U03: a 600 mm climb is a ramp, discarded (> 500 mm) */
	peak = drive_over(60000, 1400, 1600, &starts, &discards);
	TEST_CHECK("S4-U03", (peak < 0) && (discards >= 1), "long climb discarded");

	/* S4-U04: flat track gives no hump at all */
	peak = drive_over(0, 200, 5000, &starts, &discards);
	TEST_CHECK("S4-U04", (starts == 0) && (peak < 0), "no false hump on flat track");

	/* S4-U05: IMU answered and the level baseline finished */
	tk_dly_tsk(500);	/* baseline takes 1 s after imu_task starts */
	(void)tk_ref_flg(flg_sys, &rflg);
	TEST_CHECK("S4-U05", (rflg.flgptn & EVF_IMU_READY) != 0U,
		   "LSM303DLHC baseline done (EVF_IMU_READY)");

	TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
