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
#include "lift.h"
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

/* ---------------------------------------------------------------------
 *  S4-U05..U07: lifted / overturned detector (13 Nov), 50 Hz synthetic
 *  accelerometer.  Gravity "down" is +z (1000 mg) at rest.
 * --------------------------------------------------------------------- */
/* sin(deg) x 1000 for the test only (Bhaskara, any angle) */
LOCAL W hump_sin1000_t(W d)
{
	W	sg = 1;

	while (d > 180) d -= 360;
	while (d <= -180) d += 360;
	if (d < 0) { d = -d; sg = -1; }
	return sg * ((4 * d * (180 - d) * 1000) / (40500 - (d * (180 - d))));
}

LOCAL UW lift_rng = 12345U;
LOCAL W noise(W amp)
{
	lift_rng = (lift_rng * 1103515245U) + 12345U;
	return (W)((lift_rng >> 16) % (UW)(2 * amp + 1)) - amp;
}

/* Feed n readings: tilt deg about the forward axis (0 = level, 180 =
   upside down), extra push up (mg), sideways/forward push (mg), noise.
   Returns the first result that is not LIFT_NONE (and when). */
LOCAL LIFT_RESULT lift_feed(LIFT_DET *d, W *f, INT n, W tilt_deg, W up_mg, W fwd_mg,
			    W noise_mg, BOOL on_hump, INT *at)
{
	INT		i;
	LIFT_RESULT	r, first = LIFT_NONE;
	W		raw[3];
	W		s1000 = hump_sin1000_t(tilt_deg);
	W		c1000 = hump_sin1000_t(tilt_deg + 90);

	for (i = 0; i < n; i++) {
		W	g = 1000 + up_mg;

		raw[0] = fwd_mg + noise(noise_mg);
		raw[1] = ((g * s1000) / 1000) + noise(noise_mg);
		raw[2] = ((g * c1000) / 1000) + noise(noise_mg);
		f[0] += (raw[0] - f[0]) >> 2;
		f[1] += (raw[1] - f[1]) >> 2;
		f[2] += (raw[2] - f[2]) >> 2;
		r = lift_update(d, raw, f, on_hump);
		if ((r != LIFT_NONE) && (first == LIFT_NONE)) {
			first = r;
			*at = i;
		}
	}
	return first;
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

	{
		LIFT_DET	ld;
		W		f[3];
		INT		at = -1;
		LIFT_RESULT	r1, r2, r3, r4;

		/* S4-U05: things that must NOT stop the car */
		lift_init(&ld, 0, 0, 1000);  f[0] = 0; f[1] = 0; f[2] = 1000;
		r1 = lift_feed(&ld, f, 250, 0, 0, 0, 40, FALSE, &at);		/* 5 s still   */
		r2 = lift_feed(&ld, f, 25, 0, 0, 300, 80, FALSE, &at);		/* speeding up */
		r3 = lift_feed(&ld, f, 50, 15, 0, 0, 60, TRUE, &at);		/* 15 deg hump */
		r3 = (LIFT_RESULT)(r3 | lift_feed(&ld, f, 1, 15, 2500, 0, 0, TRUE, &at));	/* landing */
		r3 = (LIFT_RESULT)(r3 | lift_feed(&ld, f, 25, 0, 0, 0, 60, FALSE, &at));
		r4 = lift_feed(&ld, f, 1, 0, 2500, 0, 0, FALSE, &at);		/* one 2.5 g bump  */
		r4 = (LIFT_RESULT)(r4 | lift_feed(&ld, f, 1, 0, 1200, 0, 0, FALSE, &at));	/* + echo */
		r4 = (LIFT_RESULT)(r4 | lift_feed(&ld, f, 25, 0, 0, 0, 40, FALSE, &at));
		TEST_CHECK("S4-U05", (r1 == LIFT_NONE) && (r2 == LIFT_NONE) &&
			   (r3 == LIFT_NONE) && (r4 == LIFT_NONE),
			   "no stop: still, speeding up, 15 deg hump + landing, 2.5 g bump");

		/* S4-U05b (13 Nov): a 60 ms 0.6 g jolt (button pressed, knock,
		   tile edge) is not a lift; a slower 0.3 g lift still is */
		r1 = lift_feed(&ld, f, 3, 0, 600, 0, 40, FALSE, &at);
		r1 = (LIFT_RESULT)(r1 | lift_feed(&ld, f, 25, 0, 0, 0, 40, FALSE, &at));
		r2 = lift_feed(&ld, f, 10, 0, 300, 0, 40, FALSE, &at);
		lift_init(&ld, 0, 0, 1000);  f[0] = 0; f[1] = 0; f[2] = 1000;
		TEST_CHECK("S4-U05b", (r1 == LIFT_NONE) && (r2 == LIFT_LIFTED),
			   "60 ms jolt ignored; gentle 0.3 g lift for 0.2 s caught");

		/* S4-U06: lifted by hand: 0.4 g upward push for 0.2 s */
		r1 = lift_feed(&ld, f, 10, 0, 400, 0, 60, FALSE, &at);
		TEST_CHECK("S4-U06", (r1 == LIFT_LIFTED) && (at <= 6),
			   "hand lift (0.4 g up) -> LIFTED within 140 ms");
		tm_printf((UB*)"[TEST] S4 info: lift seen after %d readings (%d ms)\n", at + 1, (at + 1) * 20);

		/* S4-U07: tipped on its side, upside down, dropped */
		lift_init(&ld, 0, 0, 1000);  f[0] = 0; f[1] = 0; f[2] = 1000;
		r1 = lift_feed(&ld, f, 25, 90, 0, 0, 40, FALSE, &at);
		lift_init(&ld, 0, 0, 1000);  f[0] = 0; f[1] = 0; f[2] = 1000;
		r2 = lift_feed(&ld, f, 25, 180, 0, 0, 40, FALSE, &at);
		lift_init(&ld, 0, 0, 1000);  f[0] = 0; f[1] = 0; f[2] = 1000;
		r3 = lift_feed(&ld, f, 15, 0, -1000, 0, 20, FALSE, &at);	/* free fall */
		TEST_CHECK("S4-U07", (r1 == LIFT_OVERTURNED) && (r2 == LIFT_OVERTURNED) &&
			   (r3 == LIFT_OVERTURNED),
			   "on its side, upside down, or dropped -> OVERTURNED (stop)");
	}

	TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
