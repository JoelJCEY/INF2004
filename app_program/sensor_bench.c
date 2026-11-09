/*
 *	sensor_bench.c
 *	Guided sensor bench test (TEST=1 builds only).  Runs after the unit
 *	tests, once the start-up checks have finished.  It prints what to do
 *	("[BENCH] ..."), waits for you to do it, measures, and prints one
 *	"[TEST] Sx-Byy PASS/FAIL" line per check with the numbers.
 *	Everything also appears in the dashboard's car log.
 *
 *	Covers: IR line sensors (S3-B), wheel encoders (S2-B), accelerometer
 *	and magnetometer (S4-B), ultrasonic sensor (S5-B).  The motors never
 *	run.  A step you don't do times out and is reported as FAIL.
 *	Results go in testing/<subsystem>/TEST_LOG.md.
 */

#include <stdarg.h>
#include "app.h"
#include "test_report.h"
#include "subsystem2/subsystem2.h"
#include "subsystem3/subsystem3.h"
#include "subsystem3/drv_ir_adc.h"
#include "subsystem4/subsystem4.h"
#include "subsystem4/hump.h"
#include "subsystem5/subsystem5.h"
#include "subsystem5/drv_ultrasonic.h"

#if APP_TEST

IMPORT INT tm_vsnprintf(UB *str, INT max, const UB *format, va_list ap);

#define TICK_MS		20
#define STEP_TMO_MS	25000	/* time you get for each hand step */
#define IR_BLACK_RISE	800	/* raw counts above white = "sees black" */
#define IR_HOLD_TICKS	25	/* 0.5 s */
#define ENC_START	100	/* counts: the wheel is being turned */
#define ENC_TOL_PCT	25	/* one turn within +-25 % of COUNTS_PER_REV */
#define IMU_TILT_MG	200	/* clear tilt */
#define US_PINGS	20

LOCAL INT	n_pass;
LOCAL INT	n_fail;

/* ---------------------------------------------------------------------
 *  Output helpers
 * --------------------------------------------------------------------- */
LOCAL void result(const char *id, BOOL pass, const char *fmt, ...)
{
	UB	text[170];
	va_list	ap;

	va_start(ap, fmt);
	(void)tm_vsnprintf(text, (INT)sizeof(text), (const UB*)fmt, ap);
	va_end(ap);
	if (pass) {
		n_pass++;
	} else {
		n_fail++;
	}
	app_log((UB*)"[TEST] %s %s  %s\n", id, pass ? "PASS" : "FAIL", text);
}

LOCAL void countdown(INT s)
{
	app_log((UB*)"[BENCH]   starting in %d s ...\n", s);
	tk_dly_tsk((RELTIM)(s * 1000));
	app_log((UB*)"[BENCH]   measuring\n");
}

LOCAL void sort_w(W *v, INT n)
{
	INT	i, j;
	W	t;

	for (i = 1; i < n; i++) {
		t = v[i];
		for (j = i; (j > 0) && (v[j - 1] > t); j--) {
			v[j] = v[j - 1];
		}
		v[j] = t;
	}
}

/* Running mean / spread / std of one signal */
typedef struct {
	INT		n;
	W		min, max;
	long long	sum, sumsq;
} STAT;

LOCAL void st_reset(STAT *s)
{
	s->n = 0;
	s->min = 0x7FFFFFFF;
	s->max = -0x7FFFFFFF;
	s->sum = 0;
	s->sumsq = 0;
}

LOCAL void st_add(STAT *s, W v)
{
	s->n++;
	s->sum += v;
	s->sumsq += (long long)v * v;
	if (v < s->min) s->min = v;
	if (v > s->max) s->max = v;
}

LOCAL W st_mean(const STAT *s)
{
	return (s->n > 0) ? (W)(s->sum / s->n) : 0;
}

LOCAL W st_std(const STAT *s)
{
	long long	var;

	if (s->n < 2) {
		return 0;
	}
	/* (n*sum(x^2) - sum(x)^2) / n^2: no rounding of the mean */
	var = ((s->n * s->sumsq) - (s->sum * s->sum)) / ((long long)s->n * s->n);
	if (var < 0) {
		var = 0;
	}
	if (var > 0x7FFFFFFF) {
		var = 0x7FFFFFFF;
	}
	return (W)hump_isqrt((UW)var);
}

LOCAL const char *const SIDE[3] = {"LEFT", "CENTRE", "RIGHT"};

/* ---------------------------------------------------------------------
 *  Subsystem 3: IR line sensors
 * --------------------------------------------------------------------- */
LOCAL W ir_white[3];

LOCAL void bench_ir(void)
{
	STAT	st[3];
	UW	w0, w1;
	W	rate;
	INT	i, k, t;
	BOOL	ok;

	app_log((UB*)"[BENCH] ===== IR line sensors (subsystem 3) =====\n");

	/* S3-B01: the ADC really samples at 3 kHz */
	w0 = drv_ir_adc_samples_written();
	tk_dly_tsk(2000);
	w1 = drv_ir_adc_samples_written();
	rate = (W)((w1 - w0) / 2U);
	result("S3-B01", (rate >= 2850) && (rate <= 3150),
	       "ADC sample rate %d Hz (expected 3000 +-5%%)", rate);

	/* S3-B02: white level and noise */
	app_log((UB*)"[BENCH] S3-B02  Put the car on WHITE floor, no line under any sensor. Don't move it.\n");
	countdown(5);
	for (i = 0; i < 3; i++) {
		st_reset(&st[i]);
	}
	for (k = 0; k < 100; k++) {		/* 2 s */
		for (i = 0; i < 3; i++) {
			st_add(&st[i], (W)line_raw_avg[i]);
		}
		tk_dly_tsk(TICK_MS);
	}
	for (i = 0; i < 3; i++) {
		ir_white[i] = st_mean(&st[i]);
		ok = (ir_white[i] > 0) && (ir_white[i] < 1500) && ((st[i].max - st[i].min) < 250);
		result(i == 0 ? "S3-B02L" : (i == 1 ? "S3-B02C" : "S3-B02R"), ok,
		       "%s on white: mean %d, range %d-%d, noise %d (need mean < 1500, range < 250)",
		       SIDE[i], ir_white[i], st[i].min, st[i].max, st_std(&st[i]));
	}

	/* S3-B03..05: each sensor alone sees black, and it is the RIGHT one */
	for (t = 0; t < 3; t++) {
		const char *id = (t == 0) ? "S3-B03" : ((t == 1) ? "S3-B04" : "S3-B05");
		W	rise[3], best_rise[3] = {0, 0, 0};
		INT	hold = 0, who = -1, last = -1;
		INT	waited;

		app_log((UB*)"[BENCH] %s  Put the black line (or black tape) under ONLY the %s sensor and hold it there.\n",
			id, SIDE[t]);
		for (waited = 0; waited < STEP_TMO_MS; waited += TICK_MS) {
			INT	top = 0;

			for (i = 0; i < 3; i++) {
				rise[i] = (W)line_raw_avg[i] - ir_white[i];
				if (rise[i] > best_rise[i]) best_rise[i] = rise[i];
				if (rise[i] > rise[top]) top = i;
			}
			if (rise[top] >= IR_BLACK_RISE) {
				hold = (top == last) ? hold + 1 : 1;
				last = top;
				if (hold >= IR_HOLD_TICKS) {
					who = top;
					break;
				}
			} else {
				hold = 0;
				last = -1;
			}
			tk_dly_tsk(TICK_MS);
		}
		if (who < 0) {
			result(id, FALSE, "no sensor saw black in %d s (best rise L %d C %d R %d, need %d)",
			       STEP_TMO_MS / 1000, best_rise[0], best_rise[1], best_rise[2], IR_BLACK_RISE);
		} else if (who != t) {
			result(id, FALSE, "the %s sensor reacted, not the %s one: sensor wires swapped? (rise L %d C %d R %d)",
			       SIDE[who], SIDE[t], rise[0], rise[1], rise[2]);
		} else {
			result(id, TRUE, "%s sees black: raw %d, contrast %d (rise L %d C %d R %d)",
			       SIDE[t], (W)line_raw_avg[t], rise[t], rise[0], rise[1], rise[2]);
		}
		app_log((UB*)"[BENCH]   Now take the line away.\n");
		for (waited = 0; waited < 10000; waited += TICK_MS) {
			if ((((W)line_raw_avg[0] - ir_white[0]) < 400) &&
			    (((W)line_raw_avg[1] - ir_white[1]) < 400) &&
			    (((W)line_raw_avg[2] - ir_white[2]) < 400)) {
				break;
			}
			tk_dly_tsk(TICK_MS);
		}
		tk_dly_tsk(500);
	}
}

/* ---------------------------------------------------------------------
 *  Subsystem 2: wheel encoders (turned by hand, motors off)
 * --------------------------------------------------------------------- */
/* Wait for a hand turn: start when a wheel moves, end after 2 s still */
LOCAL BOOL hand_turn(W *dl, W *dr, W need)
{
	W	l0 = motion_enc_count[0], r0 = motion_enc_count[1];
	W	pl, pr;
	INT	waited, still = 0;
	BOOL	started = FALSE;

	pl = l0;
	pr = r0;
	for (waited = 0; waited < (STEP_TMO_MS + 15000); waited += TICK_MS) {
		W l = motion_enc_count[0], r = motion_enc_count[1];

		if (!started) {
			if ((APP_ABS(l - l0) > ENC_START) || (APP_ABS(r - r0) > ENC_START)) {
				started = TRUE;
				app_log((UB*)"[BENCH]   wheel moving ... stop when done\n");
			} else if (waited > STEP_TMO_MS) {
				break;
			}
		} else {
			if ((APP_ABS(l - pl) < 5) && (APP_ABS(r - pr) < 5)) {
				if (++still >= 100) {		/* 2 s still */
					if ((APP_ABS(l - l0) >= need) || (APP_ABS(r - r0) >= need)) {
						break;
					}
				}
			} else {
				still = 0;
			}
		}
		pl = l;
		pr = r;
		tk_dly_tsk(TICK_MS);
	}
	*dl = motion_enc_count[0] - l0;
	*dr = motion_enc_count[1] - r0;
	return started;
}

LOCAL void bench_one_wheel(const char *id, INT w)
{
	W	d[2];
	W	cpr = MOTION_COUNTS_PER_REV;
	W	mine, other;
	BOOL	moved;

	app_log((UB*)"[BENCH] %s  Mark the %s tyre. Turn the %s wheel FORWARD exactly ONE full turn by hand, then let go.\n",
		id, SIDE[w * 2], SIDE[w * 2]);
	app_log((UB*)"[BENCH]   (the motor is braked, so it feels stiff; forward = the way it rolls when the car drives ahead)\n");
	moved = hand_turn(&d[0], &d[1], cpr / 2);
	mine = d[w];
	other = d[1 - w];

	if (!moved) {
		result(id, FALSE, "no encoder counted in %d s (L %d, R %d)", STEP_TMO_MS / 1000, d[0], d[1]);
	} else if (APP_ABS(other) > APP_ABS(mine)) {
		result(id, FALSE, "the %s encoder counted instead (L %d, R %d): encoder cables swapped?",
		       SIDE[(1 - w) * 2], d[0], d[1]);
	} else if (mine < 0) {
		result(id, FALSE, "%s counted BACKWARD (%d): flip MOTION_ENC_%s_INVERT in subsystem2.h",
		       SIDE[w * 2], mine, (w == 0) ? "L" : "R");
	} else if ((APP_ABS(mine - cpr) * 100) > (cpr * ENC_TOL_PCT)) {
		result(id, FALSE, "%s one turn = %d counts, expected about %d (exactly one turn? if yes, update MOTION_COUNTS_PER_REV)",
		       SIDE[w * 2], mine, cpr);
	} else if ((APP_ABS(other) * 20) > mine) {
		result(id, FALSE, "%s ok (%d) but the other wheel also counted %d: loose wire or noise?",
		       SIDE[w * 2], mine, other);
	} else {
		result(id, TRUE, "%s one turn = %d counts (setting %d, %d%% off), other wheel %d",
		       SIDE[w * 2], mine, cpr, (W)((APP_ABS(mine - cpr) * 100) / cpr), other);
	}
}

LOCAL void bench_enc(void)
{
	W	dl, dr;
	BOOL	moved;

	app_log((UB*)"[BENCH] ===== Wheel encoders (subsystem 2) =====\n");
	app_log((UB*)"[BENCH] Lift the car or put it on its back so the wheels can turn.\n");
	tk_dly_tsk(4000);
	bench_one_wheel("S2-B01", 0);
	bench_one_wheel("S2-B02", 1);

	app_log((UB*)"[BENCH] S2-B03  Turn BOTH wheels BACKWARD about half a turn, then let go.\n");
	moved = hand_turn(&dl, &dr, MOTION_COUNTS_PER_REV / 4);
	result("S2-B03", moved && (dl < -(W)ENC_START) && (dr < -(W)ENC_START),
	       "backward: L %d, R %d (both must be negative)", dl, dr);
}

/* ---------------------------------------------------------------------
 *  Subsystem 4: accelerometer and magnetometer
 * --------------------------------------------------------------------- */
/* Wait until the given axis moves by more than IMU_TILT_MG for 0.5 s.
   Returns the axis that moved most (0 x, 1 y) or -1, and the change. */
LOCAL INT wait_tilt(W *dx, W *dy)
{
	INT	waited, hold = 0, axis = -1, last = -1;
	W	bx = imu_base_mg[0], by = imu_base_mg[1];

	for (waited = 0; waited < STEP_TMO_MS; waited += TICK_MS) {
		*dx = imu_acc_raw[0] - bx;
		*dy = imu_acc_raw[1] - by;
		axis = (APP_ABS(*dx) >= APP_ABS(*dy)) ? 0 : 1;
		if (APP_ABS(axis == 0 ? *dx : *dy) > IMU_TILT_MG) {
			hold = (axis == last) ? hold + 1 : 1;
			last = axis;
			if (hold >= IR_HOLD_TICKS) {
				return axis;
			}
		} else {
			hold = 0;
			last = -1;
		}
		tk_dly_tsk(TICK_MS);
	}
	return -1;
}

LOCAL void wait_flat(void)
{
	INT	waited;

	app_log((UB*)"[BENCH]   Put it back down flat.\n");
	for (waited = 0; waited < 10000; waited += TICK_MS) {
		if ((APP_ABS(imu_acc_raw[0] - imu_base_mg[0]) < 80) &&
		    (APP_ABS(imu_acc_raw[1] - imu_base_mg[1]) < 80)) {
			break;
		}
		tk_dly_tsk(TICK_MS);
	}
	tk_dly_tsk(1000);
}

LOCAL void bench_imu(void)
{
	STAT	st[4];
	UW	ok0, fail0, ok1, fail1;
	INT	k, axis, waited;
	W	dx, dy, mag, peak;
	W	mmin[2], mmax[2];

	app_log((UB*)"[BENCH] ===== Accelerometer / magnetometer (subsystem 4) =====\n");

	/* S4-B01: still, flat: 1 g, low noise, no I2C errors */
	app_log((UB*)"[BENCH] S4-B01  Put the car FLAT on the floor and don't touch it.\n");
	countdown(4);
	for (k = 0; k < 4; k++) {
		st_reset(&st[k]);
	}
	ok0 = imu_reads_ok;
	fail0 = imu_reads_fail;
	for (k = 0; k < 150; k++) {		/* 3 s */
		W x = imu_acc_raw[0], y = imu_acc_raw[1], z = imu_acc_raw[2];

		st_add(&st[0], x);
		st_add(&st[1], y);
		st_add(&st[2], z);
		st_add(&st[3], (W)hump_isqrt((UW)((x * x) + (y * y) + (z * z))));
		tk_dly_tsk(TICK_MS);
	}
	ok1 = imu_reads_ok;
	fail1 = imu_reads_fail;
	result("S4-B01a", (ok1 - ok0) >= 120U && (fail1 == fail0),
	       "I2C reads in 3 s: %d ok, %d failed (need >= 120 ok, 0 failed)", ok1 - ok0, fail1 - fail0);
	result("S4-B01b", (st_mean(&st[3]) >= 900) && (st_mean(&st[3]) <= 1100),
	       "gravity %d mg (x %d, y %d, z %d), expected 1000 +-100", st_mean(&st[3]),
	       st_mean(&st[0]), st_mean(&st[1]), st_mean(&st[2]));
	result("S4-B01c", (st_std(&st[0]) < 30) && (st_std(&st[1]) < 30) && (st_std(&st[2]) < 30),
	       "noise x %d, y %d, z %d mg (need < 30)", st_std(&st[0]), st_std(&st[1]), st_std(&st[2]));
	result("S4-B01d", (APP_ABS(imu_base_mg[0] - st_mean(&st[0])) < 60) && (APP_ABS(imu_base_mg[1] - st_mean(&st[1])) < 60),
	       "start-up level reference x %d, y %d vs now x %d, y %d (car must be flat at power-on)",
	       imu_base_mg[0], imu_base_mg[1], st_mean(&st[0]), st_mean(&st[1]));

	/* S4-B02: nose up must read as "front higher" (hump direction) */
	app_log((UB*)"[BENCH] S4-B02  Lift the FRONT of the car about 3 cm (nose up) and hold it.\n");
	axis = wait_tilt(&dx, &dy);
	if (axis < 0) {
		result("S4-B02", FALSE, "no tilt seen in %d s (x %d, y %d mg)", STEP_TMO_MS / 1000, dx, dy);
	} else if (axis != 0) {
		result("S4-B02", FALSE, "nose-up moved Y (%d mg) not X (%d mg): the IMU is mounted sideways", dy, dx);
	} else if ((IMU_FWD_SIGN * dx) < 0) {
		result("S4-B02", FALSE, "nose-up reads as nose-DOWN (x %d mg): set IMU_FWD_SIGN to %d in subsystem4.h",
		       dx, -IMU_FWD_SIGN);
	} else {
		result("S4-B02", TRUE, "nose-up reads front higher: x %+d mg (about %d deg)", dx, (dx * 57) / 1000);
	}
	wait_flat();

	/* S4-B03: side tilt moves Y */
	app_log((UB*)"[BENCH] S4-B03  Lift the LEFT side of the car about 3 cm and hold it.\n");
	axis = wait_tilt(&dx, &dy);
	result("S4-B03", axis == 1, axis < 0 ? "no tilt seen (x %d, y %d mg)" :
	       "side tilt: y %+d mg, x %+d mg (Y must move most)", axis < 0 ? dx : dy, axis < 0 ? dy : dx);
	wait_flat();

	/* S4-B04: a knock is seen (impact detection path) */
	app_log((UB*)"[BENCH] S4-B04  Knock firmly on the car or give it a short shake (8 s).\n");
	peak = 0;
	for (waited = 0; waited < 8000; waited += TICK_MS / 2) {
		W x = imu_acc_raw[0], y = imu_acc_raw[1], z = imu_acc_raw[2];

		mag = (W)hump_isqrt((UW)((x * x) + (y * y) + (z * z)));
		if (APP_ABS(mag - 1000) > peak) {
			peak = APP_ABS(mag - 1000);
		}
		tk_dly_tsk(TICK_MS / 2);
	}
	result("S4-B04", peak > 400, "biggest jolt %d mg from 1 g (need > 400; the car stops for > 2500)", peak);

	/* S4-B05: magnetometer follows the heading */
	app_log((UB*)"[BENCH] S4-B05  Turn the car slowly through one full circle on the floor (15 s).\n");
	countdown(3);
	mmin[0] = mmin[1] = 0x7FFFFFFF;
	mmax[0] = mmax[1] = -0x7FFFFFFF;
	for (waited = 0; waited < 15000; waited += TICK_MS * 5) {
		for (k = 0; k < 2; k++) {
			W v = imu_mag_raw[k];
			if (v < mmin[k]) mmin[k] = v;
			if (v > mmax[k]) mmax[k] = v;
		}
		tk_dly_tsk(TICK_MS * 5);
	}
	result("S4-B05", ((mmax[0] - mmin[0]) > 100) && ((mmax[1] - mmin[1]) > 100),
	       "magnetometer x %d..%d, y %d..%d (each range must be > 100)", mmin[0], mmax[0], mmin[1], mmax[1]);
}

/* ---------------------------------------------------------------------
 *  Subsystem 5: ultrasonic distance
 * --------------------------------------------------------------------- */
LOCAL void bench_us_at(const char *id, W target_mm)
{
	W	v[US_PINGS];
	INT	n_ok = 0, n_far = 0, n_none = 0, waited;
	UW	seq;
	W	med, lo, hi;

	app_log((UB*)"[BENCH] %s  Hold a flat object (book, box) %d cm straight in front of the ultrasonic sensor. Measure with a ruler.\n",
		id, target_mm / 10);
	countdown(8);
	seq = scan_bench_seq;
	for (waited = 0; (waited < 6000) && ((n_ok + n_far + n_none) < US_PINGS); waited += 10) {
		if (scan_bench_seq != seq) {
			UW w = scan_bench_word;

			seq = scan_bench_seq;
			switch ((US_RESULT)(w >> 16)) {
			case US_OK:	v[n_ok++] = (W)(w & 0xFFFFU);	break;
			case US_FAR:	n_far++;			break;
			default:	n_none++;			break;
			}
		}
		tk_dly_tsk(10);
	}
	if ((n_ok + n_far + n_none) == 0) {
		result(id, FALSE, "no pings at all: scan_task not running?");
		return;
	}
	if (n_ok < (US_PINGS * 3) / 4) {
		result(id, FALSE, "only %d of %d pings returned a distance (%d too far, %d no echo)%s",
		       n_ok, n_ok + n_far + n_none, n_far, n_none,
		       (n_none > n_ok) ? ": check TRIG GP0 / ECHO GP1 wiring and 3.3 V" : "");
		return;
	}
	sort_w(v, n_ok);
	med = v[n_ok / 2];
	lo = v[1];		/* ignore one outlier each end */
	hi = v[n_ok - 2];
	result(id, (APP_ABS(med - target_mm) <= 30) && ((hi - lo) <= 30),
	       "distance %d mm (target %d +-30), spread %d mm (need <= 30), %d/%d ok",
	       med, target_mm, hi - lo, n_ok, n_ok + n_far + n_none);
}

LOCAL void bench_us(void)
{
	app_log((UB*)"[BENCH] ===== Ultrasonic sensor (subsystem 5) =====\n");
	scan_bench_on = 1;
	bench_us_at("S5-B01", 200);
	bench_us_at("S5-B02", 400);
	scan_bench_on = 0;
}

/* ---------------------------------------------------------------------
 *  Entry
 * --------------------------------------------------------------------- */
EXPORT void sensor_bench(void)
{
	MISSION_STATE	ms;
	INT		waited;

	n_pass = 0;
	n_fail = 0;
	app_log((UB*)"[BENCH] ================= SENSOR BENCH =================\n");
	app_log((UB*)"[BENCH] Follow the [BENCH] lines. Don't press START. Each step waits %d s for you.\n",
		STEP_TMO_MS / 1000);
	for (waited = 0; waited < 45000; waited += 200) {
		ctrl_get_mission(&ms);
		if (ms.state != VS_INITIALISING) {
			break;
		}
		if (waited == 0) {
			app_log((UB*)"[BENCH] waiting for the start-up checks to finish ...\n");
		}
		tk_dly_tsk(200);
	}
	app_log_quiet = 1;		/* hide ir:/enc: chatter */

	bench_ir();
	bench_enc();
	bench_imu();
	bench_us();

	app_log_quiet = 0;
	app_log((UB*)"[TEST] sensors SUMMARY %d passed, %d failed\n", n_pass, n_fail);
	app_log((UB*)"[BENCH] done. Reset the car to run normally.\n");
}

#endif /* APP_TEST */
