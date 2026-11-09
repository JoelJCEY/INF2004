/*
 *	subsystem5_test.c
 *	On-target tests for subsystem 5 (Scan + Vehicle Controller).
 *	make TEST=1.  Record results in testing/subsystem5/TEST_LOG.md.
 *
 *	S5-U01..U07 are logic tests.  Range at 3.3 V across servo angles,
 *	braking distance and detect-to-stop timing (A5, NFR2) are manual.
 */

#include "subsystem5.h"
#include "drv_ultrasonic.h"
#include "scan_profile.h"
#include "ctrl_logic.h"
#include "../test_report.h"

#if APP_TEST

/* A flat-faced box straight ahead: every angle whose ray hits the face
   at y = face_mm within |x| <= half_w reads face_mm / sin(θ) */
LOCAL void make_box(SCAN_SWEEP *s, W face_mm, W half_w, W wall_right_mm)
{
	INT	i;
	W	sn, cs, d, x;

	for (i = 0; i < SCAN_POSITIONS; i++) {
		sn = scan_sin1000((UINT)i);
		cs = scan_cos1000((UINT)i);
		d = (face_mm * 1000) / sn;
		x = (d * cs) / 1000;
		if (APP_ABS(x) <= half_w) {
			s->dist[i] = (UH)d;
		} else if ((wall_right_mm > 0) && (cs > 0)) {
			s->dist[i] = (UH)((wall_right_mm * 1000) / cs);	/* wall at x */
		} else {
			s->dist[i] = (UH)US_FAR_MM;
		}
	}
}

EXPORT void subsystem5_test(void)
{
	SCAN_SWEEP	sw;
	SCAN_RESULT	r;
	CTRL_STEP	st[CTRL_MAX_STEPS];
	INT		n, i;
	W		turn_sum = 0;

	TEST_INIT();

	/* S5-U01: 1166 µs echo = 200 mm (343 m/s, there and back) */
	TEST_CHECK("S5-U01", APP_ABS((W)drv_us_dur_to_mm(1166U) - 200) <= 1,
		   "echo duration to distance");

	/* S5-U02: 200 mm wide box 300 mm ahead: width within 25% */
	make_box(&sw, 300, 100, 0);
	TEST_CHECK("S5-U02", (scan_profile(&sw, &r) == E_OK) && (r.kind == OB_PROFILE) &&
		   (APP_ABS((W)r.width_mm - 200) <= 50) && (r.closest_mm == 300U),
		   "profile width and closest point");
	tm_printf((UB*)"[TEST] S5 info: width %d, closest %d, side %d, offset %d\n",
		  r.width_mm, r.closest_mm, r.side, r.offset_mm);

	/* S5-U03a (4 Nov): nothing at the sides: the LEFT side is chosen;
	   S5-U03b: wall close on the LEFT: the right side is chosen */
	make_box(&sw, 300, 100, 0);
	TEST_CHECK("S5-U03a", (scan_profile(&sw, &r) == E_OK) && (r.side == OBS_SIDE_LEFT),
		   "clear both sides -> bypass on the left (team choice)");
	make_box(&sw, 300, 100, 180);
	{
		INT	k;
		UH	t;

		for (k = 0; k < SCAN_POSITIONS / 2; k++) {	/* mirror: wall now on the left */
			t = sw.dist[k];
			sw.dist[k] = sw.dist[SCAN_POSITIONS - 1 - k];
			sw.dist[SCAN_POSITIONS - 1 - k] = t;
		}
	}
	TEST_CHECK("S5-U03b", (scan_profile(&sw, &r) == E_OK) && (r.side == OBS_SIDE_RIGHT),
		   "blocked left side -> bypass on the right");

	/* S5-U03: with a wall close on the right, the left side is chosen */
	make_box(&sw, 300, 100, 180);
	TEST_CHECK("S5-U03", (scan_profile(&sw, &r) == E_OK) && (r.side == OBS_SIDE_LEFT),
		   "blocked right side -> bypass on the left");

	/* S5-U04: START is rejected until INITIALISING is complete; STOP
	   stops every active state; RESET from FAULT STOP re-initialises */
	TEST_CHECK("S5-U04",
		   (ctrl_on_command(VS_INITIALISING, RC_START) == VS_INITIALISING) &&
		   (ctrl_on_command(VS_READY, RC_START) == VS_LINE_FOLLOWING) &&
		   (ctrl_on_command(VS_OBSTACLE_BYPASS, RC_STOP) == VS_STOPPED) &&
		   (ctrl_on_command(VS_FAULT_STOP, RC_START) == VS_FAULT_STOP) &&
		   (ctrl_on_command(VS_FAULT_STOP, RC_RESET) == VS_INITIALISING),
		   "START/STOP/RESET acceptance (statechart 5)");

	/* S5-U05: fault table 5.1 responses */
	TEST_CHECK("S5-U05",
		   (ctrl_on_faults(VS_READY, FLT_STALL) == VS_FAULT_STOP) &&
		   (ctrl_on_faults(VS_LINE_FOLLOWING, FLT_IMPACT) == VS_STOPPED) &&
		   (ctrl_on_faults(VS_LINE_FOLLOWING, FLT_LINE_LOST) == VS_STOPPED) &&
		   (ctrl_on_faults(VS_OBSTACLE_BYPASS, FLT_LINE_LOST) == VS_OBSTACLE_BYPASS) &&
		   (ctrl_on_faults(VS_LINE_FOLLOWING, FLT_WIFI) == VS_LINE_FOLLOWING),
		   "fault responses (Wi-Fi loss never stops the car)");

	/* S5-U06: bypass legs end facing the original direction */
	n = ctrl_plan_bypass(OBS_SIDE_LEFT, 200, 350, st);
	for (i = 0; i < n; i++) {
		if (st[i].op == STEP_TURN) {
			turn_sum += st[i].arg;
		}
	}
	/* 31 Oct: the final turn is a fixed part (counted in turn_sum) and a
	   search turn that ends on the line: before the search turn the car
	   is still 90 - CTRL_LAND_FIXED_DEG short of its original heading */
	TEST_CHECK("S5-U06", (n == 10) && (turn_sum == -(90 - CTRL_LAND_FIXED_DEG)) &&
		   (st[0].op == STEP_MOVE) && (st[0].arg < 0) && (st[6].op == STEP_SEARCH) &&
		   (st[7].op == STEP_MOVE) && (st[7].arg == CTRL_SENSOR_AHEAD_MM) &&
		   (st[9].op == STEP_SEARCH_TURN) && (st[8].arg + st[9].arg == 1 * (CTRL_LAND_FIXED_DEG + CTRL_LAND_SEARCH_DEG)),
		   "bypass plan: reverse, legs, line search, drive over the line, turn onto it");

	/* S5-U06b (4 Nov): the forward leg is a PASS leg on BOTH sides and
	   may run past the estimate */
	n = ctrl_plan_bypass(OBS_SIDE_LEFT, 200, 350, st);
	TEST_CHECK("S5-U06b", (n == 10) && (st[4].op == STEP_PASS) &&
		   (st[4].arg == 350 + CTRL_BYPASS_BACKUP_MM + CTRL_PASS_MAX_EXTRA_MM) &&
		   (ctrl_plan_bypass(OBS_SIDE_RIGHT, 200, 350, st) == 10) && (st[4].op == STEP_PASS),
		   "bypass plan: watched pass leg on both sides");

	/* S5-U07: barcode plans; straight keeps following */
	TEST_CHECK("S5-U07", (ctrl_plan_barcode(BC_U_TURN, st) >= 1) &&
		   (ctrl_plan_barcode(BC_GO_STRAIGHT, st) == 0) &&
		   (ctrl_plan_barcode(BC_UNRECOGNISED, st) == 0),
		   "barcode command to manoeuvre");

	/* S5-U08: at a cross-junction the axle is brought over the middle,
	   then the car turns; forward needs no steps */
	n = ctrl_plan_junction(BC_TURN_LEFT, 20, st);
	TEST_CHECK("S5-U08a", (n == 3) && (st[0].op == STEP_MOVE) &&
		   (st[0].arg == (CTRL_SENSOR_AHEAD_MM - 20)) &&
		   (st[1].op == STEP_TURN) && (st[2].op == STEP_SEARCH_TURN) &&
		   (st[1].arg > 0) && (st[1].arg < 90) &&
		   (st[1].arg + st[2].arg > 90),
		   "junction LEFT: axle over the middle, turn left, search past 90");
	n = ctrl_plan_junction(BC_TURN_RIGHT, CTRL_SENSOR_AHEAD_MM + 30, st);
	TEST_CHECK("S5-U08b", (n == 3) && (st[0].arg == -30) && (st[1].arg < 0) &&
		   (st[1].arg > -90) && (st[1].arg + st[2].arg < -90),
		   "junction RIGHT, overshot: back up 30 mm, turn right, search past -90");
	TEST_CHECK("S5-U08c", (ctrl_plan_junction(BC_U_TURN, CTRL_SENSOR_AHEAD_MM, st) == 2) &&
		   (st[0].op == STEP_TURN) && (st[0].arg > 90) && (st[0].arg < 180) &&
		   (st[1].op == STEP_SEARCH_TURN) && (st[0].arg + st[1].arg > 180) &&
		   (ctrl_plan_junction(BC_GO_STRAIGHT, 0, st) == 0) &&
		   (ctrl_plan_junction(BC_UNRECOGNISED, 0, st) == 0),
		   "junction U-TURN in place; FORWARD / unknown letter: straight on");

	TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
