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
#include "../subsystem4/hump.h"	/* hump_isqrt */
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
		if (app_abs(x) <= half_w) {
			s->dist[i] = (UH)d;
		} else if ((wall_right_mm > 0) && (cs > 0)) {
			/* wall at x */
			s->dist[i] = (UH)((wall_right_mm * 1000) / cs);
		} else {
			s->dist[i] = (UH)US_FAR_MM;
		}
	}
}


/* 13 Nov: drive towards a 5 cm round object at (side_mm right, ahead_mm)
   at 60 mm/s with the 8-angle glance (one ping per 180 ms, ~15 deg
   beam) and the glance memory.  Returns how far ahead the object really
   was when the car decided to stop, or -999 if it never stopped. */
LOCAL W glance_drive(W side_mm, W start_ahead_mm, BOOL stray)
{
	GLANCE_MEM	g;
	UW		k;
	W		path = 0;
	INT		gi = 0;

	gmem_clear(&g);
	for (k = 1U; k < 200U; k++) {
		UINT	idx = (UINT)((INT)SERVO_CENTRE_IDX + glance_seq[gi]);
		W	c = scan_cos1000(idx), s = scan_sin1000(idx);
		W	y = start_ahead_mm - path;
		W	best = 0;
		W	dx;
		W	pred;

		gi = (gi + 1) % GLANCE_SEQ_N;
		/* object edges and middle */
		for (dx = -25; dx <= 25; dx += 25) {
			W x = side_mm + dx;
			W r = (W)hump_isqrt((UW)((x * x) + (y * y)));
			/* within 7.5 deg */
			if ((y > 0) && (r > 0) &&
				((((x * c) + (y * s)) / r) >= 991)) {
				if ((best == 0) || (r < best)) {
					best = r;
				}
			}
		}
		if (stray && (k == 10U)) {
			best = 250;			/* one stray echo */
		}
		if ((best >= 40) && (((best * s) / 1000) < GLANCE_LOOK_MM) &&
		    (app_abs((best * c) / 1000) <= (SCAN_CAR_HALF_W_MM
		    	+ SCAN_MARGIN_MM))) {
			gmem_add(&g, path, (best * s) / 1000,
				(best * c) / 1000, k);
		}
		path += 11;				/* 60 mm/s x 180 ms */
		pred = gmem_ahead(&g, path, k);
		if ((pred != GMEM_NONE) && (pred < SCAN_STOP_DIST_MM)) {
			return start_ahead_mm - path;
		}
		if ((start_ahead_mm - path) < -300) {
			break;
		}
	}
	return -999;
}

/* S5-U01..U03: echo timing, profile width and bypass side */
LOCAL void s5_test_scan(void)
{
	SCAN_SWEEP	sw;
	SCAN_RESULT	r;

	/* S5-U01: 1166 µs echo = 200 mm (343 m/s, there and back) */
	TEST_CHECK("S5-U01", app_abs((W)drv_us_dur_to_mm(1166U) - 200) <= 1,
		   "echo duration to distance");

	/* S5-U02: 200 mm wide box 300 mm ahead: width within 25% */
	make_box(&sw, 300, 100, 0);
	TEST_CHECK("S5-U02", (scan_profile(&sw, &r) == E_OK) &&
		(r.kind == OB_PROFILE) &&
		   (app_abs((W)r.width_mm - 200) <= 50) &&
		   	(r.closest_mm == 300U),
		   "profile width and closest point");
	tm_printf((UB*)"[TEST] S5 info: width %d, closest %d, side %d, "
		       "offset %d\n",
		  r.width_mm, r.closest_mm, r.side, r.offset_mm);

	/* S5-U03a (4 Nov): nothing at the sides: the LEFT side is chosen;
	   S5-U03b: wall close on the LEFT: the right side is chosen */
	make_box(&sw, 300, 100, 0);
	TEST_CHECK("S5-U03a", (scan_profile(&sw, &r) == E_OK) &&
		(r.side == OBS_SIDE_LEFT),
		   "clear both sides -> bypass on the left (team choice)");
	make_box(&sw, 300, 100, 180);
	{
		INT	k;
		UH	t;

		/* mirror: wall now on the left */
		for (k = 0; k < SCAN_POSITIONS / 2; k++) {
			t = sw.dist[k];
			sw.dist[k] = sw.dist[SCAN_POSITIONS - 1 - k];
			sw.dist[SCAN_POSITIONS - 1 - k] = t;
		}
	}
	TEST_CHECK("S5-U03b", (scan_profile(&sw, &r) == E_OK) &&
		(r.side == OBS_SIDE_RIGHT),
		   "blocked left side -> bypass on the right");

	/* S5-U03: with a wall close on the right, the left side is chosen */
	make_box(&sw, 300, 100, 180);
	TEST_CHECK("S5-U03", (scan_profile(&sw, &r) == E_OK) &&
		(r.side == OBS_SIDE_LEFT),
		   "blocked right side -> bypass on the left");
}

/* S5-U04, U05: commands and fault responses (statechart 5, table 5.1) */
LOCAL void s5_test_statechart(void)
{
	/* S5-U04: START is rejected until INITIALISING is complete; STOP
	   stops every active state; RESET from FAULT STOP re-initialises */
	TEST_CHECK("S5-U04",
		   (ctrl_on_command(VS_INITIALISING,
		   	RC_START) == VS_INITIALISING) &&
		   (ctrl_on_command(VS_READY, RC_START) == VS_LINE_FOLLOWING) &&
		   (ctrl_on_command(VS_OBSTACLE_BYPASS,
		   	RC_STOP) == VS_STOPPED) &&
		   (ctrl_on_command(VS_FAULT_STOP,
		   	RC_START) == VS_FAULT_STOP) &&
		   (ctrl_on_command(VS_FAULT_STOP,
		   	RC_RESET) == VS_INITIALISING),
		   "START/STOP/RESET acceptance (statechart 5)");

	/* S5-U05: fault table 5.1 responses */
	TEST_CHECK("S5-U05",
		   (ctrl_on_faults(VS_READY, FLT_STALL) == VS_FAULT_STOP) &&
		   (ctrl_on_faults(VS_LINE_FOLLOWING,
		   	FLT_LIFTED) == VS_STOPPED) &&
		   (ctrl_on_faults(VS_OBSTACLE_BYPASS,
		   	FLT_LIFTED) == VS_STOPPED) &&
		   (ctrl_on_faults(VS_INITIALISING,
		   	FLT_LIFTED) == VS_FAULT_STOP) &&
		   (ctrl_on_faults(VS_LINE_FOLLOWING,
		   	FLT_LINE_LOST) == VS_STOPPED) &&
		   (ctrl_on_faults(VS_OBSTACLE_BYPASS,
		   	FLT_LINE_LOST) == VS_OBSTACLE_BYPASS) &&
		   (ctrl_on_faults(VS_LINE_FOLLOWING,
		   	FLT_WIFI) == VS_LINE_FOLLOWING),
		   "fault responses (Wi-Fi loss never stops the car)");
}

/* S5-U06..U08: bypass, barcode and junction plans */
LOCAL void s5_test_plans(void)
{
	CTRL_STEP	st[CTRL_MAX_STEPS];
	INT		n, i;
	W		turn_sum = 0;

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
	TEST_CHECK("S5-U06", (n == 10) &&
		(turn_sum == -(90 - CTRL_LAND_FIXED_DEG)) &&
		   (st[0].op == STEP_MOVE) && (st[0].arg < 0) &&
		   	(st[6].op == STEP_SEARCH) &&
		   (st[7].op == STEP_MOVE) &&
		   	(st[7].arg == CTRL_SENSOR_AHEAD_MM) &&
		   (st[9].op == STEP_SEARCH_TURN) &&
		   	(st[8].arg + st[9].arg == 1 * (CTRL_LAND_FIXED_DEG
		   		+ CTRL_LAND_SEARCH_DEG)),
		   "bypass plan: reverse, legs, line search, drive over the "
		   "line, turn onto it");

	/* S5-U06b (4 Nov): the forward leg is a PASS leg on BOTH sides and
	   may run past the estimate */
	n = ctrl_plan_bypass(OBS_SIDE_LEFT, 200, 350, st);
	TEST_CHECK("S5-U06b", (n == 10) && (st[4].op == STEP_PASS) &&
		   (st[4].arg == 350 + CTRL_BYPASS_BACKUP_MM
		   	+ CTRL_PASS_MAX_EXTRA_MM) &&
		   (ctrl_plan_bypass(OBS_SIDE_RIGHT, 200, 350, st) == 10) &&
		   	(st[4].op == STEP_PASS),
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
		   "junction LEFT: axle over the middle, turn left, search "
		   "past 90");
	n = ctrl_plan_junction(BC_TURN_RIGHT, CTRL_SENSOR_AHEAD_MM + 30, st);
	TEST_CHECK("S5-U08b", (n == 3) && (st[0].arg == -30) &&
		(st[1].arg < 0) &&
		   (st[1].arg > -90) && (st[1].arg + st[2].arg < -90),
		   "junction RIGHT, overshot: back up 30 mm, turn right, "
		   "search past -90");
	TEST_CHECK("S5-U08c",
		(ctrl_plan_junction(BC_U_TURN, CTRL_SENSOR_AHEAD_MM,
			st) == 2) &&
		   (st[0].op == STEP_TURN) && (st[0].arg > 90) &&
		   	(st[0].arg < 180) &&
		   (st[1].op == STEP_SEARCH_TURN) &&
		   	(st[0].arg + st[1].arg > 180) &&
		   (ctrl_plan_junction(BC_GO_STRAIGHT, 0, st) == 0) &&
		   (ctrl_plan_junction(BC_UNRECOGNISED, 0, st) == 0),
		   "junction U-TURN in place; FORWARD / unknown letter: "
		   "straight on");
}

/* S5-U09 (13 Nov): whole-width glance watch */
LOCAL void s5_test_glance(void)
{
	/* S5-U09 (13 Nov): the glance watch covers the whole car width.
	   Objects straight ahead, half in front of the car, in front of a
	   CORNER (10 cm off centre, car is +-8 cm) and at the edge of the
	   margin stop about 20 cm away; one 20 cm to the side does not,
	   and one stray echo does not stop the car. */
	{
		W	a0 = glance_drive(0, 700, FALSE);
		W	a1 = glance_drive(60, 700, FALSE);
		W	a2 = glance_drive(100, 700, FALSE);
		W	a3 = glance_drive(-100, 700, FALSE);
		W	a4 = glance_drive(130, 700, FALSE);
		W	a5 = glance_drive(220, 700, FALSE);
		W	a6 = glance_drive(-220, 700, TRUE);

		tm_printf((UB*)"[TEST] S5 info: glance stops at %d / %d / %d "
			       "/ %d / %d mm (0, 6, 10 R, 10 L, 13 cm off); "
			       "22 cm off: %d; stray: %d\n",
			  a0, a1, a2, a3, a4, a5, a6);
		TEST_CHECK("S5-U09", (a0 >= 120) && (a0 <= 215) &&
			(a1 >= 120) && (a1 <= 215) &&
			   (a2 >= 120) && (a2 <= 215) && (a3 >= 120) &&
			   	(a3 <= 215) &&
			   (a4 >= 100) && (a4 <= 215) && (a5 == -999) &&
			   	(a6 == -999),
			   "glance: whole car width covered, off-path and "
			   "stray echoes ignored");
	}
}

EXPORT void subsystem5_test(void)
{
	test_init();
	s5_test_scan();
	s5_test_statechart();
	s5_test_plans();
	s5_test_glance();
	test_summary("subsystem5");
}

#endif /* APP_TEST */
