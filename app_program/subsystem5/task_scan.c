/*
 *	task_scan.c
 *	Subsystem 5 -- scan_task (COMPLETE VERSION).  Statechart 6.5:
 *	WATCHING -> COARSE SWEEP -> FINE SWEEP -> PROFILE, PAUSED ON HUMP.
 *
 *	WATCHING: servo at 90 degrees, one ping every 60 ms.  Two readings
 *	inside the stop distance -> OBS_ALERT, then straight into a profile:
 *	  coarse: 30, 60, 90, 120, 150 degrees;
 *	  fine:   5-degree steps from 15 degrees outside the outermost hits;
 *	  (about 3 s in total with the car stopped: check against NFR2);
 *	  profile: closest point, width, how far the obstacle reaches left
 *	  and right of the car's centre line, the free space beyond each
 *	  edge, and the recommended side (the one needing the smaller
 *	  sideways move, so the deviation from the route stays small).
 *	ctrl_task can ask for a new profile (after reversing) or HOLD
 *	(during the bypass) through s5_scan_request.
 *	While the hump bit is set readings are ignored, but only for
 *	S5_HUMP_IGNORE_MAX_MM of travel.
 */

#include "subsystem5.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"
#include "../subsystem2/subsystem2.h"	/* odometer */

EXPORT volatile BOOL		s5_scan_ready = FALSE;
EXPORT volatile W		s5_last_mm = -1;
EXPORT volatile UW		s5_ping_count = 0U;
EXPORT volatile UW		s5_echo_isr_count = 0U;
EXPORT volatile OBSTACLE_MSG	s5_last_profile;

#define N_ANGLES	25	/* 30..150 in 5-degree steps */
#define ANGLE_IDX(a)	(((a) - 30) / 5)
#define NO_READING	(-1)

/* cos x 1000 for 30..150 degrees in 5-degree steps */
LOCAL const W cos_tab[N_ANGLES] = {
	866, 819, 766, 707, 643, 574, 500, 423, 342, 259, 174, 87, 0,
	-87, -174, -259, -342, -423, -500, -574, -643, -707, -766, -819, -866
};
/* sin x 1000, same angles */
LOCAL const W sin_tab[N_ANGLES] = {
	500, 574, 643, 707, 766, 819, 866, 906, 940, 966, 985, 996, 1000,
	996, 985, 966, 940, 906, 866, 819, 766, 707, 643, 574, 500
};

LOCAL W	scan_mm[N_ANGLES];	/* NO_READING, or mm */
LOCAL UINT	no_echo_run = 0U;

LOCAL void s5_post_obs(const OBSTACLE_MSG *p_m)
{
	(void)s5_post(mbf_obstacle, p_m, (INT)sizeof(*p_m), FLG_OBSTACLE);
}

LOCAL void s5_post_kind(OBSTACLE_KIND kind, W mm)
{
	OBSTACLE_MSG	m;

	m.kind = kind;
	m.distance_mm = mm;
	m.width_mm = 0;
	m.left_mm = 0;
	m.right_mm = 0;
	m.left_clear_mm = 0;
	m.right_clear_mm = 0;
	m.side = SIDE_NONE;
	s5_post_obs(&m);
}

/* One ping with no-echo bookkeeping.  Returns mm, or NO_READING. */
LOCAL W s5_ping_mm(BOOL *p_fault)
{
	UW		echo_us = 0U;
	UW		mm = 0U;
	S5_PING_RESULT	res = us_ping(&echo_us, &mm);

	s5_ping_count++;
	*p_fault = FALSE;
	if (res == S5_PING_NO_ECHO) {
		if (++no_echo_run >= (UINT)S5_NO_ECHO_FAULT_N) {
			*p_fault = TRUE;
		}
		return NO_READING;
	}
	no_echo_run = 0U;
	return (res == S5_PING_OK) ? (W)mm : NO_READING;
}

/* Servo angle in "maths" terms: 90 = ahead, > 90 = left. */
LOCAL void s5_point(W angle)
{
	W	a = (S5_SERVO_HIGH_IS_LEFT != 0) ? angle : (180 - angle);

	servo_set_deg((UINT)a);
	(void)tk_dly_tsk(S5_SERVO_SETTLE_MS);
}

/* Measure one angle.  Coarse angles use two pings (keep the nearer valid
 * one); fine angles one ping, to keep the whole profile near 3 s. */
LOCAL BOOL s5_measure(W angle, BOOL two_pings)
{
	BOOL	f1;
	BOOL	f2 = FALSE;
	W	a;
	W	b;

	s5_point(angle);
	a = s5_ping_mm(&f1);
	if (two_pings == TRUE) {
		(void)tk_dly_tsk(S5_SCAN_PERIOD_MS);
		b = s5_ping_mm(&f2);
		if ((a == NO_READING) || ((b != NO_READING) && (b < a))) {
			a = b;
		}
	}
	scan_mm[ANGLE_IDX(angle)] = a;
	return ((f1 == TRUE) || (f2 == TRUE)) ? TRUE : FALSE;
}

LOCAL BOOL s5_is_hit(INT i)
{
	return ((scan_mm[i] != NO_READING) && (scan_mm[i] < S5_PROFILE_RANGE_MM)) ? TRUE : FALSE;
}

/* Coarse + fine scan -> profile message.  Car is stopped. */
LOCAL void s5_profile(void)
{
	OBSTACLE_MSG	m;
	INT		i;
	INT		lo = -1;	/* lowest hit index (rightmost) */
	INT		hi = -1;	/* highest hit index (leftmost) */
	W		a;
	BOOL		fault = FALSE;
	W		y_min = 100000;	/* lateral, + = right of centre */
	W		y_max = -100000;
	W		fwd_min = 100000;

	for (i = 0; i < N_ANGLES; i++) {
		scan_mm[i] = NO_READING;
	}
	for (a = 30; a <= 150; a += S5_COARSE_STEP_DEG) {
		if (s5_measure(a, TRUE) == TRUE) {
			fault = TRUE;
		}
	}
	if (fault == TRUE) {
		s5_raise_fault(FAULT_NO_ECHO);
		s5_post_kind(OBS_SENSOR_FAULT, 0);
		s5_point(90);
		return;
	}
	for (i = 0; i < N_ANGLES; i++) {
		if (s5_is_hit(i) == TRUE) {
			if (lo < 0) {
				lo = i;
			}
			hi = i;
		}
	}
	if (lo < 0) {
		s5_post_kind(OBS_CLEAR, -1);	/* gone: nothing in range */
		s5_point(90);
		return;
	}
	/* fine sweep: 5-degree steps, from 15 degrees outside the outer
	 * coarse hits (a coarse step is 30 degrees, so an edge lies within
	 * 30 degrees of the last hit; 15 covers the nearer half and the
	 * profile stays near 3 s -- widen if edges are missed) */
	{
		INT	f_lo = (lo - 3 < 0) ? 0 : (lo - 3);
		INT	f_hi = (hi + 3 >= N_ANGLES) ? (N_ANGLES - 1) : (hi + 3);
		for (i = f_lo; i <= f_hi; i++) {
			if ((i % 6) != 0) {		/* coarse angles already done */
				(void)s5_measure(30 + (i * 5), FALSE);
			}
		}
	}
	lo = -1;
	hi = -1;
	for (i = 0; i < N_ANGLES; i++) {
		if (s5_is_hit(i) == TRUE) {
			W	y = (scan_mm[i] * cos_tab[i]) / 1000;
			W	x = (scan_mm[i] * sin_tab[i]) / 1000;
			if (lo < 0) {
				lo = i;
			}
			hi = i;
			if (y < y_min) {
				y_min = y;
			}
			if (y > y_max) {
				y_max = y;
			}
			if (x < fwd_min) {
				fwd_min = x;
			}
		}
	}
	m.kind = OBS_PROFILE;
	m.distance_mm = fwd_min;
	m.width_mm = y_max - y_min;
	m.right_mm = y_max;		/* reach to the right of centre */
	m.left_mm = -y_min;		/* reach to the left of centre */
	/* free space beyond each edge: next valid reading outside the edge */
	m.right_clear_mm = 999;
	m.left_clear_mm = 999;
	for (i = lo - 1; i >= 0; i--) {
		if (scan_mm[i] != NO_READING) {
			W	y = (scan_mm[i] * cos_tab[i]) / 1000;
			m.right_clear_mm = (y - y_max > 0) ? (y - y_max) : 0;
			break;
		}
	}
	for (i = hi + 1; i < N_ANGLES; i++) {
		if (scan_mm[i] != NO_READING) {
			W	y = -((scan_mm[i] * cos_tab[i]) / 1000);
			m.left_clear_mm = (y - m.left_mm > 0) ? (y - m.left_mm) : 0;
			break;
		}
	}
	{
		BOOL	l_ok = (m.left_clear_mm >= S5_CLEAR_NEED_MM) ? TRUE : FALSE;
		BOOL	r_ok = (m.right_clear_mm >= S5_CLEAR_NEED_MM) ? TRUE : FALSE;

		if ((l_ok == TRUE) && (r_ok == TRUE)) {
			/* smaller sideways move = smaller deviation */
			m.side = (m.left_mm <= m.right_mm) ? SIDE_LEFT : SIDE_RIGHT;
		} else if (l_ok == TRUE) {
			m.side = SIDE_LEFT;
		} else if (r_ok == TRUE) {
			m.side = SIDE_RIGHT;
		} else {
			m.side = SIDE_BLOCKED;
		}
	}
	s5_last_profile = m;
	if (S5_PRINT_ENABLE != 0) {
		tm_printf((UB *)"[S5] profile: closest %d mm, width %d, reach L %d R %d, clear L %d R %d, side %d\n",
			  (INT)m.distance_mm, (INT)m.width_mm, (INT)m.left_mm, (INT)m.right_mm,
			  (INT)m.left_clear_mm, (INT)m.right_clear_mm, (INT)m.side);
	}
	s5_post_obs(&m);
	s5_point(90);
}

EXPORT void scan_task(INT stacd, void *exinf)
{
	UINT	near_n = 0U;
	UINT	clear_n = 0U;
	BOOL	alerted = FALSE;
	BOOL	hump_seen = FALSE;
	W	hump_from_mm = 0;
	UW	seen_seq = 0U;
	ER	er;

	(void)stacd;
	(void)exinf;

	er = us_init();
	if (er < E_OK) {
		tm_printf((UB *)"[S5] ECHO interrupt could not be defined (%d)\n", (INT)er);
	}
	(void)servo_init();
	s5_scan_ready = TRUE;

	while (1) {
		BOOL	fault = FALSE;
		W	mm;
		T_RFLG	rflg;
		SCAN_REQUEST	req = s5_scan_request;

		(void)tk_dly_tsk(S5_SCAN_PERIOD_MS);

		if ((req == SCAN_REQ_PROFILE) && (s5_scan_request_seq != seen_seq)) {
			seen_seq = s5_scan_request_seq;
			s5_profile();
			continue;
		}
		if (req == SCAN_REQ_HOLD) {
			near_n = 0U;
			alerted = FALSE;
			continue;
		}

		/* hump: ignore, but only for a bounded distance */
		if ((tk_ref_flg(flg_ctrl, &rflg) == E_OK) &&
		    ((rflg.flgptn & FLG_HUMP_ACTIVE) != 0U)) {
			if (hump_seen == FALSE) {
				hump_seen = TRUE;
				hump_from_mm = s2_distance_mm;
			}
			if ((s2_distance_mm - hump_from_mm) < S5_HUMP_IGNORE_MAX_MM) {
				near_n = 0U;
				continue;
			}
		} else {
			hump_seen = FALSE;
		}

		mm = s5_ping_mm(&fault);
		if (fault == TRUE) {
			s5_raise_fault(FAULT_NO_ECHO);
			no_echo_run = 0U;
			continue;
		}
		s5_last_mm = mm;

		if ((mm != NO_READING) && (mm <= S5_STOP_MM)) {
			clear_n = 0U;
			if ((++near_n >= (UINT)S5_CONFIRM_N) && (alerted == FALSE)) {
				alerted = TRUE;
				s5_post_kind(OBS_ALERT, mm);
				s5_profile();		/* ctrl_task stops the car */
			}
		} else if ((mm == NO_READING) || (mm > S5_CLEAR_MM)) {
			near_n = 0U;
			if ((++clear_n >= (UINT)S5_CLEAR_N) && (alerted == TRUE)) {
				alerted = FALSE;
			}
		} else {
			near_n = 0U;
		}
	}
}
