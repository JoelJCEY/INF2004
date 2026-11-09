/*
 *	task_scan.c
 *	Subsystem 5 -- scan_task: obstacle watch, coarse and fine sweeps,
 *	profile and bypass side, side guard (statechart 6.5).
 *
 *	  WATCHING        line following: ping straight ahead every 60 ms;
 *	                  two readings inside the stop distance -> OB_ALERT
 *	  PAUSED ON HUMP  hump bit set: readings ignored, bounded by distance
 *	  COARSE / FINE   OBSTACLE BYPASS: full 5° sweep left to right, then
 *	                  confirm (2+ angles in path) before any bypass
 *	  GUARD           during the bypass legs: alert if too close ahead
 *	scan_task only RECOMMENDS a side; ctrl_task issues every move.
 */

#include "subsystem5.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"
#include "scan_profile.h"
#include "ctrl_logic.h"		/* CTRL_BYPASS_BACKUP_MM (31 Oct) */
#include "../subsystem2/subsystem2.h"	/* IF-07 odometry */

#define US_MIN_VALID_MM		40U	/* 28 Oct log: the car's own front bracket
					   echoes at 26-35 mm, every time; closer
					   than this is the car, not an obstacle */
#define WATCH_SLEEP_MS		60	/* 31 Oct (was 50): the 60 ms gap the
					   HC-SR04 manual asks for            */
#define HUMP_IGNORE_MAX_MM	400U	/* bound on the hump pause (F10)   */
#define GUARD_MIN_MM		120U
#define NO_ECHO_LIMIT		3
#define SWEEP_TRIES		3	/* pings per sweep angle before giving up (31 Oct) */
#define STOP_WAIT_MS		400	/* let the car stop before sweeping */

typedef enum {
	SC_IDLE		= 0,
	SC_WATCHING	= 1,
	SC_PAUSED_HUMP	= 2,
	SC_COARSE	= 3,
	SC_FINE		= 4,
	SC_GUARD	= 5
} SCAN_STATE;

EXPORT volatile UW scan_status_word;
EXPORT volatile UW scan_rescan_request;
EXPORT volatile UW scan_pass_watch;
EXPORT volatile UW scan_pass_seq;
EXPORT volatile UW scan_pass_extra_mm;

/* 31 Oct: side watch while passing the obstacle */
#define PASS_SIDE_TOL_MM	60	/* 4 Nov: later hits must be this close to the
					   first hit's sideways distance: the obstacle's
					   side is a straight face; walls, floor or other
					   things farther out no longer count */
#define PASS_SIDE_MM		300	/* echo this close SIDEWAYS = still beside it
					   (4 Nov; was 400 mm range at 45 deg = 283 sideways) */
#define PASS_GONE_PINGS		2	/* this many misses in a row = behind us   */
#define PASS_MARGIN_MM		50	/* room for the rear to swing when turning */
LOCAL UW	last_depth_mm;		/* fixed estimate from the last profile */

/* 4 Nov: GLANCE scan while driving.  The beam is ~15 deg wide, so looking
   only straight ahead it covers +-2.6 cm at 20 cm: an obstacle a few cm
   off the line could be missed and clipped by the car's corner.  The
   sensor now cycles right / centre / left / centre by GLANCE_DEG.  Each
   reading becomes a point; the car stops only for a point that is IN
   ITS PATH (sideways within half width + margin) and closer than the
   stop distance AHEAD.  GLANCE_DEG 0 = old straight-ahead watching. */
#define GLANCE_DEG		10	/* 10 or 15 */
#define GLANCE_STEPS		(GLANCE_DEG / 5)
#define GLANCE_SETTLE_MS	40	/* after the (ramped) move, before the ping */
#define GLANCE_HITS		2	/* stop: one position hits twice in a row, or
					   two positions see something at once */
LOCAL const INT glance_seq[4] = { -GLANCE_STEPS, 0, GLANCE_STEPS, 0 };	/* R, C, L, C */
LOCAL const UB	glance_pos[4] = { 0, 1, 2, 1 };	/* counter per position: R, C, L */
LOCAL UINT	glance_i;
LOCAL UB	glance_hits[3];			/* in-path hits in a row, per position */
LOCAL void glance_clear(void)
{
	glance_hits[0] = 0;
	glance_hits[1] = 0;
	glance_hits[2] = 0;
}


LOCAL BOOL	pw_on, pw_seen, pw_done;	/* side watch state */
LOCAL INT	pw_gone;
LOCAL UW	pw_start, pw_last_mm, pw_last_path;
LOCAL W		pw_side0;		/* sideways distance of the first hit */

LOCAL UINT	servo_idx = SERVO_CENTRE_IDX;

LOCAL void set_state(SCAN_STATE st, UW mm)
{
	scan_status_word = (servo_idx & 0x1FUL) | (((UW)st & 0x7UL) << 5) |
			   ((mm & 0x1FFFUL) << 16);
}

EXPORT volatile UH scan_last_sweep[SCAN_POSITIONS];
EXPORT volatile UW scan_sweep_seq;
EXPORT volatile UW scan_bench_on;
EXPORT volatile UW scan_hold;
EXPORT volatile UW scan_bench_word;
EXPORT volatile UW scan_bench_seq;

/* Point the servo (with settling time) and ping once */
LOCAL US_RESULT ping_at(UINT idx, UW *mm)
{
	UINT	steps;

	if (idx != servo_idx) {
		steps = (UINT)APP_ABS((INT)idx - (INT)servo_idx);
		drv_servo_set_idx(idx);
		servo_idx = idx;
		/* ~10 ms per 5° step plus settling, at least the ping gap */
		tk_dly_tsk((RELTIM)(SERVO_SETTLE_MS + (steps * 10U)));
	} else {
		tk_dly_tsk(WATCH_SLEEP_MS);	/* keeps pings >= 60 ms apart */
	}
	return drv_us_ping(mm);
}

/* Ping at idx; a move is short (GLANCE_DEG), so only a short settle */
LOCAL US_RESULT glance_ping(UINT idx, UW *mm)
{
	if (idx != servo_idx) {
		drv_servo_set_idx(idx);		/* ramps, returns when there */
		servo_idx = idx;
		tk_dly_tsk(GLANCE_SETTLE_MS);
	} else {
		tk_dly_tsk(WATCH_SLEEP_MS);	/* keeps pings >= 60 ms apart */
	}
	return drv_us_ping(mm);
}

LOCAL void send_obstacle(const OBSTACLE_MSG *m)
{
	if (tk_snd_mbf(mbf_obstacle, m, sizeof(*m), TMO_POL) < E_OK) {
		app_log((UB*)"scan: obstacle queue full\n");
	}
	(void)tk_set_flg(flg_sys, EVF_OBSTACLE);
}

LOCAL void send_kind(OBSTACLE_KIND k)
{
	OBSTACLE_MSG	m = {0};

	m.kind = (UB)k;
	m.servo_idx = (UB)servo_idx;
	send_obstacle(&m);
}

/* Is the alert real?  The whole front must show something in the
   car's path, closer than SCAN_CONFIRM_MM, at 2 or more neighbouring
   angles (one stray echo is not an obstacle).  Team request 30 Oct:
   scan the whole area first, then decide. */
LOCAL BOOL confirmed(const SCAN_SWEEP *sw, UW *hits_out, UW *near_out)
{
	INT	i, run = 0, best = 0;
	UW	d, nearest = 0xFFFFU;
	W	x;

	for (i = SCAN_IDX_MAX; i >= SCAN_IDX_MIN; i--) {
		d = sw->dist[i];
		x = APP_ABS(((W)d * scan_cos1000((UINT)i)) / 1000);
		if ((d != SCAN_NO_READING) && (d >= US_MIN_VALID_MM) &&
		    (d < SCAN_CONFIRM_MM) &&
		    (x <= (SCAN_CAR_HALF_W_MM + SCAN_MARGIN_MM))) {
			run++;
			if (d < nearest) nearest = d;
		} else {
			run = 0;
		}
		if (run > best) best = run;
	}
	*hits_out = (UW)best;
	*near_out = (nearest == 0xFFFFU) ? 0U : nearest;
	return (best >= SCAN_CONFIRM_HITS) ? TRUE : FALSE;
}

/* Sweep of the front, left (120°) to right (70°) in 5°
   steps, then confirm (first sweep only) and work out the profile.
   check = TRUE: this sweep decides if the alert was a real obstacle. */
LOCAL void do_sweep(BOOL check)
{
	SCAN_SWEEP	sw;
	SCAN_RESULT	res;
	OBSTACLE_MSG	m = {0};
	UW		mm = 0, hits = 0, nearest = 0;
	INT		i;
	INT		noecho = 0;
	US_RESULT	r;
	UW		t0 = app_time_ms();

	for (i = 0; i < SCAN_POSITIONS; i++) {
		sw.dist[i] = SCAN_NO_READING;
	}

	/* Wait (bounded) for the car to stop after OB_ALERT */
	while ((APP_ABS(motion_speed_avg()) > 20) &&
	       ((app_time_ms() - t0) < STOP_WAIT_MS)) {
		tk_dly_tsk(20);
	}

	set_state(SC_COARSE, 0);
	app_log((UB*)"scan: sweeping the whole front, left to right\n");
	/* 31 Oct: each angle gets up to SWEEP_TRIES pings.  An angle that
	   never answers is left unmeasured (the profile skips it).  Only
	   when more than half the angles never answer is it a sensor
	   fault.  Before, 3 silent angles in a row failed the bypass. */
	for (i = SCAN_IDX_MAX; i >= SCAN_IDX_MIN; i--) {
		INT	tries;

		r = US_NO_ECHO;
		for (tries = 0; (tries < SWEEP_TRIES) && (r == US_NO_ECHO); tries++) {
			r = ping_at((UINT)i, &mm);	/* same angle: 60 ms gap */
		}
		if (r == US_NO_ECHO) {
			noecho++;
			continue;			/* left unmeasured */
		}
		sw.dist[i] = (UH)(((r == US_OK) && (mm >= US_MIN_VALID_MM)) ? mm : US_FAR_MM);
	}
	if (noecho > 0) {
		app_log((UB*)"scan: %d of %d angles gave no echo after %d tries\n",
			  noecho, SCAN_IDX_MAX - SCAN_IDX_MIN + 1, SWEEP_TRIES);
	}
	if ((noecho * 2) > (SCAN_IDX_MAX - SCAN_IDX_MIN + 1)) {
		app_report_fault(FLT_NO_ECHO);
		send_kind(OB_SENSOR_FAULT);	/* = one failed attempt */
		drv_servo_set_idx(SERVO_CENTRE_IDX);
		servo_idx = SERVO_CENTRE_IDX;
		return;
	}

	/* Publish the sweep for the dashboard (one copy, then the count) */
	for (i = 0; i < SCAN_POSITIONS; i++) {
		scan_last_sweep[i] = sw.dist[i];
	}
	scan_sweep_seq++;

	m.servo_idx = (UB)servo_idx;
	set_state(SC_FINE, 0);
	if (check && !confirmed(&sw, &hits, &nearest)) {
		/* false alarm: closest = 0 tells ctrl_task to carry on */
		m.kind = OB_PROFILE;
		m.side = OBS_SIDE_NONE;
		app_log((UB*)"scan: NOT an obstacle (in path under %d mm at only %d angle(s), need %d). Carrying on (%d ms)\n",
			  SCAN_CONFIRM_MM, hits, SCAN_CONFIRM_HITS, app_time_ms() - t0);
	} else if (scan_profile(&sw, &res) == E_OK) {
		if (check) {
			app_log((UB*)"scan: obstacle CONFIRMED at %d mm, seen at %d angles\n",
				  nearest, hits);
		}
		m.kind = res.kind;
		m.side = res.side;
		m.width_mm = res.width_mm;
		m.closest_mm = res.closest_mm;
		m.offset_mm = res.offset_mm;
		m.depth_mm = res.depth_mm;
		last_depth_mm = res.depth_mm;
	} else {
		m.kind = OB_PROFILE;		/* obstacle gone: closest = 0 */
		m.side = OBS_SIDE_NONE;
	}
	app_log((UB*)"scan: profile kind %d side %d width %d closest %d offset %d depth %d (%d ms)\n",
		  m.kind, m.side, m.width_mm, m.closest_mm, m.offset_mm, m.depth_mm,
		  app_time_ms() - t0);
	send_obstacle(&m);

	drv_servo_set_idx(SERVO_CENTRE_IDX);
	servo_idx = SERVO_CENTRE_IDX;
}

EXPORT void scan_task(INT stacd, void *exinf)
{
	MISSION_STATE	ms;
	T_RFLG		rflg;
	UW		mm = 0;
	US_RESULT	r;
	INT		confirm = 0;
	INT		noecho = 0;
	BOOL		alerted = FALSE;
	BOOL		swept = FALSE;
	BOOL		paused = FALSE;
	BOOL		guard_sent = FALSE;
	UW		pause_start = 0;
	UW		last_req;

	(void)stacd; (void)exinf;

	drv_servo_init();
	if (drv_us_init() < E_OK) {
		app_log((UB*)"scan: ECHO interrupt not available\n");
	}
	last_req = scan_rescan_request;

	while (1) {
		if (scan_hold != 0U) {
			tk_dly_tsk(50);
			continue;
		}
		ctrl_get_mission(&ms);

		switch ((VEHICLE_STATE)ms.state) {
		case VS_LINE_FOLLOWING:
		case VS_BARCODE_MANOEUVRE:
		case VS_HUMP_CROSSING:
			swept = FALSE;
			guard_sent = FALSE;
			(void)tk_ref_flg(flg_sys, &rflg);
			if ((rflg.flgptn & EVF_HUMP_ACTIVE) != 0U) {
				if (!paused) {
					paused = TRUE;
					pause_start = motion_odo_path_01mm / 10U;
				}
				if (((motion_odo_path_01mm / 10U) - pause_start) < HUMP_IGNORE_MAX_MM) {
					/* the sensor may be seeing the hump itself */
					set_state(SC_PAUSED_HUMP, 0);
					confirm = 0;
					glance_clear();
					tk_dly_tsk(S5_SCAN_PERIOD_MS);
					break;
				}
			} else {
				paused = FALSE;
			}

			{
				UINT	gidx = (UINT)((INT)SERVO_CENTRE_IDX + glance_seq[glance_i]);
				UB	gp = glance_pos[glance_i];

				glance_i = (glance_i + 1U) % 4U;
				r = glance_ping(gidx, &mm);
				if (r == US_OK) {
					W	side = ((W)mm * scan_cos1000(gidx)) / 1000;	/* + = right */
					W	ahead = ((W)mm * scan_sin1000(gidx)) / 1000;

					noecho = 0;
					set_state(SC_WATCHING, mm);
					/* Below US_MIN_VALID_MM the HC-SR04 is not reliable
					   (its own bracket, wires, ringing): ignored */
					if (SCAN_ENABLE && (mm >= US_MIN_VALID_MM) &&
					    (ahead < SCAN_STOP_DIST_MM) &&
					    (APP_ABS(side) <= (SCAN_CAR_HALF_W_MM + SCAN_MARGIN_MM))) {
						INT	busy;

						if (glance_hits[gp] < 255U) {
							glance_hits[gp]++;
						}
						busy = (glance_hits[0] > 0U) + (glance_hits[1] > 0U) +
						       (glance_hits[2] > 0U);
						confirm = ((glance_hits[gp] >= GLANCE_HITS) || (busy >= 2)) ?
							  GLANCE_HITS : 0;
						if ((confirm >= GLANCE_HITS) && !alerted) {
							alerted = TRUE;
							send_kind(OB_ALERT);
							app_log((UB*)"scan: something at %d mm, %d deg (%d mm ahead, %d mm %s), stopping to check\n",
								  mm, SERVO_IDX_TO_DEG(gidx), ahead, APP_ABS(side),
								  (side >= 0) ? "right" : "left");
						}
					} else {
						glance_hits[gp] = 0;	/* clear here: only this position */
					}
				} else if (r == US_FAR) {
					noecho = 0;
					glance_hits[gp] = 0;
					set_state(SC_WATCHING, US_FAR_MM);
				} else {
					/* no echo: no information, keep the counts */
					if (++noecho == NO_ECHO_LIMIT) {
						app_report_fault(FLT_NO_ECHO);	/* reported, latched */
						app_log((UB*)"scan: no echo x3\n");
					}
				}
			}
			break;

		case VS_OBSTACLE_BYPASS:
			alerted = FALSE;
			confirm = 0;
			glance_clear();
			if (!swept || (scan_rescan_request != last_req)) {
				/* first sweep after an alert decides if it is real;
				   a rescan after reversing is already confirmed */
				do_sweep(!swept);
				last_req = scan_rescan_request;
				swept = TRUE;
				guard_sent = FALSE;
				break;
			}
			/* 31 Oct: PASS leg: watch the obstacle's side go by.  The
			   beam looks SCAN_PASS_LOOK_DEG, so a hit at d mm is
			   d x sin(angle) ahead of the sensor.  When it stops
			   echoing, its back end was last seen there: drive that
			   far, plus the car length and a margin, then turn in. */
			if (scan_pass_watch != 0U) {
				UINT		idx = scan_pass_watch - 1U;
				UW		path = motion_odo_path_01mm / 10U;

				if (!pw_on) {
					pw_on = TRUE;
					pw_seen = FALSE;
					pw_done = FALSE;
					pw_gone = 0;
					pw_start = path;
				}
				r = ping_at(idx, &mm);
				set_state(SC_GUARD, (r == US_OK) ? mm : US_FAR_MM);
				if (pw_done) {
					break;
				}
				if ((r == US_OK) && (mm >= US_MIN_VALID_MM) &&
				    ((((W)mm * APP_ABS(scan_cos1000(idx))) / 1000) < PASS_SIDE_MM) &&
				    (!pw_seen ||
				     (APP_ABS((((W)mm * APP_ABS(scan_cos1000(idx))) / 1000) - pw_side0) <= PASS_SIDE_TOL_MM))) {
					if (!pw_seen) {
						pw_side0 = ((W)mm * APP_ABS(scan_cos1000(idx))) / 1000;
						app_log((UB*)"scan: side of the obstacle at %d mm (%d mm to the side)\n",
							  mm, pw_side0);
					}
					pw_seen = TRUE;
					pw_gone = 0;
					pw_last_mm = mm;
					pw_last_path = path;
				} else if ((r != US_NO_ECHO) && pw_seen && (++pw_gone >= PASS_GONE_PINGS)) {
					W ahead = ((W)pw_last_mm * scan_sin1000(idx)) / 1000;
					W extra = ahead - (W)(path - pw_last_path) +
						  SCAN_CAR_LEN_MM + PASS_MARGIN_MM;

					scan_pass_extra_mm = (UW)((extra > 0) ? extra : 0);
					scan_pass_seq++;
					pw_done = TRUE;
					app_log((UB*)"scan: obstacle end seen %d mm ahead, %d mm to go\n",
						  ahead, scan_pass_extra_mm);
				} else if (!pw_seen &&
					   ((path - pw_start) >= (last_depth_mm + CTRL_BYPASS_BACKUP_MM))) {
					/* never saw its side: stop at the fixed estimate */
					scan_pass_extra_mm = 0;
					scan_pass_seq++;
					pw_done = TRUE;
					app_log((UB*)"scan: side of the obstacle not seen, using the estimate\n");
				}
				break;
			}
			pw_on = FALSE;	/* no pass watch: the next one starts fresh */
			/* GUARD: the bypass legs must not run into anything */
			r = ping_at(SERVO_CENTRE_IDX, &mm);
			set_state(SC_GUARD, (r == US_OK) ? mm : US_FAR_MM);
			if ((r == US_OK) && (mm < GUARD_MIN_MM)) {
				if ((++confirm >= 2) && !guard_sent) {
					guard_sent = TRUE;
					send_kind(OB_CLEARANCE_LOST);
				}
			} else {
				confirm = 0;
			}
			break;

		default:
			/* Not moving: no pings (resource use, 1.0) */
			alerted = FALSE;
			swept = FALSE;
			confirm = 0;
			glance_clear();
			if (servo_idx != SERVO_CENTRE_IDX) {
				drv_servo_set_idx(SERVO_CENTRE_IDX);
				servo_idx = SERVO_CENTRE_IDX;
			}
			if (scan_bench_on != 0U) {
				r = ping_at(SERVO_CENTRE_IDX, &mm);
				scan_bench_word = ((UW)r << 16) | (mm & 0xFFFFU);
				scan_bench_seq++;
				set_state(SC_IDLE, (r == US_OK) ? mm : US_FAR_MM);
			} else {
				/* Parked: a slow ping (5 Hz) so the dashboard can show
				   the distance ahead; the result is display only */
				tk_dly_tsk(150);
				r = drv_us_ping(&mm);
				set_state(SC_IDLE, (r == US_OK) ? mm :
					  ((r == US_FAR) ? US_FAR_MM : 0U));
			}
			break;
		}
	}
}
