/*
 *	task_motion.c
 *	Subsystem 2 -- motion_task: 20 ms speed control, measured moves and
 *	turns, stall and encoder fault detection (statechart 6.2).
 *
 *	Every 20 ms (released by a cyclic handler, so the period does not
 *	drift with the task's own run time):
 *	  1. read both PIO encoder counts, update speed and odometry
 *	  2. take every queued MOTION_CMD (a new command replaces the
 *	     current one; MC_STOP pre-empts anything)
 *	  3. run the current mode and compute both wheel duties
 *	  4. check the stall and encoder rules
 *	  5. write the PWM (soft start: duty rise limited per tick)
 *
 *	Control structure (8.1 Buddy 2 decision): one common-mode speed PI
 *	on the average wheel speed, plus ONE differential correction:
 *	from the line error while following, from the left/right count
 *	difference while moving or turning.  No auto-tuning, no slip model.
 */

#include "subsystem2.h"
#include "motion_ctrl.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "../subsystem3/subsystem3.h"	/* IF-08 line_latest   */
#include "../subsystem5/subsystem5.h"	/* IF-06, flg_sys      */

/* ---------------------------------------------------------------------
 *  Tuning (design values; set from bench tests under A2)
 * --------------------------------------------------------------------- */
#define FF_OFFSET_PERMILLE	150	/* overcomes static friction     */
#define FF_PERMILLE_PER_M_S	1500	/* duty per 1 m/s                */
#define LOST_SPIN_MM_S		60	/* wheel speed when spinning back to the line */
#define SYNC_DEADBAND		13	/* counts, ~1 mm: L/R difference ignored (31 Oct) */
/* Straight-line wheel sync while FOLLOWING (2 Nov): while the line error
   is 0 the car should go straight, but the two motors are never equal, so
   with the same power one wheel runs faster and the car drifts until the
   sensors pull it back.  Now, while the error is 0, the left/right count
   difference since the straight began is driven to zero (same PI as the
   measured moves).  Its integral is kept through curves as a learned
   motor trim, so the weaker motor gets its extra power straight away on
   the next straight.  See MOTION_SYNC_R_X1000 in subsystem2.h. */
#define FOLLOW_SYNC_LOG_MS	1000	/* print speeds while following (0 = off) */
/* Gradual steering while FOLLOWING (2 Nov): the steering part of the
   duty (right minus left, halved) and the forward part each move toward
   their new value by at most this much per 20 ms tick, instead of
   jumping.  A correction builds up over ~100 ms, and the spin back to a
   lost line starts as a tightening arc rather than a sudden pivot, so the
   car does not jerk (and the barcode sensor is not thrown across the
   bars).  Smaller = smoother but slower to react. */
#define STEER_SLEW_PER_TICK	30	/* duty permille per tick */
#define ACCEL_MM_S_PER_TICK	12	/* 600 mm/s^2 setpoint ramp       */
#define SLEW_PERMILLE_PER_TICK	60	/* soft start: 0 -> 600 in 200 ms */
#define SPEED_AVG_TICKS		3	/* average over 60 ms (VA Buddy 2)*/
#define LINE_STALE_MS		60	/* older line values are held     */
#define STOP_SPEED_MM_S		15
#define STOP_TIMEOUT_TICKS	50	/* 1 s */

#define ENC_CHECK_DUTY		300	/* both wheels driven above this  */
#define ENC_CHECK_MIN_COUNTS	(20000 / MOTION_UM_PER_COUNT)	/* other wheel moved >= 20 mm */
/* From counts alone a blocked wheel and a dead encoder look the same.
   The stall rule (300 ms) therefore reports a blocked wheel first; this
   longer window only catches what the stall rule cannot see.  The exact
   encoder rule is set from the A2 tests (fault table 5.1). */
#define ENC_CHECK_TICKS		50	/* 1 s */

#define SELFTEST_DUTY		400
#define SELFTEST_TICKS		10	/* 200 ms per step                */
#define SELFTEST_MIN_COUNTS	(10000 / MOTION_UM_PER_COUNT)	/* >= 10 mm forward */

/* ---------------------------------------------------------------------
 *  Published words (IF-07): motion_task is the only writer
 * --------------------------------------------------------------------- */
EXPORT volatile UW motion_odo_path_01mm;
EXPORT volatile UW motion_odo_speed = 0x80008000UL;	/* 0 mm/s, 0 mm/s */
EXPORT volatile W  motion_enc_count[2];
EXPORT volatile UW motion_mode_word;
EXPORT volatile UW motion_jitter_max_us;

/* ---------------------------------------------------------------------
 *  Task state (private to motion_task)
 * --------------------------------------------------------------------- */
typedef struct {
	MOTION_MODE	mode;
	UB		seq;		/* seq of the active command */
	W		dir;		/* +1 / -1 for MOVE and TURN */
	W		target_counts;
	W		start_l;
	W		start_r;
	W		cruise;		/* requested speed, mm/s     */
	W		v_set;		/* ramped setpoint, mm/s     */
	W		limit;		/* 0 = no limit              */
	INT		duty_l;
	INT		duty_r;
	UB		settle;		/* STOPPING: ticks at rest   */
	UB		timer;		/* generic tick counter      */
	UB		st_step;	/* SELFTEST step             */
	W		st_ref;		/* SELFTEST reference count  */
	BOOL		st_ok;
	INT		line_err;	/* last fresh line error     */
	BOOL		straight;	/* following with error 0    */
	INT		fwd;		/* forward duty now (slewed) */
	INT		steer;		/* steering duty now (slewed) */
	W		base_l;		/* counts when it began      */
	W		base_r;
	UW		log_ms;
} MOTION_CTX;

LOCAL MOTION_CTX	mc;
LOCAL MOTION_PID	pid_speed;	/* common mode: speed            */
LOCAL MOTION_PID	pid_line;	/* differential: line error      */
LOCAL MOTION_PID	pid_sync;	/* differential: L/R count sync  */
LOCAL MOTION_STALL	stall[2];	/* per wheel: one blocked wheel is a stall */

LOCAL W		cnt_l, cnt_r;		/* latest counts (sign-corrected) */
LOCAL W		hist_l[SPEED_AVG_TICKS];
LOCAL W		hist_r[SPEED_AVG_TICKS];
LOCAL UB	hist_i;
LOCAL W		v_l, v_r;		/* measured mm/s                  */
LOCAL UW	path_um;
LOCAL UB	enc_window;
LOCAL W		enc_sum_l, enc_sum_r;

/* Cyclic handler: interrupt context, only wakes the task */
EXPORT void motion_cyclic_handler(void *exinf)
{
	(void)exinf;
	tk_wup_tsk(tid_motion);
}

LOCAL void send_status(MOTION_EVENT ev)
{
	MOTION_STATUS	st = {0};

	st.event = (UB)ev;
	st.seq = mc.seq;
	st.mode = (UB)mc.mode;
	st.speed_mm_s = (H)((v_l + v_r) / 2);
	st.path_mm = motion_odo_path_01mm / 10U;
	if (tk_snd_mbf(mbf_motion_status, &st, sizeof(st), TMO_POL) < E_OK) {
		app_log((UB*)"motion: status queue full (ev %d)\n", ev);
	}
	(void)tk_set_flg(flg_sys, EVF_MOTION);
}

LOCAL void set_mode(MOTION_MODE m)
{
	if (m != mc.mode) {
		/* VA2: clear the integrators whenever the mode changes */
		motion_pid_reset(&pid_speed);
		motion_pid_reset(&pid_line);
		motion_pid_reset(&pid_sync);
		motion_stall_reset(&stall[0]);
		motion_stall_reset(&stall[1]);
		mc.v_set = 0;
		mc.timer = 0;
		mc.settle = 0;
		mc.fwd = 0;
		mc.steer = 0;
		mc.mode = m;
		motion_mode_word = (UW)m;
	}
}

LOCAL void lock_motors(MOTION_EVENT ev, UINT fault)
{
	set_mode(MM_LOCKED);
	mc.duty_l = 0;
	mc.duty_r = 0;
	drv_motor_brake_all();
	app_report_fault(fault);
	send_status(ev);
	app_log((UB*)"motion: LOCKED (event %d)\n", ev);
}

/* ---------------------------------------------------------------------
 *  1. Measurement
 * --------------------------------------------------------------------- */
LOCAL void measure(void)
{
	W	raw_l = drv_encoder_read(DRV_ENC_LEFT);
	W	raw_r = drv_encoder_read(DRV_ENC_RIGHT);
	W	new_l = MOTION_ENC_L_INVERT ? -raw_l : raw_l;
	W	new_r = MOTION_ENC_R_INVERT ? -raw_r : raw_r;
	W	d_l = new_l - cnt_l;
	W	d_r = new_r - cnt_r;
	W	sum_l = 0;
	W	sum_r = 0;
	UINT	i;

	cnt_l = new_l;
	cnt_r = new_r;

	/* Few counts arrive per 20 ms at low speed: average 3 ticks */
	hist_l[hist_i] = d_l;
	hist_r[hist_i] = d_r;
	hist_i = (UB)((hist_i + 1U) % SPEED_AVG_TICKS);
	for (i = 0; i < SPEED_AVG_TICKS; i++) {
		sum_l += hist_l[i];
		sum_r += hist_r[i];
	}
	/* counts * µm / ms = mm/s */
	v_l = (sum_l * MOTION_UM_PER_COUNT) / (SPEED_AVG_TICKS * S2_PERIOD_MS);
	v_r = (sum_r * MOTION_UM_PER_COUNT) / (SPEED_AVG_TICKS * S2_PERIOD_MS);

	path_um += (UW)(((APP_ABS(d_l) + APP_ABS(d_r)) * MOTION_UM_PER_COUNT) / 2);

	/* Publish: each a single aligned 32-bit store (10.4 rule 1) */
	motion_enc_count[0] = cnt_l;
	motion_enc_count[1] = cnt_r;
	motion_odo_speed = motion_pack_speed(v_l, v_r);
	motion_odo_path_01mm = path_um / 100U;

	/* Window for the runtime encoder rule (5.1 encoder fault) */
	if ((APP_ABS(mc.duty_l) >= ENC_CHECK_DUTY) &&
	    (APP_ABS(mc.duty_r) >= ENC_CHECK_DUTY)) {
		enc_window++;
		enc_sum_l += APP_ABS(d_l);
		enc_sum_r += APP_ABS(d_r);
	} else {
		enc_window = 0;
		enc_sum_l = 0;
		enc_sum_r = 0;
	}
}

/* ---------------------------------------------------------------------
 *  2. Commands
 * --------------------------------------------------------------------- */
LOCAL void start_measured(W counts, W dir, W speed)
{
	mc.target_counts = counts;
	mc.dir = dir;
	mc.cruise = speed;
	mc.start_l = cnt_l;
	mc.start_r = cnt_r;
}

LOCAL void handle_cmd(const MOTION_CMD *cmd)
{
	W	speed = (cmd->speed_mm_s != 0U) ? (W)cmd->speed_mm_s : 0;

	speed = APP_CLAMP(speed, 0, MOTION_MAX_MM_S);

	/* Commands that are accepted in every mode */
	switch (cmd->op) {
	case MC_SPEED_LIMIT:
		mc.limit = speed;
		return;
	case MC_LOCK:
		mc.seq = cmd->seq;
		set_mode(MM_LOCKED);
		return;
	case MC_RESET:
		mc.seq = cmd->seq;
		set_mode(MM_IDLE);
		mc.limit = 0;
		return;
	case MC_STOP:
		mc.seq = cmd->seq;
		if (mc.mode != MM_LOCKED) {
			set_mode(MM_STOPPING);
		}
		return;
	default:
		break;
	}

	if (mc.mode == MM_LOCKED) {
		app_log((UB*)"motion: locked, command %d ignored\n", cmd->op);
		return;
	}

	mc.seq = cmd->seq;
	switch (cmd->op) {
	case MC_FOLLOW_LINE:
		set_mode(MM_FOLLOW);
		mc.straight = FALSE;
		mc.cruise = (speed != 0) ? speed : MOTION_CRUISE_MM_S;
		break;
	case MC_MOVE:
		set_mode(MM_IDLE);	/* force a clean restart of the PIDs */
		set_mode(MM_MOVE);
		start_measured(motion_mm_to_counts(APP_ABS(cmd->arg)),
			       (cmd->arg >= 0) ? 1 : -1,
			       (speed != 0) ? speed : MOTION_MOVE_MM_S);
		break;
	case MC_TURN:
		set_mode(MM_IDLE);
		set_mode(MM_TURN);
		start_measured(motion_turn_counts(cmd->arg),
			       (cmd->arg >= 0) ? 1 : -1,
			       (speed != 0) ? speed : MOTION_TURN_MM_S);
		break;
	case MC_SELFTEST:
		set_mode(MM_SELFTEST);
		mc.st_step = 0;
		mc.st_ok = TRUE;
		mc.st_ref = cnt_l;
		break;
	default:
		app_log((UB*)"motion: unknown command %d\n", cmd->op);
		break;
	}
}

/* ---------------------------------------------------------------------
 *  3. Modes
 * --------------------------------------------------------------------- */
LOCAL W limited(W v)
{
	return ((mc.limit > 0) && (v > mc.limit)) ? mc.limit : v;
}

LOCAL INT feedforward(W v)
{
	if (v == 0) {
		return 0;
	}
	return (INT)(FF_OFFSET_PERMILLE + ((v * FF_PERMILLE_PER_M_S) / 1000));
}

/* Common-mode duty for a magnitude setpoint and measured magnitude */
LOCAL INT common_duty(W v_meas)
{
	return feedforward(mc.v_set) +
	       (INT)motion_pid_step(&pid_speed, mc.v_set - v_meas);
}

/* Move cur toward tgt by at most step */
LOCAL INT step_toward(INT cur, INT tgt, INT step)
{
	if (tgt > (cur + step)) {
		return cur + step;
	}
	if (tgt < (cur - step)) {
		return cur - step;
	}
	return tgt;
}

/* Gradual steering: forward and steering parts slewed, then the duties */
LOCAL void follow_output(INT fwd_target, INT steer_target)
{
	mc.fwd = step_toward(mc.fwd, fwd_target, STEER_SLEW_PER_TICK);
	mc.steer = step_toward(mc.steer, steer_target, STEER_SLEW_PER_TICK);
	mc.duty_l = mc.fwd - mc.steer;
	mc.duty_r = mc.fwd + mc.steer;
}

LOCAL void run_follow(void)
{
	UW	w = line_latest;
	UW	now = app_time_ms();
	INT	diff;
	INT	common;

	if (((line_unpack_flags(w) & LINE_F_VALID) != 0U) &&
	    (line_unpack_age_ms(w, now) <= LINE_STALE_MS)) {
		mc.line_err = line_unpack_err(w);
	}	/* else hold the last value (F6: never wait for line_task) */

	/* Line lost (line_task sends the full error): spin on the spot
	   toward the side the line was last seen, so the car turns back
	   at once instead of driving a wide circle (28 Oct) */
	if (APP_ABS(mc.line_err) >= LINE_ERR_MAX) {
		INT	dir = (mc.line_err > 0) ? 1 : -1;	/* +1 = left */

		/* jump straight to spin speed: react at once */
		mc.straight = FALSE;
		mc.v_set = limited(LOST_SPIN_MM_S);
		common = common_duty((APP_ABS(v_l) + APP_ABS(v_r)) / 2);
		/* spin = no forward part, all steering: reached gradually */
		follow_output(0, dir * common);
		return;
	}

	/* Slow down in corners: full line error = half cruise speed */
	mc.v_set = motion_ramp(mc.v_set,
			       (limited(mc.cruise) * (2000 - APP_ABS(mc.line_err))) / 2000,
			       ACCEL_MM_S_PER_TICK);
	/* Speed loop on the FASTER wheel: with the average, a slow inner
	   wheel made the outer wheel run at up to twice the setpoint. */
	common = common_duty((v_l > v_r) ? v_l : v_r);
	/* err > 0: line to the left -> right wheel faster */
	diff = (INT)motion_pid_step(&pid_line, mc.line_err);

	/* Straight-line wheel sync (see FOLLOW_SYNC_LOG_MS above) */
	{
		INT	sync;

		if (mc.line_err == 0) {
			W	e;

			if (!mc.straight) {
				mc.straight = TRUE;	/* new straight: measure from here */
				mc.base_l = cnt_l;
				mc.base_r = cnt_r;
			}
			/* left ahead of right -> e > 0 -> slow left, speed up right */
			e = (cnt_l - mc.base_l) -
			    (((cnt_r - mc.base_r) * MOTION_SYNC_R_X1000) / 1000);
			e = (e > SYNC_DEADBAND) ? (e - SYNC_DEADBAND) :
			    (e < -SYNC_DEADBAND) ? (e + SYNC_DEADBAND) : 0;
			sync = (INT)motion_pid_step(&pid_sync, e);
		} else {
			/* steering: keep only the learned motor trim (integral) */
			mc.straight = FALSE;
			sync = (INT)((pid_sync.ki_q8 * pid_sync.i_acc) / 256);
			sync = APP_CLAMP(sync, -(INT)pid_sync.out_limit, (INT)pid_sync.out_limit);
		}
		diff += sync;
#if FOLLOW_SYNC_LOG_MS > 0
		if ((app_time_ms() - mc.log_ms) >= FOLLOW_SYNC_LOG_MS) {
			mc.log_ms = app_time_ms();
			app_log((UB*)"motion: follow %s  speed L %d R %d mm/s  counts L %d R %d  trim %d\n",
				mc.straight ? "straight" : "steering", v_l, v_r,
				cnt_l - mc.base_l, cnt_r - mc.base_r, sync);
		}
#endif
	}
	/* Steering slows the inner wheel to a quarter at most: the car
	   arcs back to the line instead of pivoting on one wheel */
	if (common > 0) {
		diff = APP_CLAMP(diff, -(common * 3) / 4, (common * 3) / 4);
	}

	follow_output(common, diff);
}

/* MOVE and TURN share the same structure.  TURN: dir +1 = left (CCW):
   left wheel backward, right wheel forward. */
LOCAL void run_measured(BOOL turning)
{
	W	prog_l = (cnt_l - mc.start_l) * (turning ? -mc.dir : mc.dir);
	W	prog_r = (cnt_r - mc.start_r) * mc.dir;
	W	prog = (prog_l + prog_r) / 2;
	W	remaining = mc.target_counts - prog;
	W	v_meas;
	INT	common;
	INT	diff;

	if (remaining <= 0) {
		mc.duty_l = 0;
		mc.duty_r = 0;
		drv_motor_brake_all();
		set_mode(MM_IDLE);
		send_status(MS_DONE);
		return;
	}

	mc.v_set = motion_ramp(mc.v_set,
			       motion_decel_speed(limited(mc.cruise), remaining),
			       ACCEL_MM_S_PER_TICK);
	if (turning) {
		v_meas = ((v_r - v_l) / 2) * mc.dir;
	} else {
		v_meas = ((v_l + v_r) / 2) * mc.dir;
	}
	common = common_duty(v_meas);
	/* Left ahead of right -> slow left, speed up right */
	{
		W	e = prog_l - prog_r;

		/* ignore encoder jitter under ~1 mm (31 Oct) */
		e = (e > SYNC_DEADBAND) ? (e - SYNC_DEADBAND) :
		    (e < -SYNC_DEADBAND) ? (e + SYNC_DEADBAND) : 0;
		diff = (INT)motion_pid_step(&pid_sync, e);
	}
	if (!turning && (mc.dir < 0)) {
		/* 31 Oct: straight reverse = same power on both wheels, no
		   left/right sync.  The strong sync made the car wiggle side
		   to side while backing up (reverses are short, ~10 cm). */
		diff = 0;
	}

	if (turning) {
		mc.duty_l = -(INT)mc.dir * (common - diff);
	} else {
		mc.duty_l = (INT)mc.dir * (common - diff);
	}
	mc.duty_r = (INT)mc.dir * (common + diff);
}

LOCAL void run_stopping(void)
{
	mc.duty_l = 0;
	mc.duty_r = 0;
	if ((APP_ABS(v_l) < STOP_SPEED_MM_S) && (APP_ABS(v_r) < STOP_SPEED_MM_S)) {
		mc.settle++;
	} else {
		mc.settle = 0;
	}
	mc.timer++;
	if ((mc.settle >= 2U) || (mc.timer >= STOP_TIMEOUT_TICKS)) {
		set_mode(MM_IDLE);
		send_status(MS_STOPPED);
	}
}

/* INITIALISING self-check: drive each wheel alone, expect forward counts */
LOCAL void run_selftest(void)
{
	mc.duty_l = 0;
	mc.duty_r = 0;
	mc.timer++;

	switch (mc.st_step) {
	case 0:			/* left wheel forward */
		mc.duty_l = SELFTEST_DUTY;
		if (mc.timer >= SELFTEST_TICKS) {
			if ((cnt_l - mc.st_ref) < SELFTEST_MIN_COUNTS) {
				mc.st_ok = FALSE;
				app_log((UB*)"motion: selftest LEFT %d counts\n",
					  cnt_l - mc.st_ref);
			}
			mc.st_step = 1;
			mc.timer = 0;
		}
		break;
	case 1:			/* brake, then reference the right wheel */
		if (mc.timer >= SELFTEST_TICKS) {
			mc.st_ref = cnt_r;
			mc.st_step = 2;
			mc.timer = 0;
		}
		break;
	case 2:			/* right wheel forward */
		mc.duty_r = SELFTEST_DUTY;
		if (mc.timer >= SELFTEST_TICKS) {
			if ((cnt_r - mc.st_ref) < SELFTEST_MIN_COUNTS) {
				mc.st_ok = FALSE;
				app_log((UB*)"motion: selftest RIGHT %d counts\n",
					  cnt_r - mc.st_ref);
			}
			mc.st_step = 3;
			mc.timer = 0;
		}
		break;
	default:		/* brake and report */
		if (mc.timer >= SELFTEST_TICKS) {
			set_mode(MM_IDLE);
			if (mc.st_ok && motion_encoders_ok) {
				send_status(MS_SELFTEST_PASS);
			} else {
				app_report_fault(FLT_ENCODER);
				send_status(MS_SELFTEST_FAIL);
			}
		}
		break;
	}
}

/* ---------------------------------------------------------------------
 *  4. Fault rules
 * --------------------------------------------------------------------- */
LOCAL BOOL check_faults(void)
{
	BOOL	stall_l;
	BOOL	stall_r;
	/* 3/4 of the cruise feed-forward: a blocked wheel on a straight is
	   driven at about cruise, the inside wheel of a sharp correction
	   well below it */
	INT	drive_floor = (feedforward(mc.v_set) * 3) / 4;

	if ((mc.mode != MM_FOLLOW) && (mc.mode != MM_MOVE) && (mc.mode != MM_TURN)) {
		return FALSE;
	}
	/* Rule applied to each wheel against the common setpoint: with one
	   wheel blocked the AVERAGE speed never falls below 20 %. */
	/* A wheel only counts as commanded while it is driven at least as
	   hard as (3/4 of) cruise: the inside wheel of a line correction or a slow
	   hump crossing is meant to be slow and must not trip the rule. */
	stall_l = motion_stall_update(&stall[0],
		(APP_ABS(mc.duty_l) >= drive_floor) ? mc.v_set : 0, v_l);
	stall_r = motion_stall_update(&stall[1],
		(APP_ABS(mc.duty_r) >= drive_floor) ? mc.v_set : 0, v_r);
	if (stall_l || stall_r) {
		app_log((UB*)"motion: STALL %s wheel: set %d mm/s, measured L %d R %d mm/s, duty L %d R %d\n",
			(stall_l && stall_r) ? "BOTH" : (stall_l ? "LEFT" : "RIGHT"),
			mc.v_set, v_l, v_r, mc.duty_l, mc.duty_r);
		lock_motors(MS_STALL, FLT_STALL);
		return TRUE;
	}
	if (enc_window >= ENC_CHECK_TICKS) {
		if (((enc_sum_l == 0) && (enc_sum_r >= ENC_CHECK_MIN_COUNTS)) ||
		    ((enc_sum_r == 0) && (enc_sum_l >= ENC_CHECK_MIN_COUNTS))) {
			app_log((UB*)"motion: ENCODER %s silent: counts in 1 s L %d R %d\n",
				(enc_sum_l == 0) ? "LEFT" : "RIGHT", enc_sum_l, enc_sum_r);
			lock_motors(MS_ENCODER_FAULT, FLT_ENCODER);
			return TRUE;
		}
		enc_window = 0;
		enc_sum_l = 0;
		enc_sum_r = 0;
	}
	return FALSE;
}

/* ---------------------------------------------------------------------
 *  5. Output with soft start (rise limited; braking is immediate)
 * --------------------------------------------------------------------- */
LOCAL INT slew(INT prev, INT target)
{
	if (target == 0) {
		return 0;			/* braking: at once */
	}
	if (((target > 0) && (prev < 0)) || ((target < 0) && (prev > 0))) {
		return 0;			/* reversing: via zero */
	}
	if (APP_ABS(target) <= APP_ABS(prev)) {
		return target;			/* falling: at once */
	}
	return (INT)motion_ramp(prev, target, SLEW_PERMILLE_PER_TICK);
}

LOCAL INT	out_l, out_r;

LOCAL void apply_output(void)
{
	if ((mc.mode == MM_IDLE) || (mc.mode == MM_LOCKED) ||
	    (mc.mode == MM_STOPPING)) {
		out_l = 0;
		out_r = 0;
		drv_motor_brake_all();	/* brake, not coast: stays on the line */
		return;
	}
	out_l = slew(out_l, APP_CLAMP(mc.duty_l, -MOTOR_DUTY_MAX, MOTOR_DUTY_MAX));
	out_r = slew(out_r, APP_CLAMP(mc.duty_r, -MOTOR_DUTY_MAX, MOTOR_DUTY_MAX));
	drv_motor_set(MOTOR_LEFT, out_l);
	drv_motor_set(MOTOR_RIGHT, out_r);
}

/* ---------------------------------------------------------------------
 *  Bring-up diagnostic (MOTION_ENC_DIAG, subsystem2.h): while the car is
 *  stopped, print the raw encoder pin levels and counts whenever they
 *  change, so turning a wheel by hand shows whether A and B toggle.
 * --------------------------------------------------------------------- */
LOCAL void enc_diag(void)
{
#if MOTION_ENC_DIAG
	static UW	last_pins = 0xFFFFFFFFUL;
	static W	last_l, last_r;
	static UB	div;
	UW		pins;

	if ((mc.mode != MM_IDLE) && (mc.mode != MM_LOCKED)) {
		return;
	}
	if (++div < 5U) {		/* every 100 ms */
		return;
	}
	div = 0;
	/* GPIO_IN shows the pad level even while the PIO owns the pin */
	pins = (gpio_get_val(2) << 3) | (gpio_get_val(3) << 2) |
	       (gpio_get_val(4) << 1) | gpio_get_val(5);
	if ((pins != last_pins) || (cnt_l != last_l) || (cnt_r != last_r)) {
		app_log((UB*)"enc: L A=%d B=%d  R A=%d B=%d   counts L %d  R %d\n",
			  (pins >> 3) & 1U, (pins >> 2) & 1U, (pins >> 1) & 1U,
			  pins & 1U, cnt_l, cnt_r);
		last_pins = pins;
		last_l = cnt_l;
		last_r = cnt_r;
	}
#endif
}

/* ---------------------------------------------------------------------
 *  The task
 * --------------------------------------------------------------------- */
EXPORT void motion_task(INT stacd, void *exinf)
{
	MOTION_CMD	cmd;
	UW		prev_us = 0;
	UW		now_us;
	UW		jitter;
	BOOL		first = TRUE;

	(void)stacd; (void)exinf;

	/* Speed PI: error mm/s -> duty permille */
	motion_pid_init(&pid_speed, 256, 26, 0, 3000, 400);
	/* Line PD: error -1000..1000 -> differential duty permille */
	motion_pid_init(&pid_line, 160, 0, 64, 0, 500);	/* Kp 0.62, Kd 0.25 (28 Oct: less zig-zag on sensor steps) */
	/* Count sync PI: count difference -> differential duty */
	/* 31 Oct: ~13.5 counts per mm.  The old gains (15 duty per count,
	   limit 200) maxed out on a 1 mm difference and made the car shake
	   side to side on straight moves.  Now ~20 duty per mm, a slow
	   integral for steady drift, and a +-1 mm dead zone (SYNC_DEADBAND). */
	motion_pid_init(&pid_sync, 384, 2, 0, 6000, 150);

	cnt_l = MOTION_ENC_L_INVERT ? -drv_encoder_read(DRV_ENC_LEFT)
				    : drv_encoder_read(DRV_ENC_LEFT);
	cnt_r = MOTION_ENC_R_INVERT ? -drv_encoder_read(DRV_ENC_RIGHT)
				    : drv_encoder_read(DRV_ENC_RIGHT);
	mc.mode = MM_IDLE;
	motion_mode_word = MM_IDLE;
	drv_motor_brake_all();

	while (1) {
		tk_slp_tsk(TMO_FEVR);
		(void)tk_can_wup(TSK_SELF);	/* no catch-up bursts */

		/* Release jitter (NFR3: <= 2 ms while telemetry runs) */
		now_us = app_time_us();
		if (!first) {
			W d = (W)(now_us - prev_us) - (S2_PERIOD_MS * 1000);
			jitter = (UW)APP_ABS(d);
			if (jitter > motion_jitter_max_us) {
				motion_jitter_max_us = jitter;
			}
		}
		first = FALSE;
		prev_us = now_us;

		measure();
		enc_diag();

		while (tk_rcv_mbf(mbf_motion_cmd, &cmd, TMO_POL) == (INT)sizeof(cmd)) {
			handle_cmd(&cmd);
		}

		switch (mc.mode) {
		case MM_FOLLOW:		run_follow();			break;
		case MM_MOVE:		run_measured(FALSE);		break;
		case MM_TURN:		run_measured(TRUE);		break;
		case MM_STOPPING:	run_stopping();			break;
		case MM_SELFTEST:	run_selftest();			break;
		default:		mc.duty_l = 0; mc.duty_r = 0;	break;
		}

		if (!check_faults()) {
			if (mc.mode == MM_SELFTEST) {
				/* raw duties, no ramp, one wheel at a time */
				drv_motor_set(MOTOR_LEFT, mc.duty_l);
				drv_motor_set(MOTOR_RIGHT, mc.duty_r);
			} else {
				apply_output();
			}
		}
	}
}
