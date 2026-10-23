/*
 *	task_motion.c
 *	Subsystem 2 -- motion_task (COMPLETE VERSION), woken every 20 ms.
 *
 *	Each cycle:
 *	  1. take new commands from the Motion Command Queue (STOP pre-empts);
 *	  2. read the quadrature counts: signed speed (60 ms window), odometer;
 *	  3. set each wheel's target speed for the mode, then ramp it
 *	     (soft start, S2_ACCEL_MM_S2);
 *	  4. per-wheel PI speed loop -> motor PWM;
 *	  5. stall and encoder checks.
 *	Statechart 6.2: IDLE, FOLLOWING LINE, MOVING/TURNING, STOPPING,
 *	MOTOR FAULT.
 */

#include "subsystem2.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "../subsystem3/subsystem3.h"	/* line error slot */
#include "../subsystem5/subsystem5.h"	/* Motion Status Queue, faults */

typedef enum {
	MM_IDLE = 0,
	MM_FOLLOW,
	MM_MOVE,
	MM_TURN,
	MM_FAULT
} MOTION_MODE;

typedef struct {
	W	target;			/* mm/s wanted by the mode */
	W	ramped;			/* after soft start: what the PI uses */
	W	integ;
	W	last_count;
	W	window[S2_SPEED_WINDOW];/* signed counts per cycle */
	UINT	filled;
	W	speed;			/* measured mm/s, signed */
	W	prev_speed;
	UW	stall_ms;
	UW	silent_ms;		/* driven but no counts at all */
	W	last_sign;		/* fallback build: commanded direction */
} WHEEL;

EXPORT volatile W	s2_distance_mm = 0;
EXPORT volatile W	s2_speed_l_mm_s = 0;
EXPORT volatile W	s2_speed_r_mm_s = 0;
EXPORT volatile W	s2_enc_l = 0;
EXPORT volatile W	s2_enc_r = 0;
EXPORT volatile W	s2_accel_mm_s2 = 0;
EXPORT volatile BOOL	s2_hw_ready = FALSE;
EXPORT volatile BOOL	s2_faulted = FALSE;

LOCAL WHEEL		wheel[2];
LOCAL MOTION_MODE	mode = MM_IDLE;
LOCAL W			follow_speed = 0;
LOCAL W			speed_limit = 0;
LOCAL W			move_speed = 0;
LOCAL UW		move_target = 0U;	/* counts */
LOCAL UW		move_done = 0U;
LOCAL W			move_dir_l = 0;
LOCAL W			move_dir_r = 0;
LOCAL BOOL		move_until_line = FALSE;
LOCAL W			prev_line_err = 0;
LOCAL UW		odo_um = 0U;
LOCAL BOOL		signed_enc = FALSE;

#define S2_RAMP_STEP	((S2_ACCEL_MM_S2 * S2_PERIOD_MS) / 1000)	/* mm/s per cycle */

/* ------------------------------------------------------------------ */
LOCAL W s2_abs(W v)
{
	return (v < 0) ? -v : v;
}

LOCAL W s2_cap(W v)
{
	if (speed_limit > 0) {
		if (v > speed_limit) {
			v = speed_limit;
		} else if (v < -speed_limit) {
			v = -speed_limit;
		} else {
			;
		}
	}
	return v;
}

LOCAL void s2_wheel_reset(WHEEL *p_w, BOOL keep_ramp)
{
	UINT	i;

	p_w->target = 0;
	if (keep_ramp == FALSE) {
		p_w->ramped = 0;
	}
	p_w->integ = 0;			/* integrator reset on every mode change */
	p_w->filled = 0U;
	p_w->stall_ms = 0U;
	p_w->silent_ms = 0U;
	for (i = 0U; i < (UINT)S2_SPEED_WINDOW; i++) {
		p_w->window[i] = 0;
	}
}

LOCAL void s2_post_status(MOTION_STATUS_KIND kind, BOOL line_found)
{
	MOTION_STATUS_MSG	msg;

	msg.kind = kind;
	msg.distance_mm = s2_distance_mm;
	msg.line_found = line_found;
	(void)s5_post(mbf_motion_st, &msg, (INT)sizeof(msg), FLG_MOTION);
}

LOCAL UW s2_counts_for_mm(W mm)
{
	return ((UW)s2_abs(mm) * 1000U) / (UW)S2_UM_PER_EDGE;
}

LOCAL UW s2_counts_for_deg(W deg)
{
	UW	arc_um = ((UW)s2_abs(deg) * (UW)S2_TRACK_MM * 355U * 1000U) / (360U * 113U);

	return arc_um / (UW)S2_UM_PER_EDGE;
}

LOCAL BOOL s2_line_seen(void)
{
	W	err;
	UW	age;
	UW	flags;

	s3_line_get(&err, &age, &flags);
	return ((flags & (S3_LF_L_BLACK | S3_LF_C_BLACK)) != 0U) ? TRUE : FALSE;
}

/* ------------------------------------------------------------------ */
LOCAL void s2_start_move(MOTION_MODE m, W value, BOOL until_line)
{
	s2_wheel_reset(&wheel[S2_LEFT], TRUE);
	s2_wheel_reset(&wheel[S2_RIGHT], TRUE);
	mode = m;
	move_until_line = until_line;
	move_done = 0U;
	if (m == MM_MOVE) {
		move_speed = S2_MOVE_MM_S;
		move_target = s2_counts_for_mm(value);
		move_dir_l = (value >= 0) ? 1 : -1;
		move_dir_r = move_dir_l;
	} else {
		move_speed = S2_TURN_MM_S;
		move_target = s2_counts_for_deg(value);
		move_dir_l = (value >= 0) ? -1 : 1;	/* + = left turn */
		move_dir_r = -move_dir_l;
	}
	if (move_target == 0U) {
		mode = MM_IDLE;
		s2_post_status(MST_DONE, FALSE);
	}
}

LOCAL void s2_handle_cmd(const MOTION_CMD *p_cmd)
{
	if (mode == MM_FAULT) {
		if (p_cmd->kind == MC_RESET_FAULT) {
			mode = MM_IDLE;
			s2_faulted = FALSE;
			s2_wheel_reset(&wheel[S2_LEFT], FALSE);
			s2_wheel_reset(&wheel[S2_RIGHT], FALSE);
		}
		return;
	}
	switch (p_cmd->kind) {
	case MC_STOP:
		mode = MM_IDLE;
		break;
	case MC_FOLLOW:
		if (mode != MM_FOLLOW) {
			s2_wheel_reset(&wheel[S2_LEFT], TRUE);
			s2_wheel_reset(&wheel[S2_RIGHT], TRUE);
			prev_line_err = 0;
		}
		follow_speed = (p_cmd->speed_mm_s > 0) ? p_cmd->speed_mm_s : S2_MOVE_MM_S;
		mode = MM_FOLLOW;
		break;
	case MC_MOVE:
		s2_start_move(MM_MOVE, p_cmd->value, FALSE);
		break;
	case MC_TURN:
		s2_start_move(MM_TURN, p_cmd->value, FALSE);
		break;
	case MC_MOVE_UNTIL_LINE:
		s2_start_move(MM_MOVE, p_cmd->value, TRUE);
		break;
	case MC_TURN_UNTIL_LINE:
		s2_start_move(MM_TURN, p_cmd->value, TRUE);
		break;
	case MC_SPEED_LIMIT:
		speed_limit = (p_cmd->value > 0) ? p_cmd->value : 0;
		break;
	case MC_RESET_FAULT:
	default:
		break;
	}
}

/* Returns the signed count change this cycle. */
LOCAL W s2_update_wheel(S2_SIDE side)
{
	WHEEL	*p_w = &wheel[side];
	W	now = encoder_count(side);
	W	d = now - p_w->last_count;
	UINT	i;
	W	sum = 0;

	p_w->last_count = now;
	if (signed_enc == FALSE) {
		/* fallback counter has no direction: use the commanded one */
		if (p_w->ramped < 0) {
			p_w->last_sign = -1;
		} else if (p_w->ramped > 0) {
			p_w->last_sign = 1;
		} else {
			;
		}
		d *= p_w->last_sign;
	}
	for (i = 0U; (i + 1U) < (UINT)S2_SPEED_WINDOW; i++) {
		p_w->window[i] = p_w->window[i + 1U];
	}
	p_w->window[S2_SPEED_WINDOW - 1] = d;
	if (p_w->filled < (UINT)S2_SPEED_WINDOW) {
		p_w->filled++;
	}
	for (i = (UINT)S2_SPEED_WINDOW - p_w->filled; i < (UINT)S2_SPEED_WINDOW; i++) {
		sum += p_w->window[i];
	}
	p_w->prev_speed = p_w->speed;
	p_w->speed = (sum * S2_UM_PER_EDGE) / ((W)p_w->filled * S2_PERIOD_MS);
	return d;
}

/* Soft start: limits SPEEDING UP only.  Slowing down, stopping and
 * reversing take effect at once, so STOP is never delayed (NFR2). */
LOCAL void s2_ramp(WHEEL *p_w)
{
	W	t = p_w->target;
	W	r = p_w->ramped;

	if ((t == 0) || ((t > 0) && (r < 0)) || ((t < 0) && (r > 0))) {
		p_w->ramped = (t == 0) ? 0 : ((t > 0) ? S2_RAMP_STEP : -S2_RAMP_STEP);
	} else if (s2_abs(t) <= s2_abs(r)) {
		p_w->ramped = t;			/* slowing down: immediate */
	} else if (t > r) {
		p_w->ramped = ((t - r) > S2_RAMP_STEP) ? (r + S2_RAMP_STEP) : t;
	} else {
		p_w->ramped = ((r - t) > S2_RAMP_STEP) ? (r - S2_RAMP_STEP) : t;
	}
}

LOCAL INT s2_pi(WHEEL *p_w)
{
	W	err;
	W	duty;

	if (p_w->ramped == 0) {
		p_w->integ = 0;
		return 0;
	}
	err = p_w->ramped - p_w->speed;
	p_w->integ += err;
	if (p_w->integ > S2_INTEG_LIMIT) {
		p_w->integ = S2_INTEG_LIMIT;
	} else if (p_w->integ < -S2_INTEG_LIMIT) {
		p_w->integ = -S2_INTEG_LIMIT;
	} else {
		;
	}
	duty = ((p_w->ramped * S2_FF_PERMILLE_X1000) / 1000) +
	       ((err * S2_KP_X1000) / 1000) +
	       ((p_w->integ * S2_KI_X1000) / 1000);
	if ((p_w->ramped > 0) && (duty < 0)) {
		duty = 0;
	} else if ((p_w->ramped < 0) && (duty > 0)) {
		duty = 0;
	} else {
		;
	}
	if (duty > 1000) {
		duty = 1000;
	} else if (duty < -1000) {
		duty = -1000;
	} else {
		;
	}
	return (INT)duty;
}

/* Stall rule (Section 5.1): commanded but < 20 % for 300 ms.  Uses the
 * ramped target, so soft start never looks like a stall. */
LOCAL BOOL s2_check_stall(WHEEL *p_w)
{
	W	t = s2_abs(p_w->ramped);

	if (t < S2_RAMP_STEP) {
		p_w->stall_ms = 0U;
		return FALSE;
	}
	if ((s2_abs(p_w->speed) * 100) < (t * S2_STALL_PERCENT)) {
		p_w->stall_ms += (UW)S2_PERIOD_MS;
	} else {
		p_w->stall_ms = 0U;
	}
	return (p_w->stall_ms >= (UW)S2_STALL_MS) ? TRUE : FALSE;
}

/* Encoder fault: this wheel is driven and shows no counts at all while
 * the other wheel does count (so the car is moving). */
LOCAL BOOL s2_check_encoder(WHEEL *p_w, W d_self, W d_other)
{
	if ((s2_abs(p_w->ramped) >= S2_RAMP_STEP) && (d_self == 0) && (d_other != 0)) {
		p_w->silent_ms += (UW)S2_PERIOD_MS;
	} else {
		p_w->silent_ms = 0U;
	}
	return (p_w->silent_ms >= (UW)S2_ENC_FAULT_MS) ? TRUE : FALSE;
}

LOCAL void s2_set_targets(void)
{
	switch (mode) {
	case MM_FOLLOW: {
		W	err;
		UW	age_ms;
		UW	flags;
		W	base = s2_cap(follow_speed);
		W	dv;
		INT	i;

		s3_line_get(&err, &age_ms, &flags);
		if (age_ms > (UW)S2_LINE_STALE_MS) {
			base /= 2;
		}
		dv = ((err * S2_LINE_KP_X1000) + ((err - prev_line_err) * S2_LINE_KD_X1000)) / 1000;
		prev_line_err = err;
		wheel[S2_LEFT].target = base - dv;
		wheel[S2_RIGHT].target = base + dv;
		for (i = 0; i < 2; i++) {
			if (wheel[i].target < 0) {
				wheel[i].target = 0;
			} else if (wheel[i].target > (2 * base)) {
				wheel[i].target = 2 * base;
			} else {
				;
			}
		}
		break;
	}
	case MM_MOVE:
	case MM_TURN: {
		BOOL	found = ((move_until_line == TRUE) && (s2_line_seen() == TRUE)) ? TRUE : FALSE;

		if ((move_done >= move_target) || (found == TRUE)) {
			mode = MM_IDLE;
			wheel[S2_LEFT].target = 0;
			wheel[S2_RIGHT].target = 0;
			s2_post_status(MST_DONE, found);
		} else {
			W	v = s2_cap(move_speed);
			wheel[S2_LEFT].target = v * move_dir_l;
			wheel[S2_RIGHT].target = v * move_dir_r;
		}
		break;
	}
	case MM_IDLE:
	case MM_FAULT:
	default:
		wheel[S2_LEFT].target = 0;
		wheel[S2_RIGHT].target = 0;
		break;
	}
}

LOCAL void s2_enter_fault(UW fault_bit, MOTION_STATUS_KIND kind, const char *why)
{
	motor_stop_all();
	mode = MM_FAULT;
	s2_faulted = TRUE;
	s2_wheel_reset(&wheel[S2_LEFT], FALSE);
	s2_wheel_reset(&wheel[S2_RIGHT], FALSE);
	tm_printf((UB *)"[S2] %s: motors locked off\n", why);
	s5_raise_fault(fault_bit);
	s2_post_status(kind, FALSE);
}

EXPORT void motion_task(INT stacd, void *exinf)
{
	MOTION_CMD	cmd;
	UINT		print_div = 0U;
	ER		er;

	(void)stacd;
	(void)exinf;

	(void)motor_init();
	er = encoder_init();
	signed_enc = encoder_is_signed();
	encoder_report();
	wheel[S2_LEFT].last_count = encoder_count(S2_LEFT);
	wheel[S2_RIGHT].last_count = encoder_count(S2_RIGHT);
	wheel[S2_LEFT].last_sign = 1;
	wheel[S2_RIGHT].last_sign = 1;
	if (er < E_OK) {
		tm_printf((UB *)"[S2] encoder PIO program could not be loaded (%d)\n", (INT)er);
		s2_enter_fault(FAULT_ENCODER, MST_ENCODER_FAULT, "ENCODER FAULT");
	}
	s2_hw_ready = TRUE;

	while (1) {
		W	dl;
		W	dr;
		W	al;
		W	ar;

		(void)tk_slp_tsk(TMO_FEVR);
		(void)tk_can_wup(TSK_SELF);

		while (tk_rcv_mbf(mbf_motion_cmd, &cmd, TMO_POL) > 0) {
			s2_handle_cmd(&cmd);
		}

		dl = s2_update_wheel(S2_LEFT);
		dr = s2_update_wheel(S2_RIGHT);
		s2_enc_l = wheel[S2_LEFT].last_count;
		s2_enc_r = wheel[S2_RIGHT].last_count;
		s2_speed_l_mm_s = wheel[S2_LEFT].speed;
		s2_speed_r_mm_s = wheel[S2_RIGHT].speed;
		s2_accel_mm_s2 = (((wheel[S2_LEFT].speed - wheel[S2_LEFT].prev_speed) +
				   (wheel[S2_RIGHT].speed - wheel[S2_RIGHT].prev_speed)) * 1000) /
				 (2 * S2_PERIOD_MS);

		/* odometer: path length, so it always counts up */
		al = s2_abs(dl);
		ar = s2_abs(dr);
		odo_um += (((UW)al + (UW)ar) * (UW)S2_UM_PER_EDGE) / 2U;
		s2_distance_mm += (W)(odo_um / 1000U);
		odo_um %= 1000U;
		if ((mode == MM_MOVE) || (mode == MM_TURN)) {
			move_done += ((UW)al + (UW)ar) / 2U;
		}

		s2_set_targets();
		s2_ramp(&wheel[S2_LEFT]);
		s2_ramp(&wheel[S2_RIGHT]);

		if (mode == MM_FAULT) {
			motor_stop_all();
		} else {
			motor_set(S2_LEFT, s2_pi(&wheel[S2_LEFT]));
			motor_set(S2_RIGHT, s2_pi(&wheel[S2_RIGHT]));
			if ((signed_enc == TRUE) &&
			    ((s2_check_encoder(&wheel[S2_LEFT], dl, dr) == TRUE) ||
			     (s2_check_encoder(&wheel[S2_RIGHT], dr, dl) == TRUE))) {
				s2_enter_fault(FAULT_ENCODER, MST_ENCODER_FAULT, "ENCODER FAULT");
			} else if ((s2_check_stall(&wheel[S2_LEFT]) == TRUE) ||
				   (s2_check_stall(&wheel[S2_RIGHT]) == TRUE)) {
				s2_enter_fault(FAULT_STALL, MST_STALL, "STALL");
			} else {
				;
			}
		}

		if (S2_PRINT_ENABLE != 0) {
			if (++print_div >= 25U) {
				print_div = 0U;
				tm_printf((UB *)"[S2] mode=%d tgt L=%d R=%d meas L=%d R=%d odo=%d\n",
					  (INT)mode, (INT)wheel[S2_LEFT].ramped, (INT)wheel[S2_RIGHT].ramped,
					  (INT)wheel[S2_LEFT].speed, (INT)wheel[S2_RIGHT].speed,
					  (INT)s2_distance_mm);
			}
		}
	}
}
