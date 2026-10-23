/*
 *	task_motion.c
 *	Subsystem 2 -- motion_task (BASIC VERSION), woken every 20 ms.
 *
 *	Each cycle:
 *	  1. take any new commands from the Motion Command Queue;
 *	  2. read the encoder edge counters and update speed and odometer;
 *	  3. work out each wheel's target speed for the current mode
 *	     (follow the line / move / turn / idle);
 *	  4. run the per-wheel PI speed loop and write the motor PWM;
 *	  5. check for a stall.
 *	Statechart 6.2: IDLE, FOLLOWING LINE, MOVING/TURNING, MOTOR FAULT.
 */

#include "subsystem2.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "../subsystem3/subsystem3.h"	/* line error slot */
#include "../subsystem5/subsystem5.h"	/* Motion Status Queue */

typedef enum {
	MM_IDLE = 0,
	MM_FOLLOW,
	MM_MOVE,		/* MOVE and TURN share the "target edges" logic */
	MM_TURN,
	MM_FAULT
} MOTION_MODE;

typedef struct {
	W	target;			/* mm/s, signed */
	W	integ;			/* PI integrator */
	UW	last_count;		/* encoder reading last cycle */
	UH	window[S2_SPEED_WINDOW];/* edges per cycle, newest last */
	UINT	filled;			/* valid entries in window */
	W	speed;			/* measured mm/s, signed */
	UW	stall_ms;
	W	last_sign;		/* +1 / -1: direction last driven */
} WHEEL;

EXPORT volatile W	s2_distance_mm = 0;
EXPORT volatile W	s2_speed_l_mm_s = 0;
EXPORT volatile W	s2_speed_r_mm_s = 0;
EXPORT volatile UW	s2_enc_l = 0;
EXPORT volatile UW	s2_enc_r = 0;
EXPORT volatile BOOL	s2_hw_ready = FALSE;
EXPORT volatile BOOL	s2_faulted = FALSE;

LOCAL WHEEL		wheel[2];
LOCAL MOTION_MODE	mode = MM_IDLE;
LOCAL W			follow_speed = 0;	/* mm/s */
LOCAL W			speed_limit = 0;	/* 0 = none */
LOCAL W			move_speed = 0;
LOCAL UW		move_target_edges = 0;
LOCAL UW		move_done_edges = 0;
LOCAL W			move_dir_l = 0;		/* +1 / -1 per wheel */
LOCAL W			move_dir_r = 0;
LOCAL W			prev_line_err = 0;
LOCAL UW		odo_um = 0;		/* odometer remainder, um */

/* ------------------------------------------------------------------ */
LOCAL W s2_abs(W v)
{
	return (v < 0) ? -v : v;
}

LOCAL W s2_cap(W v)
{
	W	lim = speed_limit;

	if (lim > 0) {
		if (v > lim) {
			v = lim;
		} else if (v < -lim) {
			v = -lim;
		} else {
			;
		}
	}
	return v;
}

LOCAL void s2_wheel_reset(WHEEL *p_w)
{
	UINT	i;

	p_w->target = 0;
	p_w->integ = 0;
	p_w->filled = 0U;
	p_w->stall_ms = 0U;
	for (i = 0U; i < (UINT)S2_SPEED_WINDOW; i++) {
		p_w->window[i] = 0U;
	}
}

LOCAL void s2_post_status(MOTION_STATUS_KIND kind)
{
	MOTION_STATUS_MSG	msg;

	msg.kind = kind;
	msg.distance_mm = s2_distance_mm;
	(void)s5_post(mbf_motion_st, &msg, (INT)sizeof(msg), FLG_MOTION);
}

/* Edges a wheel must turn for a straight distance or an in-place turn. */
LOCAL UW s2_edges_for_mm(W mm)
{
	return ((UW)s2_abs(mm) * 1000U) / (UW)S2_UM_PER_EDGE;
}

LOCAL UW s2_edges_for_deg(W deg)
{
	/* arc per wheel = deg/360 x pi x track.  pi ~ 355/113. */
	UW	arc_um = ((UW)s2_abs(deg) * (UW)S2_TRACK_MM * 355U * 1000U) /
			 (360U * 113U);

	return arc_um / (UW)S2_UM_PER_EDGE;
}

/* ------------------------------------------------------------------ */
LOCAL void s2_start_move(MOTION_MODE m, W value, W speed)
{
	W	v = (speed > 0) ? speed : ((m == MM_TURN) ? S2_TURN_MM_S : S2_MOVE_MM_S);

	s2_wheel_reset(&wheel[S2_LEFT]);
	s2_wheel_reset(&wheel[S2_RIGHT]);
	mode = m;
	move_speed = v;
	move_done_edges = 0U;
	if (m == MM_MOVE) {
		move_target_edges = s2_edges_for_mm(value);
		move_dir_l = (value >= 0) ? 1 : -1;
		move_dir_r = move_dir_l;
	} else {
		move_target_edges = s2_edges_for_deg(value);
		move_dir_l = (value >= 0) ? -1 : 1;	/* + = left turn */
		move_dir_r = -move_dir_l;
	}
	if (move_target_edges == 0U) {
		mode = MM_IDLE;
		s2_post_status(MST_DONE);
	}
}

LOCAL void s2_handle_cmd(const MOTION_CMD *p_cmd)
{
	if (mode == MM_FAULT) {
		if (p_cmd->kind == MC_RESET_FAULT) {
			mode = MM_IDLE;
			s2_faulted = FALSE;
			s2_wheel_reset(&wheel[S2_LEFT]);
			s2_wheel_reset(&wheel[S2_RIGHT]);
		}
		return;		/* motors stay locked off until reset */
	}

	switch (p_cmd->kind) {
	case MC_STOP:
		mode = MM_IDLE;
		break;
	case MC_FOLLOW:
		if (mode != MM_FOLLOW) {
			s2_wheel_reset(&wheel[S2_LEFT]);
			s2_wheel_reset(&wheel[S2_RIGHT]);
			prev_line_err = 0;
		}
		follow_speed = (p_cmd->speed_mm_s > 0) ? p_cmd->speed_mm_s : S2_MOVE_MM_S;
		mode = MM_FOLLOW;
		break;
	case MC_MOVE:
		s2_start_move(MM_MOVE, p_cmd->value, p_cmd->speed_mm_s);
		break;
	case MC_TURN:
		s2_start_move(MM_TURN, p_cmd->value, p_cmd->speed_mm_s);
		break;
	case MC_SPEED_LIMIT:
		speed_limit = (p_cmd->value > 0) ? p_cmd->value : 0;
		break;
	case MC_RESET_FAULT:
	default:
		break;
	}
}

/* Read one encoder, update its speed window.  Returns edges this cycle. */
LOCAL UW s2_update_wheel(S2_SIDE side)
{
	WHEEL	*p_w = &wheel[side];
	UW	now = encoder_count(side);
	UW	d = now - p_w->last_count;
	UINT	i;
	UW	sum = 0U;

	p_w->last_count = now;
	for (i = 0U; (i + 1U) < (UINT)S2_SPEED_WINDOW; i++) {
		p_w->window[i] = p_w->window[i + 1U];
	}
	p_w->window[S2_SPEED_WINDOW - 1] = (UH)((d > 0xFFFFU) ? 0xFFFFU : d);
	if (p_w->filled < (UINT)S2_SPEED_WINDOW) {
		p_w->filled++;
	}
	for (i = (UINT)S2_SPEED_WINDOW - p_w->filled; i < (UINT)S2_SPEED_WINDOW; i++) {
		sum += p_w->window[i];
	}
	/* mm/s = edges x um_per_edge / 1000 / (n x 20 ms / 1000) */
	p_w->speed = (W)((sum * (UW)S2_UM_PER_EDGE) / (p_w->filled * (UW)S2_PERIOD_MS));
	if (p_w->target < 0) {
		p_w->last_sign = -1;
	} else if (p_w->target > 0) {
		p_w->last_sign = 1;
	} else {
		;	/* keep the last direction while coasting to a stop */
	}
	p_w->speed *= p_w->last_sign;
	return d;
}

/* PI loop: returns duty in permille (-1000..1000). */
LOCAL INT s2_pi(WHEEL *p_w)
{
	W	err;
	W	duty;

	if (p_w->target == 0) {
		p_w->integ = 0;
		return 0;
	}
	err = p_w->target - p_w->speed;
	p_w->integ += err;		/* sum of error, (mm/s) x cycles */
	if (p_w->integ > S2_INTEG_LIMIT) {
		p_w->integ = S2_INTEG_LIMIT;
	} else if (p_w->integ < -S2_INTEG_LIMIT) {
		p_w->integ = -S2_INTEG_LIMIT;
	} else {
		;
	}
	duty = ((p_w->target * S2_FF_PERMILLE_X1000) / 1000) +
	       ((err * S2_KP_X1000) / 1000) +
	       ((p_w->integ * S2_KI_X1000) / 1000);

	/* Never drive against the commanded direction (no reversing to brake). */
	if ((p_w->target > 0) && (duty < 0)) {
		duty = 0;
	} else if ((p_w->target < 0) && (duty > 0)) {
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

LOCAL BOOL s2_check_stall(WHEEL *p_w)
{
	W	t = s2_abs(p_w->target);

	if (t == 0) {
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

/* ------------------------------------------------------------------ */
LOCAL void s2_set_targets(void)
{
	switch (mode) {
	case MM_FOLLOW: {
		W	err;
		UW	age_ms;
		UW	flags;
		W	base = s2_cap(follow_speed);
		W	dv;

		s3_line_get(&err, &age_ms, &flags);
		if (age_ms > (UW)S2_LINE_STALE_MS) {
			base /= 2;	/* line data late: slow down, keep steering */
		}
		dv = ((err * S2_LINE_KP_X1000) + ((err - prev_line_err) * S2_LINE_KD_X1000)) / 1000;
		prev_line_err = err;
		wheel[S2_LEFT].target = base - dv;
		wheel[S2_RIGHT].target = base + dv;
		/* keep both wheels between 0 and 2 x base (no pivoting) */
		{
			INT	i;
			for (i = 0; i < 2; i++) {
				if (wheel[i].target < 0) {
					wheel[i].target = 0;
				} else if (wheel[i].target > (2 * base)) {
					wheel[i].target = 2 * base;
				} else {
					;
				}
			}
		}
		break;
	}
	case MM_MOVE:
	case MM_TURN:
		if (move_done_edges >= move_target_edges) {
			mode = MM_IDLE;
			wheel[S2_LEFT].target = 0;
			wheel[S2_RIGHT].target = 0;
			s2_post_status(MST_DONE);
		} else {
			W	v = s2_cap(move_speed);
			wheel[S2_LEFT].target = v * move_dir_l;
			wheel[S2_RIGHT].target = v * move_dir_r;
		}
		break;
	case MM_IDLE:
	case MM_FAULT:
	default:
		wheel[S2_LEFT].target = 0;
		wheel[S2_RIGHT].target = 0;
		break;
	}
}

EXPORT void motion_task(INT stacd, void *exinf)
{
	MOTION_CMD	cmd;
	UINT		print_div = 0U;

	(void)stacd;
	(void)exinf;

	(void)motor_init();		/* motors start stopped */
	(void)encoder_init();
	wheel[S2_LEFT].last_count = encoder_count(S2_LEFT);
	wheel[S2_RIGHT].last_count = encoder_count(S2_RIGHT);
	wheel[S2_LEFT].last_sign = 1;
	wheel[S2_RIGHT].last_sign = 1;
	s2_hw_ready = TRUE;

	while (1) {
		UW	dl;
		UW	dr;

		(void)tk_slp_tsk(TMO_FEVR);	/* cyclic handler, every 20 ms */
		(void)tk_can_wup(TSK_SELF);	/* drop any backlog */

		while (tk_rcv_mbf(mbf_motion_cmd, &cmd, TMO_POL) > 0) {
			s2_handle_cmd(&cmd);
		}

		dl = s2_update_wheel(S2_LEFT);
		dr = s2_update_wheel(S2_RIGHT);
		s2_enc_l = wheel[S2_LEFT].last_count;
		s2_enc_r = wheel[S2_RIGHT].last_count;
		s2_speed_l_mm_s = wheel[S2_LEFT].speed;
		s2_speed_r_mm_s = wheel[S2_RIGHT].speed;

		/* Odometer: mean of both wheels, always counting up. */
		odo_um += ((dl + dr) * (UW)S2_UM_PER_EDGE) / 2U;
		s2_distance_mm += (W)(odo_um / 1000U);
		odo_um %= 1000U;

		if ((mode == MM_MOVE) || (mode == MM_TURN)) {
			move_done_edges += (dl + dr) / 2U;
		}

		s2_set_targets();

		if (mode == MM_FAULT) {
			motor_stop_all();
		} else {
			motor_set(S2_LEFT, s2_pi(&wheel[S2_LEFT]));
			motor_set(S2_RIGHT, s2_pi(&wheel[S2_RIGHT]));

			if ((s2_check_stall(&wheel[S2_LEFT]) == TRUE) ||
			    (s2_check_stall(&wheel[S2_RIGHT]) == TRUE)) {
				motor_stop_all();
				mode = MM_FAULT;
				s2_faulted = TRUE;
				s2_wheel_reset(&wheel[S2_LEFT]);
				s2_wheel_reset(&wheel[S2_RIGHT]);
				tm_printf((UB *)"[S2] STALL: motors locked off\n");
				s2_post_status(MST_STALL);
			}
		}

		if (S2_PRINT_ENABLE != 0) {
			if (++print_div >= 25U) {
				print_div = 0U;
				tm_printf((UB *)"[S2] mode=%d tgt L=%d R=%d  meas L=%d R=%d mm/s  odo=%d mm\n",
					  (INT)mode, (INT)wheel[S2_LEFT].target, (INT)wheel[S2_RIGHT].target,
					  (INT)wheel[S2_LEFT].speed, (INT)wheel[S2_RIGHT].speed,
					  (INT)s2_distance_mm);
			}
		}
	}
}
