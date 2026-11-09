/*
 *	motion_ctrl.c
 *	Subsystem 2 -- pure control helpers (PID, stall rule, kinematics).
 */

#include "motion_ctrl.h"

EXPORT void motion_pid_init(MOTION_PID *pid, W kp_q8, W ki_q8, W kd_q8,
			    W i_limit, W out_limit)
{
	pid->kp_q8 = kp_q8;
	pid->ki_q8 = ki_q8;
	pid->kd_q8 = kd_q8;
	pid->i_limit = i_limit;
	pid->out_limit = out_limit;
	motion_pid_reset(pid);
}

EXPORT void motion_pid_reset(MOTION_PID *pid)
{
	pid->i_acc = 0;
	pid->prev_err = 0;
	pid->has_prev = FALSE;
}

EXPORT W motion_pid_step(MOTION_PID *pid, W err)
{
	W	d = 0;
	W	out;

	pid->i_acc += err;
	pid->i_acc = APP_CLAMP(pid->i_acc, -pid->i_limit, pid->i_limit);
	if (pid->has_prev) {
		d = err - pid->prev_err;
	}
	pid->prev_err = err;
	pid->has_prev = TRUE;

	out = (pid->kp_q8 * err + pid->ki_q8 * pid->i_acc + pid->kd_q8 * d) / 256;
	return APP_CLAMP(out, -pid->out_limit, pid->out_limit);
}

EXPORT void motion_stall_reset(MOTION_STALL *s)
{
	s->ticks = 0;
}

EXPORT BOOL motion_stall_update(MOTION_STALL *s, W cmd_mm_s, W meas_mm_s)
{
	W	cmd = APP_ABS(cmd_mm_s);
	W	meas = APP_ABS(meas_mm_s);

	if ((cmd >= MOTION_STALL_MIN_CMD) &&
	    ((meas * 100) < (cmd * MOTION_STALL_PCT))) {
		if (s->ticks < 255U) {
			s->ticks++;
		}
	} else {
		s->ticks = 0;
	}
	return (s->ticks >= MOTION_STALL_TICKS) ? TRUE : FALSE;
}

EXPORT W motion_mm_to_counts(W mm)
{
	/* mm * 1000 / um_per_count, rounded */
	return (W)(((mm * 1000) + (MOTION_UM_PER_COUNT / 2)) / MOTION_UM_PER_COUNT);
}

EXPORT W motion_counts_to_01mm(W counts)
{
	return (W)((counts * MOTION_UM_PER_COUNT) / 100);
}

EXPORT W motion_turn_counts(W angle_01deg)
{
	/* Pivot turn: each wheel travels angle * pi * track / 360.
	   In µm: angle_01deg * pi * track_mm * 1000 / 3600 */
	W	arc_um = (W)(((APP_ABS(angle_01deg) * 3141593LL / 1000LL)
			      * MOTION_TRACK_MM * MOTION_TURN_SCALE_PCT) / 360000LL);

	return (arc_um + (MOTION_UM_PER_COUNT / 2)) / MOTION_UM_PER_COUNT;
}

EXPORT W motion_ramp(W current, W target, W step)
{
	if (target > current + step) {
		return current + step;
	}
	if (target < current - step) {
		return current - step;
	}
	return target;
}

EXPORT UW motion_pack_speed(W left_mm_s, W right_mm_s)
{
	UW	l = (UW)(APP_CLAMP(left_mm_s, -32768, 32767) + 32768);
	UW	r = (UW)(APP_CLAMP(right_mm_s, -32768, 32767) + 32768);

	return (l & 0xFFFFU) | ((r & 0xFFFFU) << 16);
}

EXPORT W motion_decel_speed(W cruise_mm_s, W remaining_counts)
{
	W	remaining_mm = (remaining_counts * MOTION_UM_PER_COUNT) / 1000;
	W	v;

	if (remaining_mm >= MOTION_DECEL_MM) {
		return cruise_mm_s;
	}
	if (remaining_mm < 0) {
		remaining_mm = 0;
	}
	v = MOTION_MIN_MM_S + ((cruise_mm_s - MOTION_MIN_MM_S) * remaining_mm)
				/ MOTION_DECEL_MM;
	return (v < MOTION_MIN_MM_S) ? MOTION_MIN_MM_S : v;
}
