/*
 *	motion_api.c
 *	Subsystem 2 -- the motion API used by ctrl_task.
 *
 *	Each call builds one MOTION_CMD and posts it without waiting
 *	(TMO_POL).  ctrl_task is the only caller, so motion_task stays the
 *	only task that writes the motor pins.  Completion of a move or turn
 *	comes back as MST_DONE on the Motion Status Queue (subsystem 5).
 */

#include "subsystem2.h"

LOCAL ER motion_post(MOTION_CMD_KIND kind, W value, W speed)
{
	MOTION_CMD	cmd;

	cmd.kind = kind;
	cmd.value = value;
	cmd.speed_mm_s = speed;
	return tk_snd_mbf(mbf_motion_cmd, &cmd, (INT)sizeof(cmd), TMO_POL);
}

EXPORT ER motion_move_forward(W distance_mm)
{
	return motion_post(MC_MOVE, (distance_mm < 0) ? -distance_mm : distance_mm, 0);
}

EXPORT ER motion_move_backward(W distance_mm)
{
	return motion_post(MC_MOVE, (distance_mm < 0) ? distance_mm : -distance_mm, 0);
}

EXPORT ER motion_turn_left(W angle_deg)
{
	return motion_post(MC_TURN, (angle_deg < 0) ? -angle_deg : angle_deg, 0);
}

EXPORT ER motion_turn_right(W angle_deg)
{
	return motion_post(MC_TURN, (angle_deg < 0) ? angle_deg : -angle_deg, 0);
}

EXPORT ER motion_stop(void)
{
	return motion_post(MC_STOP, 0, 0);
}

EXPORT ER motion_follow_line(W speed_mm_s)
{
	return motion_post(MC_FOLLOW, 0, speed_mm_s);
}

EXPORT ER motion_set_speed_limit(W speed_mm_s)
{
	return motion_post(MC_SPEED_LIMIT, speed_mm_s, 0);
}

EXPORT ER motion_reset_fault(void)
{
	return motion_post(MC_RESET_FAULT, 0, 0);
}

EXPORT ER motion_move_until_line(W max_mm)
{
	return motion_post(MC_MOVE_UNTIL_LINE, max_mm, 0);
}

EXPORT ER motion_turn_until_line(W max_deg)
{
	return motion_post(MC_TURN_UNTIL_LINE, max_deg, 0);
}
