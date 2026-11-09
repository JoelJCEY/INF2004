/*
 *	motion_api.c
 *	Subsystem 2 -- the motion API required by the project brief (P2).
 *
 *	Each function posts ONE MOTION_CMD on the Motion Command Queue and
 *	returns at once; motion_task executes it on its next 20 ms tick.
 *	Only ctrl_task calls these (it is the only writer of the queue,
 *	design 8.2), so the sequence counter below has a single writer.
 */

#include "subsystem2.h"

LOCAL UB	api_seq;

LOCAL INT post(MOTION_OP op, UH speed_mm_s, W arg)
{
	MOTION_CMD	cmd;
	ER		er;

	cmd.op = (UB)op;
	cmd.seq = ++api_seq;
	cmd.speed_mm_s = speed_mm_s;
	cmd.arg = arg;
	er = tk_snd_mbf(mbf_motion_cmd, &cmd, sizeof(cmd), TMO_POL);
	if (er < E_OK) {
		app_log((UB*)"motion api: queue full, op %d dropped\n", op);
		return er;
	}
	return (INT)cmd.seq;
}

EXPORT INT moveForward(W distance_mm)
{
	return post(MC_MOVE, 0, APP_ABS(distance_mm));
}

/* 4 Nov: the same moves at a chosen speed (bypass legs) */
EXPORT INT moveForwardAt(W distance_mm, UH speed_mm_s)
{
	return post(MC_MOVE, speed_mm_s, APP_ABS(distance_mm));
}

EXPORT INT moveBackwardAt(W distance_mm, UH speed_mm_s)
{
	return post(MC_MOVE, speed_mm_s, -APP_ABS(distance_mm));
}

EXPORT INT moveBackward(W distance_mm)
{
	return post(MC_MOVE, 0, -APP_ABS(distance_mm));
}

EXPORT INT turnLeft(W angle_deg)
{
	return post(MC_TURN, 0, APP_ABS(angle_deg) * 10);
}

EXPORT INT turnRight(W angle_deg)
{
	return post(MC_TURN, 0, -APP_ABS(angle_deg) * 10);
}

EXPORT INT stop(void)
{
	return post(MC_STOP, 0, 0);
}

EXPORT INT motion_follow_line(UH speed_mm_s)
{
	return post(MC_FOLLOW_LINE, speed_mm_s, 0);
}

EXPORT INT motion_set_speed_limit(UH limit_mm_s)
{
	return post(MC_SPEED_LIMIT, limit_mm_s, 0);
}

EXPORT INT motion_lock(void)
{
	return post(MC_LOCK, 0, 0);
}

EXPORT INT motion_reset(void)
{
	return post(MC_RESET, 0, 0);
}

EXPORT INT motion_selftest(void)
{
	return post(MC_SELFTEST, 0, 0);
}
