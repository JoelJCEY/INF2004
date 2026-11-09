/*
 *	ctrl_logic.c
 *	Subsystem 5 -- Vehicle Controller rules (system statechart 5,
 *	fault table 5.1).  Safety-related stop and fault events take
 *	priority over navigation.
 */

#include "ctrl_logic.h"

EXPORT BOOL ctrl_is_active(VEHICLE_STATE s)
{
	return ((s == VS_LINE_FOLLOWING) || (s == VS_BARCODE_MANOEUVRE) ||
		(s == VS_HUMP_CROSSING) || (s == VS_OBSTACLE_BYPASS) ||
		(s == VS_LINE_RECOVERY)) ? TRUE : FALSE;
}

EXPORT VEHICLE_STATE ctrl_on_command(VEHICLE_STATE s, REMOTE_CMD c)
{
	switch (c) {
	case RC_STOP:
		/* From any active manoeuvre state */
		return ctrl_is_active(s) ? VS_STOPPED : s;
	case RC_START:
		/* Rejected until INITIALISING is complete, and in FAULT STOP */
		return ((s == VS_READY) || (s == VS_STOPPED)) ? VS_LINE_FOLLOWING : s;
	case RC_RESET:
		/* FAULT STOP re-runs the start-up checks; others go READY */
		if (s == VS_FAULT_STOP) {
			return VS_INITIALISING;
		}
		if ((s == VS_STOPPED) || (s == VS_MISSION_COMPLETE) || (s == VS_READY)) {
			return VS_READY;
		}
		return s;
	default:
		return s;
	}
}

EXPORT VEHICLE_STATE ctrl_on_faults(VEHICLE_STATE s, UINT faults)
{
	if (s == VS_FAULT_STOP) {
		return s;
	}
	if ((faults & FLT_STOP_FAULTS) != 0U) {
		return VS_FAULT_STOP;		/* from any state */
	}
	if (((faults & FLT_IMPACT) != 0U) && ctrl_is_active(s)) {
		return VS_STOPPED;
	}
	if (((faults & FLT_LINE_LOST) != 0U) &&
	    ((s == VS_LINE_FOLLOWING) || (s == VS_HUMP_CROSSING))) {
		/* Not while barcode turns or the bypass leave the line on purpose */
		return VS_STOPPED;
	}
	return s;	/* NO_ECHO, WIFI: reported only */
}

EXPORT INT ctrl_plan_bypass(UB side, W offset_mm, W depth_mm, CTRL_STEP *st)
{
	W	s;

	if (side == OBS_SIDE_LEFT) {
		s = 1;
	} else if (side == OBS_SIDE_RIGHT) {
		s = -1;
	} else {
		return 0;
	}
	/* back off first, then go round; the forward leg makes up the
	   distance reversed */
	st[0].op = STEP_MOVE;	st[0].arg = -CTRL_BYPASS_BACKUP_MM;
	st[1].op = STEP_TURN;	st[1].arg = s * 90;
	st[2].op = STEP_MOVE;	st[2].arg = offset_mm;
	st[3].op = STEP_TURN;	st[3].arg = -s * 90;
	/* Drive past while scan_task watches the obstacle's side (STEP_PASS).
	   4 Nov: both sides.  Going left (the usual side) the sensor looks
	   right; going right it looks left (task_ctrl picks the angle). */
	st[4].op = STEP_PASS;
	st[4].arg = depth_mm + CTRL_BYPASS_BACKUP_MM + CTRL_PASS_MAX_EXTRA_MM;
	st[5].op = STEP_TURN;	st[5].arg = -s * 90;
	st[6].op = STEP_SEARCH;	st[6].arg = offset_mm + CTRL_SEARCH_EXTRA_MM;
	/* 31 Oct: land ON the line.  The sensors found it, but the car
	   pivots on its wheels, CTRL_SENSOR_AHEAD_MM behind them: a plain
	   90 deg turn swung the sensors off the line (big search turns).
	   Drive the wheels over the line, then turn until the sensors meet
	   it again: turning about a point on the line ends aligned. */
	st[7].op = STEP_MOVE;	st[7].arg = CTRL_SENSOR_AHEAD_MM;
	st[8].op = STEP_TURN;	st[8].arg = s * CTRL_LAND_FIXED_DEG;
	st[9].op = STEP_SEARCH_TURN; st[9].arg = s * CTRL_LAND_SEARCH_DEG;
	return 10;
}

EXPORT INT ctrl_plan_barcode(BARCODE_CMD cmd, CTRL_STEP *st)
{
	INT	n = 0;
	W	angle;

	switch (cmd) {
	case BC_TURN_LEFT:	angle = 90;	break;
	case BC_TURN_RIGHT:	angle = -90;	break;
	case BC_U_TURN:		angle = 180;	break;
	default:
		return 0;	/* straight / unrecognised: keep following */
	}
	if (CTRL_BARCODE_OFFSET_MM > 0) {
		st[n].op = STEP_MOVE;
		st[n].arg = CTRL_BARCODE_OFFSET_MM;
		n++;
	}
	st[n].op = STEP_TURN;
	st[n].arg = angle;
	n++;
	return n;
}

EXPORT INT ctrl_plan_junction(BARCODE_CMD cmd, W past_mm, CTRL_STEP *st)
{
	INT	n = 0;
	W	angle;
	W	move = CTRL_SENSOR_AHEAD_MM - past_mm;
	W	s;

	switch (cmd) {
	case BC_TURN_LEFT:	angle = 90;	break;
	case BC_TURN_RIGHT:	angle = -90;	break;
	case BC_U_TURN:		angle = 180;	break;
	default:
		return 0;	/* forward / not in the map: straight on */
	}
	/* bring the wheel axle over the middle of the junction */
	if (APP_ABS(move) > CTRL_JUNCTION_DEADBAND_MM) {
		st[n].op = STEP_MOVE;
		st[n].arg = move;	/* + forward, - backward */
		n++;
	}
	/* fixed part, then search the last part (+ = left) */
	s = (angle > 0) ? 1 : -1;
	st[n].op = STEP_TURN;
	st[n].arg = angle - (s * (CTRL_JUNCTION_SEARCH_DEG / 2));
	n++;
	st[n].op = STEP_SEARCH_TURN;
	st[n].arg = s * CTRL_JUNCTION_SEARCH_DEG;
	n++;
	return n;
}
