/*
 *	motion_ctrl.h
 *	Subsystem 2 -- pure control helpers (no hardware, no kernel calls),
 *	so they can be unit-tested on target (subsystem2_test.c).
 *
 *	All maths is 32-bit integer / fixed point (design 10.2: internal
 *	calculations stay wide; only stored and shared values are packed).
 */

#ifndef MOTION_CTRL_H
#define MOTION_CTRL_H

#include "subsystem2.h"

/* µm travelled per encoder count: pi * D * 1000 / CPR */
#define MOTION_UM_PER_COUNT \
	((W)(((UW)MOTION_WHEEL_DIAM_MM * 3141593UL) / ((UW)MOTION_COUNTS_PER_REV * 1000UL)))

/* ---------------------------------------------------------------------
 *  PID with Q8 gains, clamped integrator (VA2: no wind-up after a turn)
 * --------------------------------------------------------------------- */
typedef struct {
	W	kp_q8;		/* proportional gain x256 */
	W	ki_q8;		/* integral gain x256 (per tick) */
	W	kd_q8;		/* derivative gain x256 (per tick) */
	W	i_acc;		/* integral of error */
	W	i_limit;	/* |i_acc| clamp */
	W	out_limit;	/* |output| clamp */
	W	prev_err;
	BOOL	has_prev;
} MOTION_PID;

IMPORT void motion_pid_init(MOTION_PID *pid, W kp_q8, W ki_q8, W kd_q8,
			    W i_limit, W out_limit);
IMPORT void motion_pid_reset(MOTION_PID *pid);
IMPORT W    motion_pid_step(MOTION_PID *pid, W err);

/* ---------------------------------------------------------------------
 *  Stall detector (design 6.2 / 5.1, F11): a non-zero speed is
 *  commanded but the measured speed stays below 20% of it for 300 ms.
 * --------------------------------------------------------------------- */
#define MOTION_STALL_PCT	20
#define MOTION_STALL_TICKS	(300 / S2_PERIOD_MS)	/* 15 ticks */
#define MOTION_STALL_MIN_CMD	40	/* mm/s: below this nothing is "commanded" */

typedef struct {
	UB	ticks;
} MOTION_STALL;

IMPORT void motion_stall_reset(MOTION_STALL *s);
/* Returns TRUE once the rule has held for MOTION_STALL_TICKS ticks. */
IMPORT BOOL motion_stall_update(MOTION_STALL *s, W cmd_mm_s, W meas_mm_s);

/* ---------------------------------------------------------------------
 *  Kinematics and shaping
 * --------------------------------------------------------------------- */
IMPORT W  motion_mm_to_counts(W mm);
IMPORT W  motion_counts_to_01mm(W counts);
IMPORT W  motion_turn_counts(W angle_01deg);	/* per-wheel arc counts */
IMPORT W  motion_ramp(W current, W target, W step);	/* soft start */
IMPORT UW motion_pack_speed(W left_mm_s, W right_mm_s);	/* offset binary */
/* Speed setpoint near the end of a move: slows down over the last
   MOTION_DECEL_MM so the brake does not overshoot. */
#define MOTION_DECEL_MM		40
#define MOTION_MIN_MM_S		60
IMPORT W  motion_decel_speed(W cruise_mm_s, W remaining_counts);

#endif /* MOTION_CTRL_H */
