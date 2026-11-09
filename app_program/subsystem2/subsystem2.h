/*
 *	subsystem2.h
 *	Subsystem 2 -- Motion Control (motion_task).
 *	Buddy 2: 2501113 (Joel).  Statechart 6.2, assumption A2.
 *
 *	Public interface of subsystem 2.  Other subsystems include it as
 *	"../subsystem2/subsystem2.h".
 *
 *	Owns: both DC motors (GP8-GP11, PWM slices 4/5) and both quadrature
 *	encoders (GP2-GP5, PIO).  motion_task is the ONLY writer of the motor
 *	pins; ctrl_task is the ONLY sender on the Motion Command Queue.
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"

#define S2_PRI_MOTION		APP_PRI_MOTION
#define S2_PERIOD_MS		20	/* PID period, 50 Hz (NFR1) */

/* ---------------------------------------------------------------------
 *  Geometry and speeds.  ESTIMATES, not measurements: wheel size and
 *  encoder slots from a past INF2004 car with the same kit wheels
 *  (204.2 mm circumference, 20-slot disc), track width from its 90° turn.
 *  Replace with the A2 measurements (S2-M01, ruler, 1 m run, 90° turn).
 * --------------------------------------------------------------------- */
#define MOTION_WHEEL_DIAM_MM	65	/* est.: 204.2 mm / pi               */
#define MOTION_COUNTS_PER_REV	2750	/* measured 26 Oct by hand: L 2885, R 2601 */
#define MOTION_TRACK_MM		117	/* est.: wheel centre to wheel centre */
/* 4 Nov: pivot turns went too far (90 deg commanded -> about 110 on the
   track: tyre scrub and the 117 mm estimate).  Every turn on the spot
   (motion_turn_counts) is scaled by this.  To re-tune: command a 90 deg
   turn, measure the real angle A, set 90 * (current value) / A. */
#define MOTION_TURN_SCALE_PCT	82
#define MOTION_CRUISE_MM_S	60	/* 0.06 m/s, 28 Oct */
#define MOTION_MAX_MM_S		500	/* 0.5 m/s demonstration maximum     */
#define MOTION_TURN_MM_S	40	/* wheel speed while pivoting        */
#define MOTION_MOVE_MM_S	40	/* default speed for measured moves  */

/* 1 = print raw encoder pins and counts on the console while the car is
   stopped (bring-up / S2-M01).  Set to 0 once the encoders work. */
#define MOTION_ENC_DIAG		1

/* Straight-line wheel sync while following (task_motion.c, 2 Nov): on a
   straight the right wheel's counts are scaled by this /1000 before they
   are compared with the left's.  1000 = both wheels give the same counts
   per turn.  If the car STILL drifts on a straight with the sync on, the
   two encoders differ: drive 1 m straight, read "counts L / R" on the
   dashboard and set this to 1000 x L / R (the 26 Oct hand count,
   2885 / 2601, would give 1109). */
#define MOTION_SYNC_R_X1000	1000

/* Encoder sign: set to 1 if a wheel counts down while driving forward */
#define MOTION_ENC_L_INVERT	0	/* hand test: left counts up turning forward */
#define MOTION_ENC_R_INVERT	1	/* mirrored mounting on the right */

/* ---------------------------------------------------------------------
 *  IF-05  Motion Command Queue (ctrl_task -> motion_task)
 *  µT-Kernel message buffer mbf_motion_cmd, owned by subsystem 2.
 * --------------------------------------------------------------------- */
typedef enum {
	MC_STOP		= 0,	/* brake to rest; pre-empts anything        */
	MC_FOLLOW_LINE	= 1,	/* PID line following at speed_mm_s         */
	MC_MOVE		= 2,	/* arg = signed distance mm (+ forward)     */
	MC_TURN		= 3,	/* arg = signed angle 0.1° (+ left / CCW)   */
	MC_SPEED_LIMIT	= 4,	/* speed_mm_s = limit, 0 = no limit         */
	MC_LOCK		= 5,	/* FAULT STOP: motors locked off until reset*/
	MC_RESET	= 6,	/* clear lock / stall, go idle (braked)     */
	MC_SELFTEST	= 7	/* INITIALISING motor + encoder check       */
} MOTION_OP;

typedef struct {
	UB	op;		/* MOTION_OP                       */
	UB	seq;		/* echoed back in MOTION_STATUS    */
	UH	speed_mm_s;	/* 0..MOTION_MAX_MM_S              */
	W	arg;		/* see MOTION_OP                   */
} MOTION_CMD;
APP_STATIC_ASSERT(sizeof(MOTION_CMD) == 8, motion_cmd_size);

#define S2_CMD_QUEUE_MSGS	8
IMPORT ID	mbf_motion_cmd;

/* ---------------------------------------------------------------------
 *  IF-06  Motion Status Queue (motion_task -> ctrl_task)
 *  Message buffer owned by ctrl_task (mbf_motion_status, subsystem5.h),
 *  followed by tk_set_flg(flg_sys, EVF_MOTION).
 * --------------------------------------------------------------------- */
typedef enum {
	MS_DONE			= 1,	/* move or turn reached its target      */
	MS_STOPPED		= 2,	/* wheels at rest after MC_STOP         */
	MS_STALL		= 3,	/* also sets FLT_STALL                  */
	MS_ENCODER_FAULT	= 4,	/* also sets FLT_ENCODER                */
	MS_SELFTEST_PASS	= 5,
	MS_SELFTEST_FAIL	= 6	/* also sets FLT_ENCODER                */
} MOTION_EVENT;

typedef struct {
	UB	event;		/* MOTION_EVENT                    */
	UB	seq;		/* seq of the command it refers to */
	UB	mode;		/* MOTION_MODE at the time         */
	UB	reserved;
	H	speed_mm_s;	/* average of both wheels          */
	H	reserved2;
	UW	path_mm;	/* total distance travelled        */
} MOTION_STATUS;
APP_STATIC_ASSERT(sizeof(MOTION_STATUS) == 12, motion_status_size);

/* motion_task modes (statechart 6.2) */
typedef enum {
	MM_IDLE		= 0,	/* braked, waiting for a command */
	MM_FOLLOW	= 1,	/* FOLLOWING LINE                */
	MM_MOVE		= 2,	/* MOVING                        */
	MM_TURN		= 3,	/* TURNING                       */
	MM_STOPPING	= 4,	/* STOPPING                      */
	MM_LOCKED	= 5,	/* STALLED / FAULT: locked off   */
	MM_SELFTEST	= 6
} MOTION_MODE;

/* ---------------------------------------------------------------------
 *  Motion API required by the project brief (P2, interface map 11.2).
 *  Called by ctrl_task only: each call posts one MOTION_CMD and returns
 *  its sequence number (0..255) or a negative error code.
 * --------------------------------------------------------------------- */
IMPORT INT moveForward(W distance_mm);
IMPORT INT moveForwardAt(W distance_mm, UH speed_mm_s);	/* 4 Nov */
IMPORT INT moveBackwardAt(W distance_mm, UH speed_mm_s);
IMPORT INT moveBackward(W distance_mm);
IMPORT INT turnLeft(W angle_deg);
IMPORT INT turnRight(W angle_deg);
IMPORT INT stop(void);
/* Extra commands used by the Vehicle Controller */
IMPORT INT motion_follow_line(UH speed_mm_s);
IMPORT INT motion_set_speed_limit(UH limit_mm_s);	/* 0 = none */
IMPORT INT motion_lock(void);
IMPORT INT motion_reset(void);
IMPORT INT motion_selftest(void);

/* ---------------------------------------------------------------------
 *  IF-07  Odometry (motion_task -> line_task, imu_task, ctrl_task)
 *  Each value is ONE aligned 32-bit word with motion_task as its only
 *  writer, so a reader always sees a whole value (design 10.4 rule 1).
 * --------------------------------------------------------------------- */
IMPORT volatile UW motion_odo_path_01mm;  /* total path, 0.1 mm, wraps  */
IMPORT volatile UW motion_odo_speed;      /* L | R<<16, offset binary   */
IMPORT volatile W  motion_enc_count[2];   /* raw signed counts L, R     */
IMPORT volatile UW motion_mode_word;      /* MOTION_MODE                */
IMPORT volatile UW motion_jitter_max_us;  /* worst release jitter (A1)  */

/* Unpack helpers for motion_odo_speed (offset binary, mm/s) */
Inline H motion_speed_left(void)
{
	return (H)((INT)(motion_odo_speed & 0xFFFFU) - 32768);
}
Inline H motion_speed_right(void)
{
	return (H)((INT)(motion_odo_speed >> 16) - 32768);
}
Inline H motion_speed_avg(void)
{
	UW	w = motion_odo_speed;	/* one read: L and R from one tick */
	INT	l = (INT)(w & 0xFFFFU) - 32768;
	INT	r = (INT)(w >> 16) - 32768;
	return (H)((l + r) / 2);
}

/* ---------------------------------------------------------------------
 *  Private to subsystem 2, shared between its own .c files
 * --------------------------------------------------------------------- */
IMPORT ID	tid_motion;
IMPORT ID	cyc_motion;
IMPORT void	motion_task(INT stacd, void *exinf);
IMPORT void	motion_cyclic_handler(void *exinf);
IMPORT BOOL	motion_encoders_ok;

#endif /* SUBSYSTEM2_H */
