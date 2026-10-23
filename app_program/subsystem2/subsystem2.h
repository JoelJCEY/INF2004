/*
 *	subsystem2.h
 *	Subsystem 2 -- Motion control: motors, encoders, speed control (Buddy 2).
 *
 *	BASIC VERSION.
 *	  - Encoders: rising edges of each encoder's B output counted by a
 *	    spare PWM slice (no CPU, no interrupt, but no direction: the
 *	    sign comes from the commanded direction).
 *	  - Speed: per-wheel PI loop every 20 ms on a 100 ms speed window.
 *	  - Line following: PD steering on line_task's latest line error.
 *	  - Moves and turns measured with encoder edges.
 *	The complete version swaps in PIO quadrature decoding and soft start.
 *
 *	Public interface (used by ctrl_task only): the motion API below.
 *	Each call posts one command on the Motion Command Queue.
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"

#define S2_PRI_MOTION		11
#define S2_PERIOD_MS		20	/* NFR1: PID every 20 ms */
#define S2_PRINT_ENABLE		0	/* 1 = print speeds every 0.5 s */

/* Pins (Section 2) */
#define S2_PIN_M1A		8	/* left motor,  PWM slice 4 A */
#define S2_PIN_M1B		9
#define S2_PIN_M2A		10	/* right motor, PWM slice 5 A */
#define S2_PIN_M2B		11
#define S2_PIN_ENC_L_A		2	/* Grove 2 */
#define S2_PIN_ENC_L_B		3	/* PWM slice 1 B: edge counter */
#define S2_PIN_ENC_R_A		4	/* Grove 3 */
#define S2_PIN_ENC_R_B		5	/* PWM slice 2 B: edge counter */

/* ---- Calibration constants: MEASURE THESE (assumption A2) ---------- */
/* Distance per counted edge, in micrometres.  Turn a wheel exactly one
 * revolution by hand, read the bring-up edge count N, then
 * UM_PER_EDGE = (pi x wheel diameter in um) / N.                       */
#define S2_UM_PER_EDGE		1000	/* placeholder: 1 mm per edge */
#define S2_TRACK_MM		130	/* distance between wheel centres */
#define S2_LEFT_INVERT		0	/* 1 if "forward" spins backwards */
#define S2_RIGHT_INVERT		0

/* Speed loop (permille duty).  FF: duty needed per mm/s, x1000. */
#define S2_FF_PERMILLE_X1000	1500	/* 300 mm/s -> 450 permille */
#define S2_KP_X1000		800
#define S2_KI_X1000		200
#define S2_INTEG_LIMIT		2000	/* clamp on sum of error: 2000 x 0.2 = 400 permille */
#define S2_SPEED_WINDOW		5	/* cycles = 100 ms speed average */

/* Line PD steering: speed difference (mm/s) per unit of line error. */
#define S2_LINE_KP_X1000	150	/* error +-1500 -> +-225 mm/s */
#define S2_LINE_KD_X1000	300
#define S2_LINE_STALE_MS	80	/* older line error = slow down */

/* Stall: speed commanded but measured < 20 % of it for 300 ms. */
#define S2_STALL_PERCENT	20
#define S2_STALL_MS		300

#define S2_MOVE_MM_S		200	/* speed for encoder-measured moves */
#define S2_TURN_MM_S		150	/* wheel speed during turns */

typedef enum {
	S2_LEFT = 0,
	S2_RIGHT = 1
} S2_SIDE;

/* ---- Motion Command Queue (owned by motion_task) ------------------- */
typedef enum {
	MC_STOP = 0,		/* brake, cancel any move */
	MC_FOLLOW,		/* follow the line at speed_mm_s */
	MC_MOVE,		/* straight, signed distance_mm */
	MC_TURN,		/* in place, signed angle_deg (+ = left) */
	MC_SPEED_LIMIT,		/* cap speed in every mode (0 = no cap) */
	MC_RESET_FAULT		/* clear MOTOR FAULT after a stall */
} MOTION_CMD_KIND;

typedef struct {
	MOTION_CMD_KIND	kind;
	W		value;		/* mm, degrees or mm/s */
	W		speed_mm_s;	/* 0 = default speed */
} MOTION_CMD;

IMPORT ID	mbf_motion_cmd;

/* Motion API (project brief: moveForward, moveBackward, turnLeft,
 * turnRight, stop).  Call from ctrl_task only. */
IMPORT ER	motion_move_forward(W distance_mm);
IMPORT ER	motion_move_backward(W distance_mm);
IMPORT ER	motion_turn_left(W angle_deg);
IMPORT ER	motion_turn_right(W angle_deg);
IMPORT ER	motion_stop(void);
IMPORT ER	motion_follow_line(W speed_mm_s);
IMPORT ER	motion_set_speed_limit(W speed_mm_s);
IMPORT ER	motion_reset_fault(void);

/* ---- Published values: written only by motion_task, one aligned word
 * each, so any task may read them without a lock. --------------------- */
IMPORT volatile W	s2_distance_mm;		/* odometer (always counts up) */
IMPORT volatile W	s2_speed_l_mm_s;	/* signed */
IMPORT volatile W	s2_speed_r_mm_s;
IMPORT volatile UW	s2_enc_l;		/* edge counts */
IMPORT volatile UW	s2_enc_r;
IMPORT volatile BOOL	s2_hw_ready;
IMPORT volatile BOOL	s2_faulted;

IMPORT ID	tid_motion;
IMPORT ID	cyc_motion;
IMPORT void	motion_task(INT stacd, void *exinf);
IMPORT void	motion_cyclic_handler(void *exinf);

#endif /* SUBSYSTEM2_H */
