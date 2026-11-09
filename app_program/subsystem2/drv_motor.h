/*
 *	drv_motor.h
 *	Subsystem 2 -- Robo Pico DC motor driver (M1: GP8/GP9, M2: GP10/GP11).
 *
 *	Driver truth table (design 2, pin table):
 *	  A = PWM, B = 0  -> forward      A = 0, B = PWM -> reverse
 *	  A = B = 0       -> brake        A = B = 1      -> coast
 *	Owner: motion_task only.  No other task may call these (design 8.2).
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include "../app.h"

/* On the IS11 car the motor on terminal M1 (GP8/GP9) drives the RIGHT
   wheel and M2 (GP10/GP11) the LEFT wheel, the opposite of the design's
   pin table (found in bring-up, 26 Oct).  Set to 0 if the wires are
   moved to match the design (left motor on M1). */
#define MOTOR_SWAP_SIDES	1

#if MOTOR_SWAP_SIDES
#define MOTOR_L_A_PIN		10	/* M2A, PWM slice 5A */
#define MOTOR_L_B_PIN		11	/* M2B, PWM slice 5B */
#define MOTOR_R_A_PIN		8	/* M1A, PWM slice 4A */
#define MOTOR_R_B_PIN		9	/* M1B, PWM slice 4B */
#else
#define MOTOR_L_A_PIN		8	/* M1A, PWM slice 4A */
#define MOTOR_L_B_PIN		9	/* M1B, PWM slice 4B */
#define MOTOR_R_A_PIN		10	/* M2A, PWM slice 5A */
#define MOTOR_R_B_PIN		11	/* M2B, PWM slice 5B */
#endif

/* 125 MHz / (6249 + 1) = 20 kHz: inaudible and within the ≤ 20 kHz limit */
#define MOTOR_PWM_WRAP		6249U
#define MOTOR_DUTY_MAX		1000	/* duty in permille, signed */

/* Set to 1 if a wheel turns the wrong way and its wires cannot be swapped */
#define MOTOR_L_INVERT		1	/* left motor spins backward on forward duty */
#define MOTOR_R_INVERT		0

#define MOTOR_LEFT		0
#define MOTOR_RIGHT		1

IMPORT void drv_motor_init(void);
/* duty: -1000..+1000 permille, + = forward.  0 = brake. */
IMPORT void drv_motor_set(UINT motor, INT duty);
IMPORT void drv_motor_brake_all(void);
IMPORT void drv_motor_coast_all(void);

#endif /* DRV_MOTOR_H */
