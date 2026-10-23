/*
 *	drv_motor.h
 *	Subsystem 2 -- motor driver on the Robo Pico M1/M2 outputs.
 *	Call only from motion_task (the owner of PWM slices 4 and 5).
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include "subsystem2.h"

#define MOTOR_PWM_WRAP		6249U	/* 125 MHz / 6250 = 20 kHz, inaudible */
#define MOTOR_DUTY_MAX		1000	/* duty is given in permille */

IMPORT ER	motor_init(void);
/* duty: -1000 (full reverse) .. 0 (stop) .. +1000 (full forward) */
IMPORT void	motor_set(S2_SIDE side, INT duty_permille);
IMPORT void	motor_stop_all(void);

/* Used by drv_encoder.c too: idempotent, safe to repeat */
IMPORT void	pwm_block_release_reset(void);

#endif /* DRV_MOTOR_H */
