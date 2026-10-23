/*
 *	drv_servo.h
 *	Subsystem 5 -- hobby servo on GP12 (PWM slice 6 A), 50 Hz.
 *	Call only from scan_task.
 */

#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include "subsystem5.h"

#define SERVO_PWM_DIV_INT	125U	/* 125 MHz / 125 = 1 MHz: 1 count = 1 us */
#define SERVO_PWM_WRAP		19999U	/* 20 000 us = 50 Hz */

IMPORT ER	servo_init(void);
IMPORT void	servo_set_deg(UINT deg);	/* 0..180, clamped */

#endif /* DRV_SERVO_H */
