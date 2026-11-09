/*
 *	drv_servo.h
 *	Subsystem 5 -- scan servo on GP12 (Servo port 1), PWM slice 6A, 50 Hz.
 *	Owner: scan_task only.
 */

#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include "subsystem5.h"

#define SERVO_US_0DEG		500	/* pulse at 0° (check the servo's datasheet) */
#define SERVO_US_180DEG		2500	/* pulse at 180° */
#define SERVO_SETTLE_MS		80	/* per 5-15° step before a ping */
/* 31 Oct: gentle moves.  The pulse is changed 1° at a time with a pause
   in between, instead of jumping to the new angle at full speed.  Less
   jerk on the sensor and bracket, smaller current spikes from the
   battery.  1° every 12 ms = about 80°/s (a hobby servo jumps at
   400-600°/s; 31 Oct: 6 ms was still too sharp, the sensor came loose).
   A software change cannot lower the servo's torque. */
#define SERVO_RAMP_DEG		1	/* degrees per ramp step  */
#define SERVO_RAMP_MS		12	/* pause between ramp steps */

IMPORT void drv_servo_init(void);
IMPORT void drv_servo_set_idx(UINT idx);	/* 0..24 = 30°..150° */

#endif /* DRV_SERVO_H */
