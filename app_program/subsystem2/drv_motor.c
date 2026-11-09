/*
 *	drv_motor.c
 *	Subsystem 2 -- Robo Pico DC motor driver (PWM slices 4 and 5).
 */

#include "drv_motor.h"

LOCAL const UB motor_pin_a[2] = { MOTOR_L_A_PIN, MOTOR_R_A_PIN };
LOCAL const UB motor_pin_b[2] = { MOTOR_L_B_PIN, MOTOR_R_B_PIN };
LOCAL const BOOL motor_invert[2] = { MOTOR_L_INVERT, MOTOR_R_INVERT };

EXPORT void drv_motor_init(void)
{
	UINT	m;

	app_unreset(RESETS_RESET_PWM);

	for (m = 0; m < 2U; m++) {
		/* Both channels of the slice: same wrap, start braked */
		pwm_set_pin(motor_pin_a[m]);
		pwm_set_pin(motor_pin_b[m]);
		pwm_set_wrap(motor_pin_a[m], MOTOR_PWM_WRAP);
		pwm_set_cc(motor_pin_a[m], 0);
		pwm_set_cc(motor_pin_b[m], 0);
		pwm_set_enabled(motor_pin_a[m], TRUE);
	}
}

EXPORT void drv_motor_set(UINT motor, INT duty)
{
	UW	cc;
	INT	d;

	if (motor > 1U) {
		return;
	}
	d = APP_CLAMP(duty, -MOTOR_DUTY_MAX, MOTOR_DUTY_MAX);
	if (motor_invert[motor]) {
		d = -d;
	}
	cc = ((UW)APP_ABS(d) * (MOTOR_PWM_WRAP + 1U)) / (UW)MOTOR_DUTY_MAX;

	if (d > 0) {			/* forward: A = PWM, B = 0 */
		pwm_set_cc(motor_pin_b[motor], 0);
		pwm_set_cc(motor_pin_a[motor], cc);
	} else if (d < 0) {		/* reverse: A = 0, B = PWM */
		pwm_set_cc(motor_pin_a[motor], 0);
		pwm_set_cc(motor_pin_b[motor], cc);
	} else {			/* brake: A = B = 0 */
		pwm_set_cc(motor_pin_a[motor], 0);
		pwm_set_cc(motor_pin_b[motor], 0);
	}
}

EXPORT void drv_motor_brake_all(void)
{
	drv_motor_set(MOTOR_LEFT, 0);
	drv_motor_set(MOTOR_RIGHT, 0);
}

EXPORT void drv_motor_coast_all(void)
{
	UINT	m;

	for (m = 0; m < 2U; m++) {
		pwm_set_cc(motor_pin_a[m], MOTOR_PWM_WRAP + 1U);
		pwm_set_cc(motor_pin_b[m], MOTOR_PWM_WRAP + 1U);
	}
}
