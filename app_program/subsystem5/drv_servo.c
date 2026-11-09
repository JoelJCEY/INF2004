/*
 *	drv_servo.c
 *	Subsystem 5 -- 50 Hz servo PWM.  125 MHz / 40 = 3.125 MHz,
 *	62500 counts = 20 ms, so 1 µs = 3.125 counts.
 */

#include "drv_servo.h"

#define SERVO_SLICE		6U
#define SERVO_DIV_INT		40U
#define SERVO_WRAP		62499U
#define PWM_DIV_REG(s)		(PWM_BASE + ((s) * 0x14UL) + PWM_CHx_DIV)

EXPORT void drv_servo_init(void)
{
	app_unreset(RESETS_RESET_PWM);
	pwm_set_pin(SERVO_PIN);
	out_w(PWM_DIV_REG(SERVO_SLICE), SERVO_DIV_INT << 4);	/* 8.4 fixed point */
	pwm_set_wrap(SERVO_PIN, SERVO_WRAP);
	drv_servo_set_idx(SERVO_CENTRE_IDX);
	pwm_set_enabled(SERVO_PIN, TRUE);
}

LOCAL INT	cur_deg = -1;	/* last angle written; -1 = not yet known */

LOCAL void write_deg(UW deg)
{
	UW	us = SERVO_US_0DEG + ((deg * (SERVO_US_180DEG - SERVO_US_0DEG)) / 180U);

	pwm_set_cc(SERVO_PIN, (us * 25U) / 8U);	/* x 3.125 */
}

EXPORT void drv_servo_set_idx(UINT idx)
{
	INT	target = (INT)SERVO_IDX_TO_DEG((idx > 24U) ? 24U : idx);
	INT	step;

	scan_status_word = (scan_status_word & ~0x1FUL) | (idx & 0x1FUL);

	if (cur_deg < 0) {
		/* first command after power-up: position unknown, set it */
		write_deg((UW)target);
		cur_deg = target;
		return;
	}
	/* ramp 1° at a time (SERVO_RAMP_xx), returns once there */
	while (cur_deg != target) {
		step = target - cur_deg;
		if (step > SERVO_RAMP_DEG) {
			step = SERVO_RAMP_DEG;
		} else if (step < -SERVO_RAMP_DEG) {
			step = -SERVO_RAMP_DEG;
		}
		cur_deg += step;
		write_deg((UW)cur_deg);
		(void)tk_dly_tsk(SERVO_RAMP_MS);
	}
}
