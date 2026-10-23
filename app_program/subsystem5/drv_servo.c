/*
 *	drv_servo.c
 *	Subsystem 5 -- servo PWM.  libbsp has no clock-divider call, so the
 *	slice's DIV register is written directly.
 */

#include "drv_servo.h"

#define REG_ALIAS_CLR		0x3000U
#define PWM_SLICE_OF(pin)	(((UINT)(pin) >> 1) & 0x07U)
#define PWM_DIV(sl)		(PWM_BASE + PWM_CHx_DIV + ((sl) * 0x14U))

/* Same idempotent helper as subsystem 2 (kept local so this subsystem
   builds and runs on its own).  The kernel leaves PWM in reset because
   USE_PTMR is off. */
LOCAL void s5_pwm_release_reset(void)
{
	if ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0U) {
		out_w(RESETS_RESET + REG_ALIAS_CLR, RESETS_RESET_PWM);
		while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0U) {
			;
		}
	}
}

EXPORT ER servo_init(void)
{
	s5_pwm_release_reset();
	(void)pwm_set_pin(S5_PIN_SERVO);
	out_w(PWM_DIV(PWM_SLICE_OF(S5_PIN_SERVO)), SERVO_PWM_DIV_INT << 4);
	(void)pwm_set_wrap(S5_PIN_SERVO, SERVO_PWM_WRAP);
	servo_set_deg(90U);
	(void)pwm_set_enabled(S5_PIN_SERVO, TRUE);
	return E_OK;
}

EXPORT void servo_set_deg(UINT deg)
{
	UINT	d = (deg > 180U) ? 180U : deg;
	UW	pulse_us = S5_SERVO_MIN_US +
			   (((S5_SERVO_MAX_US - S5_SERVO_MIN_US) * (UW)d) / 180U);

	(void)pwm_set_cc(S5_PIN_SERVO, pulse_us);	/* 1 count = 1 us */
}
