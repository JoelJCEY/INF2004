/*
 *	drv_motor.c
 *	Subsystem 2 -- motor driver.  Robo Pico truth table per motor:
 *	    A = PWM, B = 0   forward
 *	    A = 0,   B = PWM reverse
 *	    A = 0,   B = 0   stop (check against the Robo Pico datasheet
 *	                      whether this brakes or coasts)
 */

#include "drv_motor.h"

/* Atomic "clear bits" alias of any RP2040 peripheral register */
#define REG_ALIAS_CLR		0x3000U

/*
 * The kernel only releases the PWM block from reset when it uses PWM as
 * its own timer (USE_PTMR, off in this template), so we do it here.
 * The atomic alias avoids a read-modify-write on the shared RESETS
 * register.  Safe to call more than once.
 */
EXPORT void pwm_block_release_reset(void)
{
	if ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0U) {
		out_w(RESETS_RESET + REG_ALIAS_CLR, RESETS_RESET_PWM);
		while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0U) {
			;	/* takes a few clock cycles */
		}
	}
}

LOCAL void motor_pins_init(UINT pin_a, UINT pin_b)
{
	(void)pwm_set_pin(pin_a);
	(void)pwm_set_pin(pin_b);
	(void)pwm_set_wrap(pin_a, MOTOR_PWM_WRAP);	/* one TOP per slice */
	(void)pwm_set_cc(pin_a, 0U);
	(void)pwm_set_cc(pin_b, 0U);
	(void)pwm_set_enabled(pin_a, TRUE);		/* enables the slice */
}

EXPORT ER motor_init(void)
{
	pwm_block_release_reset();
	motor_pins_init(S2_PIN_M1A, S2_PIN_M1B);
	motor_pins_init(S2_PIN_M2A, S2_PIN_M2B);
	return E_OK;
}

EXPORT void motor_set(S2_SIDE side, INT duty_permille)
{
	UINT	pin_a;
	UINT	pin_b;
	UW	cc;
	INT	duty = duty_permille;
	BOOL	invert;

	if (side == S2_LEFT) {
		pin_a = S2_PIN_M1A;
		pin_b = S2_PIN_M1B;
		invert = (S2_LEFT_INVERT != 0) ? TRUE : FALSE;
	} else {
		pin_a = S2_PIN_M2A;
		pin_b = S2_PIN_M2B;
		invert = (S2_RIGHT_INVERT != 0) ? TRUE : FALSE;
	}

	if (duty > MOTOR_DUTY_MAX) {
		duty = MOTOR_DUTY_MAX;
	} else if (duty < -MOTOR_DUTY_MAX) {
		duty = -MOTOR_DUTY_MAX;
	} else {
		;	/* in range */
	}
	if (invert == TRUE) {
		duty = -duty;
	}

	if (duty >= 0) {
		cc = ((UW)duty * (MOTOR_PWM_WRAP + 1U)) / (UW)MOTOR_DUTY_MAX;
		(void)pwm_set_cc(pin_b, 0U);
		(void)pwm_set_cc(pin_a, cc);
	} else {
		cc = ((UW)(-duty) * (MOTOR_PWM_WRAP + 1U)) / (UW)MOTOR_DUTY_MAX;
		(void)pwm_set_cc(pin_a, 0U);
		(void)pwm_set_cc(pin_b, cc);
	}
}

EXPORT void motor_stop_all(void)
{
	motor_set(S2_LEFT, 0);
	motor_set(S2_RIGHT, 0);
}
