/*
 *	drv_encoder.c
 *	Subsystem 2 -- edge counting with PWM slices 1 (GP3) and 2 (GP5).
 */

#include "drv_encoder.h"
#include "drv_motor.h"		/* pwm_block_release_reset() */

#define PWM_SLICE_OF(pin)	(((UINT)(pin) >> 1) & 0x07U)
#define PWM_CSR(sl)		(PWM_BASE + PWM_CHx_CSR + ((sl) * 0x14U))
#define PWM_DIV(sl)		(PWM_BASE + PWM_CHx_DIV + ((sl) * 0x14U))
#define PWM_CTR(sl)		(PWM_BASE + PWM_CHx_CTR + ((sl) * 0x14U))
#define PWM_TOP(sl)		(PWM_BASE + PWM_CHx_TOP + ((sl) * 0x14U))

#define PWM_CSR_DIVMODE_RISE	(2U << 4)	/* count rising edges on B */
#define PWM_DIV_ONE		(1U << 4)	/* integer divider = 1 */

LOCAL UH	enc_last_hw[2];		/* last 16-bit hardware reading */
LOCAL UW	enc_total[2];		/* extended 32-bit count */

LOCAL void encoder_slice_init(UINT pin_b)
{
	UINT	slice = PWM_SLICE_OF(pin_b);

	(void)pwm_set_pin(pin_b);		/* PWM function, input enabled */
	set_w(GPIO(pin_b), GPIO_PUE);		/* pull-up for open-collector outputs */
	out_w(PWM_CSR(slice), 0U);		/* stop, clear mode */
	out_w(PWM_DIV(slice), PWM_DIV_ONE);
	out_w(PWM_TOP(slice), 0xFFFFU);
	out_w(PWM_CTR(slice), 0U);
	out_w(PWM_CSR(slice), PWM_CSR_DIVMODE_RISE | PWM_CH_CSR_EN);
}

EXPORT ER encoder_init(void)
{
	pwm_block_release_reset();
	encoder_slice_init(S2_PIN_ENC_L_B);
	encoder_slice_init(S2_PIN_ENC_R_B);

	/* A outputs as plain inputs for now (PIO takes them in step 2) */
	(void)gpio_set_pin(S2_PIN_ENC_L_A, GPIO_MODE_IN);
	(void)gpio_set_pin(S2_PIN_ENC_R_A, GPIO_MODE_IN);
	set_w(GPIO(S2_PIN_ENC_L_A), GPIO_PUE);
	set_w(GPIO(S2_PIN_ENC_R_A), GPIO_PUE);

	enc_last_hw[S2_LEFT] = 0U;
	enc_last_hw[S2_RIGHT] = 0U;
	enc_total[S2_LEFT] = 0U;
	enc_total[S2_RIGHT] = 0U;
	return E_OK;
}

EXPORT UW encoder_count(S2_SIDE side)
{
	UINT	pin = (side == S2_LEFT) ? S2_PIN_ENC_L_B : S2_PIN_ENC_R_B;
	UH	now = (UH)(in_w(PWM_CTR(PWM_SLICE_OF(pin))) & 0xFFFFU);
	UH	diff = (UH)(now - enc_last_hw[side]);	/* wraps correctly */

	enc_last_hw[side] = now;
	enc_total[side] += (UW)diff;
	return enc_total[side];
}
