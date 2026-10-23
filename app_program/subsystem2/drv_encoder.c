/*
 *	drv_encoder.c
 *	Subsystem 2 -- PIO quadrature encoders (through ../sdk_shim), with a
 *	PWM edge-count fallback for WIFI=none builds.
 */

#include "drv_encoder.h"
#include "drv_motor.h"		/* pwm_block_release_reset() */

#if TM_WIFI_CYW43
/* ================================================================== */
#include "../sdk_shim/sdk_pio_shim.h"

EXPORT ER encoder_init(void)
{
	INT	r = (INT)shim_quad_init(S2_PIN_ENC_L_A, S2_PIN_ENC_R_A);

	return (r < 0) ? E_LIMIT : E_OK;
}

EXPORT W encoder_count(S2_SIDE side)
{
	W	c = (W)shim_quad_count((side == S2_LEFT) ? 0 : 1);
	BOOL	inv = (side == S2_LEFT) ? (S2_LEFT_INVERT != 0) : (S2_RIGHT_INVERT != 0);

	return (inv == TRUE) ? -c : c;
}

EXPORT BOOL encoder_is_signed(void)
{
	return TRUE;
}

EXPORT void encoder_report(void)
{
	int	blk = -1;
	int	sl = -1;
	int	sr = -1;

	(void)shim_quad_where(&blk, &sl, &sr);
	tm_printf((UB *)"[S2] PIO quadrature encoders: pio%d, SM %d (left) and %d (right), offset 0\n",
		  (INT)blk, (INT)sl, (INT)sr);
}

#else /* !TM_WIFI_CYW43 */
/* ================================================================== */
/* Fallback: PWM slices 1 and 2 count rising edges on the B pins. */
#define PWM_SLICE_OF(pin)	(((UINT)(pin) >> 1) & 0x07U)
#define PWM_CSR(sl)		(PWM_BASE + PWM_CHx_CSR + ((sl) * 0x14U))
#define PWM_DIV(sl)		(PWM_BASE + PWM_CHx_DIV + ((sl) * 0x14U))
#define PWM_CTR(sl)		(PWM_BASE + PWM_CHx_CTR + ((sl) * 0x14U))
#define PWM_TOP(sl)		(PWM_BASE + PWM_CHx_TOP + ((sl) * 0x14U))

LOCAL UH	enc_hw[2];
LOCAL W		enc_total[2];

LOCAL void enc_slice(UINT pin_b)
{
	UINT	sl = PWM_SLICE_OF(pin_b);

	(void)pwm_set_pin(pin_b);
	set_w(GPIO(pin_b), GPIO_PUE);
	out_w(PWM_CSR(sl), 0U);
	out_w(PWM_DIV(sl), 1U << 4);
	out_w(PWM_TOP(sl), 0xFFFFU);
	out_w(PWM_CTR(sl), 0U);
	out_w(PWM_CSR(sl), (2U << 4) | PWM_CH_CSR_EN);
}

EXPORT ER encoder_init(void)
{
	pwm_block_release_reset();
	enc_slice(S2_PIN_ENC_L_B);
	enc_slice(S2_PIN_ENC_R_B);
	return E_OK;
}

EXPORT W encoder_count(S2_SIDE side)
{
	UINT	pin = (side == S2_LEFT) ? S2_PIN_ENC_L_B : S2_PIN_ENC_R_B;
	UH	now = (UH)(in_w(PWM_CTR(PWM_SLICE_OF(pin))) & 0xFFFFU);

	enc_total[side] += (W)(UH)(now - enc_hw[side]);
	enc_hw[side] = now;
	return enc_total[side];
}

EXPORT BOOL encoder_is_signed(void)
{
	return FALSE;
}

EXPORT void encoder_report(void)
{
	tm_printf((UB *)"[S2] WIFI=none build: PWM edge counters (no direction)\n");
}
#endif
