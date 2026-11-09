/*
 *	drv_indicators.c
 *	Subsystem 5 -- buttons and buzzer (kernel register access).
 *	The NeoPixel's PIO code is in drv_neopixel_pio.c (Pico SDK side).
 */

#include "drv_indicators.h"

#define BUZZER_SLICE		3U
#define BUZZER_DIV_INT		64U			/* 125 MHz / 64 */
#define BUZZER_CLK_HZ		(125000000UL / BUZZER_DIV_INT)
#define PWM_DIV_REG(s)		(PWM_BASE + ((s) * 0x14UL) + PWM_CHx_DIV)

/* Plain-C entry points of the PIO side (drv_neopixel_pio.c) */
IMPORT int  neopixel_pio_init(unsigned int pin);
IMPORT void neopixel_pio_put(unsigned int grb, unsigned int count);

EXPORT void drv_buttons_init(void)
{
	gpio_set_pin(BTN_START_PIN, GPIO_MODE_IN);
	gpio_set_pin(BTN_STOP_PIN, GPIO_MODE_IN);
	set_w(GPIO(BTN_START_PIN), GPIO_PUE);	/* buttons pull to GND */
	set_w(GPIO(BTN_STOP_PIN), GPIO_PUE);
}

EXPORT BOOL drv_button_raw(UINT pin)
{
	return (gpio_get_val(pin) == 0U) ? TRUE : FALSE;
}

EXPORT void drv_buzzer_init(void)
{
	app_unreset(RESETS_RESET_PWM);
	pwm_set_pin(BUZZER_PIN);
	out_w(PWM_DIV_REG(BUZZER_SLICE), BUZZER_DIV_INT << 4);
	pwm_set_cc(BUZZER_PIN, 0);
	pwm_set_wrap(BUZZER_PIN, 1000U);
	pwm_set_enabled(BUZZER_PIN, TRUE);
}

EXPORT void drv_buzzer_tone(UW hz)
{
	UW	wrap;

	if (hz == 0U) {
		pwm_set_cc(BUZZER_PIN, 0);
		return;
	}
	wrap = (BUZZER_CLK_HZ / hz) - 1U;
	if (wrap > 0xFFFFU) {
		wrap = 0xFFFFU;
	}
	pwm_set_wrap(BUZZER_PIN, wrap);
	pwm_set_cc(BUZZER_PIN, wrap / 2U);	/* 50% duty */
}

EXPORT ER drv_neopixel_init(void)
{
	return (neopixel_pio_init(NEOPIXEL_PIN) == 0) ? E_OK : E_LIMIT;
}

EXPORT void drv_neopixel_set(UW rgb)
{
	UW	r = (rgb >> 16) & 0xFFU;
	UW	g = (rgb >> 8) & 0xFFU;
	UW	b = rgb & 0xFFU;

	neopixel_pio_put((g << 16) | (r << 8) | b, 2U);	/* WS2812 order: GRB */
}
