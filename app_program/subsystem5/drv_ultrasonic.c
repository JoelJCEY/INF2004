/*
 *	drv_ultrasonic.c
 *	Subsystem 5 -- HC-SR04+ polled with the 1 MHz TIMER (TIMERAWL).
 *
 *	The task busy-waits for at most ~9 ms per ping.  Higher-priority
 *	tasks can still preempt it, which would stretch a measurement; step 2
 *	replaces the waiting with an ECHO edge interrupt that timestamps both
 *	edges and wakes scan_task with tk_wup_tsk().
 */

#include "drv_ultrasonic.h"

EXPORT UW us_now(void)
{
	return in_w(TIMER_TIMERAWL);
}

LOCAL void us_busy_wait(UW us)
{
	UW	t0 = us_now();

	while ((us_now() - t0) < us) {
		;
	}
}

EXPORT ER us_init(void)
{
	(void)gpio_set_pin(S5_PIN_TRIG, GPIO_MODE_OUT);
	(void)gpio_set_val(S5_PIN_TRIG, 0U);
	(void)gpio_set_pin(S5_PIN_ECHO, GPIO_MODE_IN);
	clr_w(GPIO(S5_PIN_ECHO), GPIO_PUE);	/* sensor drives the line */
	set_w(GPIO(S5_PIN_ECHO), GPIO_PDE);	/* reads 0 if unplugged */
	return E_OK;
}

EXPORT S5_PING_RESULT us_ping(UW *p_echo_us, UW *p_mm)
{
	UW	t_start;
	UW	t_rise;
	UW	width;

	if (gpio_get_val(S5_PIN_ECHO) != 0U) {
		return S5_PING_BUSY;
	}

	(void)gpio_set_val(S5_PIN_TRIG, 1U);
	us_busy_wait(S5_TRIG_PULSE_US);
	(void)gpio_set_val(S5_PIN_TRIG, 0U);

	/* Wait for ECHO to rise (the sensor first sends its 40 kHz burst) */
	t_start = us_now();
	while (gpio_get_val(S5_PIN_ECHO) == 0U) {
		if ((us_now() - t_start) > S5_RISE_TMO_US) {
			return S5_PING_NO_ECHO;
		}
	}

	/* Measure how long ECHO stays high */
	t_rise = us_now();
	while (gpio_get_val(S5_PIN_ECHO) != 0U) {
		if ((us_now() - t_rise) > S5_ECHO_TMO_US) {
			return S5_PING_OUT_OF_RANGE;
		}
	}
	width = us_now() - t_rise;

	/* Sound at ~20 C: 343 m/s = 0.343 mm/us, halved for the round trip */
	*p_echo_us = width;
	*p_mm = (width * 343U) / 2000U;
	return S5_PING_OK;
}
