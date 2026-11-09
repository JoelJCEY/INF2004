/*
 *	drv_ultrasonic.c
 *	Subsystem 5 -- HC-SR04+ driver (TRIG GP0, ECHO GP1).
 *
 *	31 Oct: the echo is now timed by HARDWARE.  GP1 is the B pin of
 *	PWM slice 0.  In "level" mode that slice counts only while its B pin
 *	is high, so with a 1 MHz count it holds the ECHO pulse width in µs,
 *	whatever the CPU is doing.
 *
 *	Why: the old version timed the pulse by polling GP1 from scan_task
 *	(priority 13).  Wi-Fi (4), USB (3), ctrl (10), motion (11) and line
 *	(12) all pre-empt it.  A pre-emption while ECHO is high makes the
 *	pulse look longer, so the bracket echo (~3 cm) could read as 7.5 cm
 *	or 24 cm: a phantom obstacle inside the stop distance.  The counter
 *	stops by itself when ECHO falls, so reading it late is harmless.
 *
 *	us_echo_word: last raw result for diagnostics, one aligned 32-bit
 *	store, written by the pinging task only.
 */

#include "drv_ultrasonic.h"

EXPORT volatile UW us_echo_word;

#define ECHO_START_MAX_US	2000U	/* echo must start within 2 ms */

/* PWM slice 0 as a gated 1 MHz counter on GP1 (channel B) */
#define US_SLICE		0UL
#define US_REG(off)		(PWM_BASE + (US_SLICE * 0x14UL) + (off))
#define US_CSR			US_REG(PWM_CHx_CSR)
#define US_DIV			US_REG(PWM_CHx_DIV)
#define US_CTR			US_REG(PWM_CHx_CTR)
#define US_TOP			US_REG(PWM_CHx_TOP)
#define US_CSR_DIVMODE_LEVEL	(1UL << 4)	/* count while B is high */
#define US_DIV_INT		125UL		/* 125 MHz / 125 = 1 MHz */

EXPORT ER drv_us_init(void)
{
	/* TRIG: plain SIO output (GP0 is UART0 TX only in CRASH_REPORT builds) */
	gpio_set_pin(US_TRIG_PIN, GPIO_MODE_OUT);
	gpio_set_val(US_TRIG_PIN, 0);

	/* Gated counter.  Slice 0 is free (TEAM.md: slices 3-6 are used;
	   GP16/GP17 share slice 0 but run as I2C, not PWM). */
	app_unreset(RESETS_RESET_PWM);
	out_w(US_CSR, 0);
	out_w(US_DIV, US_DIV_INT << 4);		/* 8.4 fixed point */
	out_w(US_TOP, 0xFFFFUL);
	out_w(US_CTR, 0);
	out_w(US_CSR, US_CSR_DIVMODE_LEVEL | PWM_CH_CSR_EN);

	/* ECHO: input to the PWM block, output driver off, pull-down so a
	   missing module reads "no echo".  GPIO_IN still shows the pin. */
	set_w(GPIO(US_ECHO_PIN), GPIO_OD | GPIO_IE | GPIO_PDE);
	clr_w(GPIO(US_ECHO_PIN), GPIO_PUE);
	out_w(GPIO_CTRL(US_ECHO_PIN), GPIO_CTRL_FUNCSEL_PWM);

	us_echo_word = 0;
	return E_OK;
}

EXPORT UW drv_us_dur_to_mm(UW dur_us)
{
	/* mm = µs * 0.343 / 2 */
	return (dur_us * 343UL) / 2000UL;
}

/* One ping.  hw_us: pulse width from the hardware counter (the real
   value).  poll_us (may be NULL): the same pulse timed the old way, by
   polling, for the test build to compare. */
LOCAL US_RESULT ping(UW *hw_us, UW *poll_us)
{
	UW	t0, tr = 0, dur = 0;
	BOOL	saw_rise = FALSE;

	/* An HC-SR04 that heard nothing keeps ECHO high for a long time; a
	   trigger then is ignored: report "far" and do not trigger */
	if (gpio_get_val(US_ECHO_PIN) != 0U) {
		return US_FAR;
	}

	out_w(US_CTR, 0);			/* ECHO is low: counter idle */
	gpio_set_val(US_TRIG_PIN, 1);
	app_delay_us(10);			/* 10 µs trigger pulse */
	gpio_set_val(US_TRIG_PIN, 0);
	t0 = app_time_us();

	/* Wait for the rise.  If this task was pre-empted, a short pulse may
	   have come and gone already: the counter then holds it. */
	while (gpio_get_val(US_ECHO_PIN) == 0U) {
		if (in_w(US_CTR) != 0UL) {
			break;			/* pulse happened while away */
		}
		if ((app_time_us() - t0) > ECHO_START_MAX_US) {
			return US_NO_ECHO;	/* ECHO never rose */
		}
	}
	if (gpio_get_val(US_ECHO_PIN) != 0U) {
		saw_rise = TRUE;
		tr = app_time_us();
	}
	while (gpio_get_val(US_ECHO_PIN) != 0U) {
		if (in_w(US_CTR) > ECHO_TIMEOUT_US) {
			return US_FAR;		/* nothing within ~1 m: normal */
		}
	}
	if (saw_rise) {
		dur = app_time_us() - tr;	/* old way: includes pre-emption */
	}
	*hw_us = in_w(US_CTR) & 0xFFFFUL;	/* new way: exact */
	if (*hw_us > ECHO_TIMEOUT_US) {
		return US_FAR;
	}
	if (poll_us != NULL) {
		*poll_us = dur;			/* 0 = pulse missed by polling */
	}
	return US_OK;
}

EXPORT US_RESULT drv_us_ping(UW *mm)
{
	UW		hw = 0;
	US_RESULT	r = ping(&hw, NULL);

	if (r == US_OK) {
		us_echo_word = ECHO_VALID | (hw & ECHO_DUR_MASK);	/* diagnostics */
		*mm = drv_us_dur_to_mm(hw);
	}
	return r;
}

EXPORT US_RESULT drv_us_ping_both(UW *hw_us, UW *poll_us)
{
	return ping(hw_us, poll_us);
}
