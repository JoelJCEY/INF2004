/*
 *	drv_ultrasonic.c
 *	Subsystem 5 -- HC-SR04+ with the ECHO edge interrupt on GP7.
 *
 *	GPIO interrupts on the RP2040 all arrive on IO_IRQ_BANK0 (IRQ 13).
 *	The template's Wi-Fi port polls and never uses it, so it is free.
 *	Timestamps come from the 1 MHz TIMER (TIMERAWL).
 */

#include "drv_ultrasonic.h"

#define INTNO_IO_BANK0		13
#define ECHO_INTPRI		2

/* IO_BANK0 interrupt registers for GPIO 0-7 (word 0). */
#define IO_INTR0		(IO_BANK0_BASE + 0x0F0U)	/* raw, edges W1C */
#define IO_PROC0_INTE0		(IO_BANK0_BASE + 0x100U)
#define IO_PROC0_INTS0		(IO_BANK0_BASE + 0x120U)
#define ECHO_EDGE_LOW		(1U << ((S5_PIN_ECHO * 4U) + 2U))
#define ECHO_EDGE_HIGH		(1U << ((S5_PIN_ECHO * 4U) + 3U))

#if S5_PIN_ECHO > 7
#error "ECHO pin must be GP0-GP7 for the register word used here"
#endif

LOCAL volatile UW	us_word = 0U;	/* written ONLY by the ISR */
LOCAL UW		us_t_rise = 0U;	/* ISR private */

IMPORT volatile UW	s5_echo_isr_count;

LOCAL void us_echo_isr(UINT intno)
{
	UW	ints = in_w(IO_PROC0_INTS0);
	UW	now = in_w(TIMER_TIMERAWL);
	UW	w = us_word;

	if ((ints & ECHO_EDGE_HIGH) != 0U) {
		out_w(IO_INTR0, ECHO_EDGE_HIGH);	/* W1C */
		us_t_rise = now;
		w = (w & ~(0xFFU << US_WORD_RISE_SHIFT)) |
		    ((((w >> US_WORD_RISE_SHIFT) + 1U) & 0xFFU) << US_WORD_RISE_SHIFT);
	}
	if ((ints & ECHO_EDGE_LOW) != 0U) {
		UW	dur = now - us_t_rise;

		out_w(IO_INTR0, ECHO_EDGE_LOW);		/* W1C */
		if (dur > US_WORD_DUR_MASK) {
			dur = US_WORD_DUR_MASK;
		}
		w = (w & ~(0xFFU << US_WORD_DONE_SHIFT)) |
		    ((((w >> US_WORD_DONE_SHIFT) + 1U) & 0xFFU) << US_WORD_DONE_SHIFT);
		w = (w & ~(US_WORD_DUR_MASK | US_WORD_VALID)) | dur | US_WORD_VALID;
		us_word = w;				/* single aligned store */
		s5_echo_isr_count++;
		(void)tk_wup_tsk(tid_scan);
	} else {
		us_word = w;
	}
	ClearInt(intno);
}

EXPORT ER us_init(void)
{
	T_DINT	dint;
	ER	er;

	(void)gpio_set_pin(S5_PIN_TRIG, GPIO_MODE_OUT);
	(void)gpio_set_val(S5_PIN_TRIG, 0U);
	(void)gpio_set_pin(S5_PIN_ECHO, GPIO_MODE_IN);
	clr_w(GPIO(S5_PIN_ECHO), GPIO_PUE);
	set_w(GPIO(S5_PIN_ECHO), GPIO_PDE);

	dint.intatr = TA_HLNG;
	dint.inthdr = (FP)us_echo_isr;
	er = tk_def_int(INTNO_IO_BANK0, &dint);
	if (er < E_OK) {
		return er;
	}
	out_w(IO_INTR0, ECHO_EDGE_LOW | ECHO_EDGE_HIGH);
	set_w(IO_PROC0_INTE0, ECHO_EDGE_LOW | ECHO_EDGE_HIGH);
	ClearInt(INTNO_IO_BANK0);
	EnableInt(INTNO_IO_BANK0, ECHO_INTPRI);
	return E_OK;
}

EXPORT UW us_read_word(void)
{
	return us_word;
}

EXPORT S5_PING_RESULT us_ping(UW *p_echo_us, UW *p_mm)
{
	UW	before = us_word;
	UW	after;
	UW	t0;

	if (gpio_get_val(S5_PIN_ECHO) != 0U) {
		return S5_PING_BUSY;	/* still high from the last ping */
	}
	(void)tk_can_wup(TSK_SELF);	/* forget old wake-ups */

	(void)gpio_set_val(S5_PIN_TRIG, 1U);
	t0 = in_w(TIMER_TIMERAWL);
	while ((in_w(TIMER_TIMERAWL) - t0) < S5_TRIG_PULSE_US) {
		;
	}
	(void)gpio_set_val(S5_PIN_TRIG, 0U);

	/* Sleep until the ISR sees the falling edge.  With the 10 ms kernel
	 * tick this waits 20-30 ms at most; an echo that long already means
	 * "nothing within 1 m". */
	(void)tk_slp_tsk(20);
	after = us_word;

	if (((after ^ before) >> US_WORD_DONE_SHIFT) != 0U) {
		UW	dur = after & US_WORD_DUR_MASK;
		if (dur > S5_ECHO_TMO_US) {
			return S5_PING_OUT_OF_RANGE;
		}
		*p_echo_us = dur;
		*p_mm = (dur * 343U) / 2000U;
		return S5_PING_OK;
	}
	if ((((after ^ before) >> US_WORD_RISE_SHIFT) & 0xFFU) != 0U) {
		return S5_PING_OUT_OF_RANGE;	/* rose, still high: nothing near */
	}
	return S5_PING_NO_ECHO;			/* never rose: sensor problem */
}
