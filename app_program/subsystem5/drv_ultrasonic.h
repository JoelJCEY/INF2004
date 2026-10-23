/*
 *	drv_ultrasonic.h
 *	Subsystem 5 -- HC-SR04+ (3.3 V version) with an ECHO edge interrupt
 *	(COMPLETE VERSION).  Call only from scan_task.
 *
 *	The ECHO ISR is the only application interrupt.  It is the ONLY
 *	writer of one aligned 32-bit result word (Section 10):
 *	  bits  0-12  echo duration in us (0-8191; 6,000 us is about 1 m)
 *	  bit  13     valid: a full pulse was timed
 *	  bits 16-23  count of rising edges seen (wraps)
 *	  bits 24-31  count of completed pulses (wraps)
 *	scan_task only reads it.  On every falling edge the ISR wakes
 *	scan_task with tk_wup_tsk().
 */

#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include "subsystem5.h"

#define US_WORD_DUR_MASK	0x1FFFU
#define US_WORD_VALID		(1U << 13)
#define US_WORD_RISE_SHIFT	16
#define US_WORD_DONE_SHIFT	24

IMPORT ER		us_init(void);
IMPORT UW		us_read_word(void);	/* one aligned load */
/* Trigger one ping and sleep until the echo ends (or ~20-30 ms). */
IMPORT S5_PING_RESULT	us_ping(UW *p_echo_us, UW *p_mm);

#endif /* DRV_ULTRASONIC_H */
