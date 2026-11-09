/*
 *	drv_ultrasonic.h
 *	Subsystem 5 -- HC-SR04+ (3.3 V version) on Grove 1: TRIG GP0, ECHO GP1.
 *
 *	Only the ECHO pin uses an interrupt (design 4).  us_echo_isr
 *	timestamps both edges, publishes ONE 32-bit result word
 *	(us_echo_word, layout in subsystem5.h) and wakes scan_task with
 *	tk_wup_tsk.  Run the module at 3.3 V only: at 5 V its ECHO output
 *	would damage GP1.  Unplug it before a CRASH_REPORT=1 build.
 */

#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include "subsystem5.h"

#define US_FAR_MM		1000U	/* "nothing within ~1 m" */

typedef enum {
	US_OK		= 0,	/* *mm valid                          */
	US_FAR		= 1,	/* echo longer than the 6 ms timeout   */
	US_NO_ECHO	= 2	/* ECHO never rose: a fault if 3 in a row */
} US_RESULT;

IMPORT ER        drv_us_init(void);
/* One ping, pulse timed by PWM slice 0 hardware (31 Oct).  Busy-waits
   at most ~8 ms.  Call from the task that owns GP0/GP1 only. */
IMPORT US_RESULT drv_us_ping(UW *mm);
/* Test build: the same ping, timed both by hardware and the old way
   (polling, affected by pre-emption).  poll_us 0 = polling missed it. */
IMPORT US_RESULT drv_us_ping_both(UW *hw_us, UW *poll_us);
/* Convert an echo duration to mm (speed of sound 343 m/s) */
IMPORT UW        drv_us_dur_to_mm(UW dur_us);

#endif /* DRV_ULTRASONIC_H */
