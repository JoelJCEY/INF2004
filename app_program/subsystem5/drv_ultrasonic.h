/*
 *	drv_ultrasonic.h
 *	Subsystem 5 -- HC-SR04+ (3.3 V version), polled bring-up driver.
 *	Call only from scan_task.
 */

#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include "subsystem5.h"

IMPORT UW		us_now(void);		/* 1 MHz hardware timer */
IMPORT ER		us_init(void);
/* One ping.  On S5_PING_OK, *p_echo_us and *p_mm are set. */
IMPORT S5_PING_RESULT	us_ping(UW *p_echo_us, UW *p_mm);

#endif /* DRV_ULTRASONIC_H */
