/*
 *	drv_status.h
 *	Subsystem 5 -- ctrl_task's own outputs: status NeoPixel (GP18, PIO
 *	WS2812), buzzer (GP22, PWM slice 3A) and the RP2040 hardware watchdog.
 *	Call only from ctrl_task.
 */

#ifndef DRV_STATUS_H
#define DRV_STATUS_H

#include "subsystem5.h"

IMPORT void	status_init(void);
IMPORT void	status_show_state(VEHICLE_STATE s);	/* one colour per state */
IMPORT void	status_beep(UW ms);			/* non-blocking */
IMPORT void	status_tick(void);			/* call every loop */

/* Watchdog: started only if the watchdog tick is running.  Returns TRUE
 * if it is now armed. */
IMPORT BOOL	watchdog_start(UW timeout_ms);
IMPORT void	watchdog_feed(void);

#endif /* DRV_STATUS_H */
