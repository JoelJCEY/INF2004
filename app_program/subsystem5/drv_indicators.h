/*
 *	drv_indicators.h
 *	Subsystem 5 -- ctrl_task's on-board devices: START/STOP buttons
 *	(GP20/GP21), status NeoPixels (GP18, PIO WS2812) and piezo buzzer
 *	(GP22, PWM slice 3A).  Owner: ctrl_task only.
 */

#ifndef DRV_INDICATORS_H
#define DRV_INDICATORS_H

#include "subsystem5.h"

/* Buttons: polled every 50 ms, pressed after 2 equal samples */
IMPORT void drv_buttons_init(void);
IMPORT BOOL drv_button_raw(UINT pin);		/* TRUE = pressed (low) */

/* Buzzer: frequency in Hz, 0 = off */
IMPORT void drv_buzzer_init(void);
IMPORT void drv_buzzer_tone(UW hz);

/* NeoPixel colour 0xRRGGBB on both on-board pixels */
IMPORT ER   drv_neopixel_init(void);
IMPORT void drv_neopixel_set(UW rgb);

#endif /* DRV_INDICATORS_H */
