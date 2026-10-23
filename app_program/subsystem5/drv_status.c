/*
 *	drv_status.c
 *	Subsystem 5 -- NeoPixel, buzzer and watchdog for ctrl_task.
 */

#include "drv_status.h"

#if TM_WIFI_CYW43
#include "../sdk_shim/sdk_pio_shim.h"
#endif

/* ---- buzzer: PWM slice 3 channel A, 2 kHz square wave ---- */
#define BUZ_SLICE		3U
#define BUZ_TOP			62499U	/* 125 MHz / 62,500 = 2 kHz */
#define REG_ALIAS_CLR		0x3000U

/* ---- watchdog (RP2040 datasheet 4.7) ---- */
#define WDG_BASE		0x40058000U
#define WDG_CTRL		(WDG_BASE + 0x00U)
#define WDG_LOAD		(WDG_BASE + 0x04U)
#define WDG_TICK		(WDG_BASE + 0x2CU)
#define WDG_CTRL_ENABLE		(1U << 30)
#define WDG_CTRL_PAUSE_ALL	(7U << 24)	/* pause while debugging */
#define WDG_TICK_RUNNING	(1U << 10)
#define PSM_WDSEL		(0x40010000U + 0x08U)
#define PSM_WDSEL_ALL_BUT_OSC	(0x0001FFFFU & ~0x3U)

LOCAL UW	beep_until = 0U;
LOCAL BOOL	beeping = FALSE;
LOCAL UW	wdg_load = 0U;
LOCAL BOOL	wdg_armed = FALSE;

LOCAL void pwm_release_reset(void)
{
	if ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0U) {
		out_w(RESETS_RESET + REG_ALIAS_CLR, RESETS_RESET_PWM);
		while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0U) {
			;
		}
	}
}

EXPORT void status_init(void)
{
	pwm_release_reset();
	(void)pwm_set_pin(S5_PIN_BUZZER);
	(void)pwm_set_wrap(S5_PIN_BUZZER, BUZ_TOP);
	(void)pwm_set_cc(S5_PIN_BUZZER, 0U);
	(void)pwm_set_enabled(S5_PIN_BUZZER, TRUE);
#if TM_WIFI_CYW43
	if (shim_ws2812_init(S5_PIN_NEOPIXEL, 125000000U) < 0) {
		tm_printf((UB *)"[CTRL] NeoPixel: no free PIO space\n");
	}
#endif
}

EXPORT void status_show_state(VEHICLE_STATE s)
{
#if TM_WIFI_CYW43
	/* GRB, dim (max 0x20) so it does not dazzle */
	static const UW grb[VS_NUM_STATES] = {
		0x101000U,	/* INITIALISING    yellow */
		0x000020U,	/* READY           blue */
		0x200000U,	/* LINE FOLLOWING  green */
		0x102000U,	/* BARCODE         cyan-ish */
		0x101010U,	/* HUMP            white */
		0x082000U,	/* OBSTACLE        orange */
		0x200020U,	/* MISSION COMPLETE */
		0x002000U,	/* STOPPED         red */
		0x002000U,	/* FAULT STOP      red (blinks, see status_tick) */
		0x200010U	/* LINE RECOVERY */
	};
	shim_ws2812_put_grb(grb[s]);
#else
	(void)s;
#endif
}

EXPORT void status_beep(UW ms)
{
	(void)pwm_set_cc(S5_PIN_BUZZER, BUZ_TOP / 2U);
	beeping = TRUE;
	beep_until = s5_now_ms() + ms;
}

EXPORT void status_tick(void)
{
	if ((beeping == TRUE) && ((W)(s5_now_ms() - beep_until) >= 0)) {
		(void)pwm_set_cc(S5_PIN_BUZZER, 0U);
		beeping = FALSE;
	}
}

EXPORT BOOL watchdog_start(UW timeout_ms)
{
	if ((in_w(WDG_TICK) & WDG_TICK_RUNNING) == 0U) {
		return FALSE;	/* no 1 us tick: the watchdog would never fire */
	}
	/* RP2040-E1: the counter decrements twice per tick, so load 2 x us */
	wdg_load = timeout_ms * 1000U * 2U;
	if (wdg_load > 0xFFFFFFU) {
		wdg_load = 0xFFFFFFU;
	}
	clr_w(WDG_CTRL, WDG_CTRL_ENABLE);
	set_w(PSM_WDSEL, PSM_WDSEL_ALL_BUT_OSC);
	set_w(WDG_CTRL, WDG_CTRL_PAUSE_ALL);
	out_w(WDG_LOAD, wdg_load);
	set_w(WDG_CTRL, WDG_CTRL_ENABLE);
	wdg_armed = TRUE;
	return TRUE;
}

EXPORT void watchdog_feed(void)
{
	if (wdg_armed == TRUE) {
		out_w(WDG_LOAD, wdg_load);
	}
}
