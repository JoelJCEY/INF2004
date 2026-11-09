/*
 *	app_main.c
 *	START HERE.  usermain() below is the program's entry point.
 *
 *	IS11 Autonomous Robotic Car: six tasks on micro T-Kernel 3.0,
 *	single-core build (SMP=0, design 1.0).
 *
 *	  reset -> kernel init -> initial task -> usermain()
 *
 *	usermain() runs at priority 1 (highest).  On the single-core build
 *	nothing created here runs until usermain() blocks, which gives a
 *	fixed boot order:
 *	  1. motion (subsystem 2) initialises FIRST, because the PIO
 *	     quadrature program must load at offset 0 of a PIO block before
 *	     the CYW43 Wi-Fi driver (priority 4 kernel task) claims PIO
 *	     memory (assumption A2, resource matrix 11.1).
 *	  2. every other subsystem creates its kernel objects (_init)
 *	  3. every subsystem starts its tasks (_start)
 *	and then the initial task sleeps forever.
 *
 *	Inter-subsystem interfaces: TEAM.md section 5 (IF-01 .. IF-14).
 */

#include "app.h"

/* ---------------------------------------------------------------------
 *  RP2040 registers used by the shared helpers
 * --------------------------------------------------------------------- */
#define WDT_BASE		0x40058000UL
#define WDT_CTRL		(WDT_BASE + 0x00UL)
#define WDT_LOAD		(WDT_BASE + 0x04UL)
#define WDT_REASON		(WDT_BASE + 0x08UL)
#define WDT_TICK		(WDT_BASE + 0x2CUL)
#define WDT_CTRL_ENABLE		(1UL << 30)
#define WDT_CTRL_PAUSE_ALL	(7UL << 24)	/* pause while debugging */
#define WDT_REASON_TIMER	(1UL << 0)
#define WDT_TICK_ENABLE		(1UL << 9)
#define WDT_TICK_CYCLES_12MHZ	12UL		/* clk_ref = 12 MHz XOSC */

#define PSM_WDSEL		(0x40010000UL + 0x08UL)
#define PSM_WDSEL_ALL_BUT_OSC	0x0001FFFCUL	/* keep ROSC and XOSC */

LOCAL BOOL	wdt_rebooted;		/* written once, before any task runs */
LOCAL UW	wdt_load;		/* written once by app_watchdog_start */

EXPORT UW app_time_us(void)
{
	return in_w(TIMER_TIMERAWL);
}

EXPORT UW app_time_ms(void)
{
	SYSTIM	t;

	(void)tk_get_otm(&t);	/* time since power-on (not the calendar clock) */
	return t.lo;
}

EXPORT void app_delay_us(UW us)
{
	UW	start = app_time_us();

	while ((app_time_us() - start) < us) {
		;	/* busy-wait: only for short pulses (TRIG) */
	}
}

EXPORT void app_unreset(UW mask)
{
	UINT	intsts;

	DI(intsts);
	clr_w(RESETS_RESET, mask);
	EI(intsts);
	while ((in_w(RESETS_RESET_DONE) & mask) != mask) {
		;
	}
}

EXPORT BOOL app_watchdog_caused_reboot(void)
{
	return wdt_rebooted;
}

EXPORT void app_watchdog_start(UW timeout_ms)
{
	/* RP2040-E1: the counter decrements twice per tick, so load 2x */
	UW	load = timeout_ms * 1000UL * 2UL;

	if (load > 0x00FFFFFFUL) {
		load = 0x00FFFFFFUL;
	}
	wdt_load = load;
	out_w(PSM_WDSEL, PSM_WDSEL_ALL_BUT_OSC);	/* reset all but clocks */
	out_w(WDT_LOAD, load);
	out_w(WDT_CTRL, WDT_CTRL_ENABLE | WDT_CTRL_PAUSE_ALL);
}

EXPORT void app_watchdog_feed(void)
{
	/* LOAD is write-only: re-write the saved value to restart the count */
	if (wdt_load != 0UL) {
		out_w(WDT_LOAD, wdt_load);
	}
}

/*
 * Create and start one task (declared in app.h for every subsystem).
 *   prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or 0 for "any core".
 */
EXPORT ID app_start_task_stk(FP entry, PRI pri, UW prc, SZ stksz,
			     const char *name)
{
	T_CTSK	ctsk = {0};
	ID	tid;
	ER	er;

	ctsk.task    = entry;
	ctsk.itskpri = pri;
	ctsk.stksz   = stksz;
	ctsk.tskatr  = TA_HLNG | TA_RNG3;
#if TK_SUPPORT_SMP
	if (prc != 0) {
		ctsk.tskatr |= TA_ASSPRC;
		ctsk.assprc  = prc;
	}
#else
	(void)prc;
#endif
	tid = tk_cre_tsk(&ctsk);
	if (tid < E_OK) {
		CHECK(tid, name);
		return tid;
	}
	er = tk_sta_tsk(tid, 0);
	CHECK(er, name);
	return (er < E_OK) ? er : tid;
}

EXPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name)
{
	return app_start_task_stk(entry, pri, prc, TASK_STACK_SIZE, name);
}

#if APP_TEST
IMPORT void sensor_bench(void);
#endif

EXPORT INT usermain(void)
{
	ER	er_motion;

	/* The 1 MHz TIMER counts watchdog ticks: make sure they run. */
	if ((in_w(WDT_TICK) & WDT_TICK_ENABLE) == 0UL) {
		out_w(WDT_TICK, WDT_TICK_CYCLES_12MHZ | WDT_TICK_ENABLE);
	}
	/* Read the watchdog reset reason once (design 5.1, last row) */
	wdt_rebooted = ((in_w(WDT_REASON) & WDT_REASON_TIMER) != 0UL)
		       ? TRUE : FALSE;

	/* Phase 0: motion first, before anything can block (PIO offset 0) */
	er_motion = subsystem2_init();

	app_log((UB*)"\n=== IS11 autonomous car, micro T-Kernel on Pico W ===\n");
#if TK_SUPPORT_SMP
	app_log((UB*)"WARNING: design assumes SMP=0 (single-core build)\n");
#endif
	if (wdt_rebooted) {
		app_log((UB*)"boot: WATCHDOG RESET detected\n");
	}
	CHECK(er_motion, "subsystem2_init");

	/* Phase 1: every other subsystem creates its kernel objects.
	   Subsystem 5 owns the System Event Group and most queues, so it
	   goes next; the others only create objects they receive on. */
	CHECK(subsystem5_init(), "subsystem5_init");
	CHECK(subsystem1_init(), "subsystem1_init");
	CHECK(subsystem3_init(), "subsystem3_init");
	CHECK(subsystem4_init(), "subsystem4_init");

	/* Phase 2: start tasks, highest priority first */
	CHECK(subsystem5_start(), "subsystem5_start");	/* ctrl + scan */
	CHECK(subsystem2_start(), "subsystem2_start");	/* motion      */
	CHECK(subsystem3_start(), "subsystem3_start");	/* line        */
	CHECK(subsystem4_start(), "subsystem4_start");	/* imu         */
	CHECK(subsystem1_start(), "subsystem1_start");	/* comms       */

	app_log((UB*)"all subsystems started\n\n");

#if APP_TEST
	app_log((UB*)"[TEST] === test run start ===\n");
	subsystem1_test();
	subsystem2_test();
	subsystem3_test();
	subsystem4_test();
	subsystem5_test();
	app_log((UB*)"[TEST] === test run end ===\n");
	sensor_bench();		/* guided hand tests of every sensor */
#endif

#if APP_US_DIAG
	us_diag();		/* never returns */
#endif
	/* Sleep forever.  Returning from usermain() would shut down. */
	tk_slp_tsk(TMO_FEVR);
	return 0;
}
