/*
 *	app.h
 *	Definitions shared by the whole team (IS11).  Changed only by team
 *	agreement.  Design reference: IS11 Design Review (Week 6 + Tribunal 1).
 *
 *	Subsystem map (one folder per buddy, TEAM.md section 1):
 *	  subsystem1/  Buddy 1  Communication          comms_task
 *	  subsystem2/  Buddy 2  Motion Control         motion_task
 *	  subsystem3/  Buddy 3  Line & Barcode Sensing line_task
 *	  subsystem4/  Buddy 4  Terrain Monitoring     imu_task
 *	  subsystem5/  Buddy 5  Obstacle Scanning      scan_task
 *	                        Vehicle Controller     ctrl_task (integration)
 *
 *	Anything that belongs to ONE subsystem goes in that subsystem's own
 *	header instead (app_program/subsystemN/subsystemN.h).
 */

#ifndef APP_H
#define APP_H

#include <tk/tkernel.h>		/* micro T-Kernel API (tk_xxx_yyy) */
#include <tm/tmonitor.h>	/* T-Monitor API: tm_printf() console output */
#include "app_log.h"		/* app_log(): console + dashboard log */
#include <bsp/libbsp.h>		/* Board helpers: gpio_set_pin/val, pwm_... */

/* The PIO encoder, NeoPixel and ADC/DMA drivers use the Pico SDK hardware
   files that the template only links into WIFI=cyw43 builds (the default).
   Team build: make SMP=0 NET=lwip MQTT=1 -j8  (design 1.0: SMP=0). */
#if !TM_WIFI_CYW43
#error "IS11 firmware needs WIFI=cyw43 (the default): the PIO and DMA drivers come from the Pico SDK"
#endif

/* ---------------------------------------------------------------------
 *  Compile-time check (Barr-C: static asserts on every packed struct)
 * --------------------------------------------------------------------- */
#define APP_STATIC_ASSERT(cond, name)	\
	typedef char app_static_assert_##name[(cond) ? 1 : -1]

/* ---------------------------------------------------------------------
 *  Task priorities (design Section 3, TEAM.md section 4).  1 is the
 *  highest; 1-4 belong to the kernel (USB console 3, Wi-Fi 4).
 *  Safety and movement above reporting; motion before line; scan before
 *  IMU (open proposal F7: swap scan/IMU after the A1 jitter logs).
 * --------------------------------------------------------------------- */
#define APP_PRI_CTRL		10	/* Vehicle Controller (Highest)   */
#define APP_PRI_MOTION		11	/* Motion Control (High)          */
#define APP_PRI_LINE		12	/* Line & Barcode Sensing (High)  */
#define APP_PRI_SCAN		13	/* Obstacle Scanning (Medium)     */
#define APP_PRI_IMU		14	/* Terrain Monitoring (Medium)    */
#define APP_PRI_COMMS		20	/* Communication (Low, NFR3)      */

#define TASK_STACK_SIZE		2048	/* bytes; tm_printf needs ~1 KB */

/* Create and start a task.  prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or
   0 for "any core".  Defined in app_main.c. */
IMPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name);
IMPORT ID app_start_task_stk(FP entry, PRI pri, UW prc, SZ stksz,
			     const char *name);

/* ---------------------------------------------------------------------
 *  Shared helpers (app_main.c)
 * --------------------------------------------------------------------- */
/* Microseconds since boot, from the RP2040 1 MHz timer (wraps ~71 min).
   Safe from tasks and ISRs. */
IMPORT UW app_time_us(void);
/* Milliseconds since boot (kernel system time, low 32 bits). */
IMPORT UW app_time_ms(void);
/* Busy-wait for a few microseconds (use only for < 50 us). */
IMPORT void app_delay_us(UW us);
/* Release RP2040 peripheral blocks from reset.  Only CLEARS reset bits,
   never asserts them, so a block another owner already uses is untouched. */
IMPORT void app_unreset(UW mask);
/* Watchdog (design 5 / F8): enabled by ctrl_task, fed by ctrl_task only. */
IMPORT BOOL app_watchdog_caused_reboot(void);	/* read once at boot */
IMPORT void app_watchdog_start(UW timeout_ms);
IMPORT void app_watchdog_feed(void);

/* ---------------------------------------------------------------------
 *  Vehicle state (4 bits, design 10.3).  Held by ctrl_task only.
 *  POWER OFF is not a firmware state.  LINE RECOVERY is reserved.
 * --------------------------------------------------------------------- */
typedef enum {
	VS_INITIALISING		= 0,
	VS_READY		= 1,
	VS_LINE_FOLLOWING	= 2,
	VS_BARCODE_MANOEUVRE	= 3,
	VS_HUMP_CROSSING	= 4,
	VS_OBSTACLE_BYPASS	= 5,
	VS_MISSION_COMPLETE	= 6,
	VS_STOPPED		= 7,
	VS_FAULT_STOP		= 8,
	VS_LINE_RECOVERY	= 9,	/* reserved: added during integration */
	VS_COUNT		= 10
} VEHICLE_STATE;
#define VS_BITS			4

/* ---------------------------------------------------------------------
 *  Barcode command (3 bits, design 10.3).  The letter-to-command
 *  mapping arrives over MQTT before the demo (IF-15 below).
 * --------------------------------------------------------------------- */
typedef enum {
	BC_NONE			= 0,
	BC_TURN_LEFT		= 1,
	BC_TURN_RIGHT		= 2,
	BC_GO_STRAIGHT		= 3,
	BC_U_TURN		= 4,
	BC_UNRECOGNISED		= 5,
	BC_END_OF_COURSE	= 6	/* provisional: see APP_END_MARKER */
} BARCODE_CMD;
#define BC_BITS			3

/* ---------------------------------------------------------------------
 *  IF-15  Barcode map (MQTT -> line_task, ctrl_task).  NOT hard-coded:
 *  the instructor names the four letters just before the demo and the
 *  dashboard sends them over MQTT (wire format in subsystem1.h).
 *  ONE aligned 32-bit word, ONE writer: comms_bridge_on_command()
 *  (subsystem 1).  Readers take a copy (design 10.4 rule 1):
 *    bits  0-7   letter for GO STRAIGHT (forward)
 *    bits  8-15  letter for U-TURN
 *    bits 16-23  letter for TURN LEFT
 *    bits 24-31  letter for TURN RIGHT
 *  0 in a byte = no letter set for that movement.
 * --------------------------------------------------------------------- */
#define APP_BARCODE_MAP(fwd, uturn, left, right)			\
	((UW)(UB)(fwd) | ((UW)(UB)(uturn) << 8) |			\
	 ((UW)(UB)(left) << 16) | ((UW)(UB)(right) << 24))
/* Map at power-on, before MQTT delivers one.  0 = none (the demo rule).
   For a test without Wi-Fi only, e.g. APP_BARCODE_MAP('F','U','A','Z'). */
#define APP_BARCODE_MAP_DEFAULT	0UL

IMPORT volatile UW app_barcode_map;

/* Barcode TEST mode (dashboard switch, MQTT): 1 = the car never drives
   (START refused, a moving car is stopped) and the barcode reader works
   with the car standing still, so a barcode can be slid under IR_L by
   hand.  Every read is logged with how many bars were counted.  Not
   retained: a reboot always comes back in normal mode.
   ONE writer: comms_bridge_on_command() (subsystem 1). */
IMPORT volatile UW app_barcode_test;

/* Movement for a letter under a map; BC_UNRECOGNISED if not in it */
Inline BARCODE_CMD app_barcode_lookup(UW map, UB letter)
{
	BARCODE_CMD	c = BC_UNRECOGNISED;

	if (letter == 0U) {
		c = BC_NONE;
	} else if ((UB)(map & 0xFFU) == letter) {
		c = BC_GO_STRAIGHT;
	} else if ((UB)((map >> 8) & 0xFFU) == letter) {
		c = BC_U_TURN;
	} else if ((UB)((map >> 16) & 0xFFU) == letter) {
		c = BC_TURN_LEFT;
	} else if ((UB)((map >> 24) & 0xFFU) == letter) {
		c = BC_TURN_RIGHT;
	} else {
		/* not one of the four letters */
	}
	return c;
}

/* End-of-course marker: open question to staff (8.3 "course rules").
   A letter in the MQTT map always means its movement, never "end".
   Set to '\0' to disable. */
#define APP_END_MARKER		'Z'

/* ---------------------------------------------------------------------
 *  Remote command (2 bits, design 10.3)
 * --------------------------------------------------------------------- */
typedef enum {
	RC_NONE		= 0,	/* reserved code: rejected on the wire */
	RC_START	= 1,
	RC_STOP		= 2,
	RC_RESET	= 3
} REMOTE_CMD;
#define RC_BITS		2

/* ---------------------------------------------------------------------
 *  Fault mask (8 bits, design 5.1).  A task REPORTS a fault by setting
 *  its bit in the System Event Group (tk_set_flg is atomic).  ctrl_task
 *  handles it, clears the flag bit and latches it in its snapshot until
 *  RESET (design 10.4, rule 2).  Nobody else writes the snapshot.
 * --------------------------------------------------------------------- */
#define FLT_STALL	0x01U	/* motion_task  -> FAULT STOP           */
#define FLT_ENCODER	0x02U	/* motion_task  -> FAULT STOP           */
#define FLT_IMU		0x04U	/* imu_task     -> FAULT STOP           */
#define FLT_BYPASS	0x08U	/* ctrl_task    -> FAULT STOP (2 fails) */
#define FLT_IMPACT	0x10U	/* imu_task     -> STOPPED              */
#define FLT_LINE_LOST	0x20U	/* line_task    -> STOPPED              */
#define FLT_NO_ECHO	0x40U	/* scan_task    -> failed bypass attempt*/
#define FLT_WIFI	0x80U	/* comms_task   -> reported only (T9)   */
#define FLT_ALL		0xFFU
#define FLT_STOP_FAULTS	(FLT_STALL | FLT_ENCODER | FLT_IMU | FLT_BYPASS)

/* ---------------------------------------------------------------------
 *  System Event Group (one µT-Kernel event flag, owned by ctrl_task,
 *  ID flg_sys in subsystem5.h).  Bit layout of the 32-bit pattern:
 *    bits 0-7   fault reports (FLT_xxx above)
 *    bits 8-15  "a message is waiting" signals for ctrl_task
 *    bits 16-23 state bits with a single named writer
 *  Data never travels in the flag: it travels in the message buffers.
 * --------------------------------------------------------------------- */
#define EVF_NAV			(1U << 8)	/* line_task  -> Navigation Command Queue  */
#define EVF_TERRAIN		(1U << 9)	/* imu_task   -> Terrain Status Queue      */
#define EVF_OBSTACLE		(1U << 10)	/* scan_task  -> Obstacle Profile Queue    */
#define EVF_MOTION		(1U << 11)	/* motion_task-> Motion Status Queue       */
#define EVF_REMOTE		(1U << 12)	/* comms      -> Remote Command Queue      */
#define EVF_CTRL_WAKE		(0x0000FFFFU)	/* everything ctrl_task waits for          */

#define EVF_HUMP_ACTIVE		(1U << 16)	/* writer: imu_task only (set/clear)       */
#define EVF_IMU_READY		(1U << 17)	/* writer: imu_task (baseline done)        */
#define EVF_LINE_READY		(1U << 18)	/* writer: line_task (ADC/DMA running)     */

/* ---------------------------------------------------------------------
 *  Every subsystem provides these.  usermain() calls ALL the _init()
 *  functions first and only then ALL the _start() functions, so every
 *  kernel object exists before any task can use it.
 * --------------------------------------------------------------------- */
IMPORT ER subsystem1_init(void);
IMPORT ER subsystem2_init(void);
IMPORT ER subsystem3_init(void);
IMPORT ER subsystem4_init(void);
IMPORT ER subsystem5_init(void);

IMPORT ER subsystem1_start(void);
IMPORT ER subsystem2_start(void);
IMPORT ER subsystem3_start(void);
IMPORT ER subsystem4_start(void);
IMPORT ER subsystem5_start(void);

/* 1 = build the ultrasonic diagnostic (subsystem5/us_diag.c) instead of normal
   running: the car never drives, GP0/GP1 are tested every 5 s */
#ifndef APP_US_DIAG
#define APP_US_DIAG		0
#endif
#if APP_US_DIAG
IMPORT void us_diag(void);
#endif

#if APP_TEST
IMPORT void subsystem1_test(void);
IMPORT void subsystem2_test(void);
IMPORT void subsystem3_test(void);
IMPORT void subsystem4_test(void);
IMPORT void subsystem5_test(void);
#endif

/* Print a readable message if a kernel call failed (negative error code) */
#define CHECK(er, what)							\
	do {								\
		INT check_er_ = (INT)(er);	/* evaluate once */	\
		if (check_er_ < E_OK) {					\
			tm_printf((UB*)"ERROR %s: %d\n", (what), check_er_); \
		}							\
	} while (0)

/* Clamp helper used by several subsystems */
#define APP_CLAMP(v, lo, hi)	(((v) < (lo)) ? (lo) : (((v) > (hi)) ? (hi) : (v)))
#define APP_ABS(v)		(((v) < 0) ? -(v) : (v))

#endif /* APP_H */
