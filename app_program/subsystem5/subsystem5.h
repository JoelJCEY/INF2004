/*
 *	subsystem5.h
 *	Subsystem 5 -- Obstacle Scanning (scan_task) and the shared Vehicle
 *	Controller (ctrl_task).  Buddy 5: 2500118 (Marcus), integration lead.
 *	Statechart 6.5, system statechart 5, fault table 5.1, assumption A5.
 *
 *	scan_task owns: HC-SR04+ (GP0 TRIG, GP1 ECHO + ECHO ISR, Grove 1,
 *	3.3 V only) and the scan servo (GP12, PWM slice 6A, 50 Hz).
 *	ctrl_task owns: START GP20, STOP GP21, NeoPixel GP18 (PIO), buzzer
 *	GP22 (PWM slice 3A), the watchdog, and the mission state.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"

#define S5_PRI_CTRL		APP_PRI_CTRL
#define S5_PRI_SCAN		APP_PRI_SCAN
#define S5_CTRL_TICK_MS		50	/* regular check + button poll */
#define S5_SCAN_PERIOD_MS	60	/* ping interval while watching */

/* 1 = obstacle detection and bypass active.  0 = scan_task never raises
   an alert (bring-up: line following tests while the ultrasonic is not
   yet on its servo).  Set back to 1 for obstacle tests. */
#define SCAN_ENABLE		1

#define US_TRIG_PIN		0	/* Grove 1: unplug for CRASH_REPORT=1 */
#define US_ECHO_PIN		1
#define SERVO_PIN		12
#define BTN_START_PIN		20
#define BTN_STOP_PIN		21
#define NEOPIXEL_PIN		18
#define BUZZER_PIN		22

/* ---------------------------------------------------------------------
 *  Kernel objects owned by ctrl_task (it receives on all of them)
 * --------------------------------------------------------------------- */
IMPORT ID	flg_sys;		/* System Event Group (app.h EVF_/FLT_) */
IMPORT ID	mtx_mission;		/* mission state, TA_INHERIT (F5)       */
IMPORT ID	mbf_nav;		/* IF-09 line_task   -> ctrl_task       */
IMPORT ID	mbf_terrain;		/* IF-11 imu_task    -> ctrl_task       */
IMPORT ID	mbf_obstacle;		/* IF-13 scan_task   -> ctrl_task       */
IMPORT ID	mbf_motion_status;	/* IF-06 motion_task -> ctrl_task       */
IMPORT ID	mbf_remote;		/* IF-03 comms       -> ctrl_task       */

#define S5_QUEUE_MSGS		8	/* depth of every queue above */

/* Report a fault from any task: sets its bit in the event flag. */
Inline void app_report_fault(UINT flt)
{
	(void)tk_set_flg(flg_sys, flt & FLT_ALL);
}

/* ---------------------------------------------------------------------
 *  IF-04  Mission state (ctrl_task writes, others read a copy).
 *  Protected by mtx_mission (TA_INHERIT); the critical section is a
 *  struct copy only.  comms_task never takes this mutex (NFR3).
 * --------------------------------------------------------------------- */
typedef struct {
	UB	state;		/* VEHICLE_STATE (4 bits used)       */
	UB	fault_mask;	/* latched FLT_xxx                   */
	UB	bypass_fails;	/* failed bypass attempts (0..2)     */
	UB	bypass_side;	/* OBS_SIDE_xxx while bypassing      */
} MISSION_STATE;
APP_STATIC_ASSERT(sizeof(MISSION_STATE) == 4, mission_state_size);

IMPORT void ctrl_get_mission(MISSION_STATE *out);

/* Sensor bench (TEST builds): while scan_bench_on != 0 and the car is
   not moving, scan_task pings straight ahead every ~70 ms and publishes
   (US_RESULT << 16) | mm in scan_bench_word, then increments
   scan_bench_seq. */
/* Last full sweep (mm per servo position 30..150 deg in 5 deg steps,
   0 = not measured, 1000 = nothing within ~1 m) and a count that
   changes after each sweep.  Dashboard only. */
IMPORT volatile UH scan_last_sweep[25];
IMPORT volatile UW scan_sweep_seq;
IMPORT volatile UW scan_bench_on;
/* 1 = scan_task does nothing (ultrasonic diagnostic owns GP0/GP1) */
IMPORT volatile UW scan_hold;
IMPORT volatile UW scan_bench_word;
IMPORT volatile UW scan_bench_seq;

/* ---------------------------------------------------------------------
 *  IF-13  Obstacle Profile Queue (scan_task -> ctrl_task)
 * --------------------------------------------------------------------- */
typedef enum {
	OB_ALERT		= 1,	/* 2 readings inside the stop distance  */
	OB_PROFILE		= 2,	/* sweep finished: fields valid         */
	OB_SENSOR_FAULT		= 3,	/* 3 x no echo: a failed attempt        */
	OB_CLEARANCE_LOST	= 4,	/* side guard tripped during a bypass   */
	OB_NO_CLEARANCE		= 5	/* profile: neither side is wide enough */
} OBSTACLE_KIND;

#define OBS_SIDE_NONE		0U
#define OBS_SIDE_LEFT		1U
#define OBS_SIDE_RIGHT		2U

typedef struct {
	UB	kind;		/* OBSTACLE_KIND                         */
	UB	side;		/* recommended bypass side               */
	UB	servo_idx;	/* 0..24 = 30°..150° in 5° steps         */
	UB	reserved;
	UH	width_mm;	/* obstacle width                        */
	UH	closest_mm;	/* closest reading                       */
	UH	offset_mm;	/* lateral distance to clear on that side*/
	UH	depth_mm;	/* estimated length to drive past it     */
} OBSTACLE_MSG;
APP_STATIC_ASSERT(sizeof(OBSTACLE_MSG) == 12, obstacle_msg_size);

/* ---------------------------------------------------------------------
 *  IF-03  Remote Command Queue (comms -> ctrl_task)
 * --------------------------------------------------------------------- */
typedef struct {
	UB	cmd;		/* REMOTE_CMD (2 bits used) */
	UB	seq;		/* sender's sequence number */
	UH	reserved;
} REMOTE_MSG;
APP_STATIC_ASSERT(sizeof(REMOTE_MSG) == 4, remote_msg_size);

/* ---------------------------------------------------------------------
 *  ECHO ISR result word (design 10.3 / 10.4 rule 3).  us_echo_isr is its
 *  ONLY writer; scan_task only reads.  One aligned 32-bit store:
 *    bits 0-12  echo duration µs (saturates at 8191)
 *    bit  13    VALID: falling edge seen, duration complete
 *    bit  14    ROSE:  rising edge seen for this ping
 *    bits 16-31 ping sequence (incremented on every rising edge)
 * --------------------------------------------------------------------- */
#define ECHO_DUR_MASK		0x1FFFU
#define ECHO_VALID		(1UL << 13)
#define ECHO_ROSE		(1UL << 14)
#define ECHO_SEQ_SHIFT		16
#define ECHO_TIMEOUT_US		6000U	/* ~1 m of range (C1) */

IMPORT volatile UW us_echo_word;

/* Scan status for telemetry (writer scan_task only):
     bits 0-4 servo index, bits 5-7 scan state, bits 16-28 last mm */
IMPORT volatile UW scan_status_word;

/* Rescan request (ctrl_task -> scan_task, writer ctrl_task only): each
   increment asks for one more full sweep during OBSTACLE BYPASS, after
   the car has reversed (6.5: "reverses a short distance and rescans"). */
IMPORT volatile UW scan_rescan_request;
/* 31 Oct: watch the obstacle's side while passing it (bypass "pass" leg).
   scan_pass_watch:  written by ctrl_task.  0 = off, else servo index + 1
                     to look at while driving the pass leg.
   scan_pass_seq:    written by scan_task, +1 when the obstacle is behind.
   scan_pass_extra_mm: written by scan_task before scan_pass_seq: how much
                     farther to drive from that moment. */
/* Pass leg look angle (4 Nov): bypass on the LEFT = obstacle on the
   car's RIGHT, so look right.  The servo cannot go past 70 deg on the
   right (at 60 it hit the car), so that side looks only 20 deg right of
   ahead and sees the obstacle's side further ahead; task_scan judges
   "beside it" by sideways distance, which works for any angle. */
#define SCAN_PASS_LOOK_RIGHT_DEG 70	/* bypass on the left  */
#define SCAN_PASS_LOOK_LEFT_DEG	135	/* bypass on the right */
#define SCAN_DEG_TO_IDX(d)	(((d) - 30) / 5)
IMPORT volatile UW scan_pass_watch;
IMPORT volatile UW scan_pass_seq;
IMPORT volatile UW scan_pass_extra_mm;

/* Servo index <-> angle: 30°..150° in 5° steps (5 bits, design 10.3) */
#define SERVO_MIN_DEG		30
#define SERVO_MAX_DEG		150
#define SERVO_STEP_DEG		5
#define SERVO_CENTRE_IDX	12	/* 90°: straight ahead */
#define SERVO_IDX_TO_DEG(i)	(SERVO_MIN_DEG + ((INT)(i) * SERVO_STEP_DEG))

/* ---------------------------------------------------------------------
 *  Private to subsystem 5
 * --------------------------------------------------------------------- */
IMPORT ID	tid_ctrl;
IMPORT ID	tid_scan;
IMPORT void	ctrl_task(INT stacd, void *exinf);
IMPORT void	scan_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM5_H */
