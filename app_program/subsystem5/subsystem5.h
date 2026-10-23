/*
 *	subsystem5.h
 *	Subsystem 5 -- Obstacle scanning (scan_task) and the Vehicle
 *	Controller (ctrl_task, shared integration).  Buddy 5.
 *
 *	BASIC VERSION.  This header is the integration contract: the mission
 *	state, the ctrl_task event flag and every message that is sent TO
 *	ctrl_task.  ctrl_task owns (creates) all of these objects.
 *
 *	Rule (Section 4 of the design review): data travels in message
 *	buffers; event flag bits only say "something is waiting".  A sender
 *	always sends the message first and sets the flag bit second.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"

/* ------------------------------------------------------------------ */
/* Priorities (design order: ctrl > motion > line > scan > imu > comms).
 * 1-4 are the kernel's (USB console 3, Wi-Fi 4).  Agreed in TEAM.md.  */
#define S5_PRI_CTRL		10
#define S5_PRI_SCAN		13
#define S5_PRINT_ENABLE		1	/* 0 = keep scan/ctrl debug quiet */

/* ------------------------------------------------------------------ */
/* Vehicle (mission) state, Section 5.  POWER OFF is not a firmware
 * state.  Stored in 4 bits in the complete version; a plain enum here. */
typedef enum {
	VS_INITIALISING = 0,
	VS_READY,
	VS_LINE_FOLLOWING,
	VS_BARCODE_MANOEUVRE,
	VS_HUMP_CROSSING,
	VS_OBSTACLE_BYPASS,	/* basic: stop and wait until the path clears */
	VS_MISSION_COMPLETE,
	VS_STOPPED,
	VS_FAULT_STOP,
	VS_LINE_RECOVERY,	/* used by the complete version only */
	VS_NUM_STATES
} VEHICLE_STATE;

/* Read from any task; written only by ctrl_task (one aligned word). */
IMPORT volatile VEHICLE_STATE	s5_vehicle_state;
IMPORT const char * const	s5_state_name[VS_NUM_STATES];

/* ------------------------------------------------------------------ */
/* ctrl_task event flag (owned by ctrl_task).                          */
IMPORT ID	flg_ctrl;

#define FLG_NAV			(1U << 0)	/* message on mbf_nav        */
#define FLG_TERRAIN		(1U << 1)	/* message on mbf_terrain    */
#define FLG_OBSTACLE		(1U << 2)	/* message on mbf_obstacle   */
#define FLG_MOTION		(1U << 3)	/* message on mbf_motion_st  */
#define FLG_EVENTS_ALL		(FLG_NAV | FLG_TERRAIN | FLG_OBSTACLE | FLG_MOTION)

/* State bit, set and cleared by imu_task only (never by ctrl_task).
 * scan_task ignores readings while it is set (it may see the hump).   */
#define FLG_HUMP_ACTIVE		(1U << 8)

/* ------------------------------------------------------------------ */
/* Barcode command (FR2).  The letter-to-command table is in
 * subsystem3/barcode.c and is still to be confirmed with staff.       */
typedef enum {
	BC_NONE = 0,
	BC_TURN_LEFT,
	BC_TURN_RIGHT,
	BC_STRAIGHT,
	BC_U_TURN,
	BC_UNRECOGNISED,
	BC_END_OF_COURSE	/* reserved: only if staff confirm a marker */
} BARCODE_CMD;

/* Navigation Command Queue: line_task -> ctrl_task */
typedef enum {
	NAV_BARCODE = 0,	/* value = BARCODE_CMD, letter = decoded char */
	NAV_LINE_LOST,
	NAV_LINE_FOUND
} NAV_KIND;

typedef struct {
	NAV_KIND	kind;
	BARCODE_CMD	cmd;
	UB		letter;		/* decoded Code 39 character   */
	W		distance_mm;	/* odometer when it happened   */
} NAV_MSG;
IMPORT ID	mbf_nav;

/* Terrain Status Queue: imu_task -> ctrl_task */
typedef enum {
	TER_HUMP_START = 0,
	TER_HUMP_END,		/* value = peak height estimate, mm */
	TER_HUMP_DISCARDED,	/* climb too long: ramp or bad reading */
	TER_IMPACT,		/* value = |a| deviation, mg */
	TER_IMU_FAULT
} TERRAIN_KIND;

typedef struct {
	TERRAIN_KIND	kind;
	W		value;
} TERRAIN_MSG;
IMPORT ID	mbf_terrain;

/* Obstacle Profile Queue: scan_task -> ctrl_task */
typedef enum {
	OBS_ALERT = 0,		/* two readings inside the stop distance */
	OBS_CLEAR,		/* path ahead clear again               */
	OBS_SENSOR_FAULT	/* ECHO never rose, 3 times in a row     */
} OBSTACLE_KIND;

typedef struct {
	OBSTACLE_KIND	kind;
	W		distance_mm;
} OBSTACLE_MSG;
IMPORT ID	mbf_obstacle;

/* Motion Status Queue: motion_task -> ctrl_task */
typedef enum {
	MST_DONE = 0,		/* the last move/turn finished           */
	MST_STALL		/* stall detected: motors locked off     */
} MOTION_STATUS_KIND;

typedef struct {
	MOTION_STATUS_KIND	kind;
	W			distance_mm;
} MOTION_STATUS_MSG;
IMPORT ID	mbf_motion_st;

/* Helper for producers: send, then raise the flag bit. */
IMPORT ER	s5_post(ID mbf, const void *msg, INT size, UINT flag_bit);

/* Milliseconds since boot (kernel operating time, wraps after 49 days). */
IMPORT UW	s5_now_ms(void);

/* ------------------------------------------------------------------ */
/* Fault mask (Section 5.1).  Latched by ctrl_task until RESET. */
#define FAULT_STALL		(1U << 0)
#define FAULT_ENCODER		(1U << 1)
#define FAULT_IMU		(1U << 2)
#define FAULT_FAILED_BYPASS	(1U << 3)
#define FAULT_IMPACT		(1U << 4)
#define FAULT_LINE_LOST		(1U << 5)
#define FAULT_NO_ECHO		(1U << 6)
#define FAULT_WIFI_LOSS		(1U << 7)

IMPORT volatile UW	s5_fault_mask;	/* written only by ctrl_task */

/* ------------------------------------------------------------------ */
/* Ultrasonic + servo (scan_task) */
#define S5_PIN_TRIG		6	/* Grove 5 (its GP26 wire unused) */
#define S5_PIN_ECHO		7	/* Grove 7, HC-SR04+ 3.3 V only */
#define S5_PIN_SERVO		12	/* servo port 1, PWM slice 6A */
#define S5_PIN_BTN_START	20	/* on-board, pressed = 0 */
#define S5_PIN_BTN_STOP		21	/* on-board, pressed = 0 */

#define S5_TRIG_PULSE_US	10U
#define S5_RISE_TMO_US		3000U	/* ECHO must rise within this */
#define S5_ECHO_TMO_US		6000U	/* ~1 m; longer = nothing in range */
#define S5_SCAN_PERIOD_MS	60
#define S5_STOP_MM		200	/* provisional: set from braking test */
#define S5_CLEAR_MM		300	/* path counts as clear above this */
#define S5_CONFIRM_N		2	/* readings inside stop distance */
#define S5_CLEAR_N		3
#define S5_NO_ECHO_FAULT_N	3
#define S5_SERVO_MIN_US		500U	/* calibrate: pulse for 0 deg */
#define S5_SERVO_MAX_US		2500U	/* calibrate: pulse for 180 deg */

typedef enum {
	S5_PING_OK = 0,
	S5_PING_OUT_OF_RANGE,	/* nothing within ~1 m: normal */
	S5_PING_NO_ECHO,	/* ECHO never rose: fault */
	S5_PING_BUSY
} S5_PING_RESULT;

/* Read by the tests and by ctrl_task for telemetry. Written by scan_task. */
IMPORT volatile BOOL	s5_scan_ready;
IMPORT volatile W	s5_last_mm;	/* -1 = nothing in range */
IMPORT volatile UW	s5_ping_count;

/* ctrl_task settings */
#define S5_CTRL_TICK_MS		50	/* regular check + button poll */
#define S5_TELEMETRY_EVERY	4	/* 4 x 50 ms = 200 ms (5 Hz) */
#define S5_INIT_WAIT_MS		3000	/* subsystems must be ready by then */
#define S5_REQUIRE_IMU		1	/* 0 = drive without the IMU (bench) */
#define S5_RESET_HOLD_MS	1000	/* hold both buttons = RESET */
#define S5_CRUISE_MM_S		300	/* provisional cruise speed (A3) */
#define S5_HUMP_MM_S		150	/* speed limit while on a hump */

/* Barcode execution (course rules still to be confirmed with staff):
 * after a decode, keep following for EXEC_DELAY_MM, drive EXEC_FWD_MM
 * so the wheels are over the junction, then turn. */
#define S5_EXEC_DELAY_MM	100
#define S5_EXEC_FWD_MM		60

IMPORT ID	tid_ctrl;
IMPORT ID	tid_scan;
IMPORT ID	cyc_scan;
IMPORT void	ctrl_task(INT stacd, void *exinf);
IMPORT void	scan_task(INT stacd, void *exinf);
IMPORT void	scan_cyclic_handler(void *exinf);

#endif /* SUBSYSTEM5_H */
