/*
 *	subsystem5.h
 *	Subsystem 5 -- Obstacle scanning (scan_task) and the Vehicle
 *	Controller (ctrl_task, shared integration).  Buddy 5.
 *
 *	COMPLETE VERSION.  This header is the integration contract: the
 *	mission state, the ctrl_task event flag (events + fault bits) and
 *	every message sent TO ctrl_task.  ctrl_task owns all of these objects.
 *
 *	Rules (Sections 4, 5.1 and 10 of the design review):
 *	  - data travels in message buffers; event flag bits only signal;
 *	    a sender sends the message first, then sets the bit;
 *	  - faults are reported by SETTING a fault bit (s5_raise_fault);
 *	    only ctrl_task clears it, after copying it into the latched
 *	    fault mask, which stays set until RESET;
 *	  - ctrl_task is the only writer of the mission state, the motion
 *	    commands and the telemetry snapshot.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"

/* ------------------------------------------------------------------ */
#define S5_PRI_CTRL		10
#define S5_PRI_SCAN		13
#define S5_PRINT_ENABLE		1

/* Vehicle state (Section 5).  9 firmware states + LINE RECOVERY = 10,
 * so 4 bits in the packed telemetry (Section 10). */
typedef enum {
	VS_INITIALISING = 0,
	VS_READY,
	VS_LINE_FOLLOWING,
	VS_BARCODE_MANOEUVRE,
	VS_HUMP_CROSSING,
	VS_OBSTACLE_BYPASS,
	VS_MISSION_COMPLETE,
	VS_STOPPED,
	VS_FAULT_STOP,
	VS_LINE_RECOVERY,
	VS_NUM_STATES
} VEHICLE_STATE;

IMPORT volatile VEHICLE_STATE	s5_vehicle_state;	/* one aligned word */
IMPORT const char * const	s5_state_name[VS_NUM_STATES];

/* Mission state: the larger record, protected by a mutex created with
 * priority inheritance (TA_INHERIT).  Critical sections are a struct
 * copy only.  comms_task never takes it (NFR3). */
typedef struct {
	VEHICLE_STATE	state;
	UW		fault_mask;
	W		last_barcode;
	W		bypass_failures;
	BOOL		hump_active;
} MISSION_STATE;
IMPORT ID	mtx_mission;
IMPORT void	s5_mission_get(MISSION_STATE *p_out);

/* ------------------------------------------------------------------ */
/* ctrl_task event flag.  Bits 0-7: "message waiting".  Bit 8: hump state
 * (imu_task only).  Bits 16-23: fault bits (any task sets, ctrl clears). */
IMPORT ID	flg_ctrl;

#define FLG_NAV			(1U << 0)
#define FLG_TERRAIN		(1U << 1)
#define FLG_OBSTACLE		(1U << 2)
#define FLG_MOTION		(1U << 3)
#define FLG_REMOTE		(1U << 4)
#define FLG_EVENTS_ALL		(FLG_NAV | FLG_TERRAIN | FLG_OBSTACLE | FLG_MOTION | FLG_REMOTE)
#define FLG_HUMP_ACTIVE		(1U << 8)
#define FLG_FAULT_SHIFT		16
#define FLG_FAULTS_ALL		(0xFFU << FLG_FAULT_SHIFT)

/* Fault mask (Section 5.1), 8 bits. */
#define FAULT_STALL		(1U << 0)
#define FAULT_ENCODER		(1U << 1)
#define FAULT_IMU		(1U << 2)
#define FAULT_FAILED_BYPASS	(1U << 3)
#define FAULT_IMPACT		(1U << 4)
#define FAULT_LINE_LOST		(1U << 5)
#define FAULT_NO_ECHO		(1U << 6)
#define FAULT_WIFI_LOSS		(1U << 7)

IMPORT void		s5_raise_fault(UW fault_bit);	/* any task */
IMPORT volatile UW	s5_fault_mask;			/* latched, ctrl only */
IMPORT volatile BOOL	s5_watchdog_reset;		/* read once at boot */

/* ------------------------------------------------------------------ */
typedef enum {
	BC_NONE = 0,
	BC_TURN_LEFT,
	BC_TURN_RIGHT,
	BC_STRAIGHT,
	BC_U_TURN,
	BC_UNRECOGNISED,
	BC_END_OF_COURSE	/* reserved: only if staff confirm a marker */
} BARCODE_CMD;		/* 3 bits */

/* Navigation Command Queue: line_task -> ctrl_task */
typedef enum {
	NAV_BARCODE = 0,
	NAV_LINE_LOST,
	NAV_LINE_FOUND,
	NAV_JUNCTION_RIGHT	/* wide isolated stripe under IR_R */
} NAV_KIND;

typedef struct {
	NAV_KIND	kind;
	BARCODE_CMD	cmd;
	UB		letter;
	W		distance_mm;
} NAV_MSG;
IMPORT ID	mbf_nav;

/* Terrain Status Queue: imu_task -> ctrl_task */
typedef enum {
	TER_HUMP_START = 0,
	TER_HUMP_END,		/* value = peak height estimate, mm */
	TER_HUMP_DISCARDED,
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
	OBS_PROFILE,		/* coarse + fine scan finished */
	OBS_CLEAR,
	OBS_SENSOR_FAULT	/* profile = "sensor fault" (ECHO never rose x3) */
} OBSTACLE_KIND;

typedef enum {
	SIDE_NONE = 0,
	SIDE_LEFT,
	SIDE_RIGHT,
	SIDE_BLOCKED		/* neither side has room */
} BYPASS_SIDE;		/* 2 bits */

typedef struct {
	OBSTACLE_KIND	kind;
	W		distance_mm;	/* closest point */
	W		width_mm;
	W		left_mm;	/* how far the obstacle reaches left of the
					   car's centre line (negative = it ends
					   right of centre) */
	W		right_mm;	/* same, to the right */
	W		left_clear_mm;	/* free space beyond the left edge */
	W		right_clear_mm;
	BYPASS_SIDE	side;		/* recommended side */
} OBSTACLE_MSG;
IMPORT ID	mbf_obstacle;

/* Motion Status Queue: motion_task -> ctrl_task */
typedef enum {
	MST_DONE = 0,
	MST_STALL,
	MST_ENCODER_FAULT
} MOTION_STATUS_KIND;

typedef struct {
	MOTION_STATUS_KIND	kind;
	W			distance_mm;
	BOOL			line_found;	/* "until line" moves only */
} MOTION_STATUS_MSG;
IMPORT ID	mbf_motion_st;

/* Remote Command Queue: comms (MQTT) -> ctrl_task.  2-bit command. */
typedef enum {
	RC_NONE = 0,
	RC_START = 1,
	RC_STOP = 2,
	RC_RESET = 3
} REMOTE_CMD;

typedef struct {
	REMOTE_CMD	cmd;
} REMOTE_MSG;
IMPORT ID	mbf_remote;

IMPORT ER	s5_post(ID mbf, const void *msg, INT size, UINT flag_bit);
IMPORT UW	s5_now_ms(void);

/* ------------------------------------------------------------------ */
/* scan_task.  ctrl_task asks for a (re)scan through one word it alone
 * writes; scan_task only reads it. */
typedef enum {
	SCAN_REQ_WATCH = 0,	/* look straight ahead */
	SCAN_REQ_PROFILE,	/* do a coarse + fine scan now */
	SCAN_REQ_HOLD		/* servo centred, no alerts (during bypass) */
} SCAN_REQUEST;
IMPORT volatile SCAN_REQUEST	s5_scan_request;
IMPORT volatile UW		s5_scan_request_seq;	/* bump to re-request */

#define S5_PIN_TRIG		6	/* Grove 5 (its GP26 wire unused) */
#define S5_PIN_ECHO		7	/* Grove 7, HC-SR04+ 3.3 V only */
#define S5_PIN_SERVO		12	/* servo port 1, PWM slice 6A */
#define S5_PIN_BTN_START	20
#define S5_PIN_BTN_STOP		21
#define S5_PIN_NEOPIXEL		18
#define S5_PIN_BUZZER		22	/* PWM slice 3A */

#define S5_TRIG_PULSE_US	10U
#define S5_RISE_TMO_US		3000U
#define S5_ECHO_TMO_US		6000U	/* ~1 m */
#define S5_SCAN_PERIOD_MS	60
#define S5_STOP_MM		200	/* provisional: set from braking test */
#define S5_CLEAR_MM		300
#define S5_CONFIRM_N		2
#define S5_CLEAR_N		3
#define S5_NO_ECHO_FAULT_N	3
#define S5_HUMP_IGNORE_MAX_MM	300	/* ignore window bounded by distance */
#define S5_SERVO_MIN_US		500U
#define S5_SERVO_MAX_US		2500U
#define S5_SERVO_SETTLE_MS	100	/* SG90: about 0.1 s per 60 deg */
#define S5_COARSE_STEP_DEG	30	/* 30, 60, 90, 120, 150 */
#define S5_FINE_STEP_DEG	5
#define S5_PROFILE_RANGE_MM	600	/* readings closer than this = obstacle */
#define S5_CLEAR_NEED_MM	250	/* free space needed beside the edge */

/* Servo angle convention: 90 = straight ahead; above 90 = LEFT.  If your
 * servo is mounted the other way, set this to 0. */
#define S5_SERVO_HIGH_IS_LEFT	1

typedef enum {
	S5_PING_OK = 0,
	S5_PING_OUT_OF_RANGE,	/* nothing within ~1 m: normal */
	S5_PING_NO_ECHO,	/* ECHO never rose: fault */
	S5_PING_BUSY
} S5_PING_RESULT;

IMPORT volatile BOOL	s5_scan_ready;
IMPORT volatile W	s5_last_mm;
IMPORT volatile UW	s5_ping_count;
IMPORT volatile UW	s5_echo_isr_count;
IMPORT volatile OBSTACLE_MSG	s5_last_profile;	/* telemetry copy */

/* ------------------------------------------------------------------ */
/* ctrl_task settings */
#define S5_CTRL_TICK_MS		50
#define S5_TELEMETRY_MS		200	/* 5 Hz */
#define S5_INIT_WAIT_MS		3000
#define S5_REQUIRE_IMU		1
#define S5_RESET_HOLD_MS	1000
#define S5_CRUISE_MM_S		300
#define S5_HUMP_MM_S		150
#define S5_SLOW_ZONE_MM_S	150	/* after an alert, before stopping */

#define S5_CAL_SWEEP		1	/* 1 = turn over the line at start-up */
#define S5_CAL_SWEEP_DEG	25

/* Barcode execution (course rules to be confirmed): execute at the next
 * right junction, or after EXEC_MAX_MM if none is seen. */
#define S5_EXEC_MAX_MM		250
#define S5_EXEC_FWD_MM		60

/* Bypass (draft of the sub-statechart in Section 5; minimise deviation). */
#define S5_CAR_HALF_WIDTH_MM	80
#define S5_BYPASS_MARGIN_MM	60
#define S5_BYPASS_DEPTH_MM	150	/* assumed obstacle depth */
#define S5_CAR_LENGTH_MM	180
#define S5_SEARCH_EXTRA_MM	150	/* search past the expected line */
#define S5_BYPASS_MAX_FAILS	2
#define S5_REVERSE_MM		100

/* Line recovery: turn toward the last side, then the other way. */
#define S5_RECOVER_DEG_1	45
#define S5_RECOVER_DEG_2	90

#define S5_WATCHDOG_ENABLE	1
#define S5_WATCHDOG_MS		1000

IMPORT ID	tid_ctrl;
IMPORT ID	tid_scan;
IMPORT void	ctrl_task(INT stacd, void *exinf);
IMPORT void	scan_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM5_H */
