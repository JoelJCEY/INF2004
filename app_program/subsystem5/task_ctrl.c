/*
 *	task_ctrl.c
 *	Subsystem 5 -- ctrl_task, the Vehicle Controller (system statechart,
 *	design 5).  Highest application priority, event driven plus a
 *	regular 50 ms check.
 *
 *	Each wake-up, in this order (safety before navigation, 5 / 11.1):
 *	  1. remote commands and START/STOP buttons
 *	  2. fault reports (event flag bits 0-7) -> latched fault mask
 *	  3. motion status (manoeuvre steps, self-test)
 *	  4. obstacle reports, 5. terrain reports, 6. navigation commands
 *	  7. periodic: start-up sequence, line search, telemetry (200 ms),
 *	     NeoPixel, buzzer, watchdog feed
 *	ctrl_task is the only writer of the Motion Command Queue, the
 *	Telemetry Queue and the mission state.
 */

#include "subsystem5.h"
#include "ctrl_logic.h"
#include "drv_indicators.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"

#define TLM_PERIOD_MS		200	/* 5 Hz (NFR4) */
#define WDT_TIMEOUT_MS		1000	/* ctrl wakes at least every 50 ms */
#define HUMP_SPEED_MM_S		40	/* speed limit while crossing a hump */
#define INIT_SENSOR_TMO_MS	5000
#define CAL_SWEEP_DEG		25	/* IR calibration sweep each way */
#define ALIGN_SEARCH_DEG	35	/* after the sweep: look this far each way for the line */
#define APP_CAL_SWEEP		1	/* 0: keep the default thresholds */
#define BYPASS_TARGET_MS	5000	/* NFR2: < 5 s (target 3 s) */

typedef enum {
	INIT_WAIT_SENSORS = 0,
	INIT_SELFTEST,
	INIT_CAL_SWEEP,
	INIT_CAL_APPLY,
	INIT_ALIGN,		/* turn until the sensors are back on the line */
	INIT_DONE
} INIT_PHASE;

typedef enum {
	BP_WAIT_PROFILE = 0,
	BP_LEGS,
	BP_REVERSING
} BYPASS_PHASE;

LOCAL MISSION_STATE	mission;	/* ctrl_task's working copy           */
LOCAL MISSION_STATE	mission_shared;	/* published copy, under mtx_mission  */

/* Step executor */
LOCAL CTRL_STEP		steps[CTRL_MAX_STEPS];
LOCAL INT		nsteps;
LOCAL INT		step_i;
LOCAL BOOL		pass_active;	/* STEP_PASS running (31 Oct) */
LOCAL UW		pass_seq0;
LOCAL INT		wait_seq = -1;
LOCAL BOOL		search_active;

LOCAL INIT_PHASE	init_phase;
LOCAL BYPASS_PHASE	bp_phase;
LOCAL BOOL		bp_rescanned;
LOCAL UW		phase_t0;
LOCAL UW		bypass_t0;

/* Telemetry inputs kept by ctrl_task */
LOCAL UB		last_remote;
LOCAL UB		last_letter;
LOCAL UB		last_bc;
/* Demo rule (IF-15): a decoded letter is STORED and carried out at the
   next cross-junction.  0 = nothing stored.  Written by ctrl_task only. */
LOCAL UB		pending_letter;
LOCAL UB		hump_count;
LOCAL UH		hump_peak_max;
LOCAL OBSTACLE_MSG	last_obs;
LOCAL BOOL		obs_valid;
LOCAL UH		tlm_dropped;
LOCAL UW		buzz_off_at;

/* ---------------------------------------------------------------------
 *  Mission state: IF-04 (critical section = struct copy only)
 * --------------------------------------------------------------------- */
LOCAL void publish_mission(void)
{
	if (tk_loc_mtx(mtx_mission, TMO_FEVR) == E_OK) {
		mission_shared = mission;
		(void)tk_unl_mtx(mtx_mission);
	}
}

EXPORT void ctrl_get_mission(MISSION_STATE *out)
{
	if (tk_loc_mtx(mtx_mission, TMO_FEVR) == E_OK) {
		*out = mission_shared;
		(void)tk_unl_mtx(mtx_mission);
	}
}

/* ---------------------------------------------------------------------
 *  Indicators
 * --------------------------------------------------------------------- */
LOCAL const UW state_colour[VS_COUNT] = {
	0x000020U,	/* INITIALISING    blue    */
	0x101010U,	/* READY           white   */
	0x002000U,	/* LINE FOLLOWING  green   */
	0x002020U,	/* BARCODE         cyan    */
	0x202000U,	/* HUMP            yellow  */
	0x301000U,	/* OBSTACLE BYPASS orange  */
	0x200020U,	/* COMPLETE        purple  */
	0x100000U,	/* STOPPED         dim red */
	0x400000U,	/* FAULT STOP      red     */
	0x200010U	/* LINE RECOVERY (reserved) */
};

LOCAL void beep(UW hz, UW ms)
{
	drv_buzzer_tone(hz);
	buzz_off_at = app_time_ms() + ms;
	if (buzz_off_at == 0U) {
		buzz_off_at = 1U;
	}
}

/* ---------------------------------------------------------------------
 *  Step executor (manoeuvres through the motion API, P2)
 * --------------------------------------------------------------------- */
LOCAL void steps_done(void);

LOCAL void post_step(void)
{
	const CTRL_STEP	*s;

	search_active = FALSE;
	pass_active = FALSE;
	scan_pass_watch = 0;
	if (step_i >= nsteps) {
		wait_seq = -1;
		steps_done();
		return;
	}
	s = &steps[step_i];
	switch (s->op) {
	case STEP_PASS:
		/* drive past the obstacle while scan_task watches its side */
		pass_seq0 = scan_pass_seq;
		pass_active = TRUE;
		wait_seq = moveForwardAt(s->arg, CTRL_BYPASS_MM_S);
		scan_pass_watch = (UW)((mission.bypass_side == OBS_SIDE_LEFT) ?
				SCAN_DEG_TO_IDX(SCAN_PASS_LOOK_RIGHT_DEG) :
				SCAN_DEG_TO_IDX(SCAN_PASS_LOOK_LEFT_DEG)) + 1U;
		
		break;
	case STEP_TURN:
		wait_seq = (s->arg >= 0) ? turnLeft(s->arg) : turnRight(-s->arg);
		break;
	case STEP_SEARCH:
		search_active = TRUE;
		wait_seq = moveForward(s->arg);
		break;
	case STEP_SEARCH_TURN:
		search_active = TRUE;
		wait_seq = (s->arg >= 0) ? turnLeft(s->arg) : turnRight(-s->arg);
		break;
	default:
		if (mission.state == VS_OBSTACLE_BYPASS) {
			/* 4 Nov: straight bypass legs at CTRL_BYPASS_MM_S */
			wait_seq = (s->arg >= 0) ? moveForwardAt(s->arg, CTRL_BYPASS_MM_S) :
						   moveBackwardAt(-s->arg, CTRL_BYPASS_MM_S);
		} else {
			wait_seq = (s->arg >= 0) ? moveForward(s->arg) : moveBackward(-s->arg);
		}
		break;
	}
}

LOCAL void start_steps(INT n)
{
	nsteps = n;
	step_i = 0;
	post_step();
}

LOCAL void clear_steps(void)
{
	nsteps = 0;
	step_i = 0;
	wait_seq = -1;
	search_active = FALSE;
	pass_active = FALSE;
	scan_pass_watch = 0;
}

/* ---------------------------------------------------------------------
 *  State entry (system statechart 5)
 * --------------------------------------------------------------------- */
LOCAL void enter(VEHICLE_STATE s)
{
	VEHICLE_STATE	prev = (VEHICLE_STATE)mission.state;

	app_log((UB*)"ctrl: state %d -> %d\n", prev, s);
	mission.state = (UB)s;
	if (s != VS_OBSTACLE_BYPASS) {
		mission.bypass_side = OBS_SIDE_NONE;
	}
	publish_mission();
	drv_neopixel_set(state_colour[s]);

	switch (s) {
	case VS_INITIALISING:
		clear_steps();
		init_phase = INIT_WAIT_SENSORS;
		phase_t0 = app_time_ms();
		line_cal_request = LINE_CAL_IDLE;
		(void)motion_reset();
		break;
	case VS_READY:
		clear_steps();
		(void)motion_reset();
		(void)motion_set_speed_limit(0);
		break;
	case VS_LINE_FOLLOWING:
		clear_steps();
		(void)motion_set_speed_limit(0);
		(void)motion_follow_line(MOTION_CRUISE_MM_S);
		break;
	case VS_BARCODE_MANOEUVRE:
		/* steps[] were planned by the caller */
		start_steps(nsteps);
		break;
	case VS_HUMP_CROSSING:
		(void)motion_set_speed_limit(HUMP_SPEED_MM_S);
		break;
	case VS_OBSTACLE_BYPASS:
		clear_steps();
		(void)stop();			/* NFR2: leave LINE FOLLOWING, stop */
		bp_phase = BP_WAIT_PROFILE;
		bp_rescanned = FALSE;
		bypass_t0 = app_time_ms();
		break;
	case VS_STOPPED:
	case VS_MISSION_COMPLETE:
		clear_steps();
		(void)stop();
		if (s == VS_MISSION_COMPLETE) {
			beep(1500U, 600U);
		}
		break;
	case VS_FAULT_STOP:
		clear_steps();
		(void)motion_lock();		/* motors locked off until RESET */
		beep(400U, 800U);
		app_log((UB*)"ctrl: FAULT STOP, mask 0x%02x\n", mission.fault_mask);
		break;
	default:
		break;
	}
}

/* ---------------------------------------------------------------------
 *  Obstacle bypass helpers
 * --------------------------------------------------------------------- */
LOCAL void bypass_failed(const char *why)
{
	mission.bypass_fails++;
	publish_mission();
	app_log((UB*)"ctrl: bypass attempt failed (%s), %d of 2\n", why,
		  mission.bypass_fails);
	if (mission.bypass_fails >= 2U) {
		mission.fault_mask |= (UB)FLT_BYPASS;
		enter(VS_FAULT_STOP);
		return;
	}
	/* Reverse a short distance, then rescan once */
	steps[0].op = STEP_MOVE;
	steps[0].arg = -CTRL_REVERSE_MM;
	bp_phase = BP_REVERSING;
	start_steps(1);
}

LOCAL void steps_done(void)
{
	switch ((VEHICLE_STATE)mission.state) {
	case VS_BARCODE_MANOEUVRE:
		enter(VS_LINE_FOLLOWING);
		break;
	case VS_OBSTACLE_BYPASS:
		if (bp_phase == BP_REVERSING) {
			bp_phase = BP_WAIT_PROFILE;
			scan_rescan_request++;		/* writer: ctrl_task only */
		} else {
			UW t = app_time_ms() - bypass_t0;
			app_log((UB*)"ctrl: bypass complete in %d ms%s\n", t,
				  (t > BYPASS_TARGET_MS) ? " (over the NFR2 limit)" : "");
			mission.bypass_fails = 0;
			enter(VS_LINE_FOLLOWING);
		}
		break;
	case VS_INITIALISING:
		if (init_phase == INIT_CAL_SWEEP) {
			line_cal_request = LINE_CAL_APPLY;
			init_phase = INIT_CAL_APPLY;
			phase_t0 = app_time_ms();
		} else if (init_phase == INIT_ALIGN) {
			/* all search turns done (or the line was found and the
			   rest skipped) */
			if ((line_unpack_flags(line_latest) & LINE_F_ON_LINE) != 0U) {
				app_log((UB*)"ctrl: car is on the line, ready\n");
			} else {
				app_log((UB*)"ctrl: line NOT found within %d deg after calibration: place the car on the line\n",
					ALIGN_SEARCH_DEG);
			}
			init_phase = INIT_DONE;
			enter(VS_READY);
		}
		break;
	default:
		break;
	}
}

/* ---------------------------------------------------------------------
 *  Event handlers
 * --------------------------------------------------------------------- */
LOCAL void on_command(REMOTE_CMD c)
{
	VEHICLE_STATE	cur = (VEHICLE_STATE)mission.state;
	VEHICLE_STATE	nxt = ctrl_on_command(cur, c);

	last_remote = (UB)c;
	if ((c == RC_START) && (app_barcode_test != 0U)) {
		app_log((UB*)"ctrl: START refused: barcode TEST mode is on (switch it off on the dashboard)\n");
		beep(400U, 150U);
		return;
	}
	if (c == RC_RESET) {
		if ((cur == VS_FAULT_STOP) || (cur == VS_STOPPED) ||
		    (cur == VS_MISSION_COMPLETE) || (cur == VS_READY)) {
			mission.fault_mask = 0;		/* latched until RESET */
			mission.bypass_fails = 0;
			pending_letter = 0;		/* forget a stored barcode */
			hump_peak_max = 0;
			hump_count = 0;
			obs_valid = FALSE;
			(void)motion_reset();
		}
	}
	if (nxt != cur) {
		if (c == RC_START) {
			beep(2000U, 100U);
		}
		enter(nxt);
	} else if (c != RC_NONE) {
		app_log((UB*)"ctrl: command %d ignored in state %d\n", c, cur);
	}
}

LOCAL void on_faults(UINT f)
{
	VEHICLE_STATE	cur = (VEHICLE_STATE)mission.state;
	VEHICLE_STATE	nxt = ctrl_on_faults(cur, f);
	UINT		latch = f;

	if (((f & FLT_LINE_LOST) != 0U) && (nxt != VS_STOPPED) && (cur != VS_FAULT_STOP)) {
		latch &= ~FLT_LINE_LOST;	/* deliberate: turning or bypassing */
	}
	mission.fault_mask |= (UB)latch;
	app_log((UB*)"ctrl: fault 0x%02x in state %d -> state %d\n", f, cur, nxt);
	publish_mission();
	if (nxt != cur) {
		enter(nxt);
	}
}

LOCAL void on_motion(const MOTION_STATUS *m)
{
	if ((mission.state == VS_INITIALISING) && (init_phase == INIT_SELFTEST) &&
	    ((INT)m->seq == wait_seq)) {
		wait_seq = -1;
		if (m->event == MS_SELFTEST_PASS) {
			app_log((UB*)"ctrl: motor + encoder self-check passed\n");
			if (APP_CAL_SWEEP) {
				line_cal_request = LINE_CAL_RECORD;
				steps[0].op = STEP_TURN; steps[0].arg = CAL_SWEEP_DEG;
				steps[1].op = STEP_TURN; steps[1].arg = -2 * CAL_SWEEP_DEG;
				steps[2].op = STEP_TURN; steps[2].arg = CAL_SWEEP_DEG;
				init_phase = INIT_CAL_SWEEP;
				start_steps(3);
			} else {
				init_phase = INIT_DONE;
				enter(VS_READY);
			}
		}	/* FAIL: FLT_ENCODER arrives as a fault bit -> FAULT STOP */
		return;
	}
	if ((wait_seq >= 0) && ((INT)m->seq == wait_seq) &&
	    ((m->event == MS_DONE) || (m->event == MS_STOPPED))) {
		if (search_active && (m->event == MS_DONE) &&
		    (mission.state != VS_INITIALISING) &&
		    (steps[step_i].op == STEP_SEARCH)) {
			/* Searched the whole distance without finding the line */
			app_log((UB*)"ctrl: line not found after bypass\n");
			mission.fault_mask |= (UB)FLT_LINE_LOST;
			enter(VS_STOPPED);
			return;
		}
		if (pass_active) {
			app_log((UB*)"ctrl: pass leg reached its maximum (obstacle end not seen)\n");
		}
		step_i++;
		post_step();
	}
}

LOCAL void on_obstacle(const OBSTACLE_MSG *m)
{
	VEHICLE_STATE	cur = (VEHICLE_STATE)mission.state;
	BOOL		forward_leg;

	if (m->kind == OB_ALERT) {
		if (ctrl_is_active(cur) && (cur != VS_OBSTACLE_BYPASS)) {
			enter(VS_OBSTACLE_BYPASS);
		}
		return;
	}
	if (cur != VS_OBSTACLE_BYPASS) {
		return;
	}
	switch (m->kind) {
	case OB_PROFILE:
		if (bp_phase != BP_WAIT_PROFILE) {
			break;
		}
		last_obs = *m;
		obs_valid = TRUE;
		if (m->closest_mm == 0U) {
			app_log((UB*)"ctrl: sweep found no obstacle, back to line following\n");
			enter(VS_LINE_FOLLOWING);
			break;
		}
		mission.bypass_side = m->side;
		publish_mission();
		bp_phase = BP_LEGS;
		start_steps(ctrl_plan_bypass(m->side, m->offset_mm, m->depth_mm, steps));
		break;
	case OB_NO_CLEARANCE:
		if (bp_phase != BP_WAIT_PROFILE) {
			break;
		}
		if (!bp_rescanned) {
			bp_rescanned = TRUE;	/* reverse and rescan once first */
			steps[0].op = STEP_MOVE;
			steps[0].arg = -CTRL_REVERSE_MM;
			bp_phase = BP_REVERSING;
			start_steps(1);
		} else {
			bypass_failed("no side clear");
		}
		break;
	case OB_SENSOR_FAULT:
		bypass_failed("sensor fault");
		break;
	case OB_CLEARANCE_LOST:
		/* only while driving FORWARD: the guard also sees the
		   obstacle while the car backs away from it */
		forward_leg = (bp_phase == BP_LEGS) && (step_i < nsteps) &&
			      (steps[step_i].op != STEP_TURN) &&
			      (steps[step_i].arg > 0);
		if (forward_leg) {
			bypass_failed("clearance lost");
		}
		break;
	default:
		break;
	}
}

LOCAL void on_terrain(const TERRAIN_MSG *m)
{
	VEHICLE_STATE	cur = (VEHICLE_STATE)mission.state;

	switch (m->kind) {
	case TR_HUMP_START:
		if (cur == VS_LINE_FOLLOWING) {
			enter(VS_HUMP_CROSSING);
		}
		break;
	case TR_HUMP_END:
		if (hump_count < 255U) {
			hump_count++;
		}
		if (m->peak_01mm > hump_peak_max) {
			hump_peak_max = m->peak_01mm;
		}
		/* fall through */
	case TR_HUMP_DISCARD:
		if (cur == VS_HUMP_CROSSING) {
			enter(VS_LINE_FOLLOWING);
		}
		break;
	default:		/* TR_IMPACT arrives as FLT_IMPACT */
		break;
	}
}

LOCAL const char *bc_name(BARCODE_CMD c)
{
	const char	*p;

	switch (c) {
	case BC_GO_STRAIGHT:	p = "FORWARD";		break;
	case BC_U_TURN:		p = "U-TURN";		break;
	case BC_TURN_LEFT:	p = "LEFT";		break;
	case BC_TURN_RIGHT:	p = "RIGHT";		break;
	default:		p = "not in the map";	break;
	}
	return p;
}

/* Carry out the stored letter at a cross-junction (demo steps 3-5) */
LOCAL void on_cross_junction(const NAV_MSG *m)
{
	UW		map = app_barcode_map;		/* one copy (IF-15) */
	BARCODE_CMD	cmd;
	W		past_mm = (W)((motion_odo_path_01mm / 10U) - m->path_mm);
	INT		n;

	if (pending_letter == 0U) {
		app_log((UB*)"ctrl: cross-junction, no barcode stored: straight on\n");
		return;
	}
	if (mission.state != VS_LINE_FOLLOWING) {
		app_log((UB*)"ctrl: cross-junction in state %d: barcode '%c' kept for the next one\n",
			mission.state, (INT)pending_letter);
		return;
	}
	cmd = app_barcode_lookup(map, pending_letter);
	app_log((UB*)"ctrl: cross-junction: barcode '%c' -> %s (sensors %d mm past the middle)\n",
		(INT)pending_letter, bc_name(cmd), past_mm);
	if (map == 0U) {
		app_log((UB*)"ctrl: NO BARCODE MAP received yet (send it from the dashboard)\n");
	}
	last_letter = pending_letter;
	last_bc = (UB)cmd;
	pending_letter = 0;			/* used up: next barcode next */
	n = ctrl_plan_junction(cmd, past_mm, steps);
	if (n > 0) {
		nsteps = n;
		beep(2000U, 80U);
		enter(VS_BARCODE_MANOEUVRE);	/* back to LINE FOLLOWING after */
	}
}

LOCAL void on_nav(const NAV_MSG *m)
{
	BARCODE_CMD	cmd;

	switch (m->kind) {
	case NAV_BARCODE:
		/* Telemetry shows every read; the movement is looked up again
		   at the junction, so a map sent after the read still applies */
		cmd = app_barcode_lookup(app_barcode_map, m->letter);
		last_letter = m->letter;
		last_bc = (cmd == BC_UNRECOGNISED) ? m->cmd : (UB)cmd;
		beep(2500U, 80U);			/* acknowledgement tone */
		if ((mission.state != VS_LINE_FOLLOWING) &&
		    (mission.state != VS_HUMP_CROSSING)) {
			break;
		}
		if ((cmd == BC_UNRECOGNISED) && (m->cmd == BC_END_OF_COURSE)) {
			enter(VS_MISSION_COMPLETE);
			break;
		}
		if (pending_letter != 0U) {
			app_log((UB*)"ctrl: barcode '%c' replaces stored '%c' (no junction in between)\n",
				(INT)m->letter, (INT)pending_letter);
		}
		pending_letter = m->letter;
		app_log((UB*)"ctrl: barcode '%c' stored (%s): carried out at the next cross-junction\n",
			(INT)m->letter, bc_name(cmd));
		break;
	case NAV_JUNCTION_CROSS:
		on_cross_junction(m);
		break;
	case NAV_READ_REJECTED:
		if (app_barcode_test != 0U) {
			beep(600U, 250U);		/* test mode: low tone = bad read */
		}
		break;
	case NAV_JUNCTION_RIGHT:
		/* Isolated wide stripe under the barcode sensor alone */
		app_log((UB*)"ctrl: single stripe on the barcode sensor at %d mm\n", m->path_mm);
		break;
	default:			/* LINE_LOST arrives as FLT_LINE_LOST */
		break;
	}
}

/* ---------------------------------------------------------------------
 *  Periodic work
 * --------------------------------------------------------------------- */
LOCAL void init_step(void)
{
	T_RFLG	rflg;
	UW	now = app_time_ms();

	switch (init_phase) {
	case INIT_WAIT_SENSORS:
		(void)tk_ref_flg(flg_sys, &rflg);
		if (((rflg.flgptn & EVF_IMU_READY) != 0U) &&
		    ((rflg.flgptn & EVF_LINE_READY) != 0U)) {
			init_phase = INIT_SELFTEST;
			wait_seq = motion_selftest();
		} else if ((now - phase_t0) > INIT_SENSOR_TMO_MS) {
			app_log((UB*)"ctrl: sensors not ready at start: IMU %s, IR %s\n",
				((rflg.flgptn & EVF_IMU_READY) != 0U) ? "ok" : "NOT READY",
				((rflg.flgptn & EVF_LINE_READY) != 0U) ? "ok" : "NOT READY");
			on_faults(((rflg.flgptn & EVF_IMU_READY) == 0U) ? FLT_IMU : FLT_ENCODER);
		} else {
			/* keep waiting */
		}
		break;
	case INIT_CAL_APPLY:
		if ((now - phase_t0) > 200U) {	/* line_task applies within 20 ms */
			line_cal_request = LINE_CAL_IDLE;
			/* The sweep may end a few degrees off: turn back onto the
			   line (left first, then right, then back to the start) */
			init_phase = INIT_ALIGN;
			if ((line_unpack_flags(line_latest) & LINE_F_ON_LINE) != 0U) {
				start_steps(0);		/* already on it */
			} else {
				app_log((UB*)"ctrl: off the line after calibration, searching\n");
				steps[0].op = STEP_SEARCH_TURN; steps[0].arg = ALIGN_SEARCH_DEG;
				steps[1].op = STEP_SEARCH_TURN; steps[1].arg = -2 * ALIGN_SEARCH_DEG;
				steps[2].op = STEP_TURN;        steps[2].arg = ALIGN_SEARCH_DEG;
				start_steps(3);
			}
		}
		break;
	default:
		break;
	}
}

LOCAL void send_telemetry(void)
{
	TLM_SNAPSHOT	t = {0};
	UW		lw = line_latest;
	UW		iw = imu_status_word;
	UW		sw = scan_status_word;
	T_RFLG		rflg;

	(void)tk_ref_flg(flg_sys, &rflg);
	t.status = (((UW)mission.state & TLM_STATE_MASK) << TLM_STATE_SHIFT) |
		   (((UW)last_bc & TLM_BARCODE_MASK) << TLM_BARCODE_SHIFT) |
		   (((UW)last_remote & TLM_REMOTE_MASK) << TLM_REMOTE_SHIFT) |
		   (((UW)line_unpack_flags(lw) & TLM_LINEF_MASK) << TLM_LINEF_SHIFT) |
		   ((sw & TLM_SERVO_MASK) << TLM_SERVO_SHIFT) |
		   (((rflg.flgptn & EVF_HUMP_ACTIVE) != 0U ? 1UL : 0UL) << TLM_HUMP_SHIFT) |
		   ((obs_valid ? 1UL : 0UL) << TLM_OBST_SHIFT) |
		   ((app_watchdog_caused_reboot() ? 1UL : 0UL) << TLM_WDT_SHIFT) |
		   (((UW)mission.bypass_side & TLM_SIDE_MASK) << TLM_SIDE_SHIFT) |
		   (((UW)mission.bypass_fails & TLM_FAILS_MASK) << TLM_FAILS_SHIFT) |
		   ((iw & TLM_MEVENT_MASK) << TLM_MEVENT_SHIFT) |
		   ((comms_state_word == CS_ONLINE ? 1UL : 0UL) << TLM_LINK_SHIFT);
	t.fault_mask = mission.fault_mask;
	t.letter = last_letter;
	t.hump_count = hump_count;
	t.speed_l_mm_s = motion_speed_left();
	t.speed_r_mm_s = motion_speed_right();
	t.enc_l = motion_enc_count[0];
	t.enc_r = motion_enc_count[1];
	t.path_mm = motion_odo_path_01mm / 10U;
	t.line_err = (H)line_unpack_err(lw);
	t.hump_peak_01mm = hump_peak_max;
	t.obs_width_mm = obs_valid ? last_obs.width_mm : 0U;
	t.obs_closest_mm = obs_valid ? last_obs.closest_mm : 0U;
	t.uptime_ms = app_time_ms();
	t.tlm_dropped = tlm_dropped;

	/* Never block: a full queue drops the newest snapshot (6.1) */
	if (tk_snd_mbf(mbf_telemetry, &t, sizeof(t), TMO_POL) < E_OK) {
		tlm_dropped++;
	}
}

/* Debounced START/STOP buttons: act on the press edge */
LOCAL void poll_buttons(void)
{
	static UB	hist_start, hist_stop;
	static BOOL	down_start, down_stop;

	hist_start = (UB)((hist_start << 1) | (drv_button_raw(BTN_START_PIN) ? 1U : 0U));
	hist_stop = (UB)((hist_stop << 1) | (drv_button_raw(BTN_STOP_PIN) ? 1U : 0U));

	if (((hist_stop & 0x3U) == 0x3U) && !down_stop) {
		down_stop = TRUE;
		on_command(RC_STOP);		/* STOP first */
	} else if ((hist_stop & 0x3U) == 0U) {
		down_stop = FALSE;
	}
	if (((hist_start & 0x3U) == 0x3U) && !down_start) {
		down_start = TRUE;
		/* START when stopped/ready; RESET when in FAULT STOP or done */
		if ((mission.state == VS_FAULT_STOP) || (mission.state == VS_MISSION_COMPLETE)) {
			on_command(RC_RESET);
		} else {
			on_command(RC_START);
		}
	} else if ((hist_start & 0x3U) == 0U) {
		down_start = FALSE;
	}
}

/* ---------------------------------------------------------------------
 *  The task
 * --------------------------------------------------------------------- */
EXPORT void ctrl_task(INT stacd, void *exinf)
{
	UINT		ptn;
	ER		er;
	REMOTE_MSG	rm;
	MOTION_STATUS	ms;
	OBSTACLE_MSG	om;
	TERRAIN_MSG	tm;
	NAV_MSG		nm;
	UW		now;
	UW		last_tlm = 0;
	UW		last_btn = 0;
	UW		lw;

	(void)stacd; (void)exinf;

	drv_buttons_init();
	drv_buzzer_init();
	if (drv_neopixel_init() != E_OK) {
		app_log((UB*)"ctrl: no PIO space for the NeoPixel\n");
	}
	app_watchdog_start(WDT_TIMEOUT_MS);
	mission.state = (UB)VS_INITIALISING;
	enter(VS_INITIALISING);

	while (1) {
		ptn = 0;
		er = tk_wai_flg(flg_sys, EVF_CTRL_WAKE, TWF_ORW, &ptn, S5_CTRL_TICK_MS);
		app_watchdog_feed();
		if (er == E_OK) {
			/* clear exactly what we saw, before draining the queues */
			(void)tk_clr_flg(flg_sys, ~(ptn & EVF_CTRL_WAKE));
		}
		now = app_time_ms();

		/* 1. Commands first: STOP must never wait behind navigation */
		while (tk_rcv_mbf(mbf_remote, &rm, TMO_POL) == (INT)sizeof(rm)) {
			on_command((REMOTE_CMD)(rm.cmd & 0x3U));
		}
		if ((now - last_btn) >= S5_CTRL_TICK_MS) {
			last_btn = now;
			poll_buttons();
		}

		/* 2. Faults */
		if ((ptn & FLT_ALL) != 0U) {
			on_faults(ptn & FLT_ALL);
		}

		/* 3-6. Reports */
		while (tk_rcv_mbf(mbf_motion_status, &ms, TMO_POL) == (INT)sizeof(ms)) {
			on_motion(&ms);
		}
		while (tk_rcv_mbf(mbf_obstacle, &om, TMO_POL) == (INT)sizeof(om)) {
			on_obstacle(&om);
		}
		while (tk_rcv_mbf(mbf_terrain, &tm, TMO_POL) == (INT)sizeof(tm)) {
			on_terrain(&tm);
		}
		while (tk_rcv_mbf(mbf_nav, &nm, TMO_POL) == (INT)sizeof(nm)) {
			on_nav(&nm);
		}

		/* 7. Periodic */
		if (mission.state == VS_INITIALISING) {
			init_step();
		}
		if (pass_active && (scan_pass_seq != pass_seq0)) {
			/* obstacle is behind: drive only the rest, then turn in */
			UW extra = scan_pass_extra_mm;

			pass_active = FALSE;
			scan_pass_watch = 0;
			app_log((UB*)"ctrl: obstacle passed, %d mm more then turning in\n", extra);
			wait_seq = (extra >= 10U) ? moveForwardAt((W)extra, CTRL_BYPASS_MM_S) : stop();
		}
		/* Barcode TEST mode switched on while driving: stop at once */
		if ((app_barcode_test != 0U) && ctrl_is_active((VEHICLE_STATE)mission.state)) {
			app_log((UB*)"ctrl: barcode TEST mode on: stopping\n");
			enter(VS_STOPPED);
		}
		if (search_active) {
			lw = line_latest;
			if ((line_unpack_flags(lw) & LINE_F_ON_LINE) != 0U) {
				/* line found: stop this leg, then re-align */
				search_active = FALSE;
				if (mission.state == VS_INITIALISING) {
					step_i = nsteps - 1;	/* skip the rest of the search */
				}
				wait_seq = stop();
			}
		}
		if ((now - last_tlm) >= TLM_PERIOD_MS) {
			last_tlm = now;
			send_telemetry();
		}
		if ((buzz_off_at != 0U) && ((W)(now - buzz_off_at) >= 0)) {
			buzz_off_at = 0;
			drv_buzzer_tone(0);
		}
	}
}
