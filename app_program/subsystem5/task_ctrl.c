/*
 *	task_ctrl.c
 *	Subsystem 5 -- ctrl_task, the Vehicle Controller (COMPLETE VERSION).
 *
 *	Highest application priority, event-driven (plus a 50 ms tick for the
 *	buttons, timers, telemetry and the watchdog).  The only task that
 *	writes the mission state, sends motion commands and builds telemetry.
 *
 *	System statechart (Section 5) with the complete-version additions:
 *	  INITIALISING -> READY when every subsystem is ready.
 *	  READY --START--> calibration sweep over the line (first run only)
 *	      --> LINE FOLLOWING.
 *	  LINE FOLLOWING --barcode--> BARCODE MANOEUVRE: executed at the next
 *	      right junction, or after S5_EXEC_MAX_MM if none is seen.
 *	  LINE FOLLOWING --hump--> HUMP CROSSING (speed limit).
 *	  active --obstacle alert--> OBSTACLE BYPASS: stop, profile, then
 *	      the bypass sequence (DRAFT of the sub-statechart still to be
 *	      reviewed by Buddies 2, 3, 5):
 *	        turn 90 to the side, out by (reach + half car + margin),
 *	        turn back, along (closest + depth + car length), turn toward
 *	        the line, move until the line is seen, turn to align, follow.
 *	      Side blocked: reverse 100 mm and rescan once.  A failed attempt
 *	      (no line found, sensor fault, still blocked) counts; the second
 *	      failure is FAULT STOP (FAILED BYPASS).
 *	  LINE LOST --> LINE RECOVERY: turn toward the last known side up to
 *	      45 deg, then the other way up to 90 deg; if still lost: STOPPED.
 *	  active --STOP (button or MQTT), impact--> STOPPED.
 *	  stall / encoder / IMU fault / 2 failed bypasses --> FAULT STOP.
 *	  RESET (both buttons 1 s, or MQTT) --> INITIALISING.
 */

#include "subsystem5.h"
#include "drv_status.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"

typedef enum {
	BX_NONE = 0,
	BX_WAIT_JUNCTION,
	BX_FORWARD,
	BX_TURN
} BX_STAGE;

typedef enum {
	BP_NONE = 0,
	BP_WAIT_PROFILE,
	BP_REVERSE,
	BP_LEG1_TURN_OUT,
	BP_LEG2_OUT,
	BP_LEG3_TURN_ALONG,
	BP_LEG4_ALONG,
	BP_LEG5_TURN_IN,
	BP_LEG6_SEARCH,
	BP_LEG7_ALIGN
} BP_STAGE;

IMPORT MISSION_STATE	s5_mission;	/* subsystem5.c, under mtx_mission */

LOCAL VEHICLE_STATE	st = VS_INITIALISING;
LOCAL UW		st_since_ms = 0U;
LOCAL BOOL		calibrated = FALSE;
LOCAL INT		cal_stage = 0;
LOCAL BX_STAGE		bx_stage = BX_NONE;
LOCAL BARCODE_CMD	bx_cmd = BC_NONE;
LOCAL W			bx_from_mm = 0;
LOCAL BP_STAGE		bp_stage = BP_NONE;
LOCAL W			bp_side = 1;		/* +1 left, -1 right */
LOCAL W			bp_lateral = 0;
LOCAL W			bp_along = 0;
LOCAL BOOL		bp_reversed = FALSE;
LOCAL W			bp_failures = 0;
LOCAL INT		rec_stage = 0;
LOCAL W			rec_side = 1;
LOCAL W			last_barcode = BC_NONE;
LOCAL W			last_letter = 0;
LOCAL BOOL		hump_active = FALSE;
LOCAL UW		tel_seq = 0U;
LOCAL BOOL		wdg_started = FALSE;

/* ------------------------------------------------------------------ */
LOCAL void ctrl_publish_mission(void)
{
	if (tk_loc_mtx(mtx_mission, 20) == E_OK) {
		s5_mission.state = st;
		s5_mission.fault_mask = s5_fault_mask;
		s5_mission.last_barcode = last_barcode;
		s5_mission.bypass_failures = bp_failures;
		s5_mission.hump_active = hump_active;
		(void)tk_unl_mtx(mtx_mission);
	}
}

LOCAL void ctrl_enter(VEHICLE_STATE s)
{
	if (s != st) {
		tm_printf((UB *)"[CTRL] %s -> %s\n", s5_state_name[st], s5_state_name[s]);
	}
	st = s;
	s5_vehicle_state = s;
	st_since_ms = s5_now_ms();
	status_show_state(s);
	ctrl_publish_mission();
}

LOCAL BOOL ctrl_active(void)
{
	return ((st == VS_LINE_FOLLOWING) || (st == VS_BARCODE_MANOEUVRE) ||
		(st == VS_HUMP_CROSSING) || (st == VS_OBSTACLE_BYPASS) ||
		(st == VS_LINE_RECOVERY)) ? TRUE : FALSE;
}

LOCAL void ctrl_clear_stages(void)
{
	bx_stage = BX_NONE;
	bp_stage = BP_NONE;
	rec_stage = 0;
	cal_stage = 0;
	s3_cal_request = CAL_IDLE;
	s5_scan_request = SCAN_REQ_WATCH;
}

LOCAL void ctrl_follow(void)
{
	ctrl_clear_stages();
	(void)motion_set_speed_limit((hump_active == TRUE) ? S5_HUMP_MM_S : 0);
	(void)motion_follow_line(S5_CRUISE_MM_S);
	ctrl_enter((hump_active == TRUE) ? VS_HUMP_CROSSING : VS_LINE_FOLLOWING);
}

LOCAL void ctrl_stop(VEHICLE_STATE s)
{
	(void)motion_stop();
	ctrl_clear_stages();
	ctrl_enter(s);
}

LOCAL void ctrl_fault_stop(UW bit)
{
	s5_fault_mask |= bit;
	status_beep(800U);
	ctrl_stop(VS_FAULT_STOP);
}

LOCAL void ctrl_turn(W deg)		/* + = left */
{
	if (deg >= 0) {
		(void)motion_turn_left(deg);
	} else {
		(void)motion_turn_right(-deg);
	}
}

LOCAL void ctrl_reset(void)
{
	tm_printf((UB *)"[CTRL] RESET\n");
	(void)motion_stop();
	(void)motion_reset_fault();
	(void)motion_set_speed_limit(0);
	s5_fault_mask = 0U;
	bp_failures = 0;
	ctrl_clear_stages();
	ctrl_enter(VS_INITIALISING);
}

LOCAL void ctrl_start(void)
{
	if ((st != VS_READY) && (st != VS_STOPPED)) {
		return;
	}
	if ((calibrated == FALSE) && (S5_CAL_SWEEP != 0)) {
		/* calibration sweep: left, right, back to the centre */
		ctrl_clear_stages();
		cal_stage = 1;
		s3_cal_request = CAL_RECORD;
		ctrl_enter(VS_INITIALISING);
		ctrl_turn(S5_CAL_SWEEP_DEG);
	} else {
		ctrl_follow();
	}
}

/* ------------------------------------------------------------------ */
/* Bypass */
LOCAL void bp_fail(const char *why)
{
	bp_failures++;
	tm_printf((UB *)"[CTRL] bypass attempt failed (%s), failures = %d\n", why, (INT)bp_failures);
	if (bp_failures >= S5_BYPASS_MAX_FAILS) {
		ctrl_fault_stop(FAULT_FAILED_BYPASS);
		return;
	}
	(void)motion_stop();
	bp_reversed = FALSE;
	bp_stage = BP_WAIT_PROFILE;
	s5_scan_request = SCAN_REQ_PROFILE;
	s5_scan_request_seq++;
}

LOCAL void bp_plan(const OBSTACLE_MSG *p_m)
{
	W	reach;

	if ((p_m->distance_mm < 0) || (p_m->side == SIDE_NONE)) {
		ctrl_follow();		/* nothing there any more */
		return;
	}
	if (p_m->side == SIDE_BLOCKED) {
		if (bp_reversed == FALSE) {
			bp_reversed = TRUE;
			bp_stage = BP_REVERSE;
			(void)motion_move_backward(S5_REVERSE_MM);
		} else {
			bp_fail("both sides blocked");
		}
		return;
	}
	bp_side = (p_m->side == SIDE_LEFT) ? 1 : -1;
	reach = (bp_side > 0) ? p_m->left_mm : p_m->right_mm;
	if (reach < 0) {
		reach = 0;	/* obstacle ends before the centre line */
	}
	bp_lateral = reach + S5_CAR_HALF_WIDTH_MM + S5_BYPASS_MARGIN_MM;
	bp_along = p_m->distance_mm + S5_BYPASS_DEPTH_MM + S5_CAR_LENGTH_MM;
	tm_printf((UB *)"[CTRL] bypass %s: out %d mm, along %d mm\n",
		  (bp_side > 0) ? "LEFT" : "RIGHT", (INT)bp_lateral, (INT)bp_along);
	s5_scan_request = SCAN_REQ_HOLD;
	bp_stage = BP_LEG1_TURN_OUT;
	ctrl_turn(bp_side * 90);
}

LOCAL void bp_next(BOOL line_found)
{
	switch (bp_stage) {
	case BP_REVERSE:
		bp_stage = BP_WAIT_PROFILE;
		s5_scan_request = SCAN_REQ_PROFILE;
		s5_scan_request_seq++;
		break;
	case BP_LEG1_TURN_OUT:
		bp_stage = BP_LEG2_OUT;
		(void)motion_move_forward(bp_lateral);
		break;
	case BP_LEG2_OUT:
		bp_stage = BP_LEG3_TURN_ALONG;
		ctrl_turn(-bp_side * 90);
		break;
	case BP_LEG3_TURN_ALONG:
		bp_stage = BP_LEG4_ALONG;
		(void)motion_move_forward(bp_along);
		break;
	case BP_LEG4_ALONG:
		bp_stage = BP_LEG5_TURN_IN;
		ctrl_turn(-bp_side * 90);
		break;
	case BP_LEG5_TURN_IN:
		bp_stage = BP_LEG6_SEARCH;
		(void)motion_move_until_line(bp_lateral + S5_SEARCH_EXTRA_MM);
		break;
	case BP_LEG6_SEARCH:
		if (line_found == TRUE) {
			bp_stage = BP_LEG7_ALIGN;
			ctrl_turn(bp_side * 90);
		} else {
			bp_fail("line not found");
		}
		break;
	case BP_LEG7_ALIGN:
		tm_printf((UB *)"[CTRL] bypass done, line reacquired\n");
		ctrl_follow();
		break;
	case BP_WAIT_PROFILE:
	case BP_NONE:
	default:
		break;
	}
}

/* ------------------------------------------------------------------ */
LOCAL void ctrl_on_motion(const MOTION_STATUS_MSG *p_m)
{
	if (p_m->kind != MST_DONE) {
		return;		/* stall / encoder faults arrive as fault bits */
	}
	if ((st == VS_INITIALISING) && (cal_stage > 0)) {
		cal_stage++;
		if (cal_stage == 2) {
			ctrl_turn(-2 * S5_CAL_SWEEP_DEG);
		} else if (cal_stage == 3) {
			ctrl_turn(S5_CAL_SWEEP_DEG);
		} else {
			s3_cal_request = CAL_FINISH;	/* result checked in timers */
		}
	} else if (st == VS_BARCODE_MANOEUVRE) {
		if (bx_stage == BX_FORWARD) {
			bx_stage = BX_TURN;
			if (bx_cmd == BC_TURN_LEFT) {
				ctrl_turn(90);
			} else if (bx_cmd == BC_TURN_RIGHT) {
				ctrl_turn(-90);
			} else {
				ctrl_follow();
			}
		} else if (bx_stage == BX_TURN) {
			ctrl_follow();
		} else {
			;
		}
	} else if (st == VS_OBSTACLE_BYPASS) {
		bp_next(p_m->line_found);
	} else if (st == VS_LINE_RECOVERY) {
		if (p_m->line_found == TRUE) {
			tm_printf((UB *)"[CTRL] line recovered\n");
			ctrl_follow();
		} else if (rec_stage == 1) {
			rec_stage = 2;
			(void)motion_turn_until_line(-rec_side * (S5_RECOVER_DEG_1 + S5_RECOVER_DEG_2));
		} else {
			s5_fault_mask |= FAULT_LINE_LOST;
			ctrl_stop(VS_STOPPED);
		}
	} else {
		;
	}
}

LOCAL void ctrl_execute_barcode(void)
{
	if (bx_cmd == BC_U_TURN) {
		bx_stage = BX_TURN;
		ctrl_turn(180);
	} else {
		bx_stage = BX_FORWARD;
		(void)motion_move_forward(S5_EXEC_FWD_MM);
	}
}

LOCAL void ctrl_on_nav(const NAV_MSG *p_n)
{
	switch (p_n->kind) {
	case NAV_BARCODE:
		last_barcode = p_n->cmd;
		last_letter = p_n->letter;
		if ((st != VS_LINE_FOLLOWING) && (st != VS_HUMP_CROSSING)) {
			break;
		}
		status_beep(80U);
		if (p_n->cmd == BC_END_OF_COURSE) {
			ctrl_stop(VS_MISSION_COMPLETE);
		} else if ((p_n->cmd == BC_UNRECOGNISED) || (p_n->cmd == BC_NONE)) {
			tm_printf((UB *)"[CTRL] barcode '%c' not a command: carry on\n", (INT)p_n->letter);
		} else {
			bx_cmd = p_n->cmd;
			bx_from_mm = p_n->distance_mm;
			bx_stage = BX_WAIT_JUNCTION;
			ctrl_enter(VS_BARCODE_MANOEUVRE);	/* keeps following */
		}
		ctrl_publish_mission();
		break;
	case NAV_JUNCTION_RIGHT:
		if ((st == VS_BARCODE_MANOEUVRE) && (bx_stage == BX_WAIT_JUNCTION)) {
			ctrl_execute_barcode();
		}
		break;
	case NAV_LINE_LOST:
		if ((st == VS_LINE_FOLLOWING) || (st == VS_HUMP_CROSSING) ||
		    ((st == VS_BARCODE_MANOEUVRE) && (bx_stage == BX_WAIT_JUNCTION))) {
			W	err;
			UW	age;
			UW	flags;

			s3_line_get(&err, &age, &flags);
			rec_side = (err >= 0) ? 1 : -1;	/* + = line was on the left */
			ctrl_clear_stages();
			rec_stage = 1;
			ctrl_enter(VS_LINE_RECOVERY);
			(void)motion_turn_until_line(rec_side * S5_RECOVER_DEG_1);
		}
		break;
	case NAV_LINE_FOUND:
	default:
		break;
	}
}

LOCAL void ctrl_on_terrain(const TERRAIN_MSG *p_t)
{
	switch (p_t->kind) {
	case TER_HUMP_START:
		hump_active = TRUE;
		if (ctrl_active() == TRUE) {
			(void)motion_set_speed_limit(S5_HUMP_MM_S);
		}
		if (st == VS_LINE_FOLLOWING) {
			ctrl_enter(VS_HUMP_CROSSING);
		}
		break;
	case TER_HUMP_END:
	case TER_HUMP_DISCARDED:
		hump_active = FALSE;
		if (ctrl_active() == TRUE) {
			(void)motion_set_speed_limit(0);
		}
		if (st == VS_HUMP_CROSSING) {
			ctrl_enter(VS_LINE_FOLLOWING);
		}
		break;
	case TER_IMPACT:
		if (ctrl_active() == TRUE) {
			s5_fault_mask |= FAULT_IMPACT;
			status_beep(300U);
			ctrl_stop(VS_STOPPED);
		}
		break;
	case TER_IMU_FAULT:
	default:
		break;		/* arrives as a fault bit too */
	}
	ctrl_publish_mission();
}

LOCAL void ctrl_on_obstacle(const OBSTACLE_MSG *p_o)
{
	switch (p_o->kind) {
	case OBS_ALERT:
		if ((st == VS_LINE_FOLLOWING) || (st == VS_HUMP_CROSSING) ||
		    (st == VS_BARCODE_MANOEUVRE)) {
			tm_printf((UB *)"[CTRL] obstacle at %d mm\n", (INT)p_o->distance_mm);
			(void)motion_stop();
			ctrl_clear_stages();
			bp_reversed = FALSE;
			bp_stage = BP_WAIT_PROFILE;	/* scan_task profiles now */
			ctrl_enter(VS_OBSTACLE_BYPASS);
		}
		break;
	case OBS_PROFILE:
		if ((st == VS_OBSTACLE_BYPASS) && (bp_stage == BP_WAIT_PROFILE)) {
			bp_plan(p_o);
		}
		break;
	case OBS_CLEAR:
		if ((st == VS_OBSTACLE_BYPASS) && (bp_stage == BP_WAIT_PROFILE)) {
			ctrl_follow();
		}
		break;
	case OBS_SENSOR_FAULT:
	default:
		if ((st == VS_OBSTACLE_BYPASS) && (bp_stage == BP_WAIT_PROFILE)) {
			bp_fail("ultrasonic sensor fault");	/* counts toward FAULT STOP */
		}
		break;
	}
}

LOCAL void ctrl_on_remote(const REMOTE_MSG *p_r)
{
	switch (p_r->cmd) {
	case RC_STOP:			/* forwarded immediately, any state */
		if ((ctrl_active() == TRUE) || (st == VS_INITIALISING)) {
			ctrl_stop(VS_STOPPED);
		}
		break;
	case RC_START:
		ctrl_start();
		break;
	case RC_RESET:
		ctrl_reset();
		break;
	case RC_NONE:
	default:
		break;
	}
}

LOCAL void ctrl_on_faults(UW faults)
{
	UW	stop_now = faults & (FAULT_STALL | FAULT_ENCODER |
				     ((S5_REQUIRE_IMU != 0) ? FAULT_IMU : 0U));

	s5_fault_mask |= faults;	/* latched until RESET */
	if (stop_now != 0U) {
		ctrl_fault_stop(stop_now);
	}
	/* NO ECHO is acted on through the obstacle profile; WIFI LOSS is
	 * reported only, the car keeps driving. */
	ctrl_publish_mission();
}

/* ------------------------------------------------------------------ */
LOCAL BOOL btn_down(UINT pin)
{
	return (gpio_get_val(pin) == 0U) ? TRUE : FALSE;
}

LOCAL void ctrl_buttons(void)
{
	static BOOL	prev_start = FALSE;
	static BOOL	prev_stop = FALSE;
	static UW	both_since = 0U;
	BOOL		start = btn_down(S5_PIN_BTN_START);
	BOOL		stop = btn_down(S5_PIN_BTN_STOP);
	UW		now = s5_now_ms();

	if ((start == TRUE) && (stop == TRUE)) {
		if (both_since == 0U) {
			both_since = now;
		} else if ((now - both_since) >= (UW)S5_RESET_HOLD_MS) {
			both_since = 0U;
			ctrl_reset();
		} else {
			;
		}
		prev_start = TRUE;
		prev_stop = TRUE;
		return;
	}
	both_since = 0U;
	if ((stop == TRUE) && (prev_stop == FALSE) &&
	    ((ctrl_active() == TRUE) || (cal_stage > 0))) {
		ctrl_stop(VS_STOPPED);
	}
	if ((start == FALSE) && (prev_start == TRUE)) {
		ctrl_start();		/* on release */
	}
	prev_start = start;
	prev_stop = stop;
}

LOCAL void ctrl_timers(void)
{
	UW	now = s5_now_ms();

	if ((st == VS_INITIALISING) && (cal_stage == 0)) {
		BOOL	imu_ok = ((s4_ok == TRUE) || (S5_REQUIRE_IMU == 0)) ? TRUE : FALSE;

		if ((s2_hw_ready == TRUE) && (s3_ready == TRUE) && (s5_scan_ready == TRUE) &&
		    (imu_ok == TRUE) && ((now - st_since_ms) > 1500U)) {
			ctrl_enter(VS_READY);
			tm_printf((UB *)"[CTRL] READY: put the car on the line, press START (GP20)\n");
			if ((wdg_started == FALSE) && (S5_WATCHDOG_ENABLE != 0)) {
				wdg_started = TRUE;
				tm_printf((UB *)"[CTRL] watchdog %s\n",
					  (watchdog_start(S5_WATCHDOG_MS) == TRUE) ? "armed" :
					  "NOT armed (watchdog tick not running)");
			}
		} else if ((now - st_since_ms) > (UW)S5_INIT_WAIT_MS) {
			tm_printf((UB *)"[CTRL] init timeout: motion=%d line=%d scan=%d imu=%d\n",
				  (INT)s2_hw_ready, (INT)s3_ready, (INT)s5_scan_ready, (INT)s4_ok);
			ctrl_fault_stop((imu_ok == FALSE) ? FAULT_IMU : FAULT_ENCODER);
		} else {
			;
		}
	}

	if ((st == VS_INITIALISING) && (cal_stage > 3) && (s3_cal_result != 0)) {
		s3_cal_request = CAL_IDLE;
		cal_stage = 0;
		if (s3_cal_result > 0) {
			calibrated = TRUE;
			ctrl_follow();
		} else {
			status_beep(500U);
			ctrl_enter(VS_READY);	/* keep defaults; START again to run */
			calibrated = TRUE;	/* do not sweep again */
		}
	}

	if ((st == VS_BARCODE_MANOEUVRE) && (bx_stage == BX_WAIT_JUNCTION) &&
	    ((s2_distance_mm - bx_from_mm) >= S5_EXEC_MAX_MM)) {
		ctrl_execute_barcode();		/* no junction seen: execute anyway */
	}
}

LOCAL void ctrl_send_telemetry(void)
{
	S1_TELEMETRY	t;
	W		err;
	UW		age;
	UW		flags;
	OBSTACLE_MSG	prof = s5_last_profile;

	s3_line_get(&err, &age, &flags);
	t.t_ms = s5_now_ms();
	t.seq = tel_seq++;
	t.state = (W)st;
	t.last_barcode = last_barcode;
	t.last_letter = last_letter;
	t.fault_mask = s5_fault_mask;
	t.speed_l_mm_s = s2_speed_l_mm_s;
	t.speed_r_mm_s = s2_speed_r_mm_s;
	t.enc_l = s2_enc_l;
	t.enc_r = s2_enc_r;
	t.distance_mm = s2_distance_mm;
	t.line_err = err;
	t.line_flags = flags;
	t.hump_active = (hump_active == TRUE) ? 1 : 0;
	t.hump_peak_mm = s4_peak_mm;
	t.obstacle_mm = s5_last_mm;
	t.obstacle_width_mm = prof.width_mm;
	t.bypass_side = (W)prof.side;
	t.motion_event = (W)s4_motion_event;
	t.watchdog_reset = (s5_watchdog_reset == TRUE) ? 1 : 0;
	t.wifi_up = 0;
	(void)tk_snd_mbf(mbf_telemetry, &t, (INT)sizeof(t), TMO_POL);
}

/* ------------------------------------------------------------------ */
EXPORT void ctrl_task(INT stacd, void *exinf)
{
	UINT			ptn = 0U;
	ER			er;
	NAV_MSG			nav;
	TERRAIN_MSG		ter;
	OBSTACLE_MSG		obs;
	MOTION_STATUS_MSG	mst;
	REMOTE_MSG		rem;
	UW			last_tel = 0U;

	(void)stacd;
	(void)exinf;

	(void)gpio_set_pin(S5_PIN_BTN_START, GPIO_MODE_IN);
	(void)gpio_set_pin(S5_PIN_BTN_STOP, GPIO_MODE_IN);
	set_w(GPIO(S5_PIN_BTN_START), GPIO_PUE);
	set_w(GPIO(S5_PIN_BTN_STOP), GPIO_PUE);
	status_init();
	if (s5_watchdog_reset == TRUE) {
		tm_printf((UB *)"[CTRL] last reset was a WATCHDOG reset\n");
	}
	ctrl_enter(VS_INITIALISING);

	while (1) {
		er = tk_wai_flg(flg_ctrl, FLG_EVENTS_ALL | FLG_FAULTS_ALL,
				TWF_ORW | TWF_BITCLR, &ptn, S5_CTRL_TICK_MS);
		watchdog_feed();

		/* Safety first: faults, remote STOP, motion, terrain, obstacle,
		 * then navigation. */
		if ((er == E_OK) && ((ptn & FLG_FAULTS_ALL) != 0U)) {
			ctrl_on_faults((ptn & FLG_FAULTS_ALL) >> FLG_FAULT_SHIFT);
		}
		while (tk_rcv_mbf(mbf_remote, &rem, TMO_POL) > 0) {
			ctrl_on_remote(&rem);
		}
		while (tk_rcv_mbf(mbf_motion_st, &mst, TMO_POL) > 0) {
			ctrl_on_motion(&mst);
		}
		while (tk_rcv_mbf(mbf_terrain, &ter, TMO_POL) > 0) {
			ctrl_on_terrain(&ter);
		}
		while (tk_rcv_mbf(mbf_obstacle, &obs, TMO_POL) > 0) {
			ctrl_on_obstacle(&obs);
		}
		while (tk_rcv_mbf(mbf_nav, &nav, TMO_POL) > 0) {
			ctrl_on_nav(&nav);
		}

		ctrl_buttons();
		ctrl_timers();
		status_tick();

		if ((s5_now_ms() - last_tel) >= (UW)S5_TELEMETRY_MS) {
			last_tel = s5_now_ms();
			ctrl_send_telemetry();
		}
	}
}
