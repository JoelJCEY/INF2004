/*
 *	task_ctrl.c
 *	Subsystem 5 -- ctrl_task, the Vehicle Controller (BASIC VERSION).
 *
 *	Highest application priority.  It sleeps on its event flag and wakes
 *	when another task posts a message, or every 50 ms to poll the buttons
 *	and run timers.  It is the only task that:
 *	  - writes the mission state (s5_vehicle_state),
 *	  - sends motion commands (through the motion API),
 *	  - sends telemetry snapshots (Telemetry Queue).
 *
 *	System statechart (Section 5), basic behaviour:
 *	  INITIALISING -> READY when every subsystem reports ready.
 *	  READY / STOPPED --START (GP20)--> LINE FOLLOWING.
 *	  LINE FOLLOWING --barcode--> BARCODE MANOEUVRE --done--> LINE FOLLOWING
 *	  LINE FOLLOWING --hump start--> HUMP CROSSING (speed limit) --end-->
 *	  any active --obstacle--> OBSTACLE BYPASS: basic version STOPS and
 *	      waits for the path to clear, then follows again (no bypass yet).
 *	  any active --STOP (GP21), impact, LINE LOST--> STOPPED.
 *	  any state --stall / IMU fault--> FAULT STOP.  Hold both buttons for
 *	      1 s = RESET -> INITIALISING.
 */

#include "subsystem5.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem1/command_codec.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"

typedef enum {
	BX_NONE = 0,
	BX_WAIT_DELAY,		/* keep following until the junction */
	BX_FORWARD,		/* wheels over the junction */
	BX_TURN			/* turning; follow again when done */
} BX_STAGE;

LOCAL VEHICLE_STATE	st = VS_INITIALISING;
LOCAL UW		st_since_ms = 0U;
LOCAL BX_STAGE		bx_stage = BX_NONE;
LOCAL BARCODE_CMD	bx_cmd = BC_NONE;
LOCAL W			bx_start_mm = 0;
LOCAL W			last_barcode = BC_NONE;
LOCAL W			last_letter = 0;
LOCAL BOOL		hump_active = FALSE;
LOCAL UW		tel_seq = 0U;

/* ------------------------------------------------------------------ */
LOCAL void ctrl_enter(VEHICLE_STATE s)
{
	if (s != st) {
		tm_printf((UB *)"[CTRL] %s -> %s\n", s5_state_name[st], s5_state_name[s]);
	}
	st = s;
	s5_vehicle_state = s;
	st_since_ms = s5_now_ms();
}

LOCAL BOOL ctrl_active(void)
{
	return ((st == VS_LINE_FOLLOWING) || (st == VS_BARCODE_MANOEUVRE) ||
		(st == VS_HUMP_CROSSING) || (st == VS_OBSTACLE_BYPASS) ||
		(st == VS_LINE_RECOVERY)) ? TRUE : FALSE;
}

LOCAL void ctrl_follow(void)
{
	(void)motion_set_speed_limit((hump_active == TRUE) ? S5_HUMP_MM_S : 0);
	(void)motion_follow_line(S5_CRUISE_MM_S);
	bx_stage = BX_NONE;
	ctrl_enter((hump_active == TRUE) ? VS_HUMP_CROSSING : VS_LINE_FOLLOWING);
}

LOCAL void ctrl_stop(VEHICLE_STATE s)
{
	(void)motion_stop();
	bx_stage = BX_NONE;
	ctrl_enter(s);
}

LOCAL void ctrl_fault(UW bit)
{
	s5_fault_mask |= bit;
	ctrl_stop(VS_FAULT_STOP);
}

LOCAL void ctrl_reset(void)
{
	(void)motion_stop();
	(void)motion_reset_fault();
	(void)motion_set_speed_limit(0);
	s5_fault_mask = 0U;
	ctrl_enter(VS_INITIALISING);
}

LOCAL void ctrl_remote_commands(void)
{
	UB command;
	BOOL stop_pending = FALSE;

	while (tk_rcv_mbf(mbf_remote_stop, &command, TMO_POL) > 0) {
		stop_pending = TRUE;
	}
	if (stop_pending == TRUE) {
		/* STOP wins over START/RESET received in the same controller cycle. */
		if (st == VS_FAULT_STOP) {
			(void)motion_stop();
		} else {
			ctrl_stop(VS_STOPPED);
		}
		while (tk_rcv_mbf(mbf_remote_other, &command, TMO_POL) > 0) {
			;
		}
		return;
	}
	while (tk_rcv_mbf(mbf_remote_other, &command, TMO_POL) > 0) {
		if (command == (UB)S1_CMD_RESET) {
			ctrl_reset();
		} else if ((command == (UB)S1_CMD_START) &&
			   ((st == VS_READY) || (st == VS_STOPPED))) {
			ctrl_follow();
		}
	}
}

/* ------------------------------------------------------------------ */
LOCAL void ctrl_on_motion(const MOTION_STATUS_MSG *p_m)
{
	if (p_m->kind == MST_STALL) {
		ctrl_fault(FAULT_STALL);
		return;
	}
	/* MST_DONE: next stage of a barcode manoeuvre */
	if ((st == VS_BARCODE_MANOEUVRE) && (bx_stage == BX_FORWARD)) {
		bx_stage = BX_TURN;
		if (bx_cmd == BC_TURN_LEFT) {
			(void)motion_turn_left(90);
		} else if (bx_cmd == BC_TURN_RIGHT) {
			(void)motion_turn_right(90);
		} else {
			ctrl_follow();		/* straight: just carry on */
		}
	} else if ((st == VS_BARCODE_MANOEUVRE) && (bx_stage == BX_TURN)) {
		ctrl_follow();
	} else {
		;
	}
}

LOCAL void ctrl_on_nav(const NAV_MSG *p_n)
{
	if (p_n->kind == NAV_BARCODE) {
		last_barcode = p_n->cmd;
		last_letter = p_n->letter;
		if ((st != VS_LINE_FOLLOWING) && (st != VS_HUMP_CROSSING)) {
			return;		/* only act on barcodes while following */
		}
		if (p_n->cmd == BC_END_OF_COURSE) {
			ctrl_stop(VS_MISSION_COMPLETE);
		} else if ((p_n->cmd == BC_UNRECOGNISED) || (p_n->cmd == BC_NONE)) {
			tm_printf((UB *)"[CTRL] barcode '%c' not a command: carry on\n",
				  (INT)p_n->letter);
		} else {
			bx_cmd = p_n->cmd;
			bx_start_mm = p_n->distance_mm;
			bx_stage = BX_WAIT_DELAY;
			ctrl_enter(VS_BARCODE_MANOEUVRE);	/* still following */
		}
	} else if (p_n->kind == NAV_LINE_LOST) {
		if (ctrl_active() == TRUE) {
			s5_fault_mask |= FAULT_LINE_LOST;
			ctrl_stop(VS_STOPPED);
		}
	} else {
		;	/* NAV_LINE_FOUND: nothing to do in the basic version */
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
			ctrl_stop(VS_STOPPED);
		}
		break;
	case TER_IMU_FAULT:
	default:
		s5_fault_mask |= FAULT_IMU;
		if (S5_REQUIRE_IMU != 0) {
			ctrl_fault(FAULT_IMU);
		}
		break;
	}
}

LOCAL void ctrl_on_obstacle(const OBSTACLE_MSG *p_o)
{
	if (p_o->kind == OBS_ALERT) {
		if (ctrl_active() == TRUE) {
			tm_printf((UB *)"[CTRL] obstacle at %d mm: stop and wait\n",
				  (INT)p_o->distance_mm);
			ctrl_stop(VS_OBSTACLE_BYPASS);
		}
	} else if (p_o->kind == OBS_CLEAR) {
		if (st == VS_OBSTACLE_BYPASS) {
			ctrl_follow();
		}
	} else {
		s5_fault_mask |= FAULT_NO_ECHO;	/* reported; basic does not bypass */
	}
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
			tm_printf((UB *)"[CTRL] RESET\n");
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

	if ((stop == TRUE) && (prev_stop == FALSE) && (ctrl_active() == TRUE)) {
		ctrl_stop(VS_STOPPED);
	}
	if ((start == FALSE) && (prev_start == TRUE)) {
		/* act on release, so a START+STOP press is not taken as START */
		if ((st == VS_READY) || (st == VS_STOPPED)) {
			ctrl_follow();
		}
	}
	prev_start = start;
	prev_stop = stop;
}

LOCAL void ctrl_timers(void)
{
	UW	now = s5_now_ms();

	if (st == VS_INITIALISING) {
		BOOL	imu_ok = ((s4_ok == TRUE) || (S5_REQUIRE_IMU == 0)) ? TRUE : FALSE;

		if ((s2_hw_ready == TRUE) && (s3_ready == TRUE) && (s5_scan_ready == TRUE) &&
		    (imu_ok == TRUE) && ((now - st_since_ms) > 1500U)) {
			ctrl_enter(VS_READY);
			tm_printf((UB *)"[CTRL] READY: press START (GP20)\n");
		} else if ((now - st_since_ms) > (UW)S5_INIT_WAIT_MS) {
			tm_printf((UB *)"[CTRL] init timeout: motion=%d line=%d scan=%d imu=%d\n",
				  (INT)s2_hw_ready, (INT)s3_ready, (INT)s5_scan_ready, (INT)s4_ok);
			if (imu_ok == FALSE) {
				s5_fault_mask |= FAULT_IMU;
			} else {
				s5_fault_mask |= FAULT_ENCODER;	/* a subsystem did not start */
			}
			ctrl_enter(VS_FAULT_STOP);
		} else {
			;
		}
	}

	if ((st == VS_BARCODE_MANOEUVRE) && (bx_stage == BX_WAIT_DELAY) &&
	    ((s2_distance_mm - bx_start_mm) >= S5_EXEC_DELAY_MM)) {
		if (bx_cmd == BC_U_TURN) {
			bx_stage = BX_TURN;
			(void)motion_turn_left(180);
		} else {
			bx_stage = BX_FORWARD;
			(void)motion_move_forward(S5_EXEC_FWD_MM);
		}
	}
}

LOCAL void ctrl_send_telemetry(void)
{
	S1_TELEMETRY	t;
	W		err;
	UW		age;
	UW		flags;

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
	t.wifi_up = (s1_wifi_up == TRUE) ? 1 : 0;
	(void)tk_snd_mbf(mbf_telemetry, &t, (INT)sizeof(t), TMO_POL);	/* drop if full */
}

/* ------------------------------------------------------------------ */
EXPORT void ctrl_task(INT stacd, void *exinf)
{
	UINT			ptn;
	NAV_MSG			nav;
	TERRAIN_MSG		ter;
	OBSTACLE_MSG		obs;
	MOTION_STATUS_MSG	mst;
	UW			last_tel = 0U;

	(void)stacd;
	(void)exinf;

	(void)gpio_set_pin(S5_PIN_BTN_START, GPIO_MODE_IN);
	(void)gpio_set_pin(S5_PIN_BTN_STOP, GPIO_MODE_IN);
	set_w(GPIO(S5_PIN_BTN_START), GPIO_PUE);
	set_w(GPIO(S5_PIN_BTN_STOP), GPIO_PUE);
	ctrl_enter(VS_INITIALISING);

	while (1) {
		(void)tk_wai_flg(flg_ctrl, FLG_EVENTS_ALL, TWF_ORW | TWF_BITCLR,
				 &ptn, S5_CTRL_TICK_MS);

		/* Safety first: motion faults, then terrain/obstacle, then nav. */
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
		ctrl_remote_commands(); /* STOP overrides a simultaneous button START. */
		ctrl_timers();

		if ((s5_now_ms() - last_tel) >= (UW)(S5_CTRL_TICK_MS * S5_TELEMETRY_EVERY)) {
			last_tel = s5_now_ms();
			ctrl_send_telemetry();
		}
	}
}
