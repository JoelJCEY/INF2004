/*
 *	subsystem5.c
 *	Subsystem 5 -- creates every object owned by ctrl_task (event flag,
 *	mission-state mutex, the five queues into ctrl_task) and starts
 *	ctrl_task and scan_task.  Also reads the watchdog reset reason once.
 */

#include "subsystem5.h"

#define S5_QUEUE_DEPTH		8
#define WATCHDOG_REASON_R	(0x40058000U + 0x08U)

EXPORT ID	flg_ctrl;
EXPORT ID	mtx_mission;
EXPORT ID	mbf_nav;
EXPORT ID	mbf_terrain;
EXPORT ID	mbf_obstacle;
EXPORT ID	mbf_motion_st;
EXPORT ID	mbf_remote;
EXPORT ID	tid_ctrl;
EXPORT ID	tid_scan;

EXPORT volatile VEHICLE_STATE	s5_vehicle_state = VS_INITIALISING;
EXPORT volatile UW		s5_fault_mask = 0U;
EXPORT volatile BOOL		s5_watchdog_reset = FALSE;
EXPORT volatile SCAN_REQUEST	s5_scan_request = SCAN_REQ_WATCH;
EXPORT volatile UW		s5_scan_request_seq = 0U;

/* Protected by mtx_mission; written by ctrl_task only. */
EXPORT MISSION_STATE		s5_mission = { VS_INITIALISING, 0U, 0, 0, FALSE };

EXPORT const char * const s5_state_name[VS_NUM_STATES] = {
	"INITIALISING", "READY", "LINE FOLLOWING", "BARCODE MANOEUVRE",
	"HUMP CROSSING", "OBSTACLE BYPASS", "MISSION COMPLETE", "STOPPED",
	"FAULT STOP", "LINE RECOVERY"
};

LOCAL ID s5_make_mbf(INT msg_size)
{
	T_CMBF	cmbf = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.maxmsz = msg_size;
	cmbf.bufsz = S5_QUEUE_DEPTH * (msg_size + 8);
	return tk_cre_mbf(&cmbf);
}

EXPORT ER s5_post(ID mbf, const void *msg, INT size, UINT flag_bit)
{
	ER	er = tk_snd_mbf(mbf, (void *)msg, size, TMO_POL);

	(void)tk_set_flg(flg_ctrl, flag_bit);
	return er;
}

/* tk_set_flg is atomic, so any task can raise a fault bit safely. */
EXPORT void s5_raise_fault(UW fault_bit)
{
	(void)tk_set_flg(flg_ctrl, (fault_bit & 0xFFU) << FLG_FAULT_SHIFT);
}

EXPORT void s5_mission_get(MISSION_STATE *p_out)
{
	if (tk_loc_mtx(mtx_mission, 20) == E_OK) {
		*p_out = s5_mission;
		(void)tk_unl_mtx(mtx_mission);
	}
}

EXPORT UW s5_now_ms(void)
{
	SYSTIM	t;

	(void)tk_get_otm(&t);
	return (UW)t.lo;
}

EXPORT ER subsystem5_init(void)
{
	T_CFLG	cflg = {0};
	T_CMTX	cmtx = {0};

	/* bit 0 = timer timeout, bit 1 = forced reset (RP2040 datasheet) */
	s5_watchdog_reset = ((in_w(WATCHDOG_REASON_R) & 0x1U) != 0U) ? TRUE : FALSE;

	cflg.flgatr = TA_TFIFO | TA_WMUL;
	flg_ctrl = tk_cre_flg(&cflg);
	if (flg_ctrl < E_OK) {
		return flg_ctrl;
	}
	cmtx.mtxatr = TA_INHERIT;
	mtx_mission = tk_cre_mtx(&cmtx);
	if (mtx_mission < E_OK) {
		return mtx_mission;
	}
	mbf_nav = s5_make_mbf((INT)sizeof(NAV_MSG));
	mbf_terrain = s5_make_mbf((INT)sizeof(TERRAIN_MSG));
	mbf_obstacle = s5_make_mbf((INT)sizeof(OBSTACLE_MSG));
	mbf_motion_st = s5_make_mbf((INT)sizeof(MOTION_STATUS_MSG));
	mbf_remote = s5_make_mbf((INT)sizeof(REMOTE_MSG));
	if ((mbf_nav < E_OK) || (mbf_terrain < E_OK) || (mbf_obstacle < E_OK) ||
	    (mbf_motion_st < E_OK) || (mbf_remote < E_OK)) {
		return E_NOMEM;
	}
	return E_OK;
}

EXPORT ER subsystem5_start(void)
{
	tid_ctrl = app_start_task(ctrl_task, S5_PRI_CTRL, 0, "ctrl_task");
	if (tid_ctrl < E_OK) {
		return tid_ctrl;
	}
	tid_scan = app_start_task(scan_task, S5_PRI_SCAN, 0, "scan_task");
	return (tid_scan < E_OK) ? tid_scan : E_OK;
}
