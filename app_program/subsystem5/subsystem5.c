/*
 *	subsystem5.c
 *	Subsystem 5 -- creates every object owned by ctrl_task (event flag and
 *	the four queues into ctrl_task), the scan cyclic handler, and starts
 *	ctrl_task and scan_task.
 */

#include "subsystem5.h"

#define S5_QUEUE_DEPTH		8

EXPORT ID	flg_ctrl;
EXPORT ID	mbf_nav;
EXPORT ID	mbf_terrain;
EXPORT ID	mbf_obstacle;
EXPORT ID	mbf_motion_st;
EXPORT ID	tid_ctrl;
EXPORT ID	tid_scan;
EXPORT ID	cyc_scan;

EXPORT volatile VEHICLE_STATE	s5_vehicle_state = VS_INITIALISING;
EXPORT volatile UW		s5_fault_mask = 0U;

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

	(void)tk_set_flg(flg_ctrl, flag_bit);	/* wake ctrl_task even if full */
	return er;
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
	T_CCYC	ccyc = {0};

	cflg.flgatr = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0U;
	flg_ctrl = tk_cre_flg(&cflg);
	if (flg_ctrl < E_OK) {
		return flg_ctrl;
	}
	mbf_nav = s5_make_mbf((INT)sizeof(NAV_MSG));
	mbf_terrain = s5_make_mbf((INT)sizeof(TERRAIN_MSG));
	mbf_obstacle = s5_make_mbf((INT)sizeof(OBSTACLE_MSG));
	mbf_motion_st = s5_make_mbf((INT)sizeof(MOTION_STATUS_MSG));
	if ((mbf_nav < E_OK) || (mbf_terrain < E_OK) ||
	    (mbf_obstacle < E_OK) || (mbf_motion_st < E_OK)) {
		return E_NOMEM;
	}

	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)scan_cyclic_handler;
	ccyc.cyctim = S5_SCAN_PERIOD_MS;
	ccyc.cycphs = S5_SCAN_PERIOD_MS;
	cyc_scan = tk_cre_cyc(&ccyc);
	return (cyc_scan < E_OK) ? cyc_scan : E_OK;
}

EXPORT ER subsystem5_start(void)
{
	tid_ctrl = app_start_task(ctrl_task, S5_PRI_CTRL, 0, "ctrl_task");
	if (tid_ctrl < E_OK) {
		return tid_ctrl;
	}
	tid_scan = app_start_task(scan_task, S5_PRI_SCAN, 0, "scan_task");
	if (tid_scan < E_OK) {
		return tid_scan;
	}
	return tk_sta_cyc(cyc_scan);
}

EXPORT void scan_cyclic_handler(void *exinf)
{
	(void)exinf;
	(void)tk_wup_tsk(tid_scan);
}
