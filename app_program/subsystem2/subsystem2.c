/*
 *	subsystem2.c
 *	Subsystem 2 -- Motion control: creates the Motion Command Queue and
 *	the 20 ms cyclic handler, starts motion_task.
 */

#include "subsystem2.h"

#define S2_CMD_QUEUE_DEPTH	8

EXPORT ID	tid_motion;
EXPORT ID	cyc_motion;
EXPORT ID	mbf_motion_cmd;

EXPORT ER subsystem2_init(void)
{
	T_CMBF	cmbf = {0};
	T_CCYC	ccyc = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.maxmsz = (INT)sizeof(MOTION_CMD);
	cmbf.bufsz = S2_CMD_QUEUE_DEPTH * ((INT)sizeof(MOTION_CMD) + 8);
	mbf_motion_cmd = tk_cre_mbf(&cmbf);
	if (mbf_motion_cmd < E_OK) {
		return mbf_motion_cmd;
	}

	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)motion_cyclic_handler;
	ccyc.cyctim = S2_PERIOD_MS;
	ccyc.cycphs = S2_PERIOD_MS;
	cyc_motion = tk_cre_cyc(&ccyc);
	return (cyc_motion < E_OK) ? cyc_motion : E_OK;
}

EXPORT ER subsystem2_start(void)
{
	tid_motion = app_start_task(motion_task, S2_PRI_MOTION, 0, "motion_task");
	if (tid_motion < E_OK) {
		return tid_motion;
	}
	return tk_sta_cyc(cyc_motion);
}

EXPORT void motion_cyclic_handler(void *exinf)
{
	(void)exinf;
	(void)tk_wup_tsk(tid_motion);
}
