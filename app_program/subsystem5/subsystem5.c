/*
 *	subsystem5.c
 *	Subsystem 5 -- Obstacle Scanning + Vehicle Controller: creates the
 *	System Event Group, the mission-state mutex and every queue that
 *	ctrl_task receives on, then starts ctrl_task and scan_task.
 */

#include "subsystem5.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"

#define CTRL_STACK_SIZE		3072

EXPORT ID	flg_sys;
EXPORT ID	mtx_mission;
EXPORT ID	mbf_nav;
EXPORT ID	mbf_terrain;
EXPORT ID	mbf_obstacle;
EXPORT ID	mbf_motion_status;
EXPORT ID	mbf_remote;
EXPORT ID	tid_ctrl;
EXPORT ID	tid_scan;

/* One message buffer holding S5_QUEUE_MSGS messages of msgsz bytes
   (µT-Kernel stores a small header with every message) */
LOCAL ID make_queue(INT msgsz)
{
	T_CMBF	cmbf = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = (SZ)(S5_QUEUE_MSGS * (msgsz + (INT)sizeof(INT)));
	cmbf.maxmsz = msgsz;
	return tk_cre_mbf(&cmbf);
}

EXPORT ER subsystem5_init(void)
{
	T_CFLG	cflg = {0};
	T_CMTX	cmtx = {0};

	/* System Event Group: ctrl_task is the only waiter */
	cflg.flgatr  = TA_TFIFO | TA_WSGL;
	cflg.iflgptn = 0;
	flg_sys = tk_cre_flg(&cflg);
	if (flg_sys < E_OK) {
		return flg_sys;
	}

	/* Mission state lock with priority inheritance (F5) */
	cmtx.mtxatr = TA_INHERIT;
	mtx_mission = tk_cre_mtx(&cmtx);
	if (mtx_mission < E_OK) {
		return mtx_mission;
	}

	mbf_nav = make_queue((INT)sizeof(NAV_MSG));
	mbf_terrain = make_queue((INT)sizeof(TERRAIN_MSG));
	mbf_obstacle = make_queue((INT)sizeof(OBSTACLE_MSG));
	mbf_motion_status = make_queue((INT)sizeof(MOTION_STATUS));
	mbf_remote = make_queue((INT)sizeof(REMOTE_MSG));
	if ((mbf_nav < E_OK) || (mbf_terrain < E_OK) || (mbf_obstacle < E_OK) ||
	    (mbf_motion_status < E_OK) || (mbf_remote < E_OK)) {
		return E_LIMIT;
	}
	return E_OK;
}

EXPORT ER subsystem5_start(void)
{
	tid_ctrl = app_start_task_stk(ctrl_task, S5_PRI_CTRL, 0, CTRL_STACK_SIZE,
				      "ctrl_task");
	if (tid_ctrl < E_OK) {
		return tid_ctrl;
	}
	tid_scan = app_start_task(scan_task, S5_PRI_SCAN, 0, "scan_task");
	return (tid_scan < E_OK) ? tid_scan : E_OK;
}
