/*
 *	subsystem2.c
 *	Subsystem 2 -- Motion Control: creates its objects and starts its task.
 *
 *	subsystem2_init() runs FIRST in usermain(), before anything can
 *	block, so the PIO quadrature program gets offset 0 before the CYW43
 *	Wi-Fi driver loads its own PIO program (A2).
 */

#include "subsystem2.h"
#include "drv_motor.h"
#include "drv_encoder.h"

#define ENC_L_A_PIN	2	/* Grove 2: A GP2, B GP3 */
#define ENC_R_A_PIN	4	/* Grove 3: A GP4, B GP5 */

EXPORT ID	mbf_motion_cmd;		/* IF-05 */
EXPORT ID	tid_motion;
EXPORT ID	cyc_motion;
EXPORT BOOL	motion_encoders_ok;

EXPORT ER subsystem2_init(void)
{
	T_CMBF		cmbf = {0};
	T_CCYC		ccyc = {0};
	DRV_ENC_INFO	info;

	/* Motors first: braked (A = B = 0) from the first moment */
	drv_motor_init();

	/* Encoders: log PIO block and offset at boot (A2 validation) */
	motion_encoders_ok = (drv_encoder_init(ENC_L_A_PIN, ENC_R_A_PIN, &info) == 0)
			     ? TRUE : FALSE;
	if (motion_encoders_ok) {
		app_log((UB*)"motion: encoders on PIO%d offset %d, SM %d/%d\n",
			  info.pio_index, info.offset, info.sm[0], info.sm[1]);
	} else {
		app_log((UB*)"motion: ENCODER PIO LOAD FAILED (offset 0 taken)\n");
	}

	/* IF-05 Motion Command Queue: 8 fixed-size messages */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = (SZ)(S2_CMD_QUEUE_MSGS * (sizeof(MOTION_CMD) + sizeof(INT)));
	cmbf.maxmsz = (INT)sizeof(MOTION_CMD);
	mbf_motion_cmd = tk_cre_mbf(&cmbf);
	if (mbf_motion_cmd < E_OK) {
		return mbf_motion_cmd;
	}

	/* 20 ms release: started in subsystem2_start() */
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
