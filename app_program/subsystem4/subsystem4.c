/*
 *	subsystem4.c
 *	Subsystem 4 -- IMU: 10 ms cyclic handler and imu_task.
 */

#include "subsystem4.h"

EXPORT ID	tid_imu;
EXPORT ID	cyc_imu;

EXPORT ER subsystem4_init(void)
{
	T_CCYC	ccyc = {0};

	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)imu_cyclic_handler;
	ccyc.cyctim = S4_PERIOD_MS;
	ccyc.cycphs = 100;	/* give the sensor time to power up */
	cyc_imu = tk_cre_cyc(&ccyc);
	return (cyc_imu < E_OK) ? cyc_imu : E_OK;
}

EXPORT ER subsystem4_start(void)
{
	tid_imu = app_start_task(imu_task, S4_PRI_IMU, 0, "imu_task");
	if (tid_imu < E_OK) {
		return tid_imu;
	}
	return tk_sta_cyc(cyc_imu);
}

EXPORT void imu_cyclic_handler(void *exinf)
{
	(void)exinf;
	(void)tk_wup_tsk(tid_imu);
}
