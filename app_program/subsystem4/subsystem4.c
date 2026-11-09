/*
 *	subsystem4.c
 *	Subsystem 4 -- Terrain Monitoring: creates its cyclic handler and
 *	starts imu_task.  The I2C bus is brought up by imu_task itself.
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
	ccyc.cycphs = S4_PERIOD_MS;
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
