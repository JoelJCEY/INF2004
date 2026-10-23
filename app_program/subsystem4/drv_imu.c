/*
 *	drv_imu.c
 *	Subsystem 4 -- LSM303DLHC accelerometer driver (bring-up).
 */

#include "drv_imu.h"
#include <tk/device.h>
#include "../../device/include/dev_i2c.h"

LOCAL ID	imu_dd = E_NOEXS;

EXPORT ER imu_open(void)
{
	imu_dd = tk_opn_dev((UB *)"iica", TD_UPDATE);
	return (imu_dd < E_OK) ? imu_dd : E_OK;
}

EXPORT ER imu_check_id(BOOL *p_ok)
{
	UB	id[3] = { 0U, 0U, 0U };
	INT	i;
	ER	er = E_OK;

	for (i = 0; i < 3; i++) {
		er = i2c_read_reg(imu_dd, IMU_ADDR_MAG, IMU_IRA_REG_M + (UW)i, &id[i]);
		if (er < E_OK) {
			break;
		}
	}
	tm_printf((UB *)"[S4] mag ID regs: 0x%02x 0x%02x 0x%02x (expect 0x48 0x34 0x33)\n",
		  (INT)id[0], (INT)id[1], (INT)id[2]);
	*p_ok = ((er >= E_OK) && (id[0] == 0x48U) && (id[1] == 0x34U) &&
		 (id[2] == 0x33U)) ? TRUE : FALSE;
	return (er < E_OK) ? er : E_OK;
}

EXPORT ER imu_config_accel(BOOL *p_ok)
{
	UB	r1 = 0U;
	UB	r4 = 0U;
	ER	er;

	er = i2c_write_reg(imu_dd, IMU_ADDR_ACCEL, IMU_CTRL_REG1_A, IMU_REG1_VALUE);
	if (er >= E_OK) {
		er = i2c_write_reg(imu_dd, IMU_ADDR_ACCEL, IMU_CTRL_REG4_A, IMU_REG4_VALUE);
	}
	if (er >= E_OK) {
		er = i2c_read_reg(imu_dd, IMU_ADDR_ACCEL, IMU_CTRL_REG1_A, &r1);
	}
	if (er >= E_OK) {
		er = i2c_read_reg(imu_dd, IMU_ADDR_ACCEL, IMU_CTRL_REG4_A, &r4);
	}
	tm_printf((UB *)"[S4] accel CTRL1=0x%02x CTRL4=0x%02x (expect 0x57 0x28)\n",
		  (INT)r1, (INT)r4);
	*p_ok = ((er >= E_OK) && (r1 == IMU_REG1_VALUE) && (r4 == IMU_REG4_VALUE))
		? TRUE : FALSE;
	return (er < E_OK) ? er : E_OK;
}

/* 16-bit little-endian pair -> signed 12-bit count -> mg.  The sensor
   left-justifies the 12-bit value, so divide by 16 (not >> on a signed). */
LOCAL W imu_to_mg(UB lo, UB hi)
{
	H	raw = (H)(UH)(((UH)hi << 8) | (UH)lo);

	return ((W)raw / 16) * IMU_MG_PER_LSB;
}

EXPORT ER imu_read_accel(IMU_ACCEL *p_out)
{
	T_I2C_EXEC	exec;
	UB		reg = IMU_OUT_X_L_A | IMU_AUTO_INC;
	UB		buf[6];
	SZ		asz = 0;
	ER		er;

	exec.sadr = IMU_ADDR_ACCEL;
	exec.snd_size = 1;
	exec.snd_data = &reg;
	exec.rcv_size = 6;
	exec.rcv_data = buf;

	er = tk_swri_dev(imu_dd, TDN_I2C_EXEC, &exec, sizeof(exec), &asz);
	if (er >= E_OK) {
		p_out->x_mg = imu_to_mg(buf[0], buf[1]);
		p_out->y_mg = imu_to_mg(buf[2], buf[3]);
		p_out->z_mg = imu_to_mg(buf[4], buf[5]);
	}
	return (er < E_OK) ? er : E_OK;
}
