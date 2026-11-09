/*
 *	drv_lsm303.c
 *	Subsystem 4 -- LSM303DLHC driver on I2C0.
 */

#include "drv_lsm303.h"
#include "drv_i2c0.h"
#include "subsystem4.h"

#define CTRL_REG1_A	0x20U
#define CTRL_REG4_A	0x23U
#define OUT_X_L_A	0x28U
#define AUTO_INC	0x80U
#define CRA_REG_M	0x00U
#define CRB_REG_M	0x01U
#define MR_REG_M	0x02U
#define OUT_X_H_M	0x03U
#define IRA_REG_M	0x0AU

#define REG1_100HZ_XYZ	0x57U	/* ODR 100 Hz, normal, X/Y/Z on    */
#define REG4_8G_HR	0xA8U	/* BDU on, FS = ±8 g, high-resolution mode */
#define CRA_30HZ	0x14U
#define CRB_1G3		0x20U
#define MR_CONTINUOUS	0x00U

EXPORT ER drv_lsm303_init(void)
{
	UB	id = 0;
	UB	check = 0;
	ER	er;

	drv_i2c0_init(IMU_SDA_PIN, IMU_SCL_PIN);

	er = drv_i2c0_write_reg(LSM303_ACC_ADDR, CTRL_REG1_A, REG1_100HZ_XYZ);
	if (er == E_OK) {
		er = drv_i2c0_write_reg(LSM303_ACC_ADDR, CTRL_REG4_A, REG4_8G_HR);
	}
	if (er == E_OK) {
		/* The DLHC accelerometer has no WHO_AM_I: read back instead */
		er = drv_i2c0_read_regs(LSM303_ACC_ADDR, CTRL_REG4_A, &check, 1U);
		if ((er == E_OK) && (check != REG4_8G_HR)) {
			er = E_IO;
		}
	}
	if (er == E_OK) {
		er = drv_i2c0_read_regs(LSM303_MAG_ADDR, IRA_REG_M, &id, 1U);
		if ((er == E_OK) && (id != 0x48U)) {	/* 'H' */
			er = E_IO;
		}
	}
	if (er == E_OK) {
		(void)drv_i2c0_write_reg(LSM303_MAG_ADDR, CRA_REG_M, CRA_30HZ);
		(void)drv_i2c0_write_reg(LSM303_MAG_ADDR, CRB_REG_M, CRB_1G3);
		er = drv_i2c0_write_reg(LSM303_MAG_ADDR, MR_REG_M, MR_CONTINUOUS);
	}
	return er;
}

EXPORT ER drv_lsm303_read_acc(LSM303_ACC *a)
{
	UB	b[6];
	ER	er;

	er = drv_i2c0_read_regs(LSM303_ACC_ADDR, OUT_X_L_A | AUTO_INC, b, 6U);
	if (er != E_OK) {
		return er;
	}
	/* 12-bit, left-justified, little-endian */
	a->x = ((W)(H)((UH)b[0] | ((UH)b[1] << 8)) >> 4) * LSM303_MG_PER_LSB;
	a->y = ((W)(H)((UH)b[2] | ((UH)b[3] << 8)) >> 4) * LSM303_MG_PER_LSB;
	a->z = ((W)(H)((UH)b[4] | ((UH)b[5] << 8)) >> 4) * LSM303_MG_PER_LSB;
	return E_OK;
}

EXPORT ER drv_lsm303_read_mag(LSM303_MAG *m)
{
	UB	b[6];
	ER	er;

	er = drv_i2c0_read_regs(LSM303_MAG_ADDR, OUT_X_H_M, b, 6U);
	if (er != E_OK) {
		return er;
	}
	/* Big-endian, order X, Z, Y */
	m->x = (H)(((UH)b[0] << 8) | b[1]);
	m->z = (H)(((UH)b[2] << 8) | b[3]);
	m->y = (H)(((UH)b[4] << 8) | b[5]);
	return E_OK;
}
