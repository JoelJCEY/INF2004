/*
 *	drv_imu.h
 *	Subsystem 4 -- LSM303DLHC (GY-511) through the kernel I2C driver
 *	("iica" = I2C0 on GP16/GP17, 400 kHz).  Call only from imu_task.
 *
 *	Needs (shared configuration, agree with the team first):
 *	  config/config_device.h                      DEVCNF_USE_IIC = 1
 *	  device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h I2C0 pins 16 / 17
 */

#ifndef DRV_IMU_H
#define DRV_IMU_H

#include "subsystem4.h"

#define IMU_ADDR_ACCEL		0x19U	/* 7-bit address */
#define IMU_ADDR_MAG		0x1EU

/* Accelerometer registers */
#define IMU_CTRL_REG1_A		0x20U
#define IMU_CTRL_REG4_A		0x23U
#define IMU_OUT_X_L_A		0x28U
#define IMU_AUTO_INC		0x80U	/* sub-address MSB: auto-increment */

/* 0x57 = 100 Hz ODR, normal power, X/Y/Z on */
#define IMU_REG1_VALUE		0x57U
/* 0x28 = FS +-8 g (bits 5:4 = 10), high-resolution (bit 3) */
#define IMU_REG4_VALUE		0x28U
#define IMU_MG_PER_LSB		4	/* +-8 g, 12-bit HR: 4 mg per LSB */

/* Magnetometer identity registers: read back 'H', '4', '3' */
#define IMU_IRA_REG_M		0x0AU

typedef struct {
	W	x_mg;
	W	y_mg;
	W	z_mg;
} IMU_ACCEL;

IMPORT ER	imu_open(void);
IMPORT ER	imu_check_id(BOOL *p_ok);
IMPORT ER	imu_config_accel(BOOL *p_ok);
IMPORT ER	imu_read_accel(IMU_ACCEL *p_out);

#endif /* DRV_IMU_H */
