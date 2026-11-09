/*
 *	drv_lsm303.h
 *	Subsystem 4 -- LSM303DLHC accelerometer (0x19) + magnetometer (0x1E).
 *	No gyroscope: tilt comes from the accelerometer (design 6.4).
 */

#ifndef DRV_LSM303_H
#define DRV_LSM303_H

#include "../app.h"

#define LSM303_ACC_ADDR		0x19U
#define LSM303_MAG_ADDR		0x1EU
#define LSM303_MG_PER_LSB	4	/* ±8 g, high resolution, 12-bit */

typedef struct {
	W	x, y, z;		/* accelerometer, mg      */
} LSM303_ACC;

typedef struct {
	H	x, y, z;		/* magnetometer, raw LSB  */
} LSM303_MAG;

/* Configure ±8 g (so impacts are not clipped), 100 Hz, HR mode, and the
   magnetometer at 30 Hz.  Returns E_OK or E_IO when the chip does not
   answer (IMU fault, design 5.1). */
IMPORT ER drv_lsm303_init(void);
IMPORT ER drv_lsm303_read_acc(LSM303_ACC *a);
IMPORT ER drv_lsm303_read_mag(LSM303_MAG *m);

#endif /* DRV_LSM303_H */
