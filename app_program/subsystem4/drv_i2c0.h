/*
 *	drv_i2c0.h
 *	Subsystem 4 -- minimal polling I2C0 master (GP16 SDA, GP17 SCL).
 *
 *	Register-level driver kept in the owning subsystem (TEAM.md section 3)
 *	instead of switching on the kernel's I2C device driver, which would
 *	also claim I2C1 pins.  imu_task is the only caller; LSM303DLHC is the
 *	only device on the bus, so there is no bus mutex.
 */

#ifndef DRV_I2C0_H
#define DRV_I2C0_H

#include "../app.h"

#define I2C0_BAUD_HZ		400000U
#define I2C0_TIMEOUT_US		2000U	/* per transfer */

IMPORT void drv_i2c0_init(UINT sda_pin, UINT scl_pin);
/* Write wlen bytes, then (if rlen > 0) repeated-start and read rlen.
   Returns E_OK, E_IO (NACK / abort) or E_TMOUT. */
IMPORT ER   drv_i2c0_xfer(UB addr, const UB *wbuf, UINT wlen, UB *rbuf, UINT rlen);
IMPORT ER   drv_i2c0_write_reg(UB addr, UB reg, UB val);
IMPORT ER   drv_i2c0_read_regs(UB addr, UB reg, UB *buf, UINT n);

#endif /* DRV_I2C0_H */
