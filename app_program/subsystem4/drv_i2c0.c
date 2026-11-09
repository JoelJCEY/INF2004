/*
 *	drv_i2c0.c
 *	Subsystem 4 -- polling I2C0 master for the RP2040 (DW_apb_i2c).
 */

#include "drv_i2c0.h"

#define I2C0_REG(off)		(0x40044000UL + (off))
#define IC_CON			I2C0_REG(0x00)
#define IC_TAR			I2C0_REG(0x04)
#define IC_DATA_CMD		I2C0_REG(0x10)
#define IC_FS_SCL_HCNT		I2C0_REG(0x1C)
#define IC_FS_SCL_LCNT		I2C0_REG(0x20)
#define IC_INTR_MASK		I2C0_REG(0x30)
#define IC_RAW_INTR_STAT	I2C0_REG(0x34)
#define IC_RX_TL		I2C0_REG(0x38)
#define IC_TX_TL		I2C0_REG(0x3C)
#define IC_CLR_TX_ABRT		I2C0_REG(0x54)
#define IC_CLR_STOP_DET		I2C0_REG(0x60)
#define IC_ENABLE		I2C0_REG(0x6C)
#define IC_TXFLR		I2C0_REG(0x74)
#define IC_RXFLR		I2C0_REG(0x78)
#define IC_SDA_HOLD		I2C0_REG(0x7C)
#define IC_ENABLE_STATUS	I2C0_REG(0x9C)
#define IC_FS_SPKLEN		I2C0_REG(0xA0)

#define CON_MASTER		(1UL << 0)
#define CON_SPEED_FAST		(2UL << 1)
#define CON_RESTART_EN		(1UL << 5)
#define CON_SLAVE_DISABLE	(1UL << 6)
#define CON_TX_EMPTY_CTRL	(1UL << 8)
#define RAW_TX_ABRT		(1UL << 6)
#define RAW_STOP_DET		(1UL << 9)
#define CMD_READ		(1UL << 8)
#define CMD_STOP		(1UL << 9)
#define CMD_RESTART		(1UL << 10)
#define I2C_FIFO_DEPTH		16U

#define CLK_PERI_HZ		125000000UL

LOCAL void i2c_enable(BOOL on)
{
	out_w(IC_ENABLE, on ? 1UL : 0UL);
	while ((in_w(IC_ENABLE_STATUS) & 1UL) != (on ? 1UL : 0UL)) {
		;
	}
}

EXPORT void drv_i2c0_init(UINT sda_pin, UINT scl_pin)
{
	UW	period = (CLK_PERI_HZ + (I2C0_BAUD_HZ / 2U)) / I2C0_BAUD_HZ;
	UW	lcnt = (period * 3U) / 5U;	/* same split as the Pico SDK */
	UW	hcnt = period - lcnt;
	UW	spklen = (lcnt < 16U) ? 1U : (lcnt / 16U);
	UW	hold = ((CLK_PERI_HZ * 3U) / 10000000U) + 1U;

	app_unreset(RESETS_RESET_I2C0);

	out_w(GPIO_CTRL(sda_pin), GPIO_CTRL_FUNCSEL_I2C);
	out_w(GPIO(sda_pin), GPIO_IE | GPIO_PUE | GPIO_SHEMITT | GPIO_DRIVE_4MA);
	out_w(GPIO_CTRL(scl_pin), GPIO_CTRL_FUNCSEL_I2C);
	out_w(GPIO(scl_pin), GPIO_IE | GPIO_PUE | GPIO_SHEMITT | GPIO_DRIVE_4MA);

	i2c_enable(FALSE);
	out_w(IC_CON, CON_MASTER | CON_SPEED_FAST | CON_RESTART_EN |
		      CON_SLAVE_DISABLE | CON_TX_EMPTY_CTRL);
	out_w(IC_TX_TL, 0);
	out_w(IC_RX_TL, 0);
	out_w(IC_INTR_MASK, 0);			/* polling only */
	out_w(IC_FS_SCL_HCNT, hcnt);
	out_w(IC_FS_SCL_LCNT, lcnt);
	out_w(IC_FS_SPKLEN, spklen);
	out_w(IC_SDA_HOLD, hold);
	i2c_enable(TRUE);
}

LOCAL BOOL timed_out(UW start)
{
	return ((app_time_us() - start) > I2C0_TIMEOUT_US) ? TRUE : FALSE;
}

EXPORT ER drv_i2c0_xfer(UB addr, const UB *wbuf, UINT wlen, UB *rbuf, UINT rlen)
{
	UW	start = app_time_us();
	UINT	i;
	UW	cmd;
	ER	er = E_OK;

	if ((wlen + rlen) == 0U) {
		return E_PAR;
	}
	i2c_enable(FALSE);
	out_w(IC_TAR, addr);
	i2c_enable(TRUE);
	(void)in_w(IC_CLR_TX_ABRT);
	(void)in_w(IC_CLR_STOP_DET);

	for (i = 0; (i < wlen) && (er == E_OK); i++) {
		while (in_w(IC_TXFLR) >= I2C_FIFO_DEPTH) {
			if (timed_out(start)) { er = E_TMOUT; break; }
		}
		cmd = wbuf[i];
		if ((i == (wlen - 1U)) && (rlen == 0U)) {
			cmd |= CMD_STOP;
		}
		out_w(IC_DATA_CMD, cmd);
	}
	for (i = 0; (i < rlen) && (er == E_OK); i++) {
		cmd = CMD_READ;
		if ((i == 0U) && (wlen > 0U)) {
			cmd |= CMD_RESTART;
		}
		if (i == (rlen - 1U)) {
			cmd |= CMD_STOP;
		}
		out_w(IC_DATA_CMD, cmd);
		while (in_w(IC_RXFLR) == 0U) {
			if ((in_w(IC_RAW_INTR_STAT) & RAW_TX_ABRT) != 0U) { er = E_IO; break; }
			if (timed_out(start)) { er = E_TMOUT; break; }
		}
		if (er == E_OK) {
			rbuf[i] = (UB)(in_w(IC_DATA_CMD) & 0xFFU);
		}
	}
	/* Wait for the STOP (also ends a write-only transfer) */
	while ((er == E_OK) && ((in_w(IC_RAW_INTR_STAT) & RAW_STOP_DET) == 0U)) {
		if ((in_w(IC_RAW_INTR_STAT) & RAW_TX_ABRT) != 0U) { er = E_IO; break; }
		if (timed_out(start)) { er = E_TMOUT; break; }
	}
	if ((in_w(IC_RAW_INTR_STAT) & RAW_TX_ABRT) != 0U) {
		er = E_IO;		/* NACK: device missing or wrong address */
	}
	(void)in_w(IC_CLR_TX_ABRT);
	(void)in_w(IC_CLR_STOP_DET);
	return er;
}

EXPORT ER drv_i2c0_write_reg(UB addr, UB reg, UB val)
{
	UB	b[2];

	b[0] = reg;
	b[1] = val;
	return drv_i2c0_xfer(addr, b, 2U, NULL, 0U);
}

EXPORT ER drv_i2c0_read_regs(UB addr, UB reg, UB *buf, UINT n)
{
	return drv_i2c0_xfer(addr, &reg, 1U, buf, n);
}
