/*
 *----------------------------------------------------------------------
 *    Device Driver for μT-Kernel 3.00.05
 *
 *    Copyright (C) 2020-2021 by Ken Sakamura.
 *    This software is distributed under the T-License 2.2.
 *----------------------------------------------------------------------
 *
 *    Released by TRON Forum(http://www.tron.org) at 2021/11.
 *
 *----------------------------------------------------------------------
 */


/*
 *	config_device.h
 *	Device configuration definition
 */

#ifndef	__DEV_CONFIG_H__
#define	__DEV_CONFIG_H__

/* ------------------------------------------------------------------------ */
/* Device usage settings
 *	1: Use   0: Do not use
 */

#define DEVCNF_USE_SER		1		// Serial communication device 
#define DEVCNF_USE_ADC		0		// A/D conversion device
		/* IS11: off.  line_task owns the ADC (round-robin + DMA,
		   TEAM.md section 3), so the kernel driver must not touch it. */
#define DEVCNF_USE_IIC		0		// I2C communication device
		/* Off by default so no pins are claimed at boot.  To use I2C,
		   set this to 1 and choose the pins in
		   device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h. */

#endif	/* __DEV_CONFIG_H__ */