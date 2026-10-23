/*
 *	subsystem4.h
 *	Subsystem 4 -- IMU terrain monitoring: humps and impacts (Buddy 4).
 *
 *	BASIC VERSION.
 *	  - LSM303DLHC accelerometer at 100 Hz, +-8 g, over I2C0 (kernel
 *	    driver "iica", GP16/GP17).
 *	  - Tilt from the accelerometer (no gyroscope), level baseline taken
 *	    in the first second, light low-pass filter.
 *	  - Hump: LEVEL -> CLIMBING -> DESCENDING -> LEVEL; peak height
 *	    estimate = wheelbase x sin(max tilt).  A climb longer than 500 mm,
 *	    or a climb that goes level again without descending, is discarded.
 *	  - Impact: |a| far from 1 g (higher threshold on a hump).
 *	The complete version rebuilds the hump profile from encoder distance
 *	and ignores tilt while the car accelerates or brakes.
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"

#define S4_PRI_IMU		14
#define S4_PERIOD_MS		10	/* 100 Hz = sensor ODR */
#define S4_PRINT_ENABLE		0	/* 1 = print tilt every 0.5 s */

/* ---- Mounting and geometry: CHECK ON THE CAR (assumption A4) ------- */
#define S4_FWD_AXIS		0	/* 0 = sensor X points forward, 1 = Y */
#define S4_FWD_SIGN		1	/* -1 if tilting nose-up reads negative */
#define S4_WHEELBASE_MM		150	/* front wheels to rear wheels/caster */

/* Thresholds (provisional; set from the flat-track noise log, A4).
 * Tilt is handled as sin(tilt) x 1000: 70 ~ 4 degrees. */
#define S4_SIN_ENTER		70
#define S4_SIN_LEVEL		25
#define S4_ENTER_N		3	/* samples above S4_SIN_ENTER */
#define S4_LEVEL_N		20	/* 200 ms level = hump over */
#define S4_CLIMB_MAX_MM		500	/* longer climb: ramp / bad reading */
#define S4_BASELINE_N		100	/* 1 s at rest for the level baseline */
#define S4_IMPACT_MG		2500	/* | |a| - 1 g | above this = impact */
#define S4_IMPACT_HUMP_MG	3500	/* higher on a hump (landing) */
#define S4_IMPACT_HOLDOFF_MS	1000
#define S4_FAULT_READS		10	/* consecutive I2C errors = IMU fault */

/* Read by tests and telemetry.  Written only by imu_task. */
IMPORT volatile BOOL	s4_ok;		/* sensor found and configured */
IMPORT volatile UW	s4_samples;
IMPORT volatile W	s4_sin_x1000;	/* filtered tilt, nose-up positive */
IMPORT volatile W	s4_peak_mm;	/* highest hump of the run */
IMPORT volatile W	s4_last_mag_mg;

IMPORT ID	tid_imu;
IMPORT ID	cyc_imu;
IMPORT void	imu_task(INT stacd, void *exinf);
IMPORT void	imu_cyclic_handler(void *exinf);

#endif /* SUBSYSTEM4_H */
