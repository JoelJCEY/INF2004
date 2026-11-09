/*
 *	subsystem4.h
 *	Subsystem 4 -- Terrain Monitoring (imu_task).
 *	Buddy 4: 2501315 (Xavier).  Statechart 6.4, assumption A4.
 *
 *	Owns: LSM303DLHC on I2C0 (GP16 SDA, GP17 SCL, 400 kHz), the only
 *	device on that bus, so no bus mutex is needed.
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"

#define S4_PRI_IMU		APP_PRI_IMU
#define S4_PERIOD_MS		20	/* 50 Hz (design 10-20 ms) */

#define IMU_SDA_PIN		16
#define IMU_SCL_PIN		17

/* ---------------------------------------------------------------------
 *  Motion-event enum (3 bits, agreed to pack, design 8.5 Version B)
 * --------------------------------------------------------------------- */
typedef enum {
	ME_STATIONARY	= 0,
	ME_CRUISING	= 1,
	ME_ACCELERATING	= 2,	/* or braking */
	ME_TURNING	= 3,
	ME_CLIMBING	= 4,
	ME_DESCENDING	= 5,
	ME_IMPACT	= 6
} MOTION_EVENT_CLASS;

/* ---------------------------------------------------------------------
 *  IF-11  Terrain Status Queue (imu_task -> ctrl_task)
 *  Message buffer mbf_terrain, owned by ctrl_task (subsystem5.h),
 *  followed by tk_set_flg(flg_sys, EVF_TERRAIN).
 * --------------------------------------------------------------------- */
typedef enum {
	TR_HUMP_START	= 1,	/* front wheels started climbing         */
	TR_HUMP_END	= 2,	/* rear wheels crossed: peak_01mm valid  */
	TR_IMPACT	= 3,	/* also sets FLT_IMPACT                  */
	TR_HUMP_DISCARD	= 4	/* > 500 mm climb or false trigger       */
} TERRAIN_KIND;

typedef struct {
	UB	kind;		/* TERRAIN_KIND            */
	UB	event;		/* MOTION_EVENT_CLASS      */
	UH	peak_01mm;	/* hump peak, 0.1 mm       */
	UW	path_mm;	/* odometry when it happened */
} TERRAIN_MSG;
APP_STATIC_ASSERT(sizeof(TERRAIN_MSG) == 8, terrain_msg_size);

/* ---------------------------------------------------------------------
 *  IF-12  Hump bit: EVF_HUMP_ACTIVE in flg_sys (app.h).  imu_task is its
 *  only writer (set on CLIMBING, cleared on report or discard);
 *  scan_task reads it with tk_ref_flg() and ignores echoes meanwhile.
 *
 *  Status word for telemetry (writer imu_task only):
 *    bits 0-2  MOTION_EVENT_CLASS     bits 3-6  humps reported (sat. 15)
 *    bits 8-23 highest peak this run, 0.1 mm
 * --------------------------------------------------------------------- */
IMPORT volatile UW imu_status_word;

/* Axis mapping: the X axis points forward, nose-up gives +X (validate
   with sensor bench S4-B02; change the sign if the board is mounted the
   other way round) */
#define IMU_FWD_SIGN		1

/* Latest raw readings (writer imu_task; sensor bench, diagnostics) */
IMPORT volatile W  imu_acc_raw[3];	/* mg                     */
IMPORT volatile W  imu_mag_raw[3];	/* raw LSB                */
IMPORT volatile W  imu_base_mg[2];	/* level baseline x, y    */
IMPORT volatile UW imu_reads_ok;
IMPORT volatile UW imu_reads_fail;

/* ---------------------------------------------------------------------
 *  Private to subsystem 4
 * --------------------------------------------------------------------- */
IMPORT ID	tid_imu;
IMPORT ID	cyc_imu;
IMPORT void	imu_task(INT stacd, void *exinf);
IMPORT void	imu_cyclic_handler(void *exinf);

#endif /* SUBSYSTEM4_H */
