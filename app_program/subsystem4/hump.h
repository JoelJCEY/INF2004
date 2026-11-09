/*
 *	hump.h
 *	Subsystem 4 -- hump detector and height reconstruction (pure logic).
 *
 *	The accelerometer gives the car's tilt, i.e. the height difference
 *	dh between the front (drive) wheels and the rear (caster):
 *	    dh = L * sin(pitch) = L * a_fwd / |a|
 *	Because the rear follows the same path, the height under the front
 *	wheels is rebuilt step by step over encoder distance s:
 *	    h(s) = dh(s) + h(s - L),   h = 0 before the hump
 *	The highest h is the hump peak (FR3, NFR5).  All in 32-bit µm.
 */

#ifndef HUMP_H
#define HUMP_H

#include "subsystem4.h"

#define HUMP_WHEELBASE_MM	110	/* drive axle to caster: measure (A4) */
#define HUMP_STEP_MM		5	/* profile grid                        */
#define HUMP_ENTRY_UM		6000	/* front 6 mm above rear (~3 deg)      */
#define HUMP_ENTRY_MM		20	/* ...held for 20 mm: start wobble / tilt ignored */
#define HUMP_EXIT_UM		1000	/* "level" band                        */
#define HUMP_PEAK_DROP_UM	3000	/* peak confirmed after a 3 mm drop    */
#define HUMP_MAX_CLIMB_MM	500	/* longer: ramp or bad reading         */
#define HUMP_FALSE_LEVEL_MM	50	/* level again without a peak: discard */
#define HUMP_PROFILE_STEPS	200	/* 1 m of profile                      */

typedef enum {
	HS_LEVEL	= 0,
	HS_CLIMBING	= 1,
	HS_DESCENDING	= 2,
	HS_REAR_CROSSING= 3
} HUMP_STATE;

typedef enum {
	HE_NONE		= 0,
	HE_START	= 1,
	HE_END		= 2,	/* *peak_um valid */
	HE_DISCARD	= 3
} HUMP_EVENT;

typedef struct {
	HUMP_STATE	state;
	UW		start_01mm;	/* path at hump start            */
	UW		mark_01mm;	/* path of the last state change */
	UW		level_01mm;	/* path where "level" began      */
	BOOL		level_run;
	BOOL		entry_run;
	UW		entry_01mm;	/* path where dh first passed entry */
	INT		k;		/* next profile index            */
	W		peak_um;
	W		prof[HUMP_PROFILE_STEPS];
} HUMP_DET;

IMPORT void       hump_reset(HUMP_DET *h);
IMPORT HUMP_EVENT hump_update(HUMP_DET *h, W dh_um, UW path_01mm, W *peak_um);
/* dh from accelerometer (mg): L * a_fwd / |a| in µm */
IMPORT W          hump_dh_um(W a_fwd_mg, W a_mag_mg);
IMPORT UW         hump_isqrt(UW v);

#endif /* HUMP_H */
