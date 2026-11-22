/*
 *	lift.h
 *	Subsystem 4 -- "lifted or overturned" detector (pure logic).
 *
 *	13 Nov requirement change: the IMU no longer detects collisions.  It
 *	detects the car being LIFTED (the demo's end-of-run signal) or
 *	OVERTURNED, and the motors must stop at once.
 *
 *	Only the accelerometer is used (the LSM303DLHC has no gyro).  At
 *	start-up the car stands level, so its reading g0 (about 1 g) is
 *	"which way is down".  Every 20 ms:
 *	  LIFTED      the push UP along g0 is more than LIFT_UP_MG above
 *	              1 g, averaged over LIFT_AVG_N readings (120 ms).  Lifting
 *	              a car by hand pushes it up at roughly 0.3-1 g for
 *	              0.1-0.3 s.  Driving, accelerating and turning push
 *	              SIDEWAYS to g0, and a hump tilts the car, which makes the
 *	              reading along g0 smaller, never bigger.  A bump spike
 *	              lasts one reading and is averaged away.
 *	  OVERTURNED  the filtered reading points more than LIFT_TILT_DEG away
 *	              from g0 for LIFT_TILT_N readings: on its side, upside
 *	              down, or held tilted after being picked up.  The
 *	              steepest hump is about 15 deg.
 */
#ifndef LIFT_H
#define LIFT_H

#include "subsystem4.h"

#define LIFT_UP_MG		200	/* upward push to count as a lift   */
#define LIFT_UP_HUMP_MG		400	/* while crossing a hump (landing)  */
/* readings averaged (6 x 20 ms; 13 Nov, was 4) */
#define LIFT_AVG_N		6
/* (13 Nov, was 3) ...and at least this many of them over the limit: a lift is a
   steady push, a bump is one or two readings */
#define LIFT_UP_N		5
#define LIFT_TILT_COS1000	500	/* cos(60 deg): tilted this far     */
#define LIFT_TILT_N		3	/* readings in a row (60 ms)        */

typedef enum {
	LIFT_NONE	= 0,
	LIFT_LIFTED	= 1,
	LIFT_OVERTURNED	= 2
} LIFT_RESULT;

typedef struct {
	W	g0[3];			/* level reading at start-up (mg) */
	W	g0_mag;
	W	up[LIFT_AVG_N];		/* push along g0, last readings   */
	INT	up_over;		/* how many of them over the limit */
	INT	ui;
	INT	un;
	INT	tilt_n;
	W	last_up_mg;		/* averaged push above 1 g (log)  */
	W	last_cos1000;		/* tilt from level, x 1000 (log)  */
} LIFT_DET;

IMPORT void        lift_init(LIFT_DET *d, W x0, W y0, W z0);
/* raw: this reading (mg).  filt: low-pass filtered reading (mg).
   on_hump: TRUE while the hump detector is not LEVEL. */
IMPORT LIFT_RESULT lift_update(LIFT_DET *d, const W raw[3], const W filt[3],
	BOOL on_hump);

#endif /* LIFT_H */
