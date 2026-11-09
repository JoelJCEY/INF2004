/*
 *	scan_profile.h
 *	Subsystem 5 -- obstacle profile from a servo sweep (pure logic).
 *
 *	Servo angle 90° looks straight ahead.  Low angles look RIGHT, high
 *	angles look LEFT (check the mounting, SCAN_RIGHT_IS_LOW_ANGLE).
 *	Each reading becomes a point: x = d cos(θ) (right +), y = d sin(θ).
 *	Width needs the two edge angles: one straight-ahead ping cannot give
 *	it, the beam is ~15° wide (8.4 Buddy 5).
 */

#ifndef SCAN_PROFILE_H
#define SCAN_PROFILE_H

#include "subsystem5.h"

#define SCAN_POSITIONS		25	/* 30°..150° in 5° steps */
#define SCAN_RIGHT_IS_LOW_ANGLE	1
#define SCAN_STOP_DIST_MM	200	/* 20 cm (31 Oct; 30 Oct was 15, before that 7 cm): readings under 4 cm are the car's
					   own bracket, so 5 cm left too small a
					   window to catch a real obstacle */
#define SCAN_OBST_BAND_MM	150	/* points within closest+band = same obstacle */
#define SCAN_CONFIRM_MM		300	/* after the stop: must be closer than this
					   (31 Oct: 30 cm, was 25; stays 10 cm
					   beyond the stop distance) */
#define SCAN_CONFIRM_HITS	2	/* ...at this many neighbouring 5° angles   */
#define SCAN_RANGE_MM		400	/* farther points are background (was 600; 28 Oct:
					   floor echoes appear farther out)       */
/* Sweep only straight ahead (team, 28 Oct): 60..120 deg instead of
   30..150, so the sensor does not pick up the floor or things off to
   the side.  Indexes into the 5-degree table. */
/* 31 Oct: 70..120 deg (was 30..150).  The full swing shook the sensor
   loose and at 60 deg it hit the car.  The sweep still covers the car's
   path (half width + margin = 14 cm) at the 25 cm confirm distance. */
#define SCAN_IDX_MIN		8	/*  70 deg: right end */
#define SCAN_IDX_MAX		18	/* 120 deg: left end  */
#define SCAN_EDGE_EXTRA_MM	100	/* extra sideways move if the obstacle
					   is wider than the sweep */
#define SCAN_CAR_HALF_W_MM	80	/* half the car width                     */
#define SCAN_MARGIN_MM		60	/* clearance kept on the passing side     */
#define SCAN_PASS_EXTRA_MM	40	/* 31 Oct: extra sideways room on the
					   bypass only (30 Oct video: car passed
					   right beside the bottle)               */
#define SCAN_OBST_DEPTH_MM	150	/* depth assumed for the side-corridor check */
/* 4 Nov: the forward-leg estimate assumes the obstacle is as deep as it
   is wide (bottles, boxes), clamped to this range.  The sensor cannot see
   behind it; a fixed 150 mm sent the car far past a 7 cm bottle. */
#define SCAN_DEPTH_MIN_MM	50
#define SCAN_DEPTH_MAX_MM	200
#define SCAN_CAR_LEN_MM		200
#define SCAN_NO_READING		0U	/* dist[] value for "not measured"        */

typedef struct {
	UH	dist[SCAN_POSITIONS];	/* mm, US_FAR_MM if far, 0 if not measured */
} SCAN_SWEEP;

typedef struct {
	UB	kind;		/* OB_PROFILE or OB_NO_CLEARANCE */
	UB	side;		/* OBS_SIDE_xxx                  */
	UB	lo_idx;		/* right edge index              */
	UB	hi_idx;		/* left edge index               */
	UH	width_mm;
	UH	closest_mm;
	UH	offset_mm;	/* lateral move for the chosen side */
	UH	depth_mm;	/* forward leg past the obstacle    */
} SCAN_RESULT;

IMPORT W   scan_cos1000(UINT idx);	/* cos of the servo angle x1000 */
IMPORT W   scan_sin1000(UINT idx);
IMPORT ER  scan_profile(const SCAN_SWEEP *s, SCAN_RESULT *r);

#endif /* SCAN_PROFILE_H */
