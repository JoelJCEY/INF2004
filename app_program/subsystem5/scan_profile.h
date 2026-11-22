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

/* ---------------------------------------------------------------------
 *  13 Nov: glance watch covers the WHOLE car width (task_scan.c).
 *  While driving the sensor looks 90, 80, 70, 80, 90, 100, 110, 100 deg
 *  (straight, right, right, ..., left).  With the ~15 deg beam that is
 *  +-27.5 deg: at the 20 cm stop distance about +-10 cm, more than the
 *  car's +-8 cm, so an object in front of a CORNER is seen too.
 *  A corner object is only in the side beams while it is still far
 *  away (it slides out of the beam as the car gets close), so each
 *  in-path echo is REMEMBERED as a point on the track (odometry path +
 *  distance ahead).  Two points within GMEM_MATCH_MM of each other =
 *  a real object (one stray echo is not enough).  The car stops when
 *  the remembered object is predicted to be SCAN_STOP_DIST_MM ahead,
 *  even if the beam no longer sees it.
 * --------------------------------------------------------------------- */
#define GLANCE_SEQ_N		8
#define GLANCE_LOOK_MM		350	/* in-path echoes up to this far ahead are remembered */
#define GMEM_N			12	/* remembered points                   */
#define GMEM_MATCH_MM		60	/* same object: this close along and across */
#define GMEM_KEEP_GLANCES	16	/* forget a point after 2 full cycles (~3 s) */
#define GMEM_NONE		(-1)

typedef struct {
	W	at[GMEM_N];		/* path mm where the point is          */
	W	side[GMEM_N];		/* mm, + = right                      */
	UW	when[GMEM_N];		/* glance counter when it was seen    */
	UB	used[GMEM_N];
	UINT	next;
	W	obj_at;			/* confirmed object (path mm), or GMEM_NONE */
} GLANCE_MEM;

IMPORT const INT glance_seq[GLANCE_SEQ_N];	/* servo index offsets from centre */
IMPORT void gmem_clear(GLANCE_MEM *g);
/* One in-path echo: ahead/side from the car now, at odometry path_mm. */
IMPORT void gmem_add(GLANCE_MEM *g, W path_mm, W ahead_mm, W side_mm, UW glance_no);
/* Forget old / passed points.  Returns the predicted distance ahead of
   the confirmed object, or GMEM_NONE. */
IMPORT W    gmem_ahead(GLANCE_MEM *g, W path_mm, UW glance_no);
/* Edge-only obstacle: the stop sweep sees it at the end angle only */
IMPORT BOOL scan_confirm_run(INT best_run, BOOL touches_end);

IMPORT W   scan_cos1000(UINT idx);	/* cos of the servo angle x1000 */
IMPORT W   scan_sin1000(UINT idx);
IMPORT ER  scan_profile(const SCAN_SWEEP *s, SCAN_RESULT *r);

#endif /* SCAN_PROFILE_H */
