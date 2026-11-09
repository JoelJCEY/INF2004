/*
 *	scan_profile.c
 *	Subsystem 5 -- obstacle width, edges and bypass side from a sweep.
 */

#include "scan_profile.h"
#include "drv_ultrasonic.h"		/* US_FAR_MM */
#include "../subsystem4/hump.h"		/* hump_isqrt */

/* cos(30° + 5° i) x 1000, i = 0..24: the lateral component */
LOCAL const H cos_tab[SCAN_POSITIONS] = {
	 866,  819,  766,  707,  643,  574,  500,  423,  342,  259,
	 174,   87,    0,  -87, -174, -259, -342, -423, -500, -574,
	-643, -707, -766, -819, -866
};

/* sin(30° + 5° i) x 1000: the forward component of each reading */
LOCAL const H sin_tab[SCAN_POSITIONS] = {
	 500,  574,  643,  707,  766,  819,  866,  906,  940,  966,
	 985,  996, 1000,  996,  985,  966,  940,  906,  866,  819,
	 766,  707,  643,  574,  500
};

EXPORT W scan_cos1000(UINT idx)
{
	W	c = cos_tab[(idx < SCAN_POSITIONS) ? idx : 12U];

	return SCAN_RIGHT_IS_LOW_ANGLE ? c : -c;	/* x > 0 = right */
}

EXPORT W scan_sin1000(UINT idx)
{
	return sin_tab[(idx < SCAN_POSITIONS) ? idx : 12U];
}

LOCAL BOOL measured(const SCAN_SWEEP *s, INT i)
{
	return (s->dist[i] != SCAN_NO_READING) ? TRUE : FALSE;
}

/* Is any measured point outside the obstacle, on this side (right:
   x > 0), inside the corridor the car would use to pass at a lateral
   offset of need_mm? */
LOCAL BOOL side_blocked(const SCAN_SWEEP *s, INT lo, INT hi, BOOL right,
			W need_mm, W closest)
{
	INT	i;
	W	x, y, d;

	for (i = 0; i < SCAN_POSITIONS; i++) {
		if (((i >= lo) && (i <= hi)) || !measured(s, i) ||
		    (s->dist[i] >= SCAN_RANGE_MM)) {	/* far = floor / background */
			continue;
		}
		d = s->dist[i];
		x = (d * scan_cos1000((UINT)i)) / 1000;
		y = (d * scan_sin1000((UINT)i)) / 1000;
		if ((right && (x <= 0)) || (!right && (x >= 0))) {
			continue;
		}
		if ((APP_ABS(x) < (need_mm + SCAN_CAR_HALF_W_MM)) &&
		    (y < (closest + SCAN_OBST_DEPTH_MM + SCAN_CAR_LEN_MM))) {
			return TRUE;
		}
	}
	return FALSE;
}

EXPORT ER scan_profile(const SCAN_SWEEP *s, SCAN_RESULT *r)
{
	INT	i, ci = -1, lo, hi;
	W	closest = 0x7FFF;
	W	x_lo, y_lo, x_hi, y_hi, dx, dy;
	W	need_r, need_l;
	BOOL	blk_r, blk_l;

	/* Seed: the closest point in the car's own path (the one that
	   raised the alert); a wall off to the side is not the obstacle */
	for (i = 0; i < SCAN_POSITIONS; i++) {
		W x;

		if (!measured(s, i) || (s->dist[i] >= closest)) {
			continue;
		}
		x = APP_ABS(((W)s->dist[i] * scan_cos1000((UINT)i)) / 1000);
		if (x <= (SCAN_CAR_HALF_W_MM + SCAN_MARGIN_MM)) {
			closest = s->dist[i];
			ci = i;
		}
	}
	/* Nothing in the car's own path: no obstacle to go around, even if
	   something stands off to the side (28 Oct: a wall or the car's own
	   bracket at the sweep ends started a bypass of nothing) */
	if ((ci < 0) || (closest >= SCAN_RANGE_MM)) {
		return E_OBJ;		/* nothing in range: no obstacle */
	}

	/* Grow the obstacle from the closest point while neighbours are
	   within the band (unmeasured positions are skipped over) */
	lo = ci;
	for (i = ci - 1; i >= 0; i--) {
		if (!measured(s, i)) continue;
		if ((W)s->dist[i] > closest + SCAN_OBST_BAND_MM) break;
		lo = i;
	}
	hi = ci;
	for (i = ci + 1; i < SCAN_POSITIONS; i++) {
		if (!measured(s, i)) continue;
		if ((W)s->dist[i] > closest + SCAN_OBST_BAND_MM) break;
		hi = i;
	}

	x_lo = ((W)s->dist[lo] * scan_cos1000((UINT)lo)) / 1000;
	y_lo = ((W)s->dist[lo] * scan_sin1000((UINT)lo)) / 1000;
	x_hi = ((W)s->dist[hi] * scan_cos1000((UINT)hi)) / 1000;
	y_hi = ((W)s->dist[hi] * scan_sin1000((UINT)hi)) / 1000;
	dx = x_lo - x_hi;
	dy = y_lo - y_hi;

	r->lo_idx = (UB)lo;
	r->hi_idx = (UB)hi;
	r->closest_mm = (UH)closest;
	r->width_mm = (UH)hump_isqrt((UW)((dx * dx) + (dy * dy)));
	/* Forward leg after the sideways move.  31 Oct: it must also cover
	   the gap to the obstacle (closest).  Without it the leg ended with
	   the car's rear still beside the obstacle (30 Oct video), and the
	   turn back in clipped it.  Distance reversed is added by ctrl. */
	r->depth_mm = (UH)(closest +
			   APP_CLAMP((W)r->width_mm, SCAN_DEPTH_MIN_MM, SCAN_DEPTH_MAX_MM) +
			   SCAN_CAR_LEN_MM);

	/* Lateral move to clear each edge (FR4: keep the detour short) */
	need_r = ((x_lo > x_hi) ? x_lo : x_hi);		/* right-most edge */
	need_l = -((x_lo < x_hi) ? x_lo : x_hi);	/* left-most edge  */
	need_r = ((need_r > 0) ? need_r : 0) + SCAN_CAR_HALF_W_MM + SCAN_MARGIN_MM
		 + SCAN_PASS_EXTRA_MM;
	need_l = ((need_l > 0) ? need_l : 0) + SCAN_CAR_HALF_W_MM + SCAN_MARGIN_MM
		 + SCAN_PASS_EXTRA_MM;
	/* The narrow sweep may end before the obstacle does: an edge that
	   touches the sweep limit is not the real edge, so keep extra room */
	if (lo <= SCAN_IDX_MIN) {
		need_r += SCAN_EDGE_EXTRA_MM;	/* low angle = right side */
	}
	if (hi >= SCAN_IDX_MAX) {
		need_l += SCAN_EDGE_EXTRA_MM;
	}

	blk_r = side_blocked(s, lo, hi, TRUE, need_r, closest);
	blk_l = side_blocked(s, lo, hi, FALSE, need_l, closest);

	r->kind = OB_PROFILE;
	/* 4 Nov (team): always go LEFT unless the left is blocked
	   (31 Oct it was right first) */
	if (!blk_l) {
		r->side = (UB)OBS_SIDE_LEFT;
		r->offset_mm = (UH)need_l;
	} else if (!blk_r) {
		r->side = (UB)OBS_SIDE_RIGHT;
		r->offset_mm = (UH)need_r;
	} else {
		r->kind = OB_NO_CLEARANCE;
		r->side = (UB)OBS_SIDE_NONE;
		r->offset_mm = 0;
	}
	return E_OK;
}
