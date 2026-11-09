/*
 *	hump.c
 *	Subsystem 4 -- hump detector (statechart 6.4).
 */

#include "hump.h"

#define L_STEPS		(HUMP_WHEELBASE_MM / HUMP_STEP_MM)

EXPORT void hump_reset(HUMP_DET *h)
{
	h->state = HS_LEVEL;
	h->k = 0;
	h->peak_um = 0;
	h->level_run = FALSE;
	h->entry_run = FALSE;
}

EXPORT UW hump_isqrt(UW v)
{
	UW	r = 0;
	UW	bit = 1UL << 30;

	while (bit > v) {
		bit >>= 2;
	}
	while (bit != 0U) {
		if (v >= r + bit) {
			v -= r + bit;
			r = (r >> 1) + bit;
		} else {
			r >>= 1;
		}
		bit >>= 2;
	}
	return r;
}

EXPORT W hump_dh_um(W a_fwd_mg, W a_mag_mg)
{
	if (a_mag_mg <= 0) {
		return 0;
	}
	/* L(mm) * 1000 * a_fwd / |a| */
	return (W)(((W)HUMP_WHEELBASE_MM * 1000 * a_fwd_mg) / a_mag_mg);
}

/* Extend the profile up to the current distance */
LOCAL BOOL extend_profile(HUMP_DET *h, W dh_um, UW path_01mm)
{
	UW	travelled = path_01mm - h->start_01mm;
	W	rear;

	while (travelled >= (UW)(h->k * HUMP_STEP_MM * 10)) {
		if (h->k >= HUMP_PROFILE_STEPS) {
			return FALSE;
		}
		rear = (h->k >= L_STEPS) ? h->prof[h->k - L_STEPS] : 0;
		h->prof[h->k] = dh_um + rear;
		if (h->prof[h->k] > h->peak_um) {
			h->peak_um = h->prof[h->k];
		}
		h->k++;
	}
	return TRUE;
}

LOCAL W current_height(const HUMP_DET *h)
{
	return (h->k > 0) ? h->prof[h->k - 1] : 0;
}

EXPORT HUMP_EVENT hump_update(HUMP_DET *h, W dh_um, UW path_01mm, W *peak_um)
{
	W	hf;

	if (h->state == HS_LEVEL) {
		if (dh_um <= HUMP_ENTRY_UM) {
			h->entry_run = FALSE;
			return HE_NONE;
		}
		if (!h->entry_run) {
			h->entry_run = TRUE;
			h->entry_01mm = path_01mm;
		}
		/* Climb only once the tilt has held for HUMP_ENTRY_MM */
		if ((path_01mm - h->entry_01mm) >= (UW)(HUMP_ENTRY_MM * 10)) {
			h->state = HS_CLIMBING;
			h->start_01mm = h->entry_01mm;
			h->mark_01mm = path_01mm;
			h->k = 0;
			h->peak_um = 0;
			h->level_run = FALSE;
			(void)extend_profile(h, dh_um, path_01mm);
			return HE_START;
		}
		return HE_NONE;
	}

	if (!extend_profile(h, dh_um, path_01mm)) {
		hump_reset(h);		/* longer than the profile: not a hump */
		return HE_DISCARD;
	}
	hf = current_height(h);

	switch (h->state) {
	case HS_CLIMBING:
		if ((path_01mm - h->start_01mm) > (UW)(HUMP_MAX_CLIMB_MM * 10)) {
			hump_reset(h);
			return HE_DISCARD;	/* ramp or bad reading */
		}
		if ((h->peak_um - hf) >= HUMP_PEAK_DROP_UM) {
			h->state = HS_DESCENDING;	/* peak confirmed */
			h->mark_01mm = path_01mm;
			break;
		}
		/* Level-return exit: a trigger that never formed a peak */
		if ((APP_ABS(dh_um) < HUMP_EXIT_UM) && (h->peak_um < HUMP_PEAK_DROP_UM)) {
			if (!h->level_run) {
				h->level_run = TRUE;
				h->level_01mm = path_01mm;
			} else if ((path_01mm - h->level_01mm) >= (UW)(HUMP_FALSE_LEVEL_MM * 10)) {
				hump_reset(h);
				return HE_DISCARD;
			} else {
				/* still counting */
			}
		} else {
			h->level_run = FALSE;
		}
		break;
	case HS_DESCENDING:
		if (hf <= HUMP_EXIT_UM) {
			h->state = HS_REAR_CROSSING;	/* front wheels down */
			h->mark_01mm = path_01mm;
		}
		break;
	case HS_REAR_CROSSING:
		/* Rear wheels cross; report once they are past and level */
		if (((path_01mm - h->mark_01mm) >= (UW)((HUMP_WHEELBASE_MM + 20) * 10)) &&
		    (APP_ABS(dh_um) < HUMP_EXIT_UM)) {
			*peak_um = h->peak_um;
			hump_reset(h);
			return HE_END;
		}
		break;
	default:
		hump_reset(h);
		break;
	}
	return HE_NONE;
}
