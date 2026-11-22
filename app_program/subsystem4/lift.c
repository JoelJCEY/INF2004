/*
 *	lift.c
 *	Subsystem 4 -- lifted / overturned detector (see lift.h).
 */

#include "lift.h"
#include "hump.h"		/* hump_isqrt */

EXPORT void lift_init(LIFT_DET *d, W x0, W y0, W z0)
{
	INT	i;

	d->g0[0] = x0;
	d->g0[1] = y0;
	d->g0[2] = z0;
	d->g0_mag = (W)hump_isqrt((UW)((x0 * x0) + (y0 * y0) + (z0 * z0)));
	if (d->g0_mag < 500) {
		d->g0_mag = 1000;	/* bad baseline: assume 1 g (still works for tilt sign) */
	}
	for (i = 0; i < LIFT_AVG_N; i++) {
		d->up[i] = 0;
	}
	d->ui = 0;
	d->un = 0;
	d->up_over = 0;
	d->tilt_n = 0;
	d->last_up_mg = 0;
	d->last_cos1000 = 1000;
}

/* reading . g0 / |g0|  = the part of the reading along "down" (mg) */
LOCAL W along_g0(const LIFT_DET *d, const W a[3])
{
	return ((a[0] * d->g0[0]) + (a[1] * d->g0[1]) + (a[2] * d->g0[2])) / d->g0_mag;
}

EXPORT LIFT_RESULT lift_update(LIFT_DET *d, const W raw[3], const W filt[3], BOOL on_hump)
{
	INT	i;
	W	sum = 0;
	W	fmag;
	W	cos1000;

	/* LIFTED: average push along g0 above 1 g (g0_mag) */
	d->up[d->ui] = along_g0(d, raw) - d->g0_mag;
	d->ui = (d->ui + 1) % LIFT_AVG_N;
	if (d->un < LIFT_AVG_N) {
		d->un++;
	}
	d->up_over = 0;
	for (i = 0; i < d->un; i++) {
		sum += d->up[i];
		if (d->up[i] > (on_hump ? LIFT_UP_HUMP_MG : LIFT_UP_MG)) {
			d->up_over++;
		}
	}
	d->last_up_mg = sum / d->un;

	/* OVERTURNED: angle between the filtered reading and g0 */
	fmag = (W)hump_isqrt((UW)((filt[0] * filt[0]) + (filt[1] * filt[1]) + (filt[2] * filt[2])));
	cos1000 = (fmag > 100) ? ((along_g0(d, filt) * 1000) / fmag) : 0;	/* ~free fall: no direction */
	d->last_cos1000 = cos1000;
	d->tilt_n = (cos1000 < LIFT_TILT_COS1000) ? (d->tilt_n + 1) : 0;

	if ((d->un >= LIFT_AVG_N) && (d->up_over >= LIFT_UP_N) &&
	    (d->last_up_mg > (on_hump ? LIFT_UP_HUMP_MG : LIFT_UP_MG))) {
		return LIFT_LIFTED;
	}
	if (d->tilt_n >= LIFT_TILT_N) {
		return LIFT_OVERTURNED;
	}
	return LIFT_NONE;
}
