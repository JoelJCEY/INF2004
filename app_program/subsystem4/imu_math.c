/*
 *	imu_math.c
 *	Subsystem 4 -- integer square root and atan2.
 */

#include "imu_math.h"

EXPORT UW imu_isqrt(UW n)
{
	UW	root = 0U;
	UW	bit = 1UL << 30;
	UW	rem = n;

	while (bit > rem) {
		bit >>= 2;
	}
	while (bit != 0U) {
		if (rem >= (root + bit)) {
			rem -= root + bit;
			root = (root >> 1) + bit;
		} else {
			root >>= 1;
		}
		bit >>= 2;
	}
	return root;
}

/*
 * atan(r) for r = a / 32768 in [0, 1], result in hundredths of a degree:
 *   atan(r) ~ 45 r + r (1 - r) (14.02 + 3.80 r)   degrees
 * (a standard polynomial approximation, max error about 0.25 degree).
 */
LOCAL UW imu_atan_unit_deg100(UW a)
{
	UW	b = 32768U - a;
	UW	t = (a * b) >> 15;			/* r (1 - r), Q15 */
	UW	k = 1402U + ((380U * a) >> 15);	/* (14.02 + 3.80 r) x 100 */

	return ((4500U * a) + (t * k)) >> 15;
}

EXPORT W imu_atan2_deg10(W y, W x)
{
	UW	ay = (y < 0) ? (UW)(-y) : (UW)y;
	UW	ax = (x < 0) ? (UW)(-x) : (UW)x;
	W	ang;

	if ((ax == 0U) && (ay == 0U)) {
		return 0;
	}
	/* Inputs here are accelerations in mg (|v| < 20000), so v * 32768
	   fits in 32 bits. */
	if (ay <= ax) {
		ang = (W)(imu_atan_unit_deg100((ay * 32768U) / ax) / 10U);
	} else {
		ang = 900 - (W)(imu_atan_unit_deg100((ax * 32768U) / ay) / 10U);
	}
	if (x < 0) {
		ang = 1800 - ang;
	}
	if (y < 0) {
		ang = -ang;
	}
	return ang;
}
