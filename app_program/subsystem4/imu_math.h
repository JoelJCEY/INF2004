/*
 *	imu_math.h
 *	Subsystem 4 -- integer-only helpers (no libm is linked, and the M0+
 *	has no FPU, so floats would be slow software emulation).
 */

#ifndef IMU_MATH_H
#define IMU_MATH_H

#include "subsystem4.h"

IMPORT UW	imu_isqrt(UW n);
/* atan2(y, x) in tenths of a degree, -1800..1800.  Error < 0.3 deg. */
IMPORT W	imu_atan2_deg10(W y, W x);

#endif /* IMU_MATH_H */
