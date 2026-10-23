/*
 *	drv_encoder.h
 *	Subsystem 2 -- wheel encoders (COMPLETE VERSION).
 *
 *	Wi-Fi builds (the default): both quadrature encoders are decoded in
 *	hardware by the pico-examples PIO program, one state machine per
 *	wheel, A/B on adjacent pins (GP2/GP3, GP4/GP5).  Counts are signed
 *	and x4 (every edge of A and B), with no interrupts.
 *	WIFI=none builds fall back to the basic PWM-slice edge counter
 *	(B rising edges only, unsigned), because the SDK PIO library is only
 *	linked in Wi-Fi builds.
 *	Call only from motion_task.
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include "subsystem2.h"

IMPORT ER	encoder_init(void);
/* Count since init.  Signed when encoder_is_signed() is TRUE. */
IMPORT W	encoder_count(S2_SIDE side);
IMPORT BOOL	encoder_is_signed(void);
/* Which PIO block / state machines / offset were used (boot log, A2). */
IMPORT void	encoder_report(void);

#endif /* DRV_ENCODER_H */
