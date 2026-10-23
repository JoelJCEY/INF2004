/*
 *	drv_encoder.h
 *	Subsystem 2 -- wheel encoder counting, bring-up version.
 *
 *	Step 1 counts rising edges of each encoder's B output in hardware,
 *	using a spare PWM slice in "count edges on pin B" mode (no CPU, no
 *	interrupt).  It does NOT know direction: that comes in step 2 with a
 *	PIO quadrature decoder that also uses the A outputs.
 *	Call only from motion_task.
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include "subsystem2.h"

IMPORT ER	encoder_init(void);
/* Total edges since encoder_init().  Extends the 16-bit hardware counter
   in software, so call at least once per 65535 edges. */
IMPORT UW	encoder_count(S2_SIDE side);

#endif /* DRV_ENCODER_H */
