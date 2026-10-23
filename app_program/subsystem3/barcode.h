/*
 *	barcode.h
 *	Subsystem 3 -- Code 39 reader fed one IR_R sample at a time.
 *	Pure logic (no hardware), so it can be unit-tested on target.
 */

#ifndef BARCODE_H
#define BARCODE_H

#include "subsystem3.h"
#include "../subsystem5/subsystem5.h"	/* BARCODE_CMD */

#define BC_ELEMENTS		29	/* 3 characters x 9 + 2 gaps */
#define BC_QUIET_MIN		40	/* samples of white before a start */

typedef enum {
	BCR_NONE = 0,		/* nothing finished this sample */
	BCR_DECODED,		/* *letter = character, read is valid */
	BCR_REJECTED		/* partial / invalid / over-long read */
} BC_RESULT;

IMPORT void		barcode_reset(void);
/* Feed one sample: black = TRUE if IR_R is over a bar.  Returns the
 * outcome; on BCR_DECODED *p_letter holds the middle character. */
IMPORT BC_RESULT	barcode_feed(BOOL black, UB *p_letter);
/* Decode 29 element widths (bar, space, bar ...).  Tries both reading
 * directions.  Returns the middle character or 0 if invalid. */
IMPORT UB		barcode_decode29(const UH widths[BC_ELEMENTS]);
IMPORT BARCODE_CMD	barcode_command(UB letter);

#endif /* BARCODE_H */
