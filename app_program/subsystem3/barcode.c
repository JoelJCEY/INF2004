/*
 *	barcode.c
 *	Subsystem 3 -- Code 39 reader (statechart 6.3: WAITING FOR BARCODE,
 *	READING BARS, DECODING).
 *
 *	A Code 39 symbol "*X*" is 3 characters of 9 elements (5 bars, 4
 *	spaces, exactly 3 of them wide) separated by 2 narrow gaps: 29
 *	elements.  Widths are measured in samples (1 kHz), and each
 *	character is decoded from which 3 of its 9 elements are widest, so
 *	the result does not depend on the car's speed as long as the speed is
 *	steady across the symbol.  Both reading directions are tried.
 */

#include "barcode.h"

typedef enum {
	BS_QUIET = 0,		/* counting white before a start */
	BS_READING
} BC_STATE;

LOCAL BC_STATE	bc_state = BS_QUIET;
LOCAL BOOL	bc_colour = FALSE;	/* colour of the current run */
LOCAL UW	bc_run = 0U;		/* length of the current run */
LOCAL UH	bc_w[BC_ELEMENTS];
LOCAL UINT	bc_n = 0U;		/* completed elements */
LOCAL UW	bc_min = 0U;		/* narrowest element so far */
LOCAL BOOL	bc_in = FALSE;		/* debounced input */
LOCAL UINT	bc_pending = 0U;	/* samples the raw input has disagreed */

#define BC_MAX_BAR		400U	/* black longer than this: not a bar */
#define BC_DEBOUNCE		2U	/* a colour must hold 2 samples to count */
#define BC_END_X_NARROW		5U	/* white > 5 narrow widths = quiet zone */
#define BC_WIDE_RATIO_X10	15	/* narrowest wide >= 1.5 x widest narrow */

/* Code 39 patterns, 9 elements, first element = most significant bit,
 * 1 = wide.  Standard table. */
typedef struct {
	UB	ch;
	UH	pat;
} C39_ENTRY;

LOCAL const C39_ENTRY c39_table[] = {
	{ '0', 0x034 }, { '1', 0x121 }, { '2', 0x061 }, { '3', 0x160 },
	{ '4', 0x031 }, { '5', 0x130 }, { '6', 0x070 }, { '7', 0x025 },
	{ '8', 0x124 }, { '9', 0x064 }, { 'A', 0x109 }, { 'B', 0x049 },
	{ 'C', 0x148 }, { 'D', 0x019 }, { 'E', 0x118 }, { 'F', 0x058 },
	{ 'G', 0x00D }, { 'H', 0x10C }, { 'I', 0x04C }, { 'J', 0x01C },
	{ 'K', 0x103 }, { 'L', 0x043 }, { 'M', 0x142 }, { 'N', 0x013 },
	{ 'O', 0x112 }, { 'P', 0x052 }, { 'Q', 0x007 }, { 'R', 0x106 },
	{ 'S', 0x046 }, { 'T', 0x016 }, { 'U', 0x181 }, { 'V', 0x0C1 },
	{ 'W', 0x1C0 }, { 'X', 0x091 }, { 'Y', 0x190 }, { 'Z', 0x0D0 },
	{ '-', 0x085 }, { '.', 0x184 }, { ' ', 0x0C4 }, { '*', 0x094 },
	{ '$', 0x0A8 }, { '/', 0x0A2 }, { '+', 0x08A }, { '%', 0x02A }
};
#define C39_COUNT	(sizeof(c39_table) / sizeof(c39_table[0]))

/* Letter -> command.  TO BE CONFIRMED WITH TEACHING STAFF: the newer
 * sample set has A, F, Z, W, X and Y, but no B, C or D. */
EXPORT BARCODE_CMD barcode_command(UB letter)
{
	BARCODE_CMD	cmd;

	switch (letter) {
	case 'A':
		cmd = BC_TURN_LEFT;
		break;
	case 'B':
		cmd = BC_TURN_RIGHT;
		break;
	case 'C':
		cmd = BC_STRAIGHT;
		break;
	case 'D':
		cmd = BC_U_TURN;
		break;
	default:
		cmd = BC_UNRECOGNISED;	/* FR2: report and continue straight */
		break;
	}
	return cmd;
}

/* One 9-element character -> ASCII, or 0. */
LOCAL UB c39_char(const UH *p_w)
{
	UINT	idx[3] = { 0U, 0U, 0U };
	UINT	i;
	UINT	k;
	UH	pat = 0U;
	UH	min_wide = 0xFFFFU;
	UH	max_narrow = 0U;

	/* indices of the 3 widest elements (simple selection) */
	for (k = 0U; k < 3U; k++) {
		UH	best = 0U;
		UINT	best_i = 0U;
		for (i = 0U; i < 9U; i++) {
			BOOL	used = FALSE;
			UINT	j;
			for (j = 0U; j < k; j++) {
				if (idx[j] == i) {
					used = TRUE;
				}
			}
			if ((used == FALSE) && (p_w[i] >= best)) {
				best = p_w[i];
				best_i = i;
			}
		}
		idx[k] = best_i;
		pat |= (UH)(1U << (8U - best_i));
	}
	for (i = 0U; i < 9U; i++) {
		if ((pat & (UH)(1U << (8U - i))) != 0U) {
			if (p_w[i] < min_wide) {
				min_wide = p_w[i];
			}
		} else if (p_w[i] > max_narrow) {
			max_narrow = p_w[i];
		} else {
			;
		}
	}
	/* wide must be clearly wider than narrow (about 3x in practice) */
	if (((UW)min_wide * 10U) < ((UW)max_narrow * (UW)BC_WIDE_RATIO_X10)) {
		return 0U;
	}
	for (i = 0U; i < C39_COUNT; i++) {
		if (c39_table[i].pat == pat) {
			return c39_table[i].ch;
		}
	}
	return 0U;
}

LOCAL UB c39_decode_dir(const UH *p_w)
{
	UB	a = c39_char(&p_w[0]);
	UB	b = c39_char(&p_w[10]);
	UB	c = c39_char(&p_w[20]);

	if ((a == '*') && (c == '*') && (b != 0U) && (b != '*')) {
		return b;
	}
	return 0U;
}

EXPORT UB barcode_decode29(const UH widths[BC_ELEMENTS])
{
	UH	rev[BC_ELEMENTS];
	UINT	i;
	UB	ch = c39_decode_dir(widths);

	if (ch == 0U) {
		for (i = 0U; i < (UINT)BC_ELEMENTS; i++) {
			rev[i] = widths[(UINT)BC_ELEMENTS - 1U - i];
		}
		ch = c39_decode_dir(rev);
	}
	return ch;
}

EXPORT void barcode_reset(void)
{
	bc_state = BS_QUIET;
	bc_colour = FALSE;
	bc_run = 0U;
	bc_n = 0U;
	bc_min = 0U;
	bc_in = FALSE;
	bc_pending = 0U;
}

EXPORT BC_RESULT barcode_feed(BOOL raw_black, UB *p_letter)
{
	BC_RESULT	res = BCR_NONE;
	BOOL		black;

	/* Debounce: a single-sample glitch is ignored.  Both edges of a bar
	 * are delayed by the same amount, so widths are unchanged. */
	if (raw_black != bc_in) {
		if (++bc_pending >= BC_DEBOUNCE) {
			bc_in = raw_black;
			bc_pending = 0U;
		}
	} else {
		bc_pending = 0U;
	}
	black = bc_in;

	if (black == bc_colour) {
		if (bc_run < 0xFFFFU) {
			bc_run++;
		}
	} else {
		/* a run just ended */
		if ((bc_state == BS_READING) && (bc_run > 0U)) {
			if (bc_n < (UINT)BC_ELEMENTS) {
				bc_w[bc_n] = (UH)bc_run;
				if ((bc_min == 0U) || (bc_run < bc_min)) {
					bc_min = bc_run;
				}
				bc_n++;
			} else {
				/* a 30th element: over-long read */
				bc_state = BS_QUIET;
				bc_n = 0U;
				bc_min = 0U;
				bc_colour = black;
				bc_run = 1U;
				return BCR_REJECTED;
			}
		} else if ((bc_state == BS_QUIET) && (black == TRUE) &&
			   (bc_run >= (UW)BC_QUIET_MIN)) {
			bc_state = BS_READING;	/* start bar after a quiet zone */
			bc_n = 0U;
			bc_min = 0U;
		} else {
			;
		}
		bc_colour = black;
		bc_run = 1U;
	}

	if (bc_state == BS_READING) {
		UW	end_len = bc_min * BC_END_X_NARROW;

		if (end_len < ((UW)BC_QUIET_MIN / 2U)) {
			end_len = (UW)BC_QUIET_MIN / 2U;
		}
		if ((black == TRUE) && (bc_run > BC_MAX_BAR)) {
			/* far too wide for a bar: the line or a junction */
			bc_state = BS_QUIET;
			bc_n = 0U;
			bc_min = 0U;
			res = BCR_REJECTED;
		} else if ((black == FALSE) && (bc_n > 0U) && (bc_run > end_len)) {
			/* trailing quiet zone: the symbol has ended */
			if (bc_n == (UINT)BC_ELEMENTS) {
				UB	ch = barcode_decode29(bc_w);
				res = (ch != 0U) ? BCR_DECODED : BCR_REJECTED;
				*p_letter = ch;
			} else {
				res = BCR_REJECTED;	/* partial read */
			}
			bc_state = BS_QUIET;	/* this white counts as quiet */
			bc_n = 0U;
			bc_min = 0U;
		} else {
			;
		}
	}
	return res;
}
