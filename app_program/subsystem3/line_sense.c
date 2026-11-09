/*
 *	line_sense.c
 *	Subsystem 3 -- line error, Code 39 decoding and the barcode reader.
 */

#include "line_sense.h"

/* ---------------------------------------------------------------------
 *  Calibration
 * --------------------------------------------------------------------- */
EXPORT void line_cal_defaults(LINE_CAL *cal)
{
	UINT	i;

	for (i = 0; i < 3U; i++) {
		cal->white[i] = LINE_DEF_WHITE;
		cal->black[i] = LINE_DEF_BLACK;
	}
}

EXPORT INT line_normalise(const LINE_CAL *cal, UINT ch, INT raw)
{
	INT	w = (INT)cal->white[ch];
	INT	b = (INT)cal->black[ch];
	INT	n;

	if (b == w) {
		return 0;
	}
#if LINE_BLACK_IS_HIGH
	n = ((raw - w) * 1000) / (b - w);
#else
	n = ((w - raw) * 1000) / (w - b);
#endif
	return APP_CLAMP(n, 0, 1000);
}

/* ---------------------------------------------------------------------
 *  Line position (28 Oct, team rule): keep the CENTRE sensor on the
 *  line and go straight while it is; the RIGHT sensor only adds a right
 *  correction.  (The C+R = centred rule zig-zagged: the line rarely
 *  covered both sensors at once.)
 *    C black, R white  ->     0   straight on
 *    C black, R black  ->  -200   line drifting right: ease right
 *    C white, R black  ->  -600   line right: steer right
 *    neither           ->  *on_line = FALSE: line_task turns back toward
 *                                 the side it was last seen (centre
 *                                 alone last = line went LEFT), then
 *                                 goes straight
 *  IR_L (barcode sensor) is passed as 0 by line_task.
 *  > 0 means the line is to the left: steer left.
 * --------------------------------------------------------------------- */
#define ERR_EASE	200
#define ERR_TURN	600

EXPORT INT line_error(INT n_l, INT n_c, INT n_r, BOOL *on_line)
{
	BOOL	l = (n_l >= LINE_BLACK_N) ? TRUE : FALSE;
	BOOL	c = (n_c >= LINE_BLACK_N) ? TRUE : FALSE;
	BOOL	r = (n_r >= LINE_BLACK_N) ? TRUE : FALSE;

	*on_line = (l || c || r) ? TRUE : FALSE;
	if (c) {
		if (r) {
			return -ERR_EASE;
		}
		if (l) {
			return ERR_EASE;
		}
		return 0;
	}
	if (r && !l) {
		return -ERR_TURN;
	}
	if (l && !r) {
		return ERR_TURN;
	}
	return 0;
}

/* ---------------------------------------------------------------------
 *  Code 39 table: 9 elements B S B S B S B S B, bit 8 = first element,
 *  1 = wide.  Every pattern has exactly three wide elements.
 * --------------------------------------------------------------------- */
typedef struct {
	UH	pattern;
	UB	ch;
} C39_ENTRY;

LOCAL const C39_ENTRY c39_table[] = {
	{0x034, '0'}, {0x121, '1'}, {0x061, '2'}, {0x160, '3'},
	{0x031, '4'}, {0x130, '5'}, {0x070, '6'}, {0x025, '7'},
	{0x124, '8'}, {0x064, '9'}, {0x109, 'A'}, {0x049, 'B'},
	{0x148, 'C'}, {0x019, 'D'}, {0x118, 'E'}, {0x058, 'F'},
	{0x00D, 'G'}, {0x10C, 'H'}, {0x04C, 'I'}, {0x01C, 'J'},
	{0x103, 'K'}, {0x043, 'L'}, {0x142, 'M'}, {0x013, 'N'},
	{0x112, 'O'}, {0x052, 'P'}, {0x007, 'Q'}, {0x106, 'R'},
	{0x046, 'S'}, {0x016, 'T'}, {0x181, 'U'}, {0x0C1, 'V'},
	{0x1C0, 'W'}, {0x091, 'X'}, {0x190, 'Y'}, {0x0D0, 'Z'},
	{0x085, '-'}, {0x184, '.'}, {0x0C4, ' '}, {0x094, '*'},
	{0x0A8, '$'}, {0x0A2, '/'}, {0x08A, '+'}, {0x02A, '%'}
};
#define C39_TABLE_LEN	(sizeof(c39_table) / sizeof(c39_table[0]))

/* Classify 9 widths into a pattern.  Returns -1 if the 3 widest are not
   clearly wider than the other 6. */
LOCAL INT c39_pattern(const UW *w, INT step)
{
	UW	sorted[9];
	UW	t;
	UW	thr;
	INT	i, j;
	INT	pat = 0;

	for (i = 0; i < 9; i++) {
		sorted[i] = w[i * step];
	}
	for (i = 1; i < 9; i++) {		/* insertion sort, descending */
		t = sorted[i];
		for (j = i - 1; (j >= 0) && (sorted[j] < t); j--) {
			sorted[j + 1] = sorted[j];
		}
		sorted[j + 1] = t;
	}
	/* narrowest wide (sorted[2]) vs widest narrow (sorted[3]) */
	if ((sorted[2] * 10U) < (sorted[3] * CODE39_MIN_RATIO_X10)) {
		return -1;
	}
	thr = (sorted[2] + sorted[3]) / 2U;
	for (i = 0; i < 9; i++) {
		pat = (pat << 1) | ((w[i * step] > thr) ? 1 : 0);
	}
	return pat;
}

LOCAL INT c39_lookup(INT pat)
{
	UINT	i;

	for (i = 0; i < C39_TABLE_LEN; i++) {
		if ((INT)c39_table[i].pattern == pat) {
			return (INT)c39_table[i].ch;
		}
	}
	return -1;
}

/* Decode in one direction: dir = +1 forwards, -1 backwards */
LOCAL INT c39_try(const UW *widths, INT dir, UB *letter)
{
	INT	c[3];
	INT	k;
	INT	pat;
	const UW *base;

	for (k = 0; k < 3; k++) {
		base = (dir > 0) ? &widths[k * 10] : &widths[CODE39_ELEMENTS - 1 - (k * 10)];
		pat = c39_pattern(base, dir);
		if (pat < 0) {
			return C39_ERR_RATIO;
		}
		c[k] = c39_lookup(pat);
		if (c[k] < 0) {
			return C39_ERR_CHAR;
		}
	}
	if ((c[0] != '*') || (c[2] != '*')) {
		return C39_ERR_STARTSTOP;
	}
	*letter = (UB)c[1];
	return C39_OK;
}

EXPORT INT code39_decode(const UW *widths, INT n, UB *letter)
{
	INT	r;

	if (n != CODE39_ELEMENTS) {
		return C39_ERR_COUNT;
	}
	r = c39_try(widths, 1, letter);
	if (r != C39_OK) {
		INT r2 = c39_try(widths, -1, letter);	/* read backwards */
		if (r2 == C39_OK) {
			return C39_OK;
		}
	}
	return r;
}

EXPORT BARCODE_CMD line_letter_to_cmd(UB letter, UW map)
{
	BARCODE_CMD	c = app_barcode_lookup(map, letter);

	if (c != BC_UNRECOGNISED) {
		return c;	/* in the MQTT map: its movement wins */
	}
	if ((APP_END_MARKER != '\0') && (letter == (UB)APP_END_MARKER)) {
		return BC_END_OF_COURSE;
	}
	return BC_UNRECOGNISED;	/* reported, car continues straight */
}

/* ---------------------------------------------------------------------
 *  Barcode reader
 * --------------------------------------------------------------------- */
EXPORT void bc_reader_reset(BC_READER *r)
{
	r->state = BCS_WAIT_GAP;
	r->black = FALSE;
	r->run_um = 0;
	r->n = 0;
	r->max_um = 0;
	r->samples = 0;
	r->overflow = FALSE;
	r->last_n = 0;
	r->pos = 0;
	r->edge_pos = 0;
	r->prev_edge_pos = 0;
	r->start_pos = 0;
	r->quiet_pos = 0;
	r->can_undo = FALSE;
	r->bars = 0;
	r->recross = 0;
}

LOCAL BC_RESULT bc_finish(BC_READER *r, UB *letter, INT *reason)
{
	BC_RESULT	res = BCR_NONE;
	INT		d;

	r->last_n = r->overflow ? (BC_MAX_ELEMENTS + 1) : r->n;
	if (r->overflow) {
		*reason = C39_ERR_COUNT;	/* over-long read */
		res = BCR_REJECT;
	} else if (r->n == 1) {
		/* One stripe with white on both sides: a branch, not a bar */
		res = (r->widths[0] >= BC_JUNCTION_MIN_UM) ? BCR_JUNCTION : BCR_NONE;
	} else if (r->n > 1) {
		d = code39_decode(r->widths, r->n, letter);
		if (d == C39_OK) {
			res = BCR_BARCODE;
		} else {
			*reason = d;
			res = BCR_REJECT;
		}
	}
	r->state = BCS_ARMED;		/* the trailing white was a quiet zone */
	r->n = 0;
	r->max_um = 0;
	r->samples = 0;
	r->overflow = FALSE;
	return res;
}

LOCAL void bc_recompute(BC_READER *r)
{
	INT	i;
	W	sum = 0;

	r->max_um = 0;
	for (i = 0; i < r->n; i++) {
		sum += (W)r->widths[i];
		if (r->widths[i] > r->max_um) {
			r->max_um = r->widths[i];
		}
	}
	r->edge_pos = r->start_pos + sum;
}

/* The sensor came back over the edge it just crossed (or noise flipped
   the level without real movement): take that edge back.  TRUE if done. */
LOCAL BOOL bc_undo(BC_READER *r)
{
	if ((r->state == BCS_READING) && !r->overflow) {
		if (r->n > 0) {
			r->n--;			/* drop the element that edge ended */
			bc_recompute(r);
		} else {
			r->state = BCS_ARMED;	/* the first bar had not started */
			r->edge_pos = r->quiet_pos;
		}
	} else if (r->can_undo && (r->state != BCS_READING)) {
		r->edge_pos = r->prev_edge_pos;
		r->can_undo = FALSE;
	} else {
		return FALSE;
	}
	if (r->black && (r->bars > 0U)) {
		r->bars--;			/* the undone edge started a line */
	}
	r->recross++;
	return TRUE;
}

EXPORT BC_RESULT bc_reader_feed(BC_READER *r, INT n_r, W um,
				UB *letter, INT *reason)
{
	BOOL	level = r->black;
	UW	end_um;
	W	w;

	/* Threshold with hysteresis (VA2): edge noise cannot split a bar */
	if (r->black && (n_r < LINE_WHITE_N)) {
		level = FALSE;
	} else if (!r->black && (n_r > LINE_BLACK_N)) {
		level = TRUE;
	}
	r->pos += um;

	if (level != r->black) {
		if ((r->pos <= (r->edge_pos + BC_RECROSS_UM)) && bc_undo(r)) {
			/* debounced: back on the level before that edge */
		} else {
			/* An element just ended: its width is the distance the
			   sensor moved between its two edges */
			w = r->pos - r->edge_pos;
			if (w < 0) {
				w = 0;
			}
			if (r->state == BCS_READING) {
				if (r->n < BC_MAX_ELEMENTS) {
					r->widths[r->n++] = (UW)w;
					if ((UW)w > r->max_um) {
						r->max_um = (UW)w;
					}
				} else {
					r->overflow = TRUE;
				}
			} else if ((r->state == BCS_ARMED) && level) {
				r->state = BCS_READING;		/* first bar starts */
				r->n = 0;
				r->max_um = 0;
				r->samples = 0;
				r->start_pos = r->pos;
				r->quiet_pos = r->edge_pos;
			} else {
				/* WAIT_GAP: any bar restarts the quiet-zone count */
			}
			if (level) {
				r->bars++;
			}
			r->prev_edge_pos = r->edge_pos;
			r->edge_pos = r->pos;
			r->can_undo = TRUE;
		}
		r->black = level;
	}
	w = r->pos - r->edge_pos;
	r->run_um = (w > 0) ? (UW)w : 0U;

	switch (r->state) {
	case BCS_WAIT_GAP:
		if (!r->black && (r->run_um >= BC_QUIET_MIN_UM)) {
			r->state = BCS_ARMED;
		}
		break;
	case BCS_READING:
		r->samples++;
		end_um = r->max_um * 2U;
		if (end_um < BC_QUIET_MIN_UM) {
			end_um = BC_QUIET_MIN_UM;
		}
		if (!r->black && (r->run_um >= end_um)) {
			return bc_finish(r, letter, reason);
		}
		if ((r->samples > BC_MAX_SAMPLES) ||
		    ((r->pos - r->start_pos) > BC_MAX_READ_UM)) {
			UW	keep_bars = r->bars;
			UW	keep_rx = r->recross;
			W	keep_pos = r->pos;

			*reason = C39_ERR_COUNT;
			bc_reader_reset(r);
			r->bars = keep_bars;	/* the line count runs on */
			r->recross = keep_rx;
			r->pos = keep_pos;
			r->edge_pos = keep_pos;
			r->black = (level) ? TRUE : FALSE;
			return BCR_REJECT;
		}
		break;
	default:
		break;
	}
	return BCR_NONE;
}

/* ---------------------------------------------------------------------
 *  Encoder distance filter (see line_sense.h)
 * --------------------------------------------------------------------- */
EXPORT void enc_dist_init(ENC_DIST *e, W cnt_l, W cnt_r, UW now_ms)
{
	e->prev_l = cnt_l;
	e->prev_r = cnt_r;
	e->prev_ms = now_ms;
	e->rejects = 0;
}

EXPORT W enc_dist_step(ENC_DIST *e, W cnt_l, W cnt_r, UW now_ms,
		       W um_per_count, W side_mm, W track_mm)
{
	W	dl = (cnt_l - e->prev_l) * um_per_count;
	W	dr = (cnt_r - e->prev_r) * um_per_count;
	UW	dt = now_ms - e->prev_ms;
	W	limit;

	e->prev_l = cnt_l;
	e->prev_r = cnt_r;
	e->prev_ms = now_ms;
	if (dt > 1000U) {
		dt = 1000U;		/* long gap: cap the allowance */
	}
	limit = ((W)ENC_DIST_MAX_MM_S * (W)dt) + ENC_DIST_SLACK_UM;	/* mm/s x ms = µm */
	if ((APP_ABS(dl) > limit) || (APP_ABS(dr) > limit)) {
		e->rejects++;
		return 0;		/* impossible jump: bad reading, drop it */
	}
	return ((dl + dr) / 2) - (((dr - dl) * side_mm) / track_mm);
}

/* ---------------------------------------------------------------------
 *  Cross-junction detector (see line_sense.h)
 * --------------------------------------------------------------------- */
#define CROSS_FAR_UM	0x40000000UL	/* "not seen for a long way" */

EXPORT void cross_reset(CROSS_DET *d)
{
	d->state = CXS_IDLE;
	d->l_black = FALSE;
	d->run_um = 0;
	d->stripe_um = 0;
	d->since_c_um = CROSS_FAR_UM;
	d->since_r_um = CROSS_FAR_UM;
	d->c_seen = FALSE;
	d->r_seen = FALSE;
}

LOCAL UW cross_add(UW v, UW um)
{
	return (v < CROSS_FAR_UM) ? (v + um) : CROSS_FAR_UM;
}

EXPORT void cross_feed_cr(CROSS_DET *d, UINT ch, INT n, UW um)
{
	UW	*since = (ch == 1U) ? &d->since_c_um : &d->since_r_um;

	*since = (n >= LINE_BLACK_N) ? 0U : cross_add(*since, um);
}

EXPORT BOOL cross_feed_l(CROSS_DET *d, INT n_l, UW um, UW *back_um)
{
	BOOL	level = d->l_black;
	BOOL	found = FALSE;
	UW	need;

	/* same hysteresis as the barcode reader */
	if (d->l_black && (n_l < LINE_WHITE_N)) {
		level = FALSE;
	} else if (!d->l_black && (n_l > LINE_BLACK_N)) {
		level = TRUE;
	} else {
		/* no change */
	}

	if (level == d->l_black) {
		d->run_um = cross_add(d->run_um, um);
	} else {
		if (level) {
			/* white -> black: a stripe starts.  Only a candidate
			   after a quiet zone; a bar soon after a stripe means
			   a barcode, so the candidate is dropped. */
			if ((d->state == CXS_IDLE) && (d->run_um >= CROSS_QUIET_UM)) {
				d->state = CXS_STRIPE;
				d->c_seen = FALSE;
				d->r_seen = FALSE;
			} else {
				d->state = CXS_IDLE;
			}
		} else if (d->state == CXS_STRIPE) {
			/* black -> white: the stripe ended; check its width */
			d->stripe_um = d->run_um;
			d->state = ((d->run_um >= CROSS_MIN_UM) &&
				    (d->run_um <= CROSS_MAX_UM)) ? CXS_AFTER : CXS_IDLE;
		} else {
			d->state = CXS_IDLE;
		}
		d->l_black = level;
		d->run_um = um;
	}

	if ((d->state == CXS_STRIPE) || (d->state == CXS_AFTER)) {
		if (d->since_c_um <= CROSS_SLACK_UM) {
			d->c_seen = TRUE;
		}
		if (d->since_r_um <= CROSS_SLACK_UM) {
			d->r_seen = TRUE;
		}
	}
	if ((d->state == CXS_STRIPE) && (d->run_um > CROSS_MAX_UM)) {
		d->state = CXS_IDLE;		/* too long: not a crossing line */
	}
	if ((d->state == CXS_AFTER) && !d->l_black) {
		need = d->stripe_um * 2U;
		if (need < CROSS_QUIET_UM) {
			need = CROSS_QUIET_UM;
		}
		if (d->run_um >= need) {
			d->state = CXS_IDLE;
			found = (d->c_seen && d->r_seen) ? TRUE : FALSE;
			*back_um = d->run_um + (d->stripe_um / 2U);
		}
	}
	return found;
}
