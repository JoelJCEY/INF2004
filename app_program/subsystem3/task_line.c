/*
 *	task_line.c
 *	Subsystem 3 -- line_task (COMPLETE VERSION), woken every 20 ms.
 *
 *	Each cycle it walks through the samples the DMA wrote since the last
 *	cycle (about 60: 20 per sensor).  The channel of each sample comes
 *	from a counter that steps 0, 1, 2 and carries on across ring wraps
 *	(the ring length, 512, is not a multiple of 3), so no division is
 *	needed in the sampling path.
 *	  - IR_L and IR_C: averaged over the cycle -> line error.
 *	  - IR_R: every sample goes to the Code 39 reader.
 */

#include "subsystem3.h"
#include "drv_adc_dma.h"
#include "barcode.h"
#include "../subsystem2/subsystem2.h"	/* odometer */
#include "../subsystem5/subsystem5.h"	/* Navigation Command Queue */

#define S3_LOST_LEVEL		150	/* both line sensors below: lost */
#define S3_RING_SAFE		450U	/* process at most this many per cycle */

EXPORT volatile BOOL	s3_ready = FALSE;
EXPORT volatile UW	s3_samples = 0U;
EXPORT volatile UW	s3_overruns = 0U;
EXPORT volatile UH	s3_last_norm[S3_NUM_IR];
EXPORT volatile UB	s3_last_letter = 0U;
EXPORT volatile UW	s3_reads_ok = 0U;
EXPORT volatile UW	s3_reads_bad = 0U;

/* Latest-value slot.  Word = line error (low 16 bits, two's complement)
 * | flags (bits 16-23).  The writer stores the word, then the time. */
LOCAL volatile UW	slot_word = 0U;
LOCAL volatile UW	slot_time_ms = 0U;

LOCAL W		white_raw[S3_NUM_IR] = { S3_WHITE_L, S3_WHITE_C, S3_WHITE_R };
LOCAL W		black_raw[S3_NUM_IR] = { S3_BLACK_L, S3_BLACK_C, S3_BLACK_R };

EXPORT volatile CAL_REQUEST	s3_cal_request = CAL_IDLE;
EXPORT volatile W		s3_cal_result = 0;
EXPORT volatile UW		s3_junctions = 0U;

LOCAL UH	cal_min[S3_NUM_IR];
LOCAL UH	cal_max[S3_NUM_IR];
LOCAL BOOL	cal_running = FALSE;

/* Calibration (statechart 5: INITIALISING).  Raw values are recorded
 * while ctrl_task sweeps the car across the line. */
LOCAL void s3_cal_sample(UINT ch, UH raw)
{
	if (raw < cal_min[ch]) {
		cal_min[ch] = raw;
	}
	if (raw > cal_max[ch]) {
		cal_max[ch] = raw;
	}
}

LOCAL void s3_cal_step(void)
{
	INT	i;

	if ((s3_cal_request == CAL_RECORD) && (cal_running == FALSE)) {
		for (i = 0; i < S3_NUM_IR; i++) {
			cal_min[i] = 0x0FFFU;
			cal_max[i] = 0U;
		}
		cal_running = TRUE;
		s3_cal_result = 0;
	} else if ((s3_cal_request == CAL_FINISH) && (cal_running == TRUE)) {
		BOOL	ok = TRUE;
		cal_running = FALSE;
		/* IR_L and IR_C must both have seen black and white; IR_R
		 * keeps its fallback numbers if it never crossed the line. */
		for (i = 0; i < S3_NUM_IR; i++) {
			if (((W)cal_max[i] - (W)cal_min[i]) < S3_CAL_MIN_SPAN) {
				if (i != S3_IR_R) {
					ok = FALSE;
				}
			}
		}
		if (ok == TRUE) {
			for (i = 0; i < S3_NUM_IR; i++) {
				if (((W)cal_max[i] - (W)cal_min[i]) >= S3_CAL_MIN_SPAN) {
					/* modules read HIGH on black (check S3_BLACK_x) */
					if (S3_BLACK_L > S3_WHITE_L) {
						white_raw[i] = cal_min[i];
						black_raw[i] = cal_max[i];
					} else {
						white_raw[i] = cal_max[i];
						black_raw[i] = cal_min[i];
					}
				}
			}
			tm_printf((UB *)"[S3] calibrated: L %d-%d  C %d-%d  R %d-%d\n",
				  (INT)white_raw[0], (INT)black_raw[0], (INT)white_raw[1],
				  (INT)black_raw[1], (INT)white_raw[2], (INT)black_raw[2]);
		} else {
			tm_printf((UB *)"[S3] calibration failed (car not over the line?): keeping defaults\n");
		}
		s3_cal_result = (ok == TRUE) ? 1 : -1;
	} else if ((s3_cal_request == CAL_IDLE) && (cal_running == TRUE)) {
		cal_running = FALSE;	/* sweep cancelled (STOP pressed) */
	} else {
		;
	}
}

EXPORT void s3_line_get(W *p_err, UW *p_age_ms, UW *p_flags)
{
	UW	t1;
	UW	t2;
	UW	w;

	do {
		t1 = slot_time_ms;
		w = slot_word;
		t2 = slot_time_ms;
	} while (t1 != t2);

	*p_err = (W)(H)(UH)(w & 0xFFFFU);
	*p_flags = (w >> 16) & 0xFFU;
	*p_age_ms = s5_now_ms() - t1;
}

LOCAL void s3_publish(W err, UW flags)
{
	slot_word = ((UW)(UH)(H)err) | ((flags & 0xFFU) << 16);
	slot_time_ms = s5_now_ms();
}

/* raw 12-bit -> 0 (white) .. 1000 (black), works either polarity */
LOCAL W s3_norm(INT ch, UH raw)
{
	W	span = black_raw[ch] - white_raw[ch];
	W	n;

	if (span == 0) {
		return 0;
	}
	n = (((W)raw - white_raw[ch]) * 1000) / span;
	if (n < 0) {
		n = 0;
	} else if (n > 1000) {
		n = 1000;
	} else {
		;
	}
	return n;
}

LOCAL void s3_post_nav(NAV_KIND kind, BARCODE_CMD cmd, UB letter)
{
	NAV_MSG	msg;

	msg.kind = kind;
	msg.cmd = cmd;
	msg.letter = letter;
	msg.distance_mm = s2_distance_mm;
	(void)s5_post(mbf_nav, &msg, (INT)sizeof(msg), FLG_NAV);
}

EXPORT void line_task(INT stacd, void *exinf)
{
	UW	rd;
	UINT	ch = 0U;		/* channel of sample rd: 0, 1, 2, 0 ... */
	BOOL	r_black = FALSE;	/* IR_R state with hysteresis */
	W	last_sign = 1;
	BOOL	lost = FALSE;
	BOOL	lost_reported = FALSE;
	W	lost_at_mm = 0;
	UB	last_letter = 0U;
	W	last_letter_mm = -100000;
	UINT	print_div = 0U;
	ER	er;
	W	r_black_from_mm = 0;
	W	r_white_from_mm = 0;

	(void)stacd;
	(void)exinf;

	er = adcdma_init();
	if (er < E_OK) {
		tm_printf((UB *)"[S3] ERROR: no free DMA channel (%d)\n", (INT)er);
		(void)tk_slp_tsk(TMO_FEVR);
	}
	tm_printf((UB *)"[S3] ADC round-robin + DMA running on DMA channel %d\n",
		  (INT)adcdma_channel());
	barcode_reset();
	rd = 0U;
	s3_publish(0, 0U);
	s3_ready = TRUE;

	while (1) {
		UW	wr;
		W	sum[2] = { 0, 0 };
		W	cnt[2] = { 0, 0 };
		W	n_l;
		W	n_c;
		W	err;
		UW	flags = 0U;

		(void)tk_slp_tsk(TMO_FEVR);
		(void)tk_can_wup(TSK_SELF);

		s3_cal_step();
		wr = adcdma_written();
		if ((wr - rd) > S3_RING_SAFE) {
			/* we fell behind: skip ahead by a multiple of 3 so the
			 * channel counter stays right */
			UW	skip = (wr - rd) - S3_RING_SAFE;
			skip += (3U - (skip % 3U)) % 3U;
			rd += skip;
			s3_overruns++;
		}
		if (adcdma_check_errors() == TRUE) {
			s3_overruns++;
		}

		while (rd != wr) {
			UH	raw = adcdma_sample(rd);
			W	n = s3_norm((INT)ch, raw);

			s3_last_norm[ch] = (UH)n;
			if (ch == (UINT)S3_IR_R) {
				BC_RESULT	res;
				UB		letter = 0U;

				if (cal_running == TRUE) {
					s3_cal_sample(ch, raw);
				}
				if ((r_black == FALSE) && (n > S3_BAR_ON)) {
					r_black = TRUE;
					r_black_from_mm = s2_distance_mm;
				} else if ((r_black == TRUE) && (n < S3_BAR_OFF)) {
					W	len = s2_distance_mm - r_black_from_mm;
					W	gap = r_black_from_mm - r_white_from_mm;
					r_black = FALSE;
					r_white_from_mm = s2_distance_mm;
					/* wide isolated stripe = right junction */
					if ((len >= S3_JUNCTION_MIN_MM) && (gap >= S3_JUNCTION_MIN_MM)) {
						s3_junctions++;
						s3_post_nav(NAV_JUNCTION_RIGHT, BC_NONE, 0U);
					}
				} else {
					;
				}
				res = barcode_feed(r_black, &letter);
				if (res == BCR_DECODED) {
					W	d = s2_distance_mm;
					s3_reads_ok++;
					if ((letter != last_letter) ||
					    ((d - last_letter_mm) > S3_REPEAT_MM)) {
						s3_last_letter = letter;
						s3_post_nav(NAV_BARCODE, barcode_command(letter), letter);
						tm_printf((UB *)"[S3] barcode '%c'\n", (INT)letter);
					}
					last_letter = letter;
					last_letter_mm = d;
				} else if (res == BCR_REJECTED) {
					s3_reads_bad++;
				} else {
					;
				}
			} else {
				if (cal_running == TRUE) {
					s3_cal_sample(ch, raw);
				}
				sum[ch] += n;
				cnt[ch]++;
			}
			rd++;
			s3_samples++;
			ch++;
			if (ch == (UINT)S3_NUM_IR) {
				ch = 0U;
			}
		}

		n_l = (cnt[S3_IR_L] > 0) ? (sum[S3_IR_L] / cnt[S3_IR_L]) : 0;
		n_c = (cnt[S3_IR_C] > 0) ? (sum[S3_IR_C] / cnt[S3_IR_C]) : 0;
		if (n_l > S3_LINE_BLACK) {
			flags |= S3_LF_L_BLACK;
		}
		if (n_c > S3_LINE_BLACK) {
			flags |= S3_LF_C_BLACK;
		}
		if (r_black == TRUE) {
			flags |= S3_LF_R_BLACK;
		}

		if ((n_l < S3_LOST_LEVEL) && (n_c < S3_LOST_LEVEL)) {
			/* both white: keep the last known side, saturated */
			err = last_sign * S3_ERR_MAX;
			flags |= S3_LF_LOST;
			if (lost == FALSE) {
				lost = TRUE;
				lost_at_mm = s2_distance_mm;
			} else if ((s2_distance_mm - lost_at_mm) > S3_LOST_MM) {
				flags |= S3_LF_LINE_LOST;
				if (lost_reported == FALSE) {
					lost_reported = TRUE;
					s3_post_nav(NAV_LINE_LOST, BC_NONE, 0U);
				}
			} else {
				;
			}
		} else {
			W	sign = (n_l >= S3_LINE_BLACK) ? 1 : -1;

			err = (n_l - S3_LINE_BLACK) + (sign * (1000 - n_c));
			if (err > S3_ERR_MAX) {
				err = S3_ERR_MAX;
			} else if (err < -S3_ERR_MAX) {
				err = -S3_ERR_MAX;
			} else {
				;
			}
			if (err != 0) {
				last_sign = (err > 0) ? 1 : -1;
			}
			if (lost_reported == TRUE) {
				s3_post_nav(NAV_LINE_FOUND, BC_NONE, 0U);
			}
			lost = FALSE;
			lost_reported = FALSE;
		}
		s3_publish(err, flags);

		if (S3_PRINT_ENABLE != 0) {
			if (++print_div >= 25U) {
				print_div = 0U;
				tm_printf((UB *)"[S3] L=%4d C=%4d R=%4d err=%5d flags=0x%02x ok=%d bad=%d ovr=%d\n",
					  (INT)n_l, (INT)n_c, (INT)s3_last_norm[S3_IR_R], (INT)err,
					  (INT)flags, (INT)s3_reads_ok, (INT)s3_reads_bad, (INT)s3_overruns);
			}
		}
	}
}
