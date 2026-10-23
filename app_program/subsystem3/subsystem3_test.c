/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3.  Built only with: make TEST=1
 *	S3-U01..U03 are pure logic (Code 39 decoder) and need no hardware.
 */

#include "subsystem3.h"
#include "barcode.h"
#include "../test_report.h"

#if APP_TEST

/* Build the 29 element widths of "*<ch>*" with narrow = n, wide = 3n. */
LOCAL void t_make(UB ch, UH n, UH out[BC_ELEMENTS])
{
	/* patterns of '*' and a few letters (first element = MSB) */
	UH	star = 0x094U;
	UH	mid = (ch == 'A') ? 0x109U : ((ch == 'Z') ? 0x0D0U : 0x091U); /* else 'X' */
	UH	pats[3];
	UINT	c;
	UINT	k;
	UINT	i = 0U;

	pats[0] = star;
	pats[1] = mid;
	pats[2] = star;
	for (c = 0U; c < 3U; c++) {
		for (k = 0U; k < 9U; k++) {
			out[i++] = ((pats[c] & (UH)(1U << (8U - k))) != 0U) ? (UH)(3U * n) : n;
		}
		if (c < 2U) {
			out[i++] = n;	/* inter-character gap */
		}
	}
}

EXPORT void subsystem3_test(void)
{
	UH	w[BC_ELEMENTS];
	UH	r[BC_ELEMENTS];
	UINT	i;
	UW	before;

	TEST_INIT();

	t_make('A', 10U, w);
	TEST_CHECK("S3-U01", barcode_decode29(w) == 'A', "decode *A* forwards");
	for (i = 0U; i < (UINT)BC_ELEMENTS; i++) {
		r[i] = w[(UINT)BC_ELEMENTS - 1U - i];
	}
	TEST_CHECK("S3-U02", barcode_decode29(r) == 'A', "decode *A* backwards");
	t_make('Z', 4U, w);
	w[5] = (UH)(w[5] + 1U);		/* a little width noise */
	TEST_CHECK("S3-U03", (barcode_decode29(w) == 'Z') &&
		   (barcode_command('A') == BC_TURN_LEFT) &&
		   (barcode_command('Z') == BC_UNRECOGNISED), "decode *Z* + command map");

	tk_dly_tsk(200);
	TEST_CHECK("S3-U04", s3_ready == TRUE, "ADC + DMA started");
	before = s3_samples;
	tk_dly_tsk(1000);
	TEST_CHECK("S3-U05", ((s3_samples - before) > 2700U) && ((s3_samples - before) < 3300U),
		   "3,000 samples per second (1 kHz per channel)");
	TEST_CHECK("S3-U06", s3_overruns == 0U, "no ring overrun or ADC error");

	TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */
