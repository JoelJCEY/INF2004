/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3 (Line & Barcode).  make TEST=1
 *	Record results in testing/subsystem3/TEST_LOG.md.
 *
 *	S3-U01..U07 feed synthetic data to the pure logic in line_sense.c.
 *	Barcode passes on the real mat (A3: 20 passes per barcode, both
 *	directions, >= 95%) are manual tests, see TEST_LOG.md.
 */

#include "subsystem3.h"
#include "line_sense.h"
#include "drv_ir_adc.h"
#include "../test_report.h"

#if APP_TEST

#define NARROW_UM	3000U
#define WIDE_UM		9000U

/* Build the 29 widths of "*<c>*" from 9-bit patterns (bit 8 first) */
LOCAL void build_symbol(UH pat_mid, UW *w)
{
	const UH pats[3] = { 0x094, pat_mid, 0x094 };	/* '*' X '*' */
	INT	k, i, n = 0;

	for (k = 0; k < 3; k++) {
		for (i = 8; i >= 0; i--) {
			w[n++] = ((pats[k] >> i) & 1U) ? WIDE_UM : NARROW_UM;
		}
		if (k < 2) {
			w[n++] = NARROW_UM;	/* inter-character gap */
		}
	}
}

/* Feed a run of one level to the reader, 1 sample = um µm */
LOCAL BC_RESULT feed_run(BC_READER *r, BOOL black, UW len_um, UW um,
			 UB *letter, INT *reason)
{
	BC_RESULT	res = BCR_NONE;
	BC_RESULT	x;
	UW		d;

	for (d = 0; d < len_um; d += um) {
		x = bc_reader_feed(r, black ? 1000 : 0, um, letter, reason);
		if (x != BCR_NONE) {
			res = x;
		}
	}
	return res;
}

/* Drive the cross detector over: white 30 mm, then `bars` stripes of
   width_um under IR_L (white gaps of width_um between them), then white
   60 mm.  C and R are black (or white) the whole time.  300 µm/sample. */
LOCAL UW cross_back_um;

LOCAL INT cross_run(BOOL c_black, BOOL r_black, UW width_um, UINT bars)
{
	CROSS_DET	d;
	INT		found = 0;
	UINT		b;
	UW		x, back = 0;

	cross_reset(&d);
	for (b = 0; b <= (bars * 2U); b++) {
		BOOL	black = ((b % 2U) == 1U) ? TRUE : FALSE;
		UW	len = (b == 0U) ? 30000U : width_um;

		for (x = 0; x < len; x += 300U) {
			cross_feed_cr(&d, 1U, c_black ? 1000 : 0, 300U);
			cross_feed_cr(&d, 2U, r_black ? 1000 : 0, 300U);
			if (cross_feed_l(&d, black ? 1000 : 0, 300U, &back)) {
				found++;
				cross_back_um = back;
			}
		}
	}
	for (x = 0; x < 60000U; x += 300U) {
		cross_feed_cr(&d, 1U, c_black ? 1000 : 0, 300U);
		cross_feed_cr(&d, 2U, r_black ? 1000 : 0, 300U);
		if (cross_feed_l(&d, 0, 300U, &back)) {
			found++;
			cross_back_um = back;
		}
	}
	return found;
}

/* S3-U11 helper: '*A*' laid out on the floor (edge positions in µm from a
   20 mm quiet zone), and a sensor path that goes forward 300 µm a sample
   but every `every_um` swings BACK `back_um` and comes forward again (the
   car correcting toward the line).  Returns the reader result. */
LOCAL BC_RESULT wiggle_read(UW every_um, UW back_um, UB *letter, INT *bars)
{
	BC_READER	r;
	UW		w[CODE39_ELEMENTS];
	W		edge[CODE39_ELEMENTS + 1];
	W		pos = 0, end, mark = 0;
	BC_RESULT	res = BCR_NONE, x;
	INT		i, reason = 0;
	UW		k;

	build_symbol(0x109, w);			/* 'A' */
	edge[0] = 20000;
	for (i = 0; i < CODE39_ELEMENTS; i++) {
		edge[i + 1] = edge[i] + (W)w[i];
	}
	end = edge[CODE39_ELEMENTS] + 40000;
	bc_reader_reset(&r);
	while (pos < end) {
		W	step = 300;
		BOOL	black = FALSE;

		if ((every_um != 0U) && ((pos - mark) >= (W)every_um)) {
			for (k = 0; k < (back_um / 300U); k++) {	/* swing back */
				pos -= 300;
				black = FALSE;
				for (i = 0; i < CODE39_ELEMENTS; i += 2) {
					if ((pos >= edge[i]) && (pos < edge[i + 1])) black = TRUE;
				}
				x = bc_reader_feed(&r, black ? 1000 : 0, -300, letter, &reason);
				if (x != BCR_NONE) res = x;
			}
			mark = pos + (W)back_um;
		}
		pos += step;
		black = FALSE;
		for (i = 0; i < CODE39_ELEMENTS; i += 2) {
			if ((pos >= edge[i]) && (pos < edge[i + 1])) black = TRUE;
		}
		x = bc_reader_feed(&r, black ? 1000 : 0, step, letter, &reason);
		if (x != BCR_NONE) res = x;
	}
	*bars = (INT)r.bars;
	return res;
}

EXPORT void subsystem3_test(void)
{
	UW		w[CODE39_ELEMENTS];
	UW		rev[CODE39_ELEMENTS];
	UB		letter = 0;
	INT		i, reason = 0;
	BOOL		on_line;
	BC_READER	r;
	BC_RESULT	res;
	UW		wr1, wr2;
	UW		map;

	TEST_INIT();

	/* S3-U01: '*A*' decodes forwards */
	build_symbol(0x109, w);
	TEST_CHECK("S3-U01", (code39_decode(w, CODE39_ELEMENTS, &letter) == C39_OK)
		   && (letter == 'A'), "Code 39 '*A*' forwards");

	/* S3-U02: the same symbol read backwards (U-turn, reverse pass) */
	for (i = 0; i < CODE39_ELEMENTS; i++) {
		rev[i] = w[CODE39_ELEMENTS - 1 - i];
	}
	letter = 0;
	TEST_CHECK("S3-U02", (code39_decode(rev, CODE39_ELEMENTS, &letter) == C39_OK)
		   && (letter == 'A'), "Code 39 '*A*' backwards");

	/* S3-U03: a partial read (28 elements) is rejected (VA2) */
	TEST_CHECK("S3-U03", code39_decode(w, 28, &letter) == C39_ERR_COUNT,
		   "28-element read rejected");

	/* S3-U04: letters map to commands through the MQTT map (IF-15), not
	   a fixed table; a mapped 'Z' is a movement, not the end marker */
	map = APP_BARCODE_MAP('F', 'Z', 'A', 'W');
	TEST_CHECK("S3-U04a", (line_letter_to_cmd('F', map) == BC_GO_STRAIGHT) &&
		   (line_letter_to_cmd('Z', map) == BC_U_TURN) &&
		   (line_letter_to_cmd('A', map) == BC_TURN_LEFT) &&
		   (line_letter_to_cmd('W', map) == BC_TURN_RIGHT) &&
		   (line_letter_to_cmd('X', map) == BC_UNRECOGNISED),
		   "letter-to-movement from the MQTT map (FR2)");
	TEST_CHECK("S3-U04b", (line_letter_to_cmd('A', 0U) == BC_UNRECOGNISED) &&
		   (line_letter_to_cmd('Z', 0U) == BC_END_OF_COURSE),
		   "no map: nothing hard-coded, only the end marker");

	/* S3-U09: every letter A-Z decodes forwards AND backwards */
	{
		static const UH pat_az[26] = {
			0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00D, 0x10C,
			0x04C, 0x01C, 0x103, 0x043, 0x142, 0x013, 0x112, 0x052,
			0x007, 0x106, 0x046, 0x016, 0x181, 0x0C1, 0x1C0, 0x091,
			0x190, 0x0D0 };
		INT	k, ok = 0;
		UB	l1, l2;

		for (k = 0; k < 26; k++) {
			build_symbol(pat_az[k], w);
			for (i = 0; i < CODE39_ELEMENTS; i++) {
				rev[i] = w[CODE39_ELEMENTS - 1 - i];
			}
			l1 = 0;
			l2 = 0;
			if ((code39_decode(w, CODE39_ELEMENTS, &l1) == C39_OK) &&
			    (code39_decode(rev, CODE39_ELEMENTS, &l2) == C39_OK) &&
			    (l1 == (UB)('A' + k)) && (l2 == (UB)('A' + k))) {
				ok++;
			}
		}
		TEST_CHECK("S3-U09", ok == 26, "A-Z: each letter decodes the same both ways");
	}

	/* S3-U05: whole reader at 300 mm/s (300 µm per sample): quiet zone,
	   '*Z*', quiet zone -> one barcode, no reject */
	bc_reader_reset(&r);
	build_symbol(0x0D0, w);
	res = feed_run(&r, FALSE, 20000U, 300U, &letter, &reason);
	for (i = 0; i < CODE39_ELEMENTS; i++) {
		BC_RESULT x = feed_run(&r, (i % 2) == 0, w[i], 300U, &letter, &reason);
		if (x != BCR_NONE) res = x;
	}
	letter = 0;
	res = feed_run(&r, FALSE, 30000U, 300U, &letter, &reason);
	TEST_CHECK("S3-U05", (res == BCR_BARCODE) && (letter == 'Z') && (r.last_n == 29),
		   "reader decodes '*Z*' from 1 kHz samples, 29 elements counted");

	/* S3-U06: an 18 mm stripe with white both sides is a junction,
	   not a wide bar (F3, A3) */
	bc_reader_reset(&r);
	(void)feed_run(&r, FALSE, 20000U, 300U, &letter, &reason);
	(void)feed_run(&r, TRUE, 18000U, 300U, &letter, &reason);
	res = feed_run(&r, FALSE, 40000U, 300U, &letter, &reason);
	TEST_CHECK("S3-U06", res == BCR_JUNCTION, "18 mm single stripe -> junction");

	/* S3-U07: line error sign convention (IF-08) */
	TEST_CHECK("S3-U07a", line_error(900, 200, 0, &on_line) > 0 && on_line,
		   "IR_L black -> error > 0 (line to the left)");
	TEST_CHECK("S3-U07b", line_error(0, 0, 1000, &on_line) < 0 && on_line,
		   "only IR_R black -> error < 0 (steer right)");
	TEST_CHECK("S3-U07d", line_error(0, 1000, 0, &on_line) == 0 && on_line,
		   "only IR_C black -> error 0 (straight)");
	TEST_CHECK("S3-U07f", line_error(0, 1000, 1000, &on_line) < 0 && on_line,
		   "IR_C + IR_R black -> gentle right");
	(void)line_error(0, 300, 100, &on_line);
	TEST_CHECK("S3-U07e", !on_line, "all below black -> not on the line (keep last side)");
	TEST_CHECK("S3-U07c", line_unpack_err(line_pack(-1000, LINE_F_VALID, 1234)) == -1000,
		   "line word packs -1000 losslessly");

	/* S3-U10: cross-junction detector (IF-09 kind 6) */
	TEST_CHECK("S3-U10a", cross_run(TRUE, TRUE, 18000U, 1U) == 1,
		   "18 mm line under L, C and R -> one cross-junction");
	TEST_CHECK("S3-U10b", cross_run(FALSE, TRUE, 18000U, 1U) == 0,
		   "stripe under L and R but C white -> not a cross");
	TEST_CHECK("S3-U10c", cross_run(TRUE, FALSE, 18000U, 1U) == 0,
		   "left-only branch (R white) -> not a cross");
	TEST_CHECK("S3-U10d", cross_run(TRUE, TRUE, 9000U, 15U) == 0,
		   "barcode-like bars (several, close together) -> not a cross");
	TEST_CHECK("S3-U10e", cross_run(TRUE, TRUE, 120000U, 1U) == 0,
		   "120 mm black under L (main line) -> not a cross");
	TEST_CHECK("S3-U10f", cross_back_um >= 18000U,
		   "reported middle is behind the sensor row");

	/* S3-U11: debounce.  The sensor swings back 1.5 mm every 4 mm (and
	   2.4 mm every 7 mm) while crossing '*A*': each re-crossed line is
	   taken back, so it still decodes and counts 15 lines, not 16-17 */
	{
		INT	bars = 0;

		letter = 0;
		res = wiggle_read(0U, 0U, &letter, &bars);
		TEST_CHECK("S3-U11a", (res == BCR_BARCODE) && (letter == 'A') && (bars == 15),
			   "straight pass: 'A', 15 lines");
		letter = 0;
		res = wiggle_read(4000U, 1500U, &letter, &bars);
		TEST_CHECK("S3-U11b", (res == BCR_BARCODE) && (letter == 'A') && (bars == 15),
			   "swinging back 1.5 mm every 4 mm: still 'A', 15 lines");
		letter = 0;
		res = wiggle_read(7000U, 2400U, &letter, &bars);
		TEST_CHECK("S3-U11c", (res == BCR_BARCODE) && (letter == 'A') && (bars == 15),
			   "swinging back 2.4 mm every 7 mm: still 'A', 15 lines");
	}
	/* S3-U12: a 0.3 mm black speck in the quiet zone is noise, not a line */
	bc_reader_reset(&r);
	(void)feed_run(&r, FALSE, 6000U, 300U, &letter, &reason);
	(void)feed_run(&r, TRUE, 300U, 300U, &letter, &reason);
	(void)feed_run(&r, FALSE, 20000U, 300U, &letter, &reason);
	TEST_CHECK("S3-U12", (r.bars == 0U) && (r.state == BCS_ARMED),
		   "0.3 mm speck ignored, quiet zone still counts");

	/* S3-U13: encoder distance filter (74 µm per count, sensor 25 mm left
	   of centre, track 117 mm) */
	{
		ENC_DIST	e;
		W		d1, d2, d3, d4;

		enc_dist_init(&e, 1000, 2000, 0U);
		d1 = enc_dist_step(&e, 1016, 2016, 20U, 74, 25, 117);	/* straight, ~1.2 mm */
		d2 = enc_dist_step(&e, 1020, 2036, 40U, 74, 25, 117);	/* steering left   */
		d3 = enc_dist_step(&e, 1010, 2046, 60U, 74, 25, 117);	/* pivot left: IR_L goes back */
		d4 = enc_dist_step(&e, 9000, 2050, 80U, 74, 25, 117);	/* 590 mm in 20 ms: bad read  */
		TEST_CHECK("S3-U13", (d1 == 16 * 74) && (d2 > 0) && (d2 < (18 * 74)) &&
			   (d3 < 0) && (d4 == 0) && (e.rejects == 1U),
			   "encoder distance: exact travel, slower/backwards IR_L when steering, jump dropped");
	}

	/* S3-U08: DMA ring keeps advancing (hardware sampling alive) */
	wr1 = drv_ir_adc_write_index();
	tk_dly_tsk(30);
	wr2 = drv_ir_adc_write_index();
	TEST_CHECK("S3-U08", wr1 != wr2, "DMA ring write index advances");

	tm_printf((UB*)"[TEST] S3 info: line word 0x%08x, reads %d ok / %d rejected\n",
		  line_latest, line_read_stats & 0xFFFFU, line_read_stats >> 16);

	TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */
