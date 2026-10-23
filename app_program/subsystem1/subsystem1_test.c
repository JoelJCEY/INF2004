/*
 *	subsystem1_test.c
 *	On-target tests for subsystem 1.  Built only with: make TEST=1
 *	S1-U01..U03 are pure logic: the packed layout and its golden vector.
 *	The same golden vector is checked by tools/is11_decode.py on the laptop.
 */

#include "subsystem1.h"
#include "../test_report.h"

#if APP_TEST

/* Golden snapshot and its expected bytes (generated with the layout in
 * subsystem1.h; tools/is11_decode.py --selftest decodes the same bytes). */
LOCAL const UB golden[S1_PACKED_BYTES] = {
	0x01, 0x32, 0x05, 0x33, 0x78, 0x22, 0xB5, 0xA9, 0x34, 0x01,
	0x8D, 0x04, 0x00, 0x90, 0x04, 0x80, 0xCA, 0x71, 0x65, 0x08,
	0x06, 0x01
};

LOCAL void make_golden(S1_TELEMETRY *p)
{
	p->t_ms = 0U;
	p->seq = 0x82U;
	p->state = 2;		/* LINE FOLLOWING */
	p->last_barcode = 3;	/* STRAIGHT */
	p->last_letter = 'A';
	p->fault_mask = 0x05U;	/* stall + IMU */
	p->line_flags = 0x03U;	/* L and C black */
	p->line_err = -125;
	p->speed_l_mm_s = 290;
	p->speed_r_mm_s = 310;
	p->distance_mm = 12340;
	p->enc_l = 4660;
	p->enc_r = 4672;
	p->hump_active = 0;
	p->hump_peak_mm = 42;
	p->obstacle_mm = 1990;	/* -> 199 cm */
	p->obstacle_width_mm = 210;
	p->bypass_side = 2;	/* RIGHT */
	p->motion_event = 1;	/* CRUISING */
	p->watchdog_reset = 0;
	p->wifi_up = 1;
}

EXPORT void subsystem1_test(void)
{
	S1_TELEMETRY	t;
	S1_TELEMETRY	u;
	UB		b[S1_PACKED_BYTES];
	INT		i;
	BOOL		same = TRUE;
	UW		before;

	TEST_INIT();

	make_golden(&t);
	(void)s1_pack(&t, b);
	for (i = 0; i < S1_PACKED_BYTES; i++) {
		if (b[i] != golden[i]) {
			same = FALSE;
		}
	}
	TEST_CHECK("S1-U01", same == TRUE, "packed bytes match the golden vector");
	if (same == FALSE) {
		tm_printf((UB *)"[TEST] packed:");
		for (i = 0; i < S1_PACKED_BYTES; i++) {
			tm_printf((UB *)" %02x", (INT)b[i]);
		}
		tm_printf((UB *)"\n");
	}
	s1_unpack(b, &u);
	TEST_CHECK("S1-U02", (u.line_err == t.line_err) && (u.speed_l_mm_s == 290) &&
		   (u.distance_mm == 12340) && (u.obstacle_mm == 1990) && (u.last_letter == 'A'),
		   "unpack(pack(x)) == x for signed and offset fields");
	t.line_err = 99999;	/* out of range must clamp, not wrap */
	t.speed_l_mm_s = -5000;
	(void)s1_pack(&t, b);
	s1_unpack(b, &u);
	TEST_CHECK("S1-U03", (u.line_err == 2047) && (u.speed_l_mm_s == -1024),
		   "out-of-range values clamp");

	before = s1_rx_count;
	tk_dly_tsk(2000);
	TEST_CHECK("S1-U04", ((s1_rx_count - before) >= 8U) && ((s1_rx_count - before) <= 12U),
		   "about 10 snapshots in 2 s (5 Hz)");

	TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
