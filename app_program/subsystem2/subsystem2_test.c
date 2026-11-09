/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2 (Motion).  Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.
 *
 *	S2-U01..U08 are logic tests and are safe on the bench.  The motors do
 *	not move in these tests: the hardware tests (hand-turn counts, PWM
 *	sweep, 1 m run, 90° turn) are manual, see TEST_LOG.md.
 */

#include "subsystem2.h"
#include "motion_ctrl.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem2_test(void)
{
	MOTION_PID	pid;
	MOTION_STALL	st;
	INT		i;
	BOOL		tripped;
	W		out;
	UW		w;

	TEST_INIT();

	/* S2-U01: encoders loaded at offset 0 of a PIO block (A2) */
	TEST_CHECK("S2-U01", motion_encoders_ok, "PIO quadrature program loaded at offset 0");

	/* S2-U02: distance <-> counts round trip within one count */
	out = motion_mm_to_counts(1000);
	TEST_CHECK("S2-U02",
		   APP_ABS(motion_counts_to_01mm(out) - 10000) <= (MOTION_UM_PER_COUNT / 100 + 1),
		   "1000 mm -> counts -> 0.1 mm round trip");

	/* S2-U03: a 90° pivot equals a quarter of the track circumference */
	out = motion_turn_counts(900);
	/* arc in µm = pi * track / 4 * 1000; allow 1 mm */
	TEST_CHECK("S2-U03",
		   APP_ABS((out * MOTION_UM_PER_COUNT) -
			   ((MOTION_TRACK_MM * 3141593 / 4000) * MOTION_TURN_SCALE_PCT / 100)) <= 1000,
		   "90 deg turn arc = pi * track / 4 x turn scale");

	/* S2-U04: integrator is clamped and cleared (VA2) */
	motion_pid_init(&pid, 256, 256, 0, 100, 10000);
	for (i = 0; i < 1000; i++) {
		(void)motion_pid_step(&pid, 50);
	}
	TEST_CHECK("S2-U04a", pid.i_acc == 100, "integrator clamped at its limit");
	motion_pid_reset(&pid);
	TEST_CHECK("S2-U04b", pid.i_acc == 0, "integrator cleared on reset");

	/* S2-U05: stall rule = < 20% of commanded for 300 ms (F11) */
	motion_stall_reset(&st);
	tripped = FALSE;
	for (i = 0; i < MOTION_STALL_TICKS - 1; i++) {
		tripped |= motion_stall_update(&st, 300, 50);	/* 17% */
	}
	TEST_CHECK("S2-U05a", !tripped, "no stall before 300 ms");
	TEST_CHECK("S2-U05b", motion_stall_update(&st, 300, 50), "stall at 300 ms below 20%");
	motion_stall_reset(&st);
	tripped = FALSE;
	for (i = 0; i < 50; i++) {
		tripped |= motion_stall_update(&st, 300, 70);	/* 23% */
	}
	TEST_CHECK("S2-U05c", !tripped, "no stall at 23% of commanded");
	motion_stall_reset(&st);
	tripped = FALSE;
	for (i = 0; i < 50; i++) {
		tripped |= motion_stall_update(&st, 20, 0);	/* below min cmd */
	}
	TEST_CHECK("S2-U05d", !tripped, "no stall when nothing is commanded");

	/* S2-U06: odometry speed word packs signed values losslessly */
	w = motion_pack_speed(-500, 499);
	TEST_CHECK("S2-U06",
		   ((INT)(w & 0xFFFFU) - 32768 == -500) && ((INT)(w >> 16) - 32768 == 499),
		   "speed word offset-binary round trip");

	/* S2-U07: soft start ramp never exceeds its step */
	TEST_CHECK("S2-U07", motion_ramp(0, 600, 60) == 60 && motion_ramp(590, 600, 60) == 600,
		   "ramp limited to one step per tick");

	/* S2-U08: motion_task release jitter <= 2 ms (NFR3), after 1 s */
	tk_dly_tsk(1000);
	TEST_CHECK("S2-U08", motion_jitter_max_us <= 2000U, "motion_task jitter <= 2 ms");
	tm_printf((UB*)"[TEST] S2 info: jitter max %d us, mode %d, path %d.%d mm\n",
		  motion_jitter_max_us, motion_mode_word,
		  motion_odo_path_01mm / 10U, motion_odo_path_01mm % 10U);

	TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
