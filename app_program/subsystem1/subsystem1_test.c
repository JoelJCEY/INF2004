/*
 *	subsystem1_test.c
 *	On-target tests for subsystem 1 (Communication).  make TEST=1
 *	Record results in testing/subsystem1/TEST_LOG.md.
 *
 *	S1-U01 uses the SAME golden vector as tools/is11_mqtt.py, so the Pico
 *	encoder and the laptop decoder are checked against one byte string
 *	(design 10.5 "decoder check").  STOP latency, reconnection and the
 *	10-minute jitter run (A1) are manual tests, see TEST_LOG.md.
 */

#include "subsystem1.h"
#include "../test_report.h"

#if APP_TEST

LOCAL const UB golden_tlm[WIRE_TLM_LEN] = {
	0x01, 0x07, 0x92, 0x92, 0x01, 0x10, 0x21, 0x41,
	0xD4, 0xFE, 0x2B, 0x01, 0xC0, 0x1D, 0xFE, 0xFF,
	0xF1, 0xFB, 0x09, 0x00, 0xD2, 0x04, 0x00, 0x00,
	0x18, 0xFC, 0xCD, 0x00, 0xB4, 0x00, 0xBE, 0x00,
	0x04, 0x03, 0x02, 0x01
};

EXPORT void subsystem1_test(void)
{
	TLM_SNAPSHOT	s = {0};
	UB		buf[WIRE_TLM_LEN];
	REMOTE_MSG	m;
	INT		i, len;
	BOOL		same = TRUE;
	const UB	stop_ok[3]  = { 0x01, 0x02, 0x10 };
	const UB	bad_ver[3]  = { 0x02, 0x02, 0x10 };
	const UB	reserved[3] = { 0x01, 0x00, 0x10 };
	const UB	bad_bits[3] = { 0x01, 0x42, 0x10 };

	TEST_INIT();

	/* S1-U01: telemetry encoder matches the golden vector exactly */
	/* LINE_FOLLOWING, TURN_LEFT, last START, line flags 0x9,
	   servo 90° (index 12), MQTT link up */
	s.status = 0x10019292UL;
	s.fault_mask = 0x21U;
	s.letter = 'A';
	s.speed_l_mm_s = -300;
	s.speed_r_mm_s = 299;
	s.enc_l = -123456;
	s.enc_r = 654321;
	s.path_mm = 1234U;
	s.line_err = -1000;
	s.hump_peak_01mm = 205U;
	s.obs_width_mm = 180U;
	s.obs_closest_mm = 190U;
	s.uptime_ms = 0x01020304UL;
	len = comms_encode_tlm(&s, 7U, buf);
	for (i = 0; i < WIRE_TLM_LEN; i++) {
		if (buf[i] != golden_tlm[i]) {
			same = FALSE;
		}
	}
	TEST_CHECK("S1-U01", (len == WIRE_TLM_LEN) && same, "telemetry golden vector");

	/* S1-U02: a valid STOP decodes; invalid frames are rejected */
	TEST_CHECK("S1-U02a", (comms_decode_cmd(stop_ok, 3, &m) == E_OK) &&
		   (m.cmd == RC_STOP) && (m.seq == 0x10U), "STOP command decodes");
	TEST_CHECK("S1-U02b", comms_decode_cmd(bad_ver, 3, &m) != E_OK, "wrong version rejected");
	TEST_CHECK("S1-U02c", comms_decode_cmd(reserved, 3, &m) != E_OK, "reserved code 0 rejected");
	TEST_CHECK("S1-U02d", comms_decode_cmd(bad_bits, 3, &m) != E_OK, "reserved bits rejected");
	TEST_CHECK("S1-U02e", comms_decode_cmd(stop_ok, 2, &m) != E_OK, "short frame rejected");

	/* S1-U04: barcode map frame (IF-15) */
	{
		const UB map_ok[7]   = { 0x01, 0x10, 0x05, 'F', 'Z', 'A', 'W' };
		const UB map_dup[7]  = { 0x01, 0x10, 0x05, 'F', 'F', 'A', 'W' };
		const UB map_low[7]  = { 0x01, 0x10, 0x05, 'f', 'Z', 'A', 'W' };
		const UB map_star[7] = { 0x01, 0x10, 0x05, '*', 'Z', 'A', 'W' };
		UW	map = 0;

		TEST_CHECK("S1-U04a", (comms_decode_map(map_ok, 7, &map) == E_OK) &&
			   (map == APP_BARCODE_MAP('F', 'Z', 'A', 'W')) &&
			   (app_barcode_lookup(map, 'Z') == BC_U_TURN),
			   "map frame decodes: forward F, u-turn Z, left A, right W");
		TEST_CHECK("S1-U04b", (comms_decode_map(map_dup, 7, &map) != E_OK) &&
			   (comms_decode_map(map_low, 7, &map) != E_OK) &&
			   (comms_decode_map(map_star, 7, &map) != E_OK) &&
			   (comms_decode_map(map_ok, 6, &map) != E_OK),
			   "map frame: repeated, lower-case, '*' or short rejected");
		TEST_CHECK("S1-U04c", (comms_decode_cmd(map_ok, 7, &m) != E_OK) &&
			   !comms_is_map_frame(stop_ok, 3),
			   "map frame and START/STOP frame never mixed up");
	}

	/* S1-U05: barcode test mode frame */
	{
		const UB t_on[4]  = { 0x01, 0x11, 0x07, 0x01 };
		const UB t_off[4] = { 0x01, 0x11, 0x07, 0x00 };
		const UB t_bad[4] = { 0x01, 0x11, 0x07, 0x02 };
		UW	on = 9;

		TEST_CHECK("S1-U05", (comms_decode_bc_test(t_on, 4, &on) == E_OK) && (on == 1U) &&
			   (comms_decode_bc_test(t_off, 4, &on) == E_OK) && (on == 0U) &&
			   (comms_decode_bc_test(t_bad, 4, &on) != E_OK) &&
			   (comms_decode_cmd(t_on, 4, &m) != E_OK) && !comms_is_map_frame(t_on, 4),
			   "barcode test on/off frame decodes, never mixed up with the others");
	}

	/* S1-U03: packed status fields round-trip through their masks */
	s.status = ((UW)VS_FAULT_STOP << TLM_STATE_SHIFT) |
		   ((UW)BC_U_TURN << TLM_BARCODE_SHIFT) |
		   (24UL << TLM_SERVO_SHIFT) | (1UL << TLM_LINK_SHIFT);
	TEST_CHECK("S1-U03", (TLM_FIELD(s.status, STATE) == VS_FAULT_STOP) &&
		   (TLM_FIELD(s.status, BARCODE) == BC_U_TURN) &&
		   (TLM_FIELD(s.status, SERVO) == 24U) && (TLM_BIT(s.status, LINK) == 1U),
		   "status word fields (state 4 bits, barcode 3, servo 5)");

	tm_printf((UB*)"[TEST] S1 info: link state %d, outbox dropped %d, rx rejected %d\n",
		  comms_state_word, comms_outbox_dropped, comms_rx_rejected);

	TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
