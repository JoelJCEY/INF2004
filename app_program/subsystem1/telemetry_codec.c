/*
 *	telemetry_codec.c
 *	Subsystem 1 -- MQTT wire format v1 (layout in subsystem1.h).
 *
 *	Explicit byte order (little-endian) with shifts and masks, never a
 *	raw struct copy, so the laptop decoder does not depend on how the
 *	compiler lays out TLM_SNAPSHOT (design 10.2, T2).
 */

#include "subsystem1.h"

LOCAL void put16(UB *p, UW v)
{
	p[0] = (UB)(v & 0xFFU);
	p[1] = (UB)((v >> 8) & 0xFFU);
}

LOCAL void put32(UB *p, UW v)
{
	p[0] = (UB)(v & 0xFFU);
	p[1] = (UB)((v >> 8) & 0xFFU);
	p[2] = (UB)((v >> 16) & 0xFFU);
	p[3] = (UB)((v >> 24) & 0xFFU);
}

EXPORT INT comms_encode_tlm(const TLM_SNAPSHOT *s, UB seq, UB *buf)
{
	buf[0] = (UB)WIRE_VERSION;
	buf[1] = seq;
	put32(&buf[2], s->status);
	buf[6] = s->fault_mask;
	buf[7] = s->letter;
	put16(&buf[8], (UW)(UH)s->speed_l_mm_s);	/* two's complement i16 */
	put16(&buf[10], (UW)(UH)s->speed_r_mm_s);
	put32(&buf[12], (UW)s->enc_l);
	put32(&buf[16], (UW)s->enc_r);
	put32(&buf[20], s->path_mm);
	put16(&buf[24], (UW)(UH)s->line_err);
	put16(&buf[26], s->hump_peak_01mm);
	put16(&buf[28], s->obs_width_mm);
	put16(&buf[30], s->obs_closest_mm);
	put32(&buf[32], s->uptime_ms);
	return WIRE_TLM_LEN;
}

EXPORT ER comms_decode_cmd(const UB *buf, INT len, REMOTE_MSG *out)
{
	if ((len != WIRE_CMD_LEN) || (buf[0] != (UB)WIRE_VERSION)) {
		return E_PAR;			/* wrong length or version */
	}
	if (((buf[1] & 0xFCU) != 0U) || ((buf[1] & 0x03U) == (UB)RC_NONE)) {
		return E_PAR;			/* reserved bits or code 0 */
	}
	out->cmd = (UB)(buf[1] & 0x03U);
	out->seq = buf[2];
	out->reserved = 0;
	return E_OK;
}

EXPORT ER comms_decode_bc_test(const UB *buf, INT len, UW *on)
{
	if ((len != WIRE_TEST_LEN) || (buf[0] != (UB)WIRE_VERSION) ||
	    (buf[1] != (UB)WIRE_OP_BC_TEST) || (buf[3] > 1U)) {
		return E_PAR;
	}
	*on = (UW)buf[3];
	return E_OK;
}

EXPORT BOOL comms_is_map_frame(const UB *buf, INT len)
{
	return ((len == WIRE_MAP_LEN) && (buf[0] == (UB)WIRE_VERSION) &&
		(buf[1] == (UB)WIRE_OP_MAP)) ? TRUE : FALSE;
}

LOCAL BOOL map_letter_ok(UB c)
{
	return (((c >= (UB)'A') && (c <= (UB)'Z')) ||
		((c >= (UB)'0') && (c <= (UB)'9'))) ? TRUE : FALSE;
}

EXPORT ER comms_decode_map(const UB *buf, INT len, UW *map)
{
	INT	i, j;

	if (!comms_is_map_frame(buf, len)) {
		return E_PAR;
	}
	for (i = 3; i < WIRE_MAP_LEN; i++) {
		if (!map_letter_ok(buf[i])) {
			return E_PAR;			/* not A-Z / 0-9 */
		}
		for (j = 3; j < i; j++) {
			if (buf[j] == buf[i]) {
				return E_PAR;		/* same letter twice */
			}
		}
	}
	*map = APP_BARCODE_MAP(buf[3], buf[4], buf[5], buf[6]);
	return E_OK;
}
