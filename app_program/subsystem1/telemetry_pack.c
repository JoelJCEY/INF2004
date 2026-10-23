/*
 *	telemetry_pack.c
 *	Subsystem 1 -- packed telemetry v1 (layout in subsystem1.h).
 *
 *	Explicit masks and shifts into a byte array, least-significant bit
 *	first: the result does not depend on how the compiler lays out a
 *	struct (no C bit-fields are sent; Barr-C 5.3a / 5.5b).  Signed fields
 *	use offset binary.  Out-of-range values are clamped, never wrapped
 *	(except the encoder counters, which are sent as their low 24 bits
 *	and are expected to wrap).
 */

#include "subsystem1.h"

typedef struct {
	UB	*p;
	UINT	bit;
} BITW;

LOCAL void put(BITW *p_w, UW value, UINT nbits)
{
	UINT	i;

	for (i = 0U; i < nbits; i++) {
		UINT	byte = p_w->bit >> 3;
		UINT	off = p_w->bit & 7U;

		if (((value >> i) & 1U) != 0U) {
			p_w->p[byte] |= (UB)(1U << off);
		}
		p_w->bit++;
	}
}

LOCAL UW get(const UB *p, UINT *p_bit, UINT nbits)
{
	UW	v = 0U;
	UINT	i;

	for (i = 0U; i < nbits; i++) {
		UINT	byte = *p_bit >> 3;
		UINT	off = *p_bit & 7U;

		if (((p[byte] >> off) & 1U) != 0U) {
			v |= (1UL << i);
		}
		(*p_bit)++;
	}
	return v;
}

LOCAL UW clamp_u(W v, W max)
{
	if (v < 0) {
		return 0U;
	}
	return (UW)((v > max) ? max : v);
}

/* offset binary: value + offset, clamped to 0 .. 2^nbits - 1 */
LOCAL UW offs(W v, W offset, W max)
{
	return clamp_u(v + offset, max);
}

EXPORT INT s1_pack(const S1_TELEMETRY *p_t, UB out[S1_PACKED_BYTES])
{
	BITW	w;
	UINT	i;
	W	obst_cm;

	for (i = 0U; i < (UINT)S1_PACKED_BYTES; i++) {
		out[i] = 0U;
	}
	w.p = out;
	w.bit = 0U;
	obst_cm = (p_t->obstacle_mm < 0) ? 255 : (W)clamp_u(p_t->obstacle_mm / 10, 254);

	put(&w, S1_PACKED_VERSION, 8U);
	put(&w, clamp_u(p_t->state, 15), 4U);
	put(&w, clamp_u(p_t->last_barcode, 7), 3U);
	put(&w, (p_t->hump_active != 0) ? 1U : 0U, 1U);
	put(&w, p_t->fault_mask & 0xFFU, 8U);
	put(&w, p_t->line_flags & 0x07U, 3U);
	put(&w, ((p_t->line_flags & 0x10U) != 0U) ? 1U : 0U, 1U);	/* LINE LOST */
	put(&w, offs(p_t->line_err, 2048, 4095), 12U);
	put(&w, offs(p_t->speed_l_mm_s, 1024, 2047), 11U);
	put(&w, offs(p_t->speed_r_mm_s, 1024, 2047), 11U);
	put(&w, clamp_u(p_t->distance_mm / 10, 65535), 16U);
	put(&w, (UW)p_t->enc_l & 0xFFFFFFU, 24U);
	put(&w, (UW)p_t->enc_r & 0xFFFFFFU, 24U);
	put(&w, clamp_u(p_t->hump_peak_mm, 255), 8U);
	put(&w, (UW)obst_cm, 8U);
	put(&w, clamp_u(p_t->obstacle_width_mm / 10, 63), 6U);
	put(&w, clamp_u(p_t->bypass_side, 3), 2U);
	put(&w, clamp_u(p_t->motion_event, 7), 3U);
	put(&w, (p_t->watchdog_reset != 0) ? 1U : 0U, 1U);
	put(&w, p_t->seq & 0xFFU, 8U);
	put(&w, clamp_u(p_t->last_letter, 255), 8U);
	return S1_PACKED_BYTES;
}

EXPORT void s1_unpack(const UB in[S1_PACKED_BYTES], S1_TELEMETRY *p_t)
{
	UINT	b = 8U;		/* skip the version byte */
	W	obst_cm;

	p_t->state = (W)get(in, &b, 4U);
	p_t->last_barcode = (W)get(in, &b, 3U);
	p_t->hump_active = (W)get(in, &b, 1U);
	p_t->fault_mask = get(in, &b, 8U);
	p_t->line_flags = get(in, &b, 3U);
	p_t->line_flags |= (get(in, &b, 1U) != 0U) ? 0x10U : 0U;
	p_t->line_err = (W)get(in, &b, 12U) - 2048;
	p_t->speed_l_mm_s = (W)get(in, &b, 11U) - 1024;
	p_t->speed_r_mm_s = (W)get(in, &b, 11U) - 1024;
	p_t->distance_mm = (W)get(in, &b, 16U) * 10;
	p_t->enc_l = (W)get(in, &b, 24U);
	p_t->enc_r = (W)get(in, &b, 24U);
	p_t->hump_peak_mm = (W)get(in, &b, 8U);
	obst_cm = (W)get(in, &b, 8U);
	p_t->obstacle_mm = (obst_cm == 255) ? -1 : (obst_cm * 10);
	p_t->obstacle_width_mm = (W)get(in, &b, 6U) * 10;
	p_t->bypass_side = (W)get(in, &b, 2U);
	p_t->motion_event = (W)get(in, &b, 3U);
	p_t->watchdog_reset = (W)get(in, &b, 1U);
	p_t->seq = get(in, &b, 8U);
	p_t->last_letter = (W)get(in, &b, 8U);
}
