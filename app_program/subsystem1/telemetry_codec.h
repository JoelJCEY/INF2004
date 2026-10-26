#ifndef S1_TELEMETRY_CODEC_H
#define S1_TELEMETRY_CODEC_H

#include <stdint.h>
#include "subsystem1.h"
#include "../subsystem5/subsystem5.h"

/* Version 1 MQTT payload, 36 bytes. All multi-byte values are little-endian.
 *  0 version (1); 1-4 milliseconds; 5-8 sequence;
 *  9 state bits 0-3, barcode bits 4-6, Wi-Fi bit 7; 10 fault mask;
 *  11-12 left speed; 13-14 right speed; 15-18 left encoder;
 *  19-22 right encoder; 23-26 distance; 27-28 line error;
 *  29 line flags; 30 hump active (bit 0); 31-32 hump peak;
 *  33-34 obstacle distance (-1 means none); 35 last barcode letter.
 * Signed 16/32-bit numbers use two's complement. The signed 16-bit fields
 * are provisional until physical measurement establishes their ranges.
 * This is an explicit wire format; never publish an S1_TELEMETRY C struct.
 */
static inline void s1_put16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
}

static inline void s1_put32(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
    out[2] = (uint8_t)(value >> 16);
    out[3] = (uint8_t)(value >> 24);
}

static inline int s1_pack_state_barcode(const S1_TELEMETRY *t, uint8_t *out)
{
    if (t == 0 || out == 0 || t->state < 0 || t->state >= VS_NUM_STATES ||
        t->last_barcode < BC_NONE || t->last_barcode > BC_END_OF_COURSE ||
        (t->wifi_up != 0 && t->wifi_up != 1)) {
        return 0;
    }
    *out = (uint8_t)((uint8_t)t->state |
                     ((uint8_t)t->last_barcode << 4) |
                     ((uint8_t)t->wifi_up << 7));
    return 1;
}

static inline int s1_fits16(W x)
{
    return x >= -32768 && x <= 32767;
}

static inline unsigned int s1_encode_telemetry(const S1_TELEMETRY *t,
                                               uint8_t *out,
                                               unsigned int capacity)
{
    uint8_t packed;
    if (t == 0 || out == 0 || capacity < S1_MQTT_FRAME_SIZE ||
        !s1_pack_state_barcode(t, &packed) || t->fault_mask > 255U ||
        t->line_flags > 255U || t->last_letter < 0 || t->last_letter > 255 ||
        (t->hump_active != 0 && t->hump_active != 1) ||
        !s1_fits16(t->speed_l_mm_s) || !s1_fits16(t->speed_r_mm_s) ||
        !s1_fits16(t->line_err) || !s1_fits16(t->hump_peak_mm) ||
        !s1_fits16(t->obstacle_mm)) {
        return 0;
    }
    out[0] = 1;
    s1_put32(out + 1, (uint32_t)t->t_ms);
    s1_put32(out + 5, (uint32_t)t->seq);
    out[9] = packed;
    out[10] = (uint8_t)t->fault_mask;
    s1_put16(out + 11, (uint16_t)t->speed_l_mm_s);
    s1_put16(out + 13, (uint16_t)t->speed_r_mm_s);
    s1_put32(out + 15, (uint32_t)t->enc_l);
    s1_put32(out + 19, (uint32_t)t->enc_r);
    s1_put32(out + 23, (uint32_t)t->distance_mm);
    s1_put16(out + 27, (uint16_t)t->line_err);
    out[29] = (uint8_t)t->line_flags;
    out[30] = (uint8_t)t->hump_active;
    s1_put16(out + 31, (uint16_t)t->hump_peak_mm);
    s1_put16(out + 33, (uint16_t)t->obstacle_mm);
    out[35] = (uint8_t)t->last_letter;
    return S1_MQTT_FRAME_SIZE;
}

#endif /* S1_TELEMETRY_CODEC_H */
