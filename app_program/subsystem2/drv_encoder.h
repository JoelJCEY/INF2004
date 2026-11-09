/*
 *	drv_encoder.h
 *	Subsystem 2 -- PIO quadrature encoder driver (left GP2/GP3, right GP4/GP5).
 *
 *	Plain C interface (stdint only): drv_encoder.c includes Pico SDK
 *	headers, which must not be mixed with tk/tkernel.h in one file
 *	(size_t clash, see lib/libwifi/.../cyw43_utk_kernel.c).
 *	Owner: motion_task only (design 9).
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <stdint.h>

#define DRV_ENC_LEFT	0
#define DRV_ENC_RIGHT	1

typedef struct {
	int8_t	pio_index;	/* 0 or 1, -1 if not loaded */
	int8_t	sm[2];		/* state machine per wheel */
	int8_t	offset;		/* program offset: must be 0 */
} DRV_ENC_INFO;

/* Load the program at offset 0 of PIO0 (falls back to PIO1) and start one
   state machine per wheel.  Must run before the CYW43 driver claims PIO
   memory (A2).  Returns 0 on success, negative on failure. */
int32_t drv_encoder_init(uint32_t pin_left_a, uint32_t pin_right_a,
			 DRV_ENC_INFO *info);

/* Signed count of one wheel (4 counts per encoder cycle). */
int32_t drv_encoder_read(uint32_t wheel);

#endif /* DRV_ENCODER_H */
