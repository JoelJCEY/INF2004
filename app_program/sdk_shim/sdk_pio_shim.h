/*
 *	sdk_pio_shim.h
 *	Plain-C interface to sdk_pio_shim.c (no SDK or kernel types), so it
 *	can be included from both sides.  Wi-Fi builds only.
 */

#ifndef SDK_PIO_SHIM_H
#define SDK_PIO_SHIM_H

#include <stdint.h>

/* Quadrature encoders (Buddy 2, motion_task only).  0 = ok, < 0 = no
 * PIO block with offset 0..23 free, or no free state machine. */
int	shim_quad_init(unsigned int pin_left_a, unsigned int pin_right_a);
int32_t	shim_quad_count(int side);	/* 0 = left, 1 = right */
int	shim_quad_where(int *p_block, int *p_sm_left, int *p_sm_right);

/* WS2812 status NeoPixel (ctrl_task only). */
int	shim_ws2812_init(unsigned int pin, uint32_t sys_hz);
void	shim_ws2812_put_grb(uint32_t grb);

#endif /* SDK_PIO_SHIM_H */
