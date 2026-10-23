/*
 *	sdk_pio_shim.c
 *	Shared (integration lead) -- the only file that includes Pico SDK
 *	PIO headers.  Used by motion_task (encoders) and ctrl_task (NeoPixel).
 *
 *	The SDK headers and the micro T-Kernel headers define some of the
 *	same names (size_t, register base addresses), so they cannot be
 *	included in one file.  This file includes ONLY SDK headers and offers
 *	plain-C functions (stdint types) to the rest of the application,
 *	the same way the template's Wi-Fi port keeps its SDK code apart.
 *
 *	The SDK PIO library (pio.c) is linked only in Wi-Fi builds, so the
 *	whole file is empty in a WIFI=none build.
 *
 *	Programs: pico-examples quadrature_encoder.pio and ws2812.pio
 *	(BSD-3-Clause, (c) Raspberry Pi Ltd), assembled with pioasm 2.2.0.
 */

#if TM_WIFI_CYW43

#include <stdint.h>
#include <stdbool.h>
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "sdk_pio_shim.h"

/* ---- quadrature encoder (24 instructions, must load at offset 0) ---- */
static const uint16_t quad_instr[] = {
	0x000f, 0x000e, 0x0015, 0x000f, 0x0015, 0x000f, 0x000f, 0x000e,
	0x000e, 0x000f, 0x000f, 0x0015, 0x000f, 0x0015, 0x008f, 0xa0c2,
	0x8000, 0x60c2, 0x4002, 0xa0e6, 0xa0a6, 0xa04a, 0x0097, 0xa04a
};
static const struct pio_program quad_program = {
	.instructions = quad_instr, .length = 24, .origin = 0, .pio_version = 0,
};
#define QUAD_WRAP_TARGET	15u
#define QUAD_WRAP		23u

/* ---- WS2812 NeoPixel (4 instructions, side-set 1, 800 kHz) ---- */
static const uint16_t ws_instr[] = { 0x6321, 0x1223, 0x1200, 0xa242 };
static const struct pio_program ws_program = {
	.instructions = ws_instr, .length = 4, .origin = -1, .pio_version = 0,
};
#define WS_CYCLES_PER_BIT	10u	/* T1 3 + T2 3 + T3 4 */

static PIO	quad_pio = NULL;
static int	quad_sm[2] = { -1, -1 };
static int32_t	quad_last[2] = { 0, 0 };
static PIO	ws_pio = NULL;
static int	ws_sm = -1;

static void quad_sm_start(uint sm, uint pin_a)
{
	pio_sm_config c = pio_get_default_sm_config();

	pio_sm_set_consecutive_pindirs(quad_pio, sm, pin_a, 2, false);
	pio_gpio_init(quad_pio, pin_a);
	pio_gpio_init(quad_pio, pin_a + 1u);
	gpio_pull_up(pin_a);
	gpio_pull_up(pin_a + 1u);
	sm_config_set_wrap(&c, QUAD_WRAP_TARGET, QUAD_WRAP);
	sm_config_set_in_pins(&c, pin_a);
	sm_config_set_jmp_pin(&c, pin_a);
	sm_config_set_in_shift(&c, false, false, 32);
	sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE);
	sm_config_set_clkdiv_int_frac(&c, 1, 0);
	pio_sm_init(quad_pio, sm, 0, &c);
	pio_sm_set_enabled(quad_pio, sm, true);
}

int shim_quad_init(unsigned int pin_left_a, unsigned int pin_right_a)
{
	PIO blocks[2] = { pio1, pio0 };	/* the Wi-Fi driver normally takes pio0 */
	int i;

	for (i = 0; i < 2; i++) {
		if (pio_can_add_program_at_offset(blocks[i], &quad_program, 0)) {
			quad_pio = blocks[i];
			break;
		}
	}
	if (quad_pio == NULL) {
		return -1;
	}
	(void)pio_add_program_at_offset(quad_pio, &quad_program, 0);
	quad_sm[0] = pio_claim_unused_sm(quad_pio, false);
	quad_sm[1] = pio_claim_unused_sm(quad_pio, false);
	if ((quad_sm[0] < 0) || (quad_sm[1] < 0)) {
		return -2;
	}
	quad_sm_start((uint)quad_sm[0], pin_left_a);
	quad_sm_start((uint)quad_sm[1], pin_right_a);
	return 0;
}

int32_t shim_quad_count(int side)
{
	uint n;
	uint sm;

	if ((side < 0) || (side > 1) || (quad_sm[side] < 0)) {
		return 0;
	}
	sm = (uint)quad_sm[side];
	/* the program pushes the count all the time without blocking:
	 * drain the FIFO and keep the newest value */
	n = pio_sm_get_rx_fifo_level(quad_pio, sm);
	while (n > 0u) {
		quad_last[side] = (int32_t)pio_sm_get(quad_pio, sm);
		n--;
	}
	return quad_last[side];
}

int shim_quad_where(int *p_block, int *p_sm_left, int *p_sm_right)
{
	*p_block = (quad_pio == pio1) ? 1 : ((quad_pio == pio0) ? 0 : -1);
	*p_sm_left = quad_sm[0];
	*p_sm_right = quad_sm[1];
	return 0;
}

int shim_ws2812_init(unsigned int pin, uint32_t sys_hz)
{
	PIO blocks[2] = { pio0, pio1 };
	int i;
	int offset = -1;
	pio_sm_config c;
	uint32_t div_x256;

	for (i = 0; i < 2; i++) {
		if (pio_can_add_program(blocks[i], &ws_program)) {
			ws_pio = blocks[i];
			offset = (int)pio_add_program(ws_pio, &ws_program);
			break;
		}
	}
	if (offset < 0) {
		return -1;
	}
	ws_sm = pio_claim_unused_sm(ws_pio, false);
	if (ws_sm < 0) {
		return -2;
	}
	pio_gpio_init(ws_pio, pin);
	pio_sm_set_consecutive_pindirs(ws_pio, (uint)ws_sm, pin, 1, true);
	c = pio_get_default_sm_config();
	sm_config_set_wrap(&c, (uint)offset, (uint)offset + 3u);
	sm_config_set_sideset(&c, 1, false, false);
	sm_config_set_sideset_pins(&c, pin);
	sm_config_set_out_shift(&c, false, true, 24);
	sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
	/* 800 kHz bit rate: div = sys / (800k x 10), in 1/256 steps */
	div_x256 = (uint32_t)(((uint64_t)sys_hz * 256u) / (800000u * WS_CYCLES_PER_BIT));
	sm_config_set_clkdiv_int_frac(&c, (uint16_t)(div_x256 >> 8), (uint8_t)(div_x256 & 0xFFu));
	pio_sm_init(ws_pio, (uint)ws_sm, (uint)offset, &c);
	pio_sm_set_enabled(ws_pio, (uint)ws_sm, true);
	return 0;
}

void shim_ws2812_put_grb(uint32_t grb)
{
	if ((ws_sm >= 0) && !pio_sm_is_tx_fifo_full(ws_pio, (uint)ws_sm)) {
		pio_sm_put(ws_pio, (uint)ws_sm, grb << 8u);
	}
}

#endif /* TM_WIFI_CYW43 */
