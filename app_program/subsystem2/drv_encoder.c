/*
 *	drv_encoder.c
 *	Subsystem 2 -- PIO quadrature encoder driver.
 *
 *	The pico-examples quadrature program counts every edge of A and B in
 *	hardware, so there are no encoder interrupts (design 8.2, A2).  It
 *	uses a computed jump table and must be loaded at offset 0.
 *
 *	This file includes Pico SDK headers only (no tk/tkernel.h).
 */

#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "quadrature_encoder.pio.h"
#include "drv_encoder.h"

static PIO	enc_pio;
static uint32_t	enc_sm[2];
static int32_t	enc_ready;

static void enc_sm_start(PIO pio, uint32_t sm, uint32_t pin)
{
	pio_sm_config	c;

	/* A on pin, B on pin + 1: inputs with pull-ups */
	pio_sm_set_consecutive_pindirs(pio, sm, pin, 2, false);
	pio_gpio_init(pio, pin);
	pio_gpio_init(pio, pin + 1U);
	gpio_pull_up(pin);
	gpio_pull_up(pin + 1U);

	c = quadrature_encoder_program_get_default_config(0);
	sm_config_set_in_pins(&c, pin);
	sm_config_set_jmp_pin(&c, pin);
	sm_config_set_in_shift(&c, false, false, 32);
	sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE);
	sm_config_set_clkdiv_int_frac(&c, 1, 0);	/* full speed: 12.5 Msteps/s */

	pio_sm_init(pio, sm, 0, &c);
	pio_sm_set_enabled(pio, sm, true);
}

int32_t drv_encoder_init(uint32_t pin_left_a, uint32_t pin_right_a,
			 DRV_ENC_INFO *info)
{
	PIO	candidates[2] = { pio0, pio1 };
	int32_t	i;
	int32_t	sm_l;
	int32_t	sm_r;

	info->pio_index = -1;
	info->sm[0] = -1;
	info->sm[1] = -1;
	info->offset = -1;

	for (i = 0; i < 2; i++) {
		PIO p = candidates[i];

		if (!pio_can_add_program_at_offset(p, &quadrature_encoder_program, 0)) {
			continue;
		}
		sm_l = pio_claim_unused_sm(p, false);
		sm_r = (sm_l >= 0) ? pio_claim_unused_sm(p, false) : -1;
		if (sm_r < 0) {
			if (sm_l >= 0) {
				pio_sm_unclaim(p, (uint32_t)sm_l);
			}
			continue;
		}
		(void)pio_add_program_at_offset(p, &quadrature_encoder_program, 0);
		enc_pio = p;
		enc_sm[0] = (uint32_t)sm_l;
		enc_sm[1] = (uint32_t)sm_r;
		enc_sm_start(p, enc_sm[0], pin_left_a);
		enc_sm_start(p, enc_sm[1], pin_right_a);
		info->pio_index = (int8_t)i;
		info->sm[0] = (int8_t)sm_l;
		info->sm[1] = (int8_t)sm_r;
		info->offset = 0;
		enc_ready = 1;
		return 0;
	}
	return -1;
}

int32_t drv_encoder_read(uint32_t wheel)
{
	uint32_t	sm;
	uint32_t	n;
	uint32_t	value = 0U;

	if ((enc_ready == 0) || (wheel > 1U)) {
		return 0;
	}
	sm = enc_sm[wheel];

	/* The program pushes the count continuously (non-blocking).  Drain
	   the FIFO and take one fresh value, ~4 SM cycles away. */
	n = pio_sm_get_rx_fifo_level(enc_pio, sm) + 1U;
	while (n > 0U) {
		value = pio_sm_get_blocking(enc_pio, sm);
		n--;
	}
	return (int32_t)value;
}
