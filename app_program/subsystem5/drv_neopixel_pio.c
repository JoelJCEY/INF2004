/*
 *	drv_neopixel_pio.c
 *	Subsystem 5 -- WS2812 NeoPixel output on GP18 through PIO.
 *	Pico SDK headers only (no tk/tkernel.h).  Prefers PIO1, because
 *	PIO0 offset 0 holds the encoder program (11.1, PIO blocks).
 */

#include "hardware/pio.h"
#include "ws2812.pio.h"

int  neopixel_pio_init(unsigned int pin);
void neopixel_pio_put(unsigned int grb, unsigned int count);

static PIO	np_pio;
static int	np_sm = -1;

int neopixel_pio_init(unsigned int pin)
{
	PIO		order[2] = { pio1, pio0 };
	pio_sm_config	c;
	int		i;
	int		sm;
	int		offset;

	for (i = 0; i < 2; i++) {
		if (!pio_can_add_program(order[i], &ws2812_program)) {
			continue;
		}
		sm = pio_claim_unused_sm(order[i], false);
		if (sm < 0) {
			continue;
		}
		offset = pio_add_program(order[i], &ws2812_program);
		np_pio = order[i];
		np_sm = sm;

		pio_gpio_init(np_pio, pin);
		pio_sm_set_consecutive_pindirs(np_pio, (uint)sm, pin, 1, true);
		c = ws2812_program_get_default_config((uint)offset);
		sm_config_set_sideset_pins(&c, pin);
		sm_config_set_out_shift(&c, false, true, 24);
		sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
		/* 800 kHz x 10 cycles per bit: 125 MHz / 8 MHz = 15.625 */
		sm_config_set_clkdiv_int_frac(&c, 15, 160);
		pio_sm_init(np_pio, (uint)sm, (uint)offset, &c);
		pio_sm_set_enabled(np_pio, (uint)sm, true);
		return 0;
	}
	return -1;
}

void neopixel_pio_put(unsigned int grb, unsigned int count)
{
	unsigned int	i;

	if (np_sm < 0) {
		return;
	}
	for (i = 0; i < count; i++) {
		/* Never block ctrl_task: skip the update if the FIFO is full */
		if (pio_sm_is_tx_fifo_full(np_pio, (uint)np_sm)) {
			return;
		}
		pio_sm_put(np_pio, (uint)np_sm, grb << 8U);
	}
}
