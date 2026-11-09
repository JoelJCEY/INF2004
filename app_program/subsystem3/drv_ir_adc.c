/*
 *	drv_ir_adc.c
 *	Subsystem 3 -- ADC round-robin + DMA ring for the three IR sensors.
 *	This file includes Pico SDK headers only (no tk/tkernel.h).
 */

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/resets.h"
#include "hardware/structs/adc.h"
#include "hardware/regs/dreq.h"
#include "drv_ir_adc.h"

#define ADC_CLK_HZ	48000000U	/* clk_adc from PLL_USB */
#define RING_BITS	10U		/* 1 KB = 512 x 16-bit  */

static volatile uint16_t ir_ring[IR_ADC_RING_SAMPLES]
	__attribute__((aligned(IR_ADC_RING_SAMPLES * sizeof(uint16_t))));
static int32_t	ir_dma = -1;

int32_t drv_ir_adc_init(void)
{
	dma_channel_config	c;
	uint32_t		pin;
	int32_t			ch;

	/* Analogue pins: digital input off, no pulls, no function */
	for (pin = 26U; pin <= 28U; pin++) {
		gpio_set_function(pin, GPIO_FUNC_NULL);
		gpio_disable_pulls(pin);
		gpio_set_input_enabled(pin, false);
		if (pin == IR_DO_PULLUP_PIN) {
			gpio_pull_up(pin);	/* DO output: see drv_ir_adc.h */
		}
	}

	unreset_block_wait(RESETS_RESET_ADC_BITS);
	adc_hw->cs = ADC_CS_EN_BITS;
	while ((adc_hw->cs & ADC_CS_READY_BITS) == 0U) {
		;
	}

	/* Sample period = 1 + INT cycles of 48 MHz */
	adc_hw->div = ((ADC_CLK_HZ / IR_ADC_RATE_HZ) - 1U) << ADC_DIV_INT_LSB;
	adc_hw->fcs = ADC_FCS_EN_BITS | ADC_FCS_DREQ_EN_BITS |
		      (1U << ADC_FCS_THRESH_LSB) |
		      ADC_FCS_OVER_BITS | ADC_FCS_UNDER_BITS;	/* W1C */
	while ((adc_hw->fcs & ADC_FCS_EMPTY_BITS) == 0U) {
		(void)adc_hw->fifo;
	}

	ch = dma_claim_unused_channel(false);
	if (ch < 0) {
		return -1;
	}
	c = dma_channel_get_default_config((uint32_t)ch);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, true);
	channel_config_set_ring(&c, true, RING_BITS);	/* wrap the write */
	channel_config_set_dreq(&c, DREQ_ADC);
	dma_channel_configure((uint32_t)ch, &c, ir_ring, &adc_hw->fifo,
			      0xFFFFFFFFU, true);	/* ~16 days at 3 kHz */
	ir_dma = ch;

	/* First conversion is ADC0, then 1, 2, 0, ... in ring order */
	adc_hw->cs = ADC_CS_EN_BITS | (0U << ADC_CS_AINSEL_LSB) |
		     (0x7U << ADC_CS_RROBIN_LSB) | ADC_CS_START_MANY_BITS;
	return ch;
}

uint32_t drv_ir_adc_write_index(void)
{
	uint32_t	wa;

	if (ir_dma < 0) {
		return 0U;
	}
	wa = dma_hw->ch[ir_dma].write_addr;
	return ((wa - (uint32_t)ir_ring) / sizeof(uint16_t)) & (IR_ADC_RING_SAMPLES - 1U);
}

uint32_t drv_ir_adc_samples_written(void)
{
	if (ir_dma < 0) {
		return 0U;
	}
	return 0xFFFFFFFFU - dma_hw->ch[ir_dma].transfer_count;
}

const volatile uint16_t *drv_ir_adc_ring(void)
{
	return ir_ring;
}

uint32_t drv_ir_adc_take_errors(void)
{
	uint32_t	f = adc_hw->fcs & (ADC_FCS_OVER_BITS | ADC_FCS_UNDER_BITS);

	adc_hw->fcs = (adc_hw->fcs & ~(ADC_FCS_OVER_BITS | ADC_FCS_UNDER_BITS)) | f;
	return f;
}
