/*
 *	drv_ir_adc.h
 *	Subsystem 3 -- IR sensors: ADC round-robin over ADC0..ADC2 + DMA ring.
 *
 *	Hardware-timed sampling (NFR1): the ADC free-runs at 3 kHz total
 *	(1 kHz per channel) and a DMA channel copies every result into a
 *	1 KB ring (512 x 16-bit = 170 ms), so a late line_task loses nothing.
 *	Samples stay 16-bit (12-bit results; hardware width, design 10.2).
 *	The round-robin mask is ADC0..ADC2 only: never ADC3/GP29, which is
 *	the Wi-Fi SPI clock (11.1).
 *
 *	Plain C interface: drv_ir_adc.c includes Pico SDK headers only.
 *	Owner: line_task only.
 */

#ifndef DRV_IR_ADC_H
#define DRV_IR_ADC_H

#include <stdint.h>

#define IR_ADC_CHANNELS		3U
#define IR_ADC_RING_SAMPLES	512U		/* power of two (DMA ring) */
#define IR_ADC_RATE_HZ		3000U		/* all channels together   */

/* The CENTRE sensor is wired to its module's DO pin on GP28 (28 Oct).
   DO comes from an LM393 comparator, which on many modules only pulls
   DOWN (open collector): with the ADC pad's pulls off nothing pulls it
   up and the reading never changes.  Turn on the pad's internal pull-up
   on this pin.  0 = no pull-up (all sensors on AO). */
#define IR_DO_PULLUP_PIN	0U

/* Start the ADC and its DMA channel (claimed with
   dma_claim_unused_channel, not a fixed number).  Returns the DMA
   channel (>= 0) or a negative value on failure. */
int32_t drv_ir_adc_init(void);

/* Ring index the DMA will write next (0..IR_ADC_RING_SAMPLES-1). */
uint32_t drv_ir_adc_write_index(void);

/* Total samples the DMA has written since start (from its transfer
   counter).  Sample n is in ring[n % 512] and came from ADC n % 3, so
   the channel can always be recovered, even after an overrun. */
uint32_t drv_ir_adc_samples_written(void);

/* Read-only view of the ring. */
const volatile uint16_t *drv_ir_adc_ring(void);

/* Returns and clears the ADC FIFO overflow/underflow flags. */
uint32_t drv_ir_adc_take_errors(void);

#endif /* DRV_IR_ADC_H */
