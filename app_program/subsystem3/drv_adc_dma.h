/*
 *	drv_adc_dma.h
 *	Subsystem 3 -- free-running ADC round-robin (ADC0, ADC1, ADC2) with
 *	DMA into a 1 KB ring.  Register level: the SDK hardware_adc library
 *	is not in the template build.  Call only from line_task.
 *
 *	Do NOT also open the kernel ADC driver ("adca") in this image.
 */

#ifndef DRV_ADC_DMA_H
#define DRV_ADC_DMA_H

#include "subsystem3.h"

#define ADCDMA_RING_SAMPLES	512U	/* 1 KB of 16-bit samples */
#define ADCDMA_RATE_HZ		3000U	/* total = 1 kHz per channel */

IMPORT ER	adcdma_init(void);
/* Total samples written by DMA since start (wraps at 2^32). */
IMPORT UW	adcdma_written(void);
/* Sample at absolute index n (valid only for the last 512). */
IMPORT UH	adcdma_sample(UW n);
/* TRUE if the ADC FIFO overflowed or a conversion error happened since
 * the last call (flags are cleared by the call). */
IMPORT BOOL	adcdma_check_errors(void);
IMPORT INT	adcdma_channel(void);	/* DMA channel used, for the boot log */

#endif /* DRV_ADC_DMA_H */
