/*
 *	drv_adc_dma.c
 *	Subsystem 3 -- ADC round-robin + DMA ring (RP2040 datasheet ch. 2.5, 4.9).
 *
 *	ADC clock is 48 MHz.  One conversion every 16,000 clocks = 3 kHz in
 *	total, i.e. 1 kHz on each of ADC0, ADC1, ADC2.  The ADC FIFO raises a
 *	DMA request for every sample; the DMA channel copies the 16-bit result
 *	into a 1024-byte ring (write-address wrapping, RING_SIZE = 10) and
 *	counts down from 0xFFFFFFFF, so "samples written" = start - count.
 *	At 3 kHz the count lasts about 16 days, far longer than any run.
 */

#include "drv_adc_dma.h"

/* ---- ADC registers (base 0x4004c000) ---- */
#define ADC_BASE_A		0x4004C000U
#define ADC_CS_R		(ADC_BASE_A + 0x00U)
#define ADC_FCS_R		(ADC_BASE_A + 0x08U)
#define ADC_FIFO_R		(ADC_BASE_A + 0x0CU)
#define ADC_DIV_R		(ADC_BASE_A + 0x10U)
#define ADC_INTE_R		(ADC_BASE_A + 0x18U)

#define ADC_CS_EN_B		(1U << 0)
#define ADC_CS_START_MANY_B	(1U << 3)
#define ADC_CS_READY_B		(1U << 8)
#define ADC_CS_ERR_STICKY_B	(1U << 10)
#define ADC_CS_AINSEL_SHIFT	12
#define ADC_CS_RROBIN_SHIFT	16

#define ADC_FCS_EN_B		(1U << 0)
#define ADC_FCS_ERR_B		(1U << 2)
#define ADC_FCS_DREQ_EN_B	(1U << 3)
#define ADC_FCS_EMPTY_B		(1U << 8)
#define ADC_FCS_UNDER_B		(1U << 10)
#define ADC_FCS_OVER_B		(1U << 11)
#define ADC_FCS_THRESH_SHIFT	24

#define ADC_RROBIN_MASK		0x07U	/* ADC0..ADC2 only, never ADC3 */
#define ADC_DIV_INT		((48000000U / ADCDMA_RATE_HZ) - 1U)

/* ---- DMA registers (base 0x50000000) ---- */
#define DMA_BASE_A		0x50000000U
#define DMA_CH(n)		(DMA_BASE_A + ((UW)(n) * 0x40U))
#define DMA_READ_ADDR(n)	(DMA_CH(n) + 0x00U)
#define DMA_WRITE_ADDR(n)	(DMA_CH(n) + 0x04U)
#define DMA_TRANS_COUNT(n)	(DMA_CH(n) + 0x08U)
#define DMA_CTRL_TRIG(n)	(DMA_CH(n) + 0x0CU)
#define DMA_CHAN_ABORT		(DMA_BASE_A + 0x444U)

#define DMA_CTRL_EN_B		(1U << 0)
#define DMA_CTRL_SIZE_HALF	(1U << 2)
#define DMA_CTRL_INCR_WRITE_B	(1U << 5)
#define DMA_CTRL_RING_SHIFT	6
#define DMA_CTRL_RING_SEL_B	(1U << 10)	/* wrap the write address */
#define DMA_CTRL_CHAIN_SHIFT	11
#define DMA_CTRL_TREQ_SHIFT	15
#define DMA_CTRL_ERR_MASK	(7U << 29)
#define DREQ_ADC		36U

#define ADCDMA_START_COUNT	0xFFFFFFFFU

/* The DMA ring must be aligned to its own size. */
LOCAL UH	adc_ring[ADCDMA_RING_SAMPLES] __attribute__((aligned(1024)));
LOCAL INT	dma_ch = -1;

#if TM_WIFI_CYW43
/* The Wi-Fi driver claims its DMA channels through the SDK claim list
 * (dma.c is linked in Wi-Fi builds), so ask the same list for ours. */
extern int dma_claim_unused_channel(_Bool required);
#endif

LOCAL INT adcdma_pick_channel(void)
{
#if TM_WIFI_CYW43
	return (INT)dma_claim_unused_channel(0);
#else
	return 11;	/* no other DMA user in a WIFI=none build */
#endif
}

LOCAL void adc_pad_analog(UINT pin)
{
	out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_NULL);
	clr_w(GPIO(pin), GPIO_IE | GPIO_PUE | GPIO_PDE);
}

EXPORT ER adcdma_init(void)
{
	UW	ctrl;

	dma_ch = adcdma_pick_channel();
	if (dma_ch < 0) {
		return E_LIMIT;
	}

	/* The kernel releases the DMA block from reset only in Wi-Fi builds
	 * (the radio uses DMA).  Do it here otherwise; atomic clear alias. */
	if ((in_w(RESETS_RESET_DONE) & RESETS_RESET_DMA) == 0U) {
		out_w(RESETS_RESET + 0x3000U, RESETS_RESET_DMA);
		while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_DMA) == 0U) {
			;
		}
	}

	adc_pad_analog(26U);
	adc_pad_analog(27U);
	adc_pad_analog(28U);

	/* Stop the ADC, empty the FIFO, clear sticky flags. */
	out_w(ADC_INTE_R, 0U);
	out_w(ADC_CS_R, ADC_CS_EN_B);
	while ((in_w(ADC_CS_R) & ADC_CS_READY_B) == 0U) {
		;
	}
	out_w(ADC_FCS_R, 0U);
	while ((in_w(ADC_FCS_R) & ADC_FCS_EMPTY_B) == 0U) {
		(void)in_w(ADC_FIFO_R);
	}
	out_w(ADC_FCS_R, ADC_FCS_UNDER_B | ADC_FCS_OVER_B);	/* W1C */

	/* DMA: FIFO -> ring, halfwords, write wraps every 1024 bytes. */
	out_w(DMA_READ_ADDR(dma_ch), ADC_FIFO_R);
	out_w(DMA_WRITE_ADDR(dma_ch), (UW)&adc_ring[0]);
	out_w(DMA_TRANS_COUNT(dma_ch), ADCDMA_START_COUNT);
	ctrl = DMA_CTRL_EN_B | DMA_CTRL_SIZE_HALF | DMA_CTRL_INCR_WRITE_B |
	       (10U << DMA_CTRL_RING_SHIFT) | DMA_CTRL_RING_SEL_B |
	       ((UW)dma_ch << DMA_CTRL_CHAIN_SHIFT) |	/* chain to self = none */
	       (DREQ_ADC << DMA_CTRL_TREQ_SHIFT);
	out_w(DMA_CTRL_TRIG(dma_ch), ctrl);

	/* ADC: FIFO with DREQ at 1 sample, 3 kHz, round-robin from ADC0. */
	out_w(ADC_FCS_R, ADC_FCS_EN_B | ADC_FCS_DREQ_EN_B | ADC_FCS_ERR_B |
	      (1U << ADC_FCS_THRESH_SHIFT));
	out_w(ADC_DIV_R, ADC_DIV_INT << 8);
	out_w(ADC_CS_R, ADC_CS_EN_B | (0U << ADC_CS_AINSEL_SHIFT) |
	      (ADC_RROBIN_MASK << ADC_CS_RROBIN_SHIFT));
	set_w(ADC_CS_R, ADC_CS_START_MANY_B);
	return E_OK;
}

EXPORT UW adcdma_written(void)
{
	return ADCDMA_START_COUNT - in_w(DMA_TRANS_COUNT(dma_ch));
}

EXPORT UH adcdma_sample(UW n)
{
	/* Bit 15 is the per-sample error flag (FCS.ERR); keep 12 data bits. */
	return (UH)(adc_ring[n & (ADCDMA_RING_SAMPLES - 1U)] & 0x0FFFU);
}

EXPORT BOOL adcdma_check_errors(void)
{
	UW	fcs = in_w(ADC_FCS_R);
	UW	cs = in_w(ADC_CS_R);
	BOOL	bad = FALSE;

	if ((fcs & (ADC_FCS_OVER_B | ADC_FCS_UNDER_B)) != 0U) {
		set_w(ADC_FCS_R, ADC_FCS_OVER_B | ADC_FCS_UNDER_B);	/* W1C */
		bad = TRUE;
	}
	if ((cs & ADC_CS_ERR_STICKY_B) != 0U) {
		set_w(ADC_CS_R, ADC_CS_ERR_STICKY_B);			/* W1C */
		bad = TRUE;
	}
	if ((in_w(DMA_CTRL_TRIG(dma_ch)) & DMA_CTRL_ERR_MASK) != 0U) {
		bad = TRUE;
	}
	return bad;
}

EXPORT INT adcdma_channel(void)
{
	return dma_ch;
}
