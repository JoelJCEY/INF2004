/*
 *	subsystem3.c
 *	Subsystem 3 -- Line & Barcode Sensing: starts the ADC/DMA sampling,
 *	creates its cyclic handler and starts line_task.
 */

#include "subsystem3.h"

EXPORT ID	tid_line;
EXPORT ID	cyc_line;

EXPORT ER subsystem3_init(void)
{
	T_CCYC	ccyc = {0};

	/* The ADC and its DMA ring are started by line_task itself (its
	   owning task), so ring index 0 is known to hold ADC0. */
	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)line_cyclic_handler;
	ccyc.cyctim = S3_PERIOD_MS;
	ccyc.cycphs = S3_PERIOD_MS;
	cyc_line = tk_cre_cyc(&ccyc);
	return (cyc_line < E_OK) ? cyc_line : E_OK;
}

EXPORT ER subsystem3_start(void)
{
	tid_line = app_start_task(line_task, S3_PRI_LINE, 0, "line_task");
	if (tid_line < E_OK) {
		return tid_line;
	}
	return tk_sta_cyc(cyc_line);
}
