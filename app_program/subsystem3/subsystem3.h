/*
 *	subsystem3.h
 *	Subsystem 3 -- IR line following and Code 39 barcode decoding (Buddy 3).
 *
 *	COMPLETE VERSION (adds start-up calibration, right-junction
 *	detection and read diagnostics to the basic behaviour below).
 *	  - ADC round-robin on ADC0-ADC2 (never ADC3 = GP29, the Wi-Fi clock)
 *	    at 1 kHz per channel, written by DMA into a 1 KB ring (512
 *	    samples, about 170 ms), so no sample is lost if line_task is late.
 *	  - Every 20 ms: line error from IR_L and IR_C, published as
 *	    "latest value + timestamp" (one aligned word each).
 *	  - IR_R samples feed the Code 39 reader (width ratios, 29 elements,
 *	    either direction, same barcode ignored for 200 mm).
 *	  - Both line sensors white: keep the last side for 200 mm of
 *	    travel, then report LINE LOST.
 *	  - Calibration: while ctrl_task turns the car over the line at
 *	    start-up, line_task records each sensor's min and max and uses
 *	    them as white/black; the numbers below are only the fallback.
 *	  - Junction: an isolated black stripe under IR_R at least
 *	    S3_JUNCTION_MIN_MM long (by odometer) is a right junction, not
 *	    a barcode (provisional value: check a junction stripe on the mat).
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"

#define S3_PRI_LINE		12
#define S3_PERIOD_MS		20
#define S3_PRINT_ENABLE		0	/* 1 = print line error every 0.5 s */

#define S3_NUM_IR		3
#define S3_IR_L			0	/* ADC0, GP26, Grove 6 */
#define S3_IR_C			1	/* ADC1, GP27, Grove 6 */
#define S3_IR_R			2	/* ADC2, GP28, Grove 7 */

/* ---- Calibration (assumption A3): raw 12-bit readings measured with
 * the bring-up logger over the real mat.  These modules usually read
 * HIGH over black; if yours read LOW over black, just swap the values. */
#define S3_WHITE_L		400
#define S3_BLACK_L		3200
#define S3_WHITE_C		400
#define S3_BLACK_C		3200
#define S3_WHITE_R		400
#define S3_BLACK_R		3200

/* Normalised level: 0 = white, 1000 = black.  Barcode thresholds with
 * hysteresis so edge noise does not split or merge bars. */
#define S3_BAR_ON		600	/* white -> black above this */
#define S3_BAR_OFF		400	/* black -> white below this */
#define S3_LINE_BLACK		500	/* a line sensor counts as "on black" */

#define S3_LOST_MM		200	/* last-known side kept for this far */
#define S3_REPEAT_MM		200	/* same barcode ignored this far */
#define S3_ERR_MAX		1500	/* saturated line error */

/* ---- Line error slot (provider: line_task, readers: motion_task, ctrl).
 * err > 0 means the line is to the LEFT of the car (steer left).      */
#define S3_LF_L_BLACK		(1U << 0)
#define S3_LF_C_BLACK		(1U << 1)
#define S3_LF_R_BLACK		(1U << 2)
#define S3_LF_LOST		(1U << 3)	/* both line sensors white */
#define S3_LF_LINE_LOST		(1U << 4)	/* and for more than 200 mm */

IMPORT void	s3_line_get(W *p_err, UW *p_age_ms, UW *p_flags);

#define S3_JUNCTION_MIN_MM	40	/* provisional */
#define S3_CAL_MIN_SPAN		800	/* raw counts black-white to accept */

/* Calibration handshake: ctrl_task writes the request, line_task the result. */
typedef enum {
	CAL_IDLE = 0,
	CAL_RECORD,		/* ctrl: car is sweeping, record min/max */
	CAL_FINISH		/* ctrl: sweep done, compute thresholds */
} CAL_REQUEST;
IMPORT volatile CAL_REQUEST	s3_cal_request;		/* ctrl_task only */
IMPORT volatile W		s3_cal_result;		/* line_task: 0 none, 1 ok, -1 failed */
IMPORT volatile UW		s3_junctions;

/* Read by tests and telemetry.  Written only by line_task. */
IMPORT volatile BOOL	s3_ready;
IMPORT volatile UW	s3_samples;		/* total samples processed */
IMPORT volatile UW	s3_overruns;		/* ring overruns seen */
IMPORT volatile UH	s3_last_norm[S3_NUM_IR];/* 0..1000 */
IMPORT volatile UB	s3_last_letter;		/* last decoded character */
IMPORT volatile UW	s3_reads_ok;
IMPORT volatile UW	s3_reads_bad;

IMPORT ID	tid_line;
IMPORT ID	cyc_line;
IMPORT void	line_task(INT stacd, void *exinf);
IMPORT void	line_cyclic_handler(void *exinf);

#endif /* SUBSYSTEM3_H */
