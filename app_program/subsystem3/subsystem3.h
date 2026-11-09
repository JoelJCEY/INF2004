/*
 *	subsystem3.h
 *	Subsystem 3 -- Line & Barcode Sensing (line_task).
 *	Buddy 3: 2500991 (Gillton).  Statechart 6.3, assumption A3.
 *
 *	Owns: IR_L GP26/ADC0, IR_C GP27/ADC1, IR_R GP28/ADC2 (ADC round-robin
 *	+ DMA ring, 1 kHz per channel).  Line error from IR_L and IR_C only;
 *	IR_R reads barcodes and right-hand junctions.
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"

#define S3_PRI_LINE		APP_PRI_LINE
#define S3_PERIOD_MS		20

#define IR_L_PIN		26	/* ADC0 */
#define IR_C_PIN		27	/* ADC1 */
#define IR_R_PIN		28	/* ADC2 (VBAT-sense jumper open) */

/* Which physical sensor is wired to which ADC input.  1 = the CENTRE
   and RIGHT sensor cables are the other way round (CENTRE on GP28,
   RIGHT on GP27), as found in bring-up on 28 Oct.  0 = as in the pin
   table above.  Checked by sensor bench steps S3-B04 / S3-B05. */
#define IR_SWAP_C_R		1

/* CENTRE sensor wired to the module's DO (digital) pin instead of AO
   (28 Oct).  The ADC then reads about 0 or about 4095 only, so the
   centre sensor is either "black" (1000) or "white" (0) and the start-up
   sweep does not scale it.  Its switch point is the blue screw on the
   module.  IR_C_DO_BLACK_HIGH: 1 = DO is high (LED off) over black,
   the usual LM393 module; set 0 if the CENTRE bar fills on WHITE. */
/* IR_L is the barcode sensor (28 Oct): it feeds the Code 39 reader
   only and never steers the car */
#define IR_BARCODE_SENSOR	0U
/* 0 = barcodes are read and logged but the car does NOT act on them
   (movement testing, 28 Oct).  Set 1 for the real course. */
#define BARCODE_ACTIONS		1

/* 1 = follow the line with the RIGHT sensor alone, riding the line's
   right edge: right sensor on black -> steer right (off the line),
   on white -> steer left (back onto it).  Works on left and right
   curves without the centre sensor.  0 = centre + right rule. */
#define LINE_RIGHT_EDGE_ONLY	0
#define EDGE_STEER		400	/* steering error used on each side */
#define EDGE_LOST_MM		150U	/* white this far: line really lost */

#define IR_C_DIGITAL		1
#define IR_C_DO_PIN		7	/* the CENTRE module's DO wire goes to GP7 (28 Oct) */
#define IR_C_DO_BLACK_HIGH	1

/* ---------------------------------------------------------------------
 *  IF-08  Latest line error + timestamp (line_task -> motion_task)
 *  Replaces the Week 6 Line Error Queue (F6).  ONE aligned 32-bit word,
 *  line_task is the only writer, motion_task reads it every 20 ms:
 *    bits  0-15  timestamp, low 16 bits of app_time_ms()
 *    bits 16-27  line error, offset binary (error + 2048), -1000..+1000
 *    bits 28-31  LINE_F_xxx flags
 *  Sign: error > 0 means the line is to the LEFT of the car centre
 *  (IR_L sees more black), so the car must steer left.
 * --------------------------------------------------------------------- */
#define LINE_ERR_MAX		1000
#define LINE_F_VALID		0x1U	/* a fresh value was computed        */
#define LINE_F_SATURATED	0x2U	/* both white: last-known-side value  */
#define LINE_F_LOST		0x4U	/* > 200 mm without the line          */
#define LINE_F_ON_LINE		0x8U	/* any IR sensor sees black            */

IMPORT volatile UW line_latest;

Inline UW line_pack(INT err, UINT flags, UW t_ms)
{
	INT	e = APP_CLAMP(err, -2048, 2047) + 2048;

	return (t_ms & 0xFFFFU) | (((UW)e & 0x0FFFU) << 16) |
	       (((UW)flags & 0xFU) << 28);
}
Inline INT line_unpack_err(UW w)
{
	return (INT)((w >> 16) & 0x0FFFU) - 2048;
}
Inline UINT line_unpack_flags(UW w)
{
	return (UINT)(w >> 28);
}
Inline UW line_unpack_age_ms(UW w, UW now_ms)
{
	return (now_ms - (w & 0xFFFFU)) & 0xFFFFU;
}

/* ---------------------------------------------------------------------
 *  IF-09  Navigation Command Queue (line_task -> ctrl_task)
 *  Message buffer mbf_nav, owned by ctrl_task (subsystem5.h), followed
 *  by tk_set_flg(flg_sys, EVF_NAV).
 * --------------------------------------------------------------------- */
typedef enum {
	NAV_BARCODE		= 1,	/* cmd + letter valid                  */
	NAV_JUNCTION_RIGHT	= 2,	/* wide single stripe on IR_R (F3)     */
	NAV_LINE_LOST		= 3,	/* also sets FLT_LINE_LOST             */
	NAV_LINE_FOUND		= 4,	/* line seen again after LINE_LOST     */
	NAV_READ_REJECTED	= 5,	/* diagnostic: partial/invalid read    */
	NAV_JUNCTION_CROSS	= 6	/* cross-junction (line_sense.h); path_mm
					   = odometry at the MIDDLE of the
					   crossing line, not "now"            */
} NAV_KIND;

typedef struct {
	UB	kind;		/* NAV_KIND                      */
	UB	cmd;		/* BARCODE_CMD (3 bits used)     */
	UB	letter;		/* decoded Code 39 character     */
	UB	reason;		/* reject reason (diagnostic)    */
	UW	path_mm;	/* odometry when it happened     */
} NAV_MSG;
APP_STATIC_ASSERT(sizeof(NAV_MSG) == 8, nav_msg_size);

/* ---------------------------------------------------------------------
 *  IF-10  IR calibration (ctrl_task -> line_task), during INITIALISING
 *  ctrl_task sweeps the car over the line and asks line_task to record
 *  the black/white extremes.  Request word: writer ctrl_task only.
 *  Result word: writer line_task only.
 * --------------------------------------------------------------------- */
#define LINE_CAL_IDLE		0U
#define LINE_CAL_RECORD		1U
#define LINE_CAL_APPLY		2U

#define LINE_CALRES_NONE	0U	/* defaults in use        */
#define LINE_CALRES_OK		1U	/* thresholds from sweep  */
#define LINE_CALRES_LOW		2U	/* low contrast: defaults */

IMPORT volatile UW line_cal_request;
IMPORT volatile UW line_cal_result;

/* Diagnostics for telemetry: good reads (bits 0-15), rejects (16-31) */
IMPORT volatile UW line_read_stats;
/* Last decoded letter (bits 0-7) and BARCODE_CMD (bits 8-10) */
IMPORT volatile UW line_last_barcode;
/* Raw ADC averages L, C, R of the last 20 ms (sensor bench, diagnostics) */
IMPORT volatile UW line_raw_avg[3];
/* Normalised 0..1000 (0 white, 1000 black) L, C, R of the same cycle */
IMPORT volatile UW line_norm[3];
/* Lines (black bars) seen by the barcode sensor IR_L, counted on every
   1 kHz sample with the reader's hysteresis, whether or not a barcode is
   decoded (dashboard "Lines scanned").  Writer: line_task only.
     line_bar_total  bars since boot
     line_bar_pass   bits 0-15 bars in the current pass, bits 16-31 bars
                     in the last finished pass (a pass ends after
                     LINE_PASS_GAP_MS of white)
     line_reader_n   elements (bars + gaps) the Code 39 reader holds now */
IMPORT volatile UW line_bar_total;
IMPORT volatile UW line_bar_pass;
IMPORT volatile UW line_reader_n;
IMPORT volatile UW line_bar_recross;	/* re-crossed lines ignored (debounce) */
IMPORT volatile UW line_enc_rejects;	/* bad encoder jumps dropped (filter)  */

/* Last finished barcode read, for the dashboard's barcode picture
   (2 Nov).  Writer line_task only; readers use the sequence like a lock:
   odd = being written, read again later; changed while copying = discard.
     line_bc_w01mm[]  element widths, 0.1 mm, bar first (bar, gap, bar..)
     line_bc_info     bits 0-7 elements (41 = more than 40), bits 8-15
                      letter (0 = rejected), bits 16-23 reject reason
                      (-C39_ERR_xxx) */
#define LINE_BC_SHOW_MAX	40
IMPORT volatile UH line_bc_w01mm[LINE_BC_SHOW_MAX];
IMPORT volatile UW line_bc_info;
IMPORT volatile UW line_bc_seq;
#define LINE_PASS_GAP_MS	1000U

/* How far IR_L (the barcode sensor) sits LEFT of the car's centre line,
   in mm.  MEASURE on the car (ruler, centre of the sensor's spot to the
   middle between the wheels).  Used to work out how fast that sensor
   moves along a barcode while the car is steering, so a bar crossed
   twice during a correction is counted once (BC_RECROSS_UM). */
#define S3_IR_L_SIDE_MM		25

/* ---------------------------------------------------------------------
 *  Private to subsystem 3
 * --------------------------------------------------------------------- */
IMPORT ID	tid_line;
IMPORT ID	cyc_line;
IMPORT void	line_task(INT stacd, void *exinf);
IMPORT void	line_cyclic_handler(void *exinf);

#endif /* SUBSYSTEM3_H */
