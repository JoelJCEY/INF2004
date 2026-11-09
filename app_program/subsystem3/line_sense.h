/*
 *	line_sense.h
 *	Subsystem 3 -- pure line and barcode logic (no hardware, no kernel
 *	calls), unit-tested in subsystem3_test.c.
 */

#ifndef LINE_SENSE_H
#define LINE_SENSE_H

#include "subsystem3.h"

/* ---------------------------------------------------------------------
 *  Calibration and normalisation (12-bit raw -> 0..1000, 1000 = black)
 *  Defaults are ESTIMATES (a past INF2004 car read white below ~1800 and
 *  black above it on a 12-bit ADC): replace with the A3 logs from the mat.
 *  The start-up calibration sweep overwrites them when contrast is good.
 * --------------------------------------------------------------------- */
/* 1 = print raw and normalised IR values every 500 ms (bring-up, S3-M01).
   Set to 0 once the thresholds are set. */
#define LINE_DIAG		1

#define LINE_BLACK_IS_HIGH	1	/* analogue IR modules: black = high V */
#define LINE_DEF_WHITE		250	/* measured 26 Oct: white 130-250    */
#define LINE_DEF_BLACK		3700	/* measured 26 Oct: black 3600-4060  */
#define LINE_MIN_CONTRAST	300	/* raw counts between black and white (was 800;
					   lowered 28 Oct so a weak centre sensor is
					   scaled up instead of ignored) */

typedef struct {
	UH	white[3];	/* IR_L, IR_C, IR_R */
	UH	black[3];
} LINE_CAL;

IMPORT void line_cal_defaults(LINE_CAL *cal);
IMPORT INT  line_normalise(const LINE_CAL *cal, UINT ch, INT raw);

/* Line error from IR_L and IR_C only (FR1).  n_l, n_c: 0..1000.
   Returns -1000..+1000 (> 0: line to the left).  *on_line is TRUE when
   either sensor sees black. */
#define LINE_BLACK_N		400	/* hysteresis: black above ... (was 600, lowered 28 Oct) */
#define LINE_WHITE_N		250	/* ... white below (was 400)    */
IMPORT INT  line_error(INT n_l, INT n_c, INT n_r, BOOL *on_line);

/* ---------------------------------------------------------------------
 *  Code 39 (design 6.3, VA2): 29 elements = '*' X '*' with two
 *  inter-character gaps.  Wide/narrow by relative width (~3:1), so the
 *  exact speed does not matter; tried forwards then backwards.
 * --------------------------------------------------------------------- */
#define CODE39_ELEMENTS		29
#define CODE39_MIN_RATIO_X10	15	/* narrowest wide >= 1.5 x widest narrow */

#define C39_OK			0
#define C39_ERR_COUNT		(-1)	/* not 29 elements      */
#define C39_ERR_RATIO		(-2)	/* wide/narrow unclear  */
#define C39_ERR_STARTSTOP	(-3)	/* no '*' at both ends  */
#define C39_ERR_CHAR		(-4)	/* not in the table     */

IMPORT INT  code39_decode(const UW *widths, INT n, UB *letter);
/* Letter -> movement under the MQTT barcode map (app.h IF-15).  A
   letter in the map always gives its movement; otherwise the end
   marker gives BC_END_OF_COURSE and anything else BC_UNRECOGNISED. */
IMPORT BARCODE_CMD line_letter_to_cmd(UB letter, UW map);

/* ---------------------------------------------------------------------
 *  Barcode reader state machine (statechart 6.3), fed one IR_R sample
 *  at a time (1 kHz).  Widths are measured in µm travelled, using the
 *  encoder speed (at 1 kHz, µm per sample = mm/s).
 * --------------------------------------------------------------------- */
#define BC_MAX_ELEMENTS		40
#define BC_QUIET_MIN_UM		12000U	/* white needed before and after   */
#define BC_JUNCTION_MIN_UM	16000U	/* single stripe wider than any bar */
#define BC_REPEAT_IGNORE_MM	200U	/* same barcode ignored for 200 mm */
#define BC_MAX_SAMPLES		10000U	/* > 10 s inside a read: reject     */
#define BC_MAX_READ_UM		400000	/* > 400 mm of travel inside one read:
					   not a barcode (2 Nov: distance, so a
					   slow car is not cut off at 3 s)    */
/* Debounce (2 Nov): the reader keeps the SENSOR's position along the
   barcode (signed: it goes backwards when the car swings back toward the
   line).  An edge that comes back within BC_RECROSS_UM of the edge just
   crossed is the sensor re-crossing the same line, or noise: that edge is
   UNDONE instead of adding two extra elements.  The real narrow bar is a
   few mm, so 0.5 mm cannot hide a real one. */
#define BC_RECROSS_UM		500
typedef enum {
	BCS_WAIT_GAP	= 0,	/* WAITING FOR BARCODE (needs quiet zone) */
	BCS_ARMED	= 1,	/* quiet zone seen, waiting for a bar     */
	BCS_READING	= 2	/* READING BARS                            */
} BC_STATE;

typedef enum {
	BCR_NONE	= 0,
	BCR_BARCODE	= 1,	/* letter valid                 */
	BCR_JUNCTION	= 2,	/* single wide stripe           */
	BCR_REJECT	= 3	/* reason valid                 */
} BC_RESULT;

typedef struct {
	BC_STATE	state;
	BOOL		black;		/* hysteresis level            */
	UW		run_um;		/* current run length          */
	UW		widths[BC_MAX_ELEMENTS];
	INT		n;
	UW		max_um;
	UW		samples;
	BOOL		overflow;
	INT		last_n;		/* elements in the read that just
					   finished (bars + spaces; a whole
					   Code 39 letter is 29 = 15 bars) */
	W		pos;		/* sensor position, µm, signed  */
	W		edge_pos;	/* position of the last edge    */
	W		prev_edge_pos;	/* ... and of the one before    */
	W		start_pos;	/* first bar of this read       */
	W		quiet_pos;	/* start of the white before it */
	BOOL		can_undo;	/* last edge may still be undone */
	UW		bars;		/* black lines crossed (undone
					   re-crossings removed)        */
	UW		recross;	/* edges undone (debounced)     */
} BC_READER;

/* ---------------------------------------------------------------------
 *  Encoder distance filter for the barcode sensor (2 Nov).
 *  Bar widths come from the wheel ENCODER COUNTS that changed during
 *  each 20 ms cycle (exact distance), not from the 60 ms averaged speed,
 *  turned into how far IR_L itself moved (it sits S3_IR_L_SIDE_MM left of
 *  centre, so it slows / reverses while the car steers back):
 *      d = (dL + dR) / 2 - (dR - dL) * side / track
 *  A count jump the car cannot physically make in the elapsed time
 *  (faster than ENC_DIST_MAX_MM_S, plus ENC_DIST_SLACK_UM) is a bad
 *  encoder reading: that cycle's distance is dropped and counted.
 * --------------------------------------------------------------------- */
#define ENC_DIST_MAX_MM_S	1000	/* 2 x the 0.5 m/s demo maximum */
#define ENC_DIST_SLACK_UM	2000

typedef struct {
	W	prev_l;
	W	prev_r;
	UW	prev_ms;
	UW	rejects;	/* implausible jumps dropped */
} ENC_DIST;

IMPORT void enc_dist_init(ENC_DIST *e, W cnt_l, W cnt_r, UW now_ms);
/* Travel of the sensor (µm, signed, + = forward) since the last call */
IMPORT W    enc_dist_step(ENC_DIST *e, W cnt_l, W cnt_r, UW now_ms,
			  W um_per_count, W side_mm, W track_mm);

IMPORT void      bc_reader_reset(BC_READER *r);
/* n_r: normalised sensor 0..1000.  um: distance the SENSOR moved along
   the barcode during this sample, negative when it moves backwards. */
IMPORT BC_RESULT bc_reader_feed(BC_READER *r, INT n_r, W um,
				UB *letter, INT *reason);

/* ---------------------------------------------------------------------
 *  Cross-junction detector (demo rule: the stored barcode is carried out
 *  at the next cross-junction).  A crossing line runs under the whole
 *  sensor row, so a cross-junction is:
 *    - an ISOLATED stripe under IR_L (the barcode sensor, outside the
 *      line): white for CROSS_QUIET_UM before it, white for
 *      max(2 x its width, CROSS_QUIET_UM) after it, the same isolation
 *      rule the barcode reader uses, so a barcode bar never counts;
 *    - CROSS_MIN_UM..CROSS_MAX_UM wide (the crossing line, ~18 mm);
 *    - IR_C AND IR_R on black within CROSS_SLACK_UM of that stripe (the
 *      line continues on the right as well: not a left-only branch).
 *  Fed one sample at a time at 1 kHz per channel, like the reader.
 *  DESIGN VALUES: check on the real mat (testing/subsystem3, S3-M05).
 * --------------------------------------------------------------------- */
#define CROSS_MIN_UM		8000U	/* narrower: noise / thin bar    */
#define CROSS_MAX_UM		60000U	/* wider: the main line under IR_L */
#define CROSS_QUIET_UM		12000U	/* = BC_QUIET_MIN_UM            */
#define CROSS_SLACK_UM		40000U	/* sensors not exactly in a row  */

typedef enum {
	CXS_IDLE	= 0,	/* waiting for white before a stripe */
	CXS_STRIPE	= 1,	/* IR_L on a candidate stripe        */
	CXS_AFTER	= 2	/* stripe ended: waiting for white   */
} CX_STATE;

typedef struct {
	CX_STATE	state;
	BOOL		l_black;	/* IR_L level with hysteresis   */
	UW		run_um;		/* current IR_L run             */
	UW		stripe_um;	/* width of the candidate       */
	UW		since_c_um;	/* distance since IR_C was black */
	UW		since_r_um;	/* distance since IR_R was black */
	BOOL		c_seen;
	BOOL		r_seen;
} CROSS_DET;

IMPORT void cross_reset(CROSS_DET *d);
/* Centre / right sensor sample (normalised 0..1000) */
IMPORT void cross_feed_cr(CROSS_DET *d, UINT ch, INT n, UW um);
/* IR_L sample.  TRUE once per cross-junction; *back_um = distance from
   the middle of the crossing line to the sensor row right now. */
IMPORT BOOL cross_feed_l(CROSS_DET *d, INT n_l, UW um, UW *back_um);

#endif /* LINE_SENSE_H */
