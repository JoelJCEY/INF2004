/*
 *	task_line.c
 *	Subsystem 3 -- line_task (statechart 6.3).
 *
 *	Every 20 ms: take every new sample the DMA wrote into the ring since
 *	the last cycle (about 20 per channel), then
 *	  - IR_L, IR_C: average -> line error -> publish line_latest (IF-08)
 *	  - IR_R: feed each sample to the barcode reader (Code 39, junction)
 *	  - line lost: straight on for 10 s of driving, then NAV_LINE_LOST
 *	  - calibration request from ctrl_task (IF-10)
 *	The channel of each sample comes from the DMA's own sample count
 *	(sample n came from ADC n % 3), so it carries across ring wraps
 *	(512 is not a multiple of 3) and resynchronises after an overrun.
 */

#include "subsystem3.h"
#include "line_sense.h"
#include "drv_ir_adc.h"
#include "../subsystem2/subsystem2.h"	/* IF-07 odometry */
#include "../subsystem2/motion_ctrl.h"	/* MOTION_UM_PER_COUNT */
#include "../subsystem5/subsystem5.h"	/* IF-09, flg_sys */
#include "../subsystem5/ctrl_logic.h"	/* ctrl_is_active (16 Nov lift check) */

/* no line for 10 s of driving: LINE LOST (car stops), 28 Oct (was 200 mm) */
#define LOST_TIME_MS		10000U
/* C and R are digital line sensors. A left curve uses a forward rolling
   correction when the middle loses black and R is white. Right-side gap
   recovery is retained below. The 10-second line-lost fault is unchanged. */
#define ROLLING_CURVE_ERR 600 /* same magnitude as right-only correction */
#define JUNCTION_HOLD_MM 100U
/* 16 Nov: lifted check from the IR sensors.  Held in the air, nothing
   reflects, so all three sensors read BLACK while the wheels keep
   turning.  On the mat all three are black together only on a crossing
   line (under ~90 mm).  Longer than this while driving = lifted. */
#define IR_LIFT_MM	250U

/* Existing right-side gap/search behavior is retained. */
#define LOST_GRACE_MM 15U
#define LOST_SEARCH_DEG 90U
#define LOST_SEARCH_DELAY_MS 1500U
#define LOST_RECOVERY_START_ERR 180
#define LOST_RECOVERY_MAX_ERR 600
#define LOST_RECOVERY_RAMP_MS 1000U

EXPORT volatile UW line_latest;
EXPORT volatile UW line_cal_request;
EXPORT volatile UW line_cal_result;
EXPORT volatile UW line_read_stats;
EXPORT volatile UW line_last_barcode;
EXPORT volatile UW line_bar_total;
EXPORT volatile UW line_bar_pass;
EXPORT volatile UW line_reader_n;
EXPORT volatile UW line_bar_recross;
EXPORT volatile UW line_enc_rejects;
EXPORT volatile UH line_bc_w01mm[LINE_BC_SHOW_MAX];
EXPORT volatile UW line_bc_info;
EXPORT volatile UW line_bc_seq;

/* Copy the read that just finished for the dashboard (seqlock) */
LOCAL void publish_read(const BC_READER *r, UB ch, INT why)
{
	INT	i;
	INT	n = r->last_n;
	INT	k = (n > LINE_BC_SHOW_MAX) ? LINE_BC_SHOW_MAX : n;
	UW	w;

	line_bc_seq++;				/* odd: writing */
	for (i = 0; i < k; i++) {
		w = r->widths[i] / 100U;	/* µm -> 0.1 mm */
		line_bc_w01mm[i] = (UH)((w > 0xFFFFU) ? 0xFFFFU : w);
	}
	line_bc_info = ((UW)n & 0xFFU) | ((UW)ch << 8)
		| (((UW)(-why) & 0xFFU) << 16);
	line_bc_seq++;				/* even: done */
}

LOCAL LINE_CAL	cal;
LOCAL BC_READER	reader;
LOCAL CROSS_DET	cross;		/* cross-junction detector (IF-09 kind 6) */

/* Barcode TEST mode (app_barcode_test): the car stands still, so the
   wheels give no distance.  Each sample then counts as this much travel
   (0.1 m/s): only the RATIO of bar widths matters to the decoder, so a
   barcode slid under the sensor at a steady hand speed still decodes. */
#define BC_TEST_UM_PER_SAMPLE	100U

/* The barcode reader only moves on while the car rolls FORWARD: during a
   turn or spin on the spot (one wheel backwards) it is paused (2 Nov). */

LOCAL const char *read_reason(INT r)
{
	const char	*p;

	switch (r) {
	case C39_ERR_COUNT:	p = "need exactly 29 (15 "
				    "bars)";		break;
	case C39_ERR_RATIO:	p = "wide and narrow bars too alike";	break;
	case C39_ERR_STARTSTOP:	p = "no '*' at both "
				    "ends";			break;
	case C39_ERR_CHAR:	p = "not a Code 39 pattern";		break;
	default:		p = "?";				break;
	}
	return p;
}

EXPORT void line_cyclic_handler(void *exinf)
{
	(void)exinf;
	tk_wup_tsk(tid_line);
}

LOCAL void send_nav_at(NAV_KIND kind, BARCODE_CMD cmd, UB letter, INT reason,
			UW path_mm)
{
	NAV_MSG	m = {0};

	m.kind = (UB)kind;
	m.cmd = (UB)cmd;
	m.letter = letter;
	m.reason = (UB)(-reason);
	m.path_mm = path_mm;
	if (tk_snd_mbf(mbf_nav, &m, sizeof(m), TMO_POL) < E_OK) {
		app_log((UB*)"line: nav queue full\n");
	}
	(void)tk_set_flg(flg_sys, EVF_NAV);
}

LOCAL void send_nav(NAV_KIND kind, BARCODE_CMD cmd, UB letter, INT reason)
{
	send_nav_at(kind, cmd, letter, reason, motion_odo_path_01mm / 10U);
}

/* Calibration bookkeeping */
EXPORT volatile UW line_raw_avg[3];
EXPORT volatile UW line_norm[3];

LOCAL BOOL	cal_active;
LOCAL UH	cal_min[3], cal_max[3];

LOCAL void calibration_step(const INT avg[3])
{
	UW	req = line_cal_request;
	UINT	i;
	BOOL	ok = TRUE;

	if (req == LINE_CAL_RECORD) {
		if (!cal_active) {
			for (i = 0; i < 3U; i++) {
				cal_min[i] = 0xFFFFU;
				cal_max[i] = 0U;
			}
			cal_active = TRUE;
		}
		for (i = 0; i < 3U; i++) {
			if ((UH)avg[i] < cal_min[i]) {
				cal_min[i] = (UH)avg[i];
			}
			if ((UH)avg[i] > cal_max[i]) {
				cal_max[i] = (UH)avg[i];
			}
		}
	} else if ((req == LINE_CAL_APPLY) && cal_active) {
		cal_active = FALSE;
		/* Each sensor that saw black and white during the sweep gets
		   its own range, so a weak sensor is scaled up to 0..1000.  A
		   sensor that never crossed the line (IR_L sits far out)
		   borrows the range of the sensor with the best contrast. */
		{
			UINT	best = 0U;

			for (i = 1U; i < 3U; i++) {
				if (IR_C_DIGITAL && (i == 1U)) {
					/* digital: 0/4095, not a scale */
					continue;
				}
				if ((cal_max[i] - cal_min[i]) >
				    (cal_max[best] - cal_min[best])) {
					best = i;
				}
			}
			ok = ((cal_max[best] - cal_min[best]) >=
			      LINE_MIN_CONTRAST) ? TRUE : FALSE;
			if (ok) {
				for (i = 0; i < 3U; i++) {
					UINT src = ((cal_max[i] - cal_min[i]) >=
						    LINE_MIN_CONTRAST)
						    	? i : best;
#if LINE_BLACK_IS_HIGH
					/* own white level */
					cal.white[i] = cal_min[i];
					cal.black[i] = cal_min[i]
						+ (cal_max[src] - cal_min[src]);
#else
					cal.white[i] = cal_max[i];
					cal.black[i] = cal_max[i]
						- (cal_max[src] - cal_min[src]);
#endif
				}
				line_cal_result = LINE_CALRES_OK;
			} else {
				line_cal_result = LINE_CALRES_LOW;
			}
		}
		app_log((UB*)"line: calibration %s  L %d-%d  C %d-%d  R "
			     "%d-%d\n",
			  ok ? "OK" : "LOW CONTRAST (defaults kept)",
			  cal_min[0], cal_max[0], cal_min[1], cal_max[1],
			  cal_min[2], cal_max[2]);
	} else {
		/* idle */
	}
}

/* Normalise one sensor: analogue ones through the calibration, a
   digital (DO-wired) centre sensor straight to 0 or 1000 */
LOCAL INT sensor_norm(UINT i, INT raw)
{
#if IR_C_DIGITAL
	if (i == 1U) {
		/* read the DO wire on its own GPIO, not the ADC sample */
		BOOL high = (gpio_get_val(IR_C_DO_PIN) != 0U) ? TRUE : FALSE;

		(void)raw;
		return (high == (IR_C_DO_BLACK_HIGH ? TRUE : FALSE)) ? 1000 : 0;
	}
#endif
#if IR_R_DIGITAL
	if (i == 2U) {
		/* 16 Nov: RIGHT is on its module's DO pin (about 0 or 4095 on
		   the ADC).  Fixed scale, not the start-up sweep: a sweep in
		   which RIGHT saw only black (car started off the mat or in
		   the hand) set its "white" to 4085 and it never read black
		   again until the next restart. */
		INT n = (raw * 1000) / 4095;

		return app_clamp(n, 0, 1000);
	}
#endif
	return line_normalise(&cal, i, raw);
}

/* Wheel travel (mm) for the car to turn d degrees on the spot */
LOCAL UW lost_deg_to_mm(UW d)
{
	return (MOTION_TRACK_MM * 314U * d) / 36000U;
}

/* ---------------------------------------------------------------------
 *  line_task state.  Barr-C (functions kept short): the task loop is
 *  split into the steps below; they share this context, which lives on
 *  line_task's stack exactly like the locals it replaces.
 * --------------------------------------------------------------------- */
typedef struct {
	const volatile UH *ring;
	UW	rd;		/* total samples consumed       */
	UW	wr;
	W	sum[3];
	W	cnt[3];
	INT	avg[3];
	INT	n_l, n_c, n_r;
	INT	err;
	INT	last_sign;
	BOOL	on_line;
	BOOL	lost_mode;
	BOOL	lost_reported;
	UINT	search_leg;
	UW	leg_start_mm;
	UW	lost_ms;		/* driving time without the line */
	UW	junction_until_mm;
	UW	ignore_until_mm;
	UW	path_mm;
	UW	early_until_mm;		/* 16 Nov: C+R early hold ends here */
	BOOL	early_hold;
	UW	um_per_sample;
	/* IR_L travel this cycle not yet given */
	W	bc_left;
	/* IR_L samples left in this cycle */
	UW	l_samples;
	/* encoder distance filter */
	ENC_DIST enc;
	/* spinning: barcode reader paused */
	BOOL	bc_paused;
	UW	bars_seen;		/* reader.bars already counted        */
	UINT	flags;
	UB	letter;
	INT	reason;
	UW	good, bad;
	UW	bars_now;		/* lines scanned, see line_bar_pass */
	UW	bars_last;
	UW	white_ms;
	LINE_TRACK track;
	BOOL	crossing;
} LINE_CTX;

/* Before the loop: sensors, reader, detector and the ADC DMA ring */
LOCAL void line_start(LINE_CTX *c)
{
	line_cal_defaults(&cal);
	bc_reader_reset(&reader);
	enc_dist_init(&c->enc, motion_enc_count[0], motion_enc_count[1],
		      app_time_ms());
	cross_reset(&cross);

	/* Owning task starts the hardware sampling; it never stops (NFR1) */
#if IR_C_DIGITAL
	/* CENTRE sensor DO on a plain GPIO, pulled up (LM393 DO may only
	   pull down) */
	gpio_set_pin(IR_C_DO_PIN, GPIO_MODE_IN);
	set_w(GPIO(IR_C_DO_PIN), GPIO_PUE);
#endif
	if (drv_ir_adc_init() < 0) {
		app_log((UB*)"line: NO FREE DMA CHANNEL for the IR ring\n");
		/* no line data: ctrl never sees READY */
		tk_slp_tsk(TMO_FEVR);
	}
	app_log((UB*)"line: ADC round-robin 0-2 at %d Hz into DMA ring\n",
		  IR_ADC_RATE_HZ);
	c->rd = 0;		/* DMA started at ring[0] with ADC0 */
	(void)tk_set_flg(flg_sys, EVF_LINE_READY);
}

/* Start of a cycle: odometry, new samples, ring overrun, ADC errors */
LOCAL void line_cycle_begin(LINE_CTX *c)
{
	UW	n_new;

	c->path_mm = motion_odo_path_01mm / 10U;
	/* 1 kHz: mm/s == µm/sample */
	c->um_per_sample = (UW)app_abs(motion_speed_avg());
	c->wr = drv_ir_adc_samples_written();
	n_new = c->wr - c->rd;
	if (n_new > (IR_ADC_RING_SAMPLES - 30U)) {
		/* We fell (almost) a whole ring behind: skip to the newest
		   60 samples (20 per channel); the channel stays correct */
		app_log((UB*)"line: DMA ring overrun, %d samples skipped\n",
			  n_new - 60U);
		c->rd = c->wr - 60U;
	}
	if (drv_ir_adc_take_errors() != 0U) {
		app_log((UB*)"line: ADC FIFO error\n");
	}
}

/* Barcode reader distance (encoder distance filter, line_sense.h):
   exact encoder travel of IR_L since the last cycle, shared out over
   this cycle's IR_L samples */
LOCAL void line_barcode_travel(LINE_CTX *c)
{
	W	vl;
	W	vr;
	UW	i;

	c->bc_left = enc_dist_step(&c->enc, motion_enc_count[0],
				   motion_enc_count[1], app_time_ms(),
				   MOTION_UM_PER_COUNT, S3_IR_L_SIDE_MM,
				   MOTION_TRACK_MM);
	vl = (W)motion_speed_left();
	vr = (W)motion_speed_right();

	/* Forward movement only: a turn or spin on the spot (one wheel
	   backwards) sweeps the sensor sideways across the barcode, so the
	   reader is paused until it rolls forward */
	c->bc_paused = ((motion_mode_word == (UW)MM_TURN) ||
			((vl < -10) && (vr > 10)) ||
				((vl > 10) && (vr < -10)) ||
			((motion_mode_word == (UW)MM_MOVE) &&
				((vl + vr) < 0))) ?
		       TRUE : FALSE;
	if (c->bc_paused) {
		c->bc_left = 0;
		/* A sensor sweeping sideways during a pivot is not travelling
		   along a crossing stripe. Discard that candidate as well. */
		cross_reset(&cross);
	}
	c->l_samples = 0;
	for (i = c->rd; i != c->wr; i++) {
		if ((i % 3U) == 0U) {	/* ADC0 = IR_L (never swapped) */
			c->l_samples++;
		}
	}
	if ((app_barcode_test != 0U) &&
	    (app_abs(c->bc_left) < (W)(c->l_samples * 5U))) {
		/* car still: hand test */
		c->bc_left = (W)(c->l_samples * BC_TEST_UM_PER_SAMPLE);
	}
	line_enc_rejects = c->enc.rejects;
}

/* Lines scanned: one per white -> black edge (1 sample = 1 ms per
   channel), counted even if no barcode decodes.  reader.bars: lines
   crossed, re-crossings already taken back (debounced), so the count
   can go down. */
LOCAL void line_count_bars(LINE_CTX *c)
{
	if (reader.bars > c->bars_seen) {
		UW d = reader.bars - c->bars_seen;

		if (c->white_ms >= LINE_PASS_GAP_MS) {
			c->bars_now = 0;		/* new pass */
		}
		c->bars_now += d;
		line_bar_total += d;
	} else if (reader.bars < c->bars_seen) {
		UW d = c->bars_seen - reader.bars;

		c->bars_now = (c->bars_now > d) ? (c->bars_now - d) : 0U;
		line_bar_total -= d;
	} else {
		/* no change */
	}
	c->bars_seen = reader.bars;
	if (reader.black) {
		c->white_ms = 0;
	} else if (c->white_ms < LINE_PASS_GAP_MS) {
		c->white_ms++;
		if ((c->white_ms == LINE_PASS_GAP_MS) && (c->bars_now != 0U)) {
			c->bars_last = c->bars_now;	/* pass finished */
			c->bars_now = 0;
		}
	} else {
		/* long white: idle */
	}
	line_bar_pass = (c->bars_now & 0xFFFFU) |
			((c->bars_last & 0xFFFFU) << 16);
	line_reader_n = (UW)reader.n;
	line_bar_recross = reader.recross;
}

/* Act on what the barcode reader returned for this sample */
LOCAL void line_reader_result(LINE_CTX *c, BC_RESULT res)
{
	if (res == BCR_JUNCTION) {
		if (BARCODE_ACTIONS) {
			send_nav(NAV_JUNCTION_RIGHT, BC_NONE, 0, 0);
		}
	} else if ((res == BCR_BARCODE) &&
		   ((c->path_mm >= c->ignore_until_mm) ||
		    (app_barcode_test != 0U))) {
		/* (test mode: every pass counts, the car does not move) */
		BARCODE_CMD cmd = line_letter_to_cmd(c->letter,
			app_barcode_map);

		c->good++;
		line_last_barcode = (UW)c->letter | ((UW)cmd << 8);
		c->ignore_until_mm = c->path_mm + BC_REPEAT_IGNORE_MM;
		if (BARCODE_ACTIONS) {
			send_nav(NAV_BARCODE, cmd, c->letter, 0);
		}
		app_log((UB*)"line: barcode '%c' cmd %d%s\n", c->letter, cmd,
			BARCODE_ACTIONS ? "" : " (actions OFF, ignored)");
		app_log((UB*)"line: read %d elements (%d bars) -> '%c'\n",
			  reader.last_n, (reader.last_n + 1) / 2, c->letter);
	} else if (res == BCR_REJECT) {
		c->bad++;
		app_log((UB*)"line: read %d elements (%d bars) -> "
			  "rejected: %s\n",
			  reader.last_n, (reader.last_n + 1) / 2,
			  read_reason(c->reason));
		if (BARCODE_ACTIONS) {
			send_nav(NAV_READ_REJECTED, BC_NONE, 0, c->reason);
		}
	} else {
		/* nothing, or repeat inside the 200 mm window */
	}
}

/* Same IR_L sample: cross-junction detector */
LOCAL void line_cross_sample(LINE_CTX *c)
{
	static const char *const why[] = {
		"", "stripe too narrow", "stripe too wide",
		"more bars after it (barcode)",
		"centre/right did not see black"
	};
	UW back_um = 0;

	/* Use the same adaptive AO black/white decision as barcode
	   decoding. A weak IR_L can decode a barcode yet never exceed the
	   old fixed junction threshold. */
	if (!c->bc_paused &&
	    cross_feed_l(&cross, reader.black ? 1000 : 0, c->um_per_sample,
			 &back_um)) {
		UW mid_mm = c->path_mm - (back_um / 1000U);

		c->junction_until_mm = c->path_mm + JUNCTION_HOLD_MM;
		if (BARCODE_ACTIONS) {
			app_log((UB*)"line: CROSS-JUNCTION, middle at %d mm\n",
				mid_mm);
			send_nav_at(NAV_JUNCTION_CROSS, BC_NONE, 0, 0, mid_mm);
		}
	}
	/* 16 Nov: say why a crossing candidate was dropped */
	if (cross.reject != CXR_NONE) {
		app_log((UB*)"line: cross? no: %s (%d mm)\n",
			why[(cross.reject <= CXR_NO_CR) ? cross.reject : 0U],
			(INT)(cross.reject_um / 1000U));
		cross.reject = CXR_NONE;
	}
}

/* Barcode sensor (IR_L, 28 Oct): every sample goes to the barcode
   reader, none to line following */
LOCAL void line_barcode_sample(LINE_CTX *c, INT raw)
{
	INT		n_bc = sensor_norm(IR_BARCODE_SENSOR, raw);
	W		bc_um;
	BC_RESULT	res;

	/* this sample's share of the cycle's travel */
	bc_um = (c->l_samples > 0U) ? (c->bc_left / (W)c->l_samples) : 0;
	c->bc_left -= bc_um;
	if (c->l_samples > 0U) {
		c->l_samples--;
	}
	/* paused (spinning): the reader keeps its state */
	res = c->bc_paused ? BCR_NONE :
	      bc_reader_feed(&reader, n_bc, bc_um, &c->letter, &c->reason);
	if ((res == BCR_BARCODE) ||
	    ((res == BCR_REJECT) && (reader.last_n > 0))) {
		publish_read(&reader, (res == BCR_BARCODE) ? c->letter : 0U,
			     (res == BCR_BARCODE) ? 0 : c->reason);
	}
	line_count_bars(c);
	line_reader_result(c, res);
	line_cross_sample(c);
}

/* Every new sample since the last cycle: sums for the averages, IR_L
   to the barcode reader, IR_C / IR_R to the crossing detector */
LOCAL void line_consume_samples(LINE_CTX *c)
{
	UINT	chan;		/* channel of sample rd: 0,1,2  */

	c->sum[0] = c->sum[1] = c->sum[2] = 0;
	c->cnt[0] = c->cnt[1] = c->cnt[2] = 0;
	while (c->rd != c->wr) {
		INT raw = (INT)(c->ring[c->rd & (IR_ADC_RING_SAMPLES - 1U)] &
				0x0FFFU);

		/* ADC input -> sensor index 0 L, 1 C, 2 R */
		chan = (UINT)(c->rd % 3U);
#if IR_SWAP_C_R
		if (chan != 0U) {
			chan = 3U - chan;	/* ADC1 <-> ADC2 */
		}
#endif
		c->sum[chan] += raw;
		c->cnt[chan]++;
		if (chan == IR_BARCODE_SENSOR) {
			line_barcode_sample(c, raw);
		} else if (!c->bc_paused) {
			/* IR_C / IR_R sample: tells the detector the crossing
			   line also runs under the centre and the right */
			cross_feed_cr(&cross, chan, sensor_norm(chan, raw),
				      c->um_per_sample);
		} else {
			/* paused: not fed */
		}
		c->rd++;
	}
	line_read_stats = (c->good & 0xFFFFU) | (c->bad << 16);
}

#if LINE_DIAG
/* Readable sensor line on the log every 500 ms */
LOCAL void line_diag(const LINE_CTX *c)
{
	static UB div;

	if (++div >= 25U) {
		BOOL ol;

		div = 0;
		app_log((UB*)"ir: raw L %4d C %4d R %4d   "
			  "norm L %4d C %4d R %4d   err %5d\n",
			  c->avg[0], c->avg[1], c->avg[2],
			  sensor_norm(0U, c->avg[0]),
			  sensor_norm(1U, c->avg[1]),
			  sensor_norm(2U, c->avg[2]),
			  line_error(0,
				     sensor_norm(1U, c->avg[1]),
				     sensor_norm(2U, c->avg[2]), &ol));
	}
}
#endif

/* Averages of this cycle, calibration and normalised levels.  FALSE
   when there were no fresh samples this cycle. */
LOCAL BOOL line_levels(LINE_CTX *c)
{
	if ((c->cnt[0] == 0) || (c->cnt[1] == 0)) {
		return FALSE;	/* no fresh samples this cycle */
	}
	c->avg[0] = (INT)(c->sum[0] / c->cnt[0]);
	c->avg[1] = (INT)(c->sum[1] / c->cnt[1]);
	c->avg[2] = (c->cnt[2] != 0) ? (INT)(c->sum[2] / c->cnt[2]) : 0;
	line_raw_avg[0] = (UW)c->avg[0];
	line_raw_avg[1] = (UW)c->avg[1];
#if IR_C_DIGITAL
	/* dashboard shows the DO pin: 0 or 4095 */
	line_raw_avg[1] = (gpio_get_val(IR_C_DO_PIN) != 0U) ? 4095U : 0U;
#endif
	line_raw_avg[2] = (UW)c->avg[2];
	calibration_step(c->avg);
#if LINE_DIAG
	line_diag(c);
#endif
	c->n_l = sensor_norm(0U, c->avg[0]);
	c->n_c = sensor_norm(1U, c->avg[1]);
	c->n_r = sensor_norm(2U, c->avg[2]);
	line_norm[0] = (UW)c->n_l;
	line_norm[1] = (UW)c->n_c;
	line_norm[2] = (UW)c->n_r;
	return TRUE;
}

/* 16 Nov: lifted check (IR_LIFT_MM above) */
LOCAL void line_lift_check(const LINE_CTX *c)
{
	static UW	black_from_mm;
	static BOOL	lift_sent;
	T_RFLG		rf;
	BOOL		all_black = ((c->n_l >= LINE_BLACK_N) &&
				     (c->n_c >= LINE_BLACK_N) &&
				     (c->n_r >= LINE_BLACK_N)) ? TRUE : FALSE;

	(void)tk_ref_flg(flg_sys, &rf);
	if (!all_black || ((rf.flgptn & EVF_HUMP_ACTIVE) != 0U)) {
		black_from_mm = c->path_mm;
		lift_sent = FALSE;
	} else if (!lift_sent &&
		   ((c->path_mm - black_from_mm) >= IR_LIFT_MM)) {
		MISSION_STATE	ms;

		lift_sent = TRUE;
		ctrl_get_mission(&ms);
		if (ctrl_is_active((VEHICLE_STATE)ms.state)) {
			app_log((UB*)"line: all 3 IR sensors black for %d mm "
				"(no floor): LIFTED, motors stop\n",
				(INT)(c->path_mm - black_from_mm));
			app_report_fault(FLT_LIFTED);
		}
	} else {
		/* keep counting */
	}
}

/* 16 Nov: C and R both black = maybe a crossing line arriving at an
   angle, right side first.  Do not steer right for CROSS_EARLY_HOLD_MM,
   so IR_L still reaches the stripe and the detector can confirm it.
   Left corrections still work, so a real right curve only runs
   straight for a few cm. */
LOCAL void line_early_hold(LINE_CTX *c)
{
	static BOOL	cr_both;
	static UW	next_ok_mm;
	BOOL		both = ((c->n_c >= LINE_BLACK_N) &&
				(c->n_r >= LINE_BLACK_N)) ? TRUE : FALSE;

	if (both && !cr_both && !c->bc_paused &&
	    ((W)(c->path_mm - next_ok_mm) >= 0) &&
	    ((W)(c->path_mm - c->junction_until_mm) >= 0)) {
		c->early_until_mm = c->path_mm + CROSS_EARLY_HOLD_MM;
		next_ok_mm = c->path_mm + CROSS_EARLY_GAP_MM;
	}
	cr_both = both;
	c->early_hold = ((W)(c->path_mm - c->early_until_mm) < 0) ?
			TRUE : FALSE;
}

/* Line following uses IR_C and IR_R only; IR_L reads barcodes */
LOCAL void line_steer(LINE_CTX *c)
{
#if LINE_RIGHT_EDGE_ONLY
	static BOOL	r_black;
	static UW	white_from_mm;

	/* hysteresis on the right sensor */
	if (r_black ? (c->n_r < LINE_WHITE_N) : (c->n_r >= LINE_BLACK_N)) {
		r_black = !r_black;
	}
	if (r_black) {
		white_from_mm = c->path_mm;
		c->err = -EDGE_STEER;	/* on the line: ease right */
		c->last_sign = -1;
		c->on_line = TRUE;
	} else {
		c->err = EDGE_STEER;	/* off it: ease left */
		c->last_sign = 1;
		c->on_line = ((c->path_mm - white_from_mm) < EDGE_LOST_MM) ?
			     TRUE : FALSE;
	}
#else
	/* C+R overlap is normal on a right curve. Suppress steering only
	   after the separate cross detector confirms a junction. */
	BOOL	jc;
	BOOL	jr;

	c->err = line_track_step(&c->track, c->n_c, c->n_r, &c->on_line);
	jc = c->track.centre;
	jr = c->track.right;
	if (c->crossing ||
	    ((c->early_hold || ((W)(c->path_mm - c->junction_until_mm) < 0)) &&
	     (c->err < 0))) {
		c->err = 0;	/* still crossing: right sensor ignored */
	} else {
		/* normal rule */
	}

	/* A real right drift needs correction immediately. */
	/* side for recovery: right sensor last = line right.  Not updated
	   inside a junction: the crossing line must not decide which way
	   to search afterwards. */
	if (!c->crossing && !c->early_hold &&
	    ((W)(c->path_mm - c->junction_until_mm) >= 0)) {
		if (jr) {
			c->last_sign = -1;
		} else if (jc) {
			c->last_sign = 1;
		} else {
			/* keep */
		}
	}
#endif
}

/* Back on the line after a search or a LINE LOST */
LOCAL void line_found(LINE_CTX *c)
{
	/* last_sign is set above: centre alone = line went LEFT if lost
	   next, right sensor = line RIGHT */
	if (c->lost_reported) {
		send_nav(NAV_LINE_FOUND, BC_NONE, 0, 0);
	}
	if (c->lost_mode && (c->search_leg >= 1U) && (c->search_leg <= 3U)) {
		app_log((UB*)"line: found again (search step %d)\n",
			c->search_leg);
	}
	c->lost_mode = FALSE;
	c->lost_reported = FALSE;
	c->flags |= LINE_F_ON_LINE;
}

/* Log the step of the fan search that just started */
LOCAL void line_log_search_step(const LINE_CTX *c)
{
	if (c->search_leg == 1U) {
		app_log((UB*)"line: lost, searching %s up to %d deg\n",
			  (c->last_sign > 0) ? "left" : "right",
			  LOST_SEARCH_DEG);
	} else if (c->search_leg == 2U) {
		app_log((UB*)"line: not on the %s, searching the other side\n",
			  (c->last_sign > 0) ? "left" : "right");
	} else if (c->search_leg == 4U) {
		app_log((UB*)"line: not found within +-%d deg, "
			  "going straight\n", LOST_SEARCH_DEG);
	} else {
		/* step 3: back to the start heading */
	}
}

/* Fan search between two boundaries (31 Oct) */
LOCAL void line_fan_search(LINE_CTX *c)
{
	UW	leg = c->path_mm - c->leg_start_mm;
	UW	need = 0;
	INT	side = 0;	/* +1 toward last seen side */

	switch (c->search_leg) {
	case 0:
		need = LOST_GRACE_MM;
		side = 0;
		break;
	case 1:
		need = lost_deg_to_mm(LOST_SEARCH_DEG);
		side = 1;
		break;
	case 2:
		need = lost_deg_to_mm(2U * LOST_SEARCH_DEG);
		side = -1;
		break;
	case 3:
		need = lost_deg_to_mm(LOST_SEARCH_DEG);
		side = 1;
		break;
	default:
		need = 0;
		side = 0;
		break;
	}
	if ((c->search_leg <= 3U) && (leg >= need) &&
	    ((c->search_leg != 0U) || (c->lost_ms >= LOST_SEARCH_DELAY_MS))) {
		c->search_leg++;
		c->leg_start_mm = c->path_mm;
		line_log_search_step(c);
		/* new step: take its direction this tick */
		side = (c->search_leg == 1U) ? 1 : (c->search_leg == 2U) ? -1 :
		       (c->search_leg == 3U) ? 1 : 0;
	}
	c->err = side * c->last_sign * LINE_ERR_MAX;	/* full = spin */
	if (c->search_leg == 0U) {
		UW ramp_ms = (c->lost_ms < LOST_RECOVERY_RAMP_MS) ?
			c->lost_ms : LOST_RECOVERY_RAMP_MS;
		/* C/ R cannot see a line that has moved left. Tighten a
		   rolling arc gradually before resorting to a pivot. */
		c->err = c->last_sign * (LOST_RECOVERY_START_ERR +
			(INT)(ramp_ms * (LOST_RECOVERY_MAX_ERR -
			LOST_RECOVERY_START_ERR) / LOST_RECOVERY_RAMP_MS));
	}
}

/* Both line sensors white: keep rolling toward the last side, search,
   and after LOST_TIME_MS of driving report LINE LOST */
LOCAL void line_lost(LINE_CTX *c)
{
	if (!c->lost_mode) {
		c->lost_mode = TRUE;
		c->lost_ms = 0;
		c->search_leg = 0;
		c->leg_start_mm = c->path_mm;
	}
	if (app_abs(motion_speed_avg()) > 5) {
		c->lost_ms += S3_PERIOD_MS;	/* only while moving */
	}
	if (c->last_sign > 0) {
		/* Middle/right white after centre-only: left rolling arc.
		   Mirror the right-only 600 error without a pivot request. */
		c->err = ROLLING_CURVE_ERR;
	} else {
		line_fan_search(c);
	}
#if !LINE_RIGHT_EDGE_ONLY
	if (c->crossing || ((W)(c->path_mm - c->junction_until_mm) < 0)) {
		c->err = 0;	/* just crossed a junction: straight on */
		c->search_leg = 0;
		c->leg_start_mm = c->path_mm;
	}
#endif
	c->flags |= LINE_F_SATURATED;
	if (c->lost_ms >= LOST_TIME_MS) {
		c->flags |= LINE_F_LOST;
		if (!c->lost_reported) {
			c->lost_reported = TRUE;
			app_log((UB*)"line: LINE LOST: all sensors white for "
				"%d s of driving\n", LOST_TIME_MS / 1000U);
			app_report_fault(FLT_LINE_LOST);
			send_nav(NAV_LINE_LOST, BC_NONE, 0, 0);
		}
	}
}

EXPORT void line_task(INT stacd, void *exinf)
{
	LINE_CTX	c = {0};

	(void)stacd; (void)exinf;

	c.ring = drv_ir_adc_ring();
	c.last_sign = 1;
	line_start(&c);

	while (1) {
		tk_slp_tsk(TMO_FEVR);
		(void)tk_can_wup(TSK_SELF);

		line_cycle_begin(&c);
		line_barcode_travel(&c);
		line_consume_samples(&c);
		if (!line_levels(&c)) {
			continue;	/* no fresh samples this cycle */
		}
		/* On an angled approach R may see the transverse stripe before
		   C/L. Retain their recent evidence until the candidate
		   resolves, instead of steering into the right branch while
		   waiting for the full trailing quiet zone needed by
		   NAV_JUNCTION_CROSS. */
		line_lift_check(&c);
		c.crossing = cross_hold_heading(&cross);
		line_early_hold(&c);
		line_steer(&c);
		c.flags = LINE_F_VALID;
		if (c.on_line) {
			line_found(&c);
		} else {
			line_lost(&c);
		}

		/* IF-08: one aligned 32-bit store */
		line_latest = line_pack(c.err, c.flags, app_time_ms());
	}
}
