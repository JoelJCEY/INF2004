/*
 *	task_imu.c
 *	Subsystem 4 -- imu_task: tilt, hump detection and peak height,
 *	impact detection, motion-event classification (statechart 6.4).
 *
 *	At start-up (car standing still, INITIALISING): 50 samples (1 s)
 *	give the level baseline, then EVF_IMU_READY is set.
 *	Every 20 ms afterwards:
 *	  - read the accelerometer, low-pass filter it
 *	  - dh = wheelbase * a_fwd / |a|, held while the encoders show the
 *	    car braking or accelerating (F12: forward acceleration tilts the
 *	    measured gravity vector)
 *	  - hump detector over encoder distance (hump.c)
 *	  - impact: |a| far from 1 g (higher threshold on a hump)
 */

#include "subsystem4.h"
#include "drv_lsm303.h"
#include "hump.h"
#include "../subsystem2/subsystem2.h"	/* IF-07 odometry */
#include "../subsystem5/subsystem5.h"	/* IF-11, flg_sys */


EXPORT volatile W  imu_acc_raw[3];
EXPORT volatile W  imu_mag_raw[3];
EXPORT volatile W  imu_base_mg[2];
EXPORT volatile UW imu_reads_ok;
EXPORT volatile UW imu_reads_fail;

#define BASELINE_SAMPLES	50
#define LPF_SHIFT		2	/* IIR, alpha = 1/4 */
#define GATE_DV_MM_S		6	/* per tick = 300 mm/s^2 */
#define GATE_HOLD_TICKS		15	/* keep holding 300 ms after a speed change (body pitch settles) */
#define IMPACT_MG		2500	/* deviation from 1 g (set from logs) */
#define IMPACT_HUMP_MG		4000	/* landing off a hump is not an impact */
#define IMPACT_HOLDOFF_TICKS	25	/* 500 ms between impact reports */
#define TURN_DIFF_MM_S		80
#define STATIONARY_MM_S		10
#define I2C_FAIL_LIMIT		10

EXPORT volatile UW imu_status_word;

LOCAL HUMP_DET	hump;

EXPORT void imu_cyclic_handler(void *exinf)
{
	(void)exinf;
	tk_wup_tsk(tid_imu);
}

LOCAL void send_terrain(TERRAIN_KIND kind, MOTION_EVENT_CLASS ev, W peak_um)
{
	TERRAIN_MSG	m = {0};
	W		p01 = peak_um / 100;

	m.kind = (UB)kind;
	m.event = (UB)ev;
	m.peak_01mm = (UH)APP_CLAMP(p01, 0, 0xFFFF);
	m.path_mm = motion_odo_path_01mm / 10U;
	if (tk_snd_mbf(mbf_terrain, &m, sizeof(m), TMO_POL) < E_OK) {
		app_log((UB*)"imu: terrain queue full\n");
	}
	(void)tk_set_flg(flg_sys, EVF_TERRAIN);
}

/* Bring the chip up; retried every second until it answers */
LOCAL void wait_for_imu(void)
{
	BOOL	reported = FALSE;

	while (drv_lsm303_init() != E_OK) {
		if (!reported) {
			app_log((UB*)"imu: LSM303DLHC not answering on I2C0\n");
			app_report_fault(FLT_IMU);
			reported = TRUE;
		}
		tk_dly_tsk(1000);
	}
}

EXPORT void imu_task(INT stacd, void *exinf)
{
	LSM303_ACC	a;
	LSM303_MAG	mag;
	W		fx = 0, fy = 0, fz = 1000;
	W		x0 = 0, y0 = 0;
	W		sx = 0, sy = 0;
	INT		n = 0;
	W		dh = 0, dh_held = 0, a_fwd, mag_mg, peak = 0;
	H		v_prev = 0, v_now, v_l, v_r;
	UINT		fails = 0;
	UINT		tick = 0;
	UINT		holdoff = 0;
	UINT		gate = 0;
	UINT		hits = 0;	/* last 3 samples over the impact limit, 1 bit each */
	UW		humps = 0;
	W		run_peak_01mm = 0;
	MOTION_EVENT_CLASS ev;
	HUMP_EVENT	he;

	(void)stacd; (void)exinf;

	wait_for_imu();
	hump_reset(&hump);

	/* Level baseline: 1 s with the car still */
	while (n < BASELINE_SAMPLES) {
		tk_slp_tsk(TMO_FEVR);
		if (drv_lsm303_read_acc(&a) == E_OK) {
			sx += a.x;
			sy += a.y;
			n++;
		}
	}
	x0 = sx / BASELINE_SAMPLES;
	y0 = sy / BASELINE_SAMPLES;
	fx = x0;
	fy = y0;
	imu_base_mg[0] = x0;
	imu_base_mg[1] = y0;
	app_log((UB*)"imu: baseline x %d mg, y %d mg\n", x0, y0);
	(void)tk_set_flg(flg_sys, EVF_IMU_READY);

	while (1) {
		tk_slp_tsk(TMO_FEVR);
		(void)tk_can_wup(TSK_SELF);
		tick++;

		if (drv_lsm303_read_acc(&a) != E_OK) {
			imu_reads_fail++;
			if (++fails == I2C_FAIL_LIMIT) {
				app_log((UB*)"imu: I2C read failing\n");
				app_report_fault(FLT_IMU);
			}
			continue;
		}
		fails = 0;
		imu_reads_ok++;
		imu_acc_raw[0] = a.x;
		imu_acc_raw[1] = a.y;
		imu_acc_raw[2] = a.z;
		if ((tick % 5U) == 0U) {
			if (drv_lsm303_read_mag(&mag) == E_OK) {	/* heading cross-check only */
				imu_mag_raw[0] = mag.x;
				imu_mag_raw[1] = mag.y;
				imu_mag_raw[2] = mag.z;
			}
		}

		/* Low-pass filter (wide 32-bit internally, design 10.2) */
		fx += (a.x - fx) >> LPF_SHIFT;
		fy += (a.y - fy) >> LPF_SHIFT;
		fz += (a.z - fz) >> LPF_SHIFT;
		mag_mg = (W)hump_isqrt((UW)((fx * fx) + (fy * fy) + (fz * fz)));

		/* Impact uses the raw reading: filtering would hide the spike */
		{
			W raw_mag = (W)hump_isqrt((UW)((a.x * a.x) + (a.y * a.y) + (a.z * a.z)));
			W limit = (hump.state != HS_LEVEL) ? IMPACT_HUMP_MG : IMPACT_MG;

			BOOL over = (APP_ABS(raw_mag - 1000) > limit) ? TRUE : FALSE;
			UINT n_hits;

			/* 2 of the last 3 samples: one bad sample (vibration,
			   motor noise on I2C) is not an impact */
			hits = ((hits << 1) | (over ? 1U : 0U)) & 0x7U;
			n_hits = (hits & 1U) + ((hits >> 1) & 1U) + ((hits >> 2) & 1U);
			if (holdoff > 0U) {
				holdoff--;
			} else if (over && (n_hits >= 2U)) {
				holdoff = IMPACT_HOLDOFF_TICKS;
				hits = 0;
				app_report_fault(FLT_IMPACT);
				send_terrain(TR_IMPACT, ME_IMPACT, 0);
				app_log((UB*)"imu: IMPACT %d mg\n", raw_mag);
			} else if (over) {
				app_log((UB*)"imu: spike %d mg ignored\n", raw_mag);
			} else {
				/* nothing */
			}
		}

		/* Tilt -> dh, gated while the speed changes */
		v_now = motion_speed_avg();
		a_fwd = IMU_FWD_SIGN * (fx - x0);
		dh = hump_dh_um(a_fwd, mag_mg);
		if (APP_ABS(v_now - v_prev) > GATE_DV_MM_S) {
			gate = GATE_HOLD_TICKS;
		} else if (gate > 0U) {
			gate--;
		} else {
			dh_held = dh;
		}
		v_prev = v_now;

		he = hump_update(&hump, dh_held, motion_odo_path_01mm, &peak);
		switch (he) {
		case HE_START:
			(void)tk_set_flg(flg_sys, EVF_HUMP_ACTIVE);
			send_terrain(TR_HUMP_START, ME_CLIMBING, 0);
			break;
		case HE_END:
			(void)tk_clr_flg(flg_sys, ~EVF_HUMP_ACTIVE);
			humps++;
			if ((peak / 100) > run_peak_01mm) {
				run_peak_01mm = peak / 100;
			}
			send_terrain(TR_HUMP_END, ME_DESCENDING, peak);
			app_log((UB*)"imu: hump peak %d.%d mm\n", peak / 1000,
				  (peak % 1000) / 100);
			break;
		case HE_DISCARD:
			(void)tk_clr_flg(flg_sys, ~EVF_HUMP_ACTIVE);
			send_terrain(TR_HUMP_DISCARD, ME_CRUISING, 0);
			break;
		default:
			break;
		}

		/* Motion-event classification for telemetry */
		v_l = motion_speed_left();
		v_r = motion_speed_right();
		if (holdoff > (IMPACT_HOLDOFF_TICKS - 5U)) {
			ev = ME_IMPACT;
		} else if (hump.state == HS_CLIMBING) {
			ev = ME_CLIMBING;
		} else if ((hump.state == HS_DESCENDING) || (hump.state == HS_REAR_CROSSING)) {
			ev = ME_DESCENDING;
		} else if ((APP_ABS(v_l) < STATIONARY_MM_S) && (APP_ABS(v_r) < STATIONARY_MM_S)) {
			ev = ME_STATIONARY;
		} else if (APP_ABS(v_r - v_l) > TURN_DIFF_MM_S) {
			/* turn rate = (vR - vL) / track: no gyroscope on the DLHC */
			ev = ME_TURNING;
		} else if (dh != dh_held) {
			ev = ME_ACCELERATING;
		} else {
			ev = ME_CRUISING;
		}
		imu_status_word = ((UW)ev & 0x7U) |
				  (((humps > 15U) ? 15U : humps) << 3) |
				  (((UW)run_peak_01mm & 0xFFFFU) << 8);
	}
}
