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
 *	  - 13 Nov: LIFTED or OVERTURNED (lift.h) instead of impact: the car
 *	    stops at once (lifting it is the demo's end-of-run signal)
 */

#include "subsystem4.h"
#include "drv_lsm303.h"
#include "hump.h"
#include "lift.h"
#include "../subsystem2/subsystem2.h"	/* IF-07 odometry */
#include "../subsystem5/subsystem5.h"	/* IF-11, flg_sys */
#include "../subsystem5/ctrl_logic.h"	/* ctrl_is_active */


EXPORT volatile W  imu_acc_raw[3];
EXPORT volatile W  imu_mag_raw[3];
EXPORT volatile W  imu_base_mg[2];
EXPORT volatile W  imu_base_z_mg = 1000;
EXPORT volatile W  imu_acc_filt[3];
EXPORT volatile UW imu_reads_ok;
EXPORT volatile UW imu_reads_fail;
EXPORT volatile W  imu_tilt_cos1000 = 1000;
EXPORT volatile W  imu_up_mg;
EXPORT volatile UW imu_lift_now;
EXPORT volatile UW imu_lift_count;

#define BASELINE_SAMPLES	50
#define BASELINE_SKIP		10	/* 13 Nov: first 0.2 s after power-on ignored */
#define BASELINE_STILL_MG	80	/* max spread per axis while learning "flat" */
#define BASELINE_TRIES		3
#define LPF_SHIFT		2	/* IIR, alpha = 1/4 */
#define GATE_DV_MM_S		6	/* per tick = 300 mm/s^2 */
#define GATE_HOLD_TICKS		15	/* keep holding 300 ms after a speed change (body pitch settles) */
/* 13 Nov: collision (impact) detection removed by the requirement
   change.  Lifted / overturned instead (lift.h). */
#define LIFT_HOLDOFF_TICKS	50	/* 1 s between lift reports */
#define TURN_DIFF_MM_S		80
#define STATIONARY_MM_S		10
#define I2C_FAIL_LIMIT		10

EXPORT volatile UW imu_status_word;

LOCAL HUMP_DET	hump;
LOCAL LIFT_DET	lift;

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

/* 13 Nov: TRUE while the car is driving (motors may be running).  Only
   asked when a lift / overturn is seen, so the mission mutex is rare. */
LOCAL BOOL car_driving(void)
{
	MISSION_STATE	ms;

	ctrl_get_mission(&ms);
	return ctrl_is_active((VEHICLE_STATE)ms.state);
}

EXPORT void imu_task(INT stacd, void *exinf)
{
	LSM303_ACC	a;
	LSM303_MAG	mag;
	W		fx = 0, fy = 0, fz = 1000;
	W		x0 = 0, y0 = 0;
	W		sx = 0, sy = 0, sz = 0;
	INT		n = 0;
	W		dh = 0, dh_held = 0, a_fwd, mag_mg, peak = 0;
	H		v_prev = 0, v_now, v_l, v_r;
	UINT		fails = 0;
	UINT		tick = 0;
	UINT		holdoff = 0;
	UINT		gate = 0;
	UW		humps = 0;
	W		run_peak_01mm = 0;
	MOTION_EVENT_CLASS ev;
	HUMP_EVENT	he;

	(void)stacd; (void)exinf;

	wait_for_imu();
	hump_reset(&hump);

	/* Level baseline: 1 s with the car still.  13 Nov: the first
	   readings after power-on are skipped, and if the car moved while
	   it was measuring (spread over BASELINE_STILL_MG on any axis) it
	   measures again, up to BASELINE_TRIES times (start-up limit 5 s). */
	{
		INT	tries = 0;
		INT	skip = BASELINE_SKIP;
		W	lo[3], hi[3];
		BOOL	again = TRUE;

		while (again) {
			sx = 0; sy = 0; sz = 0; n = 0;
			lo[0] = lo[1] = lo[2] = 32767;
			hi[0] = hi[1] = hi[2] = -32768;
			while (n < BASELINE_SAMPLES) {
				tk_slp_tsk(TMO_FEVR);
				if (drv_lsm303_read_acc(&a) != E_OK) {
					continue;
				}
				if (skip > 0) {
					skip--;
					continue;
				}
				sx += a.x;
				sy += a.y;
				sz += a.z;
				lo[0] = (a.x < lo[0]) ? a.x : lo[0];	hi[0] = (a.x > hi[0]) ? a.x : hi[0];
				lo[1] = (a.y < lo[1]) ? a.y : lo[1];	hi[1] = (a.y > hi[1]) ? a.y : hi[1];
				lo[2] = (a.z < lo[2]) ? a.z : lo[2];	hi[2] = (a.z > hi[2]) ? a.z : hi[2];
				n++;
			}
			tries++;
			again = FALSE;
			if (((hi[0] - lo[0]) > BASELINE_STILL_MG) || ((hi[1] - lo[1]) > BASELINE_STILL_MG) ||
			    ((hi[2] - lo[2]) > BASELINE_STILL_MG)) {
				if (tries < BASELINE_TRIES) {
					app_log((UB*)"imu: car moved while learning flat (spread %d/%d/%d mg): measuring again\n",
						  hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]);
					again = TRUE;
				} else {
					app_log((UB*)"imu: car kept moving while learning flat: using it anyway (power on with the car still)\n");
				}
			}
		}
	}
	x0 = sx / BASELINE_SAMPLES;
	y0 = sy / BASELINE_SAMPLES;
	fx = x0;
	fy = y0;
	fz = sz / BASELINE_SAMPLES;
	lift_init(&lift, x0, y0, fz);	/* "down" = the level reading */
	imu_base_mg[0] = x0;
	imu_base_mg[1] = y0;
	imu_base_z_mg = fz;
	app_log((UB*)"imu: baseline x %d mg, y %d mg, z %d mg\n", x0, y0, fz);
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
		imu_acc_filt[0] = fx;	/* dashboard only (is11/car/imu) */
		imu_acc_filt[1] = fy;
		imu_acc_filt[2] = fz;
		mag_mg = (W)hump_isqrt((UW)((fx * fx) + (fy * fy) + (fz * fz)));

		/* 13 Nov: LIFTED or OVERTURNED -> stop the motors (lift.h).
		   Raw reading for the lift push, filtered one for the tilt. */
		{
			W		raw[3];
			W		filt[3];
			LIFT_RESULT	lr;

			raw[0] = a.x;	raw[1] = a.y;	raw[2] = a.z;
			filt[0] = fx;	filt[1] = fy;	filt[2] = fz;
			lr = lift_update(&lift, raw, filt, (hump.state != HS_LEVEL) ? TRUE : FALSE);
			imu_tilt_cos1000 = lift.last_cos1000;
			imu_up_mg = lift.last_up_mg;
			imu_lift_now = (UW)lr;
			if (holdoff > 0U) {
				holdoff--;
			} else if ((lr != LIFT_NONE) && !car_driving()) {
				/* 13 Nov: motors are not running, nothing to stop.
				   Picking the car up to place it, carrying it or
				   pressing its buttons no longer latches a fault. */
				holdoff = LIFT_HOLDOFF_TICKS;
			} else if (lr != LIFT_NONE) {
				holdoff = LIFT_HOLDOFF_TICKS;
				imu_lift_count++;
				app_report_fault(FLT_LIFTED);		/* ctrl_task stops the motors */
				send_terrain(TR_LIFTED, ME_LIFTED, 0);
				if (lr == LIFT_LIFTED) {
					app_log((UB*)"imu: LIFTED (pushed up %d mg): motors stop\n",
						  lift.last_up_mg);
				} else {
					app_log((UB*)"imu: OVERTURNED / tilted (cos %d/1000 from level): motors stop\n",
						  lift.last_cos1000);
				}
			} else if ((lift.last_up_mg > 80) && ((tick % 25U) == 0U) && car_driving()) {
				/* 16 Nov: near miss, so a lift that did not stop the
				   car shows how hard it was pushed up */
				app_log((UB*)"imu: push up %d mg (lift needs > %d)\n",
					lift.last_up_mg, LIFT_UP_MG);
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
		if (holdoff > (LIFT_HOLDOFF_TICKS - 5U)) {
			ev = ME_LIFTED;
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
