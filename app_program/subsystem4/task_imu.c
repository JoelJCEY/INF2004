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
/* 13 Nov: first 0.2 s after power-on ignored */
#define BASELINE_SKIP		10
/* max spread per axis while learning "flat" */
#define BASELINE_STILL_MG	80
#define BASELINE_TRIES		3
#define LPF_SHIFT		2	/* IIR, alpha = 1/4 */
#define GATE_DV_MM_S		6	/* per tick = 300 mm/s^2 */
/* keep holding 300 ms after a speed change (body pitch settles) */
#define GATE_HOLD_TICKS		15
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
	m.peak_01mm = (UH)app_clamp(p01, 0, 0xFFFF);
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

/* ---------------------------------------------------------------------
 *  imu_task state.  Barr-C (functions kept short): the task is split
 *  into the steps below; they share this context, which lives on
 *  imu_task's stack exactly like the locals it replaces.
 * --------------------------------------------------------------------- */
typedef struct {
	LSM303_ACC	a;
	W		fx, fy, fz;	/* low-pass filtered, mg           */
	W		x0, y0;		/* level baseline                  */
	W		dh, dh_held, mag_mg, peak;
	H		v_prev;
	UINT		fails;
	UINT		tick;
	UINT		holdoff;
	UINT		gate;
	UW		humps;
	W		run_peak_01mm;
} IMU_CTX;

/* Level baseline: 1 s with the car still.  13 Nov: the first readings
   after power-on are skipped, and if the car moved while it was
   measuring (spread over BASELINE_STILL_MG on any axis) it measures
   again, up to BASELINE_TRIES times (start-up limit 5 s). */
LOCAL void imu_learn_level(IMU_CTX *c)
{
	LSM303_ACC	a;
	W		sx = 0, sy = 0, sz = 0;
	INT		n;
	INT		tries = 0;
	INT		skip = BASELINE_SKIP;
	W		lo[3], hi[3];
	BOOL		again = TRUE;

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
			lo[0] = (a.x < lo[0]) ? a.x : lo[0];
			hi[0] = (a.x > hi[0]) ? a.x : hi[0];
			lo[1] = (a.y < lo[1]) ? a.y : lo[1];
			hi[1] = (a.y > hi[1]) ? a.y : hi[1];
			lo[2] = (a.z < lo[2]) ? a.z : lo[2];
			hi[2] = (a.z > hi[2]) ? a.z : hi[2];
			n++;
		}
		tries++;
		again = FALSE;
		if (((hi[0] - lo[0]) > BASELINE_STILL_MG) ||
		    ((hi[1] - lo[1]) > BASELINE_STILL_MG) ||
		    ((hi[2] - lo[2]) > BASELINE_STILL_MG)) {
			if (tries < BASELINE_TRIES) {
				app_log((UB*)"imu: car moved while learning "
					  "flat (spread %d/%d/%d mg): "
					  "measuring again\n",
					  hi[0] - lo[0], hi[1] - lo[1],
					  hi[2] - lo[2]);
				again = TRUE;
			} else {
				app_log((UB*)"imu: car kept moving while "
					  "learning flat: using it anyway "
					  "(power on with the car still)\n");
			}
		}
	}
	c->x0 = sx / BASELINE_SAMPLES;
	c->y0 = sy / BASELINE_SAMPLES;
	c->fx = c->x0;
	c->fy = c->y0;
	c->fz = sz / BASELINE_SAMPLES;
	lift_init(&lift, c->x0, c->y0, c->fz);	/* "down" = the level reading */
	imu_base_mg[0] = c->x0;
	imu_base_mg[1] = c->y0;
	imu_base_z_mg = c->fz;
	app_log((UB*)"imu: baseline x %d mg, y %d mg, z %d mg\n",
		  c->x0, c->y0, c->fz);
}

/* Read the accelerometer (and every 5th cycle the magnetometer) and
   low-pass filter it.  FALSE when the read failed. */
LOCAL BOOL imu_read(IMU_CTX *c)
{
	LSM303_MAG	mag;

	if (drv_lsm303_read_acc(&c->a) != E_OK) {
		imu_reads_fail++;
		if (++c->fails == I2C_FAIL_LIMIT) {
			app_log((UB*)"imu: I2C read failing\n");
			app_report_fault(FLT_IMU);
		}
		return FALSE;
	}
	c->fails = 0;
	imu_reads_ok++;
	imu_acc_raw[0] = c->a.x;
	imu_acc_raw[1] = c->a.y;
	imu_acc_raw[2] = c->a.z;
	if ((c->tick % 5U) == 0U) {
		/* heading cross-check only */
		if (drv_lsm303_read_mag(&mag) == E_OK) {
			imu_mag_raw[0] = mag.x;
			imu_mag_raw[1] = mag.y;
			imu_mag_raw[2] = mag.z;
		}
	}

	/* Low-pass filter (wide 32-bit internally, design 10.2).
	   app_asr: arithmetic shift right without shifting a signed value
	   (Barr-C), same result as the >> it replaces. */
	c->fx += app_asr(c->a.x - c->fx, LPF_SHIFT);
	c->fy += app_asr(c->a.y - c->fy, LPF_SHIFT);
	c->fz += app_asr(c->a.z - c->fz, LPF_SHIFT);
	imu_acc_filt[0] = c->fx;	/* dashboard only (is11/car/imu) */
	imu_acc_filt[1] = c->fy;
	imu_acc_filt[2] = c->fz;
	c->mag_mg = (W)hump_isqrt((UW)((c->fx * c->fx) + (c->fy * c->fy) +
				       (c->fz * c->fz)));
	return TRUE;
}

/* 13 Nov: LIFTED or OVERTURNED -> stop the motors (lift.h).  Raw
   reading for the lift push, filtered one for the tilt. */
LOCAL void imu_lift_step(IMU_CTX *c)
{
	W		raw[3];
	W		filt[3];
	LIFT_RESULT	lr;

	raw[0] = c->a.x;	raw[1] = c->a.y;	raw[2] = c->a.z;
	filt[0] = c->fx;	filt[1] = c->fy;	filt[2] = c->fz;
	lr = lift_update(&lift, raw, filt,
			 (hump.state != HS_LEVEL) ? TRUE : FALSE);
	imu_tilt_cos1000 = lift.last_cos1000;
	imu_up_mg = lift.last_up_mg;
	imu_lift_now = (UW)lr;
	if (c->holdoff > 0U) {
		c->holdoff--;
	} else if ((lr != LIFT_NONE) && !car_driving()) {
		/* 13 Nov: motors are not running, nothing to stop.  Picking
		   the car up to place it, carrying it or pressing its buttons
		   no longer latches a fault. */
		c->holdoff = LIFT_HOLDOFF_TICKS;
	} else if (lr != LIFT_NONE) {
		c->holdoff = LIFT_HOLDOFF_TICKS;
		imu_lift_count++;
		app_report_fault(FLT_LIFTED);	/* ctrl_task stops the motors */
		send_terrain(TR_LIFTED, ME_LIFTED, 0);
		if (lr == LIFT_LIFTED) {
			app_log((UB*)"imu: LIFTED (pushed up %d mg): "
				  "motors stop\n", lift.last_up_mg);
		} else {
			app_log((UB*)"imu: OVERTURNED / tilted (cos %d/1000 "
				  "from level): motors stop\n",
				  lift.last_cos1000);
		}
	} else if ((lift.last_up_mg > 80) && ((c->tick % 25U) == 0U) &&
		   car_driving()) {
		/* 16 Nov: near miss, so a lift that did not stop the car
		   shows how hard it was pushed up */
		app_log((UB*)"imu: push up %d mg (lift needs > %d)\n",
			lift.last_up_mg, LIFT_UP_MG);
	} else {
		/* nothing */
	}
}

/* Tilt -> dh, gated while the speed changes, then the hump detector */
LOCAL void imu_hump_step(IMU_CTX *c)
{
	H		v_now = motion_speed_avg();
	W		a_fwd = IMU_FWD_SIGN * (c->fx - c->x0);
	HUMP_EVENT	he;

	c->dh = hump_dh_um(a_fwd, c->mag_mg);
	if (app_abs(v_now - c->v_prev) > GATE_DV_MM_S) {
		c->gate = GATE_HOLD_TICKS;
	} else if (c->gate > 0U) {
		c->gate--;
	} else {
		c->dh_held = c->dh;
	}
	c->v_prev = v_now;

	he = hump_update(&hump, c->dh_held, motion_odo_path_01mm, &c->peak);
	switch (he) {
	case HE_START:
		(void)tk_set_flg(flg_sys, EVF_HUMP_ACTIVE);
		send_terrain(TR_HUMP_START, ME_CLIMBING, 0);
		break;
	case HE_END:
		(void)tk_clr_flg(flg_sys, ~EVF_HUMP_ACTIVE);
		c->humps++;
		if ((c->peak / 100) > c->run_peak_01mm) {
			c->run_peak_01mm = c->peak / 100;
		}
		send_terrain(TR_HUMP_END, ME_DESCENDING, c->peak);
		app_log((UB*)"imu: hump peak %d.%d mm\n", c->peak / 1000,
			  (c->peak % 1000) / 100);
		break;
	case HE_DISCARD:
		(void)tk_clr_flg(flg_sys, ~EVF_HUMP_ACTIVE);
		send_terrain(TR_HUMP_DISCARD, ME_CRUISING, 0);
		break;
	default:
		break;
	}
}

/* Motion-event classification for telemetry */
LOCAL void imu_motion_event(const IMU_CTX *c)
{
	H			v_l = motion_speed_left();
	H			v_r = motion_speed_right();
	MOTION_EVENT_CLASS	ev;

	if (c->holdoff > (LIFT_HOLDOFF_TICKS - 5U)) {
		ev = ME_LIFTED;
	} else if (hump.state == HS_CLIMBING) {
		ev = ME_CLIMBING;
	} else if ((hump.state == HS_DESCENDING) ||
		   (hump.state == HS_REAR_CROSSING)) {
		ev = ME_DESCENDING;
	} else if ((app_abs(v_l) < STATIONARY_MM_S) &&
		   (app_abs(v_r) < STATIONARY_MM_S)) {
		ev = ME_STATIONARY;
	} else if (app_abs(v_r - v_l) > TURN_DIFF_MM_S) {
		/* turn rate = (vR - vL) / track: no gyroscope on the DLHC */
		ev = ME_TURNING;
	} else if (c->dh != c->dh_held) {
		ev = ME_ACCELERATING;
	} else {
		ev = ME_CRUISING;
	}
	imu_status_word = ((UW)ev & 0x7U) |
			  (((c->humps > 15U) ? 15U : c->humps) << 3) |
			  (((UW)c->run_peak_01mm & 0xFFFFU) << 8);
}

EXPORT void imu_task(INT stacd, void *exinf)
{
	IMU_CTX	c = {0};

	(void)stacd; (void)exinf;

	c.fz = 1000;
	wait_for_imu();
	hump_reset(&hump);
	imu_learn_level(&c);
	(void)tk_set_flg(flg_sys, EVF_IMU_READY);

	while (1) {
		tk_slp_tsk(TMO_FEVR);
		(void)tk_can_wup(TSK_SELF);
		c.tick++;

		if (!imu_read(&c)) {
			continue;
		}
		imu_lift_step(&c);
		imu_hump_step(&c);
		imu_motion_event(&c);
	}
}
