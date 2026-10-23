/*
 *	task_imu.c
 *	Subsystem 4 -- imu_task (COMPLETE VERSION), woken every 10 ms.
 *	Statechart 6.4: LEVEL, CLIMBING, DESCENDING, REAR WHEELS CROSSING
 *	(+ DISCARD).  See subsystem4.h for the method.
 */

#include "subsystem4.h"
#include "drv_imu.h"
#include "imu_math.h"
#include "../subsystem2/subsystem2.h"	/* odometer, speed, acceleration */
#include "../subsystem5/subsystem5.h"	/* Terrain Status Queue, hump bit */

typedef enum {
	HS_LEVEL = 0,
	HS_CLIMBING,
	HS_DESCENDING,
	HS_REAR_CROSSING
} HUMP_STATE;

EXPORT volatile BOOL		s4_ok = FALSE;
EXPORT volatile UW		s4_samples = 0U;
EXPORT volatile W		s4_sin_x1000 = 0;
EXPORT volatile W		s4_peak_mm = 0;
EXPORT volatile W		s4_last_mag_mg = 0;
EXPORT volatile MOTION_EVENT	s4_motion_event = ME_STATIONARY;
EXPORT volatile W		s4_turn_rate_dps = 0;

/* Height under the front wheels, one entry per 5 mm of travel since the
 * hump started (mm).  Index wraps; only the last 640 mm are kept. */
LOCAL W		prof[S4_PROFILE_STEPS];
LOCAL W		prof_steps = 0;		/* steps recorded this hump */

LOCAL void s4_post(TERRAIN_KIND kind, W value)
{
	TERRAIN_MSG	msg;

	msg.kind = kind;
	msg.value = value;
	(void)s5_post(mbf_terrain, &msg, (INT)sizeof(msg), FLG_TERRAIN);
}

LOCAL void s4_hump_bit(BOOL on)
{
	if (on == TRUE) {
		(void)tk_set_flg(flg_ctrl, FLG_HUMP_ACTIVE);
	} else {
		(void)tk_clr_flg(flg_ctrl, ~FLG_HUMP_ACTIVE);
	}
}

LOCAL void s4_wait_tick(void)
{
	(void)tk_slp_tsk(TMO_FEVR);
	(void)tk_can_wup(TSK_SELF);
}

/* Rear wheels are WHEELBASE behind the front ones on the same path. */
LOCAL W s4_h_rear(W step)
{
	W	back = step - (S4_WHEELBASE_MM / S4_STEP_MM);

	if ((back < 0) || ((prof_steps - back) >= S4_PROFILE_STEPS)) {
		return 0;	/* rear still on the flat before the hump */
	}
	return prof[back % S4_PROFILE_STEPS];
}

/* Add profile steps up to the current odometer. */
LOCAL W s4_profile_update(W start_mm, W sinv)
{
	W	target = (s2_distance_mm - start_mm) / S4_STEP_MM;
	W	h = 0;

	while (prof_steps <= target) {
		h = s4_h_rear(prof_steps) + ((S4_WHEELBASE_MM * sinv) / 1000);
		prof[prof_steps % S4_PROFILE_STEPS] = h;
		prof_steps++;
	}
	return (prof_steps > 0) ? prof[(prof_steps - 1) % S4_PROFILE_STEPS] : 0;
}

LOCAL MOTION_EVENT s4_classify(HUMP_STATE hs, BOOL impact)
{
	W	v = (s2_speed_l_mm_s + s2_speed_r_mm_s) / 2;
	W	acc = s2_accel_mm_s2;
	W	tr = s4_turn_rate_dps;

	if (impact == TRUE) {
		return ME_IMPACT;
	}
	if (hs == HS_CLIMBING) {
		return ME_CLIMBING;
	}
	if ((hs == HS_DESCENDING) || (hs == HS_REAR_CROSSING)) {
		return ME_DESCENDING;
	}
	if ((v < 20) && (v > -20)) {
		return ME_STATIONARY;
	}
	if ((tr > 30) || (tr < -30)) {
		return ME_TURNING;
	}
	if (acc > S4_ACCEL_GATE_MM_S2) {
		return ME_ACCELERATING;
	}
	if (acc < -S4_ACCEL_GATE_MM_S2) {
		return ME_BRAKING;
	}
	return ME_CRUISING;
}

EXPORT void imu_task(INT stacd, void *exinf)
{
	IMU_ACCEL	a;
	BOOL		ok = FALSE;
	ER		er;
	W		base_fwd = 0;
	W		base_sum = 0;
	UINT		base_n = 0U;
	W		filt = 0;
	HUMP_STATE	hs = HS_LEVEL;
	UINT		enter_n = 0U;
	UINT		level_n = 0U;
	W		start_mm = 0;
	W		peak = 0;
	BOOL		peak_confirmed = FALSE;
	W		descend_end_mm = 0;
	UINT		err_run = 0U;
	UW		impact_quiet_until = 0U;
	UINT		print_div = 0U;

	(void)stacd;
	(void)exinf;

	s4_wait_tick();
	er = imu_open();
	if (er >= E_OK) {
		er = imu_check_id(&ok);
	}
	if ((er >= E_OK) && (ok == TRUE)) {
		er = imu_config_accel(&ok);
	}
	if ((er < E_OK) || (ok == FALSE)) {
		tm_printf((UB *)"[S4] IMU not found or not configured (err %d)\n", (INT)er);
		s5_raise_fault(FAULT_IMU);
		s4_post(TER_IMU_FAULT, (W)er);
		(void)tk_slp_tsk(TMO_FEVR);
	}
	s4_ok = TRUE;

	while (1) {
		W	mag;
		W	fwd;
		W	sinv;
		W	dev;
		W	h;
		BOOL	impact = FALSE;
		BOOL	gated;

		s4_wait_tick();
		er = imu_read_accel(&a);
		if (er < E_OK) {
			if (++err_run == (UINT)S4_FAULT_READS) {
				s5_raise_fault(FAULT_IMU);
				s4_post(TER_IMU_FAULT, (W)er);
			}
			continue;
		}
		err_run = 0U;
		s4_samples++;

		mag = (W)imu_isqrt((UW)((a.x_mg * a.x_mg) + (a.y_mg * a.y_mg) + (a.z_mg * a.z_mg)));
		s4_last_mag_mg = mag;
		fwd = ((S4_FWD_AXIS == 0) ? a.x_mg : a.y_mg) * S4_FWD_SIGN;

		/* turn rate from the encoders: (vR - vL) / track, in deg/s */
		s4_turn_rate_dps = ((s2_speed_r_mm_s - s2_speed_l_mm_s) * 573) /
				   (S2_TRACK_MM * 10);

		if (base_n < (UINT)S4_BASELINE_N) {
			base_sum += fwd;
			base_n++;
			if (base_n == (UINT)S4_BASELINE_N) {
				base_fwd = base_sum / (W)S4_BASELINE_N;
				tm_printf((UB *)"[S4] level baseline: fwd = %d mg\n", (INT)base_fwd);
			}
			continue;
		}

		/* tilt, frozen while accelerating or braking */
		gated = ((s2_accel_mm_s2 > S4_ACCEL_GATE_MM_S2) ||
			 (s2_accel_mm_s2 < -S4_ACCEL_GATE_MM_S2)) ? TRUE : FALSE;
		if (gated == FALSE) {
			sinv = (mag > 0) ? (((fwd - base_fwd) * 1000) / mag) : 0;
			filt += (sinv - filt) / 4;
		}
		s4_sin_x1000 = filt;

		dev = mag - 1000;
		if (dev < 0) {
			dev = -dev;
		}
		if ((dev > ((hs == HS_LEVEL) ? S4_IMPACT_MG : S4_IMPACT_HUMP_MG)) &&
		    ((W)(s5_now_ms() - impact_quiet_until) >= 0)) {
			impact = TRUE;
			s4_post(TER_IMPACT, dev);
			impact_quiet_until = s5_now_ms() + (UW)S4_IMPACT_HOLDOFF_MS;
		}

		switch (hs) {
		case HS_LEVEL:
			enter_n = (filt > S4_SIN_ENTER) ? (enter_n + 1U) : 0U;
			if (enter_n >= (UINT)S4_ENTER_N) {
				hs = HS_CLIMBING;
				start_mm = s2_distance_mm;
				prof_steps = 0;
				peak = 0;
				peak_confirmed = FALSE;
				level_n = 0U;
				s4_hump_bit(TRUE);
				s4_post(TER_HUMP_START, 0);
			}
			break;
		case HS_CLIMBING:
			h = s4_profile_update(start_mm, filt);
			if (h > peak) {
				peak = h;
			}
			if (filt < -S4_SIN_ENTER) {
				hs = HS_DESCENDING;
				level_n = 0U;
			} else if ((s2_distance_mm - start_mm) > S4_CLIMB_MAX_MM) {
				hs = HS_LEVEL;
				s4_hump_bit(FALSE);
				s4_post(TER_HUMP_DISCARDED, peak);
			} else {
				/* level-return exit: a false trigger with no peak */
				level_n = ((filt < S4_SIN_LEVEL) && (filt > -S4_SIN_LEVEL)) ? (level_n + 1U) : 0U;
				if ((level_n >= (UINT)S4_LEVEL_N) && (peak < S4_PEAK_DROP_MM)) {
					hs = HS_LEVEL;
					s4_hump_bit(FALSE);
					s4_post(TER_HUMP_DISCARDED, peak);
				}
			}
			break;
		case HS_DESCENDING:
			h = s4_profile_update(start_mm, filt);
			if (h > peak) {
				peak = h;
			} else if ((peak - h) >= S4_PEAK_DROP_MM) {
				peak_confirmed = TRUE;	/* dropped 3 mm: real peak */
			} else {
				;
			}
			level_n = ((filt < S4_SIN_LEVEL) && (filt > -S4_SIN_LEVEL)) ? (level_n + 1U) : 0U;
			if (level_n >= (UINT)S4_LEVEL_N) {
				hs = HS_REAR_CROSSING;	/* front is down; rear still on it */
				descend_end_mm = s2_distance_mm;
			}
			break;
		case HS_REAR_CROSSING:
		default:
			h = s4_profile_update(start_mm, filt);
			if ((s2_distance_mm - descend_end_mm) >= S4_WHEELBASE_MM) {
				hs = HS_LEVEL;
				enter_n = 0U;
				s4_hump_bit(FALSE);
				if (peak_confirmed == TRUE) {
					if (peak > s4_peak_mm) {
						s4_peak_mm = peak;
					}
					s4_post(TER_HUMP_END, peak);
					tm_printf((UB *)"[S4] hump: peak about %d mm (run max %d mm)\n",
						  (INT)peak, (INT)s4_peak_mm);
				} else {
					s4_post(TER_HUMP_DISCARDED, peak);
				}
			}
			break;
		}
		s4_motion_event = s4_classify(hs, impact);

		if (S4_PRINT_ENABLE != 0) {
			if (++print_div >= 50U) {
				print_div = 0U;
				tm_printf((UB *)"[S4] |a|=%d sin=%d state=%d event=%d turn=%d dps\n",
					  (INT)mag, (INT)filt, (INT)hs, (INT)s4_motion_event,
					  (INT)s4_turn_rate_dps);
			}
		}
	}
}
