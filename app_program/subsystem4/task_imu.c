/*
 *	task_imu.c
 *	Subsystem 4 -- imu_task (BASIC VERSION), woken every 10 ms.
 *	Statechart 6.4 (simplified): LEVEL, CLIMBING, DESCENDING.
 */

#include "subsystem4.h"
#include "drv_imu.h"
#include "imu_math.h"
#include "../subsystem2/subsystem2.h"	/* odometer */
#include "../subsystem5/subsystem5.h"	/* Terrain Status Queue, hump bit */

typedef enum {
	HS_LEVEL = 0,
	HS_CLIMBING,
	HS_DESCENDING
} HUMP_STATE;

EXPORT volatile BOOL	s4_ok = FALSE;
EXPORT volatile UW	s4_samples = 0U;
EXPORT volatile W	s4_sin_x1000 = 0;
EXPORT volatile W	s4_peak_mm = 0;
EXPORT volatile W	s4_last_mag_mg = 0;

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

/* Wait here for a cyclic wake-up (10 ms). */
LOCAL void s4_wait_tick(void)
{
	(void)tk_slp_tsk(TMO_FEVR);
	(void)tk_can_wup(TSK_SELF);
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
	W		climb_start_mm = 0;
	W		hump_peak = 0;
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
		s4_post(TER_IMU_FAULT, (W)er);
		(void)tk_slp_tsk(TMO_FEVR);
	}
	s4_ok = TRUE;

	while (1) {
		W	mag;
		W	fwd;
		W	sinv;
		W	dev;
		W	h_mm;

		s4_wait_tick();
		er = imu_read_accel(&a);
		if (er < E_OK) {
			if (++err_run == (UINT)S4_FAULT_READS) {
				s4_post(TER_IMU_FAULT, (W)er);
			}
			continue;
		}
		err_run = 0U;
		s4_samples++;

		mag = (W)imu_isqrt((UW)((a.x_mg * a.x_mg) + (a.y_mg * a.y_mg) + (a.z_mg * a.z_mg)));
		s4_last_mag_mg = mag;
		fwd = ((S4_FWD_AXIS == 0) ? a.x_mg : a.y_mg) * S4_FWD_SIGN;

		/* level baseline: first second, car standing still */
		if (base_n < (UINT)S4_BASELINE_N) {
			base_sum += fwd;
			base_n++;
			if (base_n == (UINT)S4_BASELINE_N) {
				base_fwd = base_sum / (W)S4_BASELINE_N;
				tm_printf((UB *)"[S4] level baseline: fwd = %d mg\n", (INT)base_fwd);
			}
			continue;
		}

		/* sin(tilt) x 1000, then a light low-pass (alpha = 1/4) */
		sinv = (mag > 0) ? (((fwd - base_fwd) * 1000) / mag) : 0;
		filt += (sinv - filt) / 4;
		s4_sin_x1000 = filt;
		h_mm = (S4_WHEELBASE_MM * filt) / 1000;	/* front minus rear */

		/* impact */
		dev = mag - 1000;
		if (dev < 0) {
			dev = -dev;
		}
		if ((dev > ((hs == HS_LEVEL) ? S4_IMPACT_MG : S4_IMPACT_HUMP_MG)) &&
		    ((W)(s5_now_ms() - impact_quiet_until) >= 0)) {
			s4_post(TER_IMPACT, dev);
			impact_quiet_until = s5_now_ms() + (UW)S4_IMPACT_HOLDOFF_MS;
		}

		switch (hs) {
		case HS_LEVEL:
			enter_n = (filt > S4_SIN_ENTER) ? (enter_n + 1U) : 0U;
			if (enter_n >= (UINT)S4_ENTER_N) {
				hs = HS_CLIMBING;
				climb_start_mm = s2_distance_mm;
				hump_peak = h_mm;
				level_n = 0U;
				s4_hump_bit(TRUE);
				s4_post(TER_HUMP_START, 0);
			}
			break;
		case HS_CLIMBING:
			if (h_mm > hump_peak) {
				hump_peak = h_mm;
			}
			if (filt < -S4_SIN_ENTER) {
				hs = HS_DESCENDING;
				level_n = 0U;
			} else if ((s2_distance_mm - climb_start_mm) > S4_CLIMB_MAX_MM) {
				hs = HS_LEVEL;		/* ramp or bad reading */
				s4_hump_bit(FALSE);
				s4_post(TER_HUMP_DISCARDED, hump_peak);
			} else {
				/* false trigger: level again without descending */
				level_n = ((filt < S4_SIN_LEVEL) && (filt > -S4_SIN_LEVEL)) ? (level_n + 1U) : 0U;
				if (level_n >= (UINT)S4_LEVEL_N) {
					hs = HS_LEVEL;
					s4_hump_bit(FALSE);
					s4_post(TER_HUMP_DISCARDED, hump_peak);
				}
			}
			break;
		case HS_DESCENDING:
		default:
			level_n = ((filt < S4_SIN_LEVEL) && (filt > -S4_SIN_LEVEL)) ? (level_n + 1U) : 0U;
			if (level_n >= (UINT)S4_LEVEL_N) {
				hs = HS_LEVEL;
				enter_n = 0U;
				if (hump_peak > s4_peak_mm) {
					s4_peak_mm = hump_peak;
				}
				s4_hump_bit(FALSE);
				s4_post(TER_HUMP_END, hump_peak);
				tm_printf((UB *)"[S4] hump: peak about %d mm (run max %d mm)\n",
					  (INT)hump_peak, (INT)s4_peak_mm);
			}
			break;
		}

		if (S4_PRINT_ENABLE != 0) {
			if (++print_div >= 50U) {
				print_div = 0U;
				tm_printf((UB *)"[S4] |a|=%d mg sin=%d h=%d mm state=%d\n",
					  (INT)mag, (INT)filt, (INT)h_mm, (INT)hs);
			}
		}
	}
}
