/*
 *	task_scan.c
 *	Subsystem 5 -- scan_task (BASIC VERSION), woken every 60 ms.
 *
 *	The servo stays at 90 degrees (straight ahead).  One polled ping per
 *	cycle.  Two readings inside the stop distance -> OBS_ALERT; three
 *	readings beyond S5_CLEAR_MM (or nothing within ~1 m) -> OBS_CLEAR.
 *	Readings are ignored while the hump bit is set (the sensor may be
 *	looking at the hump).  ECHO that never rises three times in a row is
 *	reported as a sensor fault; an echo longer than ~6 ms just means
 *	nothing is within ~1 m.
 *	The complete version adds the ECHO interrupt and the coarse/fine scan.
 */

#include "subsystem5.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"

EXPORT volatile BOOL	s5_scan_ready = FALSE;
EXPORT volatile W	s5_last_mm = -1;
EXPORT volatile UW	s5_ping_count = 0U;

LOCAL void s5_post_obstacle(OBSTACLE_KIND kind, W mm)
{
	OBSTACLE_MSG	msg;

	msg.kind = kind;
	msg.distance_mm = mm;
	(void)s5_post(mbf_obstacle, &msg, (INT)sizeof(msg), FLG_OBSTACLE);
}

EXPORT void scan_task(INT stacd, void *exinf)
{
	UINT	near_n = 0U;
	UINT	clear_n = 0U;
	UINT	no_echo_n = 0U;
	BOOL	alerted = FALSE;

	(void)stacd;
	(void)exinf;

	(void)us_init();
	(void)servo_init();		/* centres the servo at 90 degrees */
	s5_scan_ready = TRUE;

	while (1) {
		UW		echo_us = 0U;
		UW		mm = 0U;
		S5_PING_RESULT	res;
		UINT		flgptn = 0U;

		(void)tk_slp_tsk(TMO_FEVR);
		(void)tk_can_wup(TSK_SELF);

		res = us_ping(&echo_us, &mm);
		s5_ping_count++;

		{
			T_RFLG	rflg;
			if (tk_ref_flg(flg_ctrl, &rflg) == E_OK) {
				flgptn = rflg.flgptn;
			}
		}
		if ((flgptn & FLG_HUMP_ACTIVE) != 0U) {
			near_n = 0U;
			continue;	/* the sensor may be seeing the hump */
		}

		if (res == S5_PING_NO_ECHO) {
			if (++no_echo_n == (UINT)S5_NO_ECHO_FAULT_N) {
				s5_post_obstacle(OBS_SENSOR_FAULT, 0);
			}
			continue;
		}
		no_echo_n = 0U;
		if (res == S5_PING_BUSY) {
			continue;
		}

		s5_last_mm = (res == S5_PING_OK) ? (W)mm : -1;

		if ((res == S5_PING_OK) && ((W)mm <= S5_STOP_MM)) {
			clear_n = 0U;
			if ((++near_n >= (UINT)S5_CONFIRM_N) && (alerted == FALSE)) {
				alerted = TRUE;
				s5_post_obstacle(OBS_ALERT, (W)mm);
				if (S5_PRINT_ENABLE != 0) {
					tm_printf((UB *)"[S5] obstacle at %d mm\n", (INT)mm);
				}
			}
		} else if ((res == S5_PING_OUT_OF_RANGE) || ((W)mm > S5_CLEAR_MM)) {
			near_n = 0U;
			if ((++clear_n >= (UINT)S5_CLEAR_N) && (alerted == TRUE)) {
				alerted = FALSE;
				s5_post_obstacle(OBS_CLEAR, s5_last_mm);
			}
		} else {
			near_n = 0U;	/* between stop and clear distance */
		}
	}
}
