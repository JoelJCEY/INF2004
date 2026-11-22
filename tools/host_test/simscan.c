/* Runs the real scan_task (task_scan.c + scan_profile.c) against a
   scripted mission, servo, ultrasonic and odometry, and prints every
   output, so two builds of the task can be compared line by line. */
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "subsystem5/subsystem5.h"
#include "subsystem5/drv_ultrasonic.h"
#include "subsystem5/drv_servo.h"
#include "subsystem2/subsystem2.h"

ID flg_sys = 3, mbf_obstacle = 4;
volatile UW motion_odo_path_01mm, motion_odo_speed;
static UW now_ms, alert_ms;
static UINT servo = 12, flags;
static int phase;	/* 0 follow, 1 bypass after alert, 2 stopped */
static double car_x;	/* mm along the track */
static jmp_buf done;
static unsigned seed = 777U;

static void speeds(int v) { motion_odo_speed = ((UW)(v + 32768) << 16) | (UW)(v + 32768); }
UW app_time_ms(void) { return now_ms; }
UW hump_isqrt(UW v) { return (UW)sqrt((double)v); }
void drv_servo_init(void) { }
void drv_servo_set_idx(UINT idx) { servo = idx; printf("  servo %u\n", idx); }
ER drv_us_init(void) { return E_OK; }
US_RESULT drv_us_ping(UW *mm)
{
	/* obstacle: box at x 900..1000 mm, sideways -40..+90 mm (+ = right) */
	double deg = 30.0 + 5.0 * servo, best = 1e9;
	int i;
	seed = seed * 1103515245U + 12345U;
	if (((seed >> 16) % 17U) == 0U) { return US_NO_ECHO; }	/* stray miss */
	for (i = -40; i <= 90; i += 5) {
		double ahead = 900.0 - car_x - (phase == 1 && now_ms > alert_ms + 1500U ? -300.0 : 0.0);
		double side = i + (phase == 1 && now_ms > alert_ms + 1500U ? 200.0 : 0.0);
		double a = atan2(ahead, side) * 180.0 / 3.14159265;
		double d = sqrt(ahead * ahead + side * side);
		if (ahead > 0 && fabs(a - deg) < 8.0 && d < best) { best = d; }
	}
	if (phase == 1 && now_ms > alert_ms + 11500U && servo == 12) { best = 90; }	/* guard */
	if (best > 999.0) { return US_FAR; }
	*mm = (UW)best;
	return US_OK;
}
ER tk_dly_tsk(RELTIM t)
{
	now_ms += t;
	if (phase == 0 || (phase == 1 && now_ms > alert_ms + 1500U && now_ms < alert_ms + 9000U)) {
		car_x += 0.06 * t;
		speeds(60);
	} else {
		speeds(0);
	}
	motion_odo_path_01mm = (UW)(car_x * 10.0);
	printf("t%u x%d st %08x pass %u/%u sweep %u bench %08x\n", (unsigned)now_ms, (int)car_x,
	       (unsigned)scan_status_word, (unsigned)scan_pass_seq, (unsigned)scan_pass_extra_mm,
	       (unsigned)scan_sweep_seq, (unsigned)scan_bench_word);
	if (phase == 1 && now_ms > alert_ms + 1500U && scan_pass_watch == 0U && now_ms < alert_ms + 9000U) {
		scan_pass_watch = 9U;	/* idx 8 = 70 deg + 1 */
	}
	if (phase == 1 && now_ms >= alert_ms + 9000U) { scan_pass_watch = 0; }
	if (phase == 1 && now_ms >= alert_ms + 9500U && now_ms < alert_ms + 9600U) { scan_rescan_request++; }
	if (phase == 1 && now_ms >= alert_ms + 13000U) { phase = 2; scan_bench_on = 1; }
	if (now_ms > alert_ms + 15000U && phase == 2) { longjmp(done, 1); }
	if (now_ms > 60000U) { longjmp(done, 1); }
	return E_OK;
}
ER tk_ref_flg(ID id, T_RFLG *r)
{
	(void)id;
	r->flgptn = ((now_ms >= 1500U) && (now_ms < 2600U)) ? EVF_HUMP_ACTIVE : 0U;
	return E_OK;
}
ER tk_set_flg(ID id, UINT p) { (void)id; flags |= p; printf("  set_flg %x\n", p); return E_OK; }
ER tk_snd_mbf(ID id, const void *m, INT sz, TMO tmo)
{
	const OBSTACLE_MSG *o = m;
	(void)id; (void)sz; (void)tmo;
	printf("  obstacle kind %u side %u idx %u w %u c %u off %u depth %u\n", o->kind, o->side,
	       o->servo_idx, o->width_mm, o->closest_mm, o->offset_mm, o->depth_mm);
	if (o->kind == OB_ALERT && phase == 0) { phase = 1; alert_ms = now_ms; }
	return E_OK;
}
void ctrl_get_mission(MISSION_STATE *out)
{
	memset(out, 0, sizeof(*out));
	out->state = (now_ms < 1000U) ? VS_READY : (phase == 0) ? VS_LINE_FOLLOWING :
		     (phase == 1) ? VS_OBSTACLE_BYPASS : VS_STOPPED;
}

int main(void)
{
	setvbuf(stdout, NULL, _IONBF, 0);
	speeds(0);
	if (!setjmp(done)) { scan_task(0, NULL); }
	printf("end t%u\n", (unsigned)now_ms);
	return 0;
}
