/* Runs the real imu_task (task_imu.c + hump.c + lift.c) against a scripted
   accelerometer and odometry, and prints everything it outputs each cycle,
   so two builds of the task can be compared line by line.
     simimu flat | simimu move   ("move": car not still while learning level) */
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "subsystem4/subsystem4.h"
#include "subsystem4/drv_lsm303.h"
#include "subsystem2/subsystem2.h"
#include "subsystem5/subsystem5.h"
#include "subsystem5/ctrl_logic.h"

ID tid_imu = 1, mbf_terrain = 2, flg_sys = 3;
volatile UW motion_odo_path_01mm, motion_odo_speed;
static unsigned tick;
static int moving_baseline;
static UINT flags;
static jmp_buf done;
static unsigned seed = 12345U;

static int noise(int amp)	/* deterministic, includes negatives */
{
	seed = seed * 1103515245U + 12345U;
	return (int)((seed >> 16) % (unsigned)(2 * amp + 1)) - amp;
}
static void speeds(int l, int r)
{
	motion_odo_speed = ((UW)(l + 32768) << 16) | (UW)(r + 32768);
}

ER drv_lsm303_init(void) { return E_OK; }
ER drv_lsm303_read_mag(LSM303_MAG *m) { m->x = 100; m->y = -50; m->z = 300; return E_OK; }
ER drv_lsm303_read_acc(LSM303_ACC *a)
{
	unsigned t = tick;
	a->x = noise(15); a->y = noise(15); a->z = 1000 + noise(15);
	if (moving_baseline && t < 40) { a->x += noise(200); }
	if (t >= 100 && t < 180) { a->x += 250; }		/* climbing   */
	if (t >= 180 && t < 260) { a->x -= 250; }		/* descending */
	if (t >= 450 && t < 458) { a->z += 450; }		/* lifted     */
	if (t >= 500 && t < 520) { a->z = 300; a->y = 900; }	/* tilted     */
	if (t >= 540 && t < 553) { return E_IO; }		/* I2C fails  */
	return E_OK;
}
ER tk_slp_tsk(TMO t)
{
	(void)t;
	printf("t%u st %08x lift %u cos %d up %d f %d %d %d flg %x\n", tick,
	       (unsigned)imu_status_word, (unsigned)imu_lift_now,
	       (int)imu_tilt_cos1000, (int)imu_up_mg, (int)imu_acc_filt[0],
	       (int)imu_acc_filt[1], (int)imu_acc_filt[2], flags);
	if (++tick > 600U) { longjmp(done, 1); }
	motion_odo_path_01mm += 12U;			/* 60 mm/s */
	if (tick >= 350 && tick < 360) { speeds(60 + (int)(tick - 350) * 8, 60 + (int)(tick - 350) * 8); }
	else if (tick >= 400 && tick < 420) { speeds(20, 120); }
	else if (tick >= 580) { speeds(0, 0); }
	else { speeds(60, 60); }
	return E_OK;
}
ER tk_dly_tsk(RELTIM t) { (void)t; return E_OK; }
ER tk_wup_tsk(ID id) { (void)id; return E_OK; }
INT tk_can_wup(ID id) { (void)id; return 0; }
ER tk_set_flg(ID id, UINT p) { (void)id; flags |= p; printf("  set_flg %x\n", p); return E_OK; }
ER tk_clr_flg(ID id, UINT p) { (void)id; flags &= p; printf("  clr_flg %x\n", p); return E_OK; }
ER tk_snd_mbf(ID id, const void *m, INT sz, TMO tmo)
{
	const TERRAIN_MSG *t = m;
	(void)id; (void)sz; (void)tmo;
	printf("  terrain kind %u ev %u peak %u path %u\n", t->kind, t->event, t->peak_01mm, (unsigned)t->path_mm);
	return E_OK;
}
void ctrl_get_mission(MISSION_STATE *out) { memset(out, 0, sizeof(*out)); out->state = VS_LINE_FOLLOWING; }
BOOL ctrl_is_active(VEHICLE_STATE s) { return s == VS_LINE_FOLLOWING; }

int main(int argc, char **argv)
{
	setvbuf(stdout, NULL, _IONBF, 0);
	moving_baseline = (argc > 1) && !strcmp(argv[1], "move");
	speeds(60, 60);
	if (!setjmp(done)) { imu_task(0, NULL); }
	printf("end\n");
	return 0;
}
