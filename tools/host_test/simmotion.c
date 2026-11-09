/* Host simulation: runs the REAL task_motion.c against a first-order
   motor + encoder model.  tk_slp_tsk() advances the plant by 20 ms. */
#include <setjmp.h>
#include <stdlib.h>
#include <math.h>
#include "subsystem2/subsystem2.h"
#include "subsystem2/motion_ctrl.h"
#include "subsystem3/subsystem3.h"
#include "subsystem5/subsystem5.h"

ID tid_motion, mbf_motion_cmd = 1, mbf_motion_status = 2, flg_sys = 3;
BOOL motion_encoders_ok = TRUE;
volatile UW line_latest;
static jmp_buf done;
static int ticks, max_ticks;
static double v[2], pos_um[2], frac[2]; static long cnt[2];
static int duty[2]; static int block_left; static int block_at = -1;
static MOTION_CMD q[8]; static int qn, qr;
static MOTION_STATUS last; static int got_status; static UINT flags;
static UW now_ms; static double gain[2] = {0.8, 0.8}; static int mark_at = -1; static double mark_pos[2];
static int step_at = -1, step_err = 0, step_show = 0, spin_at = -1;

void drv_motor_set(UINT m, INT d){ duty[m] = d; }
void drv_motor_brake_all(void){ duty[0] = duty[1] = 0; }
int32_t drv_encoder_read(uint32_t w){ return (int32_t)(w == 1 ? -cnt[1] : cnt[0]); } /* right inverted */
ER tk_slp_tsk(TMO t){ (void)t; int i;
  if (ticks++ >= max_ticks) longjmp(done, 1);
  if (block_at >= 0 && ticks >= block_at) block_left = 1;
  if (ticks == mark_at) { mark_pos[0] = pos_um[0]; mark_pos[1] = pos_um[1]; }
  if (step_at >= 0 && ticks >= step_at) {
    int e = (spin_at >= 0 && ticks >= spin_at) ? 1000 : step_err;
    line_latest = line_pack(e, LINE_F_VALID | (e == 1000 ? LINE_F_SATURATED : LINE_F_ON_LINE), now_ms + 20);
    if (step_show > 0 && ((ticks - step_at) < step_show || (spin_at >= 0 && ticks >= spin_at && ticks - spin_at < step_show)))
      printf("      t+%3d ms  err %4d  duty L %4d R %4d\n", (ticks - step_at) * 20, e, duty[0], duty[1]);
  }
  for (i = 0; i < 2; i++) {
    double d = duty[i], eff = (fabs(d) > 120) ? (d > 0 ? d - 120 : d + 120) : 0;
    double target = gain[i] * eff, tau = (d == 0) ? 0.03 : 0.1;
    if (i == 0 && block_left) { v[0] = 0; continue; }
    v[i] += (target - v[i]) * 0.02 / tau;
    pos_um[i] += v[i] * 20.0;           /* mm/s * 20 ms = um */
    frac[i] += v[i] * 20.0 / MOTION_UM_PER_COUNT;
    while (frac[i] >= 1) { cnt[i]++; frac[i] -= 1; }
    while (frac[i] <= -1) { cnt[i]--; frac[i] += 1; }
  }
  now_ms += 20; return 0; }
INT tk_can_wup(ID id){ (void)id; return 0; }
ER tk_wup_tsk(ID id){ (void)id; return 0; }
INT tk_rcv_mbf(ID id, void *m, TMO t){ (void)id; (void)t; if (qr == qn) return -1; *(MOTION_CMD*)m = q[qr++]; return sizeof(MOTION_CMD); }
ER tk_snd_mbf(ID id, const void *m, INT sz, TMO t){ (void)t; (void)sz;
  if (id == mbf_motion_cmd) { q[qn++] = *(const MOTION_CMD*)m; return 0; }
  last = *(const MOTION_STATUS*)m; got_status = 1;
  printf("    status ev=%d seq=%d at tick %d\n", last.event, last.seq, ticks); return 0; }
ER tk_set_flg(ID id, UINT p){ (void)id; flags |= p; return 0; }
UW app_time_us(void){ return now_ms * 1000; } UW app_time_ms(void){ return now_ms; }
/* unused kernel shims */
UINT gpio_get_val(UINT n){(void)n;return 0;}
ER tk_dly_tsk(RELTIM t){(void)t;return 0;} ER tk_ref_flg(ID i,T_RFLG*r){(void)i;r->flgptn=0;return 0;}
UW in_w(UW a){(void)a;return 0;} void out_w(UW a,UW b){(void)a;(void)b;} void set_w(UW a,UW b){(void)a;(void)b;} void clr_w(UW a,UW b){(void)a;(void)b;}
ER tk_def_int(UINT n,const T_DINT*d){(void)n;(void)d;return 0;} void EnableInt(UINT n,INT l){(void)n;(void)l;} void ClearInt(UINT n){(void)n;}

static void run(int n){ max_ticks = ticks + n; if (!setjmp(done)) motion_task(0, NULL); }
static void reset_plant(void){ v[0]=v[1]=pos_um[0]=pos_um[1]=0; }

int main(void){
  printf("um/count %d\n", MOTION_UM_PER_COUNT);
  printf("1) moveForward(1000)\n"); reset_plant(); moveForward(1000); run(1600);
  printf("   wheel travel L %.1f mm R %.1f mm (target 1000, NFR6 <= 30)\n", pos_um[0]/1000, pos_um[1]/1000);
  printf("2) turnLeft(90)\n"); reset_plant(); turnLeft(90); run(600);
  { double th = (pos_um[1]-pos_um[0])/1000.0/MOTION_TRACK_MM*180/M_PI;
    printf("   heading change %.1f deg (target 90, NFR6 <= 5)\n", th); }
  printf("3) moveBackward(300)\n"); reset_plant(); moveBackward(300); run(500);
  printf("   wheel travel L %.1f mm R %.1f mm\n", pos_um[0]/1000, pos_um[1]/1000);
  printf("4) follow line, left wheel blocked after 1 s\n"); flags = 0;
  line_latest = line_pack(0, LINE_F_VALID|LINE_F_ON_LINE, 0);
  motion_follow_line(300); block_at = ticks + 50; run(110);
  printf("   FLT_STALL %s, mode %u (5 = LOCKED)\n",
         (flags & FLT_STALL) ? "set" : "NOT set", motion_mode_word);
  printf("   (stall reported %s)\n", got_status && last.event == MS_STALL ? "as MS_STALL" : "NOT");
  printf("5) follow a straight line (error 0), LEFT motor 15%% weaker\n");
  block_at = -1; block_left = 0; qn = qr = 0; flags = 0; reset_plant();
  gain[0] = 0.68; gain[1] = 0.8;
  line_latest = line_pack(0, LINE_F_VALID|LINE_F_ON_LINE, 0);
  motion_reset(); motion_follow_line(300);
  mark_at = ticks + 150;                       /* measure the last 3 s of 6 s */
  run(300);
  printf("   last 3 s: wheel travel L %.1f mm R %.1f mm, difference %.1f mm (%.2f%%)\n",
    (pos_um[0]-mark_pos[0])/1000, (pos_um[1]-mark_pos[1])/1000,
    (pos_um[0]-mark_pos[0]-(pos_um[1]-mark_pos[1]))/1000,
    100.0*(pos_um[0]-mark_pos[0]-(pos_um[1]-mark_pos[1]))/(pos_um[1]-mark_pos[1]));
  printf("   speed now L %.0f R %.0f mm/s\n", v[0], v[1]);
  printf("6) gradual steering: error 0 -> +400 (line to the left), then line lost (spin)\n");
  qn = qr = 0; reset_plant(); gain[0] = gain[1] = 0.8;
  line_latest = line_pack(0, LINE_F_VALID|LINE_F_ON_LINE, 0);
  motion_reset(); motion_follow_line(300);
  step_at = ticks + 100; step_err = 400; step_show = 8; spin_at = step_at + 40;
  run(160);
  step_at = -1;
  return 0; }
