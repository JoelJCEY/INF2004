/* PC run of the sensor bench (app_program/sensor_bench.c) against a
   scripted "user" and fake sensors.  ./benchsim good  -> all PASS
                                       ./benchsim bad   -> each fault caught */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "app.h"
#include "subsystem2/subsystem2.h"
#include "subsystem4/subsystem4.h"
#include "subsystem5/subsystem5.h"
#include "subsystem5/drv_ultrasonic.h"

volatile UW line_raw_avg[3];
volatile W imu_acc_raw[3], imu_mag_raw[3], imu_base_mg[2] = {-8, 24};
volatile UW imu_reads_ok, imu_reads_fail;
volatile W motion_enc_count[2];
volatile UW scan_bench_on, scan_bench_word, scan_bench_seq, app_log_quiet;
static int bad, idle; static long t; static char step[16]; static long step_t0; static int away, flat;
static int pass_n, fail_n;

INT tm_vsnprintf(UB *s, INT m, const UB *f, va_list ap){int n=vsnprintf((char*)s,m,(const char*)f,ap);return n<m?n:m-1;}
uint32_t drv_ir_adc_samples_written(void){return (uint32_t)(t*3);}
void ctrl_get_mission(MISSION_STATE *o){memset(o,0,sizeof *o);o->state=(t<3000)?0:1;}
int sim_log(const UB *fmt, ...){
  char b[300]; va_list ap; va_start(ap,fmt); vsnprintf(b,sizeof b,(const char*)fmt,ap); va_end(ap);
  printf("%6.1f  %s", t/1000.0, b);
  const char *p=strstr(b,"[BENCH] S"); if(p){sscanf(p+8,"%15s",step);step_t0=t;away=0;flat=0;}
  if(strstr(b,"take the line away")) away=1;
  if(strstr(b,"back down flat")) flat=1;
  if(strstr(b,"[TEST] S")){ if(strstr(b," PASS "))pass_n++; else fail_n++; }
  return 0;
}
static int noise(int a){return (rand()%(2*a+1))-a;}
static int is(const char*s){return strcmp(step,s)==0;}
static void world(void){
  long dt=t-step_t0; int act=!idle&&dt>3000;
  /* IR */
  int white[3]={200,150,180};
  for(int i=0;i<3;i++) line_raw_avg[i]=white[i]+noise(15);
  int tgt=is("S3-B03")?0:is("S3-B04")?1:is("S3-B05")?2:-1;
  if(tgt>=0&&act&&!away){int phys=bad?(tgt==0?2:tgt==2?0:1):tgt; line_raw_avg[phys]=3500+noise(30);
    if(phys==1){line_raw_avg[0]+=200;line_raw_avg[2]+=200;}}
  /* encoders: turn over 2 s */
  double f=act?fmin(1.0,(dt-3000)/2000.0):0;
  static W base_l,base_r; static char last[16];
  if(strcmp(last,step)){strcpy(last,step);base_l=motion_enc_count[0];base_r=motion_enc_count[1];}
  if(is("S2-B01")) motion_enc_count[0]=base_l+(W)((bad?-1:1)*2800*f);
  if(is("S2-B02")) motion_enc_count[1]=base_r+(W)(2650*f);
  if(is("S2-B03")){motion_enc_count[0]=base_l-(W)(1500*f);motion_enc_count[1]=base_r-(W)(1400*f);}
  /* IMU */
  imu_acc_raw[0]=-8+noise(8); imu_acc_raw[1]=24+noise(8); imu_acc_raw[2]=995+noise(8);
  if(is("S4-B02")&&act&&!flat) imu_acc_raw[0]+=bad?-340:340;
  if(is("S4-B03")&&act&&!flat) imu_acc_raw[1]+=300;
  if(is("S4-B04")&&dt>3000&&dt<3015) imu_acc_raw[2]+=1500;
  imu_reads_ok=(UW)(t/20);
  double a=is("S4-B05")?(dt/15000.0)*6.283:0; imu_mag_raw[0]=(W)(300*cos(a)); imu_mag_raw[1]=(W)(300*sin(a)); imu_mag_raw[2]=-400;
  /* ultrasonic, one ping per 70 ms */
  static long next; if(scan_bench_on&&t>=next){next=t+70;scan_bench_seq++;
    W mm=is("S5-B01")?200:400; scan_bench_word=bad?((UW)US_NO_ECHO<<16):(((UW)US_OK<<16)|(UW)(mm+noise(6)));}
}
ER tk_dly_tsk(RELTIM ms){for(RELTIM i=0;i<ms;i++){t++;world();}return 0;}
void sensor_bench(void);
int main(int c,char**v){bad=(c>1&&!strcmp(v[1],"bad"));idle=(c>1&&!strcmp(v[1],"idle"));srand(1);sensor_bench();
  printf("== %s: %d PASS, %d FAIL ==\n",bad?"bad":"good",pass_n,fail_n);return 0;}
