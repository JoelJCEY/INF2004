/* Host-only stubs for tools/host_test (not part of the firmware). */
#include "app.h"
#include "subsystem1/subsystem1.h"
#include "subsystem2/subsystem2.h"
#include "subsystem3/subsystem3.h"
#include "subsystem4/subsystem4.h"
#include "subsystem5/subsystem5.h"
ER tk_dly_tsk(RELTIM t){(void)t;return 0;}
ER tk_ref_flg(ID id, T_RFLG *r){(void)id;r->flgptn=0xFFFFFFFFu;return 0;}
ER tk_set_flg(ID id, UINT p){(void)id;(void)p;return 0;}
ER tk_snd_mbf(ID id,const void*m,INT sz,TMO t){(void)id;(void)m;(void)sz;(void)t;return 0;}
INT tk_rcv_mbf(ID id,void*m,TMO t){(void)id;(void)m;(void)t;return -1;}
ER tk_wup_tsk(ID id){(void)id;return 0;} INT tk_can_wup(ID id){(void)id;return 0;} ER tk_slp_tsk(TMO t){(void)t;return 0;}
UW in_w(UW a){(void)a;return 0;} void out_w(UW a,UW v){(void)a;(void)v;} void set_w(UW a,UW v){(void)a;(void)v;} void clr_w(UW a,UW v){(void)a;(void)v;}
ER tk_def_int(UINT n,const T_DINT*d){(void)n;(void)d;return 0;} void EnableInt(UINT n,INT l){(void)n;(void)l;} void ClearInt(UINT n){(void)n;}
ER gpio_set_pin(UINT no, UINT mode){(void)no;(void)mode;return 0;} ER gpio_set_val(UINT no, UINT val){(void)no;(void)val;return 0;}
ER tk_dis_dsp(void){return 0;} ER tk_ena_dsp(void){return 0;}
UINT gpio_get_val(UINT no){(void)no;return 0;}
UW app_time_us(void){return 0;} UW app_time_ms(void){return 0;} void app_delay_us(UW us){(void)us;}
/* published words the tests read */
volatile UW comms_state_word, comms_outbox_dropped, comms_rx_rejected;
volatile UW motion_odo_path_01mm, motion_odo_speed=0x80008000u, motion_mode_word, motion_jitter_max_us;
BOOL motion_encoders_ok = TRUE;
volatile UW line_latest, line_read_stats;
ID flg_sys, tid_scan;
volatile UW scan_status_word;
static UW wr; UW drv_ir_adc_write_index(void){ return (wr += 7) & 511; }
void app_unreset(UW mask){(void)mask;}	/* 31 Oct: drv_ultrasonic.c */
