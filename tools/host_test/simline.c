/* Runs the real line task against scripted ADC DMA and centre DO inputs.
   Unlike pure sensor tests, this includes the junction hold and lost search. */
#include <assert.h>
#include <setjmp.h>
#include <string.h>
#include "subsystem3/line_sense.h"
#include "subsystem3/drv_ir_adc.h"
#include "subsystem2/motion_ctrl.h"
#include "subsystem5/subsystem5.h"
#include "subsystem5/ctrl_logic.h"
#include <stdio.h>
#include <stdlib.h>

ID tid_line = 1, mbf_nav = 2, flg_sys = 3;
volatile UW motion_odo_path_01mm, motion_mode_word = MM_FOLLOW;
volatile UW motion_odo_speed = 0x803c803cU; /* both wheels at 60 mm/s */
volatile W motion_enc_count[2];
volatile UW app_barcode_map, app_barcode_test;
static UH ring[IR_ADC_RING_SAMPLES];
static UW written, tick;
static UINT centre;
static int scenario, crosses, cross_tick;
static int barcode_reads;
static int long_left;
static int barcode_only;
static int lost_stop, lost_reports;
static UINT fault_bits;
static UW barcode_edges[29];
static jmp_buf finished;

static int barcode_black(UW position)
{
    UW start=30000;
    if (position < start) return 0;
    for (unsigned i=0;i<29;i++) {
        if (position < start+barcode_edges[i]) return (i%2)==0;
    }
    return 0;
}

/* Kernel sleep advances one 20 ms cycle and fills 20 samples per ADC. */
ER tk_slp_tsk(TMO t)
{
    int c = 1, r = 0, l = 0;
    (void)t;
    if (getenv("SIMLINE_TRACE")) /* per-cycle output, for before/after diffs */
        printf("t%u err %d flags %x nav %d cross %d\n", (unsigned)tick,
               (int)line_unpack_err(line_latest),
               (unsigned)line_unpack_flags(line_latest), barcode_reads, crosses);
    if (tick) {
        INT e = line_unpack_err(line_latest);
        unsigned prev = tick - 1;
        if (scenario == 0) {
            /* 16 Nov early hold: C and R both black at tick 20, so a
               right correction is held at 0 for CROSS_EARLY_HOLD_MM
               (45 mm = about 37 cycles at 1.2 mm per cycle), even
               while only R sees the line.  This scenario feeds a fixed
               sensor pattern (the car does not turn), so it only checks
               the rule, not whether a real car stays on a right curve. */
            if (prev >= 19 && prev < 55) assert(e == 0);
            if (prev >= 60 && prev < 80) assert(e < 0 && e > -LINE_ERR_MAX);
            if (prev >= 82) assert(e == 0);
            assert(app_abs(e) < LINE_ERR_MAX);
            assert(crosses == 0);
        } else if (scenario == 1) {
            unsigned end=lost_stop ? 620 : long_left ? 270 : 28;
            if (prev >= (barcode_only ? 32U : 21U) && prev < end)
                assert(e > 0 && e < LINE_ERR_MAX);
            if (barcode_only && prev>=32 && prev<end)
                assert((line_unpack_flags(line_latest)&LINE_F_ON_LINE)==0);
            if (prev >= end+2) assert(e == 0);
            assert(crosses == 0);
        } else if (scenario < 4) {
            if (prev >= 28 && prev < 69) assert(e == 0);
            if (crosses && (int)prev > cross_tick && (int)prev < cross_tick + 10)
                assert(e == 0); /* confirmed crossing still holds straight */
        }
    }
    if (tick >= (lost_stop ? 650U : scenario>=4 || long_left ? 400U : 120U)) longjmp(finished, 1);
    motion_mode_word=MM_FOLLOW;
    if (scenario == 0) {
        r = (tick >= 20 && tick < 55) || (tick >= 60 && tick < 80);
        c = !(tick >= 30 && tick < 60);
    } else if (scenario == 1) {
        c = !(tick >= (barcode_only ? 30U : 20U) && tick < (lost_stop ? 620U : long_left ? 270U : 28U));
        if (barcode_only) l=tick>=20 && tick<270;
    } else if (scenario == 2) {
        l = r = tick >= 25 && tick < 41;
        if (crosses && (int)tick < cross_tick + 10) { c = 0; r = 1; }
    } else if (scenario == 3) {
        /* Curve exit, then a skewed cross: R meets the transverse line
           before L, and C briefly leaves black while crossing it. */
        r = tick >= 20 && tick < 45;
        l = tick >= 28 && tick < 44;
        c = !(tick >= 35 && tick < 48);
        if (crosses && (int)tick < cross_tick + 10) { c = 0; r = 1; }
    } else {
        r = tick >= 250 && tick < 267;
        c = 1;
        if (scenario==5 && tick>=200 && tick<218) {
            motion_mode_word=MM_TURN;
            r=1;
        }
    }
    centre = c;
    for (unsigned i = 0; i < 60; i++) {
        unsigned ch = written % 3;
        /* Physical ADC1=right, ADC2=centre: IR_SWAP_C_R=1. */
        int black = ch == 0 ? l : ch == 1 ? r : c;
        if (scenario>=4 && ch==0) {
            UW position=(written/3)*60;
            black=barcode_black(position) || (position>=300000 && position<320000);
            if(scenario==5 && tick>=200 && tick<218) black=1;
        }
        ring[written++ & (IR_ADC_RING_SAMPLES - 1)] = black ?
            ((scenario==6 && ch==0) ? 1630 : 3700) : 250;
    }
    tick++;
    motion_odo_path_01mm = tick * S3_PERIOD_MS * 60 / 100;
    motion_enc_count[0] = motion_enc_count[1] = tick * 1200 / MOTION_UM_PER_COUNT;
    return E_OK;
}
UW app_time_ms(void) { return tick * S3_PERIOD_MS; }
UINT gpio_get_val(UINT n) { (void)n; return centre; }
ER gpio_set_pin(UINT n, UINT mode) { (void)n; (void)mode; return E_OK; }
void set_w(UW a, UW v) { (void)a; (void)v; }
ER tk_set_flg(ID id, UINT f) { (void)id; fault_bits|=f; return E_OK; }
ER tk_wup_tsk(ID id) { (void)id; return E_OK; }
INT tk_can_wup(ID id) { (void)id; return 0; }
ER tk_snd_mbf(ID id, const void *m, INT sz, TMO timeout)
{
    const NAV_MSG *nav = m;
    (void)id; (void)sz; (void)timeout;
    if (nav->kind==NAV_LINE_LOST) lost_reports++;
    if (nav->kind == NAV_BARCODE) {
        assert(scenario>=4 && nav->letter=='A' && nav->cmd==BC_U_TURN);
        barcode_reads++;
    }
    if (nav->kind == NAV_JUNCTION_CROSS) {
        if (scenario>=4) assert(barcode_reads==1 && tick>=267);
        crosses++; cross_tick = (int)tick - 1;
    }
    return E_OK;
}
/* 16 Nov lift check: no hump, car driving (line following) */
ER tk_ref_flg(ID id, T_RFLG *r) { (void)id; r->flgptn = 0; return E_OK; }
void ctrl_get_mission(MISSION_STATE *out)
{
    memset(out, 0, sizeof(*out));
    out->state = VS_LINE_FOLLOWING;
}
BOOL ctrl_is_active(VEHICLE_STATE s) { return s == VS_LINE_FOLLOWING; }
int32_t drv_ir_adc_init(void) { return 0; }
uint32_t drv_ir_adc_samples_written(void) { return written; }
const volatile uint16_t *drv_ir_adc_ring(void) { return ring; }
uint32_t drv_ir_adc_take_errors(void) { return 0; }

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    assert(argc == 2);
    long_left=!strcmp(argv[1], "long-left");
    barcode_only=!strcmp(argv[1], "barcode-only");
    lost_stop=!strcmp(argv[1], "lost-stop");
    if (barcode_only) long_left=1;
    if (lost_stop) long_left=1;
    scenario = !strcmp(argv[1], "right") ? 0 : (!strcmp(argv[1], "left") || long_left) ? 1 :
               !strcmp(argv[1], "cross") ? 2 : !strcmp(argv[1], "skew-cross") ? 3 :
               !strcmp(argv[1], "barcode-cross") ? 4 :
               !strcmp(argv[1], "pivot-cross") ? 5 : 6;
    if (scenario>=4) {
        const UH patterns[3]={0x094,0x109,0x094}; /* *A* */
        unsigned i=0; UW edge=0;
        for (int ch=0;ch<3;ch++) {
            for (int bit=8;bit>=0;bit--) {
                edge+=((patterns[ch]>>bit)&1) ? 9000 : 3000;
                barcode_edges[i++]=edge;
            }
            if (ch<2) { edge+=3000; barcode_edges[i++]=edge; }
        }
        app_barcode_map=app_barcode_map_pack('F','A','L','R');
    }
    if (!setjmp(finished)) line_task(0, NULL);
    if (scenario >= 2) assert(crosses == 1);
    if (scenario>=4) assert(barcode_reads==1);
    if (lost_stop) assert(lost_reports==1 && (fault_bits&FLT_LINE_LOST));
    printf("line task %s regression: PASS\n", argv[1]);
    return 0;
}
