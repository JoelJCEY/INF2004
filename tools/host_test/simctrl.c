/* Exercise the actual barcode consumer and manoeuvre step executor. */
#include <assert.h>
#include "../../app_program/subsystem5/task_ctrl.c"

ID mbf_motion_cmd = 1, mtx_mission = 2;
volatile UW app_barcode_map, app_barcode_test;
volatile UW motion_odo_path_01mm, line_latest, line_cal_request, line_norm[3];
volatile UW scan_pass_seq, scan_pass_watch, scan_rescan_request;
static MOTION_CMD last_cmd;
static int sent;
UW app_time_ms(void) { return 1000; }
ER tk_loc_mtx(ID id, TMO t) { (void)id; (void)t; return 0; }
ER tk_unl_mtx(ID id) { (void)id; return 0; }
ER tk_snd_mbf(ID id, const void *m, INT n, TMO t)
{ (void)id; (void)n; (void)t; last_cmd = *(const MOTION_CMD *)m; sent++; return 0; }
void drv_buzzer_tone(UW hz) { (void)hz; }
void drv_neopixel_set(UW rgb) { (void)rgb; }

static void check_uturn(VEHICLE_STATE initial)
{
    NAV_MSG barcode = {.kind=NAV_BARCODE, .letter='A', .cmd=BC_U_TURN};
    NAV_MSG junction = {.kind=NAV_JUNCTION_CROSS, .path_mm=100};
    MOTION_STATUS status = {.event=MS_DONE};
    mission.state=initial; pending_letter=0; sent=0;
    app_barcode_map=APP_BARCODE_MAP('F','A','L','R');
    motion_odo_path_01mm=(100+CTRL_SENSOR_AHEAD_MM)*10;
    on_nav(&barcode);
    assert(pending_letter=='A' && sent==0);
    on_nav(&junction);
    assert(pending_letter==0 && mission.state==VS_BARCODE_MANOEUVRE);
    assert(last_cmd.op==MC_TURN && last_cmd.arg==1500);
    status.seq=(UB)wait_seq; on_motion(&status);
    assert(search_active && last_cmd.op==MC_TURN && last_cmd.arg==600);
    /* Right sensor alone must not stop the turn early. */
    line_latest=line_pack(-600,LINE_F_VALID|LINE_F_ON_LINE,1000);
    line_norm[1]=0; update_line_search(1000); update_line_search(1000);
    assert(search_active && last_cmd.op==MC_TURN);
    line_norm[1]=1000; update_line_search(1000);
    assert(search_active); /* centre must be stable for two controller checks */
    update_line_search(1000);
    assert(search_active); /* repeated reads of one sample do not count */
    line_latest=line_pack(0,LINE_F_VALID|LINE_F_ON_LINE,1020);
    update_line_search(1020);
    assert(!search_active && last_cmd.op==MC_STOP);
    status.seq=(UB)wait_seq; status.event=MS_STOPPED; on_motion(&status);
    assert(mission.state==VS_LINE_FOLLOWING && last_cmd.op==MC_FOLLOW_LINE);
}

int main(void)
{
    check_uturn(VS_LINE_FOLLOWING);
    check_uturn(VS_HUMP_CROSSING);
    printf("barcode controller U-turn, hump-state junction and centred landing: PASS\n");
    return 0;
}
