/*
 * subsystem1_test.c
 * On-target tests for subsystem 1. Built only with: make TEST=1
 */

#include "subsystem1.h"
#include "command_codec.h"
#include "telemetry_codec.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem1_test(void)
{
    UW before;
    uint8_t stop_byte[] = {0x02};
    uint8_t bad_byte[] = {0x82};
    uint8_t packed = 0;
    S1_TELEMETRY sample = {0};

    TEST_INIT();

    /* Controller sends about 10 snapshots in 2 seconds. */
    before = s1_rx_count;
    tk_dly_tsk(2000);
    TEST_CHECK("S1-U01",
               ((s1_rx_count - before) >= 8U) &&
               ((s1_rx_count - before) <= 12U),
               "about 10 telemetry snapshots in 2 s");

    /* Command byte: accept STOP, reject unexpected bits. */
    TEST_CHECK("S1-U02",
               s1_decode_command(stop_byte, sizeof(stop_byte)) == S1_CMD_STOP &&
               s1_decode_command(bad_byte, sizeof(bad_byte)) == S1_CMD_INVALID,
               "STOP accepted; byte with extra bits rejected");

    /* Pack READY and TURN RIGHT into one byte. */
    sample.state = VS_READY;
    sample.last_barcode = BC_TURN_RIGHT;
    TEST_CHECK("S1-U03",
               s1_pack_state_barcode(&sample, &packed) == 1 &&
               packed == 0x21,
               "state and barcode packed into expected byte");

    /* Reject a state outside the group's list. */
    sample.state = VS_NUM_STATES;
    TEST_CHECK("S1-U04",
               s1_pack_state_barcode(&sample, &packed) == 0,
               "invalid state rejected");

    TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */