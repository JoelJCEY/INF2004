/*
 * subsystem1_test.c
 * On-target tests for subsystem 1. Built only with: make TEST=1
 */

#include "subsystem1.h"
#include "command_codec.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem1_test(void)
{
    UW before;
    uint8_t stop_byte[] = {0x02};
    uint8_t bad_byte[] = {0x82};

    TEST_INIT();

    /* S1-U01: ctrl_task sends about 5 telemetry snapshots per second. */
    before = s1_rx_count;
    tk_dly_tsk(2000);
    TEST_CHECK("S1-U01",
               ((s1_rx_count - before) >= 8U) &&
               ((s1_rx_count - before) <= 12U),
               "about 10 telemetry snapshots in 2 s");

    /* S1-U02: accept STOP and reject a byte with unexpected bits. */
    TEST_CHECK("S1-U02",
               s1_decode_command(stop_byte, sizeof(stop_byte)) == S1_CMD_STOP &&
               s1_decode_command(bad_byte, sizeof(bad_byte)) == S1_CMD_INVALID,
               "STOP accepted; byte with extra bits rejected");

    TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */