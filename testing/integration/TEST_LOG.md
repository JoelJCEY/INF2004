# Integration — test log

**Owners:** both sides of each interface  ·  **Code:** `app_program/`

See `testing/README.md` for how to run tests and record results.
Nothing has been measured yet: every target below comes from the IS11 Design Review.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | comms + motion: 5 Hz telemetry while motion runs; remote STOP | IF-01, IF-03, IF-05, NFR3/4 | manual | `make TEST=1`, S2-U08 with MQTT online, then 20 × `is11_mqtt.py stop` | jitter ≤ 2 ms; stop ≤ 500 ms 20/20 |
| INT-02 | binary MQTT interface | IF-01, IF-03 | manual | `python3 tools/is11_mqtt.py selftest`, then live frames | golden vector matches; bad commands rejected and counted |
| INT-03 | line following + telemetry | IF-07, IF-08, IF-09 | manual | Track run with barcodes while publishing | no DMA overrun line; barcode behaviour as S3-M02 |
| INT-04 | terrain + motion | IF-07, IF-11, IF-12 | manual | Known hump while following the line | HUMP CROSSING then LINE FOLLOWING; peak meets NFR5 |
| INT-05 | obstacle + motion | IF-04, IF-05, IF-06, IF-13 | manual | Box on the line | no collision, correct side, line reacquired, NFR2 |
| INT-06 | concurrent fault snapshot | IF-02, 10.4 | manual | Trigger impact and line lost close together | both bits in one telemetry fault mask; nothing lost |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console
(`make SMP=0 NET=lwip MQTT=1 TEST=1 -j8`); **manual** = a person observes or
measures the result (scope, ruler, stopwatch, video).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
