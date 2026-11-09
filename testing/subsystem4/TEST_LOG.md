# Subsystem 4 (Terrain) — test log

**Owner:** Buddy 4, 2501315 (Xavier)  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.
Nothing has been measured yet: every target below comes from the IS11 Design Review.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | integer square root | — | auto | `subsystem4_test.c` | PASS |
| S4-U02 | 20 mm hump reconstructed within 15 % | FR3, NFR5 | auto | `subsystem4_test.c` | PASS |
| S4-U03 | 600 mm climb discarded | 6.4 | auto | `subsystem4_test.c` | PASS |
| S4-U04 | no false hump on flat track | NFR5 | auto | `subsystem4_test.c` | PASS |
| S4-U05 | LSM303DLHC answers, baseline done | A4 | auto | `subsystem4_test.c` | PASS (FAIL if the IMU is unplugged) |
| S4-M01 | noise on flat track with motors running | A4 | manual | 5 m flat run; log `imu:` lines and telemetry | no hump reported; set IMPACT_MG from the log |
| S4-M02 | known hump height | NFR5, A4 | manual | 5 runs over a measured hump | peak within ±5 mm or ±15 % |
| S4-M03 | impact | 5.1 | manual | Tap the car into a soft block at 0.3 m/s | STOPPED, FLT_IMPACT; no impact when landing off the hump |
| S4-M04 | axis and wheelbase | A4 | manual | Tilt the car nose-up 10°; measure drive axle to caster | dh > 0; HUMP_WHEELBASE_MM updated |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console
(`make SMP=0 NET=lwip MQTT=1 TEST=1 -j8`); **manual** = a person observes or
measures the result (scope, ruler, stopwatch, video).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
