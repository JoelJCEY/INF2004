# Subsystem 2 (Motion Control) — test log

**Owner:** Buddy 2, 2501113 (Joel)  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.
Nothing has been measured yet: every target below comes from the IS11 Design Review.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | PIO quadrature program at offset 0 | A2 | auto | `subsystem2_test.c` | PASS; boot log `motion: encoders on PIOx offset 0` |
| S2-U02 | 1000 mm → counts → 0.1 mm round trip | NFR6 | auto | `subsystem2_test.c` | PASS |
| S2-U03 | 90° turn arc = π·track/4 | NFR6 | auto | `subsystem2_test.c` | PASS |
| S2-U04a/b | PID integrator clamped and cleared | VA2 | auto | `subsystem2_test.c` | PASS |
| S2-U05a-d | stall rule: < 20 % of commanded for 300 ms | 5.1, F11 | auto | `subsystem2_test.c` | PASS |
| S2-U06 | odometry speed word round trip | IF-07, 10.4 | auto | `subsystem2_test.c` | PASS |
| S2-U07 | soft-start ramp step | VA2 | auto | `subsystem2_test.c` | PASS |
| S2-U08 | motion_task release jitter | NFR3 | auto | `subsystem2_test.c` (repeat with telemetry running) | ≤ 2000 µs |
| S2-M01 | hand-turn counts per revolution and sign | A2 | manual | Wheels off the ground; turn each wheel 10 turns forward; read `enc_l/enc_r` in telemetry | counts = 10 × MOTION_COUNTS_PER_REV, both positive; update subsystem2.h |
| S2-M02 | PWM sweep with Wi-Fi active | A2 | manual | Run each wheel at 30/60/90 % with MQTT publishing; compare counts with marked rotations | no missing counts |
| S2-M03 | 1 m straight run | NFR6, A2 | manual | `moveForward(1000)` on the mat ×5; tape measure | error ≤ 30 mm |
| S2-M04 | 90° turn accuracy | NFR6, A2 | manual | `turnLeft(90)` / `turnRight(90)` ×5; protractor on the mat | error ≤ 5° |
| S2-M05 | stall lock-off | 5.1 | manual | Hold one wheel while following the line | FAULT STOP within ~0.3 s, motors braked, FLT_STALL in telemetry |
| S2-M06 | braking distance from 0.3 and 0.5 m/s | NFR2, F10 | manual | Follow line; STOP; measure travel after the command | value recorded; sets SCAN_STOP_DIST_MM |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console
(`make SMP=0 NET=lwip MQTT=1 TEST=1 -j8`); **manual** = a person observes or
measures the result (scope, ruler, stopwatch, video).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
