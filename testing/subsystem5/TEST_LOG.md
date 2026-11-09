# Subsystem 5 (Obstacle Scanning + Vehicle Controller) — test log

**Owner:** Buddy 5, 2500118 (Marcus)  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.
Nothing has been measured yet: every target below comes from the IS11 Design Review.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | echo duration to distance | A5 | auto | `subsystem5_test.c` | PASS |
| S5-U02 | profile width and closest point | FR4 | auto | `subsystem5_test.c` | PASS |
| S5-U03 | blocked right side → bypass left | FR4 | auto | `subsystem5_test.c` | PASS |
| S5-U04 | START/STOP/RESET acceptance | 5, FR5 | auto | `subsystem5_test.c` | PASS |
| S5-U05 | fault table responses | 5.1 | auto | `subsystem5_test.c` | PASS |
| S5-U06 | bypass plan: 7 legs, net heading 0 | FR4 | auto | `subsystem5_test.c` | PASS |
| S5-U07 | barcode command to manoeuvre | FR2 | auto | `subsystem5_test.c` | PASS |
| S5-U08a-c | junction plan: axle moved over the middle, then turn; forward / unknown letter = straight on | FR2, IF-15 | auto | `subsystem5_test.c` | PASS |
| S5-M01 | HC-SR04+ range at 3.3 V across servo angles | A5, F4 | manual | Check 3.3 V on VCC before plugging ECHO; boxes at 100-800 mm, 30°-150° | valid echo rate and error recorded |
| S5-M02 | ISR word integrity | 10.5, C1 | manual | 1000 pings compared with the ECHO pulse width on a scope | every reading matches ±1 % |
| S5-M03 | detect-to-stop | NFR2 | manual | Approach a box at 0.3 m/s ×10 | at rest ≥ 50 mm before it, no contact |
| S5-M04 | full bypass cycle | FR4, NFR2 | manual | Box on the line; stopwatch from alert to LINE FOLLOWING | no contact, line reacquired, < 5 s (target 3 s) |
| S5-M05 | watchdog | 5, F8 | manual | Build with a deliberate hang in ctrl_task (test branch only) | MCU resets within 1 s; first telemetry has the watchdog bit |
| S5-M07 | IR sensor row to wheel axle | IF-15 | manual | Ruler: IR sensor row to the middle of the drive wheels, along the car | value in `CTRL_SENSOR_AHEAD_MM` (`ctrl_logic.h`, also used by the bypass landing); after a junction turn the centre sensor is on the new line |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console
(`make SMP=0 NET=lwip MQTT=1 TEST=1 -j8`); **manual** = a person observes or
measures the result (scope, ruler, stopwatch, video).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
