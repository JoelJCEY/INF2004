# Subsystem 1 (Communication) — test log

**Owner:** Buddy 1, 2501327 (Daryl)  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.
Nothing has been measured yet: every target below comes from the IS11 Design Review.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | telemetry frame equals the golden vector | 10.5 decoder check | auto | `subsystem1_test.c` | `[TEST] S1-U01 PASS` |
| S1-U02a-e | valid STOP decodes; bad version, code 0, reserved bits, short frame rejected | FR5, 6.1 | auto | `subsystem1_test.c` | 5 × PASS |
| S1-U03 | status word fields keep 4/3/5-bit widths | NFR7, 10.3 | auto | `subsystem1_test.c` | PASS |
| S1-U04a-c | barcode map frame decodes; repeated / lower-case / '*' / short frames rejected; never mixed up with START/STOP | IF-15 | auto | `subsystem1_test.c` | PASS |
| S1-U05 | barcode test on/off frame decodes; never mixed up with START/STOP or the map | barcode test mode | auto | `subsystem1_test.c` | PASS |
| S1-M05 | barcode map reaches the car and survives a reboot | IF-15 | manual | Dashboard: Send map to car; then power-cycle the car | `comms: barcode map set` after each MQTT connect (retained message) |
| S1-M01 | laptop decodes live telemetry at 5 Hz | NFR4, A1 | manual | `python3 tools/is11_mqtt.py listen --broker <ip>` for 60 s | ≈300 frames, gaps never > 500 ms |
| S1-M02 | remote STOP latency | NFR4, A1 | manual | Car following the line; `is11_mqtt.py stop`; video at 240 fps or GPIO scope on M1A | motors stopped ≤ 500 ms, 20/20 trials |
| S1-M03 | reconnection after the access point returns | NFR4, A1 | manual | Switch the AP off for 30 s, on again; watch `comms: link state` lines | publishing resumes ≤ 10 s; car never stopped |
| S1-M04 | Last Will on power loss | 6.1 | manual | Pull power while listening on `is11/car/status` | `offline` within the 10 s keep-alive × 1.5 |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console
(`make SMP=0 NET=lwip MQTT=1 TEST=1 -j8`); **manual** = a person observes or
measures the result (scope, ruler, stopwatch, video).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
