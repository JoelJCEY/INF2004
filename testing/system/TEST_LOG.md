# System — test log

**Owners:** whole team  ·  **Code:** `app_program/`

See `testing/README.md` for how to run tests and record results.
Nothing has been measured yet: every target below comes from the IS11 Design Review.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | full course run, autonomous | FR1-FR5 | manual | Place on start, press START (GP20) | course completed; telemetry the whole run |
| SYS-02 | STOP button from every active state | 5 | manual | Press STOP (GP21) in each state | STOPPED; START resumes |
| SYS-03 | resource use before/after packing | NFR7, 10.5 | manual | `arm-none-eabi-size`, map file, sizeof() of every message, baseline tag vs packed | table of RAM and payload sizes |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console
(`make SMP=0 NET=lwip MQTT=1 TEST=1 -j8`); **manual** = a person observes or
measures the result (scope, ruler, stopwatch, video).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
