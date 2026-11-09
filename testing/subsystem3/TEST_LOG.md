# Subsystem 3 (Line & Barcode) — test log

**Owner:** Buddy 3, 2500991 (Gillton)  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.
Nothing has been measured yet: every target below comes from the IS11 Design Review.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | '*A*' decodes forwards | FR2 | auto | `subsystem3_test.c` | PASS |
| S3-U02 | '*A*' decodes backwards | FR2, 6.3 | auto | `subsystem3_test.c` | PASS |
| S3-U03 | 28-element read rejected | VA2 | auto | `subsystem3_test.c` | PASS |
| S3-U04a-b | letter-to-movement comes from the MQTT map (nothing hard-coded); a mapped 'Z' is a movement, not the end marker | FR2, IF-15 | auto | `subsystem3_test.c` | PASS |
| S3-U05 | reader decodes '*Z*' from 1 kHz samples at 0.3 m/s | NFR1 | auto | `subsystem3_test.c` | PASS |
| S3-U06 | 18 mm single stripe is a junction | F3, A3 | auto | `subsystem3_test.c` | PASS |
| S3-U07a-c | line error sign and packing | IF-08 | auto | `subsystem3_test.c` | PASS |
| S3-U08 | DMA ring write index advances | NFR1 | auto | `subsystem3_test.c` | PASS |
| S3-U09 | every letter A-Z decodes to the same letter forwards and backwards | FR2, demo rule | auto | `subsystem3_test.c` | PASS |
| S3-U10a-f | cross-junction detector: 18 mm line under L+C+R is one cross; C or R white, barcode bars, or 120 mm of black are not; reported middle is behind the sensors | IF-09 kind 6 | auto | `subsystem3_test.c` | PASS |
| S3-M01 | raw ADC levels on black, white, barcodes at 0.15/0.3/0.5 m/s | A3 | manual | Log `line_latest` + console; vary room lighting | black/white separate by ≥ 800 counts; set LINE_DEF_* |
| S3-M02 | barcode decode rate | NFR1, A3 | manual | 20 passes per barcode, both directions, each sample size | ≥ 95 % correct |
| S3-M03 | tightest curve at 0.3 m/s | FR1, F9 | manual | 10 laps | no LINE LOST |
| S3-M04 | line lost after 200 mm | 6.3, 5.1 | manual | Lift the car off the line while rolling | STOPPED after ~200 mm, FLT_LINE_LOST |
| S3-M05 | cross-junction detection on the real mat | IF-09 kind 6 | manual | Drive over 10 cross-junctions, 10 barcodes and 10 left/right branches; count `line: CROSS-JUNCTION` lines | 10/10 crosses, 0 on barcodes or branches; else adjust CROSS_xxx in `line_sense.h` |
| S3-U11a-c | debounce: the sensor swinging back over lines it crossed (1.5 mm every 4 mm, 2.4 mm every 7 mm) still reads 'A' and counts 15 lines | FR2, NFR1 | auto | `subsystem3_test.c` | PASS |
| S3-U12 | a 0.3 mm speck in the quiet zone is not a line | FR2 | auto | `subsystem3_test.c` | PASS |
| S3-U13 | encoder distance filter: exact travel from counts, IR_L slower/backwards when steering, impossible jump dropped | FR2 | auto | `subsystem3_test.c` | PASS |
| S3-M09 | barcode read while the car corrects | FR2 | manual | Barcodes placed where the line bends; 10 passes each | log shows `holding steady` during the read; 15 lines and the right letter, no search spin mid-barcode |
| S3-M08 | IR_L offset from the car's centre line | FR2 | manual | Ruler, sensor spot to the middle between the wheels | value in `S3_IR_L_SIDE_MM` (`subsystem3.h`) |
| S3-M07 | barcode hand test (car still) | FR2, NFR1 | manual | Dashboard: Start barcode test; slide each sample barcode 20 times each way | ≥ 95 % OK; rejected reads show the bars counted and why |
| S3-M06 | demo sequence | FR2, IF-15 | manual | Send a map from the dashboard; barcode, then cross-junction, for each of the 4 letters, read both directions | car does the mapped movement at the junction 8/8, then follows the new line |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console
(`make SMP=0 NET=lwip MQTT=1 TEST=1 -j8`); **manual** = a person observes or
measures the result (scope, ruler, stopwatch, video).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
