# Barcode demo: store the letter, act at the cross-junction

What the demo asks for, and where each part lives.

| Demo step | What the car does | Code |
|---|---|---|
| Instructor names 4 letters | Dashboard card **Barcode map** sends them over MQTT (retained, QoS 1). The car logs `comms: barcode map set: ...` | `tools/dashboard.html`, `subsystem1/comms_bridge.c`, `telemetry_codec.c` (`comms_decode_map`) |
| 1. Scan in either direction | Code 39 reader on IR_L tries forwards, then backwards. All 26 letters A-Z are in the table | `subsystem3/line_sense.c` (`code39_decode`) |
| 2. Store the letter | `ctrl: barcode 'A' stored (L): carried out at the next cross-junction` | `subsystem5/task_ctrl.c` (`on_nav`) |
| 3. Keep driving to a cross-junction | Line following carries on. The cross-junction detector watches all three sensors | `subsystem3/line_sense.c` (`cross_feed_l`, `cross_feed_cr`), `task_line.c` |
| 4. Do the mapped movement | Wheel axle moved over the middle of the junction, then turn left, right or 180° (fixed turn, then a search turn that stops on the new line), or straight on | `subsystem5/task_ctrl.c` (`on_cross_junction`), `ctrl_logic.c` (`ctrl_plan_junction`) |
| 5. Repeat | Stored letter cleared; back to LINE FOLLOWING; next barcode is stored | `task_ctrl.c` |

## Before the demo

1. Measure **IR sensor row to the middle of the drive wheels** and put it in
   `CTRL_SENSOR_AHEAD_MM` (`app_program/subsystem5/ctrl_logic.h`, now a 70 mm
   estimate). The bypass landing uses the same value.
2. Check `APP_END_MARKER` (`app.h`, `'Z'`). An unmapped `Z` still ends the
   run (MISSION COMPLETE). If the course has no end marker, set it to `'\0'`.
   A letter in the map always means its movement, never "end".
3. Run S3-M05 (cross-junction detection) and S3-M06 (demo sequence) on the real mat.

## Sending the map

- Dashboard: type the four letters, press **Send map to car**, and check that
  "Car confirmed" shows the same letters.
- Or: `python3 tools/is11_mqtt.py map F U A Z --broker <ip>` (forward, U-turn,
  left, right).

The message is retained, so the broker gives it to the car again after a
reboot or Wi-Fi drop. Until a map arrives, every letter goes straight on and
the log says `NO BARCODE MAP received yet`.

## Testing barcodes without driving

Dashboard card **Barcode test**: press **Start barcode test**. The car then
refuses START (and stops if it was driving), and the reader works with the
car standing still: each sample counts as 0.1 mm of travel, and only the
ratio of bar widths matters. Slide the barcode under the LEFT sensor at a
steady hand speed, either way round, in under 3 s.

Every read prints `line: read 29 elements (15 bars) -> 'A'` or
`... -> rejected: <why>`. The card shows the letter, the bars counted (a
whole letter is 15 bars and 14 gaps), the OK/rejected counts and the reads
per letter. High beep = read OK, low beep = rejected. The test switch is
not retained: a reboot always comes back in normal mode.

## Seeing what a read got wrong

Every finished read (decoded or rejected) is published on `is11/car/bc`
(retained): the width of each bar and gap in 0.1 mm. The dashboard's
**Barcode picture** card draws it to scale, labels each element narrow or
wide, and lines it up against the expected `*X*` pattern (the letter read,
the closest letter, or one you pick), marking missing, extra and
wrong-width elements. The last 10 reads can be clicked back.

## Cross-junction rule (design values, check on the mat)

An isolated stripe under IR_L (8-60 mm wide, white before and after), while
IR_C and IR_R also see black within 40 mm of it. Barcode bars are never
isolated, and a left-only branch has IR_R white. Thresholds: `CROSS_xxx`
in `line_sense.h`.
