# PicoCar IS11, basic version

The whole mission in its simplest working form, written to be the first
thing that runs on the car. It is also the **plain-types baseline** for
the Week 13 before/after comparison (NFR7): every message is a plain C
struct, with no bit packing.

> **Status:** compiles with no warnings (also with `-Wall -Wextra -Wshadow`)
> in all four builds: `make SMP=0`, `TEST=1`, `CONSOLE=uart WIFI=none` and
> `NET=lwip MQTT=1`, using Pico SDK 2.2.0 and arm-none-eabi-gcc 13.2.
> **Nothing has run on the car yet.** The Code 39 decoder was tested on a
> PC with simulated sensor samples: 2,996 of 3,000 random barcodes read
> correctly (any letter, either direction, 150–500 mm/s, ±15 % speed change,
> sensor noise), none read as the wrong letter, and no false reads from
> junction stripes or cut-off barcodes. Every number in the code is a design
> value or a placeholder until you measure it.

## What it does

| Part | Basic behaviour |
|---|---|
| ctrl_task (S5, priority 10) | Mission state machine: INITIALISING → READY → (START) → LINE FOLLOWING. Barcode → BARCODE MANOEUVRE → back. Hump → HUMP CROSSING (slower). Obstacle → **stops and waits** until the path is clear (no bypass yet). STOP button, impact or LINE LOST → STOPPED. Stall or IMU fault → FAULT STOP. Telemetry snapshot every 200 ms. |
| motion_task (S2, 11) | 20 ms loop. Per-wheel PI speed control with feed-forward; PD steering from the line error; encoder-measured moves and turns (the brief's API: `motion_move_forward`, `motion_move_backward`, `motion_turn_left`, `motion_turn_right`, `motion_stop`). Stall = measured speed < 20 % of commanded for 300 ms. Encoders: **edge counters on the B pins (no direction)**. |
| line_task (S3, 12) | ADC round-robin on ADC0–ADC2 at 1 kHz per channel, DMA into a 1 KB ring. Line error from IR_L and IR_C, published as latest value + timestamp. Code 39 on IR_R: hysteresis, 2-sample debounce, 29 elements, width ratios, both directions, same barcode ignored for 200 mm. Both sensors white: last side kept for 200 mm, then LINE LOST. **Fixed thresholds** (from your bring-up logs). |
| scan_task (S5, 13) | Servo straight ahead, one polled ping every 60 ms. Two readings ≤ 200 mm → alert; three > 300 mm → clear. Ignored while the hump bit is set. ECHO never rising 3 times → sensor fault. |
| imu_task (S4, 14) | Accelerometer at 100 Hz, ±8 g. Level baseline in the first second. Hump: LEVEL → CLIMBING → DESCENDING → LEVEL; peak ≈ wheelbase × sin(max tilt). Climb > 500 mm, or a climb that goes level without descending, is discarded. Impact: \|a\| far from 1 g. |
| comms_task (S1, 15) | Prints each snapshot as one CSV line (`T,...`), watches the Wi-Fi link. **No MQTT changes**; with `NET=lwip MQTT=1` the kernel heartbeat still runs. |

## Fill these in before driving

| File | Constant | How to get it |
|---|---|---|
| `subsystem2/subsystem2.h` | `S2_UM_PER_EDGE` | Bring-up: turn a wheel exactly once by hand, read the edge count N. Value = π × wheel diameter (µm) ÷ N. |
| | `S2_TRACK_MM` | Ruler: centre of left tyre to centre of right tyre. |
| | `S2_LEFT_INVERT`, `S2_RIGHT_INVERT` | Set to 1 if that wheel runs backwards for "forward". |
| | `S2_FF_PERMILLE_X1000`, `S2_KP_X1000`, `S2_KI_X1000` | Start with the defaults; tune on a stand while printing speeds (`S2_PRINT_ENABLE 1`). |
| `subsystem3/subsystem3.h` | `S3_WHITE_x`, `S3_BLACK_x` | Bring-up IR logger over the real mat (min/max per sensor). |
| `subsystem4/subsystem4.h` | `S4_FWD_AXIS`, `S4_FWD_SIGN` | Tilt the nose up: `s4_sin_x1000` must go positive (`S4_PRINT_ENABLE 1`). |
| | `S4_WHEELBASE_MM` | Ruler: front axle to rear wheels/caster. |
| `subsystem5/subsystem5.h` | `S5_SERVO_MIN_US`, `S5_SERVO_MAX_US` | So that 90° points straight ahead. |
| | `S5_STOP_MM` | From the braking-distance test (provisional 200 mm). |
| | `S5_CRUISE_MM_S` | Start low (150) and raise it after the tightest-curve test (A3). |
| | `S5_EXEC_DELAY_MM`, `S5_EXEC_FWD_MM` | Where to turn after a barcode. Depends on the course rules (still to confirm with staff). |
| `subsystem3/barcode.c` | `barcode_command()` | Letter → command. **To confirm with staff:** the new samples have A, F, Z, W, X, Y but no B, C or D. |

## Running it

```sh
cd build_make
make SMP=0 -j8            # mtk3pico_smp0_usb_cdc_wifi.uf2
make SMP=0 TEST=1 -j8     # same, plus PASS/FAIL self-tests at boot
```

1. Flash, open the serial console. Wait for `[CTRL] READY: press START (GP20)`.
2. **Car on a stand first.** Press and release GP20: the car "follows the
   line". Check both wheels turn forward and the speeds in the CSV lines.
3. Then on the track at a low cruise speed.
4. GP21 = STOP. Hold **both** buttons for 1 s = RESET (also clears a FAULT STOP).

CSV columns: `T,t_ms,seq,state,barcode,letter,faults,vL,vR,encL,encR,dist,lineErr,lineFlags,hump,humpPeak,obstacle,wifi`.
Paste the console log into a spreadsheet to chart a run.

Without the IMU connected, set `S5_REQUIRE_IMU` to 0, otherwise the car
goes to FAULT STOP at start-up (that is the fault table's IMU rule).

## Suggested test order

1. `TEST=1` image on the bare board: S3-U01..U03 (decoder) and S4-U01..U02 (maths) pass without hardware.
2. Each subsystem with its device plugged in (see the bring-up guide).
3. On a stand: START, check wheel directions, speeds, the stall rule (hold a wheel: FAULT STOP within about 0.3 s).
4. Straight line, then the tightest curve at 150 mm/s, then raise the speed.
5. Barcodes by hand, then driving.
6. Obstacle stop, then hump.

## Known limits of the basic version (the complete version fixes these)

- Encoders count edges on one pin only, so they cannot tell direction.
- Fixed IR thresholds, no start-up calibration, no junction detection.
- Obstacle: stops and waits; no profile, no bypass, no line recovery.
- Telemetry only on the console; no MQTT START/STOP/RESET.
- Fault mask kept as a plain word; no watchdog, NeoPixel or buzzer.

## Week 13 baseline

Tag this build `baseline-plain-types`. Sizes from our `make SMP=0` build (text + data + bss, bytes):
S1 589 + 0 + 16, S2 2735 + 0 + 169, S3 2885 + 4 + 1150 (the DMA ring is 1 KB),
S4 1936 + 4 + 28, S5 3811 + 4 + 84. Re-measure on your machine with
`arm-none-eabi-size mtkernel_3/app_program/subsystem*/*.o` and keep the `.map` file.
