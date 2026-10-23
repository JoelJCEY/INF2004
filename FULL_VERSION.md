# PicoCar IS11, complete version

The design from the report (Sections 1–12), built on top of the basic
version. Move to this once the basic version drives.

> **Status:** compiles with no warnings in all five builds: default,
> `TEST=1`, `CONSOLE=uart WIFI=none`, `NET=lwip MQTT=1` and
> `NET=lwip MQTT=1 TEST=1`. The only `-Wextra` warning is inside the SDK's
> own `pio.h`. **Nothing has run on the car yet.** Checked on a PC:
> - the Code 39 decoder (same as basic);
> - the packed telemetry: the C packer and an independent Python packer
>   produce the same 22-byte golden vector;
> - the speed loop and stall rule in a simulated motor: it settles on
>   target for weak and strong motors, soft start never trips the stall
>   rule, and a blocked wheel is caught after about 0.34 s.

## What it adds to the basic version

| Part | Added |
|---|---|
| motion_task | **PIO quadrature encoders** (pico-examples program, signed ×4 counts, no interrupts); soft start (`S2_ACCEL_MM_S2`); integrator reset on every mode change; "move/turn until line" for the bypass and recovery; encoder fault (one driven wheel silent for 500 ms). Faults reported as fault bits. |
| line_task | **Calibration** at the first START: ctrl_task turns the car ±25° over the line while line_task records min/max per sensor. **Right-junction** detection (isolated IR_R stripe ≥ `S3_JUNCTION_MIN_MM`). |
| scan_task | **ECHO edge interrupt** (the only application ISR; it is the only writer of one 32-bit result word and wakes scan_task with `tk_wup_tsk`). On an alert: coarse scan 30–150° in 30° steps, fine scan in 5° steps, then a profile (closest point, width, reach left/right of the centre line, free space beyond each edge, recommended side). Hump-ignore window bounded to 300 mm. |
| imu_task | Tilt frozen while accelerating/braking; hump profile rebuilt from encoder distance (`h_front(s) = h_rear(s) + L·sinθ`), peak confirmed after a 3 mm drop, REAR WHEELS CROSSING; motion events and turn rate. |
| ctrl_task | Barcode executed at the next right junction (or after 250 mm); **draft obstacle bypass** (see below); **LINE RECOVERY** (turn toward the last side up to 45°, then the other way); MQTT START/STOP/RESET; fault bits (any task sets, ctrl latches until RESET); mission-state mutex with TA_INHERIT; **hardware watchdog**; NeoPixel colour per state; buzzer on barcode/fault. |
| comms_task | **Packed telemetry v1** (22 bytes, explicit masks/shifts, offset binary for signed fields, version byte) in a 16-deep outbox that drops the oldest when offline; Wi-Fi loss raised as a fault bit (reported only). |

## Shared changes that need agreement first

1. **`lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`**: two hooks with weak
   defaults (so the file still works for anyone who does not define them),
   a 200 ms telemetry publish on `is11/telemetry` (QoS 0), and a command
   subscription on `is11/cmd` (QoS 1). The template says this needs team
   **and instructor** agreement. The diff is in `docs/mqtt_utk.patch`.
2. **`app_program/sdk_shim/`**: the only file that includes Pico SDK PIO
   headers. The SDK and kernel headers clash, the same reason the
   template keeps its Wi-Fi code separate.
3. **Priorities** 10–15 in the design order (TEAM.md section 4).
4. Same I2C config changes as the basic version.

## Draft parts: review before trusting them

- **Obstacle bypass** (the report's still-open sub-statechart): turn 90° to
  the recommended side, out by (reach + half car width + margin), turn
  back, along (closest + assumed depth 150 mm + car length), turn toward
  the line, move until IR_L/IR_C sees it, turn to align, follow. Both
  sides blocked: reverse 100 mm and rescan once. Second failed attempt =
  FAULT STOP. Every distance is a constant in `subsystem5.h`. It does not
  yet tell the original line from a branch.
- **Junction width**: a branch line crossed by IR_R may be only as wide as
  a wide bar (about 18 mm), so `S3_JUNCTION_MIN_MM` (40) must be checked
  on the real mat. If a junction cannot be told apart this way, set it
  very high: barcodes then execute after `S5_EXEC_MAX_MM`.
- **Barcode letters and execution point**: still to confirm with staff.
- **Watchdog**: armed only if the watchdog tick is running (the boot log
  says which). Timeout 1 s, fed every 50 ms.
- **Profile time**: coarse + fine scan is about 2–2.5 s with the car
  stopped (design estimate), before the bypass legs. Time the whole cycle
  against NFR2 (< 5 s). The fine scan only goes 15° past the outer coarse
  hits; widen it if obstacle edges are missed.

## Constants to fill in

Everything in the basic list, plus:

| File | Constant | Note |
|---|---|---|
| `subsystem2.h` | `S2_UM_PER_EDGE` | Now per **quadrature count** (×4 edges): about ¼ of the basic value. |
| `subsystem3.h` | `S3_JUNCTION_MIN_MM`, `S3_CAL_MIN_SPAN` | Junction stripe length; minimum black–white span to accept calibration. |
| `subsystem4.h` | `S4_ACCEL_GATE_MM_S2` | Acceleration above which tilt is ignored. |
| `subsystem5.h` | `S5_SERVO_HIGH_IS_LEFT` | 1 if a higher servo angle points left. |
| | `S5_CAR_HALF_WIDTH_MM`, `S5_CAR_LENGTH_MM`, `S5_BYPASS_*` | Bypass geometry. |
| | `S5_PROFILE_RANGE_MM`, `S5_CLEAR_NEED_MM` | What counts as the obstacle, and how much free space a side needs. |

## MQTT

```sh
make SMP=0 NET=lwip MQTT=1 -j8          # needs config/wifi_secrets.h
mosquitto -v                            # broker on the laptop
python3 tools/is11_decode.py --selftest        # golden vector check
python3 tools/is11_decode.py --listen 192.168.1.10
python3 tools/is11_decode.py --send 192.168.1.10 stop
```

The decoder needs `pip install paho-mqtt` for `--listen` and `--send`.
Command payload: 2 bytes, `01 0c` where c = 1 START, 2 STOP, 3 RESET.
Everything else is rejected and counted (`s1_cmds_bad`).

## Suggested test order

1. `TEST=1` on the bare board: S1-U01..U03 (packing), S3-U01..U03 (decoder), S4-U01..U02 (maths).
2. Boot log: PIO block and state machines for the encoders, DMA channel, watchdog armed or not.
3. Turn each wheel by hand: counts must go **up** forward and **down** backward.
4. Calibration sweep on the line, then the basic test order.
5. Obstacle: first check the printed profile with the car held still, then let it bypass.
6. MQTT: STOP latency (NFR4: ≤ 500 ms in 20/20 trials), then a full run with telemetry.

## Week 13 comparison

Sizes from our `make SMP=0` build (text + data + bss, bytes):
S1 2314 + 0 + 32, S2 3398 + 0 + 193, S3 3509 + 28 + 1175, S4 2440 + 4 + 549, S5 7769 + 12 + 314
(the complete version has more features, so compare message sizes, not code size).
Packed telemetry: 22 bytes. The basic plain snapshot is `sizeof(S1_TELEMETRY)` = 68 bytes,
and a text/CSV line is about 80–100 bytes. Measure both on your machine for the report.
