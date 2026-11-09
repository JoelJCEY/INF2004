# Team and subsystems

Filled in for the IS11 design (Week 6 design review + Tribunal 1).  The
example code from the template has been replaced.

Fill this in at the start of the project and keep it up to date. It is the
single place that says **who owns what**, so that five people can work in
one firmware image without clashing over pins, priorities, or kernel objects.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | Communication: Wi-Fi/MQTT telemetry, remote START/STOP/RESET (`comms_task`) | 2501327 (Daryl) | | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motion Control: motors, PIO encoders, 20 ms PID, moves/turns, stall (`motion_task`) | 2501113 (Joel) | | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | Line & Barcode Sensing: IR ADC+DMA, line error, Code 39, junctions (`line_task`) | 2500991 (Gillton) | | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | Terrain Monitoring: LSM303DLHC, humps, peak height, impact (`imu_task`) | 2501315 (Xavier) | | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Obstacle Scanning (`scan_task`) + Vehicle Controller (`ctrl_task`, integration lead) | 2500118 (Marcus) | | `app_program/subsystem5/` | `testing/subsystem5/` |

Team build: `make SMP=0 NET=lwip MQTT=1 -j8` (design 1.0 uses the single-core
build; MQTT needs `config/wifi_secrets.h`, copied from the example).

Shared files (`app_program/app_main.c`, `app_program/app.h`, this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

**Why:** two subsystems using the same pin is one of the most common and
hardest-to-find mistakes in a team project. The code builds without any
error, but the hardware misbehaves. Each subsystem can pass its own tests
and then fail once they are combined, and two outputs driving the same pin
against each other can damage it. Writing the claim down first means a clash
is spotted when someone edits this table, not hours later on the bench.

**No instructor check needed.** The team keeps this table itself. You do not
need the instructor, or the rest of the team, to check or approve a claim:
if the row is empty, fill it in and commit. If two people claim the same
pin, they agree between themselves who moves. Only the *reserved* rows are
fixed.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | S5 scan_task | HC-SR04+ TRIG (Grove 1). **Unplug the ultrasonic for `CRASH_REPORT=1`** |
| GP1 | S5 scan_task | HC-SR04+ ECHO, timed by PWM slice 0 (B input, 31 Oct) (Grove 1, 3.3 V module only) |
| GP2 | S2 motion_task | left encoder A (Grove 2, PIO) |
| GP3 | S2 motion_task | left encoder B (Grove 2, PIO) |
| GP4 | S2 motion_task | right encoder A (Grove 3, PIO) |
| GP5 | S2 motion_task | right encoder B (Grove 3, PIO) |
| GP6 | | spare (Grove 5 must stay EMPTY: it shares GP26) |
| GP7 | | spare (loose wire on the Grove 7 cable, insulated) |
| GP8 | S2 motion_task | M1A left motor, PWM 4A |
| GP9 | S2 motion_task | M1B left motor, PWM 4B |
| GP10 | S2 motion_task | M2A right motor, PWM 5A |
| GP11 | S2 motion_task | M2B right motor, PWM 5B |
| GP12 | S5 scan_task | scan servo, PWM 6A, 50 Hz (Servo port 1) |
| GP13 | | spare |
| GP14 | | spare |
| GP15 | | spare |
| GP16 | S4 imu_task | I2C0 SDA, LSM303DLHC (Grove 4) |
| GP17 | S4 imu_task | I2C0 SCL, LSM303DLHC (Grove 4) |
| GP18 | S5 ctrl_task | status NeoPixels (PIO WS2812) |
| GP19 | | spare |
| GP20 | S5 ctrl_task | START button (on board) |
| GP21 | S5 ctrl_task | STOP button (on board) |
| GP22 | S5 ctrl_task | piezo buzzer, PWM 3A |
| GP26 | S3 line_task | IR_L, ADC0 (Grove 6) |
| GP27 | S3 line_task | IR_C, ADC1 (Grove 6) |
| GP28 | S3 line_task | IR_R, ADC2 (Grove 7, VBAT-sense jumper OPEN) |

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "environment sensing", "motor control", "user interface" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "environment sensing" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: the "environment sensing" subsystem contains the I2C
sensor code, and owns I2C0. Write it as `drv_<name>.c` / `drv_<name>.h` in
that subsystem's folder, never under `device/`, and call it only from the
peripheral's owning task. The kernel's own drivers in `device/` (serial,
I2C, ADC) may be used instead; switching one on in `config/config_device.h`
changes shared configuration, so record it in the table below first.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** That task can be in any subsystem, and one subsystem may own
several peripherals (one owning task each, or one task owning all of them).
Every other task, in the same subsystem or another one, asks the owning task
through a kernel object (section 5) and never touches that peripheral itself.

*Why:* this port runs tasks on both cores at the same time. If two tasks
drive the same I2C or UART block at once, their transfers get mixed up, and
the drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART1 | | | |
| I2C0 | S4 | imu_task (`drv_i2c0.c`, register level) | GP16, GP17 |
| I2C1 | | | |
| SPI0 | | | |
| SPI1 | | | |
| ADC | S3 | line_task (`drv_ir_adc.c`, round-robin ADC0-2 + DMA). Kernel ADC driver switched OFF (`DEVCNF_USE_ADC 0`) | GP26–GP28 |
| DMA (1 channel, claimed at run time) | S3 | line_task | — |
| PIO: encoder program at offset 0, 2 SMs | S2 | motion_task (loaded first in `usermain`) | GP2–GP5 |
| PIO: WS2812, 1 SM (PIO1 preferred) | S5 | ctrl_task | GP18 |
| PWM slices 4, 5 | S2 | motion_task | GP8–GP11 |
| PWM slice 6 | S5 | scan_task | GP12 |
| PWM slice 0 | S5 | scan_task (echo timer, level mode, 1 MHz) | GP1 |
| PWM slice 3 | S5 | ctrl_task | GP22 |
| IO_IRQ_BANK0 (IRQ 13) | S5 | `us_echo_isr` → wakes scan_task | GP1 |
| Watchdog | S5 | ctrl_task (only feeder, 1 s) | — |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in); IS11 code runs inside it via `subsystem1/comms_bridge.c` | GP23–25, GP29 |

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions, not even from a
"WiFi subsystem".** The calls are not safe from any other task, on either core.

What students *can* do:

- **Read the connection state** from any task, which is always safe:
  ```c
  #include "cyw43_utk.h"
  T_CYW43_UTK_STATUS st;
  cyw43_utk_get_status(&st);   /* st.ready, st.link_status, st.ip_addr, ... */
  ```
- **Send or receive data over the network** by adding code that *runs
  inside the service task*. The MQTT client is in
  `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`. It publishes a heartbeat
  today. A "cloud / reporting" subsystem would add a kernel object
  (e.g. a message buffer) that application tasks fill. The MQTT code then
  reads it with `TMO_POL` (never blocking) and publishes it.
  Because this changes shared library code, **agree it with the team and
  the instructor first**, and record it as an interface in section 5.
  It needs the `NET=lwip MQTT=1` build (README section 3).

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4). Each subsystem gets its own band by default;
change it only by team agreement. Write down every task you create.

Agreed order (design Section 3): ctrl > motion > line > scan > imu > comms.
The numbers live in `app.h` (`APP_PRI_*`).  Open proposal F7: swap scan
and imu after the A1 jitter logs.

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| 5 | 10, 13 | ctrl_task – 10 – any; scan_task – 13 – any |
| 2 | 11 | motion_task – 11 – any |
| 3 | 12 | line_task – 12 – any |
| 4 | 14 | imu_task – 14 – any |
| 1 | 20 | comms_task – 20 – any (lowest, NFR3) |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `subsystemN.h`; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 | S5 ctrl → S1 comms | message buffer (owned by S1), TMO_POL, drop newest when full | `mbf_telemetry` | `TLM_SNAPSHOT` (`subsystem1.h`), every 200 ms | design 6.1, 10.4 |
| IF-02 | all → S5 ctrl | event flag bits 0-7 (owned by S5) | `flg_sys` | `FLT_xxx` fault reports, latched by ctrl until RESET | design 5.1, 10.4 |
| IF-03 | S1 comms → S5 ctrl | message buffer + `EVF_REMOTE` (owned by S5) | `mbf_remote` | `REMOTE_MSG` START/STOP/RESET (2 bits) | design 6.1 |
| IF-04 | S5 ctrl → S5 scan (any reader) | struct copy under mutex `TA_INHERIT` (owned by S5) | `mtx_mission` via `ctrl_get_mission()` | `MISSION_STATE` | design 4, F5 |
| IF-05 | S5 ctrl → S2 motion | message buffer (owned by S2) via the motion API | `mbf_motion_cmd` | `MOTION_CMD`: `moveForward/moveBackward/turnLeft/turnRight/stop` + follow, limit, lock, reset, self-test | design 11.2, P2 |
| IF-06 | S2 motion → S5 ctrl | message buffer + `EVF_MOTION` (owned by S5) | `mbf_motion_status` | `MOTION_STATUS` (done, stopped, stall, self-test) | design 8.3 gap 1 |
| IF-07 | S2 motion → S3, S4, S5 | aligned 32-bit words, single writer | `motion_odo_path_01mm`, `motion_odo_speed`, `motion_enc_count[]` | odometry for 200 mm rules, hump profile, telemetry | design 11.1 |
| IF-08 | S3 line → S2 motion | one aligned 32-bit word, single writer | `line_latest` | line error + flags + timestamp (latest value, F6) | design 4, F6 |
| IF-09 | S3 line → S5 ctrl | message buffer + `EVF_NAV` (owned by S5) | `mbf_nav` | `NAV_MSG`: barcode, junction, line lost/found | design 4 |
| IF-10 | S5 ctrl ↔ S3 line | request word (writer ctrl) + result word (writer line) | `line_cal_request`, `line_cal_result` | IR calibration sweep in INITIALISING | design 5 |
| IF-11 | S4 imu → S5 ctrl | message buffer + `EVF_TERRAIN` (owned by S5) | `mbf_terrain` | `TERRAIN_MSG`: hump start/end + peak, impact | design 4 |
| IF-12 | S4 imu → S5 scan | event flag state bit, writer imu only | `EVF_HUMP_ACTIVE` in `flg_sys` | ignore echoes on a hump (bounded by 400 mm) | design 8.3, F10 |
| IF-13 | S5 scan → S5 ctrl | message buffer + `EVF_OBSTACLE` | `mbf_obstacle` | `OBSTACLE_MSG`: alert, profile + side, sensor fault, clearance lost | design 6.5 |
| IF-14 | ECHO ISR → S5 scan | one aligned 32-bit word, ISR only writer + `tk_wup_tsk` | `us_echo_word` | 13-bit duration + valid + rose + sequence | design 10.4 rule 3 |
| IF-15 | dashboard → S1 bridge → S3 line, S5 ctrl | 7-byte MQTT frame on `is11/car/cmd` (retained, QoS 1) → one aligned 32-bit word, writer `comms_bridge_on_command()` only | `app_barcode_map` (`app.h`) | letters for FORWARD, U-TURN, LEFT, RIGHT; ctrl stores each decoded letter and carries it out at the next cross-junction (`NAV_JUNCTION_CROSS`) | demo rule, 2 Nov |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. The numbers below are the example's usage.

| Object | Limit | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|
| Task | 32 | 1 | 1 | 1 | 1 | 2 |
| Semaphore | 16 | | | | | |
| Event flag | 16 | | | | | 1 |
| Mailbox | 12 | | | | | |
| Mutex | 12 | | | | | 1 |
| Message buffer | 12 | 2 | 1 | | | 5 |
| Fixed memory pool | 12 | | | | | |
| Variable memory pool | 12 | | | | | |
| Cyclic handler | 12 | | 1 | 1 | 1 | |
| Alarm handler | 12 | | | | | |
| Interrupt handler | — | | | | | 1 (IRQ 13) |

(The kernel's USB and WiFi service tasks also use a few of these.)
