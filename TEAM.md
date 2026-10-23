# Team and subsystems

The tables are pre-filled with the **example** in `app_program/` (marked
*example*). Replace those rows with your own as you replace the example code.

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
| 1 | Wi-Fi, MQTT telemetry and commands (Buddy 1) | | | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motion control: motors, encoders, PID (Buddy 2) | | | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | IR line following and barcode decoding (Buddy 3) | | | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU: tilt, hump height, impact (Buddy 4) | | | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Servo + ultrasonic scanning, obstacle bypass, ctrl_task (Buddy 5) | | | `app_program/subsystem5/` | `testing/subsystem5/` |

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
| GP0 | | (taken whenever the team builds with `CRASH_REPORT=1`) |
| GP1 | | (taken whenever the team builds with `CRASH_REPORT=1`) |
| GP2 | S2 | left encoder A (Grove 2), PIO quadrature |
| GP3 | S2 | left encoder B (Grove 2), PIO quadrature |
| GP4 | S2 | right encoder A (Grove 3), PIO quadrature |
| GP5 | S2 | right encoder B (Grove 3), PIO quadrature |
| GP6 | S5 | HC-SR04+ TRIG (Grove 5; its GP26 wire left unconnected) |
| GP7 | S5 | HC-SR04+ ECHO (Grove 7, cable shared with IR_R), edge interrupt |
| GP8 | S2 | left motor M1A, PWM 4A |
| GP9 | S2 | left motor M1B, PWM 4B |
| GP10 | S2 | right motor M2A, PWM 5A |
| GP11 | S2 | right motor M2B, PWM 5B |
| GP12 | S5 | servo (servo port 1), PWM 6A, 50 Hz |
| GP13 | | |
| GP14 | | |
| GP15 | | |
| GP16 | S4 | I2C0 SDA, IMU (Grove 4) |
| GP17 | S4 | I2C0 SCL, IMU (Grove 4) |
| GP18 | S5 | on-board NeoPixel (PIO WS2812), ctrl_task |
| GP19 | | |
| GP20 | S5 | on-board START button, ctrl_task (pressed = 0) |
| GP21 | S5 | on-board STOP button, ctrl_task (hold both 1 s = RESET) |
| GP22 | S5 | on-board buzzer, PWM 3A, ctrl_task |
| GP26 | S3 | IR left, ADC0 (Grove 6) |
| GP27 | S3 | IR centre, ADC1 (Grove 6) |
| GP28 | S3 | IR right, ADC2 (Grove 7) |

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
| I2C0 | S4 | imu_task | GP16, GP17 (kernel driver "iica") |
| I2C1 | | | |
| SPI0 | | | |
| SPI1 | | | |
| ADC (register level, round-robin, free-running 3 kHz) | S3 | line_task | GP26–GP28; never ADC3/GP29. Do not open the kernel driver "adca" in this image |
| DMA (one channel, claimed with dma_claim_unused_channel in Wi-Fi builds) | S3 | line_task | ADC FIFO -> 1 KB ring |
| PWM slices 4, 5 (motors) | S2 | motion_task | GP8–GP11 |
| PIO (2 state machines + 24 instructions at offset 0, whichever block is free; Wi-Fi uses the other) | S2 | motion_task | GP2–GP5 |
| PIO (1 state machine, 4 instructions) + PWM slice 3 | S5 | ctrl_task | GP18 NeoPixel, GP22 buzzer |
| IO_IRQ_BANK0 (GPIO interrupt) | S5 | us_echo_isr -> scan_task | GP7 |
| Watchdog | S5 | ctrl_task | fed every 50 ms, 1 s timeout |
| PWM slice 6 (servo) | S5 | scan_task | GP12 |
| TIMER (read only, TIMERAWL) | *shared, read only* | S5 echo timing | |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

**Shared configuration changes made for the PicoCar (agreed by the team):**

| File | Change | Why | Requested by |
|---|---|---|---|
| `config/config_device.h` | `DEVCNF_USE_IIC` 0 → 1 | IMU uses the kernel I2C driver | S4 |
| `device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h` | I2C0 pins 8/9 → 16/17 | GP8/GP9 are the left motor; the IMU is on Grove 4 | S4 |
| `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c` | telemetry + command hooks (weak defaults), topics `is11/telemetry`, `is11/cmd` | MQTT may only run in the service task | S1 (needs instructor agreement) |
| `app_program/sdk_shim/` (new folder) | the only file with Pico SDK PIO headers | SDK and kernel headers clash | S5 (integration) |

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

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| 5 (integration) | 10 | ctrl_task – 10 – any (SMP=0: one core) |
| 2 | 11 | motion_task – 11 |
| 3 | 12 | line_task – 12 |
| 5 | 13 | scan_task – 13 |
| 4 | 14 | imu_task – 14 |
| 1 | 15 | comms_task – 15 |

Order agreed in the design review (Section 3): ctrl > motion > line > scan > imu > comms.
The kernel's USB console (3) and Wi-Fi service (4) are above all of these.

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `subsystemN.h`; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 | S5 (ctrl) → S2 | message buffer (owned by S2) + motion API | `mbf_motion_cmd` | `MOTION_CMD` in `subsystem2.h`; only ctrl_task sends | |
| IF-02 | S2 → S5 | message buffer + flag bit `FLG_MOTION` (owned by S5) | `mbf_motion_st` | `MOTION_STATUS_MSG` (move done, line found) | |
| IF-03 | S3 → S2, S5 | latest value + timestamp (one aligned word each) | `s3_line_get()` | line error, flags | |
| IF-04 | S3 → S5 | message buffer + `FLG_NAV` | `mbf_nav` | `NAV_MSG`: barcode, line lost/found, right junction | |
| IF-05 | S4 → S5 | message buffer + `FLG_TERRAIN` | `mbf_terrain` | `TERRAIN_MSG`: hump start/end, impact, IMU fault | |
| IF-06 | S4 → S5 (scan) | event flag state bit (set/cleared by S4 only) | `FLG_HUMP_ACTIVE` in `flg_ctrl` | hump in progress | |
| IF-07 | S5 (scan) → S5 (ctrl) | message buffer + `FLG_OBSTACLE` | `mbf_obstacle` | `OBSTACLE_MSG`: alert, profile, clear, sensor fault | |
| IF-08 | S5 (ctrl) → S1 | message buffer (owned by S1) | `mbf_telemetry` | `S1_TELEMETRY` every 200 ms | |
| IF-09 | S2 → all | aligned words, single writer | `s2_distance_mm`, `s2_speed_*`, `s2_enc_*` | odometer, speeds, counts | |
| IF-10 | S1 (MQTT hook) → S5 | message buffer + `FLG_REMOTE` | `mbf_remote` | `REMOTE_MSG`: START/STOP/RESET | |
| IF-11 | any → S5 | event flag fault bits 16–23 (any task sets, ctrl clears) | `s5_raise_fault()` | 8-bit fault mask (Section 5.1) | |
| IF-12 | S1 → kernel MQTT client | message buffer, polled by the service task | `mbf_outbox` | packed telemetry v1, 22 bytes | |
| IF-13 | S5 (ctrl) → S3 | one word, single writer | `s3_cal_request` / `s3_cal_result` | calibration sweep | |
| IF-14 | S5 (ctrl) → S5 (scan) | one word, single writer | `s5_scan_request` | watch / profile / hold | |
| IF-15 | ECHO ISR → scan_task | one aligned word, ISR is the only writer + `tk_wup_tsk` | `us_word` | echo duration, counters | |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. The numbers below are this version's usage.

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
| Cyclic handler | 12 | | 1 | 1 | 1 |  |
| Alarm handler | 12 | | | | | |

(The kernel's USB and WiFi service tasks also use a few of these.)
