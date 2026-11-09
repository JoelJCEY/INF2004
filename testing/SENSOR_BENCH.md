# Sensor bench (all subsystems)

Guided hand test of every sensor. Code: `app_program/sensor_bench.c`.
It runs in the TEST build after the unit tests: flash
`mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt_test.uf2` (or build with `make ... TEST=1`).

1. Put the car flat on white floor, on its wheels, and power it on. Don't press START.
2. Open the serial monitor or the dashboard (car log). Follow the blue `[BENCH]` lines.
3. Each hand step waits 25 s. A step you skip is reported as FAIL.
4. Copy every `[TEST] ...-B..` line into the table below.

The motors never run. About 4 minutes in total.

| ID | What you do | PASS means | If it FAILS |
|---|---|---|---|
| S3-B01 | nothing | ADC samples at 3000 Hz +-5% | DMA/ADC setup wrong |
| S3-B02L/C/R | car on white, still | each sensor mean < 1500, range < 250 | sensor dirty, too high, or loose wire (noisy) |
| S3-B03 | black under LEFT sensor only | LEFT rises >= 800 and the most | message names the sensor that reacted: wires swapped |
| S3-B04 | black under CENTRE only | CENTRE rises >= 800 and the most | as above |
| S3-B05 | black under RIGHT only | RIGHT rises >= 800 and the most | as above |
| S2-B01 | LEFT wheel forward 1 turn | counts up, within 25% of MOTION_COUNTS_PER_REV, other wheel < 5% | backward: flip MOTION_ENC_L_INVERT; other wheel counted: encoder cables swapped |
| S2-B02 | RIGHT wheel forward 1 turn | same for the right wheel | flip MOTION_ENC_R_INVERT / cables |
| S2-B03 | both wheels backward 1/2 turn | both count down | one encoder channel dead |
| S4-B01a | car flat, still | >= 120 I2C reads in 3 s, none failed | IMU cable (SDA GP16, SCL GP17) |
| S4-B01b | " | gravity 1000 +-100 mg | wrong range setting or broken chip |
| S4-B01c | " | noise < 30 mg per axis | vibration or loose mounting |
| S4-B01d | " | level reference from power-on matches now | car was not flat at power-on |
| S4-B02 | lift the FRONT 3 cm | X moves, and reads "front higher" | "nose-DOWN": set IMU_FWD_SIGN to -1; "Y moved": IMU mounted sideways |
| S4-B03 | lift the LEFT side 3 cm | Y moves most | axes swapped |
| S4-B04 | knock / shake | jolt > 400 mg seen | accelerometer not updating |
| S4-B05 | turn the car one full circle | magnetometer x and y ranges > 100 | magnetometer not answering |
| S5-B01 | book at 20 cm from the ultrasonic | 200 +-30 mm, spread <= 30 mm, >= 75% echoes | "no echo": TRIG GP0 / ECHO GP1, 3.3 V supply |
| S5-B02 | book at 40 cm | 400 +-30 mm, spread <= 30 mm | as above; wrong distance: sensor angled |

The bench logic itself is checked on a PC: `sh tools/host_test/run.sh` runs it
against a scripted user ("good" hardware must give 20 PASS, "bad" hardware
with swapped IR wires, a reversed encoder, a flipped IMU and a dead ultrasonic
must give exactly those 6 FAILs).

## Results

| Date | Tester | ID | PASS/FAIL | Numbers printed |
|---|---|---|---|---|
| | | | | |
