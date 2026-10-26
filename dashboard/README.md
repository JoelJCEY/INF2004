# Buddy 1 MQTT integration

Run the firmware build with `SMP=0 TEST=1 NET=lwip MQTT=1`. Put your own
network and MQTT credentials in the ignored `config/wifi_secrets.h`; do not
commit that file. `config/wifi_secrets.example.h` is only a blank template.

The Pico W connects to the laptop's MQTT broker. `ctrl_task` writes a
telemetry snapshot every 200 ms to `mbf_telemetry`. `comms_task` encodes it
and sends a complete frame to `mbf_mqtt_tx` without waiting. Only the Wi-Fi
service task calls lwIP/MQTT. A full outbound buffer drops an older snapshot
to make space for a newer one. The USB CSV remains a separate debug output.

| Topic | Direction | Payload | QoS |
|---|---|---|---|
| `mtk3smp-pico/telemetry` | Pico to laptop | 36 bytes, version 1 below | 0 |
| `mtk3smp-pico/heartbeat` | Pico to laptop | 2 bytes: version 1, sequence modulo 256, every 1 s | 0 |
| `mtk3smp-pico/command` | Laptop to Pico | 1 byte: `01` START, `02` STOP, `03` RESET | 1 |

The command's other six bits must be zero. MQTT commands must **not** be
published as retained messages. Invalid messages are discarded. STOP has a
dedicated one-byte buffer and wins over START/RESET in a controller cycle.
START works only in READY or STOPPED. RESET uses the physical-button reset
path. An MQTT publish confirmation means the broker received the command;
it does not prove the motors stopped, which requires a physical timing test.

## Telemetry version 1 wire format

This is explicitly written byte by byte; it is not a raw C struct. All
multi-byte numbers are little-endian. Signed values use two's complement.

| Byte offset | Field | Width |
|---|---|---|
| 0 | Format version (=1) | 8 bits |
| 1–4 | Time since boot in ms | unsigned 32 |
| 5–8 | Sequence | unsigned 32 |
| 9 | State in bits 0–3; barcode command in bits 4–6; Wi-Fi up in bit 7 | 4 + 3 + 1 bits |
| 10 | Fault mask | 8 bits |
| 11–12 | Left speed, mm/s | signed 16 |
| 13–14 | Right speed, mm/s | signed 16 |
| 15–18 | Left encoder count | unsigned 32 |
| 19–22 | Right encoder count | unsigned 32 |
| 23–26 | Distance travelled, mm | signed 32 |
| 27–28 | Line error | signed 16 |
| 29 | Line flags | 8 bits |
| 30 | Hump active (bit 0) | 1 bit in byte |
| 31–32 | Hump peak, mm | signed 16 |
| 33–34 | Obstacle range, mm (`-1` means none) | signed 16 |
| 35 | Last barcode letter (ASCII or zero) | 8 bits |

The 36-byte length preserves all fields the group's current controller
records. The tribunal's 14–16 byte figure was an estimate, not a fixed
payload size. Physical ranges for the signed 16-bit measurements still need
validation; readings outside their documented ranges are not published.

## Laptop dashboard

Requires Python 3 and the Mosquitto command-line tools installed under
`C:\Program Files\mosquitto`. In PowerShell, from the repository root:

```powershell
py dashboard\dashboard.py --broker YOUR_LAPTOP_WIFI_IPV4
```

Enter the MQTT password at the prompt. A browser opens at
`http://127.0.0.1:8765/`. The dashboard decodes the telemetry into named
readings and has command buttons. If the laptop IP changes, run it with the
new IP and update `MQTT_BROKER_IP` in your private `wifi_secrets.h` before
rebuilding the firmware. The dashboard runs only on the laptop.

The current package has not been compiled against the Pico cross-compiler or
flashed to hardware here. After building, check the dashboard's live sequence
and confirm a one-byte STOP on the command topic reaches the controller. Test
motion release jitter and physical STOP latency separately against the
tribunal targets before claiming those deadlines are met.
