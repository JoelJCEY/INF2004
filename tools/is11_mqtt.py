#!/usr/bin/env python3
"""IS11 laptop side of the MQTT wire format v1 (app_program/subsystem1/subsystem1.h).

    python3 tools/is11_mqtt.py selftest                 # golden vector, no broker needed
    python3 tools/is11_mqtt.py listen --broker 192.168.1.10
    python3 tools/is11_mqtt.py stop   --broker 192.168.1.10   # also: start, reset
    python3 tools/is11_mqtt.py map F U A Z --broker 192.168.1.10
        barcode map (IF-15): letters for FORWARD, U-TURN, LEFT, RIGHT, in that
        order.  Sent retained, so the car gets it again after a reboot.

listen/start/stop/reset need:  pip install paho-mqtt
The golden vector below is the same byte string as S1-U01 on the Pico, so the
encoder and this decoder are checked against one reference (design 10.5).
"""
import argparse
import struct
import sys
import time

WIRE_VERSION = 1
TLM_FMT = "<BBIBBhhiiIhHHHI"          # 36 bytes, little-endian
TLM_LEN = struct.calcsize(TLM_FMT)
TOPIC_TLM, TOPIC_CMD, TOPIC_STATUS = "is11/car/tlm", "is11/car/cmd", "is11/car/status"

STATES = ["INITIALISING", "READY", "LINE_FOLLOWING", "BARCODE_MANOEUVRE",
          "HUMP_CROSSING", "OBSTACLE_BYPASS", "MISSION_COMPLETE", "STOPPED",
          "FAULT_STOP", "LINE_RECOVERY"]
BARCODES = ["NONE", "TURN_LEFT", "TURN_RIGHT", "GO_STRAIGHT", "U_TURN",
            "UNRECOGNISED", "END_OF_COURSE", "?"]
FAULTS = ["STALL", "ENCODER", "IMU", "BYPASS", "IMPACT", "LINE_LOST", "NO_ECHO", "WIFI"]
EVENTS = ["STATIONARY", "CRUISING", "ACCELERATING", "TURNING", "CLIMBING",
          "DESCENDING", "IMPACT", "?"]
COMMANDS = {"start": 1, "stop": 2, "reset": 3}

GOLDEN = bytes([
    0x01, 0x07, 0x92, 0x92, 0x01, 0x10, 0x21, 0x41,
    0xD4, 0xFE, 0x2B, 0x01, 0xC0, 0x1D, 0xFE, 0xFF,
    0xF1, 0xFB, 0x09, 0x00, 0xD2, 0x04, 0x00, 0x00,
    0x18, 0xFC, 0xCD, 0x00, 0xB4, 0x00, 0xBE, 0x00,
    0x04, 0x03, 0x02, 0x01])


def field(word, shift, bits):
    return (word >> shift) & ((1 << bits) - 1)


def decode_tlm(buf):
    """Decode one telemetry frame; raises ValueError on a bad frame."""
    if len(buf) != TLM_LEN:
        raise ValueError(f"length {len(buf)}, expected {TLM_LEN}")
    (ver, seq, status, faults, letter, vl, vr, enc_l, enc_r, path,
     line_err, peak, width, closest, uptime) = struct.unpack(TLM_FMT, buf)
    if ver != WIRE_VERSION:
        raise ValueError(f"version {ver}")
    state = field(status, 0, 4)
    if state >= len(STATES):
        raise ValueError(f"reserved state {state}")
    return {
        "seq": seq,
        "state": STATES[state],
        "barcode": BARCODES[field(status, 4, 3)],
        "last_remote": field(status, 7, 2),
        "line_flags": field(status, 9, 4),
        "servo_deg": 30 + 5 * field(status, 13, 5),
        "hump_active": field(status, 18, 1),
        "obstacle": field(status, 19, 1),
        "watchdog_reboot": field(status, 20, 1),
        "bypass_side": ["none", "left", "right", "?"][field(status, 21, 2)],
        "bypass_fails": field(status, 23, 2),
        "motion_event": EVENTS[field(status, 25, 3)],
        "mqtt_link": field(status, 28, 1),
        "faults": [n for i, n in enumerate(FAULTS) if faults & (1 << i)],
        "letter": chr(letter) if 32 <= letter < 127 else "",
        "speed_mm_s": (vl, vr),
        "encoder": (enc_l, enc_r),
        "path_mm": path,
        "line_err": line_err,
        "hump_peak_mm": peak / 10.0,
        "obstacle_mm": (width, closest),
        "uptime_s": uptime / 1000.0,
    }


def encode_cmd(name, seq):
    return bytes([WIRE_VERSION, COMMANDS[name], seq & 0xFF])


WIRE_OP_MAP = 0x10


def encode_map(letters, seq):
    """7-byte barcode map frame: forward, u-turn, left, right (subsystem1.h)."""
    letters = [str(x).strip().upper() for x in letters]
    if len(letters) != 4 or any(len(x) != 1 or not (x.isascii() and x.isalnum())
                                for x in letters):
        raise ValueError("need four single letters A-Z or digits 0-9")
    if len(set(letters)) != 4:
        raise ValueError("the four letters must all be different")
    return bytes([WIRE_VERSION, WIRE_OP_MAP, seq & 0xFF] + [ord(x) for x in letters])


def selftest():
    d = decode_tlm(GOLDEN)
    ok = (d["seq"] == 7 and d["speed_mm_s"] == (-300, 299) and
          d["encoder"] == (-123456, 654321) and d["path_mm"] == 1234 and
          d["line_err"] == -1000 and d["hump_peak_mm"] == 20.5 and
          d["obstacle_mm"] == (180, 190) and d["letter"] == "A" and
          d["faults"] == ["STALL", "LINE_LOST"] and
          d["state"] == "LINE_FOLLOWING" and d["barcode"] == "TURN_LEFT" and
          d["servo_deg"] == 90 and d["mqtt_link"] == 1)
    bad = 0
    for frame in (GOLDEN[:-1], bytes([2]) + GOLDEN[1:]):
        try:
            decode_tlm(frame)
        except ValueError:
            bad += 1
    ok = ok and bad == 2 and encode_cmd("stop", 0x10) == bytes([1, 2, 0x10])
    # same bytes as S1-U04a on the Pico
    ok = ok and encode_map("fZAW", 5) == bytes([1, 0x10, 5, 0x46, 0x5A, 0x41, 0x57])
    print("golden vector:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def client(args):
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        sys.exit("pip install paho-mqtt")
    try:
        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="is11-laptop")
    except AttributeError:          # paho-mqtt 1.x
        c = mqtt.Client(client_id="is11-laptop")
    c.connect(args.broker, args.port, 30)
    return c


def listen(args):
    c = client(args)
    last = [None]

    def on_message(_c, _u, msg):
        now = time.time()
        if msg.topic == TOPIC_STATUS:
            print(f"[status] {msg.payload.decode(errors='replace')}")
            return
        try:
            d = decode_tlm(msg.payload)
        except ValueError as e:
            print(f"[tlm] rejected: {e}")
            return
        gap = "" if last[0] is None else f" gap {1000 * (now - last[0]):.0f} ms"
        last[0] = now
        print(f"[tlm #{d['seq']:3}]{gap} {d['state']:<17} v={d['speed_mm_s']} "
              f"path={d['path_mm']} mm line={d['line_err']:+5} "
              f"bc={d['barcode']}{('/' + d['letter']) if d['letter'] else ''} "
              f"hump={d['hump_peak_mm']} mm faults={d['faults']}")

    c.on_message = on_message
    c.subscribe([(TOPIC_TLM, 0), (TOPIC_STATUS, 1)])
    c.loop_forever()


def send(args, name):
    c = client(args)
    c.loop_start()
    info = c.publish(TOPIC_CMD, encode_cmd(name, int(time.time())), qos=1)
    info.wait_for_publish(5)
    print(f"sent {name} at {time.strftime('%H:%M:%S')}")
    c.loop_stop()
    c.disconnect()


def send_map(args):
    try:
        frame = encode_map(args.letters, int(time.time()))
    except ValueError as e:
        sys.exit(f"map not sent: {e}")
    c = client(args)
    c.loop_start()
    info = c.publish(TOPIC_CMD, frame, qos=1, retain=True)
    info.wait_for_publish(5)
    f, u, l, r = (chr(b) for b in frame[3:])
    print(f"sent barcode map: forward {f}, u-turn {u}, left {l}, right {r} (retained)")
    c.loop_stop()
    c.disconnect()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("action", choices=["selftest", "listen", "start", "stop", "reset", "map"])
    p.add_argument("letters", nargs="*",
                   help="map only: letters for FORWARD U-TURN LEFT RIGHT")
    p.add_argument("--broker", default="127.0.0.1")
    p.add_argument("--port", type=int, default=1883)
    a = p.parse_args()
    if a.action == "selftest":
        sys.exit(selftest())
    if a.action == "listen":
        listen(a)
    elif a.action == "map":
        send_map(a)
    else:
        send(a, a.action)


if __name__ == "__main__":
    main()
