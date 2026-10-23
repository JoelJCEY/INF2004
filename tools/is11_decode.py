#!/usr/bin/env python3
"""
is11_decode.py -- laptop side of the IS11 PicoCar MQTT link (complete version).

  python3 is11_decode.py --selftest               check the golden vector
  python3 is11_decode.py --hex "01 32 05 ..."     decode one payload
  python3 is11_decode.py --listen BROKER_IP       print live telemetry (needs paho-mqtt)
  python3 is11_decode.py --send BROKER_IP start   send START / STOP / RESET (QoS 1)

The bit layout must match subsystem1.h / telemetry_pack.c exactly
(little-endian bit stream, least-significant bit first).
"""
import argparse, sys

# (name, bits, offset)  offset: value sent = real + offset
LAYOUT = [
    ("version", 8, 0), ("state", 4, 0), ("barcode", 3, 0), ("hump", 1, 0),
    ("faults", 8, 0), ("ir_black", 3, 0), ("line_lost", 1, 0), ("line_err", 12, 2048),
    ("speed_l", 11, 1024), ("speed_r", 11, 1024), ("distance_cm", 16, 0),
    ("enc_l", 24, 0), ("enc_r", 24, 0), ("hump_peak_mm", 8, 0), ("obstacle_cm", 8, 0),
    ("obstacle_width_cm", 6, 0), ("bypass_side", 2, 0), ("motion_event", 3, 0),
    ("watchdog_reset", 1, 0), ("seq", 8, 0), ("letter", 8, 0),
]
NBYTES = (sum(b for _, b, _ in LAYOUT) + 7) // 8      # 22

STATES = ["INITIALISING", "READY", "LINE FOLLOWING", "BARCODE MANOEUVRE", "HUMP CROSSING",
          "OBSTACLE BYPASS", "MISSION COMPLETE", "STOPPED", "FAULT STOP", "LINE RECOVERY"]
BARCODES = ["none", "TURN LEFT", "TURN RIGHT", "STRAIGHT", "U-TURN", "unrecognised", "END"]
FAULTS = ["stall", "encoder", "IMU", "failed bypass", "impact", "line lost", "no echo", "Wi-Fi loss"]
SIDES = ["none", "left", "right", "blocked"]
EVENTS = ["stationary", "cruising", "accelerating", "braking", "turning", "climbing", "descending", "impact"]

def encode(fields):
    out = bytearray(NBYTES); bit = 0
    for name, nbits, off in LAYOUT:
        v = (fields.get(name, 0) + off) & ((1 << nbits) - 1)
        for i in range(nbits):
            if (v >> i) & 1:
                out[bit >> 3] |= 1 << (bit & 7)
            bit += 1
    return bytes(out)

def decode(data):
    if len(data) != NBYTES or data[0] != 1:
        raise ValueError("not a v1 telemetry payload (%d bytes)" % len(data))
    f = {}; bit = 0
    for name, nbits, off in LAYOUT:
        v = 0
        for i in range(nbits):
            if (data[bit >> 3] >> (bit & 7)) & 1:
                v |= 1 << i
            bit += 1
        f[name] = v - off
    return f

def describe(f):
    faults = [n for i, n in enumerate(FAULTS) if (f["faults"] >> i) & 1]
    obst = "none" if f["obstacle_cm"] == 255 else "%d cm" % f["obstacle_cm"]
    return ("#%3d %-17s v=%5d/%5d mm/s dist=%6.2f m err=%5d bc=%s '%s' hump=%d peak=%d mm "
            "obst=%s w=%d cm side=%s ev=%s faults=%s%s" % (
        f["seq"], STATES[f["state"]] if f["state"] < len(STATES) else f["state"],
        f["speed_l"], f["speed_r"], f["distance_cm"] / 100.0, f["line_err"],
        BARCODES[f["barcode"]] if f["barcode"] < len(BARCODES) else f["barcode"],
        chr(f["letter"]) if 32 <= f["letter"] < 127 else "-", f["hump"], f["hump_peak_mm"],
        obst, f["obstacle_width_cm"], SIDES[f["bypass_side"]], EVENTS[f["motion_event"]],
        ",".join(faults) or "none", " WATCHDOG-RESET" if f["watchdog_reset"] else ""))

GOLDEN_FIELDS = dict(version=1, state=2, barcode=3, hump=0, faults=0x05, ir_black=3, line_lost=0,
                     line_err=-125, speed_l=290, speed_r=310, distance_cm=1234, enc_l=4660,
                     enc_r=4672, hump_peak_mm=42, obstacle_cm=199, obstacle_width_cm=21,
                     bypass_side=2, motion_event=1, watchdog_reset=0, seq=0x82, letter=ord("A"))

COMMANDS = {"start": 1, "stop": 2, "reset": 3}

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--hex")
    ap.add_argument("--listen", metavar="BROKER")
    ap.add_argument("--send", nargs=2, metavar=("BROKER", "CMD"))
    a = ap.parse_args()
    if a.selftest:
        g = encode(GOLDEN_FIELDS)
        print("golden:", " ".join("%02X" % b for b in g))
        assert decode(g) == GOLDEN_FIELDS, "round trip failed"
        print(describe(decode(g)))
        print("selftest OK (%d bytes)" % NBYTES)
    elif a.hex:
        print(describe(decode(bytes.fromhex(a.hex.replace(",", " ")))))
    elif a.listen or a.send:
        import paho.mqtt.client as mqtt
        c = mqtt.Client()
        if a.send:
            c.connect(a.send[0], 1883, 60)
            c.loop_start()
            c.publish("is11/cmd", bytes([1, COMMANDS[a.send[1].lower()]]), qos=1).wait_for_publish()
            c.loop_stop(); print("sent", a.send[1].upper())
        else:
            c.on_message = lambda cl, u, m: print(describe(decode(m.payload)))
            c.connect(a.listen, 1883, 60); c.subscribe("is11/telemetry", 0); c.loop_forever()
    else:
        ap.print_help()

if __name__ == "__main__":
    main()
