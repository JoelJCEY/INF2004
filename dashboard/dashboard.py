"""Local browser dashboard for Buddy 1's 36-byte MQTT telemetry (version 1).

Uses the Mosquitto command-line tools already installed on the laptop. No
additional Python packages or MQTT WebSocket listener are required.

Run on Windows: py dashboard/dashboard.py --broker YOUR_LAPTOP_IP
Open: http://127.0.0.1:8765/
"""

from __future__ import annotations

import argparse
import getpass
import json
import os
from pathlib import Path
import queue
import struct
import subprocess
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import webbrowser

TELEMETRY_TOPIC = "mtk3smp-pico/telemetry"
COMMAND_TOPIC = "mtk3smp-pico/command"
STATE_NAMES = (
    "INITIALISING", "READY", "LINE FOLLOWING", "BARCODE MANOEUVRE",
    "HUMP CROSSING", "OBSTACLE BYPASS", "MISSION COMPLETE", "STOPPED",
    "FAULT STOP", "LINE RECOVERY",
)
BARCODE_NAMES = (
    "NONE", "TURN LEFT", "TURN RIGHT", "STRAIGHT", "U-TURN",
    "UNRECOGNISED", "END OF COURSE",
)

PAGE = """<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>IS11 Robot Telemetry</title>
<style>
body{font:16px system-ui;margin:0;background:#f3f6fa;color:#16243a}
main{max-width:850px;margin:35px auto;padding:0 20px}
h1{margin-bottom:5px}p{color:#46566b}
.card{background:white;border:1px solid #dbe2ec;border-radius:12px;
  padding:20px;margin:18px 0;box-shadow:0 2px 8px #dbe2ec66}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:16px}
.item small{display:block;color:#5a6b80}.item strong{font-size:1.25rem}
.ok{color:#127544}.bad{color:#ac2b2b}
button{border:0;border-radius:8px;padding:12px 18px;margin:5px;
  background:#184b82;color:white;font-weight:700;cursor:pointer}
button.stop{background:#bd3030}button:disabled{opacity:.5}
</style><main>
<h1>IS11 robot telemetry</h1>
<p>Live readings received from the Pico W through the laptop MQTT broker.</p>
<div class="card"><strong id="connection">Waiting for readings…</strong>
<div id="last">No telemetry received yet.</div></div>
<div class="card grid" id="readings"></div>
<div class="card"><strong>Remote commands</strong><p>START moves the robot only
when the controller is READY or STOPPED. STOP takes priority.</p>
<button onclick="sendCommand('START')">START</button>
<button class="stop" onclick="sendCommand('STOP')">STOP</button>
<button onclick="sendCommand('RESET')">RESET</button>
<div id="command-status"></div></div>
<script>
const fields=[['state','Vehicle state'],['barcode','Barcode command'],
 ['last_letter','Last barcode letter'],['fault_mask','Fault bits'],
 ['left_speed_mm_s','Left wheel (mm/s)'],['right_speed_mm_s','Right wheel (mm/s)'],
 ['left_encoder','Left encoder'],['right_encoder','Right encoder'],
 ['distance_mm','Distance travelled (mm)'],['line_error','Line error'],
 ['line_flags','Line flags'],['hump_active','On hump'],
 ['hump_peak_mm','Hump peak (mm)'],['obstacle_mm','Obstacle (mm)'],
 ['wifi_up','Pico Wi-Fi status'],['sequence','Message number']];
async function refresh(){
 try{
  const response=await fetch('/api/telemetry',{cache:'no-store'});
  const x=await response.json();
  const online=x.received_at && (Date.now()/1000-x.received_at)<2;
  document.getElementById('connection').textContent=online?'Receiving MQTT telemetry':'No recent telemetry';
  document.getElementById('connection').className=online?'ok':'bad';
  document.getElementById('last').textContent=x.received_at?
   'Last update: '+new Date(x.received_at*1000).toLocaleTimeString():
   'Check that the Pico W and MQTT broker are running.';
  if(x.frame){document.getElementById('readings').innerHTML=fields.map(
   ([key,name])=>'<div class="item"><small>'+name+'</small><strong>'+x.frame[key]+'</strong></div>'
  ).join('');}
 }catch(e){document.getElementById('connection').textContent='Dashboard backend unavailable';}
}
async function sendCommand(name){
 if(name==='START'&&!confirm('Start the robot motors?'))return;
 const status=document.getElementById('command-status');
 try{
  const result=await fetch('/api/command',{method:'POST',
   headers:{'Content-Type':'application/json'},body:JSON.stringify({command:name})});
  const data=await result.json();
  status.textContent=data.message;
 }catch(e){status.textContent='Could not send '+name+': '+e;}
}
setInterval(refresh,300);refresh();
</script></main></html>"""


def decode_frame(payload: bytes) -> dict:
    if len(payload) != 36 or payload[0] != 1:
        raise ValueError("Expected 36-byte version 1 telemetry frame")
    packed = payload[9]
    state = packed & 0x0F
    barcode = (packed >> 4) & 0x07
    letter = payload[35]
    return {
        "uptime_ms": struct.unpack_from("<I", payload, 1)[0],
        "sequence": struct.unpack_from("<I", payload, 5)[0],
        "state": STATE_NAMES[state] if state < len(STATE_NAMES) else f"UNKNOWN {state}",
        "barcode": BARCODE_NAMES[barcode] if barcode < len(BARCODE_NAMES) else f"UNKNOWN {barcode}",
        "last_letter": chr(letter) if 32 <= letter < 127 else "-",
        "wifi_up": "UP" if packed & 0x80 else "DOWN",
        "fault_mask": f"0x{payload[10]:02X}",
        "left_speed_mm_s": struct.unpack_from("<h", payload, 11)[0],
        "right_speed_mm_s": struct.unpack_from("<h", payload, 13)[0],
        "left_encoder": struct.unpack_from("<I", payload, 15)[0],
        "right_encoder": struct.unpack_from("<I", payload, 19)[0],
        "distance_mm": struct.unpack_from("<i", payload, 23)[0],
        "line_error": struct.unpack_from("<h", payload, 27)[0],
        "line_flags": f"0x{payload[29]:02X}",
        "hump_active": "YES" if payload[30] & 1 else "NO",
        "hump_peak_mm": struct.unpack_from("<h", payload, 31)[0],
        "obstacle_mm": struct.unpack_from("<h", payload, 33)[0],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--broker", required=True, help="Laptop IP used by the Pico W")
    parser.add_argument("--username", default="pico_client")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--dashboard-port", type=int, default=8765)
    args = parser.parse_args()
    password = getpass.getpass("MQTT password: ")
    install_dir = Path(os.environ.get("ProgramFiles", "C:/Program Files")) / "mosquitto"
    sub_bin = install_dir / "mosquitto_sub.exe"
    pub_bin = install_dir / "mosquitto_pub.exe"
    if not sub_bin.is_file() or not pub_bin.is_file():
        raise SystemExit(f"Mosquitto tools not found in {install_dir}")

    shared = {"frame": None, "received_at": None}
    lock = threading.Lock()
    subscriber = subprocess.Popen(
        [str(sub_bin), "-h", args.broker, "-p", str(args.port),
         "-u", args.username, "-P", password, "-t", TELEMETRY_TOPIC,
         "-F", "%x", "-R"], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, bufsize=1,
    )

    def read_mqtt() -> None:
        assert subscriber.stdout is not None
        for line in subscriber.stdout:
            try:
                frame = decode_frame(bytes.fromhex(line.strip()))
            except (ValueError, struct.error) as exc:
                print(f"Ignored unexpected MQTT message: {exc}")
                continue
            with lock:
                shared["frame"] = frame
                shared["received_at"] = time.time()

    threading.Thread(target=read_mqtt, daemon=True).start()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            if self.path == "/api/telemetry":
                with lock:
                    body = json.dumps(shared).encode("utf-8")
                mime = "application/json"
            elif self.path == "/":
                body = PAGE.encode("utf-8")
                mime = "text/html; charset=utf-8"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self) -> None:
            if self.path != "/api/command":
                self.send_error(404)
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if length < 1 or length > 128:
                    raise ValueError("Invalid request size")
                name = json.loads(self.rfile.read(length))["command"]
                value = {"START": 1, "STOP": 2, "RESET": 3}[name]
                with tempfile.NamedTemporaryFile(delete=False) as f:
                    f.write(bytes([value]))
                    filename = f.name
                try:
                    result = subprocess.run(
                        [str(pub_bin), "-h", args.broker, "-p", str(args.port),
                         "-u", args.username, "-P", password, "-t", COMMAND_TOPIC,
                         "-q", "1", "-f", filename],
                        capture_output=True, text=True, timeout=5,
                    )
                finally:
                    os.unlink(filename)
                ok = result.returncode == 0
                message = (f"{name} delivered to broker; check robot state" if ok
                           else f"Publish failed: {result.stderr.strip()}")
            except (ValueError, KeyError, subprocess.TimeoutExpired, OSError) as exc:
                ok, message = False, f"Could not send command: {exc}"
            body = json.dumps({"message": message}).encode("utf-8")
            self.send_response(200 if ok else 400)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    url = f"http://127.0.0.1:{args.dashboard_port}/"
    server = ThreadingHTTPServer(("127.0.0.1", args.dashboard_port), Handler)
    print(f"Dashboard: {url}")
    print("Keep this terminal open. Press Ctrl+C to stop the dashboard.")
    webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        subscriber.terminate()
        try:
            subscriber.wait(timeout=2)
        except subprocess.TimeoutExpired:
            subscriber.kill()


if __name__ == "__main__":
    main()
