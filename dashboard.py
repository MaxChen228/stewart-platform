#!/usr/bin/env python3
"""Stewart Platform Dashboard - connects to ESP32 via USB Serial"""

import serial
import serial.tools.list_ports
import threading
import json
import sys
import time
from http.server import HTTPServer, SimpleHTTPRequestHandler
import urllib.parse

PORT_BAUD = 115200
WEB_PORT = 8080
STALE_TIMEOUT = 2.0

latest_status = {"motors": []}
last_update = 0.0
connected = False
ser = None
ser_lock = threading.Lock()

HTML = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Stewart Platform</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:-apple-system,sans-serif;background:#1a1a2e;color:#eee;padding:16px}
h1{text-align:center;margin-bottom:8px;color:#e94560}
.conn{text-align:center;margin-bottom:16px;font-size:13px;font-weight:bold}
.conn.ok{color:#00b894}
.conn.lost{color:#e94560}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;max-width:600px;margin:0 auto}
.card{background:#16213e;border-radius:12px;padding:16px;text-align:center;border:2px solid #0f3460;transition:all .3s}
.card.online{border-color:#00b894}
.card.spinning{border-color:#e94560;animation:pulse 1s infinite}
.card.stale{opacity:0.4}
@keyframes pulse{0%,100%{box-shadow:0 0 0 0 rgba(233,69,96,0.4)}50%{box-shadow:0 0 12px 4px rgba(233,69,96,0.2)}}
.id{font-size:28px;font-weight:bold;margin-bottom:4px}
.status{font-size:13px;margin-bottom:6px;color:#666}
.status.on{color:#00b894;font-weight:bold}
.encoder{font-size:16px;color:#74b9ff;margin-bottom:12px;font-family:monospace}
.btn{background:#e94560;color:#fff;border:none;border-radius:8px;padding:10px 20px;font-size:14px;cursor:pointer;width:100%;transition:background .2s}
.btn:hover{background:#c0392b}
.btn:disabled{background:#555;cursor:not-allowed}
.footer{text-align:center;margin-top:20px;color:#555;font-size:12px}
</style>
</head>
<body>
<h1>Stewart Platform</h1>
<div class="conn" id="conn"></div>
<div class="grid" id="grid"></div>
<div class="footer">USB Serial Dashboard</div>
<script>
function render(data){
  let ce=document.getElementById('conn');
  let stale=data.stale;
  ce.textContent=stale?'DISCONNECTED':'CONNECTED';
  ce.className='conn '+(stale?'lost':'ok');
  if(!data.motors||!data.motors.length){
    document.getElementById('grid').innerHTML='<div style="grid-column:1/-1;text-align:center;color:#666;padding:40px">Waiting for data...</div>';
    return;
  }
  let g=document.getElementById('grid');
  g.innerHTML='';
  data.motors.forEach((m)=>{
    let c=document.createElement('div');
    c.className='card'+(m.on?' online':'')+(m.spin?' spinning':'')+(stale?' stale':'');
    c.innerHTML=
      '<div class="id">M'+m.id+'</div>'+
      '<div class="status '+(m.on?'on':'')+'">'+( m.on?'ONLINE':'OFFLINE')+'</div>'+
      '<div class="encoder">'+(m.on?m.deg.toFixed(1)+'\\u00B0':'--')+'</div>'+
      '<button class="btn" '+(stale||!m.on||m.spin?'disabled':'')+' onclick="spin('+m.id+')">'+
      (m.spin?'Spinning...':'Spin 3s')+'</button>';
    g.appendChild(c);
  });
}
function spin(id){fetch('/spin?id='+id)}
function poll(){fetch('/status').then(r=>r.json()).then(render).catch(()=>{})}
setInterval(poll,100);
poll();
</script>
</body>
</html>"""


class Handler(SimpleHTTPRequestHandler):
    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(HTML.encode())
        elif parsed.path == '/status':
            stale = (time.time() - last_update) > STALE_TIMEOUT if last_update > 0 else True
            resp = {**latest_status, "stale": stale}
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(resp).encode())
        elif parsed.path == '/spin':
            qs = urllib.parse.parse_qs(parsed.query)
            mid = qs.get('id', [''])[0]
            with ser_lock:
                if ser and ser.is_open and mid:
                    try:
                        ser.write(f"SPIN:{mid}\n".encode())
                    except Exception:
                        pass
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(b"ok")
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass


def find_port():
    for p in serial.tools.list_ports.comports():
        if 'usbserial' in p.device or 'USB' in p.description or 'CH340' in p.device:
            return p.device
    return None


def serial_reader():
    global latest_status, last_update, connected, ser
    while True:
        # Connect / reconnect
        with ser_lock:
            if ser is None or not ser.is_open:
                connected = False
                port = find_port()
                if not port:
                    time.sleep(1)
                    continue
                try:
                    ser = serial.Serial(port, PORT_BAUD, timeout=1)
                    connected = True
                    print(f"Connected to {port}")
                except Exception as e:
                    print(f"Connect failed: {e}")
                    time.sleep(1)
                    continue

        # Read loop
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            if line.startswith('{'):
                data = json.loads(line)
                if 'motors' in data:
                    latest_status = data
                    last_update = time.time()
        except (serial.SerialException, OSError):
            print("Serial disconnected, waiting for reconnect...")
            connected = False
            with ser_lock:
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
            time.sleep(1)
        except (json.JSONDecodeError, UnicodeDecodeError):
            pass


def main():
    global ser, connected

    port = find_port()
    if port:
        print(f"Found ESP32 at {port}")
        try:
            ser = serial.Serial(port, PORT_BAUD, timeout=1)
            connected = True
            print("Connected!")
        except Exception as e:
            print(f"Initial connect failed: {e}")
    else:
        print("No ESP32 found, will auto-detect when plugged in...")

    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()

    print(f"\nDashboard: http://localhost:{WEB_PORT}")
    print("Press Ctrl+C to quit\n")

    httpd = HTTPServer(('', WEB_PORT), Handler)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
        with ser_lock:
            if ser and ser.is_open:
                ser.close()


if __name__ == '__main__':
    main()
