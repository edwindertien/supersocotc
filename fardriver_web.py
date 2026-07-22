#!/usr/bin/env python3
"""
FarDriver Web Tool
==================
Browser-based diagnostic dashboard + settings editor for FarDriver ND-series
controllers connected via 3.3 V TTL serial adapter.

No tkinter required — runs in any browser, including on your phone.

Usage
-----
  python3 fardriver_web.py                          # opens on http://localhost:5000
  python3 fardriver_web.py --port /dev/tty.usbserial-AK04P0KB
  python3 fardriver_web.py --port /dev/tty.usbserial-AK04P0KB --baud 9600

Then open  http://localhost:5000  in any browser.
From your phone/FP5 on the same WiFi: http://<your-mac-ip>:5000

Hardware
--------
  USB-to-TTL 3.3V adapter -> FarDriver 4-pin serial header
  Pin 1  3.3V   <- DO NOT CONNECT
  Pin 2  GND    -> adapter GND
  Pin 3  RXD    -> adapter TX
  Pin 4  TXD    -> adapter RX

Requirements
------------
  pip install flask pyserial
"""

import argparse
import collections
import glob
import json
import math
import queue
import struct
import sys
import threading
import time
from dataclasses import dataclass, field, asdict
from datetime import datetime
from typing import Optional

from flask import Flask, Response, jsonify, request, stream_with_context

# ─────────────────────────────────────────────────────────────────────────────
# Protocol: CRC
# ─────────────────────────────────────────────────────────────────────────────

_CRC_LO = [
    0,192,193,1,195,3,2,194,198,6,7,199,5,197,196,4,
    204,12,13,205,15,207,206,14,10,202,203,11,201,9,8,200,
    216,24,25,217,27,219,218,26,30,222,223,31,221,29,28,220,
    20,212,213,21,215,23,22,214,210,18,19,211,17,209,208,16,
    240,48,49,241,51,243,242,50,54,246,247,55,245,53,52,244,
    60,252,253,61,255,63,62,254,250,58,59,251,57,249,248,56,
    40,232,233,41,235,43,42,234,238,46,47,239,45,237,236,44,
    228,36,37,229,39,231,230,38,34,226,227,35,225,33,32,224,
    160,96,97,161,99,163,162,98,102,166,167,103,165,101,100,164,
    108,172,173,109,175,111,110,174,170,106,107,171,105,169,168,104,
    120,184,185,121,187,123,122,186,190,126,127,191,125,189,188,124,
    180,116,117,181,119,183,182,118,114,178,179,115,177,113,112,176,
    80,144,145,81,147,83,82,146,150,86,87,151,85,149,148,84,
    156,92,93,157,95,159,158,94,90,154,155,91,153,89,88,152,
    136,72,73,137,75,139,138,74,78,142,143,79,141,77,76,140,
    68,132,133,69,135,71,70,134,130,66,67,131,65,129,128,64,
]
_CRC_HI = [
    0,193,129,64,1,192,128,65,1,192,128,65,0,193,129,64,
    1,192,128,65,0,193,129,64,0,193,129,64,1,192,128,65,
    1,192,128,65,0,193,129,64,0,193,129,64,1,192,128,65,
    0,193,129,64,1,192,128,65,1,192,128,65,0,193,129,64,
    1,192,128,65,0,193,129,64,0,193,129,64,1,192,128,65,
    0,193,129,64,1,192,128,65,1,192,128,65,0,193,129,64,
    0,193,129,64,1,192,128,65,1,192,128,65,0,193,129,64,
    1,192,128,65,0,193,129,64,0,193,129,64,1,192,128,65,
    1,192,128,65,0,193,129,64,0,193,129,64,1,192,128,65,
    0,193,129,64,1,192,128,65,1,192,128,65,0,193,129,64,
    0,193,129,64,1,192,128,65,1,192,128,65,0,193,129,64,
    1,192,128,65,0,193,129,64,0,193,129,64,1,192,128,65,
    0,193,129,64,1,192,128,65,1,192,128,65,0,193,129,64,
    1,192,128,65,0,193,129,64,0,193,129,64,1,192,128,65,
    1,192,128,65,0,193,129,64,0,193,129,64,1,192,128,65,
    0,193,129,64,1,192,128,65,1,192,128,65,0,193,129,64,
]

def _crc(data: bytes):
    a, b = 0x3C, 0x7F
    for byte in data:
        i = a ^ byte
        a = b ^ _CRC_HI[i]
        b = _CRC_LO[i]
    return a, b

def build_syscmd(cmd: int) -> bytes:
    """Build a system command packet.  AA C6 A0 A0 88 <cmd> <crc_a> <crc_b>"""
    raw = bytes([0xAA, 0xC6, 0xA0, 0xA0, 0x88, cmd])
    ca, cb = _crc(raw)
    return raw + bytes([ca, cb])

CMD_SELF_LEARN    = build_syscmd(0x02)
CMD_FACTORY_RESET = build_syscmd(0x08)
CMD_GATHER_DATA   = build_syscmd(0x06)
CMD_NON_FOLLOW    = build_syscmd(0x01)
CMD_RESET_PINS    = build_syscmd(0x09)

# Verify factory reset CRC matches known-good from README
assert list(CMD_FACTORY_RESET[-2:]) == [0xC5, 0x09], "CRC table error"

# ─────────────────────────────────────────────────────────────────────────────
# Protocol: address map
# ─────────────────────────────────────────────────────────────────────────────

FLASH_READ_ADDR = [
    0xE2, 0xE8, 0xEE, 0x00, 0x06, 0x0C, 0x12,
    0xE2, 0xE8, 0xEE, 0x18, 0x1E, 0x24, 0x2A,
    0xE2, 0xE8, 0xEE, 0x30, 0x5D, 0x63, 0x69,
    0xE2, 0xE8, 0xEE, 0x7C, 0x82, 0x88, 0x8E,
    0xE2, 0xE8, 0xEE, 0x94, 0x9A, 0xA0, 0xA6,
    0xE2, 0xE8, 0xEE, 0xAC, 0xB2, 0xB8, 0xBE,
    0xE2, 0xE8, 0xEE, 0xC4, 0xCA, 0xD0,
    0xE2, 0xE8, 0xEE, 0xD6, 0xDC, 0xF4, 0xFA,
]

# ─────────────────────────────────────────────────────────────────────────────
# State
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class LiveData:
    gear: int = 0
    forward: bool = False
    reverse: bool = False
    motion: bool = False
    speed_kmh: float = 0.0
    voltage: float = 0.0
    line_current: float = 0.0
    motor_temp: float = 0.0
    mos_temp: float = 0.0
    batt_soc: int = 0
    modulation: float = 0.0
    angle_learn: int = 0
    model_name: str = ""
    throttle_error: bool = False
    hall_error: bool = False
    motor_temp_protect: bool = False
    ctrl_temp_protect: bool = False
    brake: bool = False
    hall_pos_error: bool = False
    phase_lost: bool = False
    low_vol_stop: bool = False
    auto_learn: bool = False
    phase_a_curr: float = 0.0
    phase_c_curr: float = 0.0
    timestamp: float = 0.0
    packets: int = 0

    def errors(self):
        e = []
        if self.throttle_error:     e.append("E: Throttle")
        if self.hall_error:         e.append("E: Hall Sensor")
        if self.motor_temp_protect: e.append("E: Motor Temp")
        if self.ctrl_temp_protect:  e.append("E: Controller Temp")
        if self.hall_pos_error:     e.append("E: Hall Position")
        if self.phase_lost:         e.append("E: Phase Lost")
        if self.low_vol_stop:       e.append("E: Low Voltage")
        return e

    def learn_status(self):
        if self.angle_learn == 0xAA:  return "learned"
        if self.angle_learn == 0x55:  return "learning"
        return "not_learned"

@dataclass
class SettingsData:
    rated_voltage: float = 0.0
    rated_power: int = 0
    rated_speed: int = 0
    pole_pairs: int = 0
    max_line_curr: float = 0.0
    max_phase_curr: float = 0.0
    custom_max_line: float = 0.0
    custom_max_phase: float = 0.0
    back_speed: int = 0
    low_speed: int = 0
    mid_speed: int = 0
    stop_back_curr: int = 0
    max_back_curr: int = 0
    throttle_low: float = 0.0
    throttle_high: float = 0.0
    low_vol_protect: float = 0.0
    motor_temp_protect: int = 0
    motor_temp_restore: int = 0
    mos_temp_protect: int = 0
    start_ki: int = 0
    mid_ki: int = 0
    max_ki: int = 0
    start_kp: int = 0
    mid_kp: int = 0
    max_kp: int = 0
    an: int = 0
    lm: int = 0
    hardware_ver: str = ""
    software_ver: str = ""

live = LiveData()
settings = SettingsData()
speed_history: collections.deque = collections.deque(maxlen=300)
current_history: collections.deque = collections.deque(maxlen=300)
voltage_history: collections.deque = collections.deque(maxlen=300)
log_entries: collections.deque = collections.deque(maxlen=200)

def add_log(msg: str, level: str = "info", terminal: bool = True):
    ts = datetime.now().strftime('%H:%M:%S.%f')[:12]
    log_entries.append({"ts": ts, "msg": msg, "level": level})
    # Only print important events to terminal, not raw RX packets
    if terminal and level != 'rx':
        prefix = {'info':'   ', 'warn':'⚠  ', 'error':'✗  ', 'tx':'→  '}.get(level, '   ')
        print(f"  {prefix}[{ts}] {msg}")

# ─────────────────────────────────────────────────────────────────────────────
# Decoder
# ─────────────────────────────────────────────────────────────────────────────

def decode(msg_id: int, data: bytes):
    addr = FLASH_READ_ADDR[msg_id] if msg_id < len(FLASH_READ_ADDR) else msg_id

    if addr == 0xE2:
        b0 = data[0]
        live.forward           = bool(b0 & 0x01)
        live.reverse           = bool(b0 & 0x02)
        live.gear              = (b0 >> 2) & 0x03
        live.motion            = bool(b0 & 0x10)
        b2 = data[2]
        live.hall_error        = bool(b2 & 0x01)
        live.throttle_error    = bool(b2 & 0x02)
        live.motor_temp_protect= bool(b2 & 0x40)
        live.ctrl_temp_protect = bool(b2 & 0x80)
        live.brake             = bool(data[3] & 0x80)
        live.modulation        = data[4] / 128.0 * 100.0
        raw_speed              = struct.unpack_from('<H', data, 6)[0]
        live.speed_kmh         = raw_speed * 0.109
        live.timestamp         = time.time()

    elif addr == 0xE8:
        live.voltage      = struct.unpack_from('<h', data, 0)[0] / 10.0
        live.line_current = struct.unpack_from('<h', data, 4)[0] / 4.0

    elif addr == 0xEE:
        def _big24(d, off):
            v = (d[off] << 16) | (d[off+1] << 8) | d[off+2]
            return 1.953125 * math.sqrt(v) if v > 0 else 0.0
        live.phase_a_curr = _big24(data, 4)
        live.phase_c_curr = _big24(data, 7)

    elif addr == 0xF4:
        live.motor_temp = struct.unpack_from('<h', data, 0)[0]
        live.batt_soc   = data[3]

    elif addr == 0xD6:
        s1 = struct.unpack_from('<H', data, 2)[0]
        live.auto_learn     = bool(s1 & 0x0010)
        live.hall_pos_error = bool(s1 & 0x0004)
        s2 = struct.unpack_from('<H', data, 4)[0]
        live.phase_lost     = bool(s2 & 0x0800)
        live.low_vol_stop   = bool(s2 & 0x0010)
        live.mos_temp       = struct.unpack_from('<h', data, 10)[0]

    elif addr == 0xA0:
        try:
            live.model_name = data[2:12].rstrip(b'\x00').decode('ascii', errors='replace')
        except Exception:
            pass

    elif addr == 0xCA:
        live.angle_learn = data[0]

    elif addr == 0x12:
        settings.pole_pairs    = data[4]
        settings.rated_voltage = struct.unpack_from('<H', data, 10)[0] / 10.0
        settings.rated_power   = struct.unpack_from('<H', data, 8)[0]

    elif addr == 0x18:
        settings.rated_speed   = struct.unpack_from('<H', data, 0)[0]
        settings.max_line_curr = struct.unpack_from('<H', data, 2)[0] / 4.0

    elif addr == 0x24:
        settings.custom_max_line  = struct.unpack_from('<H', data, 4)[0] / 4.0
        settings.custom_max_phase = struct.unpack_from('<H', data, 6)[0] / 4.0
        settings.back_speed       = struct.unpack_from('<H', data, 8)[0]
        settings.low_speed        = struct.unpack_from('<H', data, 10)[0]

    elif addr == 0x2A:
        settings.mid_speed      = struct.unpack_from('<H', data, 0)[0]
        settings.max_phase_curr = struct.unpack_from('<H', data, 6)[0] / 4.0

    elif addr == 0x30:
        settings.stop_back_curr = struct.unpack_from('<H', data, 0)[0]
        settings.max_back_curr  = struct.unpack_from('<H', data, 2)[0]

    elif addr == 0x06:
        settings.throttle_low  = data[6] / 20.0
        settings.throttle_high = data[7] / 20.0

    elif addr == 0x0C:
        settings.start_ki = data[6]; settings.mid_ki = data[7]
        settings.max_ki   = data[8]; settings.start_kp = data[9]
        settings.mid_kp   = data[10]; settings.max_kp = data[11]

    elif addr == 0x1E:
        settings.low_vol_protect = struct.unpack_from('<H', data, 2)[0] / 10.0

    elif addr == 0x82:
        settings.motor_temp_protect = data[4]
        settings.motor_temp_restore = data[5]
        settings.mos_temp_protect   = data[6]
        settings.hardware_ver = chr(data[11]) if 32 <= data[11] <= 126 else '?'
        settings.software_ver = f"{chr(data[12]) if 32 <= data[12] <= 126 else '?'}.{data[13]}"

    elif addr == 0x9A:
        settings.an = data[6] & 0x0F
        settings.lm = data[7] & 0x1F

# ─────────────────────────────────────────────────────────────────────────────
# Serial
# ─────────────────────────────────────────────────────────────────────────────

class SerialReader:
    def __init__(self):
        self._ser    = None
        self._thread = None
        self._stop   = threading.Event()
        self._lock   = threading.Lock()
        self.port    = ""
        self.baud    = 9600
        self.connected = False

    def connect(self, port: str, baud: int = 9600) -> bool:
        try:
            import serial
            self.port = port
            self.baud = baud
            self._ser = serial.Serial(port, baud, timeout=0.05, write_timeout=1.0)
            self.connected = True
            self._stop.clear()
            self._thread = threading.Thread(target=self._reader, daemon=True)
            self._thread.start()
            add_log(f"Connected to {port} at {baud} baud", "info")
            return True
        except Exception as e:
            add_log(f"Connection failed: {e}", "error")
            return False

    def disconnect(self):
        self._stop.set()
        if self._ser:
            try: self._ser.close()
            except: pass
        self.connected = False
        add_log("Disconnected", "warn")

    def send(self, data: bytes, label: str = ""):
        if self._ser and self.connected:
            with self._lock:
                self._ser.write(data)
            hex_str = ' '.join(f'{b:02X}' for b in data)
            add_log(f"TX {label}: {hex_str}", "tx")

    def _reader(self):
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(64)
                if not chunk:
                    continue
                buf.extend(chunk)
                while len(buf) >= 16:
                    idx = buf.find(0xAA)
                    if idx < 0:
                        buf.clear(); break
                    if idx > 0:
                        del buf[:idx]
                    if len(buf) < 16:
                        break
                    msg = bytes(buf[:16])
                    ca, cb = _crc(msg[:14])
                    if msg[14] == ca and msg[15] == cb:
                        msg_id = msg[1] & 0x3F
                        data   = msg[2:14]
                        decode(msg_id, data)
                        live.packets += 1
                        # Print summary to terminal every 100 packets
                        if live.packets % 100 == 0:
                            add_log(f"Packet #{live.packets} — "
                                    f"{live.speed_kmh:.1f}km/h  "
                                    f"Gear={live.gear}  "
                                    f"SoC={live.batt_soc}%  "
                                    f"{live.voltage:.1f}V  "
                                    f"{live.line_current:.1f}A", "info")
                        # Update histories every ~10 packets
                        if live.packets % 10 == 0:
                            t = time.time()
                            speed_history.append((t, live.speed_kmh))
                            current_history.append((t, live.line_current))
                            voltage_history.append((t, live.voltage))
                        addr = FLASH_READ_ADDR[msg_id] if msg_id < len(FLASH_READ_ADDR) else msg_id
                        add_log(f"RX id={msg_id:02X} addr=0x{addr:02X}  "
                                + ' '.join(f'{b:02X}' for b in data), "rx",
                                terminal=False)
                        del buf[:16]
                    else:
                        del buf[:1]
            except Exception as e:
                if not self._stop.is_set():
                    add_log(f"Serial error: {e}", "error")
                    time.sleep(0.1)

reader = SerialReader()

def find_ports():
    patterns = ['/dev/tty.usbserial*', '/dev/tty.usbmodem*',
                '/dev/ttyUSB*', '/dev/ttyACM*', 'COM*']
    ports = []
    for p in patterns:
        ports.extend(glob.glob(p))
    return sorted(ports)

# ─────────────────────────────────────────────────────────────────────────────
# Flask app
# ─────────────────────────────────────────────────────────────────────────────

import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)   # suppress per-request GET/POST lines

app = Flask(__name__)
app.config['SECRET_KEY'] = 'fardriver'

# SSE subscribers
_sse_clients: list[queue.Queue] = []
_sse_lock = threading.Lock()

def push_sse(data: dict):
    dead = []
    with _sse_lock:
        for q in _sse_clients:
            try:
                q.put_nowait(data)
            except queue.Full:
                dead.append(q)
        for q in dead:
            _sse_clients.remove(q)

def sse_pusher():
    """Background thread: push live data to all SSE subscribers ~10x/sec."""
    while True:
        time.sleep(0.1)
        d = asdict(live)
        d['errors'] = live.errors()
        d['learn_status'] = live.learn_status()
        d['connected'] = reader.connected
        d['port'] = reader.port
        push_sse({'type': 'live', 'data': d})

threading.Thread(target=sse_pusher, daemon=True).start()

@app.route('/events')
def sse():
    q: queue.Queue = queue.Queue(maxsize=50)
    with _sse_lock:
        _sse_clients.append(q)

    @stream_with_context
    def generate():
        try:
            while True:
                try:
                    msg = q.get(timeout=15)
                    yield f"data: {json.dumps(msg)}\n\n"
                except queue.Empty:
                    yield ": keepalive\n\n"
        except GeneratorExit:
            with _sse_lock:
                if q in _sse_clients:
                    _sse_clients.remove(q)

    return Response(generate(), mimetype='text/event-stream',
                    headers={'Cache-Control': 'no-cache',
                             'X-Accel-Buffering': 'no'})

@app.route('/api/ports')
def api_ports():
    return jsonify(find_ports())

@app.route('/api/connect', methods=['POST'])
def api_connect():
    d    = request.json or {}
    port = d.get('port', '')
    baud = int(d.get('baud', 9600))
    if reader.connected:
        reader.disconnect()
    ok = reader.connect(port, baud)
    if ok:
        time.sleep(0.3)
        reader.send(CMD_GATHER_DATA, "GATHER")
    return jsonify({'ok': ok})

@app.route('/api/disconnect', methods=['POST'])
def api_disconnect():
    reader.disconnect()
    return jsonify({'ok': True})

@app.route('/api/cmd/<name>', methods=['POST'])
def api_cmd(name):
    cmds = {
        'self_learn':    (CMD_SELF_LEARN,    "SELF_LEARN"),
        'factory_reset': (CMD_FACTORY_RESET, "FACTORY_RESET"),
        'gather':        (CMD_GATHER_DATA,   "GATHER"),
        'non_follow':    (CMD_NON_FOLLOW,    "NON_FOLLOW"),
        'reset_pins':    (CMD_RESET_PINS,    "RESET_PINS"),
    }
    if name not in cmds:
        return jsonify({'ok': False, 'error': 'unknown command'}), 400
    if not reader.connected:
        return jsonify({'ok': False, 'error': 'not connected'}), 400
    cmd, label = cmds[name]
    reader.send(cmd, label)
    return jsonify({'ok': True})

@app.route('/api/settings')
def api_settings():
    return jsonify(asdict(settings))

@app.route('/api/log')
def api_log():
    n = int(request.args.get('n', 100))
    return jsonify(list(log_entries)[-n:])

@app.route('/api/history')
def api_history():
    t0 = time.time() - 120
    return jsonify({
        'speed':   [[t,v] for t,v in speed_history   if t >= t0],
        'current': [[t,v] for t,v in current_history if t >= t0],
        'voltage': [[t,v] for t,v in voltage_history if t >= t0],
    })

# ─────────────────────────────────────────────────────────────────────────────
# HTML/CSS/JS (single file, no external dependencies except Chart.js CDN)
# ─────────────────────────────────────────────────────────────────────────────

HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>FarDriver Tool</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
  :root {
    --bg:     #1A1C1E;
    --panel:  #23262A;
    --border: #2E3238;
    --amber:  #F0A500;
    --green:  #3DCC7E;
    --red:    #E05252;
    --muted:  #6B7280;
    --white:  #E8EAED;
    --mono:   'Menlo', 'Consolas', monospace;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { background: var(--bg); color: var(--white); font-family: -apple-system, sans-serif; min-height: 100vh; }

  /* ── header ── */
  header {
    display: flex; align-items: center; gap: 12px; flex-wrap: wrap;
    padding: 12px 20px; border-bottom: 1px solid var(--border);
    position: sticky; top: 0; background: var(--bg); z-index: 100;
  }
  header h1 { font-size: 1.3rem; }
  header h1 span { color: var(--amber); }
  .conn-row { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; margin-left: auto; }
  select, input[type=text] {
    background: var(--panel); color: var(--white); border: 1px solid var(--border);
    border-radius: 6px; padding: 6px 10px; font-size: 0.85rem;
  }
  .dot { width: 10px; height: 10px; border-radius: 50%; background: var(--red); display: inline-block; }
  .dot.on { background: var(--green); }
  #conn-label { font-size: 0.85rem; color: var(--muted); }

  /* ── buttons ── */
  btn, button {
    cursor: pointer; border: none; border-radius: 6px; padding: 7px 16px;
    font-size: 0.85rem; font-weight: 600; transition: opacity .15s;
  }
  button:hover { opacity: .85; }
  .btn-amber { background: var(--amber); color: #111; }
  .btn-muted { background: var(--border); color: var(--white); }
  .btn-red   { background: var(--red);   color: #fff; }
  .btn-green { background: var(--green); color: #111; }

  /* ── tabs ── */
  .tabs { display: flex; gap: 2px; padding: 12px 20px 0; border-bottom: 1px solid var(--border); }
  .tab {
    padding: 8px 18px; cursor: pointer; border-radius: 6px 6px 0 0;
    font-size: 0.9rem; color: var(--muted); background: transparent;
    border: 1px solid transparent; border-bottom: none;
    transition: color .15s;
  }
  .tab.active { color: var(--amber); background: var(--panel); border-color: var(--border); }
  .tab-content { display: none; padding: 16px 20px; }
  .tab-content.active { display: block; }

  /* ── cards ── */
  .grid { display: grid; gap: 12px; }
  .grid-4 { grid-template-columns: repeat(4, 1fr); }
  .grid-3 { grid-template-columns: repeat(3, 1fr); }
  .grid-2 { grid-template-columns: repeat(2, 1fr); }
  @media(max-width:900px){ .grid-4{grid-template-columns:repeat(2,1fr);} }
  @media(max-width:600px){ .grid-4,.grid-3,.grid-2{grid-template-columns:1fr;} }

  .card {
    background: var(--panel); border: 1px solid var(--border);
    border-radius: 10px; padding: 14px 16px;
  }
  .card-label { font-size: 0.72rem; color: var(--muted); text-transform: uppercase; letter-spacing: .08em; margin-bottom: 4px; }
  .card-value { font-family: var(--mono); font-size: 1.8rem; font-weight: 700; color: var(--white); line-height: 1.1; }
  .card-value.big { font-size: 3rem; color: var(--amber); }
  .card-value.green { color: var(--green); }
  .card-value.red   { color: var(--red); }
  .card-sub { font-size: 0.8rem; color: var(--muted); margin-top: 4px; }

  /* progress bar */
  .bar-track { background: var(--border); border-radius: 4px; height: 8px; margin-top: 8px; overflow: hidden; }
  .bar-fill  { height: 100%; border-radius: 4px; background: var(--amber); transition: width .5s; }
  .bar-fill.green { background: var(--green); }
  .bar-fill.red   { background: var(--red); }

  /* ── chart ── */
  .chart-card { grid-column: 1 / -1; }
  .chart-wrap { position: relative; height: 180px; }

  /* ── settings table ── */
  .settings-sections { display: grid; grid-template-columns: repeat(auto-fill, minmax(320px, 1fr)); gap: 12px; }
  .setting-section { background: var(--panel); border: 1px solid var(--border); border-radius: 10px; padding: 14px 16px; }
  .setting-section h3 { font-size: 0.8rem; color: var(--amber); text-transform: uppercase; letter-spacing: .08em; margin-bottom: 10px; }
  .setting-row { display: flex; align-items: center; gap: 8px; padding: 4px 0; border-bottom: 1px solid var(--border); }
  .setting-row:last-child { border-bottom: none; }
  .setting-label { flex: 1; font-size: 0.82rem; color: var(--muted); }
  .setting-value { font-family: var(--mono); font-size: 0.9rem; color: var(--white); min-width: 80px; text-align: right; }
  .setting-unit  { font-size: 0.75rem; color: var(--muted); width: 36px; }

  /* ── commands ── */
  .cmd-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(240px, 1fr)); gap: 12px; }
  .cmd-card { background: var(--panel); border: 1px solid var(--border); border-radius: 10px; padding: 16px; }
  .cmd-card h3 { font-size: 1rem; color: var(--amber); margin-bottom: 6px; }
  .cmd-card p  { font-size: 0.82rem; color: var(--muted); margin-bottom: 12px; line-height: 1.5; }

  /* ── log ── */
  #log-box {
    background: var(--panel); border: 1px solid var(--border); border-radius: 10px;
    padding: 12px; height: 480px; overflow-y: auto;
    font-family: var(--mono); font-size: 0.78rem; line-height: 1.6;
  }
  .log-ts   { color: var(--muted); }
  .log-rx   { color: var(--green); }
  .log-tx   { color: var(--amber); }
  .log-error{ color: var(--red); }
  .log-warn { color: #FFC107; }
  .log-info { color: var(--white); }
  .log-controls { display: flex; gap: 8px; margin-bottom: 8px; align-items: center; }
  #log-filter { flex:1; }

  /* ── fault badges ── */
  .fault-badge {
    display: inline-block; background: var(--red); color: #fff;
    border-radius: 4px; font-size: 0.75rem; padding: 2px 8px; margin: 2px;
  }
  .no-faults { color: var(--green); font-size: 0.9rem; }
</style>
</head>
<body>

<header>
  <h1><span>FarDriver</span> Tool</h1>
  <div class="conn-row">
    <select id="port-sel"><option value="">Select port…</option></select>
    <select id="baud-sel">
      <option value="9600" selected>9600</option>
      <option value="19200">19200</option>
      <option value="115200">115200</option>
    </select>
    <button class="btn-muted" onclick="refreshPorts()">⟳</button>
    <button class="btn-amber" id="conn-btn" onclick="toggleConnect()">Connect</button>
    <span class="dot" id="conn-dot"></span>
    <span id="conn-label">Disconnected</span>
  </div>
</header>

<div class="tabs">
  <div class="tab active" onclick="showTab('dashboard')">Dashboard</div>
  <div class="tab" onclick="showTab('settings')">Settings</div>
  <div class="tab" onclick="showTab('commands')">Commands</div>
  <div class="tab" onclick="showTab('log')">Log</div>
</div>

<!-- ══════════════════════ DASHBOARD ══════════════════════ -->
<div id="tab-dashboard" class="tab-content active">
  <div class="grid grid-4" style="margin-bottom:12px">
    <div class="card" style="grid-column:span 2">
      <div class="card-label">Speed</div>
      <div class="card-value big" id="d-speed">0.0</div>
      <div class="card-sub">km/h &nbsp;|&nbsp; <span id="d-gear">GEAR —</span> &nbsp; <span id="d-dir">·</span></div>
    </div>
    <div class="card">
      <div class="card-label">Battery SoC</div>
      <div class="card-value green" id="d-soc">— %</div>
      <div class="bar-track"><div class="bar-fill green" id="d-soc-bar" style="width:0%"></div></div>
    </div>
    <div class="card">
      <div class="card-label">Voltage</div>
      <div class="card-value" id="d-volt">— V</div>
      <div class="card-sub" id="d-volt-sub"></div>
    </div>
  </div>

  <div class="grid grid-4" style="margin-bottom:12px">
    <div class="card">
      <div class="card-label">Line Current</div>
      <div class="card-value" id="d-curr">— A</div>
      <div class="card-sub" id="d-regen"></div>
    </div>
    <div class="card">
      <div class="card-label">Temperatures</div>
      <div class="card-sub" style="font-size:0.9rem;margin-top:4px">Motor &nbsp;<span id="d-mtemp" style="font-family:var(--mono);color:var(--white)">—</span> °C</div>
      <div class="card-sub" style="font-size:0.9rem;margin-top:4px">MosFET <span id="d-ctemp" style="font-family:var(--mono);color:var(--white)">—</span> °C</div>
    </div>
    <div class="card">
      <div class="card-label">Auto-Learn</div>
      <div class="card-value" style="font-size:1rem" id="d-learn">✗ Not learned</div>
      <div class="card-sub" id="d-model"></div>
    </div>
    <div class="card">
      <div class="card-label">Faults</div>
      <div id="d-faults"><span class="no-faults">✓ None</span></div>
    </div>
  </div>

  <div class="card chart-card">
    <div class="card-label" style="margin-bottom:8px">Trend — last 2 minutes</div>
    <div class="chart-wrap"><canvas id="trend-chart"></canvas></div>
  </div>
</div>

<!-- ══════════════════════ SETTINGS ══════════════════════ -->
<div id="tab-settings" class="tab-content">
  <div class="settings-sections" id="settings-container"></div>
</div>

<!-- ══════════════════════ COMMANDS ══════════════════════ -->
<div id="tab-commands" class="tab-content">
  <div class="cmd-grid">
    <div class="cmd-card">
      <h3>Self-Learn</h3>
      <p>Calibrates Hall sensor phase angles and pole pairs.<br>
         <strong>Rear wheel must be off the ground.</strong><br>
         Apply full throttle when the motor starts spinning.</p>
      <button class="btn-amber" onclick="sendCmd('self_learn', 'Rear wheel off ground?')">▶ Start Self-Learn</button>
    </div>
    <div class="cmd-card">
      <h3>Request Data</h3>
      <p>Sends the data-gather command to start the controller streaming status messages. Run this if the dashboard shows no live data.</p>
      <button class="btn-muted" onclick="sendCmd('gather')">⟳ Request Data</button>
    </div>
    <div class="cmd-card">
      <h3>Non-Following</h3>
      <p>Sets the controller to non-following status. Useful for bench testing without driving the motor.</p>
      <button class="btn-muted" onclick="sendCmd('non_follow')">■ Non-Following</button>
    </div>
    <div class="cmd-card">
      <h3>Reset Pins</h3>
      <p>Resets the controller's pin configuration to defaults. Use if input pins behave unexpectedly after wiring changes.</p>
      <button class="btn-muted" onclick="sendCmd('reset_pins')">⟳ Reset Pins</button>
    </div>
    <div class="cmd-card">
      <h3>Factory Reset</h3>
      <p>⚠️ Resets ALL parameters to factory defaults. This cannot be undone. Use only as a last resort.</p>
      <button class="btn-red" onclick="sendCmd('factory_reset', 'This will reset ALL parameters to factory defaults. Cannot be undone.')">⚠ Factory Reset</button>
    </div>
    <div class="cmd-card">
      <div class="card-label" style="margin-bottom:8px">Packets received</div>
      <div class="card-value" id="d-packets">0</div>
    </div>
  </div>
</div>

<!-- ══════════════════════ LOG ══════════════════════ -->
<div id="tab-log" class="tab-content">
  <div class="log-controls">
    <input type="text" id="log-filter" placeholder="Filter log…" oninput="filterLog()">
    <label style="font-size:0.82rem;color:var(--muted)">
      <input type="checkbox" id="log-rx" checked onchange="filterLog()"> RX
    </label>
    <label style="font-size:0.82rem;color:var(--muted)">
      <input type="checkbox" id="log-tx" checked onchange="filterLog()"> TX
    </label>
    <button class="btn-muted" onclick="clearLog()">Clear</button>
    <button class="btn-muted" onclick="loadLog()">Reload</button>
  </div>
  <div id="log-box"></div>
</div>

<script>
// ── Connection ──────────────────────────────────────────────────────────────
let connected = false;

async function refreshPorts() {
  const r = await fetch('/api/ports');
  const ports = await r.json();
  const sel = document.getElementById('port-sel');
  const cur = sel.value;
  sel.innerHTML = '<option value="">Select port…</option>';
  ports.forEach(p => {
    const o = document.createElement('option');
    o.value = o.textContent = p;
    if (p === cur) o.selected = true;
    sel.appendChild(o);
  });
}

async function toggleConnect() {
  if (connected) {
    await fetch('/api/disconnect', {method:'POST'});
  } else {
    const port = document.getElementById('port-sel').value;
    const baud = document.getElementById('baud-sel').value;
    if (!port) { alert('Select a serial port first'); return; }
    const r = await fetch('/api/connect', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify({port, baud: parseInt(baud)})
    });
    const d = await r.json();
    if (!d.ok) alert('Connection failed — check port and baud rate');
  }
}

// ── SSE live updates ─────────────────────────────────────────────────────────
const es = new EventSource('/events');
es.onmessage = e => {
  const msg = JSON.parse(e.data);
  if (msg.type === 'live') updateDashboard(msg.data);
};

function updateDashboard(d) {
  connected = d.connected;
  document.getElementById('conn-dot').className = 'dot' + (d.connected ? ' on' : '');
  document.getElementById('conn-label').textContent = d.connected
    ? `Connected · ${d.port}` : 'Disconnected';
  document.getElementById('conn-btn').textContent = d.connected ? 'Disconnect' : 'Connect';

  // Speed
  document.getElementById('d-speed').textContent = d.speed_kmh.toFixed(1);
  const gears = ['P','Low','Mid','High'];
  document.getElementById('d-gear').textContent = 'GEAR ' + (gears[d.gear] || d.gear);
  document.getElementById('d-dir').textContent =
    d.reverse ? '◀ REV' : d.forward ? '▶ FWD' : '·';

  // SoC
  const soc = d.batt_soc;
  const socEl = document.getElementById('d-soc');
  socEl.textContent = soc + ' %';
  socEl.className = 'card-value ' + (soc < 20 ? 'red' : 'green');
  const bar = document.getElementById('d-soc-bar');
  bar.style.width = soc + '%';
  bar.className = 'bar-fill ' + (soc < 20 ? 'red' : 'green');

  // Voltage
  document.getElementById('d-volt').textContent = d.voltage.toFixed(1) + ' V';

  // Current
  const curr = d.line_current;
  const currEl = document.getElementById('d-curr');
  currEl.textContent = curr.toFixed(1) + ' A';
  currEl.className = 'card-value ' + (curr < -0.5 ? 'green' : '');
  document.getElementById('d-regen').textContent = curr < -0.5 ? '⚡ REGEN' : '';

  // Temps
  const mt = document.getElementById('d-mtemp');
  mt.textContent = d.motor_temp.toFixed(0);
  mt.style.color = d.motor_temp > 100 ? 'var(--red)' : 'var(--white)';
  const ct = document.getElementById('d-ctemp');
  ct.textContent = d.mos_temp.toFixed(0);
  ct.style.color = d.mos_temp > 80 ? 'var(--red)' : 'var(--white)';

  // Learn
  const learnEl = document.getElementById('d-learn');
  const ls = d.learn_status;
  learnEl.textContent = ls === 'learned' ? '✓ Learned'
    : ls === 'learning' ? '⟳ Learning…' : '✗ Not learned';
  learnEl.style.color = ls === 'learned' ? 'var(--green)'
    : ls === 'learning' ? 'var(--amber)' : 'var(--red)';
  if (d.model_name) document.getElementById('d-model').textContent = d.model_name;

  // Faults
  const fEl = document.getElementById('d-faults');
  if (d.errors.length === 0) {
    fEl.innerHTML = '<span class="no-faults">✓ None</span>';
  } else {
    fEl.innerHTML = d.errors.map(e => `<span class="fault-badge">${e}</span>`).join('');
  }

  // Packets
  document.getElementById('d-packets').textContent = d.packets;
}

// ── Chart ────────────────────────────────────────────────────────────────────
const chartCtx = document.getElementById('trend-chart').getContext('2d');
const trendChart = new Chart(chartCtx, {
  type: 'line',
  data: {
    datasets: [
      { label: 'Speed (km/h)', borderColor: '#F0A500', backgroundColor: 'transparent',
        borderWidth: 1.5, pointRadius: 0, tension: 0.3, data: [] },
      { label: 'Current (A)',  borderColor: '#3DCC7E', backgroundColor: 'transparent',
        borderWidth: 1.2, pointRadius: 0, tension: 0.3, data: [] },
      { label: 'Voltage (V)',  borderColor: '#4A9EFF', backgroundColor: 'transparent',
        borderWidth: 1.2, pointRadius: 0, tension: 0.3, data: [] },
    ]
  },
  options: {
    animation: false,
    responsive: true, maintainAspectRatio: false,
    scales: {
      x: { type: 'linear', ticks: { color: '#6B7280', maxTicksLimit: 8 },
           grid: { color: '#2E3238' } },
      y: { ticks: { color: '#6B7280' }, grid: { color: '#2E3238' } }
    },
    plugins: { legend: { labels: { color: '#E8EAED', boxWidth: 12, font: { size: 11 } } } }
  }
});

async function updateChart() {
  const r = await fetch('/api/history');
  const h = await r.json();
  const now = Date.now() / 1000;
  const toXY = arr => arr.map(([t,v]) => ({x: t - now + 120, y: v}));
  trendChart.data.datasets[0].data = toXY(h.speed);
  trendChart.data.datasets[1].data = toXY(h.current);
  trendChart.data.datasets[2].data = toXY(h.voltage);
  trendChart.update('none');
}
setInterval(updateChart, 2000);

// ── Settings ─────────────────────────────────────────────────────────────────
const SETTINGS_LAYOUT = [
  { title: 'Motor', fields: [
    ['Rated Voltage','rated_voltage','V'],
    ['Rated Power','rated_power','W'],
    ['Rated Speed','rated_speed','RPM'],
    ['Pole Pairs','pole_pairs',''],
    ['Hardware Ver','hardware_ver',''],
    ['Software Ver','software_ver',''],
  ]},
  { title: 'Current Limits', fields: [
    ['Max Line Current','max_line_curr','A'],
    ['Max Phase Current','max_phase_curr','A'],
    ['Custom Max Line','custom_max_line','A'],
    ['Custom Max Phase','custom_max_phase','A'],
  ]},
  { title: 'Speed Levels', fields: [
    ['Low Speed','low_speed','RPM'],
    ['Mid Speed','mid_speed','RPM'],
    ['Back (Reverse)','back_speed','RPM'],
  ]},
  { title: 'Regen', fields: [
    ['Stop Back Current','stop_back_curr','A'],
    ['Max Back Current','max_back_curr','A'],
  ]},
  { title: 'Throttle', fields: [
    ['Throttle Low','throttle_low','V'],
    ['Throttle High','throttle_high','V'],
  ]},
  { title: 'Protection', fields: [
    ['Low Voltage Cutoff','low_vol_protect','V'],
    ['Motor Temp Protect','motor_temp_protect','°C'],
    ['Motor Temp Restore','motor_temp_restore','°C'],
    ['MosFET Temp Protect','mos_temp_protect','°C'],
  ]},
  { title: 'PID', fields: [
    ['Start KI','start_ki',''],['Mid KI','mid_ki',''],['Max KI','max_ki',''],
    ['Start KP','start_kp',''],['Mid KP','mid_kp',''],['Max KP','max_kp',''],
    ['AN (wave type)','an','0-16'],['LM (interval)','lm','0-31'],
  ]},
];

function buildSettingsUI() {
  const container = document.getElementById('settings-container');
  SETTINGS_LAYOUT.forEach(section => {
    const sec = document.createElement('div');
    sec.className = 'setting-section';
    sec.innerHTML = `<h3>${section.title}</h3>`;
    section.fields.forEach(([label, key, unit]) => {
      const row = document.createElement('div');
      row.className = 'setting-row';
      row.innerHTML = `
        <span class="setting-label">${label}</span>
        <span class="setting-value" id="s-${key}">—</span>
        <span class="setting-unit">${unit}</span>`;
      sec.appendChild(row);
    });
    container.appendChild(sec);
  });
}

async function updateSettings() {
  const r = await fetch('/api/settings');
  const s = await r.json();
  for (const [key, val] of Object.entries(s)) {
    const el = document.getElementById('s-' + key);
    if (el) el.textContent = typeof val === 'number'
      ? (Number.isInteger(val) ? val : val.toFixed(1)) : val;
  }
}
buildSettingsUI();
setInterval(updateSettings, 3000);
updateSettings();

// ── Commands ─────────────────────────────────────────────────────────────────
async function sendCmd(name, confirmMsg) {
  if (confirmMsg && !confirm(confirmMsg + '\n\nContinue?')) return;
  const r = await fetch(`/api/cmd/${name}`, {method:'POST'});
  const d = await r.json();
  if (!d.ok) alert('Error: ' + (d.error || 'unknown'));
}

// ── Log ───────────────────────────────────────────────────────────────────────
let allLogEntries = [];

async function loadLog() {
  const r = await fetch('/api/log?n=200');
  allLogEntries = await r.json();
  filterLog();
}

function filterLog() {
  const filter  = document.getElementById('log-filter').value.toLowerCase();
  const showRx  = document.getElementById('log-rx').checked;
  const showTx  = document.getElementById('log-tx').checked;
  const box = document.getElementById('log-box');
  box.innerHTML = allLogEntries
    .filter(e => {
      if (e.level === 'rx' && !showRx) return false;
      if (e.level === 'tx' && !showTx) return false;
      if (filter && !e.msg.toLowerCase().includes(filter)) return false;
      return true;
    })
    .map(e => `<div><span class="log-ts">[${e.ts}]</span> <span class="log-${e.level}">${escHtml(e.msg)}</span></div>`)
    .join('');
  box.scrollTop = box.scrollHeight;
}

function clearLog() {
  allLogEntries = [];
  document.getElementById('log-box').innerHTML = '';
}

function escHtml(s) {
  return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}

// Auto-reload log every 2s when log tab is visible
setInterval(() => {
  if (document.getElementById('tab-log').classList.contains('active')) loadLog();
}, 2000);

// ── Tabs ──────────────────────────────────────────────────────────────────────
function showTab(name) {
  document.querySelectorAll('.tab-content').forEach(el => el.classList.remove('active'));
  document.querySelectorAll('.tab').forEach(el => el.classList.remove('active'));
  document.getElementById('tab-' + name).classList.add('active');
  event.target.classList.add('active');
  if (name === 'log') loadLog();
  if (name === 'settings') updateSettings();
}

// ── Init ──────────────────────────────────────────────────────────────────────
refreshPorts();
setInterval(refreshPorts, 10000);
</script>
</body>
</html>"""

@app.route('/')
def index():
    return HTML

# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="FarDriver web-based controller tool")
    ap.add_argument('--port',  help='Serial port (e.g. /dev/tty.usbserial-AK04P0KB)')
    ap.add_argument('--baud',  type=int, default=9600, help='Baud rate (default 9600)')
    ap.add_argument('--host',  default='0.0.0.0', help='Host to bind (default 0.0.0.0 = all interfaces)')
    ap.add_argument('--webport', type=int, default=5000, help='Web server port (default 5000)')
    args = ap.parse_args()

    add_log("FarDriver Web Tool started", "info")
    add_log("⚠  TTL serial is 3.3 V — do NOT connect Pin 1 (3.3 V supply)!", "warn")

    if args.port:
        threading.Thread(target=lambda: (
            time.sleep(1),
            reader.connect(args.port, args.baud)
        ), daemon=True).start()

    import webbrowser
    url = f"http://localhost:{args.webport}"
    add_log(f"Opening {url}", "info")
    threading.Timer(1.2, lambda: webbrowser.open(url)).start()

    print(f"\n  FarDriver Tool  →  {url}")
    print(f"  On same WiFi   →  http://<your-ip>:{args.webport}\n")

    app.run(host=args.host, port=args.webport, debug=False, threaded=True)

if __name__ == '__main__':
    main()