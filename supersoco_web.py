#!/usr/bin/env python3
"""
Super Soco RS485 Web Monitor
=============================
Browser-based live monitor for the Super Soco TC RS485 bus.
Decodes controller, battery and speedometer telegrams and displays
them as a real-time dashboard — no tkinter required.

Also supports replaying a previously saved raw binary capture.

Usage
-----
  python3 supersoco_web.py --port /dev/tty.usbserial-AK04P0KB
  python3 supersoco_web.py --port /dev/tty.usbserial-AK04P0KB --log ride.bin
  python3 supersoco_web.py --replay ride.bin
  python3 supersoco_web.py --webport 5001       # if 5000 is taken by AirPlay

Then open  http://localhost:5000  (or the port you chose).
From your phone on the same WiFi: http://<mac-ip>:5000

Requirements
------------
  pip install flask pyserial
"""

import argparse
import collections
import csv
import glob
import json
import queue
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from enum import IntEnum, IntFlag
from typing import Optional

import logging
logging.getLogger('werkzeug').setLevel(logging.ERROR)

from flask import Flask, Response, jsonify, request, stream_with_context

# ─────────────────────────────────────────────────────────────────────────────
# Protocol
# ─────────────────────────────────────────────────────────────────────────────

BAUD_RATE         = 9600
END_BYTE          = 0x0D
MAX_PDU_LEN       = 32
TYPE_REQUEST      = 0xC55C
TYPE_RESPONSE     = 0xB66B
CTRL_SPEED_FACTOR = 0.109   # empirically calibrated: 45km/h real = raw≈413

class Unit(IntEnum):
    ECU               = 0xAA
    ENGINE_CONTROLLER = 0xDA
    BATTERY           = 0x5A
    SPEEDOMETER       = 0xBA

UNIT_NAMES = {
    Unit.ECU:               "ECU",
    Unit.ENGINE_CONTROLLER: "Controller",
    Unit.BATTERY:           "Battery",
    Unit.SPEEDOMETER:       "Speedo",
}

class ErrorBits(IntFlag):
    CTRL_DISCONNECT_99        = 0x0001
    CTRL_ERROR_98             = 0x0002
    CTRL_ERROR_97             = 0x0004
    CTRL_HALL_ERROR_96        = 0x0008
    CTRL_ERROR_95             = 0x0010
    BATTERY_DISCONNECT_94     = 0x0020
    BATTERY_CHARGE_CURRENT_93 = 0x0040
    BATTERY_CHARGE_STOPPED_92 = 0x0080
    BATTERY_OVERTEMP_91       = 0x0100
    BATTERY_DISCHARGE_90      = 0x0200
    BATTERY_ERROR_89          = 0x0400
    BATTERY_ERROR_88          = 0x0800

ERROR_NAMES = {
    ErrorBits.CTRL_DISCONNECT_99:        "E99 Ctrl disconnect",
    ErrorBits.CTRL_ERROR_98:             "E98 Ctrl error",
    ErrorBits.CTRL_ERROR_97:             "E97 Ctrl error",
    ErrorBits.CTRL_HALL_ERROR_96:        "E96 Hall sensor",
    ErrorBits.CTRL_ERROR_95:             "E95 Ctrl error",
    ErrorBits.BATTERY_DISCONNECT_94:     "E94 Batt disconnect",
    ErrorBits.BATTERY_CHARGE_CURRENT_93: "E93 Charge current",
    ErrorBits.BATTERY_CHARGE_STOPPED_92: "E92 Charge stopped",
    ErrorBits.BATTERY_OVERTEMP_91:       "E91 Batt overtemp",
    ErrorBits.BATTERY_DISCHARGE_90:      "E90 Discharge current",
    ErrorBits.BATTERY_ERROR_89:          "E89 Batt error",
    ErrorBits.BATTERY_ERROR_88:          "E88 Batt error",
}

# ─────────────────────────────────────────────────────────────────────────────
# Telegram dataclasses (same as supersoco_monitor.py)
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class BaseTelegram:
    raw:           bytes
    telegram_type: int
    destination:   int
    source:        int
    pdu:           bytes
    checksum:      int
    valid:         bool
    timestamp:     datetime = field(default_factory=datetime.now)

    @property
    def src_name(self):
        return UNIT_NAMES.get(self.source,      f"0x{self.source:02X}")
    @property
    def dst_name(self):
        return UNIT_NAMES.get(self.destination, f"0x{self.destination:02X}")
    def hex(self):
        return " ".join(f"{b:02X}" for b in self.raw)
    def type_str(self):
        return "REQ" if self.telegram_type == TYPE_REQUEST else "RSP"
    def label(self):
        return f"{self.type_str()} {self.src_name}→{self.dst_name}"


@dataclass
class SpeedometerRequest(BaseTelegram):
    soc:           int        = 0
    ctrl_current:  float      = 0.0
    speed:         int        = 0
    temp_level:    int        = 0
    hour:          int        = 0
    minute:        int        = 0
    error_bits:    ErrorBits  = ErrorBits(0)
    vehicle_state: int        = 0
    gear:          int        = 0
    speed_ctrl:    int        = 0
    range_km:      int        = 0

@dataclass
class SpeedometerResponse(BaseTelegram):
    pass

@dataclass
class BatteryResponse(BaseTelegram):
    voltage:          int   = 0
    soc:              int   = 0
    temperature:      int   = 0
    current:          int   = 0
    charge_cycles:    int   = 0
    discharge_cycles: int   = 0
    vbreaker:         int   = 0
    activity:         int   = 0
    charging:         bool  = False

@dataclass
class BatteryRequest(BaseTelegram):
    pass

@dataclass
class ControllerResponse(BaseTelegram):
    gear:        int   = 0
    current:     float = 0.0
    speed:       float = 0.0
    temperature: int   = 0
    error_code:  int   = 0
    parking:     int   = 0

@dataclass
class ControllerRequest(BaseTelegram):
    charging: bool = False

# ─────────────────────────────────────────────────────────────────────────────
# Parser
# ─────────────────────────────────────────────────────────────────────────────

def _checksum(pdu_len: int, pdu: bytes) -> int:
    cs = pdu_len
    for b in pdu:
        cs ^= b
    return cs & 0xFF

def _parse_base(raw: bytes, ts: datetime) -> Optional[BaseTelegram]:
    if len(raw) < 7:
        return None
    t_type = (raw[0] << 8) | raw[1]
    if t_type not in (TYPE_REQUEST, TYPE_RESPONSE):
        return None
    dst     = raw[2]
    src     = raw[3]
    pdu_len = raw[4]
    if pdu_len > MAX_PDU_LEN:
        return None
    if len(raw) < 5 + pdu_len + 2:
        return None
    pdu      = raw[5: 5 + pdu_len]
    checksum = raw[5 + pdu_len]
    end      = raw[5 + pdu_len + 1]
    if end != END_BYTE:
        return None
    valid = (_checksum(pdu_len, pdu) == checksum)
    return BaseTelegram(
        raw=raw, telegram_type=t_type,
        destination=dst, source=src,
        pdu=pdu, checksum=checksum, valid=valid, timestamp=ts
    )

def _specialize(base: BaseTelegram) -> BaseTelegram:
    src = base.source
    dst = base.destination
    pdu = base.pdu

    if src == Unit.ECU and dst == Unit.SPEEDOMETER and len(pdu) == 14:
        error_raw = (pdu[6] << 8) | pdu[7]
        return SpeedometerRequest(
            **vars(base),
            soc           = pdu[0],
            ctrl_current  = pdu[1] * 2.5,
            speed         = pdu[2],
            temp_level    = pdu[3],
            hour          = pdu[4],
            minute        = pdu[5],
            error_bits    = ErrorBits(error_raw),
            vehicle_state = pdu[8],
            gear          = pdu[9],
            speed_ctrl    = (pdu[10] << 8) | pdu[11],
            range_km      = pdu[13],
        )
    if src == Unit.SPEEDOMETER and dst == Unit.ECU and len(pdu) == 1:
        return SpeedometerResponse(**vars(base))
    if src == Unit.ECU and dst == Unit.BATTERY and len(pdu) == 1:
        return BatteryRequest(**vars(base))
    if src == Unit.BATTERY and dst == Unit.ECU and len(pdu) == 10:
        current_raw  = pdu[3]
        current_sign = current_raw if current_raw < 128 else current_raw - 256
        temp_raw     = pdu[2]
        temp_sign    = temp_raw if temp_raw < 128 else temp_raw - 256
        act = pdu[9]
        return BatteryResponse(
            **vars(base),
            voltage          = pdu[0],
            soc              = pdu[1],
            temperature      = temp_sign,
            current          = current_sign,
            charge_cycles    = (pdu[4] << 8) | pdu[5],
            discharge_cycles = (pdu[6] << 8) | pdu[7],
            vbreaker         = pdu[8],
            activity         = act,
            charging         = (act == 1),
        )
    if src == Unit.ECU and dst == Unit.ENGINE_CONTROLLER and len(pdu) == 2:
        return ControllerRequest(**vars(base), charging=(pdu[1] == 0x01))
    if src == Unit.ENGINE_CONTROLLER and dst == Unit.ECU and len(pdu) == 10:
        speed_raw    = (pdu[3] << 8) | pdu[4]
        current_raw  = (pdu[1] << 8) | pdu[2]
        temp_raw     = pdu[5]
        temp_sign    = temp_raw if temp_raw < 128 else temp_raw - 256
        return ControllerResponse(
            **vars(base),
            gear        = pdu[0],
            current     = current_raw * 0.1,
            speed       = speed_raw * CTRL_SPEED_FACTOR,
            temperature = temp_sign,
            error_code  = pdu[6],
            parking     = pdu[8],
        )
    return base


class TelegramParser:
    def __init__(self, callback):
        self._cb  = callback
        self._buf = bytearray()

    def feed(self, data: bytes):
        for byte in data:
            self._buf.append(byte)
            if byte == END_BYTE:
                self._try_parse()

    def _try_parse(self):
        buf = bytes(self._buf)
        for start in range(len(buf) - 1):
            pair = (buf[start] << 8) | buf[start + 1]
            if pair in (TYPE_REQUEST, TYPE_RESPONSE):
                raw  = buf[start:]
                base = _parse_base(raw, datetime.now())
                if base and base.valid:
                    tg = _specialize(base)
                    self._cb(tg)
                    self._buf.clear()
                    return
        if len(self._buf) > 64:
            self._buf = self._buf[-2:]

# ─────────────────────────────────────────────────────────────────────────────
# Live state
# ─────────────────────────────────────────────────────────────────────────────

state = {
    "speed":        0.0,
    "gear":         0,
    "soc":          0,
    "range_km":     0,
    "batt_v":       0,
    "batt_temp":    0,
    "batt_curr":    0,
    "batt_cycles_c":0,
    "batt_cycles_d":0,
    "batt_activity":"idle",
    "ctrl_temp":    0,
    "ctrl_curr":    0.0,
    "charging":     False,
    "parking":      False,
    "errors":       [],
    "total_rx":     0,
    "last_update":  None,
    "connected":    False,
    "port":         "",
    "replay_mode":  False,
}

speed_history:   collections.deque = collections.deque(maxlen=600)
current_history: collections.deque = collections.deque(maxlen=600)
voltage_history: collections.deque = collections.deque(maxlen=600)
log_entries:     collections.deque = collections.deque(maxlen=300)

def add_log(msg: str, level: str = "info", terminal: bool = True):
    ts = datetime.now().strftime('%H:%M:%S.%f')[:12]
    log_entries.append({"ts": ts, "msg": msg, "level": level})
    if terminal and level not in ('rx',):
        prefix = {'info':'   ','warn':'⚠  ','error':'✗  '}.get(level, '   ')
        print(f"  {prefix}[{ts}] {msg}")

def on_telegram(tg: BaseTelegram):
    s = state
    s["total_rx"] += 1
    s["last_update"] = tg.timestamp.isoformat()

    label = tg.label()
    hex_s = tg.hex()

    if isinstance(tg, SpeedometerRequest):
        s["speed"]     = tg.speed
        s["gear"]      = tg.gear
        s["soc"]       = tg.soc
        s["range_km"]  = tg.range_km
        s["ctrl_curr"] = tg.ctrl_current
        errors = [n for f, n in ERROR_NAMES.items() if f in tg.error_bits]
        s["errors"] = errors
        speed_history.append((time.time(), tg.speed))
        add_log(f"{label}  SoC={tg.soc}%  Speed={tg.speed}km/h  "
                f"Gear={tg.gear}  Range={tg.range_km}km  "
                f"Curr={tg.ctrl_current:.1f}A  |  {hex_s}", "rx", terminal=False)

    elif isinstance(tg, BatteryResponse):
        s["batt_v"]       = tg.voltage
        s["batt_temp"]    = tg.temperature
        s["batt_curr"]    = tg.current
        s["charging"]     = tg.charging
        s["soc"]          = tg.soc
        s["batt_cycles_c"]= tg.charge_cycles
        s["batt_cycles_d"]= tg.discharge_cycles
        act = {0:"idle", 1:"charging", 4:"discharging"}.get(tg.activity, "?")
        s["batt_activity"] = act
        voltage_history.append((time.time(), tg.voltage))
        add_log(f"{label}  {tg.voltage}V  SoC={tg.soc}%  "
                f"Temp={tg.temperature}°C  Curr={tg.current}A  "
                f"Activity={act}  |  {hex_s}", "rx", terminal=False)

    elif isinstance(tg, ControllerResponse):
        s["speed"]     = tg.speed
        s["gear"]      = tg.gear
        s["ctrl_temp"] = tg.temperature
        s["ctrl_curr"] = tg.current
        s["parking"]   = (tg.parking == 2)
        speed_history.append((time.time(), tg.speed))
        current_history.append((time.time(), tg.current))
        add_log(f"{label}  Gear={tg.gear}  Speed={tg.speed:.1f}km/h  "
                f"Curr={tg.current:.1f}A  Temp={tg.temperature}°C  "
                f"Park={'ON' if tg.parking==2 else 'off'}  "
                f"Err=0x{tg.error_code:02X}  |  {hex_s}", "rx", terminal=False)

    elif isinstance(tg, ControllerRequest):
        add_log(f"{label}  charging={'yes' if tg.charging else 'no'}  |  {hex_s}",
                "rx", terminal=False)

    elif isinstance(tg, BatteryRequest):
        add_log(f"{label}  (poll)  |  {hex_s}", "rx", terminal=False)

    elif isinstance(tg, SpeedometerResponse):
        add_log(f"{label}  (ack)  |  {hex_s}", "rx", terminal=False)

    else:
        add_log(f"{label}  (unknown)  |  {hex_s}", "rx", terminal=False)

    # Print a brief summary to terminal every 200 packets
    if s["total_rx"] % 200 == 0:
        add_log(f"Packet #{s['total_rx']} — "
                f"{s['speed']:.1f}km/h  Gear={s['gear']}  "
                f"SoC={s['soc']}%  {s['batt_v']}V  "
                f"{'⚡ REGEN' if s['batt_curr'] < -0.5 else ''}", "info")

# ─────────────────────────────────────────────────────────────────────────────
# Serial reader
# ─────────────────────────────────────────────────────────────────────────────

class SerialReader:
    def __init__(self):
        self._ser    = None
        self._thread = None
        self._stop   = threading.Event()
        self.port    = ""
        self.baud    = BAUD_RATE

    def connect(self, port: str, baud: int = BAUD_RATE) -> bool:
        try:
            import serial
            self.port = port
            self.baud = baud
            self._ser = serial.Serial(port, baud, timeout=0.1)
            state["connected"] = True
            state["port"]      = port
            state["replay_mode"] = False
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
            add_log(f"Connected to {port} at {baud} baud")
            return True
        except Exception as e:
            add_log(f"Connection failed: {e}", "error")
            return False

    def disconnect(self):
        self._stop.set()
        if self._ser:
            try: self._ser.close()
            except: pass
        state["connected"] = False
        add_log("Disconnected", "warn")

    def _run(self):
        parser = TelegramParser(on_telegram)
        raw_log = state.get("_raw_log")
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(64)
                if chunk:
                    if raw_log:
                        raw_log.write(chunk)
                    parser.feed(chunk)
            except Exception as e:
                if not self._stop.is_set():
                    add_log(f"Serial error: {e}", "error")
                    time.sleep(0.1)

reader = SerialReader()


def replay_file(path: str):
    """Feed a saved raw binary capture through the parser in a background thread."""
    def _run():
        state["replay_mode"] = True
        state["connected"]   = True
        state["port"]        = f"replay:{path}"
        add_log(f"Replaying {path} …")
        parser = TelegramParser(on_telegram)
        with open(path, "rb") as f:
            data = f.read()
        for i in range(0, len(data), 8):
            if not state["replay_mode"]:
                break
            parser.feed(data[i:i+8])
            time.sleep(0.005)
        state["connected"]   = False
        state["replay_mode"] = False
        add_log(f"Replay complete — {state['total_rx']} telegrams decoded")
    threading.Thread(target=_run, daemon=True).start()


def find_ports():
    patterns = ['/dev/tty.usbserial*', '/dev/tty.usbmodem*',
                '/dev/ttyUSB*', '/dev/ttyACM*']
    ports = []
    for p in patterns:
        ports.extend(glob.glob(p))
    return sorted(ports)

# ─────────────────────────────────────────────────────────────────────────────
# Flask + SSE
# ─────────────────────────────────────────────────────────────────────────────

app = Flask(__name__)
app.config['SECRET_KEY'] = 'supersoco'

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
    while True:
        time.sleep(0.2)
        push_sse({'type': 'state', 'data': {k: v for k, v in state.items()
                                              if not k.startswith('_')}})

threading.Thread(target=sse_pusher, daemon=True).start()

@app.route('/events')
def sse():
    q: queue.Queue = queue.Queue(maxsize=30)
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
                    headers={'Cache-Control': 'no-cache', 'X-Accel-Buffering': 'no'})

@app.route('/api/ports')
def api_ports():
    return jsonify(find_ports())

@app.route('/api/connect', methods=['POST'])
def api_connect():
    d    = request.json or {}
    port = d.get('port', '')
    baud = int(d.get('baud', BAUD_RATE))
    log  = d.get('log', '')
    if state["connected"]:
        reader.disconnect()
    if log:
        state["_raw_log"] = open(log, "wb")
        add_log(f"Logging raw bytes to {log}")
    else:
        state["_raw_log"] = None
    ok = reader.connect(port, baud)
    return jsonify({'ok': ok})

@app.route('/api/disconnect', methods=['POST'])
def api_disconnect():
    state["replay_mode"] = False
    reader.disconnect()
    if state.get("_raw_log"):
        state["_raw_log"].close()
        state["_raw_log"] = None
    return jsonify({'ok': True})

@app.route('/api/log')
def api_log():
    n      = int(request.args.get('n', 200))
    level  = request.args.get('level', '')        # filter by level
    search = request.args.get('q', '').lower()    # text search
    entries = list(log_entries)[-n:]
    if level:
        entries = [e for e in entries if e['level'] == level]
    if search:
        entries = [e for e in entries if search in e['msg'].lower()]
    return jsonify(entries)

@app.route('/api/history')
def api_history():
    t0 = time.time() - 300   # last 5 minutes
    return jsonify({
        'speed':   [[t, v] for t, v in speed_history   if t >= t0],
        'current': [[t, v] for t, v in current_history if t >= t0],
        'voltage': [[t, v] for t, v in voltage_history if t >= t0],
    })

@app.route('/api/export/csv')
def api_export_csv():
    """Download all history as CSV."""
    import io
    buf = io.StringIO()
    w   = csv.writer(buf)
    w.writerow(['timestamp', 'speed_kmh', 'current_a', 'voltage_v'])
    all_t = sorted(set(
        [t for t,_ in speed_history] +
        [t for t,_ in current_history] +
        [t for t,_ in voltage_history]
    ))
    spd  = dict(speed_history)
    curr = dict(current_history)
    volt = dict(voltage_history)
    for t in all_t:
        w.writerow([
            datetime.fromtimestamp(t).isoformat(),
            spd.get(t, ''), curr.get(t, ''), volt.get(t, '')
        ])
    return Response(buf.getvalue(), mimetype='text/csv',
                    headers={'Content-Disposition': 'attachment; filename=supersoco_log.csv'})

# ─────────────────────────────────────────────────────────────────────────────
# HTML / CSS / JS
# ─────────────────────────────────────────────────────────────────────────────

HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Super Soco Monitor</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
  :root {
    --bg:    #1A1C1E; --panel: #23262A; --border: #2E3238;
    --amber: #F0A500; --green: #3DCC7E; --red:   #E05252;
    --blue:  #4A9EFF; --muted: #6B7280; --white: #E8EAED;
    --mono:  'Menlo','Consolas',monospace;
  }
  * { box-sizing:border-box; margin:0; padding:0; }
  body { background:var(--bg); color:var(--white); font-family:-apple-system,sans-serif; }

  header {
    display:flex; align-items:center; gap:12px; flex-wrap:wrap;
    padding:10px 20px; border-bottom:1px solid var(--border);
    position:sticky; top:0; background:var(--bg); z-index:100;
  }
  header h1 { font-size:1.2rem; }
  header h1 span { color:var(--amber); }
  .conn-row { display:flex; align-items:center; gap:8px; flex-wrap:wrap; margin-left:auto; }
  select, input[type=text] {
    background:var(--panel); color:var(--white);
    border:1px solid var(--border); border-radius:6px;
    padding:6px 10px; font-size:0.85rem;
  }
  .dot { width:10px; height:10px; border-radius:50%; background:var(--red); display:inline-block; }
  .dot.on { background:var(--green); }
  .dot.replay { background:var(--blue); }
  #conn-label { font-size:0.82rem; color:var(--muted); }

  button {
    cursor:pointer; border:none; border-radius:6px;
    padding:6px 14px; font-size:0.85rem; font-weight:600;
    transition:opacity .15s;
  }
  button:hover { opacity:.85; }
  .btn-amber { background:var(--amber); color:#111; }
  .btn-muted { background:var(--border); color:var(--white); }
  .btn-blue  { background:var(--blue);  color:#111; }

  .tabs { display:flex; gap:2px; padding:10px 20px 0; border-bottom:1px solid var(--border); }
  .tab {
    padding:7px 16px; cursor:pointer; border-radius:6px 6px 0 0;
    font-size:0.88rem; color:var(--muted); background:transparent;
    border:1px solid transparent; border-bottom:none;
  }
  .tab.active { color:var(--amber); background:var(--panel); border-color:var(--border); }
  .tab-content { display:none; padding:14px 20px; }
  .tab-content.active { display:block; }

  /* grid */
  .grid { display:grid; gap:10px; }
  .g4 { grid-template-columns:repeat(4,1fr); }
  .g3 { grid-template-columns:repeat(3,1fr); }
  .g2 { grid-template-columns:repeat(2,1fr); }
  @media(max-width:900px){ .g4{grid-template-columns:repeat(2,1fr);} }
  @media(max-width:580px){ .g4,.g3,.g2{grid-template-columns:1fr;} }

  .card {
    background:var(--panel); border:1px solid var(--border);
    border-radius:10px; padding:12px 14px;
  }
  .clabel { font-size:0.7rem; color:var(--muted); text-transform:uppercase; letter-spacing:.08em; margin-bottom:4px; }
  .cval   { font-family:var(--mono); font-size:1.6rem; font-weight:700; color:var(--white); line-height:1.1; }
  .cval.big   { font-size:2.8rem; color:var(--amber); }
  .cval.green { color:var(--green); }
  .cval.red   { color:var(--red); }
  .cval.blue  { color:var(--blue); }
  .csub { font-size:0.78rem; color:var(--muted); margin-top:4px; }

  .bar-track { background:var(--border); border-radius:4px; height:7px; margin-top:7px; overflow:hidden; }
  .bar-fill  { height:100%; border-radius:4px; background:var(--amber); transition:width .6s; }
  .bar-green { background:var(--green); }
  .bar-red   { background:var(--red); }

  .chart-card { grid-column:1/-1; }
  .chart-wrap { position:relative; height:200px; }

  /* battery detail table */
  .batt-table { width:100%; border-collapse:collapse; font-size:0.82rem; margin-top:6px; }
  .batt-table td { padding:3px 6px; }
  .batt-table td:first-child { color:var(--muted); }
  .batt-table td:last-child  { font-family:var(--mono); text-align:right; }

  /* fault badges */
  .fault { display:inline-block; background:var(--red); color:#fff;
    border-radius:4px; font-size:0.73rem; padding:2px 7px; margin:2px; }
  .no-fault { color:var(--green); font-size:0.88rem; }

  /* regen badge */
  .regen-badge { color:var(--green); font-weight:600; font-size:0.82rem; }

  /* log */
  .log-controls { display:flex; gap:8px; margin-bottom:8px; align-items:center; flex-wrap:wrap; }
  #log-filter { flex:1; min-width:120px; }
  #log-box {
    background:var(--panel); border:1px solid var(--border); border-radius:10px;
    padding:10px; height:500px; overflow-y:auto;
    font-family:var(--mono); font-size:0.76rem; line-height:1.65;
  }
  .log-ts   { color:var(--muted); }
  .log-rx   { color:var(--green); }
  .log-info { color:var(--white); }
  .log-warn { color:#FFC107; }
  .log-error{ color:var(--red); }
</style>
</head>
<body>

<header>
  <h1><span>Super Soco</span> RS485 Monitor</h1>
  <div class="conn-row">
    <select id="port-sel"><option value="">Select port…</option></select>
    <select id="baud-sel">
      <option value="9600" selected>9600</option>
      <option value="19200">19200</option>
    </select>
    <input type="text" id="log-path" placeholder="log file (optional)" style="width:160px">
    <button class="btn-muted" onclick="refreshPorts()">⟳</button>
    <button class="btn-amber" id="conn-btn" onclick="toggleConnect()">Connect</button>
    <button class="btn-blue"  onclick="pickReplay()">▶ Replay</button>
    <input type="file" id="replay-input" style="display:none" accept=".bin,*" onchange="startReplay()">
    <span class="dot" id="conn-dot"></span>
    <span id="conn-label">Disconnected</span>
  </div>
</header>

<div class="tabs">
  <div class="tab active" onclick="showTab('dashboard',this)">Dashboard</div>
  <div class="tab" onclick="showTab('battery',this)">Battery Detail</div>
  <div class="tab" onclick="showTab('log',this)">Log</div>
</div>

<!-- ═════════════════ DASHBOARD ═════════════════ -->
<div id="tab-dashboard" class="tab-content active">
  <div class="grid g4" style="margin-bottom:10px">
    <div class="card" style="grid-column:span 2">
      <div class="clabel">Speed</div>
      <div class="cval big" id="d-speed">0.0</div>
      <div class="csub">km/h &nbsp;|&nbsp; <span id="d-gear">GEAR —</span>
        &nbsp; <span id="d-dir">·</span>
        &nbsp; <span id="d-park"></span></div>
    </div>
    <div class="card">
      <div class="clabel">Battery SoC</div>
      <div class="cval green" id="d-soc">— %</div>
      <div class="bar-track"><div class="bar-fill bar-green" id="d-soc-bar" style="width:0%"></div></div>
      <div class="csub" id="d-charge"></div>
    </div>
    <div class="card">
      <div class="clabel">Voltage</div>
      <div class="cval" id="d-volt">— V</div>
      <div class="csub" id="d-volt-sub"></div>
    </div>
  </div>

  <div class="grid g4" style="margin-bottom:10px">
    <div class="card">
      <div class="clabel">Current (controller)</div>
      <div class="cval" id="d-curr">— A</div>
      <div class="csub" id="d-regen"></div>
    </div>
    <div class="card">
      <div class="clabel">Battery Current</div>
      <div class="cval" id="d-bcurr">— A</div>
      <div class="csub" id="d-bact"></div>
    </div>
    <div class="card">
      <div class="clabel">Controller Temp</div>
      <div class="cval" id="d-ctemp">— °C</div>
    </div>
    <div class="card">
      <div class="clabel">Range / Faults</div>
      <div class="csub" style="font-size:0.9rem;color:var(--white)" id="d-range">— km</div>
      <div style="margin-top:6px" id="d-faults"><span class="no-fault">✓ No faults</span></div>
    </div>
  </div>

  <div class="card chart-card">
    <div class="clabel" style="margin-bottom:6px">Trend — last 5 minutes</div>
    <div class="chart-wrap"><canvas id="trend-chart"></canvas></div>
  </div>

  <div style="text-align:right;margin-top:8px;font-size:0.78rem;color:var(--muted)">
    Packets: <span id="d-packets">0</span> &nbsp;|&nbsp;
    Updated: <span id="d-ts">—</span> &nbsp;|&nbsp;
    <a href="/api/export/csv" style="color:var(--amber);text-decoration:none">⬇ Export CSV</a>
  </div>
</div>

<!-- ═════════════════ BATTERY DETAIL ═════════════════ -->
<div id="tab-battery" class="tab-content">
  <div class="grid g2">
    <div class="card">
      <div class="clabel">Battery Status</div>
      <table class="batt-table">
        <tr><td>Voltage</td>        <td><span id="b-volt">—</span> V</td></tr>
        <tr><td>State of Charge</td><td><span id="b-soc">—</span> %</td></tr>
        <tr><td>Current</td>        <td><span id="b-curr">—</span> A</td></tr>
        <tr><td>Temperature</td>    <td><span id="b-temp">—</span> °C</td></tr>
        <tr><td>Activity</td>       <td><span id="b-act">—</span></td></tr>
        <tr><td>Charge cycles</td>  <td><span id="b-cycc">—</span></td></tr>
        <tr><td>Discharge cycles</td><td><span id="b-cycd">—</span></td></tr>
      </table>
    </div>
    <div class="card">
      <div class="clabel">Controller Status</div>
      <table class="batt-table">
        <tr><td>Speed</td>       <td><span id="b-speed">—</span> km/h</td></tr>
        <tr><td>Gear</td>        <td><span id="b-gear">—</span></td></tr>
        <tr><td>Current</td>     <td><span id="b-ccurr">—</span> A</td></tr>
        <tr><td>Temperature</td> <td><span id="b-ctemp">—</span> °C</td></tr>
        <tr><td>Parking</td>     <td><span id="b-park">—</span></td></tr>
      </table>
    </div>
  </div>
</div>

<!-- ═════════════════ LOG ═════════════════ -->
<div id="tab-log" class="tab-content">
  <div class="log-controls">
    <input type="text" id="log-filter" placeholder="Search…" oninput="filterLog()">
    <label style="font-size:0.82rem;color:var(--muted)">
      <input type="checkbox" id="chk-rx" checked onchange="filterLog()"> RX
    </label>
    <label style="font-size:0.82rem;color:var(--muted)">
      <input type="checkbox" id="chk-info" checked onchange="filterLog()"> Info
    </label>
    <button class="btn-muted" onclick="clearLog()">Clear</button>
    <button class="btn-muted" onclick="loadLog()">Reload</button>
  </div>
  <div id="log-box"></div>
</div>

<script>
// ── Connection ────────────────────────────────────────────────────────────────
let isConnected = false;

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
  if (isConnected) {
    await fetch('/api/disconnect', {method:'POST'});
  } else {
    const port = document.getElementById('port-sel').value;
    const baud = document.getElementById('baud-sel').value;
    const log  = document.getElementById('log-path').value.trim();
    if (!port) { alert('Select a serial port first'); return; }
    const r = await fetch('/api/connect', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify({port, baud: parseInt(baud), log})
    });
    const d = await r.json();
    if (!d.ok) alert('Connection failed');
  }
}

function pickReplay() {
  document.getElementById('replay-input').click();
}

async function startReplay() {
  const file = document.getElementById('replay-input').files[0];
  if (!file) return;
  // Upload file then start replay
  const fd = new FormData();
  fd.append('file', file);
  // We just pass filename; server reads local path — show alert with instructions
  alert('Replay: run from command line:\n\npython3 supersoco_web.py --replay <path-to-file>');
}

// ── SSE ────────────────────────────────────────────────────────────────────────
const es = new EventSource('/events');
es.onmessage = e => {
  const msg = JSON.parse(e.data);
  if (msg.type === 'state') updateUI(msg.data);
};

function updateUI(s) {
  isConnected = s.connected;
  const dot = document.getElementById('conn-dot');
  dot.className = 'dot' + (s.replay_mode ? ' replay' : s.connected ? ' on' : '');
  document.getElementById('conn-label').textContent =
    s.replay_mode ? `Replaying · ${s.port.replace('replay:','')}` :
    s.connected   ? `Connected · ${s.port}` : 'Disconnected';
  document.getElementById('conn-btn').textContent =
    s.connected ? 'Disconnect' : 'Connect';

  // Speed
  document.getElementById('d-speed').textContent = s.speed.toFixed(1);
  const gears = ['P','Low','Mid','High'];
  document.getElementById('d-gear').textContent = 'GEAR ' + (gears[s.gear] || s.gear);
  document.getElementById('d-park').textContent = s.parking ? '🅿 PARK' : '';

  // SoC
  const socEl = document.getElementById('d-soc');
  socEl.textContent = s.soc + ' %';
  socEl.className = 'cval ' + (s.soc < 20 ? 'red' : 'green');
  const bar = document.getElementById('d-soc-bar');
  bar.style.width = s.soc + '%';
  bar.className = 'bar-fill ' + (s.soc < 20 ? 'bar-red' : 'bar-green');
  document.getElementById('d-charge').textContent =
    s.charging ? '⚡ Charging' : '';

  // Voltage
  document.getElementById('d-volt').textContent = s.batt_v + ' V';

  // Controller current
  const cEl = document.getElementById('d-curr');
  cEl.textContent = s.ctrl_curr.toFixed(1) + ' A';
  cEl.className = 'cval' + (s.ctrl_curr < -0.5 ? ' green' : '');
  document.getElementById('d-regen').innerHTML =
    s.ctrl_curr < -0.5 ? '<span class="regen-badge">⚡ REGEN</span>' : '';

  // Battery current
  const bcEl = document.getElementById('d-bcurr');
  bcEl.textContent = s.batt_curr + ' A';
  bcEl.className = 'cval' + (s.batt_curr < -0.5 ? ' green' : '');
  document.getElementById('d-bact').textContent = s.batt_activity;

  // Controller temp
  const ctEl = document.getElementById('d-ctemp');
  ctEl.textContent = s.ctrl_temp + ' °C';
  ctEl.className = 'cval' + (s.ctrl_temp > 100 ? ' red' : '');

  // Range + faults
  document.getElementById('d-range').textContent = s.range_km + ' km estimated';
  const fEl = document.getElementById('d-faults');
  fEl.innerHTML = s.errors.length
    ? s.errors.map(e => `<span class="fault">${e}</span>`).join('')
    : '<span class="no-fault">✓ No faults</span>';

  // Footer
  document.getElementById('d-packets').textContent = s.total_rx;
  document.getElementById('d-ts').textContent = s.last_update
    ? s.last_update.substring(11,19) : '—';

  // Battery detail tab
  document.getElementById('b-volt').textContent  = s.batt_v;
  document.getElementById('b-soc').textContent   = s.soc;
  document.getElementById('b-curr').textContent  = s.batt_curr;
  document.getElementById('b-temp').textContent  = s.batt_temp;
  document.getElementById('b-act').textContent   = s.batt_activity;
  document.getElementById('b-cycc').textContent  = s.batt_cycles_c;
  document.getElementById('b-cycd').textContent  = s.batt_cycles_d;
  document.getElementById('b-speed').textContent = s.speed.toFixed(1);
  document.getElementById('b-gear').textContent  = s.gear;
  document.getElementById('b-ccurr').textContent = s.ctrl_curr.toFixed(1);
  document.getElementById('b-ctemp').textContent = s.ctrl_temp;
  document.getElementById('b-park').textContent  = s.parking ? 'ON' : 'off';
}

// ── Chart ─────────────────────────────────────────────────────────────────────
const ctx = document.getElementById('trend-chart').getContext('2d');
const chart = new Chart(ctx, {
  type: 'line',
  data: {
    datasets: [
      { label:'Speed (km/h)',   borderColor:'#F0A500', backgroundColor:'transparent',
        borderWidth:1.5, pointRadius:0, tension:0.3, data:[] },
      { label:'Current (A)',    borderColor:'#3DCC7E', backgroundColor:'transparent',
        borderWidth:1.2, pointRadius:0, tension:0.3, data:[] },
      { label:'Voltage (V)',    borderColor:'#4A9EFF', backgroundColor:'transparent',
        borderWidth:1.2, pointRadius:0, tension:0.3, data:[] },
    ]
  },
  options: {
    animation: false, responsive:true, maintainAspectRatio:false,
    scales: {
      x: { type:'linear', ticks:{color:'#6B7280',maxTicksLimit:8}, grid:{color:'#2E3238'} },
      y: { ticks:{color:'#6B7280'}, grid:{color:'#2E3238'} }
    },
    plugins: { legend:{ labels:{color:'#E8EAED', boxWidth:12, font:{size:11}} } }
  }
});

async function updateChart() {
  const r = await fetch('/api/history');
  const h = await r.json();
  const now = Date.now() / 1000;
  const toXY = arr => arr.map(([t,v]) => ({x: t - now + 300, y: v}));
  chart.data.datasets[0].data = toXY(h.speed);
  chart.data.datasets[1].data = toXY(h.current);
  chart.data.datasets[2].data = toXY(h.voltage);
  chart.update('none');
}
setInterval(updateChart, 2000);

// ── Log ────────────────────────────────────────────────────────────────────────
let allLog = [];

async function loadLog() {
  const r = await fetch('/api/log?n=300');
  allLog = await r.json();
  filterLog();
}

function filterLog() {
  const q      = document.getElementById('log-filter').value.toLowerCase();
  const showRx = document.getElementById('chk-rx').checked;
  const showInf= document.getElementById('chk-info').checked;
  const box    = document.getElementById('log-box');
  box.innerHTML = allLog
    .filter(e => {
      if (e.level === 'rx'   && !showRx)  return false;
      if (e.level === 'info' && !showInf) return false;
      if (q && !e.msg.toLowerCase().includes(q)) return false;
      return true;
    })
    .map(e => `<div><span class="log-ts">[${e.ts}]</span> <span class="log-${e.level}">${esc(e.msg)}</span></div>`)
    .join('');
  box.scrollTop = box.scrollHeight;
}

function clearLog() { allLog = []; document.getElementById('log-box').innerHTML = ''; }
function esc(s) { return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;'); }

setInterval(() => {
  if (document.getElementById('tab-log').classList.contains('active')) loadLog();
}, 2000);

// ── Tabs ──────────────────────────────────────────────────────────────────────
function showTab(name, el) {
  document.querySelectorAll('.tab-content').forEach(e => e.classList.remove('active'));
  document.querySelectorAll('.tab').forEach(e => e.classList.remove('active'));
  document.getElementById('tab-'+name).classList.add('active');
  el.classList.add('active');
  if (name === 'log') loadLog();
}

// ── Init ──────────────────────────────────────────────────────────────────────
refreshPorts();
setInterval(refreshPorts, 15000);
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
    ap = argparse.ArgumentParser(description="Super Soco RS485 web monitor")
    ap.add_argument('--port',    help='Serial port')
    ap.add_argument('--baud',    type=int, default=BAUD_RATE)
    ap.add_argument('--log',     metavar='FILE', help='Save raw bytes to file')
    ap.add_argument('--replay',  metavar='FILE', help='Replay a saved raw binary')
    ap.add_argument('--host',    default='0.0.0.0')
    ap.add_argument('--webport', type=int, default=5000)
    args = ap.parse_args()

    add_log("Super Soco RS485 Web Monitor started")

    if args.replay:
        threading.Timer(1.5, lambda: replay_file(args.replay)).start()
    elif args.port:
        if args.log:
            state["_raw_log"] = open(args.log, "wb")
            add_log(f"Logging raw bytes to {args.log}")
        threading.Timer(1.0, lambda: reader.connect(args.port, args.baud)).start()

    import webbrowser
    url = f"http://localhost:{args.webport}"
    threading.Timer(1.2, lambda: webbrowser.open(url)).start()

    print(f"\n  Super Soco Monitor  →  {url}")
    print(f"  On same WiFi        →  http://<your-ip>:{args.webport}\n")

    app.run(host=args.host, port=args.webport, debug=False, threaded=True)

if __name__ == '__main__':
    main()