#!/usr/bin/env python3
"""
Super Soco RS485 Monitor
========================
Decodes and visualises the RS485 bus between the ECU, speedometer,
controller and battery on the Super Soco TC (and compatible models).

Protocol reference: https://github.com/stprograms/SuperSoco485Monitor

Usage
-----
    # Live monitoring from a serial port:
    python supersoco_monitor.py --port /dev/ttyUSB0

    # Log raw bytes to a file while monitoring:
    python supersoco_monitor.py --port /dev/ttyUSB0 --log raw.bin

    # Replay a previously saved raw binary log:
    python supersoco_monitor.py --replay raw.bin

    # Save decoded telegrams to CSV:
    python supersoco_monitor.py --port /dev/ttyUSB0 --csv data.csv

Requirements
------------
    pip install pyserial rich

    The live dashboard requires a terminal that supports ANSI colour codes
    (any modern Linux/macOS terminal, Windows Terminal on Win 10+).
"""

import argparse
import csv
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from enum import IntEnum, IntFlag
from typing import Callable, Optional

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------

BAUD_RATE   = 9600
END_BYTE    = 0x0D
MAX_PDU_LEN = 32

# ControllerResponse speed scaling.
# The C# source used 0.028 but empirical data (11.6 reported vs 45 indicated)
# shows the correct factor is ~0.109.  Derived: 45 / (raw≈413) ≈ 0.109.
# Adjust here if your bike reads differently.
CTRL_SPEED_FACTOR = 0.109

TYPE_REQUEST  = 0xC55C   # ECU → unit
TYPE_RESPONSE = 0xB66B   # unit → ECU

# Unit addresses
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

# ---------------------------------------------------------------------------
# Error / fault flags (from ErrorCode.cs)
# ---------------------------------------------------------------------------

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

# ---------------------------------------------------------------------------
# Telegram dataclasses
# ---------------------------------------------------------------------------

@dataclass
class BaseTelegram:
    raw:         bytes
    telegram_type: int      # 0xC55C or 0xB66B
    destination: int
    source:      int
    pdu:         bytes
    checksum:    int
    valid:       bool
    timestamp:   datetime = field(default_factory=datetime.now)

    @property
    def src_name(self):
        return UNIT_NAMES.get(self.source, f"0x{self.source:02X}")

    @property
    def dst_name(self):
        return UNIT_NAMES.get(self.destination, f"0x{self.destination:02X}")

    def hex(self):
        return " ".join(f"{b:02X}" for b in self.raw)

    def __str__(self):
        direction = "REQ" if self.telegram_type == TYPE_REQUEST else "RSP"
        return (f"[{self.timestamp:%H:%M:%S.%f}] {direction} "
                f"{self.src_name}→{self.dst_name}  {self.hex()}")


@dataclass
class SpeedometerRequest(BaseTelegram):
    """ECU → Speedometer  (14-byte PDU)"""
    soc:          int   = 0    # %
    ctrl_current: float = 0.0  # A  (raw * 2.5)
    speed:        int   = 0    # km/h (low-res byte)
    temp_level:   int   = 0
    hour:         int   = 0
    minute:       int   = 0
    error_bits:   ErrorBits = ErrorBits(0)
    vehicle_state:int   = 0
    gear:         int   = 0
    speed_ctrl:   int   = 0    # high-res speed word
    range_km:     int   = 0

    def __str__(self):
        errors = [n for f, n in ERROR_NAMES.items() if f in self.error_bits]
        err_str = ", ".join(errors) if errors else "none"
        return (f"[{self.timestamp:%H:%M:%S}] SPEEDO REQ  "
                f"SoC={self.soc}%  Speed={self.speed}km/h  "
                f"Gear={self.gear}  Range={self.range_km}km  "
                f"Current={self.ctrl_current:.1f}A  "
                f"Temp={self.temp_level}  Errors=[{err_str}]")


@dataclass
class SpeedometerResponse(BaseTelegram):
    """Speedometer → ECU  (1-byte PDU, ACK)"""
    def __str__(self):
        return f"[{self.timestamp:%H:%M:%S}] SPEEDO RSP  (ack)"


@dataclass
class BatteryResponse(BaseTelegram):
    """Battery → ECU  (10-byte PDU)"""
    voltage:          int   = 0   # V
    soc:              int   = 0   # %
    temperature:      int   = 0   # °C (signed)
    current:          int   = 0   # A (signed; negative = discharge)
    charge_cycles:    int   = 0
    discharge_cycles: int   = 0
    vbreaker:         int   = 0
    activity:         int   = 0   # 0=idle 1=charging 4=discharging
    charging:         bool  = False

    def __str__(self):
        act = {0: "idle", 1: "charging", 4: "discharging"}.get(self.activity, "?")
        return (f"[{self.timestamp:%H:%M:%S}] BATTERY RSP  "
                f"{self.voltage}V  SoC={self.soc}%  "
                f"Temp={self.temperature}°C  Current={self.current}A  "
                f"Cycles={self.charge_cycles}/{self.discharge_cycles}  "
                f"Activity={act}")


@dataclass
class BatteryRequest(BaseTelegram):
    """ECU → Battery  (1-byte PDU, poll)"""
    def __str__(self):
        return f"[{self.timestamp:%H:%M:%S}] BATTERY REQ  (poll)"


@dataclass
class ControllerResponse(BaseTelegram):
    """Engine Controller → ECU  (10-byte PDU)"""
    gear:        int   = 0
    current:     float = 0.0   # A
    speed:       float = 0.0   # km/h  (raw * CTRL_SPEED_FACTOR, empirically calibrated)
    temperature: int   = 0     # °C
    error_code:  int   = 0
    parking:     int   = 0     # 1=off 2=on

    def __str__(self):
        park = {1: "off", 2: "ON"}.get(self.parking, "?")
        return (f"[{self.timestamp:%H:%M:%S}] CTRL RSP  "
                f"Gear={self.gear}  Speed={self.speed:.1f}km/h  "
                f"Current={self.current:.1f}A  Temp={self.temperature}°C  "
                f"Park={park}  Err=0x{self.error_code:02X}")


@dataclass
class ControllerRequest(BaseTelegram):
    """ECU → Engine Controller  (2-byte PDU, poll)"""
    charging: bool = False

    def __str__(self):
        return (f"[{self.timestamp:%H:%M:%S}] CTRL REQ  "
                f"charging={'yes' if self.charging else 'no'}")


# ---------------------------------------------------------------------------
# Parser
# ---------------------------------------------------------------------------

def _checksum(pdu_len: int, pdu: bytes) -> int:
    """XOR of length byte and all PDU bytes."""
    cs = pdu_len
    for b in pdu:
        cs ^= b
    return cs & 0xFF


def _parse_base(raw: bytes, ts: datetime) -> Optional[BaseTelegram]:
    """Turn a raw byte block into a BaseTelegram, or None on error."""
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
    """Upgrade a BaseTelegram to its specific subclass."""
    src = base.source
    dst = base.destination
    pdu = base.pdu

    # Speedometer Request: ECU(AA) → Speedo(BA), 14-byte PDU
    if src == Unit.ECU and dst == Unit.SPEEDOMETER and len(pdu) == 14:
        error_raw = (pdu[6] << 8) | pdu[7]
        return SpeedometerRequest(
            **vars(base),
            soc          = pdu[0],
            ctrl_current = pdu[1] * 2.5,
            speed        = pdu[2],
            temp_level   = pdu[3],
            hour         = pdu[4],
            minute       = pdu[5],
            error_bits   = ErrorBits(error_raw),
            vehicle_state= pdu[8],
            gear         = pdu[9],
            speed_ctrl   = (pdu[10] << 8) | pdu[11],
            range_km     = pdu[13],
        )

    # Speedometer Response: Speedo(BA) → ECU(AA), 1-byte PDU
    if src == Unit.SPEEDOMETER and dst == Unit.ECU and len(pdu) == 1:
        return SpeedometerResponse(**vars(base))

    # Battery Request: ECU(AA) → Battery(5A), 1-byte PDU
    if src == Unit.ECU and dst == Unit.BATTERY and len(pdu) == 1:
        return BatteryRequest(**vars(base))

    # Battery Response: Battery(5A) → ECU(AA), 10-byte PDU
    if src == Unit.BATTERY and dst == Unit.ECU and len(pdu) == 10:
        current_raw = pdu[3]
        current_signed = current_raw if current_raw < 128 else current_raw - 256
        temp_raw = pdu[2]
        temp_signed = temp_raw if temp_raw < 128 else temp_raw - 256
        act = pdu[9]
        return BatteryResponse(
            **vars(base),
            voltage          = pdu[0],
            soc              = pdu[1],
            temperature      = temp_signed,
            current          = current_signed,
            charge_cycles    = (pdu[4] << 8) | pdu[5],
            discharge_cycles = (pdu[6] << 8) | pdu[7],
            vbreaker         = pdu[8],
            activity         = act,
            charging         = (act == 1),
        )

    # Controller Request: ECU(AA) → Controller(DA), 2-byte PDU
    if src == Unit.ECU and dst == Unit.ENGINE_CONTROLLER and len(pdu) == 2:
        return ControllerRequest(**vars(base), charging=(pdu[1] == 0x01))

    # Controller Response: Controller(DA) → ECU(AA), 10-byte PDU
    if src == Unit.ENGINE_CONTROLLER and dst == Unit.ECU and len(pdu) == 10:
        speed_raw   = (pdu[3] << 8) | pdu[4]
        current_raw = (pdu[1] << 8) | pdu[2]
        temp_raw    = pdu[5]
        temp_signed = temp_raw if temp_raw < 128 else temp_raw - 256
        return ControllerResponse(
            **vars(base),
            gear        = pdu[0],
            current     = current_raw * 0.1,
            speed       = speed_raw * CTRL_SPEED_FACTOR,
            temperature = temp_signed,
            error_code  = pdu[6],
            parking     = pdu[8],
        )

    return base


class TelegramParser:
    """Feed bytes in; receive decoded telegrams via callback."""

    def __init__(self, callback: Callable[[BaseTelegram], None]):
        self._cb  = callback
        self._buf = bytearray()

    def feed(self, data: bytes):
        for byte in data:
            self._buf.append(byte)
            # A telegram ends with END_BYTE (0x0D)
            if byte == END_BYTE:
                self._try_parse()

    def _try_parse(self):
        buf = bytes(self._buf)
        # Scan for a valid start sequence
        for start in range(len(buf) - 1):
            pair = (buf[start] << 8) | buf[start + 1]
            if pair in (TYPE_REQUEST, TYPE_RESPONSE):
                raw = buf[start:]
                ts  = datetime.now()
                base = _parse_base(raw, ts)
                if base and base.valid:
                    tg = _specialize(base)
                    self._cb(tg)
                    self._buf.clear()
                    return
        # Nothing valid found — keep only the last byte in case it starts next telegram
        if len(self._buf) > 64:
            self._buf = self._buf[-2:]


# ---------------------------------------------------------------------------
# Live dashboard (uses Rich if available, falls back to plain text)
# ---------------------------------------------------------------------------

def _try_import_rich():
    try:
        from rich.console import Console
        from rich.table import Table
        from rich.live import Live
        from rich.panel import Panel
        from rich.columns import Columns
        from rich import box
        return Console, Table, Live, Panel, Columns, box
    except ImportError:
        return None


class Dashboard:
    """Keeps latest state and renders a terminal dashboard."""

    def __init__(self, csv_path: Optional[str] = None, verbose: bool = False):
        self.verbose    = verbose
        self._rich      = _try_import_rich()
        self._state: dict = {
            "speed":       0.0,
            "gear":        0,
            "soc":         0,
            "range_km":    0,
            "batt_v":      0,
            "batt_temp":   0,
            "batt_curr":   0,
            "ctrl_temp":   0,
            "ctrl_curr":   0.0,
            "charging":    False,
            "parking":     False,
            "errors":      [],
            "last_update": None,
            "total_rx":    0,
        }
        self._history: dict = {
            "speed":    deque(maxlen=60),
            "soc":      deque(maxlen=60),
            "batt_v":   deque(maxlen=60),
        }
        self._csv_path = csv_path
        self._csv_file = None
        self._csv_writer = None
        if csv_path:
            self._csv_file   = open(csv_path, "w", newline="")
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                "timestamp", "type",
                "speed_kmh", "gear", "soc_pct", "range_km",
                "batt_voltage", "batt_temp_c", "batt_current_a",
                "ctrl_temp_c", "ctrl_current_a",
                "charging", "parking", "errors"
            ])

    def update(self, tg: BaseTelegram):
        s = self._state
        s["total_rx"] += 1
        s["last_update"] = tg.timestamp

        if isinstance(tg, SpeedometerRequest):
            s["speed"]    = tg.speed
            s["gear"]     = tg.gear
            s["soc"]      = tg.soc
            s["range_km"] = tg.range_km
            s["ctrl_curr"] = tg.ctrl_current
            errors = [n for f, n in ERROR_NAMES.items() if f in tg.error_bits]
            s["errors"] = errors
            self._history["speed"].append(tg.speed)
            self._history["soc"].append(tg.soc)

        elif isinstance(tg, BatteryResponse):
            s["batt_v"]    = tg.voltage
            s["batt_temp"] = tg.temperature
            s["batt_curr"] = tg.current
            s["charging"]  = tg.charging
            s["soc"]       = tg.soc   # battery SoC is authoritative when speedo msgs absent
            self._history["batt_v"].append(tg.voltage)

        elif isinstance(tg, ControllerResponse):
            s["speed"]    = tg.speed
            s["gear"]     = tg.gear   # gear always comes from controller
            s["ctrl_temp"]= tg.temperature
            s["ctrl_curr"]= tg.current
            s["parking"]  = (tg.parking == 2)
            self._history["speed"].append(tg.speed)

        if self.verbose:
            print(tg)

        if self._csv_writer:
            self._csv_writer.writerow([
                tg.timestamp.isoformat(),
                type(tg).__name__,
                s["speed"], s["gear"], s["soc"], s["range_km"],
                s["batt_v"], s["batt_temp"], s["batt_curr"],
                s["ctrl_temp"], s["ctrl_curr"],
                s["charging"], s["parking"],
                "|".join(s["errors"]),
            ])
            self._csv_file.flush()

    def render_plain(self):
        s = self._state
        err = ", ".join(s["errors"]) if s["errors"] else "none"
        ts  = s["last_update"].strftime("%H:%M:%S") if s["last_update"] else "--:--:--"
        print(
            f"\r[{ts}]  "
            f"Speed={s['speed']:.1f}km/h  Gear={s['gear']}  "
            f"SoC={s['soc']}%  Range={s['range_km']}km  "
            f"Batt={s['batt_v']}V/{s['batt_temp']}°C  "
            f"Ctrl={s['ctrl_temp']}°C  "
            f"Charging={'Y' if s['charging'] else 'N'}  "
            f"Park={'Y' if s['parking'] else 'N'}  "
            f"Errors=[{err}]  RX={s['total_rx']}",
            end="", flush=True
        )

    def render_rich(self, live, Console, Table, Panel, Columns, box):
        s   = self._state
        err = ", ".join(s["errors"]) if s["errors"] else "✓ none"
        ts  = s["last_update"].strftime("%H:%M:%S") if s["last_update"] else "--:--:--"

        def _bar(val, max_val, width=20, char="█", empty="░"):
            filled = int(round(val / max_val * width))
            filled = max(0, min(width, filled))
            return char * filled + empty * (width - filled)

        speed_bar = _bar(s["speed"], 90)
        soc_bar   = _bar(s["soc"],   100)

        tbl = Table(box=box.SIMPLE, show_header=False, padding=(0, 1))
        tbl.add_column(style="bold cyan",  width=18)
        tbl.add_column(style="white",      width=30)

        tbl.add_row("Speed",    f"{s['speed']:.1f} km/h  {speed_bar}")
        tbl.add_row("Gear",     str(s["gear"]))
        tbl.add_row("SoC",      f"{s['soc']}%  {soc_bar}")
        tbl.add_row("Range",    f"{s['range_km']} km")
        tbl.add_row("Battery",  f"{s['batt_v']} V  {s['batt_temp']}°C  {s['batt_curr']} A")
        tbl.add_row("Ctrl",     f"{s['ctrl_temp']}°C  {s['ctrl_curr']:.1f} A")
        tbl.add_row("Charging", "⚡ YES" if s["charging"] else "no")
        tbl.add_row("Parking",  "🅿 YES" if s["parking"]  else "no")
        tbl.add_row("Errors",   f"[red]{err}[/red]" if s["errors"] else f"[green]{err}[/green]")
        tbl.add_row("Packets",  str(s["total_rx"]))
        tbl.add_row("Updated",  ts)

        live.update(Panel(tbl, title="[bold yellow]Super Soco RS485 Monitor[/bold yellow]",
                          border_style="yellow"))

    def close(self):
        if self._csv_file:
            self._csv_file.close()


# ---------------------------------------------------------------------------
# Raw byte logger
# ---------------------------------------------------------------------------

class RawLogger:
    def __init__(self, path: str):
        self._f = open(path, "wb")

    def write(self, data: bytes):
        self._f.write(data)
        self._f.flush()

    def close(self):
        self._f.close()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def run_live(port: str, log_path: Optional[str], csv_path: Optional[str],
             verbose: bool, baud: int):
    try:
        import serial
    except ImportError:
        print("pyserial not installed.  Run:  pip install pyserial", file=sys.stderr)
        sys.exit(1)

    dashboard  = Dashboard(csv_path=csv_path, verbose=verbose)
    raw_logger = RawLogger(log_path) if log_path else None
    parser     = TelegramParser(dashboard.update)
    rich_mods  = dashboard._rich

    print(f"Opening {port} at {baud} baud …")
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"Could not open port: {e}", file=sys.stderr)
        sys.exit(1)

    print("Listening  (Ctrl-C to stop)\n")

    if rich_mods:
        Console, Table, Live, Panel, Columns, box = rich_mods
        console = Console()
        with Live(console=console, refresh_per_second=4) as live:
            try:
                while True:
                    chunk = ser.read(64)
                    if chunk:
                        if raw_logger:
                            raw_logger.write(chunk)
                        parser.feed(chunk)
                        dashboard.render_rich(live, Console, Table, Panel, Columns, box)
            except KeyboardInterrupt:
                pass
    else:
        try:
            while True:
                chunk = ser.read(64)
                if chunk:
                    if raw_logger:
                        raw_logger.write(chunk)
                    parser.feed(chunk)
                    dashboard.render_plain()
        except KeyboardInterrupt:
            print()

    ser.close()
    if raw_logger:
        raw_logger.close()
    dashboard.close()
    print("\nDone.")


def run_replay(replay_path: str, csv_path: Optional[str], verbose: bool):
    dashboard = Dashboard(csv_path=csv_path, verbose=verbose)
    parser    = TelegramParser(dashboard.update)
    rich_mods = dashboard._rich

    print(f"Replaying {replay_path} …\n")

    with open(replay_path, "rb") as f:
        data = f.read()

    if rich_mods:
        Console, Table, Live, Panel, Columns, box = rich_mods
        console = Console()
        # Feed byte by byte with small delay to simulate live stream
        with Live(console=console, refresh_per_second=10) as live:
            for i in range(0, len(data), 8):
                parser.feed(data[i:i+8])
                dashboard.render_rich(live, Console, Table, Panel, Columns, box)
                time.sleep(0.01)
    else:
        for i in range(0, len(data), 8):
            parser.feed(data[i:i+8])
        dashboard.render_plain()
        print()

    dashboard.close()
    print(f"\nReplayed {len(data)} bytes  →  {dashboard._state['total_rx']} telegrams decoded.")


def main():
    ap = argparse.ArgumentParser(
        description="Super Soco RS485 bus monitor and decoder",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    ap.add_argument("--port",    help="Serial port, e.g. /dev/ttyUSB0 or COM3")
    ap.add_argument("--baud",    type=int, default=BAUD_RATE, help=f"Baud rate (default {BAUD_RATE})")
    ap.add_argument("--log",     metavar="FILE", help="Save raw bytes to binary file")
    ap.add_argument("--replay",  metavar="FILE", help="Replay a saved raw binary file")
    ap.add_argument("--csv",     metavar="FILE", help="Save decoded data to CSV")
    ap.add_argument("--verbose", action="store_true", help="Print every telegram to stdout")
    args = ap.parse_args()

    if args.replay:
        run_replay(args.replay, args.csv, args.verbose)
    elif args.port:
        run_live(args.port, args.log, args.csv, args.verbose, args.baud)
    else:
        ap.print_help()
        print("\nError: specify --port or --replay", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()