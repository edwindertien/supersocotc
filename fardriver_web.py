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
  python3 fardriver_web.py --port /dev/tty.usbserial-AK04P0KB --baud 19200  # confirmed working on the bench

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
CMD_RESET_SOFT    = build_syscmd(0x05)   # reboots controller WITHOUT committing pending changes first
CMD_RESET         = build_syscmd(0x09)   # soft reset

# ─────────────────────────────────────────────────────────────────────────────
# CONFIRMED against a real captured Android-app session (silent monitor,
# 2026-07-27): the real app sends syscmd 0x04 shortly after each group of
# parameter writes, and the values genuinely persisted through a power cycle
# afterward -- this is the real save/commit trigger, not 0x05.
#
# 0x04 DOES cause the controller to reboot (confirmed by the BT module's own
# "SOK" / AT+VERSION / AT+PAWD re-handshake showing up shortly after it in
# 2 of 3 observed instances, 3-8 seconds later -- the module only redoes that
# handshake when its UART link to the controller was actually broken, i.e.
# the controller reset). The difference from 0x05 isn't "reboots vs doesn't" --
# both reboot. The difference is 0x04 commits pending changes to flash first,
# then reboots; 0x05 just reboots, discarding anything not yet committed.
# One of the three observed instances showed no visible reconnect in the
# captured window -- inconsistent, not fully understood, but the persistence
# result itself (confirmed by Edwin after a real power cycle) is solid.
CMD_COMMIT_FLASH  = build_syscmd(0x04)

def write_param(flash_addr: int, value_u16: int) -> bytes:
    """
    Write a 16-bit parameter to a flash address.
    From README: Format: AA C6 [addr] [addr] [lo] [hi] [crc_a] [crc_b]
    Both bytes 2 and 3 are the SAME flash address (addr_confirm).
    flash_addr = block_base_addr + byte_offset_within_block
    e.g. LowVolProtect: block 0x1E + byte 2 = flash addr 0x20
    """
    lo = value_u16 & 0xFF
    hi = (value_u16 >> 8) & 0xFF
    raw = bytes([0xAA, 0xC6, flash_addr, flash_addr, lo, hi])
    ca, cb = _crc(raw)
    return raw + bytes([ca, cb])

def write_word_0x0B(temp_sensor: int = None, direction: int = None,
                     brake_config: int = None) -> bytes:
    """
    Read-modify-write for packed word 0x0B (Addr06 block, bytes 10-11 of the
    12-byte payload). This word packs SEVERAL unrelated settings together:
      byte10 (word low byte):  BrakeConfig:4 (bits0-3) | TempSensor:3 (bits4-6) | PhaseExchange:1 (bit7)
      byte11 (word high byte): SlowDown:3 | PC13Config:1 | CurrAntiTheft:1 | ParkConfig:2 (bits5-6) | Direction:1 (bit7)
    Verified by compiling the real jackhumbert/fardriver-controllers fardriver.hpp
    struct with g++ and reading actual offsetof()/byte-layout results — do NOT
    trust the header's inline "// 0x.." comments for this block, they're wrong.

    brake_config bit position (0-3) and the real app's exact read-modify-write
    logic were independently confirmed by decompiling ProControlPage's
    BrakeConfig_SelectedIndexChanged handler: it does
    `data[4] = (cfg11l & 0xF0) | selected_index` -- i.e. preserves the upper
    nibble (TempSensor/PhaseExchange) and only touches bits0-3. Same thing
    this function already did for temp_sensor/direction, just one more field.

    Naively overwriting the whole word (as the old temp_sensor write did) also
    stomps BrakeConfig/PhaseExchange/SlowDown/PC13Config/CurrAntiTheft/ParkConfig
    to zero — plausibly part of why past writes near this region misbehaved.
    This function reads the last live-captured raw block and only flips the
    requested bit(s), preserving everything else — including PhaseExchange,
    which this tool no longer offers a write UI for (Direction alone covers
    the rotation-direction use case) but whose current value is still
    preserved untouched on every write here.
    """
    raw = _block_cache.get(0x06)
    if raw is None or len(raw) < 12:
        raise RuntimeError("no live 0x06 block cached yet — wait for data to "
                            "start streaming before writing temp_sensor, direction, "
                            "or brake_config (needed to safely preserve the other "
                            "settings packed into the same word)")
    b10, b11 = raw[10], raw[11]
    if temp_sensor is not None:
        b10 = (b10 & ~0x70) | ((temp_sensor & 0x07) << 4)
    if brake_config is not None:
        b10 = (b10 & ~0x0F) | (brake_config & 0x0F)
    if direction is not None:
        b11 = (b11 & ~0x80) | ((1 if direction else 0) << 7)
    word_val = b10 | (b11 << 8)
    return write_param(0x0B, word_val)

def write_block(msg_id: int, block_data: bytes) -> bytes:
    """
    Write a full 12-byte block using the 16-byte block-write format.
    Uses flag=1 (write) with the message id — mirror of the read format.
    Format: AA [0x40|msg_id] [data 12 bytes] [crc_a] [crc_b]
    This is needed for parameters with no Send() annotation (e.g. LowVolProtect).
    msg_id: index into flashReadAddr table (e.g. 11 for addr 0x1E)
    block_data: full 12-byte block with the modified value(s)
    """
    assert len(block_data) == 12
    header = 0x40 | (msg_id & 0x3F)
    raw = bytes([0xAA, header]) + block_data
    ca, cb = _crc(raw)
    return raw + bytes([ca, cb])

# Cached last-seen block data for block-write parameters
_block_cache: dict = {}   # flash_addr -> bytes (last seen 12-byte block)

def update_block_cache(addr: int, data: bytes):
    """Called by decoder to cache full blocks for potential block-writes."""
    _block_cache[addr] = bytes(data)

# ─────────────────────────────────────────────────────────────────────────────
# Battery SoC estimate — same formula the real app itself uses (found directly
# in fardriver.hpp as a helper method):
#   GetBatteryP() = 100 * (deci_volts - ZeroBattCoeff) / (FullBattCoeff - ZeroBattCoeff)
# ZeroBattCoeff/FullBattCoeff are the "0%"/"100%" reference voltages (Addr0C,
# words 0x0D/0x0E — right after PhaseOffset at 0x0C, compiler/sequential-field
# verified same as StartKI etc. in that block). This is a simple linear
# interpolation the app computes locally from raw controller voltage, as
# opposed to whatever `batt_cap` (AddrF4 byte3) is doing on the firmware
# side -- if that raw byte looks wrong, this is the more trustworthy path
# since it's exactly what the real app shows.
_soc_calib = {'zero': None, 'full': None, 'raw_deciv': None}

def _recompute_batt_soc_calc():
    z, f, v = _soc_calib['zero'], _soc_calib['full'], _soc_calib['raw_deciv']
    if z is None or f is None or v is None or f == z:
        return
    pct = 100.0 * (v - z) / (f - z)
    live.batt_soc_calc = max(0.0, min(100.0, pct))

# Parameter map for clean, STANDALONE 16-bit words only (word IS the value,
# no other settings packed into the same word — safe to overwrite outright).
# PARAM_MAP: (word_addr, scale, unit, min, max)
# WORD addressing confirmed by static_assert in fardriver.hpp:
#   offsetof(FardriverData, addrXX) == (0x12 << 1)
# Verified by actually compiling jackhumbert/fardriver-controllers' fardriver.hpp
# with g++ and reading real offsetof()/struct-layout results — the inline
# "// 0x.." comments in that header do NOT reliably match the compiled byte
# offsets, so don't trust them without cross-checking.
# DO NOT write to addr12 (word 0x12) — that is LD (weak magnetic coefficient)
#
# temp_sensor and direction are NOT here: both live inside packed word 0x0B
# (shared with BrakeConfig/PhaseExchange/SlowDown/PC13Config/CurrAntiTheft/
# ParkConfig) and are written via write_word_0x0B() below, which does a
# read-modify-write so sibling bits in that word aren't clobbered.
PARAM_MAP = {
    'rated_speed':        (0x18, 1,   'RPM', 100,  8000),  # Addr18 "// 0x18"
    'max_line_curr':      (0x19, 4,   'A',   10,   200),   # Addr18 "// 0x19"
    'low_vol_protect':    (0x1F, 10,  'V',   40,   70),    # Addr1E "// 0x1F" byte2/2=1
    'stop_back_curr':     (0x30, 1,   'A',   0,    60),    # Addr30 "// 0x30"
    'max_back_curr':      (0x31, 1,   'A',   0,    80),    # Addr30 "// 0x31"
    'back_speed':         (0x28, 1,   'RPM', 0,    3000),  # Addr24 "// 0x28"
    'low_speed':          (0x29, 1,   'RPM', 100,  6000),  # Addr24 "// 0x29"
    'max_phase_curr':     (0x2D, 4,   'A',   10,   400),   # Addr2A "// 0x2D"
    # These need addr block verification before enabling:
    # 'rated_voltage':   (0x17, 10, 'V', 48, 75),   # DO NOT USE until confirmed
    # 'rated_power':     (0x16, 1,  'W', 500, 10000),
    # 'motor_temp_protect': TBD - addr82 uint8 packing needs verification
}

# Human-readable NTC/temp sensor type names (from SiAECOSYS manual)
TEMP_SENSOR_NAMES = {
    0: 'None (disabled)',
    1: 'PTC-1000',
    2: 'NTC-230K',
    3: 'KTY84-130',
    4: 'Hypothetical',
    5: 'KTY83-122',
    6: 'NTC-10K',
    7: 'NTC-100K',
}

# Per the official FarDriver manual §12.7 "Log in": some controllers have an
# optional 30-digit password. If set, the controller reports parameters for
# viewing only — modifications require a prior Login with the correct
# password. This tool does not implement that login exchange, so if this
# reads as password-protected, that's a strong candidate explanation for
# why writes don't survive a reset.
PASSWORD_STATUS_NAMES = {
    0: 'Password-protected — Login required to modify',
    1: 'Password-protected (alt) — Login required to modify',
    2: 'No password — free to modify',
}

# Real UI labels, pulled directly from the decompiled Android app
# (ProControlPage::BrakeConfig_SelectedIndexChanged) -- these are the actual
# English strings the real app shows for this exact setting, not a guess
# from the header's enum names. "P+" isn't spelled out anywhere in the app's
# English strings; likely a modifier on the same Ground/Float pairing, exact
# meaning of "P" not confirmed.
# Bit position (word 0x0B, low byte, bits0-3) confirmed the same way.
BRAKE_CONFIG_NAMES = {
    0: '0 — Stop Valid (brake active when signal connects to ground)',
    1: '1 — Inverse Stop (brake active when signal floats/disconnects)',
    2: '2 — P+Stop',
    3: '3 — P+Inverse Stop',
    4: '4 — Disabled',
}

# Verify factory reset CRC matches known-good from README
assert list(CMD_FACTORY_RESET[-2:]) == [0xC5, 0x09], "CRC table error"

# ─────────────────────────────────────────────────────────────────────────────
# "Old-style" Sending-Commands protocol (README §Sending commands)
# ─────────────────────────────────────────────────────────────────────────────
# Distinct 8-byte packet format, distinct (much simpler) checksum from the
# two-table CRC used everywhere else in this file:
#   AA <command> <~command> <sub_command> <value1> <value2> <crc> <~crc>
#   crc = sum(all 6 preceding bytes) & 0xFF
#
# WHY THIS MATTERS FOR THE WRITE-PERSISTENCE PROBLEM:
# fardriver.hpp annotates many settings fields with "Send(cat, idx)", e.g.:
#   TempSensor : 3;  // Send(0x11, 0x01)
#   Direction  : 1;  // Send(0x12, 0x07)
#   PolePairs, MaxSpeed, RatedPower, RatedVoltage, RatedSpeed,
#   MaxLineCurr, MaxPhaseCurr, StopBackCurr, MaxBackCurr -> all Send(0x12, ...)
# This is almost certainly the packet format the REAL FarDriver app uses to
# push a settings change through firmware logic (which may be what commits it
# to flash) -- as opposed to the raw word-write (0xC6 ...) we've been using,
# which reads back correctly in RAM but has NOT been shown to survive a
# power cycle. UNTESTED against real hardware -- the cat/idx values come
# straight from the struct comments, but value1/value2 encoding is a
# reasonable guess (raw value in value1, 0 in value2), not a captured example.
def build_send_cmd(command: int, sub_command: int, value1: int = 0, value2: int = 0) -> bytes:
    command &= 0xFF
    cmd_comp = (~command) & 0xFF
    sub_command &= 0xFF
    value1 &= 0xFF
    value2 &= 0xFF
    payload = bytes([0xAA, command, cmd_comp, sub_command, value1, value2])
    crc = sum(payload) & 0xFF
    crc_comp = (~crc) & 0xFF
    return payload + bytes([crc, crc_comp])

# Verify the checksum formula against a real captured packet from context.md:
# "AA 05 FA 01 5F 5F 68 97" -- ConnectPage.cs, sent after a date/time write.
assert build_send_cmd(0x05, 0x01, 0x5F, 0x5F) == \
    bytes([0xAA, 0x05, 0xFA, 0x01, 0x5F, 0x5F, 0x68, 0x97]), "send-cmd checksum mismatch"

# CONFIRMED against a real captured Android-app session (silent monitor,
# 2026-07-27): the real app sends this exact 8-byte packet roughly once per
# second, continuously, for the entire time it's connected (values 9/111
# never change across the whole session). Command 0x13 is the login/binding
# system per the community README; sub-command 0x07 "seems to be related to
# the login/status update system". Not confirmed whether this is *required*
# for persistence or just something the real app also happens to do
# alongside it, but it's cheap to replicate and it was present throughout
# the session where persistence was confirmed working, so this tool now
# sends it too while connected.
CMD_HEARTBEAT = build_send_cmd(0x13, 0x07, 9, 111)
assert CMD_HEARTBEAT == bytes([0xAA, 0x13, 0xEC, 0x07, 0x09, 0x6F, 0x28, 0xD7]), \
    "heartbeat packet mismatch"

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
    # Companion-phone / login status bits from AddrE2 — separate from the
    # PasswordStatus preflight check (which is "is a password required").
    # These are "does the controller currently consider itself bound/OK'd by
    # a companion app". compPhoneOK verified bit7 of byte0; passOK verified
    # bits3-4 of byte1 (compiler-checked against AddrE2).
    comp_phone_ok: bool = False
    pass_ok: int = 0
    speed_kmh: float = 0.0
    raw_speed_value: int = 0  # unscaled MeasureSpeed, for recalibrating the km/h factor
    batt_soc_calc: float = 0.0  # voltage-based estimate, same formula the real app uses
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
    temp_sensor: int = 0
    direction: int = 0
    brake_config: int = 0
    phase_exchange: int = 0
    park_config: int = 0
    password_status: int = -1  # -1 = not yet observed; 0/1 = password set, 2 = no password
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
log_entries:   collections.deque = collections.deque(maxlen=2000)
write_pending: dict = {}   # param -> (expected_float, unit, timestamp)

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
    if len(data) < 12:
        return   # malformed frame
    addr = FLASH_READ_ADDR[msg_id] if msg_id < len(FLASH_READ_ADDR) else msg_id

    # addr=0x00: Calibration/fixed params — cache SaveNum for write commits
    if addr == 0x00:
        import struct as _s
        savenum = _s.unpack_from('<H', data, 10)[0]
        _block_cache['savenum'] = savenum
        return

    # addr=0x37: Curve buffer frames (2048-sample acceleration recording)
    # Bytes 2-3 carry the sample value; bytes 4-11 are zero padding.
    # These are a bulk dump on startup — just count them, don't decode as live data.
    if addr == 0x37 or addr == 0xDC:
        return

    if addr == 0xE2:
        b0 = data[0]
        live.forward           = bool(b0 & 0x01)
        live.reverse            = bool(b0 & 0x02)
        live.gear               = (b0 >> 2) & 0x03
        # bit4 = sliding_backwards ("Reversing"), bit5 = motion ("rollingV") —
        # verified by compiling the real fardriver.hpp struct; this was
        # previously reading bit4 (the wrong flag) and could show speed=0
        # while actually moving, or vice versa.
        live.motion              = bool(b0 & 0x20)
        # compPhoneOK: byte0 bit7 — "is the controller currently OK with a
        # companion phone/app" per fardriver.hpp field name. passOK: byte1
        # bits3-4 (2-bit field). Both compiler-verified against AddrE2.
        # Distinct from the PasswordStatus preflight check: that one asks
        # "does this controller require a password"; these ask "does the
        # controller currently consider itself bound/authorized by an app".
        live.comp_phone_ok       = bool(b0 & 0x80)
        b1 = data[1]
        live.pass_ok             = (b1 >> 3) & 0x03
        b2 = data[2]
        live.hall_error        = bool(b2 & 0x01)
        live.throttle_error    = bool(b2 & 0x02)
        live.motor_temp_protect= bool(b2 & 0x40)
        live.ctrl_temp_protect = bool(b2 & 0x80)
        live.brake             = bool(data[3] & 0x80)
        live.modulation        = data[4] / 128.0 * 100.0
        raw_speed              = struct.unpack_from('<H', data, 6)[0]
        live.raw_speed_value   = raw_speed
        # 0.109 (README.md, context.md) was calibrated for the STOCK Super
        # Soco RS485 bus's OWN speed field ("ControllerResponse" telegram) --
        # a different protocol, from before FarDriver was even installed.
        # It was never actually calibrated for THIS field (AddrE2.MeasureSpeed,
        # FarDriver's own internal speed value) -- reusing it here was a
        # mismatch from the start, not a "slightly stale" calibration.
        #
        # PROVISIONAL correction based on one reported data point (2026-07-27):
        # displayed speed hit ~450 when true speed was ~45 -- consistent with
        # raw_speed ≈ 4128 and a correct factor near 0.0109 (0.109 / 10).
        # This is NOT a real calibration, just a rough same-ballpark fix from
        # a single approximate observation. Get a proper one when possible:
        # hold a steady, known reference speed (phone GPS, or known wheel
        # RPM), note raw_speed_value from the dashboard at that moment, then
        # correct_factor = true_kmh / raw_speed_value.
        spd = raw_speed * 0.0109
        # Only show speed if motion flag is set -- raw value is noise at idle.
        # No longer zeroing out large values: that was hiding the actual
        # miscalibration signal instead of surfacing it.
        live.speed_kmh = spd if live.motion else 0.0
        live.timestamp         = time.time()

    elif addr == 0xE8:
        raw_deciv         = struct.unpack_from('<h', data, 0)[0]
        live.voltage      = raw_deciv / 10.0
        live.line_current = struct.unpack_from('<h', data, 4)[0] / 4.0
        _soc_calib['raw_deciv'] = raw_deciv
        _recompute_batt_soc_calc()

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
        # PhaseLostAlarm is bit 11 of s1 (bytes2-3), NOT of s2 (bytes4-5) —
        # was reading the wrong word, meaning this fault flag was effectively
        # dead. Verified by compiling the real fardriver.hpp AddrD6 struct.
        live.phase_lost     = bool(s1 & 0x0800)
        s2 = struct.unpack_from('<H', data, 4)[0]
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
        # bytes 4-5: boost line curr, 6-7: boost phase curr (optional)

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
        update_block_cache(0x06, data)
        # NOTE: verified against a compiled copy of the actual fardriver.hpp
        # struct (g++ + offsetof), not the header's inline "// 0x.." comments,
        # which don't match the real compiled layout for this block.
        # word 0x08 = ThrottleLow (lo byte) / ThrottleHigh (hi byte)
        settings.throttle_low  = data[4] / 20.0
        settings.throttle_high = data[5] / 20.0
        # word 0x0B, low byte (data[10]):  BrakeConfig:4 | TempSensor:3 (bits4-6) | PhaseExchange:1 (bit7)
        # word 0x0B, high byte (data[11]): SlowDown:3 | PC13Config:1 | CurrAntiTheft:1 | ParkConfig:2 (bits5-6) | Direction:1 (bit7)
        settings.brake_config   = data[10] & 0x0F
        settings.temp_sensor    = (data[10] >> 4) & 0x07
        settings.phase_exchange = (data[10] >> 7) & 0x01
        settings.park_config    = (data[11] >> 5) & 0x03
        settings.direction      = (data[11] >> 7) & 0x01

    elif addr == 0x0C:
        # PhaseOffset(word0x0C, bytes0-1), ZeroBattCoeff(word0x0D, bytes2-3),
        # FullBattCoeff(word0x0E, bytes4-5) -- sequential fields, no packing,
        # same block that already correctly gives StartKI at data[6] etc.
        _soc_calib['zero'] = struct.unpack_from('<h', data, 2)[0]
        _soc_calib['full'] = struct.unpack_from('<h', data, 4)[0]
        _recompute_batt_soc_calc()
        settings.start_ki = data[6]; settings.mid_ki = data[7]
        settings.max_ki   = data[8]; settings.start_kp = data[9]
        settings.mid_kp   = data[10]; settings.max_kp = data[11]

    elif addr == 0x1E:
        settings.low_vol_protect = struct.unpack_from('<H', data, 2)[0] / 10.0
        update_block_cache(0x1E, data)

    elif addr == 0x82:
        settings.motor_temp_protect = data[4]
        settings.motor_temp_restore = data[5]
        settings.mos_temp_protect   = data[6]
        update_block_cache(0x82, data)

        # HardwareVersion is data[9], SoftwareVersionMajor is data[10],
        # SoftwareVersionMinor is data[11] — compiler-verified against Addr82.
        # Was previously reading HW from data[10] (actually SW major), which
        # is why context.md notes "HW ver H/8" — data[10] really did read as
        # 8 (SW major), it just wasn't the hardware version.
        hw_byte = data[9]
        hw = chr(hw_byte) if 32 <= hw_byte <= 126 else str(hw_byte)
        settings.hardware_ver = hw

        sw_major_byte = data[10]
        sw_major = chr(sw_major_byte) if 32 <= sw_major_byte <= 126 else str(sw_major_byte)
        settings.software_ver = f"{sw_major}.{data[11]}"

    elif addr == 0x9A:
        # AN is data[4], LM is data[5] — compiler-verified against Addr9A
        # (was previously reading data[6]/data[7], which is InitVol's low byte)
        settings.an = data[4] & 0x0F
        settings.lm = data[5] & 0x1F

    elif addr == 0xB8:
        # word 0xBC, low byte: CANBaud:2 (bits0-1), unkBC:2 (bits2-3),
        # PasswordStatus:2 (bits4-5), unkBCb:2 (bits6-7) — compiler-verified
        # against the real AddrB8 struct. See PASSWORD_STATUS_NAMES / manual
        # §12.7 "Log in" — this is the field that tells us whether this
        # specific controller requires a login before writes can persist.
        update_block_cache(0xB8, data)
        settings.password_status = (data[9] >> 4) & 0x03

    elif addr == 0x7C:
        # Product info: modify year/month/day + pole pairs confirmation
        try:
            year  = (data[6] << 8) | data[7]
            month = data[8]
            day   = data[9]
            if 2020 <= year <= 2030 and 1 <= month <= 12 and 1 <= day <= 31:
                add_log(f"Controller build date: {year}-{month:02d}-{day:02d}",
                        "info", terminal=True)
        except Exception:
            pass

    elif addr == 0x63:
        # Speed ratios / gear current limits
        # bytes 0-1: low speed line ratio, 2-3: low phase ratio
        # bytes 4-5: mid speed line ratio, 6-7: mid phase ratio  
        pass

    elif addr == 0x69:
        # Temperature protection bytes
        # From log: F0 F5 23 F7 FF F6 AC 0D 00 00 41 42
        # bytes 10-11: 0x41='A', 0x42='B' — possibly firmware flags
        pass

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
        self._last_heartbeat = 0.0

    def connect(self, port: str, baud: int = 9600) -> bool:
        try:
            import serial
            self.port = port
            self.baud = baud
            self._ser = serial.Serial(port, baud, timeout=0.2, write_timeout=1.0)
            self.connected = True
            self._stop.clear()
            self._last_heartbeat = 0.0   # send one immediately once connected
            # Reset live data so stale values from previous session don't show
            live.packets = 0
            live.timestamp = 0.0
            self._thread = threading.Thread(target=self._reader, daemon=True)
            self._thread.start()
            add_log(f"Connected to {port} at {baud} baud", "info")
            return True
        except Exception as e:
            add_log(f"Connection failed: {e}", "error")
            return False

    def disconnect(self):
        self._stop.set()
        # Wait briefly for reader thread to notice stop event
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None
        if self._ser:
            try: self._ser.close()
            except: pass
            self._ser = None
        self.connected = False
        self.port = ""
        add_log("Disconnected — ready to reconnect", "warn")

    def send(self, data: bytes, label: str = ""):
        hex_str = ' '.join(f'{b:02X}' for b in data)
        if not self.connected:
            add_log(f"TX DROPPED (not connected) {label}: {hex_str}", "error",
                    terminal=True)
            return
        if not self._ser:
            add_log(f"TX DROPPED (no serial port) {label}: {hex_str}", "error",
                    terminal=True)
            return
        try:
            with self._lock:
                n = self._ser.write(data)
                self._ser.flush()
            add_log(f"TX {label} ({n}B): {hex_str}", "tx", terminal=True)
        except Exception as e:
            add_log(f"TX FAILED {label}: {e}  data={hex_str}", "error",
                    terminal=True)

    def _do_bt_handshake(self) -> bool:
        """
        Emulate the BT module AT handshake (Saleae capture analysis):

        Timing from capture:
          - Controller sends AT+VERSION (10 bytes, no terminator)
          - BT dongle responds within ~40ms: +VERSION=<16 binary bytes>\r\n (27 bytes)
          - Controller processes for ~29ms then sends AT+PAWD=<16 bytes> (24 bytes)
          - BT dongle responds: +PAWD=<16 bytes> (22 bytes, NO terminator)
          - Repeat PAWD 3 times
          - Controller switches to 0xAA data frames

        Critical: controller sends AT+VERSION in a tight loop (~75ms period).
        We must respond within one loop period. Use small read chunks (5ms
        timeout) so we catch each transmission window.

        The PAWD response has NO \r\n terminator (22 bytes exactly).
        The VERSION response DOES have \r\n (27 bytes).
        """
        # Exact bytes from Saleae capture of real BT dongle
        VERSION_RESPONSE = bytes([
            0x2b,0x56,0x45,0x52,0x53,0x49,0x4f,0x4e,0x3d,  # +VERSION=
            0x01,0x08,0x6b,0x19,0x39,0xa4,0xe2,0x7e,
            0x86,0x97,0xd8,0xe6,0x7b,0xfd,0xb1,0x67,
            0x0d,0x0a                                         # \r\n
        ])
        PAWD_RESPONSE = bytes([
            0x2b,0x50,0x41,0x57,0x44,0x3d,                  # +PAWD=
            0xce,0x1d,0x0d,0x8d,0x39,0x9f,0xd3,0xf3,
            0x86,0x91,0xf6,0x42,0x7c,0x77,0x85,0xe1          # no terminator
        ])

        add_log("Starting BT handshake emulation…", "info")
        add_log(f"VERSION_RESPONSE ({len(VERSION_RESPONSE)}B): "
                f"{VERSION_RESPONSE.hex(' ')}", "info", terminal=True)
        add_log(f"PAWD_RESPONSE ({len(PAWD_RESPONSE)}B): "
                f"{PAWD_RESPONSE.hex(' ')}", "info", terminal=True)

        # Save original timeout, switch to fast polling
        orig_timeout = self._ser.timeout
        self._ser.timeout = 0.005   # 5ms — must catch within ~40ms window

        deadline   = time.time() + 4.0
        buf        = bytearray()
        pawd_count = 0
        version_sent = False

        try:
            while time.time() < deadline and not self._stop.is_set():
                chunk = self._ser.read(32)
                if chunk:
                    buf.extend(chunk)
                    add_log(f"HS RX ({len(chunk)}B): {bytes(chunk).hex(' ')}  "
                            f"{bytes(chunk)!r}", "rx", terminal=True)

                raw = bytes(buf)

                # Data mode — we're done
                if 0xAA in raw:
                    add_log(f"✓ DATA MODE after {pawd_count} PAWD(s) ✓", "info",
                            terminal=True)
                    if write_pending:
                        add_log(f"Pending verification after reset: "
                                f"{list(write_pending.keys())}", "info", terminal=True)
                    self._ser.timeout = orig_timeout
                    return True

                # VERSION: respond as soon as we see AT+VERSION
                if b'AT+VERSION' in raw and not version_sent:
                    add_log(f"HS: AT+VERSION detected — sending +VERSION response",
                            "info", terminal=True)
                    with self._lock:
                        written = self._ser.write(VERSION_RESPONSE)
                        self._ser.flush()
                    add_log(f"HS TX +VERSION ({written}B): "
                            f"{VERSION_RESPONSE.hex(' ')}", "tx", terminal=True)
                    version_sent = True
                    buf.clear()

                # PAWD: respond to each AT+PAWD
                elif b'AT+PAWD' in raw:
                    pawd_count += 1
                    add_log(f"HS: AT+PAWD detected — sending +PAWD response "
                            f"#{pawd_count}", "info", terminal=True)
                    with self._lock:
                        written = self._ser.write(PAWD_RESPONSE)
                        self._ser.flush()
                    add_log(f"HS TX +PAWD #{pawd_count} ({written}B): "
                            f"{PAWD_RESPONSE.hex(' ')}", "tx", terminal=True)
                    buf.clear()

                # More AT+VERSION after we already responded — ignore, keep waiting
                elif b'AT+VERSION' in raw and version_sent:
                    add_log(f"HS: repeated AT+VERSION (controller hasn't seen "
                            f"our response yet — normal)", "rx", terminal=True)
                    buf.clear()

                # Trim buffer
                elif len(buf) > 128:
                    buf = buf[-32:]

        finally:
            self._ser.timeout = orig_timeout

        add_log(f"✗ BT handshake timeout: {pawd_count} PAWD(s) sent, "
                f"version_sent={version_sent}\n"
                f"  If still looping AT+VERSION: controller not seeing our TX.\n"
                f"  Check: 1) adapter TX → controller RXD pin (not TXD)\n"
                f"         2) 3.3V adapter (not 5V)\n"
                f"         3) try baud 19200\n"
                f"         4) use physical BT dongle",
                "warn", terminal=True)
        return False

    def _reader(self):
        # ── Phase 1: probe to detect AT vs data mode ─────────────────────────
        add_log("Serial reader started — probing for AT/data mode…", "info")
        time.sleep(0.3)

        # Accumulate a longer probe window to catch the full AT+VERSION string
        probe = bytearray()
        t_end = time.time() + 1.5   # up to 1.5s probe window
        while time.time() < t_end:
            chunk = self._ser.read(32)
            if chunk:
                probe.extend(chunk)
                add_log(f"Probe RX ({len(chunk)}B): {bytes(chunk)!r}  "
                        f"hex: {chunk.hex(' ')}", "rx", terminal=True)
                if b'AT+' in bytes(probe):
                    add_log(f"AT mode confirmed in probe — starting handshake", "warn")
                    break
                if 0xAA in probe:
                    add_log(f"Data mode confirmed in probe — skipping handshake", "info")
                    break
            else:
                add_log("Probe: no bytes yet…", "rx", terminal=False)

        probe = bytes(probe)
        add_log(f"Probe complete: {len(probe)} bytes total  "
                f"AT={'yes' if b'AT+' in probe else 'no'}  "
                f"DataMode={'yes' if 0xAA in probe else 'no'}", "info")

        if b'AT+' in probe:
            self._do_bt_handshake()
        elif not probe:
            add_log("No bytes received during probe — check wiring and baud rate", "warn")

        # ── Phase 2: normal FarDriver data parsing ────────────────────────────
        buf = bytearray()
        if probe:
            buf.extend(probe)   # don't discard bytes already read

        while not self._stop.is_set():
            try:
                # Confirmed via passive capture of the real app: it sends this
                # exact heartbeat roughly once per second, continuously, the
                # entire time it's connected. Replicate that here.
                now = time.time()
                if now - self._last_heartbeat >= 1.0:
                    self.send(CMD_HEARTBEAT, "HEARTBEAT")
                    self._last_heartbeat = now

                chunk = self._ser.read(64)
                if not chunk:
                    continue
                buf.extend(chunk)

                # Detect if we've fallen back into AT mode (e.g. after timeout)
                if len(buf) > 32 and b'AT+' in bytes(buf):
                    add_log("Fell back to AT mode — re-handshaking…", "warn")
                    self._do_bt_handshake()
                    self._last_heartbeat = time.time()
                    buf.clear()
                    continue

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
                        try:
                            decode(msg_id, data)
                        except Exception as dec_err:
                            addr_d = FLASH_READ_ADDR[msg_id] if msg_id < len(FLASH_READ_ADDR) else msg_id
                            add_log(f"Decode error id={msg_id:02X} addr=0x{addr_d:02X} "
                                    f"data={data.hex(' ')}: {dec_err}", "warn",
                                    terminal=True)
                        live.packets += 1
                        # Print summary to terminal every 100 packets
                        if live.packets % 100 == 0:
                            add_log(f"Packet #{live.packets} — "
                                    f"{live.speed_kmh:.1f}km/h  "
                                    f"Gear={live.gear}  "
                                    f"SoC={live.batt_soc}%  "
                                    f"{live.voltage:.1f}V  "
                                    f"{live.line_current:.1f}A", "info")
                        # Update histories every 3 packets for responsive graph
                        if live.packets % 3 == 0:
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
    d = asdict(settings)
    d['temp_sensor_names'] = TEMP_SENSOR_NAMES
    d['password_status_names'] = PASSWORD_STATUS_NAMES
    d['brake_config_names'] = BRAKE_CONFIG_NAMES
    return jsonify(d)

@app.route('/api/preflight')
def api_preflight():
    import time as _time
    checks = []
    age = _time.time() - live.timestamp if live.timestamp else 999

    def chk(name, status, detail, hint=""):
        checks.append({"name": name, "status": status,
                        "detail": detail, "hint": hint})

    # 1. Serial comms
    if not reader.connected and live.packets == 0:
        chk("Serial connection", "fail", "Not connected — no data received",
            "Select port and click Connect. Check TTL adapter is 3.3V not 5V.")
    elif not reader.connected and live.packets > 0 and age < 30:
        chk("Serial connection", "warn",
            f"Disconnected but {live.packets} packets were received "
            f"(last {age:.0f}s ago) — data shown is from previous session",
            "Reconnect to resume live data.")
    elif live.packets == 0:
        chk("Serial connection", "warn", "Connected but no packets received yet",
            "Check TX/RX not swapped (controller RXD → adapter TX).")
    elif age > 5:
        chk("Serial connection", "warn", f"Last packet {age:.0f}s ago — stream stopped",
            "Power cycle the controller and reconnect.")
    else:
        chk("Serial connection", "pass",
            f"Receiving data · {live.packets} packets · last {age:.1f}s ago")

    # 2. Battery voltage
    v = live.voltage
    if live.packets == 0:
        chk("Battery voltage", "unknown", "No data yet")
    elif v < 10:
        chk("Battery voltage", "fail", f"{v:.1f}V — controller not seeing battery",
            "Check main battery connector and inline fuse. "
            "Verify B+ and B- wires to controller are correct.")
    elif v < 52:
        chk("Battery voltage", "warn", f"{v:.1f}V — below expected 60V range",
            "Charge battery. Check for corroded battery terminal "
            "(known issue on this bike — clean with 600-grit sandpaper).")
    elif v > 68:
        chk("Battery voltage", "warn", f"{v:.1f}V — above expected 60V max",
            "Verify RatedVoltage setting in app matches your battery pack.")
    else:
        chk("Battery voltage", "pass", f"{v:.1f}V — within 52–68V range for 60V system")

    # 3. Hall sensors
    if live.packets == 0:
        chk("Hall sensors", "unknown", "No data yet")
    elif live.hall_error:
        chk("Hall sensors", "fail", "Hall error flag active (E96 equivalent)",
            "Check 5-wire Hall connector is fully seated. "
            "Measure each signal vs GND while spinning wheel slowly — "
            "all three should toggle 0V↔3.7V. "
            "100Ω to GND on one channel = shorted sensor "
            "(yellow was the faulty channel on this bike). "
            "Also verify centre sensor orientation matches the other two.")
    elif live.hall_pos_error:
        chk("Hall sensors", "fail", "Hall position error — sequence incorrect",
            "Swap any two Hall signal wires (yellow/blue/green) and retry. "
            "Or run self-learn with wheel off ground — it auto-detects sequence.")
    elif live.angle_learn == 0xAA:
        chk("Hall sensors", "pass", "Hall OK · self-learn completed (0xAA)")
    elif live.angle_learn == 0x55:
        chk("Hall sensors", "warn", "Self-learn in progress…",
            "Keep throttle at full until motor stabilises.")
    else:
        chk("Hall sensors", "warn",
            f"No errors but self-learn not yet run (angle_learn=0x{live.angle_learn:02X})",
            "Run self-learn from Commands tab with rear wheel off the ground.")

    # 4. Phase wires
    if live.packets == 0:
        chk("Phase wires (U/V/W)", "unknown", "No data yet")
    elif live.phase_lost:
        chk("Phase wires (U/V/W)", "fail", "Phase lost — open circuit on one or more phases",
            "Check all three phase bullet connectors are fully mated. "
            "Verify no wire pulled out of housing during installation.")
    elif live.modulation > 5 and live.phase_a_curr < 0.5 and live.phase_c_curr < 0.5:
        chk("Phase wires (U/V/W)", "warn",
            f"Controller outputting {live.modulation:.0f}% modulation but phase currents near zero",
            "Possible open-circuit phase wire or motor not connected. "
            "Check bullet connectors U/V/W.")
    else:
        chk("Phase wires (U/V/W)", "pass",
            f"No phase errors · Phase A={live.phase_a_curr:.1f}A · Phase C={live.phase_c_curr:.1f}A")

    # 5. Throttle
    if live.packets == 0:
        chk("Throttle", "unknown", "No data yet")
    elif live.throttle_error:
        chk("Throttle", "fail", "Throttle error flag active",
            "Check 3-wire throttle connector (+5V / GND / signal). "
            "Measure signal wire at idle: should be 0.8–1.1V. "
            "0V = signal not connected. 5V = supply and signal swapped.")
    else:
        chk("Throttle", "pass", "No throttle error")

    # 6. Brake signal
    if live.packets == 0:
        chk("Brake signal", "unknown", "No data yet")
    elif live.brake:
        chk("Brake signal", "warn", "Brake flag active — controller thinks brake is applied",
            "If lever is not pressed: brake wire may be pulled low (grounded). "
            "FarDriver default expects normally-open (floating = not braking, GND = braking). "
            "While active this prevents the motor from running.")
    else:
        chk("Brake signal", "pass", "Brake signal idle (not active)")

    # 7. Temperatures
    if live.packets == 0:
        chk("Temperatures", "unknown", "No data yet")
    elif live.motor_temp > 120:
        chk("Temperatures", "fail", f"Motor {live.motor_temp:.0f}°C — over protection threshold",
            "Let motor cool before running. If reading is implausible "
            "check TempSensor type in app matches motor thermistor type "
            "(white wire in Hall connector).")
    elif not (-30 < live.motor_temp < 100) or not (-10 < live.mos_temp < 90):
        chk("Temperatures", "warn",
            f"Motor {live.motor_temp:.0f}°C · MosFET {live.mos_temp:.0f}°C — unusual reading",
            "Check TempSensor type setting in app. "
            "Supported: NTC-10K, NTC-100K, PTC-1000, KTY84-130, KTY83-122.")
    else:
        chk("Temperatures", "pass",
            f"Motor {live.motor_temp:.0f}°C · MosFET {live.mos_temp:.0f}°C — plausible ambient values")

    # 8. Motor direction
    if live.packets == 0:
        chk("Motor direction", "unknown", "No data yet")
    else:
        chk("Motor direction", "pass" if not (live.reverse and not live.forward) else "warn",
            f"Forward={live.forward} · Reverse={live.reverse}",
            "If motor runs backwards under throttle: toggle Motor Reverse Direction "
            "in app Basic Parameters." if live.reverse and not live.forward else "")

    # 9. Self-learn
    if live.packets == 0:
        chk("Self-learn", "unknown", "No data yet")
    elif live.angle_learn == 0xAA:
        chk("Self-learn", "pass", "Completed — motor ready to run")
    elif live.angle_learn == 0x55:
        chk("Self-learn", "warn", "In progress — apply full throttle")
    else:
        chk("Self-learn", "warn", "Not yet run — motor may behave erratically",
            "Commands tab → Start Self-Learn. Rear wheel off ground. "
            "Apply full throttle when motor begins to spin.")

    # 10. Active faults
    errs = live.errors()
    if live.packets == 0:
        chk("Fault flags", "unknown", "No data yet")
    elif errs:
        chk("Fault flags", "fail", f"Active: {', '.join(errs)}",
            "Resolve all faults before running under load.")
    else:
        chk("Fault flags", "pass", "No active faults")

    # 11. Password / login status — per the official FarDriver manual
    # (§12.7 "Log in"): some controllers have an optional 30-digit password.
    # If set, parameters can be viewed but modifications require a prior
    # Login with the correct password first. This tool does not implement
    # that login exchange — if this controller reports password-protected,
    # that's a strong, documented explanation for why writes update RAM
    # immediately but don't survive a reset.
    if live.packets == 0 or settings.password_status == -1:
        chk("Password / login status", "unknown",
            "Not yet read (word 0xBC hasn't come through in the streaming cycle yet — give it a few seconds)")
    elif settings.password_status == 2:
        chk("Password / login status", "pass",
            "No password set — writes should not need a login step")
    elif settings.password_status in (0, 1):
        chk("Password / login status", "warn",
            f"Controller reports password-protected "
            f"({PASSWORD_STATUS_NAMES.get(settings.password_status, settings.password_status)})",
            "Per the FarDriver manual, modifying parameters requires clicking "
            "Login and entering the correct 30-digit password first — this "
            "tool doesn't implement that exchange. This is a strong candidate "
            "explanation for the write-persistence problem. Check whether a "
            "password was set (e.g. by a previous owner, dealer, or Super "
            "Soco's assembly) and whether it's recoverable via the product "
            "number (see manual §12.5) or a factory-recovery reset (§12.8).")
    else:
        chk("Password / login status", "unknown",
            f"Unrecognised value: {settings.password_status}")

    # 12. PassOk write gate — decompiled directly from the real FarDriver
    # Android app (MotorNet6.dll, GraphPage's live-data handler). PassOk is
    # NOT a separate handshake result -- it's parsed from the exact same
    # live telemetry bits we decode as `pass_ok` ((data[1]>>3)&0x3 in AddrE2),
    # straight off the periodic status broadcast. But the app checks it
    # before every single settings write:
    #   if PassOk == 0: refuse to send the write at all
    #   elif PassOk == 1 and BindingStat < 1: also refuse
    #   else (PassOk == 2 or 3, or BindingStat >= 1): send it
    # BindingStat is a separate app-side flag (not in telemetry, likely
    # "has this phone completed a bind flow with this controller before")
    # that we can't observe. This is the real app's own write-gating logic,
    # not a guess -- if pass_ok reads 0 or 1 here, the real app would ALSO
    # refuse to even attempt the write your controller is doing right now.
    if live.packets == 0:
        chk("PassOk write gate", "unknown", "No data yet")
    elif live.pass_ok in (2, 3):
        chk("PassOk write gate", "pass",
            f"pass_ok={live.pass_ok} — the real app would allow writes in this state")
    elif live.pass_ok == 1:
        chk("PassOk write gate", "warn",
            f"pass_ok=1 — the real app ONLY allows writes here if BindingStat >= 1 "
            "(a separate, app-side 'has this phone bound to this controller before' "
            "flag we can't observe from telemetry). Otherwise this blocks writes "
            "in the real app.",
            "This may be why writes update RAM but don't persist: the real app "
            "wouldn't even attempt the write in this state without a prior bind.")
    else:  # pass_ok == 0
        chk("PassOk write gate", "fail",
            f"pass_ok=0 — the real app refuses to send ANY settings write in this state",
            "This is the strongest lead so far for the persistence problem: if the "
            "controller is reporting pass_ok=0, the real app would never even "
            "attempt what we've been sending. Worth checking whether this value "
            "changes over time, after a bind attempt, or around the undervoltage "
            "lockout state.")

    n_fail = sum(1 for c in checks if c["status"] == "fail")
    n_warn = sum(1 for c in checks if c["status"] == "warn")
    n_pass = sum(1 for c in checks if c["status"] == "pass")
    overall = "fail" if n_fail > 0 else ("warn" if n_warn > 0 else "pass")

    return jsonify({
        "checks": checks,
        "summary": {"pass": n_pass, "warn": n_warn, "fail": n_fail},
        "overall": overall,
        "packets": live.packets,
    })


@app.route('/api/log')
def api_log():
    n = int(request.args.get('n', 1000))
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


@app.route('/api/log/raw')
def api_log_raw():
    """Plain text — also: curl http://localhost:5000/api/log/raw > fardriver.log"""
    lines = [f"[{e['ts']}] [{e['level'].upper():5s}] {e['msg']}"
             for e in log_entries]
    return Response('\n'.join(lines) + '\n', mimetype='text/plain',
                    headers={'Content-Disposition':
                             'attachment; filename=fardriver_log.txt'})


@app.route('/api/write', methods=['POST'])
def api_write():
    """
    Write a parameter and commit it to flash.
    Body: {param: 'rated_voltage', value: 60.0}

    Save is no longer optional/toggleable from the UI -- confirmed against a
    real captured Android-app session (2026-07-27) that syscmd 0x04 is the
    actual commit-to-flash trigger (not the 0x05 "reset" this tool used
    before), and that values written this way survive a real power cycle.
    0x04 also reboots the controller (confirmed via the BT module's own
    re-handshake showing up 3-8s afterward), so verification waits long
    enough to cover that reboot plus our own re-handshake and first data cycle.
    """
    if not reader.connected:
        return jsonify({'ok': False, 'error': 'not connected'}), 400
    d     = request.json or {}
    param = d.get('param', '')
    value = d.get('value')

    if value is None:
        return jsonify({'ok': False, 'error': 'value required'}), 400

    # ── Packed-word params (share word 0x0B) — read-modify-write ───────────
    if param in ('temp_sensor', 'direction', 'brake_config'):
        try:
            if param == 'temp_sensor':
                ival = int(value)
                if not (0 <= ival <= 7):
                    return jsonify({'ok': False, 'error': 'temp_sensor out of range [0-7]'}), 400
                pkt = write_word_0x0B(temp_sensor=ival)
                disp = str(ival)
            elif param == 'direction':
                ival = 1 if int(value) else 0
                pkt = write_word_0x0B(direction=ival)
                disp = 'Reverse' if ival else 'Forward'
            else:  # brake_config
                ival = int(value)
                if not (0 <= ival <= 4):
                    return jsonify({'ok': False, 'error': 'brake_config out of range [0-4]'}), 400
                pkt = write_word_0x0B(brake_config=ival)
                disp = BRAKE_CONFIG_NAMES.get(ival, str(ival))
        except RuntimeError as e:
            return jsonify({'ok': False, 'error': str(e)}), 400

        add_log(f"WRITE REQUEST (packed word 0x0B, RMW): {param} = {disp}  "
                f"pkt={pkt.hex(' ')}", "info", terminal=True)
        reader.send(pkt, f"WRITE {param}={disp} word_addr=0x0B (RMW)")
        time.sleep(0.3)

        add_log("Sending syscmd 0x04 to commit to flash…", "info", terminal=True)
        reader.send(CMD_COMMIT_FLASH, "COMMIT(0x04)")

        write_pending[param] = (ival, '', time.time())

        def _verify_packed():
            # 0x04 reboots the controller (confirmed 3-8s later in capture),
            # then our own reader has to notice and re-handshake, then wait
            # for a fresh data cycle -- give this plenty of margin.
            time.sleep(12.0)
            cur = getattr(settings, param, None)
            if cur is not None and int(cur) == ival:
                add_log(f"✓ WRITE CONFIRMED (post-reboot): {param}={cur}", "info", terminal=True)
            elif cur is not None:
                add_log(f"✗ WRITE NOT CONFIRMED: {param} reads {cur}, expected {ival}",
                        "warn", terminal=True)
            else:
                add_log(f"? VERIFY {param}: not in decoded settings", "warn", terminal=True)
            write_pending.pop(param, None)
        threading.Thread(target=_verify_packed, daemon=True).start()

        return jsonify({'ok': True, 'param': param, 'value': ival, 'addr': '0x0B'})

    # ── Standard single-word numeric params ─────────────────────────────────
    if param not in PARAM_MAP:
        return jsonify({'ok': False, 'error': f'unknown param: {param}'}), 400

    flash_addr, scale, unit, vmin, vmax = PARAM_MAP[param]
    fval = float(value)
    if not (vmin <= fval <= vmax):
        return jsonify({'ok': False,
                        'error': f'value {fval} out of range [{vmin}–{vmax}]'}), 400
    raw_val = int(round(fval * scale))

    pkt = write_param(flash_addr, raw_val)
    add_log(f"WRITE REQUEST: {param} = {fval}{unit}  "
            f"word_addr=0x{flash_addr:02X} raw={raw_val}  "
            f"pkt={pkt.hex(' ')}", "info", terminal=True)
    reader.send(pkt, f"WRITE {param}={fval}{unit} "
                     f"word_addr=0x{flash_addr:02X} raw=0x{raw_val:04X}")

    # Wait for controller to process write into RAM
    time.sleep(0.3)

    add_log("Sending syscmd 0x04 to commit to flash…", "info", terminal=True)
    reader.send(CMD_COMMIT_FLASH, "COMMIT(0x04)")

    # Record for stream verification
    write_pending[param] = (fval, unit, time.time())

    def _verify():
        # Same reboot+reconnect margin as the packed-word path above.
        time.sleep(12.0)
        cur = getattr(settings, param, None)
        if cur is not None:
            match = abs(float(cur) - fval) < 0.15
            if match:
                add_log(f"✓ WRITE CONFIRMED (post-reboot): {param}={cur}{unit} "
                        f"matches expected {fval}{unit}",
                        "info", terminal=True)
            else:
                add_log(f"✗ WRITE NOT CONFIRMED: {param} reads {cur}{unit}, "
                        f"expected {fval}{unit}  "
                        f"→ flash_addr=0x{flash_addr:02X} may be wrong",
                        "warn", terminal=True)
        else:
            add_log(f"? VERIFY {param}: not in decoded settings",
                    "warn", terminal=True)
        write_pending.pop(param, None)
    threading.Thread(target=_verify, daemon=True).start()

    return jsonify({'ok': True, 'param': param, 'value': fval,
                    'raw': raw_val, 'addr': f'0x{flash_addr:02X}'})

@app.route('/api/save', methods=['POST'])
def api_save():
    """
    Commit any pending RAM changes to flash. Confirmed against a real
    captured Android-app session (2026-07-27): syscmd 0x04 is the actual
    save trigger. Reboots the controller (3-8s later, per that capture).
    """
    if not reader.connected:
        return jsonify({'ok': False, 'error': 'not connected'}), 400
    reader.send(CMD_COMMIT_FLASH, "COMMIT(0x04)")
    return jsonify({'ok': True})

@app.route('/api/reset', methods=['POST'])
def api_reset():
    """
    Plain reboot, WITHOUT committing pending changes first (confirmed:
    this is what 0x05 actually does — use /api/save if you want to persist
    anything before restarting).
    """
    if not reader.connected:
        return jsonify({'ok': False, 'error': 'not connected'}), 400
    reader.send(CMD_RESET_SOFT, "RESET(0x05)")
    return jsonify({'ok': True})

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

  /* ── edit settings ── */
  .edit-section { background:var(--panel); border:1px solid var(--border);
    border-radius:10px; padding:14px 16px; margin-bottom:10px; }
  .edit-section h3 { font-size:0.8rem; color:var(--amber);
    text-transform:uppercase; letter-spacing:.08em; margin-bottom:10px; }
  .edit-row { display:flex; align-items:center; gap:10px; padding:6px 0;
    border-bottom:1px solid var(--border); }
  .edit-row:last-child { border-bottom:none; }
  .edit-label { flex:1; font-size:0.82rem; color:var(--muted); }
  .edit-current { font-family:var(--mono); font-size:0.85rem;
    color:var(--white); min-width:60px; text-align:right; }
  .edit-input { background:var(--border); color:var(--white); border:none;
    border-radius:4px; padding:4px 8px; width:80px;
    font-family:var(--mono); font-size:0.85rem; }
  .edit-unit { font-size:0.75rem; color:var(--muted); width:32px; }
  .edit-save { background:var(--amber); color:#111; border:none;
    border-radius:4px; padding:4px 12px; font-size:0.8rem;
    font-weight:600; cursor:pointer; }
  .edit-save:hover { opacity:.85; }
  .edit-msg { font-size:0.75rem; margin-left:6px; }
  .edit-warn { background:#2B1E00; border:1px solid var(--amber);
    border-radius:6px; padding:8px 12px; font-size:0.8rem;
    color:var(--amber); margin-bottom:12px; }
  /* ── preflight ── */
  .pf-check {
    background:var(--panel); border:1px solid var(--border); border-radius:8px;
    padding:12px 16px; display:grid;
    grid-template-columns:22px 1fr auto; gap:4px 12px; align-items:start;
  }
  .pf-icon  { font-size:1rem; padding-top:2px; }
  .pf-name  { font-weight:600; font-size:0.88rem; color:var(--white); }
  .pf-detail{ font-size:0.8rem; color:var(--muted); grid-column:2; }
  .pf-hint  {
    font-size:0.77rem; color:var(--amber); grid-column:2;
    margin-top:4px; padding:6px 10px;
    background:#23200A; border-left:3px solid var(--amber); border-radius:4px;
    line-height:1.5;
  }
  .pf-badge {
    font-size:0.7rem; font-weight:700; padding:3px 10px;
    border-radius:12px; white-space:nowrap; align-self:start; margin-top:2px;
  }
  .pf-pass    { background:#0D2B1A; color:var(--green); border:1px solid var(--green); }
  .pf-fail    { background:#2B0D0D; color:var(--red);   border:1px solid var(--red); }
  .pf-warn    { background:#2B1E00; color:var(--amber); border:1px solid var(--amber); }
  .pf-unknown { background:var(--border); color:var(--muted); border:1px solid var(--border); }
  /* ── log ── */
  #log-box {
    background: var(--panel); border: 1px solid var(--border); border-radius: 10px;
    padding: 12px; height: 560px; overflow-y: auto;
    font-family: var(--mono); font-size: 0.78rem; line-height: 1.6;
    user-select: text; -webkit-user-select: text; cursor: text;
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
  <div class="tab" onclick="showTab('preflight')">⚡ Pre-flight</div>
  <div class="tab" onclick="showTab('settings')">Settings</div>
  <div class="tab" onclick="showTab('edit')">✏ Edit</div>
  <div class="tab" onclick="showTab('commands')">Commands</div>
  <div class="tab" onclick="showTab('log')">Log</div>
</div>

<!-- ══════════════════════ DASHBOARD ══════════════════════ -->
<div id="tab-dashboard" class="tab-content active">
  <div class="grid grid-4" style="margin-bottom:12px">
    <div class="card" style="grid-column:span 2">
      <div class="card-label">Speed</div>
      <div class="card-value big" id="d-speed">0.0</div>
      <div class="card-sub">km/h (raw=<span id="d-speed-raw">0</span>, ×0.0109 — provisional) &nbsp;|&nbsp; <span id="d-gear">GEAR —</span> &nbsp; <span id="d-dir">·</span></div>
    </div>
    <div class="card">
      <div class="card-label">Battery SoC <span style="font-size:0.65rem;color:var(--muted)">(voltage-based estimate)</span></div>
      <div class="card-value green" id="d-soc">— %</div>
      <div class="bar-track"><div class="bar-fill green" id="d-soc-bar" style="width:0%"></div></div>
      <div class="card-sub" style="margin-top:4px">firmware raw byte: <span id="d-soc-raw">—</span>%</div>
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

<!-- ══════════════════════ PRE-FLIGHT ══════════════════════ -->
<div id="tab-preflight" class="tab-content">
  <div style="display:flex;align-items:center;gap:12px;margin-bottom:14px;flex-wrap:wrap">
    <h2 style="font-size:1rem;color:var(--white)">Wiring &amp; Health Check</h2>
    <button class="btn-amber" onclick="runPreflight()">⟳ Run checks</button>
    <span id="pf-summary" style="font-size:0.85rem;color:var(--muted)">Click to run — connect first</span>
  </div>
  <div id="pf-checks" style="display:flex;flex-direction:column;gap:8px"></div>
</div>

<!-- ══════════════════════ SETTINGS ══════════════════════ -->
<div id="tab-settings" class="tab-content">
  <div class="settings-sections" id="settings-container"></div>
</div>

<!-- ══════════════════════ EDIT SETTINGS ══════════════════════ -->
<div id="tab-edit" class="tab-content">
  <div class="edit-warn">
    ⚠ Changes take effect after Save. The controller will restart (confirmed:
    the commit-to-flash command reboots it, same as the old reset did, but
    now it actually saves first). Always verify values before saving. Wrong
    settings can damage the motor or controller.
  </div>

  <div class="edit-section">
    <h3>Motor / Voltage</h3>
    <div class="edit-row">
      <span class="edit-label">Rated Voltage</span>
      <span class="edit-current" id="ev-rated_voltage">—</span>
      <input class="edit-input" id="ei-rated_voltage" type="number" step="1" min="48" max="75" placeholder="60">
      <span class="edit-unit">V</span>
      <button class="edit-save" onclick="writeParam('rated_voltage')">Save</button>
      <span class="edit-msg" id="em-rated_voltage"></span>
    </div>
    <div class="edit-row">
      <span class="edit-label">Low Voltage Cutoff</span>
      <span class="edit-current" id="ev-low_vol_protect">—</span>
      <input class="edit-input" id="ei-low_vol_protect" type="number" step="0.5" min="40" max="70" placeholder="52">
      <span class="edit-unit">V</span>
      <button class="edit-save" onclick="writeParam('low_vol_protect')">Save</button>
      <span class="edit-msg" id="em-low_vol_protect"></span>
    </div>
    <div class="edit-row">
      <span class="edit-label">Rated Power</span>
      <span class="edit-current" id="ev-rated_power">—</span>
      <input class="edit-input" id="ei-rated_power" type="number" step="100" min="500" max="10000" placeholder="3000">
      <span class="edit-unit">W</span>
      <button class="edit-save" onclick="writeParam('rated_power')">Save</button>
      <span class="edit-msg" id="em-rated_power"></span>
    </div>
  </div>

  <div class="edit-section">
    <h3>Regen Braking</h3>
    <div class="edit-row">
      <span class="edit-label">Stop Back Current <span style="font-size:0.7rem">(regen at low speed)</span></span>
      <span class="edit-current" id="ev-stop_back_curr">—</span>
      <input class="edit-input" id="ei-stop_back_curr" type="number" step="1" min="0" max="60" placeholder="5">
      <span class="edit-unit">A</span>
      <button class="edit-save" onclick="writeParam('stop_back_curr')">Save</button>
      <span class="edit-msg" id="em-stop_back_curr"></span>
    </div>
    <div class="edit-row">
      <span class="edit-label">Max Back Current <span style="font-size:0.7rem">(peak regen)</span></span>
      <span class="edit-current" id="ev-max_back_curr">—</span>
      <input class="edit-input" id="ei-max_back_curr" type="number" step="1" min="0" max="80" placeholder="15">
      <span class="edit-unit">A</span>
      <button class="edit-save" onclick="writeParam('max_back_curr')">Save</button>
      <span class="edit-msg" id="em-max_back_curr"></span>
    </div>
  </div>

  <div class="edit-section">
    <h3>Temperature Protection</h3>
    <div class="edit-row">
      <span class="edit-label">Motor Temp Protect</span>
      <span class="edit-current" id="ev-motor_temp_protect">—</span>
      <input class="edit-input" id="ei-motor_temp_protect" type="number" step="5" min="60" max="160" placeholder="100">
      <span class="edit-unit">°C</span>
      <button class="edit-save" onclick="writeParam('motor_temp_protect')">Save</button>
      <span class="edit-msg" id="em-motor_temp_protect"></span>
    </div>
    <div class="edit-row">
      <span class="edit-label">Motor Temp Restore</span>
      <span class="edit-current" id="ev-motor_temp_restore">—</span>
      <input class="edit-input" id="ei-motor_temp_restore" type="number" step="5" min="50" max="150" placeholder="80">
      <span class="edit-unit">°C</span>
      <button class="edit-save" onclick="writeParam('motor_temp_restore')">Save</button>
      <span class="edit-msg" id="em-motor_temp_restore"></span>
    </div>
    <div class="edit-row">
      <span class="edit-label">MosFET Temp Protect</span>
      <span class="edit-current" id="ev-mos_temp_protect">—</span>
      <input class="edit-input" id="ei-mos_temp_protect" type="number" step="5" min="60" max="120" placeholder="85">
      <span class="edit-unit">°C</span>
      <button class="edit-save" onclick="writeParam('mos_temp_protect')">Save</button>
      <span class="edit-msg" id="em-mos_temp_protect"></span>
    </div>
  </div>

  <div class="edit-section">
    <h3>Temperature Sensor Type</h3>
    <p style="font-size:0.81rem;color:var(--muted);margin-bottom:10px;line-height:1.5">
      Must match the thermistor in your motor's Hall connector (white wire).
      Wrong type causes implausible temperature readings and may trigger false thermal protection.
      <br>For the Bosch hub motor on the Super Soco TC: try <strong>NTC-10K</strong> first.
    </p>
    <div class="edit-row">
      <span class="edit-label">Sensor Type</span>
      <span class="edit-current" id="ev-temp_sensor">—</span>
      <select class="edit-input" id="ei-temp_sensor" style="width:140px" onchange="this.dataset.touched='1'">
        <option value="0">0 — None</option>
        <option value="1">1 — PTC-1000</option>
        <option value="2">2 — NTC-230K</option>
        <option value="3">3 — KTY84-130</option>
        <option value="4">4 — Hypothetical</option>
        <option value="5">5 — KTY83-122</option>
        <option value="6">6 — NTC-10K</option>
        <option value="7">7 — NTC-100K</option>
      </select>
      <span class="edit-unit"></span>
      <button class="edit-save" onclick="writeTempSensor()">Save</button>
      <span class="edit-msg" id="em-temp_sensor"></span>
    </div>
  </div>

  <div class="edit-section">
    <h3>Brake Signal Mode</h3>
    <p style="font-size:0.81rem;color:var(--muted);margin-bottom:10px;line-height:1.5">
      Confirmed by decompiling the real app (<code>ProControlPage::BrakeConfig_SelectedIndexChanged</code>) —
      same packed word 0x0B as Temperature Sensor above, bits 0-3. Labels are
      the app's own English strings, not a guess ("P+" isn't spelled out
      anywhere, exact meaning not confirmed).
      <br><br>
      <strong>Worth checking your wiring before changing this:</strong> this
      appears to be the <em>only</em> brake-polarity setting on this
      controller — nothing suggests there's a second, independent one. If
      your handbrake and parking-button/kickstand signal are combined onto
      the same physical input with opposite electrical behavior (one
      high-side, one low-side), no single setting here can be correct for
      both at once — that combination needs fixing in the wiring itself
      (e.g. a small inverter/relay on one of the two), not just software.
    </p>
    <div class="edit-row">
      <span class="edit-label">Brake Mode</span>
      <span class="edit-current" id="ev-brake_config">—</span>
      <select class="edit-input" id="ei-brake_config" style="width:280px" onchange="this.dataset.touched='1'">
        <option value="0">0 — Stop Valid</option>
        <option value="1">1 — Inverse Stop</option>
        <option value="2">2 — P+Stop</option>
        <option value="3">3 — P+Inverse Stop</option>
        <option value="4">4 — Disabled</option>
      </select>
      <span class="edit-unit"></span>
      <button class="edit-save" onclick="writeBrakeConfig()">Save</button>
      <span class="edit-msg" id="em-brake_config"></span>
    </div>
  </div>

  <div class="edit-section">
    <h3>Motor Direction</h3>
    <p style="font-size:0.81rem;color:var(--muted);margin-bottom:10px;line-height:1.5">
      Flips forward/reverse sense electrically — no phase-wire swap needed.
      Shares packed word 0x0B with Temperature Sensor Type above; this tool
      reads-modifies-writes so setting one doesn't disturb the other.
    </p>
    <div class="edit-row">
      <span class="edit-label">Direction</span>
      <span class="edit-current" id="ev-direction">—</span>
      <select class="edit-input" id="ei-direction" style="width:140px" onchange="this.dataset.touched='1'">
        <option value="0">0 — Forward</option>
        <option value="1">1 — Reverse</option>
      </select>
      <span class="edit-unit"></span>
      <button class="edit-save" onclick="writeDirection()">Save</button>
      <span class="edit-msg" id="em-direction"></span>
    </div>
  </div>

  <div class="edit-section">
    <h3>Save &amp; Reset</h3>
    <p style="font-size:0.82rem;color:var(--muted);margin-bottom:10px">
      Individual Save buttons above already commit each parameter (syscmd
      0x04 — confirmed against a real captured app session). Use these only
      if you need to force a commit/reset separately.
    </p>
    <div style="display:flex;gap:10px;flex-wrap:wrap">
      <button class="edit-save" onclick="saveAll()">💾 Commit to Flash (0x04)</button>
      <button class="edit-save" style="background:var(--border)"
              onclick="resetCtrl()">⟳ Reset only, no save (0x05)</button>
      <span id="em-saveall" class="edit-msg"></span>
    </div>
  </div>
</div>

<!-- ══════════════════════ COMMANDS ══════════════════════ -->
<div id="tab-commands" class="tab-content">
  <div class="cmd-grid">
    <div class="cmd-card">
      <h3>Self-Learn</h3>
      <p>Calibrates Hall sensor phase angles and pole pairs.<br>
         <strong>Rear wheel must be off the ground.</strong><br>
         Apply full throttle when the motor starts spinning.</p>
      <button class="btn-amber" onclick="startSelfLearnGuided()">▶ Start Self-Learn</button>
      <p style="font-size:0.78rem;color:var(--muted);margin-top:8px;line-height:1.5">
        After the motor completes the spin→adjust→reverse→stop sequence,
        click below to commit the result. Self-learn was likely never
        actually persisting before — the underlying calibration may commit
        on its own, but the "learned" status flag is an ordinary word in
        flash like everything else, so it plausibly needs the same syscmd
        0x04 commit as any other setting.
      </p>
      <button class="edit-save" style="margin-top:6px" onclick="commitSelfLearn()">💾 Commit Self-Learn Result (0x04)</button>
      <span id="em-selflearn-commit" class="edit-msg"></span>
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
    <div class="cmd-card" style="grid-column:1/-1">
      <div class="card-label" style="margin-bottom:6px">
        Live log &nbsp;·&nbsp; Packets: <span id="d-packets">0</span>
      </div>
      <div id="mini-log" style="background:var(--bg);border:1px solid var(--border);
        border-radius:6px;padding:8px;font-family:var(--mono);font-size:0.74rem;
        line-height:1.7;height:140px;overflow-y:auto">
        <span style="color:var(--muted)">Connect to see live log…</span>
      </div>
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
    <button class="btn-muted" onclick="selectAllLog()">Select All</button>
    <button class="btn-muted" onclick="downloadLog()">⬇ Download</button>
    <span style="font-size:0.78rem;color:var(--muted)" id="log-count"></span>
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
  document.getElementById('d-speed-raw').textContent = d.raw_speed_value;
  const gears = ['P','Low','Mid','High'];
  document.getElementById('d-gear').textContent = 'GEAR ' + (gears[d.gear] || d.gear);
  document.getElementById('d-dir').textContent =
    d.reverse ? '◀ REV' : d.forward ? '▶ FWD' : '·';

  // SoC — voltage-based estimate (same formula the real app uses) is primary;
  // the raw firmware byte is shown alongside for reference/comparison
  const soc = d.batt_soc_calc;
  const socEl = document.getElementById('d-soc');
  socEl.textContent = soc.toFixed(0) + ' %';
  socEl.className = 'card-value ' + (soc < 20 ? 'red' : 'green');
  const bar = document.getElementById('d-soc-bar');
  bar.style.width = soc + '%';
  bar.className = 'bar-fill ' + (soc < 20 ? 'red' : 'green');
  document.getElementById('d-soc-raw').textContent = d.batt_soc;

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
  const r = await fetch('/api/log?n=1000');
  allLogEntries = await r.json();
  filterLog();
  // Mini-log: last 6 entries shown on all tabs
  const mini = document.getElementById('mini-log');
  if (mini) {
    const cols = {info:'var(--white)',warn:'var(--amber)',
                  error:'var(--red)',tx:'var(--amber)',rx:'var(--green)'};
    const recent = allLogEntries.slice(-6);
    mini.innerHTML = recent.length
      ? recent.map(e =>
          `<div style="color:${cols[e.level]||'#fff'}">` +
          `<span style="color:var(--muted)">[${e.ts}]</span> ${escHtml(e.msg)}</div>`
        ).join('')
      : '<span style="color:var(--muted)">Waiting…</span>';
    mini.scrollTop = mini.scrollHeight;
  }
}

function filterLog() {
  const filter  = document.getElementById('log-filter').value.toLowerCase();
  const showRx  = document.getElementById('log-rx').checked;
  const showTx  = document.getElementById('log-tx').checked;
  const box = document.getElementById('log-box');
  const cnt = document.getElementById('log-count');
  if(cnt) cnt.textContent = `${allLogEntries.length} entries`;
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
  const c = document.getElementById('log-count');
  if(c) c.textContent = '';
}

function selectAllLog() {
  const box = document.getElementById('log-box');
  const range = document.createRange();
  range.selectNodeContents(box);
  const sel = window.getSelection();
  sel.removeAllRanges();
  sel.addRange(range);
}

function downloadLog() {
  const lines = allLogEntries.map(e =>
    `[${e.ts}] [${e.level.toUpperCase().padEnd(5)}] ${e.msg}`);
  const blob = new Blob([lines.join('\n')], {type:'text/plain'});
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  const ts = new Date().toISOString().slice(0,19).replace(/:/g,'-');
  a.download = `fardriver_log_${ts}.txt`;
  a.click();
}

function escHtml(s) {
  return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}

// Auto-reload log every 500ms always — critical during BT handshake
setInterval(loadLog, 500);

// ── Edit Settings ────────────────────────────────────────────────────────────
const CURRENT_VALUES = {
  rated_voltage: null, low_vol_protect: null, rated_power: null,
  stop_back_curr: null, max_back_curr: null,
  motor_temp_protect: null, motor_temp_restore: null, mos_temp_protect: null,
};

async function updateEditCurrentValues() {
  const r = await fetch('/api/settings');
  const s = await r.json();
  const map = {
    rated_voltage: s.rated_voltage, low_vol_protect: s.low_vol_protect,
    rated_power: s.rated_power, stop_back_curr: s.stop_back_curr,
    max_back_curr: s.max_back_curr, motor_temp_protect: s.motor_temp_protect,
    motor_temp_restore: s.motor_temp_restore, mos_temp_protect: s.mos_temp_protect,
  };
  // Update temp sensor dropdown separately — only pre-fill if the user
  // hasn't touched it, so a live poll doesn't silently overwrite an
  // in-progress selection with the (possibly stale/reverted) live reading.
  // (This is what caused the "Try TempSensor via Send(...)" preset button
  // to resend whatever the controller currently read, instead of the
  // value you'd actually picked — the dropdown kept snapping back.)
  const tsEl = document.getElementById('ev-temp_sensor');
  const tsSel = document.getElementById('ei-temp_sensor');
  if (tsEl && s.temp_sensor_names) {
    const idx = s.temp_sensor !== undefined ? s.temp_sensor : 0;
    tsEl.textContent = `${idx} — ${s.temp_sensor_names[idx] || '?'}`;
    if (tsSel && !tsSel.dataset.touched) tsSel.value = String(idx);
  }
  // Update direction dropdown separately — same "don't clobber" rule
  const dirEl = document.getElementById('ev-direction');
  const dirSel = document.getElementById('ei-direction');
  if (dirEl && s.direction !== undefined) {
    dirEl.textContent = s.direction ? '1 — Reverse' : '0 — Forward';
    if (dirSel && !dirSel.dataset.touched) dirSel.value = String(s.direction);
  }
  // Update brake_config dropdown separately — same "don't clobber" rule
  const bcEl = document.getElementById('ev-brake_config');
  const bcSel = document.getElementById('ei-brake_config');
  if (bcEl && s.brake_config_names) {
    const bidx = s.brake_config !== undefined ? s.brake_config : 0;
    bcEl.textContent = s.brake_config_names[bidx] || String(bidx);
    if (bcSel && !bcSel.dataset.touched) bcSel.value = String(bidx);
  }
  for (const [k, v] of Object.entries(map)) {
    const el = document.getElementById('ev-' + k);
    if (el && v !== undefined) {
      el.textContent = typeof v === 'number'
        ? (Number.isInteger(v) ? v : v.toFixed(1)) : v;
      CURRENT_VALUES[k] = v;
      // Pre-fill input with current value if empty
      const inp = document.getElementById('ei-' + k);
      if (inp && !inp.value) inp.value = el.textContent;
    }
  }
}

async function writeParam(param) {
  const inp = document.getElementById('ei-' + param);
  const msg = document.getElementById('em-' + param);
  const val = parseFloat(inp.value);
  if (isNaN(val)) { msg.textContent = '✗ invalid'; msg.style.color='var(--red)'; return; }
  msg.textContent = '…sending'; msg.style.color='var(--muted)';
  try {
    const r = await fetch('/api/write', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify({param, value: val})
    });
    const d = await r.json();
    if (d.ok) {
      msg.textContent = `✓ saved (raw=${d.raw}) — controller will restart, check back in ~15s`;
      msg.style.color = 'var(--green)';
      setTimeout(() => updateEditCurrentValues(), 13000);
    } else {
      msg.textContent = '✗ ' + d.error;
      msg.style.color = 'var(--red)';
    }
  } catch(e) {
    msg.textContent = '✗ ' + e; msg.style.color='var(--red)';
  }
}

async function writeTempSensor() {
  const sel = document.getElementById('ei-temp_sensor');
  const msg = document.getElementById('em-temp_sensor');
  const val = parseInt(sel.value);
  msg.textContent = '…sending'; msg.style.color='var(--muted)';
  try {
    const r = await fetch('/api/write', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify({param: 'temp_sensor', value: val})
    });
    const d = await r.json();
    if (d.ok) {
      msg.textContent = `✓ sent ${val} — controller will restart, check back in ~15s`;
      msg.style.color = 'var(--green)';
      setTimeout(updateEditCurrentValues, 13000);
    } else {
      msg.textContent = '✗ ' + d.error;
      msg.style.color = 'var(--red)';
    }
  } catch(e) {
    msg.textContent = '✗ ' + e; msg.style.color='var(--red)';
  }
}

async function writeDirection() {
  const sel = document.getElementById('ei-direction');
  const msg = document.getElementById('em-direction');
  const val = parseInt(sel.value);
  msg.textContent = '…sending'; msg.style.color='var(--muted)';
  try {
    const r = await fetch('/api/write', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify({param: 'direction', value: val})
    });
    const d = await r.json();
    if (d.ok) {
      const label = val === 1 ? 'Reverse' : 'Forward';
      msg.textContent = `✓ sent ${label} — controller will restart, check back in ~15s`;
      msg.style.color = 'var(--green)';
      setTimeout(updateEditCurrentValues, 13000);
    } else {
      msg.textContent = '✗ ' + d.error;
      msg.style.color = 'var(--red)';
    }
  } catch(e) {
    msg.textContent = '✗ ' + e; msg.style.color='var(--red)';
  }
}

async function writeBrakeConfig() {
  const sel = document.getElementById('ei-brake_config');
  const msg = document.getElementById('em-brake_config');
  const val = parseInt(sel.value);
  msg.textContent = '…sending'; msg.style.color='var(--muted)';
  try {
    const r = await fetch('/api/write', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify({param: 'brake_config', value: val})
    });
    const d = await r.json();
    if (d.ok) {
      msg.textContent = `✓ sent mode ${val} — controller will restart, check back in ~15s`;
      msg.style.color = 'var(--green)';
      setTimeout(updateEditCurrentValues, 13000);
    } else {
      msg.textContent = '✗ ' + d.error;
      msg.style.color = 'var(--red)';
    }
  } catch(e) {
    msg.textContent = '✗ ' + e; msg.style.color='var(--red)';
  }
}

async function saveAll() {
  const msg = document.getElementById('em-saveall');
  msg.textContent = '…saving'; msg.style.color='var(--muted)';
  await fetch('/api/save', {method:'POST'});
  msg.textContent = '✓ saved to flash'; msg.style.color='var(--green)';
}

async function resetCtrl() {
  if (!confirm('Soft reset the controller? It will restart.')) return;
  await fetch('/api/reset', {method:'POST'});
}

// ── Self-learn sequence guide ─────────────────────────────────────────────────
// Extended self-learn command handler with step-by-step guidance
const origSelfLearn = typeof sendCmd !== 'undefined' ? sendCmd : null;

async function commitSelfLearn() {
  const msg = document.getElementById('em-selflearn-commit');
  msg.textContent = '…committing'; msg.style.color='var(--muted)';
  try {
    const r = await fetch('/api/save', {method:'POST'});
    const d = await r.json();
    if (d.ok) {
      msg.textContent = '✓ sent — controller will restart, check Auto-Learn status in ~15s';
      msg.style.color = 'var(--green)';
    } else {
      msg.textContent = '✗ ' + d.error;
      msg.style.color = 'var(--red)';
    }
  } catch(e) {
    msg.textContent = '✗ ' + e; msg.style.color='var(--red)';
  }
}

async function startSelfLearnGuided() {
  const steps = [
    '1. Lift the rear wheel completely off the ground.',
    '2. Ensure throttle is at zero (released).',
    '3. Click OK to send the self-learn command.',
    '4. When the motor starts spinning — apply FULL throttle and hold.',
    '5. The motor will: spin up → adjust phase angle → reverse → stop.',
    '6. Release throttle when motor stops. Self-learn complete.',
    '',
    'If motor does NOT spin after command:',
    '  • Swap blue and green phase wires (U↔V or V↔W)',
    '  • Retry self-learn',
    '',
    'Total time: ~10-30 seconds.'
  ];
  if (!confirm('SELF-LEARN SEQUENCE\n\n' + steps.join('\n') + '\n\nReady to proceed?')) return;
  const r = await fetch('/api/cmd/self_learn', {method:'POST'});
  const d = await r.json();
  if (d.ok) {
    alert('Self-learn command sent!\n\nNow: apply FULL throttle when the motor begins to spin.\n\nWhen it stops (spin→adjust→reverse→stop complete), click "💾 Commit Self-Learn Result (0x04)" below — this is likely the missing step that was keeping Auto-Learn status stuck on "Learning" before.');
  } else {
    alert('Error: ' + (d.error || 'unknown'));
  }
}

// ── Preflight ────────────────────────────────────────────────────────────────
async function runPreflight() {
  document.getElementById('pf-summary').textContent = 'Running checks…';
  document.getElementById('pf-checks').innerHTML = '';
  try {
    const r = await fetch('/api/preflight');
    const d = await r.json();
    const box = document.getElementById('pf-checks');
    const icons  = {pass:'✓', fail:'✗', warn:'⚠', unknown:'·'};
    const labels = {pass:'PASS', fail:'FAIL', warn:'WARN', unknown:'—'};
    d.checks.forEach(c => {
      const el = document.createElement('div');
      el.className = 'pf-check';
      el.innerHTML =
        `<span class="pf-icon">${icons[c.status]}</span>` +
        `<span class="pf-name">${escHtml(c.name)}</span>` +
        `<span class="pf-badge pf-${c.status}">${labels[c.status]}</span>` +
        `<span class="pf-detail">${escHtml(c.detail)}</span>` +
        (c.hint ? `<span class="pf-hint">💡 ${escHtml(c.hint)}</span>` : '');
      box.appendChild(el);
    });
    const s = d.summary;
    const col = d.overall==='pass'?'var(--green)':d.overall==='warn'?'var(--amber)':'var(--red)';
    const msg = d.overall==='pass' ? '✓ All checks passed' :
      d.overall==='warn' ? `⚠ ${s.warn} warning${s.warn!==1?'s':''}, ${s.pass} passed` :
      `✗ ${s.fail} failed, ${s.warn} warnings, ${s.pass} passed`;
    document.getElementById('pf-summary').innerHTML =
      `<span style="color:${col};font-weight:700">${msg}</span>` +
      ` &nbsp;·&nbsp; ${d.packets} packets`;
  } catch(e) {
    document.getElementById('pf-summary').textContent = 'Error: ' + e;
  }
}

// ── Tabs ──────────────────────────────────────────────────────────────────────
function showTab(name) {
  document.querySelectorAll('.tab-content').forEach(el => el.classList.remove('active'));
  document.querySelectorAll('.tab').forEach(el => el.classList.remove('active'));
  document.getElementById('tab-' + name).classList.add('active');
  event.target.classList.add('active');
  if (name === 'log') loadLog();
  if (name === 'settings') updateSettings();
  if (name === 'preflight') runPreflight();
  if (name === 'edit') updateEditCurrentValues();
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
    ap.add_argument('--baud',  type=int, default=19200, help='Baud rate (default 19200 — confirmed working on the bench; overrides the older 9600 default, see context.md)')
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