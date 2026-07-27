#!/usr/bin/env python3
"""
fardriver_silent_monitor.py
============================
PASSIVE, READ-ONLY serial monitor for the wire between the FarDriver's real
BT module and the controller (or the raw TTL header, tapped either
direction). Sends NOTHING to the port, ever. Purpose: capture and decode
whatever the REAL app actually sends, without our own tool's BT-handshake
emulation or writes interfering with that session in any way.

*** THIS SCRIPT NEVER WRITES TO THE SERIAL PORT ***
grep -n "write(" this file: the only write() calls are to the log file
and stdout. The serial object is opened and only ever .read() from.

Usage
-----
  pip install pyserial
  python3 fardriver_silent_monitor.py --port /dev/tty.usbserial-XXXX [--baud 19200] [--label RX-from-fardriver] [--log FILE]

Run it once tapping the controller's TX line (what the controller sends —
its periodic broadcast + any responses) and once tapping the BT module's
TX line (what the phone app actually sends — the thing we've never
directly seen). Use --label to remember which pass is which when you send
the resulting log file back.

Output format
--------------
  [HH:MM:SS.mmm] [FRAME ] <decoded interpretation>   | raw: AA ..  ..
Frame types: READ (16-byte periodic block), WRITE (8-byte word write),
SYSCMD (8-byte system command), SENDCMD (8-byte old-style command),
TEXT (AT+... handshake / printable ASCII), ??? (unrecognized, resyncing).

Decoding for READ frames reuses the exact CRC tables and field maps already
verified in fardriver_web.py this session — same bit positions for
temp_sensor/direction/phase_exchange (word 0x0B), low_vol_protect (word
0x1F), password_status (word 0xBC), and the E2 live-status bits.
"""

import argparse
import sys
import time
from datetime import datetime

try:
    import serial
except ImportError:
    print("Missing dependency. Run: pip install pyserial --break-system-packages")
    sys.exit(1)

# ─────────────────────────────────────────────────────────────────────────────
# CRC (two-table, verbatim from fardriver_web.py — verified against known-good
# factory-reset packet this session)
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

def _crc(data):
    a, b = 0x3C, 0x7F
    for byte in data:
        i = a ^ byte
        a = b ^ _CRC_HI[i]
        b = _CRC_LO[i]
    return a, b

# (command, sub_command) -> name, for the old-style Send(cat,idx) protocol
SEND_CMD_NAMES = {
    (0x11, 0x01): "temp_sensor (ETempSensor, 0-7)",
    (0x12, 0x07): "direction (0=Forward/1=Reverse)",
    (0x12, 0x01): "pole_pairs",
    (0x12, 0x02): "max_speed",
    (0x12, 0x03): "rated_power",
    (0x12, 0x04): "rated_voltage x10",
    (0x12, 0x05): "rated_speed",
    (0x12, 0x18): "stop_back_curr",
    (0x12, 0x19): "max_back_curr",
    (0x12, 0x1A): "max_phase_curr x4",
    (0x12, 0x1B): "max_line_curr x4",
    (0x05, 0x01): "(get CAN params? — sent after date/time write per community README)",
}

SYSCMD_NAMES = {
    0x01: "non-following status",
    0x02: "self-learn / balance start",
    0x03: "stop balancing",
    0x04: "get CAN parameters?",
    0x05: "RESET (soft reset — does NOT appear to save, per this session's testing)",
    0x06: "start data gathering",
    0x07: "stop gathering?",
    0x08: "unknown / phase_active?",
    0x09: "reset pins",
    0x0F: "same as 0x05 in some conditions",
}

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

TEMP_SENSOR_NAMES = {0:'None',1:'PTC-1000',2:'NTC-230K',3:'KTY84-130',
                      4:'Hypothetical',5:'KTY83-122',6:'NTC-10K',7:'NTC-100K'}
PASSWORD_STATUS_NAMES = {0:'Protected',1:'Protected(alt)',2:'NoPassword'}


def decode_read_block(addr, data):
    """Best-effort decode of the fields we've verified this session. Returns
    a short string, or '' if this block has nothing we currently decode."""
    parts = []
    if addr == 0x06 and len(data) >= 12:
        parts.append(f"throttle_lo={data[4]/20.0:.2f}V throttle_hi={data[5]/20.0:.2f}V")
        temp_sensor = (data[10] >> 4) & 0x07
        phase_ex = (data[10] >> 7) & 0x01
        brake_cfg = data[10] & 0x0F
        direction = (data[11] >> 7) & 0x01
        parts.append(f"temp_sensor={temp_sensor}({TEMP_SENSOR_NAMES.get(temp_sensor,'?')})")
        parts.append(f"direction={direction}")
        parts.append(f"phase_exchange={phase_ex}")
        parts.append(f"brake_config={brake_cfg}")
    elif addr == 0x1E and len(data) >= 4:
        lvp = (data[2] | (data[3] << 8)) / 10.0
        parts.append(f"low_vol_protect={lvp}V")
    elif addr == 0xB8 and len(data) >= 10:
        pw = (data[9] >> 4) & 0x03
        parts.append(f"password_status={pw}({PASSWORD_STATUS_NAMES.get(pw,'?')})")
    elif addr == 0xE2 and len(data) >= 2:
        b0, b1 = data[0], data[1]
        forward = bool(b0 & 0x01); reverse = bool(b0 & 0x02)
        gear = (b0 >> 2) & 0x03; motion = bool(b0 & 0x20)
        comp_phone_ok = bool(b0 & 0x80)
        pass_ok = (b1 >> 3) & 0x03
        parts.append(f"gear={gear} fwd={int(forward)} rev={int(reverse)} motion={int(motion)}")
        parts.append(f"comp_phone_ok={int(comp_phone_ok)} pass_ok={pass_ok}")
    return "  ".join(parts)


def try_parse_read16(buf):
    """AA [0x80|id] [12 data] [crc_a] [crc_b] -- 16 bytes total."""
    if len(buf) < 16 or buf[0] != 0xAA:
        return None
    flags_id = buf[1]
    if (flags_id >> 6) != 2:   # top 2 bits must be 0b10 (flags=2, "receiving")
        return None
    ca, cb = _crc(bytes(buf[0:14]))
    if (ca, cb) != (buf[14], buf[15]):
        return None
    msg_id = flags_id & 0x3F
    addr = FLASH_READ_ADDR[msg_id] if msg_id < len(FLASH_READ_ADDR) else None
    data = bytes(buf[2:14])
    return {'type': 'READ', 'len': 16, 'msg_id': msg_id, 'addr': addr, 'data': data}


def try_parse_write8(buf):
    """AA C6 [addr] [addr] [lo] [hi] [crc_a] [crc_b] -- 8 bytes total."""
    if len(buf) < 8 or buf[0] != 0xAA or buf[1] != 0xC6:
        return None
    if buf[2] != buf[3]:   # addr_confirm must match
        return None
    ca, cb = _crc(bytes(buf[0:6]))
    if (ca, cb) != (buf[6], buf[7]):
        return None
    addr = buf[2]
    value = buf[4] | (buf[5] << 8)
    if addr == 0xA0 and buf[4] == 0x88:
        # This is the syscmd sub-format: lo byte is the 0x88 marker, hi byte is the command
        return {'type': 'SYSCMD', 'len': 8, 'cmd': buf[5]}
    return {'type': 'WRITE', 'len': 8, 'addr': addr, 'value': value}


def try_parse_sendcmd8(buf):
    """AA <cmd> <~cmd> <sub> <v1> <v2> <crc> <~crc> -- old-style, 8 bytes."""
    if len(buf) < 8 or buf[0] != 0xAA:
        return None
    if buf[2] != ((~buf[1]) & 0xFF):
        return None
    checksum = sum(buf[0:6]) & 0xFF
    if buf[6] != checksum or buf[7] != ((~checksum) & 0xFF):
        return None
    return {'type': 'SENDCMD', 'len': 8, 'command': buf[1], 'sub_command': buf[3],
            'value1': buf[4], 'value2': buf[5]}


def format_frame(ts, frame, raw):
    hexstr = ' '.join(f'{b:02X}' for b in raw)
    if frame['type'] == 'READ':
        addr = frame['addr']
        addr_str = f"0x{addr:02X}" if addr is not None else "?"
        decoded = decode_read_block(addr, frame['data']) if addr is not None else ''
        tail = f"  {decoded}" if decoded else ""
        return f"[{ts}] [READ  ] id={frame['msg_id']:02X} addr={addr_str}{tail}  | raw: {hexstr}"
    if frame['type'] == 'WRITE':
        return (f"[{ts}] [WRITE ] addr=0x{frame['addr']:02X} value=0x{frame['value']:04X} "
                f"(={frame['value']})  | raw: {hexstr}")
    if frame['type'] == 'SYSCMD':
        name = SYSCMD_NAMES.get(frame['cmd'], '?')
        return f"[{ts}] [SYSCMD] cmd=0x{frame['cmd']:02X} ({name})  | raw: {hexstr}"
    if frame['type'] == 'SENDCMD':
        name = SEND_CMD_NAMES.get((frame['command'], frame['sub_command']), '?')
        return (f"[{ts}] [SENDCMD] command=0x{frame['command']:02X} "
                f"sub=0x{frame['sub_command']:02X} v1={frame['value1']} v2={frame['value2']} "
                f"({name})  | raw: {hexstr}")
    return f"[{ts}] [???   ]  | raw: {hexstr}"


def is_printable(b):
    return 0x20 <= b <= 0x7E or b in (0x0D, 0x0A)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', required=True, help='Serial port to LISTEN on (read-only)')
    ap.add_argument('--baud', type=int, default=19200)
    ap.add_argument('--label', default='', help='Free-text label embedded in the log header, '
                                                  'e.g. "TX-from-phone" or "RX-from-fardriver"')
    ap.add_argument('--log', default=None, help='Also write output to this file')
    args = ap.parse_args()

    logf = open(args.log, 'a') if args.log else None

    def out(line):
        print(line)
        if logf:
            logf.write(line + '\n')
            logf.flush()

    # Open READ-ONLY. rtscts/dsrdtr explicitly disabled so opening this port
    # cannot pulse DTR/RTS -- some USB-TTL adapters wire those to a target
    # reset pin, and we do not want this "silent" tool to reset anything.
    ser = serial.Serial(args.port, args.baud, timeout=0.05,
                         rtscts=False, dsrdtr=False)
    try:
        ser.setDTR(False)
        ser.setRTS(False)
    except Exception:
        pass  # not all backends support this; non-fatal either way

    out(f"# fardriver_silent_monitor -- PASSIVE, sends nothing")
    out(f"# port={args.port} baud={args.baud} label={args.label!r}")
    out(f"# started {datetime.now().isoformat()}")

    buf = bytearray()
    try:
        while True:
            chunk = ser.read(256)
            if chunk:
                buf.extend(chunk)

            progressed = True
            while progressed and len(buf) > 0:
                progressed = False
                ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]

                if buf[0] == 0xAA:
                    frame = (try_parse_read16(buf) or try_parse_write8(buf)
                             or try_parse_sendcmd8(buf))
                    if frame:
                        raw = bytes(buf[:frame['len']])
                        out(format_frame(ts, frame, raw))
                        del buf[:frame['len']]
                        progressed = True
                        continue
                    # 0xAA but nothing validated yet -- maybe we don't have
                    # enough bytes for the frame yet. Only give up and
                    # resync past this 0xAA if we already have a full
                    # 16 bytes buffered and none of the interpretations
                    # matched (i.e. this 0xAA is noise, not a real header).
                    if len(buf) >= 16:
                        out(f"[{ts}] [???   ]  | raw: {buf[0]:02X}  (unmatched 0xAA, resyncing)")
                        del buf[:1]
                        progressed = True
                    continue

                if is_printable(buf[0]):
                    j = 0
                    while j < len(buf) and is_printable(buf[j]):
                        j += 1
                    if j < len(buf) or len(buf) > 64:
                        text = bytes(buf[:j]).decode('ascii', errors='replace')
                        out(f"[{ts}] [TEXT  ] {text!r}")
                        del buf[:j]
                        progressed = True
                    continue

                # Unrecognized single byte -- drop and resync
                out(f"[{ts}] [???   ]  | raw: {buf[0]:02X}")
                del buf[:1]
                progressed = True

    except KeyboardInterrupt:
        out("# stopped (Ctrl-C)")
    finally:
        ser.close()
        if logf:
            logf.close()


if __name__ == '__main__':
    main()