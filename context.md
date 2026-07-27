# Super Soco TC — FarDriver Conversion: Handoff Context

This document is a complete handoff for a fresh conversation continuing this project.

---

## Project summary

2021 Super Soco TC electric motorcycle. Replacing the stock motor controller with a **FarDriver ND72360** to enable regenerative braking. The original stock controller already had regen (detected via RS485 bus capture), but was causing thermal shutdowns at max speed.

**First problem solved along the way:** Bosch hub motor Hall sensor (yellow/center channel, 100Ω short to GND) → E96 error. Turned out to be two separate faults: the original short, then the *replacement* sensor fitted 180° rotated relative to the other two (easy mistake — nothing makes the correct orientation obvious), which reversed its supply polarity and broke that sensor too. Motor runs fine once fitted with correct orientation. Battery terminal corrosion also caused a no-start (see README for full details on both).

**The big one, now resolved:** for a long stretch of this project, parameter writes (undervoltage cutoff, temperature sensor type, direction, etc.) would update the controller's RAM immediately but never survive a reset or power cycle. This is now solved — see "Write/save mechanism — RESOLVED" below. The short version: the real save command is `0x04`, not the `0x05` this project had been using; found by decompiling the actual FarDriver Android app and passively sniffing a real session with it.

---

## Hardware

| Item | Detail |
|---|---|
| Bike | 2021 Super Soco TC |
| Motor | Bosch hub motor, 1500W BLDC, 12", 3× Hall + thermistor (white wire) |
| Battery | 17S NMC, nominal 61.2V, full charge 71.4V, cutoff ~52V |
| New controller | FarDriver ND72360, model JSNJ012404, HW ver **H** (corrected — see decode bug list below), SW ver 8.5 |
| Dev machine | MacBook Pro (username Dertien), Python 3.11 via PlatformIO venv |
| Serial adapter | USB-TTL 3.3V (CP2102 or similar), port `/dev/tty.usbserial-AD026699` |

### FarDriver serial header (4-pin, labelled "USB")
```
Pin 1  3.3V  ← DO NOT CONNECT
Pin 2  GND   → adapter GND
Pin 3  RXD   → adapter TX
Pin 4  TXD   → adapter RX
Baud: 19200, 8N1  (confirmed twice over: matches the official FarDriver manual §12.1
                   "Baud rate: fixed 19200", and matches what actually works on the bench)
```

---

## RS485 bus architecture (confirmed by bus capture) — stock Super Soco protocol, unrelated to FarDriver

**Three units on the bus. Instrument panel is bus master — no ECU.**

| Address | Unit | Role |
|---|---|---|
| 0xBA | Instrument panel | ★ Bus master — polls controller + battery |
| 0xDA | Stock controller | Slave — reports speed/gear/current/temp |
| 0x5A | Battery | Slave — reports voltage/SoC/temp/cycles |

ECU socket (0xAA) is physically present on the wiring harness but **unpopulated**. There is a secondary RS485 segment for an optional second battery (same addresses).

Protocol: 9600 baud, 8N1, request `0xC5 0x5C`, response `0xB6 0x6B`, XOR checksum.

Speed factor for *this* stock bus: `0.109` (empirically calibrated against a live ride — C# source said `0.028`, that's wrong for this telegram). **Important:** this `0.109` constant is specific to the stock RS485 bus's own speed telegram. It was mistakenly reused for the FarDriver's own internal speed field too — see "Speed calibration" below, that's a separate, still-open problem.

Regen confirmed on stock TC: activates at ~28 km/h on throttle roll-off, battery reports `Activity=charging`.

**Resources:**
- https://github.com/stprograms/SuperSoco485Monitor
- https://github.com/stprograms/SuperSoco485

---

## FarDriver serial protocol

**Resources:**
- https://github.com/jackhumbert/fardriver-controllers ← primary reference: `fardriver.hpp` (C++ struct definitions), `fardriver_message.hpp`, its own README (protocol/command notes), `MANUAL.md` (translated official FarDriver manual)
- https://github.com/bobecek79/ESP32-Fardriver-BLE-Reader
- The real FarDriver Android app itself — see "Reverse-engineering the real app" below. APK: `NanjingFardriver2_4_9.Apk`, sourced from a shared Google Drive folder ("NANJING FARDRIVER Android Apps Apk and PC software with firmware"), link: https://drive.google.com/drive/folders/17K1ILh-IekDZlz2ZEMyxLLreL_TSe8y7

### BT handshake (MUST be emulated before data flows, when connecting our own tool directly)

The "USB" header is shared with the BT module. Without the physical BT dongle, the controller loops on `AT+VERSION`. The handshake must be emulated:

```
FarDriver → us:  AT+VERSION  (10 bytes, no terminator)
us → FarDriver:  +VERSION=\x01\x08k\x199\xa4\xe2~\x86\x97\xd8\xe6{\xfd\xb1g\r\n  (27 bytes)
FarDriver → us:  AT+PAWD=<16 bytes>  (24 bytes)
us → FarDriver:  +PAWD=\xce\x1d\r\x8d9\x9f\xd3\xf3\x86\x91\xf6B|w\x85\xe1  (22 bytes, NO \r\n)
× 3 rounds of PAWD, then data frames begin
```

Captured via Saleae logic analyser (files: `startupRX.txt`, `startupTX.txt`). Confirmed again independently via passive capture of a real app session (see below) — same exact bytes.

### Read frames (controller → us, 16 bytes each)

```
AA [0x80|id] [data 12 bytes] [crc_a] [crc_b]
```

CRC: two-table algorithm, init `a=0x3C, b=0x7F`. The `flashReadAddr` table maps id → block address:
```
[0xE2,0xE8,0xEE,0x00,0x06,0x0C,0x12,0xE2,0xE8,0xEE,0x18,0x1E,0x24,0x2A,
 0xE2,0xE8,0xEE,0x30,0x5D,0x63,0x69,0xE2,0xE8,0xEE,0x7C,0x82,0x88,0x8E,
 0xE2,0xE8,0xEE,0x94,0x9A,0xA0,0xA6,0xE2,0xE8,0xEE,0xAC,0xB2,0xB8,0xBE,
 0xE2,0xE8,0xEE,0xC4,0xCA,0xD0,0xE2,0xE8,0xEE,0xD6,0xDC,0xF4,0xFA]
```

### Write format

```
AA C6 [word_addr] [word_addr] [lo] [hi] [crc_a] [crc_b]
```

- Both bytes 2-3 are the **same** word address (addr_confirm)
- **WORD addressing confirmed** by `static_assert: offsetof(FardriverData, addr12) == (0x12 << 1)`
- `GetAddr(addr)` = `(uint8_t*)this + (addr << 1)` → each addr unit = 2 bytes
- **This exact packet format and CRC is byte-for-byte confirmed against the real Android app** — see below. Our own write mechanism was never the problem.

### System commands (write to addr 0xA0, format `AA C6 A0 A0 88 <cmd> <crc_a> <crc_b>`)

| Command | Packet | Effect | Status |
|---|---|---|---|
| Self-learn | `AA C6 A0 A0 88 02 45 0E` | Start Hall angle detection | in use |
| Data gather | `AA C6 A0 A0 88 06 44 CD` | Start streaming | in use |
| **Commit to flash** | `AA C6 A0 A0 88 04 C5 0C` | **The real save command** — confirmed against a real app capture; reboots the controller (3-8s later) after committing pending RAM changes | ⭐ confirmed 2026-07-27, now used by `fardriver_web.py` |
| Reset only | `AA C6 A0 A0 88 05 04 CC` | Reboots WITHOUT committing anything first — this project used to think this was the save command; it is not | corrected |
| Factory restore | `AA C6 A0 A0 88 08 C5 09` | Restore flash defaults to RAM | in use |

### Old-style "Sending commands" 8-byte protocol (separate from the above)

```
AA <command> <~command> <sub_command> <value1> <value2> <crc> <~crc>
crc = sum(the 6 preceding bytes) & 0xFF        (NOT the two-table CRC)
```

Two confirmed uses found this session:
1. **Heartbeat**: `AA 13 EC 07 09 6F 28 D7` (command=0x13, sub=0x07, v1=9, v2=111) — the real app sends this exact packet **continuously, roughly once per second, the entire time it's connected**. Command 0x13 is the login/binding system per the community README ("interacts with the login/binding system, can set password, phone number"); sub-command 0x07 "seems to be related to the login/status update system". Not confirmed whether this heartbeat is *required* for persistence to work or just something the real app also happens to do — but it was present throughout the session where persistence was confirmed working, so `fardriver_web.py` now sends it too continuously while connected, just in case.
2. Verified checksum formula against a captured example from `ConnectPage.cs`: `AA 05 FA 01 5F 5F 68 97` (command=0x05, sub=0x01, v1=0x5F, v2=0x5F — "sent after updating date & time... may get CAN params" per the community README). This command turned out to be unrelated to saving.

⚠️ **Do not experiment with command 0x13's other sub-commands.** Per the community README, sub-commands `0x10-0x13` under category 0x13 set the controller's password, and `0x14-0x24` set a bound phone number. Sending the wrong values there could genuinely change what the controller considers "authorized" — this is not the same risk class as a reversible parameter write.

---

## Reverse-engineering the real Android app

This is the technique that actually solved the persistence mystery, after a long stretch of hypothesis-testing on the serial protocol alone didn't get there. Worth documenting in full since it may be needed again (e.g. to pin down the SoC-source selector, or investigate other unconfirmed fields).

### Getting the APK

Shared Google Drive folder ("NANJING FARDRIVER Android Apps Apk and PC software with firmware"): https://drive.google.com/drive/folders/17K1ILh-IekDZlz2ZEMyxLLreL_TSe8y7 — download `NanjingFardriver2_4_9.Apk` (or the current version) from there.

### Decompiling it

**This is not a plain Java/Kotlin Android app.** It's built with **Xamarin/.NET for Android** — confirmed immediately by the presence of `libmonodroid.so`, `libmonosgen-2.0.so`, `libxamarin-app.so`, `libmono-btls-shared.so` inside the APK. A plain Java decompiler (jadx) will only show a thin bootstrap/glue layer — the actual app logic is in .NET assemblies, packed into `assemblies/assemblies.blob` (the "assembly store" format used by newer Xamarin builds to reduce file count). [`androguard`](https://github.com/androguard/androguard) (`pip install androguard`) was also tried early on for basic APK/manifest/string inspection — useful for a first look, but `monodis` ended up being what actually mattered once the Xamarin structure was clear.

Steps that worked:
```bash
# 1. Extract the APK (it's a zip)
unzip NanjingFardriver2_4_9.Apk -d apk_extracted

# 2. Unpack the assembly store blob into individual .dll files
pip install pyxamstore --break-system-packages   # not on PyPI under that exact name at
                                                   # time of writing -- clone from source instead:
git clone https://github.com/jakev/pyxamstore.git
pip install -r pyxamstore/requirements.txt --break-system-packages
pip install -e pyxamstore --break-system-packages
cd apk_extracted/assemblies
pyxamstore unpack -d .
# → produces out/MotorNet6.dll, out/MotorNet6.Android.dll, out/Plugin.BLE.dll, etc.
# MotorNet6.dll (~6.8MB) is the one that matters -- the actual app logic.
# Plugin.BLE confirms it talks to the controller over Bluetooth LE.

# 3. Disassemble the .NET DLL to readable IL (not full C#, but complete:
#    every string, method name, class name, and numeric constant is visible)
apt-get install -y mono-utils     # installs monodis (needs Java too, apt pulls it in)
monodis MotorNet6.dll > motornet6.il
# → ~375,000 lines of IL text, fully grep-able
```

A real C# decompiler (ILSpy/`ilspycmd`) would give nicer output but needs the .NET SDK and NuGet, neither reachable from a sandboxed environment without internet to nuget.org. `monodis`'s raw IL was entirely sufficient — every finding below came from grepping and reading this IL dump directly.

### Key classes found in `MotorNet6.dll`

| Class | What's in it |
|---|---|
| `ConnectPage` | BLE connection handling, `WriteAddr`/`WriteSysCmd`/`SendRs232Data`/`SendRs232DataPass`, `BindSend`, `ManageAuth`, `SaveSK`, `Encrypt`/`RSAEncrypt`, CRC init. Matches "ConnectPage.cs" already referenced in this project's earlier notes. |
| `ParaPage` | The settings/parameters screen — per-field click handlers (`DIR_Clicked`, `TCS_Clicked`, `BC_Clicked`, etc.), each building and sending its own write packet. |
| `GraphPage` | The live-telemetry/dashboard screen. **This is where incoming packets actually get parsed** — `PassOk`, `CompPhoneOk`, `rollingV` (motion), `reversing`, gear, etc. all get set here directly from the raw received bytes. |
| `App` | Static/global state: `NewVersion`, `PassOk`, `HasPassOk`, `BindingStat`, `bms`, etc. |

### Method worth knowing: `ParaPage::DIR_Clicked` (Direction save button)

This single method resolved several open questions at once:
```csharp
V0 = DIRStat.IsToggled ? 1 : 0
if (App.NewVersion) {
    data[5] = (cfg11h & 0x7F) | (V0 << 7)     // bit 7, high byte, word 0x0B
    WriteAddr(data, addr=0x0B, len=2)
    return                                     // nothing else -- no reset, no confirm, no extra save
} else {
    SendRs232Data(0x12, 7, (byte)(V0+1), (byte)((V0+1)>>8))   // old-style command, note value = bit+1
}
```
- Confirms the Direction bit position exactly (bit 7 of the high byte of word `0x0B`) — matches what this project derived independently by compiling `fardriver.hpp`.
- Confirms `App.NewVersion` controllers (which this one is — the flag gets set to `true` the first time *any* correctly-CRC'd packet is received) use the exact same plain `WriteAddr` mechanism this project has used all along. **The missing piece was never a serial command we forgot to send.**
- For old-version controllers, the old-style command's value encoding is `direction_bit + 1` (1=Forward, 2=Reverse), not raw 0/1 — a real correction to an earlier guess in this project, though moot for `NewVersion` controllers.

### `App.PassOk` — a real gate, but not the answer

`App.PassOk` is parsed directly from live telemetry in `GraphPage` (`(data[3] & 0x18) >> 3` on the raw received frame — same bits this project already decodes as `pass_ok` from `AddrE2`). Found a real write-gate in `ConnectPage`:
```
if (PassOk == 0) return;                          // refuse to send anything
if (BindingStat < 1 && PassOk == 1) return;        // also refuse
// otherwise (PassOk 2 or 3, or BindingStat >= 1): the write actually goes out
```
This looked very promising but was **ruled out empirically** — on this controller, `pass_ok` reads `2` continuously, meaning the real app would never hesitate to send a write here either. Kept as a preflight check ("PassOk write gate") since it's free, real information, but it is not what was blocking persistence.

Digging further into the same login/binding system (`ConnectPage::BindSend`) also turned up a **hardcoded default confirmation password (`"3414"`)**, set as `Confirm_password` alongside a `Confirm_PhoneNumber` derived from `App.username`, gated behind `App.PassOk == 2` and an internal `sendconfirm` flag. Interesting, but same conclusion as above — not what was blocking persistence on this controller, since the gate it sits behind is already open.

### `ButtonSaveName_Clicked`, `GetConfirmModify` — red herrings

Checked both; neither relates to settings persistence. `GetConfirmModify` is a remote-assistance (MQTT) permission dialog. `ButtonSaveName_Clicked` is for renaming the device / app-level password prompts.

---

## The passive-sniffing strategy that actually found the answer

Once decompiling the app's *logic* stopped being enough (the real question was "what does a genuine save actually look like on the wire", which static analysis alone can't fully answer), the next step was to **passively listen to a real session** instead of guessing.

### The tool: `fardriver_silent_monitor.py`

A standalone script (separate from `fardriver_web.py`) that **sends nothing, ever** — verified by grepping the file for `ser.write(` calls (only the log file gets written to; the serial port is opened read-only and only ever `.read()` from). Explicitly disables DTR/RTS on open too, since some USB-TTL adapters wire those to a reset pin and the whole point is zero interference.

It recognizes and decodes four frame types on the wire: the 16-byte periodic READ blocks, the 8-byte direct WRITE format, the 8-byte SYSCMD format, and the old-style 8-byte SENDCMD format — reusing the exact same CRC tables and field decoding already verified in `fardriver_web.py`. Anything else shows up as `[TEXT]` (AT-command handshake bytes) or `[???]` (genuinely unrecognized, logged byte-by-byte so it resyncs immediately rather than stalling).

```bash
python3 fardriver_silent_monitor.py --port /dev/tty.usbserial-XXXX --baud 19200 --label "some-label" --log out.txt
```

### The capture setup

Tap the serial line **between the real BT module and the controller** (or between the real Android app and the controller) with this tool listening passively, while using the *real* FarDriver app on a phone to actually change settings. Two passes matter:
- **RX** (what the controller sends back) — establishes ground truth for how the controller reports state.
- **TX** (what the app actually sends) — this is the side that was never observed before, and is what actually answered the persistence question.

First attempt captured RX only (TX wiring wasn't actually connected to the controller that pass) — still useful (confirmed our own decode of `pass_ok`/`comp_phone_ok` against real behaviour, and gave the first "does this survive a reboot" timing data). Second attempt captured the real app's actual TX and settled things definitively.

### What the TX capture showed

Every parameter write from the real app matched **byte-for-byte** what this project's own tool already builds:
- `low_vol_protect=54.0V` → `AA C6 1F 1F 1C 02 3F FE` — identical.
- `direction=Reverse` → `AA C6 0B 0B 60 D0 DA 97` — identical.
- `temp_sensor` changes rewrite the *entire* word `0x0B` (both bytes) each time, tracking full local state rather than doing a bit-level RMW client-side — same net effect as this project's read-modify-write.

Two things the real app does that this project never had:
1. The continuous ~1/second heartbeat (see above).
2. **`syscmd 0x04`, sent shortly after each group of parameter changes** — not `0x05`. Confirmed by directly correlating timestamps: writes → `0x04` → (3-8 seconds later) the BT module's own `SOK`/`AT+VERSION`/`AT+PAWD` re-handshake sequence, showing the controller genuinely rebooted. Two of three observed `0x04` occurrences showed this reconnect sequence (one showed no visible reconnect in the captured window — inconsistent, not fully understood, but the important result — actual persistence — was independently confirmed afterward).

**Note on a mid-investigation correction:** initially assumed `0x04` was a "quiet" commit that didn't reboot the controller, based on our own tool's active-connection logic not reporting a "Fell back to AT mode" event. That check doesn't apply to a passive capture at all — re-examining the raw `SOK`/`VERSION`/`PAWD` sequences directly showed the reboot clearly. Corrected: `0x04` and `0x05` are the same general shape (both reboot); the difference is `0x04` commits to flash *before* rebooting, `0x05` just reboots.

**Confirmed working, end to end:** after implementing `0x04` as the save trigger in `fardriver_web.py` (replacing `0x05`), Edwin set voltage=54V, temp_sensor=6, and direction via the tool itself, and **confirmed all three values survived an actual power cycle.** This is the resolution of the multi-session persistence mystery.

---

## Decode bugs found and fixed this project (compiled-struct verification method)

Recurring theme: `fardriver.hpp`'s inline `// 0x..` comments do **not** reliably match what the struct actually compiles to. The fix, every time: compile the real header with g++ and read `offsetof()` / a byte-pattern probe (set one bitfield at a time, dump the raw bytes), cross-checked against the header's own `static_assert(offsetof(...))` checks (which all pass — so this is ground truth, not guesswork).

1. **Throttle low/high** (`Addr06`): were read from `data[6]`/`data[7]` (actually the unrelated `FAIF` field); real bytes are `data[4]`/`data[5]`.
2. **Direction discovered**: `Direction : 1` lives in the same packed word `0x0B` as `TempSensor`, bit 7 of the high byte — previously undecoded because nothing read that byte. Independently confirmed via the real app's `DIR_Clicked` method (see above) — exact bit match.
3. **Packed-word write safety**: the old `temp_sensor` write overwrote the *entire* word `0x0B`, zeroing `BrakeConfig`/`PhaseExchange`/`SlowDown`/`PC13Config`/`CurrAntiTheft`/`ParkConfig`. Fixed with `write_word_0x0B()`, a proper read-modify-write.
4. **`live.motion`**: was reading bit4 (`sliding_backwards`/"Reversing"), should be bit5 (`motion`/"rollingV") of `AddrE2` byte 0.
5. **`live.phase_lost`**: was reading the wrong 16-bit word entirely (`AddrD6`) — this fault flag was effectively dead.
6. **`hardware_ver`/`software_ver`**: off by one byte (`Addr82`) — `HardwareVersion` is `data[9]`, not `data[10]` (which is really `SoftwareVersionMajor`). Explains why earlier hardware notes said "HW ver H/8" — `data[10]` really does read `8`, it's just the wrong field.
7. **`AN`/`LM`** (`Addr9A`, wave-tuning settings): same off-by-shift pattern, real bytes are `data[4]`/`data[5]` not `data[6]`/`data[7]`.
8. **Speed clamp hiding the real signal**: `live.speed_kmh` was hard-zeroed whenever the calculated value exceeded 200 — this was masking a miscalibrated factor, not handling genuine noise. Removed; raw value now exposed instead (see Speed calibration below).
9. **SoC was a nearly-meaningless raw firmware byte**: replaced with the same voltage-based linear-interpolation formula the real app itself uses (found directly in `fardriver.hpp` as a `GetBatteryP()` helper) — see below.

All other decoded blocks (`AddrE8` voltage/current, `AddrEE` phase currents, `AddrF4` motor temp, `Addr0C` PID coefficients, `Addr12` pole pairs/rated voltage/power, `Addr18`/`Addr24`/`Addr2A`/`Addr30` speed/current limits, `AddrCA` angle-learn status byte position, `AddrA0` model name) were checked the same way and are correct as-is.

---

## Speed calibration — OPEN PROBLEM

`live.speed_kmh = raw_speed × factor`, where `raw_speed` is `AddrE2.MeasureSpeed` (`data[6:8]`, confirmed correct byte position).

**The `0.109` factor this project used was never actually calibrated for this field.** Tracing it back: it comes from the *stock Super Soco RS485 bus's own* speed telegram (see top of this doc), calibrated before FarDriver was even installed, for a completely different protocol and device. Reusing it for the FarDriver's own internal speed field was a mismatch from the start.

**Current state:** changed to `0.0109` (÷10), based on one reported data point — displayed speed hit ~450 km/h when true speed was ~45 km/h, consistent with `raw_speed ≈ 4128` and a corrected factor near `45/4128 ≈ 0.0109`. **This is explicitly provisional**, not a real calibration — just a same-ballpark fix from a single approximate observation.

`raw_speed_value` is now exposed directly on the dashboard next to the speed reading specifically so a proper calibration can be done: hold a genuinely steady, known reference speed (phone GPS app, or a timed/counted wheel-rotation reference), note the raw value shown at that moment, then `correct_factor = true_kmh / raw_speed_value`.

---

## Battery SoC estimate — fixed, needs field verification

Old behavior: `live.batt_soc = data[3]` from `AddrF4` — a raw firmware byte of unknown reliability. Symptom: showed something clearly wrong (should have read ~60%, showed something else) while voltage was healthy.

**Fix:** found the real formula directly in `fardriver.hpp`, as a helper method the app itself apparently uses:
```c
float GetBatteryP() {
    return 100.f * (addrE8.deci_volts - addr0C.ZeroBattCoeff) / (addr0C.FullBattCoeff - addr0C.ZeroBattCoeff);
}
```
Simple linear interpolation between two configured reference voltages (`ZeroBattCoeff` = 0%, `FullBattCoeff` = 100%). These live in `Addr0C`, right after `PhaseOffset` (word `0x0C`): `ZeroBattCoeff` is word `0x0D`, `FullBattCoeff` is word `0x0E` — sequential fields, no bit-packing, same block that was already correctly giving `StartKI` at `data[6]`.

Implemented as `batt_soc_calc` in `fardriver_web.py`, shown as the primary SoC figure on the dashboard, with the old raw firmware byte kept alongside for comparison (labelled "firmware raw byte"). Not yet field-verified against a real, precisely-known SoC — worth checking it tracks sensibly over a full charge/discharge cycle.

There's also a documented **"Battery Signal Source"** selector (manual §7.5.1: options include "Li-Ion simulation", "lead-acid simulation", "LiFePO4 simulation" — i.e. the *firmware's own* SoC estimate depends on which battery-chemistry curve this is set to) via `EBattSignal` in `fardriver.hpp` (`AddrCA`, `BattSignal:4`, `ParkCoeff` byte). If the firmware's own raw byte is being computed from the wrong chemistry curve, that would independently explain a wrong-looking raw value even with healthy real voltage. Not yet cross-checked against the current setting.

---

## Auto-learn status — unresolved, honestly uncertain

`live.angle_learn = data[0]` from `AddrCA`. Byte **position** is compiler-verified solid. The **meaning** of specific values (`0xAA` = "learned", `0x55` = "learning", per `learn_status()`) comes from the community README's inline comment — **searched the decompiled real app specifically for a location comparing a value against both `0xAA` and `0x55` near each other (the fingerprint of this exact check) and found nothing conclusive.** Could be in an obfuscated/encrypted string resource, or the app may not surface this exact field the same way.

Observed: after triggering self-learn, status went from "not learned" (some third value) to "learning" (`0x55`) — consistent with the documented meaning and with self-learn being genuinely in progress, but not independently proof the byte values mean what we think.

**The test that would actually settle it:** once the physical spin→adjust→reverse→stop sequence completes and "💾 Commit Self-Learn Result (0x04)" is clicked (added this session, on the theory that the "learned" status flag is an ordinary flash word like everything else, and was plausibly never persisting for the same reason nothing else was), does the byte move to a *third*, different value? If yes, that's real evidence the field is meaningfully tracking state. If it sits at exactly `0x55` forever regardless of what the motor physically does, that's a much better reason to distrust the interpretation entirely.

---

## Low-voltage fault latching — worth testing, not yet confirmed

Observed: the undervoltage fault (`live.low_vol_stop`, `AddrD6`) stayed active even with real voltage comfortably above the (now correctly-persisting) 54V cutoff. This is very likely a **latching fault** — common design pattern for undervoltage protection on motor controllers, deliberately requiring a clean reboot (not just voltage recovering) to clear, to prevent rapid on/off cycling at marginal voltage. Since `LowVolProtect` persistence was only just fixed, this specific fault may simply be latched from *before* the fix took effect (checked against the old, wrong 63V default at some earlier low moment).

**Suggested test:** with voltage comfortably above 54V, use "⟳ Reset only, no save (0x05)" in the Edit tab and see if the fault clears on the resulting reboot. If it's still latched afterward with genuinely healthy voltage, that points to something else (e.g. a hysteresis margin needing more headroom above the cutoff) — not yet tested.

---

## Removed from the tool this session

- **Phase Exchange** write option — Edwin decided Direction alone covers the rotation-direction need. Removed the UI section, the `api_write` handling, and the parameter from `write_word_0x0B()`. Its *current value* is still correctly preserved (untouched) whenever TempSensor or Direction is written — the read-modify-write logic doesn't need a write UI for a bit to still protect it.
- **"RAM-only" checkbox** and the experimental "Direct Parameter Command" (`Send(cat,idx)`) UI section — both were debugging aids for the since-resolved persistence mystery. Save now always correctly commits via `0x04`.
- **`cmd_increment_savenum()`** — the old, never-actually-confirmed "increment SaveNum" theory, superseded by the confirmed `0x04` mechanism.

---

## Tool files (in `/mnt/user-data/outputs/` and Edwin's `~/git/supersocotc/`)

| File | Description |
|---|---|
| `fardriver_web.py` | Flask web tool for FarDriver: dashboard, pre-flight checks, settings display, write parameters, commands (self-learn etc.), log. Confirmed-working save mechanism (`0x04` + continuous heartbeat), corrected speed/SoC calculation, phase-exchange removed. Run: `python3 fardriver_web.py --port /dev/tty.usbserial-AD026699 --baud 19200` |
| `fardriver_silent_monitor.py` | **New this session.** Standalone, passive, read-only serial monitor — sends nothing, ever. Used to capture and decode real traffic between the actual Android app/BT module and the controller, without our own tool's handshake/writes interfering. See "passive-sniffing strategy" above. |
| `supersoco_web.py` | Flask web tool for Super Soco RS485 bus monitor (stock bus, unrelated to FarDriver). Run: `python3 supersoco_web.py --port /dev/...` |
| `supersoco_monitor.py` | Terminal RS485 monitor |
| `docs/` | Edwin will add screenshots here |
| `docs/architecture_original.svg` | Assumed 4-unit bus architecture (incorrect) |
| `docs/architecture_actual.svg` | Actual architecture: panel as master, no ECU |
| `docs/architecture_target.svg` | Target: FarDriver + Pico W RS485 bridge |
| `docs/hall_sensor_fault.svg` | Hall sensor diagnosis diagram |
| `README.md` | Full project summary |

### fardriver_web.py key implementation details

- BT handshake fully emulated (VERSION + 3× PAWD using captured Saleae bytes) when connecting directly
- Continuous heartbeat (`AA 13 EC 07 09 6F 28 D7`) sent every ~1s while connected, matching the real app
- `PARAM_MAP` uses word addresses (confirmed via static_assert analysis + real-app byte match)
- `write_param(word_addr, raw_u16)` → `AA C6 addr addr lo hi crc crc`
- `write_word_0x0B(temp_sensor=…, direction=…)` — read-modify-write for the packed word, preserves sibling bits including PhaseExchange even without a write UI for it
- Every Save now sends the write, then `CMD_COMMIT_FLASH` (`0x04`) — confirmed real mechanism
- Verification waits 12s post-write (controller reboots 3-8s after `0x04`, plus our own re-handshake time)
- Pre-flight tab: 12 health checks including Hall, phase, throttle, brake, temp, auto-learn, password/login status, PassOk write gate
- Edit tab: inline editors with live current-value display; dropdowns only sync once then respect manual selection (fixed a bug where live polling was silently overwriting in-progress selections)
- Log tab: full raw RX hex, filterable, downloadable
- Dashboard: speed shows raw value alongside calculated km/h (factor flagged provisional); SoC shows calculated estimate primary, raw firmware byte secondary

---

## Next steps

**Verification needed on the bike (all logic-tested, none hardware-confirmed yet):**
1. Speed: get a real calibration data point (steady known speed vs raw_speed_value shown), compute the correct factor
2. SoC: check the new `batt_soc_calc` estimate tracks sensibly over time / a real charge-discharge cycle; check the `BattSignal` (Battery Signal Source) setting matches actual pack chemistry
3. Auto-learn: does the status byte ever move to a third value once self-learn physically completes and "Commit Self-Learn Result" is clicked?
4. Low-voltage fault latch: does "Reset only (0x05)" with healthy voltage clear it?
5. Confirm Direction is set correctly for this motor's actual wiring (rear wheel off ground first)

**Once the above are settled:**
1. Increase StopBackCurr (2A→10A) and MaxBackCurr (4A→20A) for meaningful regen, using the now-confirmed save mechanism
2. Run/confirm throttle self-learn thresholds are sane after a proper commit

**Longer term:**
- Raspberry Pi Pico W RS485 bridge firmware: reads FarDriver serial + polls battery + emits SpeedometerRequest to stock display
- Ignition switch wiring (replace remote/key with barrel switch → KEY pin)
- Optional circular touchscreen for temperature + regen display
- Edwin will add screenshots to a `docs/` folder

---

## Key resources

| Resource | URL / Note |
|---|---|
| jackhumbert fardriver-controllers | https://github.com/jackhumbert/fardriver-controllers — `fardriver.hpp`, `fardriver_message.hpp`, README (protocol/command notes), `MANUAL.md` (translated official manual) |
| SuperSoco485Monitor | https://github.com/stprograms/SuperSoco485Monitor |
| SuperSoco485 Arduino lib | https://github.com/stprograms/SuperSoco485 |
| ESP32 FarDriver BLE reader | https://github.com/bobecek79/ESP32-Fardriver-BLE-Reader |
| Endless Sphere FarDriver thread | https://endless-sphere.com/sphere/threads/fardriver-controller-serial-protocol-reverse-engineering.121825/ |
| FarDriver Android app APK | `NanjingFardriver2_4_9.Apk`, via Google Drive folder https://drive.google.com/drive/folders/17K1ILh-IekDZlz2ZEMyxLLreL_TSe8y7 ("NANJING FARDRIVER Android Apps Apk and PC software with firmware") |
| pyxamstore | https://github.com/jakev/pyxamstore — unpacks Xamarin's assembly-store blob format into individual .dll files |
| mono-utils (`monodis`) | Ubuntu package (`apt-get install mono-utils`) — disassembles .NET DLLs to readable IL without needing the full .NET SDK |
| androguard | `pip install androguard` — tried early for basic APK/manifest/string inspection; not what ultimately solved anything, but a reasonable first step before the Xamarin structure was clear |
| Facebook group tip (unverified 3rd-hand claim) | electricmotorcyclebuilds group post suggesting writes only work in "auto-learn" mode — investigated, not the actual explanation, but prompted the productive line of inquiry that led to decompiling the app |
| Scribd "Far Driver Tuning with reg notes" | JS-walled, could not fetch content directly; a search snippet mentioned a "BINDING (REGISTERING)" step, consistent with what the app's `BindSend`/`ManageAuth` code turned out to do |