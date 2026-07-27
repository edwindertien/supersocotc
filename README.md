# Super Soco TC — Controller Conversion & RS485 Bridge

> **Goal:** Replace the stock motor controller with a tunable FarDriver ND72360 to enable regenerative braking, and build a Raspberry Pi Pico W bridge that keeps the original instrument panel working via the Super Soco RS485 protocol.

*Inspired by / adapted from:*
- https://github.com/stprograms/SuperSoco485Monitor
- https://github.com/stprograms/SuperSoco485
- https://github.com/jackhumbert/fardriver-controllers
- https://github.com/bobecek79/ESP32-Fardriver-BLE-Reader
- The real FarDriver Android app itself, decompiled — see Discovery #13

---

## Background

A 2021 Super Soco TC started throwing a **motor failure (engine failure) warning** at sustained maximum speed. After a 20-minute cool-down the bike would restart, pointing to a thermal shutdown. The stock controller provides no way to tune thermal limits, reduce phase current, or enable regenerative braking.

**First problem discovered along the way:** the motor had stopped working entirely — traced to a faulty Hall sensor (yellow channel, 100Ω short to GND) in the Bosch hub motor, plus a rotated sensor after reassembly. Error code **E96** (Hall sensor error). Fixed by repair and careful reassembly.

**Primary goal:** replace the stock controller with a FarDriver ND72360 to:
- Enable regenerative braking (stock system has none — or so we thought; see §Discoveries)
- Tune phase current limits to prevent thermal overload
- Gain telemetry visibility into motor and battery state

**Longest-running sub-problem, now resolved:** parameter writes (undervoltage cutoff, temperature sensor type, direction) would update the controller's RAM immediately but never survive a reset or power cycle, across many sessions of hypothesis-testing on the serial protocol. Solved by decompiling the real FarDriver Android app and passively sniffing a real session with it — see Discoveries #13-14.

---

## What we built

| File | Description |
|---|---|
| `supersoco_monitor.py` | Terminal-based RS485 monitor (rich dashboard) |
| `supersoco_web.py` | **Web-based** RS485 monitor (Flask + SSE, no tkinter) |
| `fardriver_web.py` | **Web-based** FarDriver diagnostic + settings tool |
| `fardriver_silent_monitor.py` | **Passive, read-only** serial sniffer — sends nothing, ever. Used to capture and decode real traffic between the actual Android app/BT module and the controller. See Discovery #14. |
| `fardriver_tool.py` | (legacy) tkinter version of FarDriver tool |

### Running the tools

```bash
pip install flask pyserial

# Monitor the Super Soco RS485 bus live:
python3 supersoco_web.py --port /dev/tty.usbserial-AK04P0KB

# Replay a saved raw binary capture:
python3 supersoco_web.py --replay ride.bin

# FarDriver diagnostic + settings (connect via 3.3V TTL serial):
python3 fardriver_web.py --port /dev/tty.usbserial-AK04P0KB --baud 19200

# Passive sniffer (sends nothing — safe to run alongside a real app session):
python3 fardriver_silent_monitor.py --port /dev/tty.usbserial-AK04P0KB --baud 19200 --log capture.txt

# All web tools open at http://localhost:5000
# If AirPlay Receiver occupies port 5000:
python3 supersoco_web.py --webport 5001
```

---

## System architecture

### What we assumed (original model)

![Original architecture](docs/architecture_original.svg)

The protocol documentation (SuperSoco485Monitor repo) describes four units on a single RS485 bus: an **ECU (0xAA)** acting as bus master polling the controller, battery, and speedometer. The ECU compiles data and pushes a SpeedometerRequest packet to the display.

### What we actually found

![Actual architecture](docs/architecture_actual.svg)

**The ECU socket is physically present on the bike but unpopulated.** No ECU is connected. Instead, the **instrument panel (0xBA) itself is the bus master** — it polls the controller and battery directly and displays the data without any intermediary.

Additionally, the tap point used first turned out to be a **secondary battery port** (separate RS485 segment for an optional second battery), not the main bus. The controller traffic appeared on this segment too, because the instrument panel broadcasts across both.

Key discovery from live bus capture:

```
[21:50:58] CTRL RSP  Gear=3  Speed=28.1km/h  Current=0.0A
[21:50:59] BATTERY RSP  65V  SoC=62%  Current=0A  Activity=charging   ← regen!
[21:50:59] CTRL REQ  charging=yes                                       ← ECU signals regen
```

**The stock Super Soco TC does have regenerative braking** — it activates on throttle roll-off above ~28 km/h. The battery reports `Activity=charging` and the controller request switches to `charging=yes`. This was not previously documented publicly for this model.

### Target architecture (in progress)

![Target architecture](docs/architecture_target.svg)

The Pico W bridge sits between the FarDriver, battery, and instrument panel:
- **UART 0:** reads FarDriver telemetry (speed, gear, current, temp, errors) via 3.3V TTL serial
- **UART 1:** polls battery via RS485 (existing Super Soco protocol, 9600 baud)
- **UART 2:** acts as bus master toward the instrument panel, sending SpeedometerRequest telegrams compiled from the above data

The stock display is retained unchanged. A future circular touchscreen could be added via Pico W WiFi or SPI for temperature and regen display.

---

## Key discoveries

### 1. Bus master is the instrument panel, not an ECU
The ECU socket on the TC wiring harness is populated with RS485 and power wires but no ECU board is fitted. The instrument panel acts as master, polling `0xDA` (controller) and `0x5A` (battery) directly.

### 2. Regenerative braking exists in the stock TC
Bus capture during a deceleration run showed the battery switching to `Activity=charging` and the controller request byte switching to `charging=yes` at speeds above ~28 km/h on throttle roll-off. Battery current goes negative (charging) during this phase.

### 3. Speed factor correction (stock bus)
The SuperSoco485Monitor C# source uses a speed factor of `0.028` for ControllerResponse telegrams. Empirical data from a live ride (controller reporting ~11.6 km/h, actual speedometer showing 45 km/h) gives a corrected factor of **`0.109`**. Note: this factor is specific to the *stock RS485 bus's own* speed telegram — see Discovery #15 for a related mix-up with the FarDriver's own speed field.

### 4. Hall sensor orientation matters
After disassembling the Bosch hub motor, the yellow Hall sensor PCB was reinstalled with a slightly different angular orientation relative to the other two. This caused the motor to run erratically (backwards, jerky, error E96) even after the electrical fault was repaired. The fix was to carefully match the original angular position of all three sensors on the PCB.

### 5. Open-collector Hall outputs need pull-ups
The three Hall signal wires are open-collector outputs. Pull-up resistors live on the **controller side**, not on the motor PCB. The yellow channel's 100Ω short to GND was enough to defeat the pull-up and prevent the signal from reaching the supply rail.

### 6. Battery terminal corrosion caused no-start
A dull grey (oxidised) terminal on one battery connector was causing a high-resistance junction. At low current draw (lights, controller boot) the bike appeared to start. Under the higher current demand of motor startup, the voltage drop across the corroded joint was sufficient to trigger BMS protection. Fix: clean with 600-grit sandpaper, apply dielectric grease.

### 7. FarDriver baud rate is 19200
Confirmed twice over: matches the official FarDriver manual (§12.1, "Baud rate: fixed 19200"), and matches what actually works end-to-end on the bench. An earlier note in this project claimed 9600 "confirmed by logic analyser" — that appears to have been wrong; 19200 is what's demonstrably correct.

### 8. jackhumbert/fardriver-controllers' own inline byte-offset comments are wrong
The `fardriver.hpp` struct's `// 0x..` comments next to individual fields do **not** match what the struct actually compiles to. Compiling the real header with g++ and reading `offsetof()` plus a byte-pattern probe (set one bitfield at a time, dump the raw bytes) gives ground truth that's cross-checked against the header's own `static_assert(offsetof(...) == addr<<1)` checks, all of which pass. Anyone working from this repo's struct comments should verify against a compile, not trust the comments directly.

### 9. Throttle voltage was being decoded from the wrong bytes — and Direction was hiding in plain sight
Using the compiled layout: the decoder read ThrottleLow/High from `data[6]`/`data[7]` of the `0x06` block, which is actually the unrelated `FAIF` field. The real bytes are `data[4]`/`data[5]`. While fixing this, the compiled layout also revealed that the **motor direction bit** lives in the same packed word (`0x0B`) as the temperature-sensor-type setting — previously undecoded because nothing read the second byte of that word. Independently confirmed later by decompiling the real app's `DIR_Clicked` method (Discovery #13) — exact bit match.

### 10. Naively overwriting a packed word corrupts its neighbours
The original `temp_sensor` write sent the sensor-type value as if it were the *entire* 16-bit word at address `0x0B` — but that word also packs `BrakeConfig`, `PhaseExchange`, `SlowDown`, `PC13Config`, `CurrAntiTheft`, and `ParkConfig`. Writing that way zeroes all of them. The tool now does a read-modify-write: it reads the last live block, flips only the requested bits, and writes the word back.

### 11. Three more decode bugs found by extending the same compile-and-verify check
Rather than assume only the Addr06 block was affected, the same "compile the real header and check `offsetof()`" method was applied to every other block the tool decodes. Found and fixed:
- **Live "motion" flag was reading the wrong bit** — could plausibly show speed=0 while actually riding.
- **"Phase Lost" fault flag was reading the wrong 16-bit word entirely** — effectively dead in the tool.
- **Hardware/software version off by one byte** — explains an earlier hardware note of "HW ver H/8": the byte read as hardware version was actually the software major-version digit. Same pattern found in the AN/LM wave-tuning bytes.

### 12. A hardcoded default confirmation password and an extensive session-gating flag, found by decompiling the app — but not the actual answer
Digging into the app's login/binding system (`ConnectPage::BindSend`, `App.PassOk`) turned up a hardcoded default confirmation password (`"3414"`) and a pervasive `PassOk`/`BindingStat` gate checked before every write in the real app. Promising, but **ruled out empirically**: this controller reports `pass_ok == 2` continuously, meaning the real app would never hesitate here either. Kept as a free diagnostic (preflight check "PassOk write gate"), but this wasn't what was blocking persistence.

### 13. The write mechanism was never broken — decompiling the real Android app proved it byte-for-byte
The FarDriver app turned out to be built with **Xamarin/.NET for Android**, not plain Java — its actual logic lives in `.NET` assemblies packed inside `assemblies/assemblies.blob` (the "assembly store" format), unpacked with [`pyxamstore`](https://github.com/jakev/pyxamstore) and disassembled to readable IL with `monodis` (Ubuntu's `mono-utils` package) — no full .NET SDK needed. Reading the real `ParaPage::DIR_Clicked` method confirmed the Direction bit position exactly, and confirmed that for this controller (`App.NewVersion == true`, set the first time any correctly-CRC'd packet is received), the real app writes settings with the **exact same plain word-write** this project had used all along, with **no extra reset or confirm step**. The missing piece was never a serial command being sent wrong.

### 14. Passively sniffing a real app session found the actual save command
Since decompiled *logic* couldn't answer "what does a real save actually send on the wire," the next step was a purpose-built passive sniffer (`fardriver_silent_monitor.py` — sends nothing, ever, verified by grep) tapping the line between a real Android phone running the actual app and the controller. The capture showed:
- Every parameter write matched this project's own packets byte-for-byte (confirming Discovery #13 held for real use, not just for one button).
- The real app sends a continuous ~1-second heartbeat (`AA 13 EC 07 09 6F 28 D7`) the entire time it's connected — never done by this project's tool before.
- **The real save command is syscmd `0x04`, not `0x05`.** Confirmed by directly correlating timestamps: writes → `0x04` → (3-8 seconds later) the BT module's own reconnect handshake, showing the controller genuinely committed and rebooted. `0x05` (what this project had been using) just reboots without committing anything first.

Implemented in `fardriver_web.py` (write → `0x04` instead of `0x05`, plus the continuous heartbeat) and **confirmed working end-to-end**: values set via the tool itself now survive an actual power cycle. This resolves a persistence problem that spanned many sessions of hypothesis-testing.

### 15. The FarDriver's own speed field was using a calibration constant from an entirely different protocol
The `0.109` speed factor (Discovery #3) was calibrated for the *stock Super Soco RS485 bus's* own speed telegram — a different device and protocol, from before FarDriver was even installed. It had been mistakenly reused for the FarDriver's own internal speed field (`AddrE2.MeasureSpeed`), which was never actually calibrated. Symptom: displayed speed reaching ~450 km/h when true speed was ~45 km/h — consistent with a clean 10x mismatch. Corrected to `0.0109`, explicitly flagged as provisional pending a real calibration (raw value now exposed on the dashboard for that purpose). Also removed a clamp that was silently zeroing the display above 200 km/h — it was hiding the miscalibration signal rather than handling genuine noise.

### 16. Battery SoC has a real formula, sitting in the header the whole time
The old SoC reading (`AddrF4` raw byte) looked unreliable — showed the wrong value while voltage was healthy. Found the actual formula directly in `fardriver.hpp` as a helper method (`GetBatteryP()`): simple linear interpolation between two configured reference voltages (`ZeroBattCoeff` = 0%, `FullBattCoeff` = 100%), both readable from `Addr0C`. Implemented the same calculation client-side; shown as the primary SoC estimate now, with the old raw byte kept alongside for comparison.

---

## Hardware notes

### FarDriver ND72360
- **Voltage:** 48–72V (run at 60V with stock battery)
- **Phase current:** 190A peak
- **Position sensor:** 120° Hall (option 0 in app)
- **Bluetooth:** built-in (BLE, device name `YQxxx`)
- **Serial:** 3.3V TTL, 4-pin header, **19200 baud** — **Pin 1 (3.3V supply) must NOT be connected**
- **Self-learn command:** `AA C6 A0 A0 88 02 45 0E`
- **Commit to flash (the real save):** `AA C6 A0 A0 88 04 C5 0C`
- **Reset only (no save):** `AA C6 A0 A0 88 05 04 CC`
- **Factory reset:** `AA C6 A0 A0 88 08 C5 09`
- **First run minimum wiring:** battery + 3× phase + Hall connector (6-wire) + throttle (3-wire) + brake signal

### Bosch Hub Motor
- OEM unit for Super Soco TC, not available as a catalog part
- 1500W nominal / 3500W peak / 150Nm peak torque
- 12" outrunner BLDC
- 6-wire Hall connector: +5V, GND, Yellow, Blue, Green (signals), White (thermistor)
- Hall sensors are open-collector; pull-ups on controller side

### RS485 bus (stock, unrelated to FarDriver)
- Protocol: `[type_hi][type_lo][dst][src][pdu_len][pdu...][checksum][0x0D]`
- Request type: `0xC5 0x5C` · Response type: `0xB6 0x6B`
- Checksum: XOR of `pdu_len` and all PDU bytes
- Baud: 9600, 8N1
- Unit addresses: ECU `0xAA`, Controller `0xDA`, Battery `0x5A`, Speedo `0xBA`
- 4-pin JST SM connector under seat (A+/A− for one segment, B+/B− for second battery segment)

---

## FarDriver protocol notes

Protocol documented at https://github.com/jackhumbert/fardriver-controllers, cross-checked against the real Android app (Discoveries #13-14) and the official translated manual (`MANUAL.md` in that same repo).

- CRC (read/write frames): two-table algorithm, init `a=0x3C, b=0x7F`
- CRC (old-style "Sending commands" 8-byte format): sum-of-bytes, `crc = sum(6 bytes) & 0xFF`
- Packet structure: `0xAA [0xC0+len] [addr] [addr] [data...] [crc_a] [crc_b]`
- Message length: 16 bytes for status messages, 8 bytes for writes/commands
- Key addresses:
  - `0xE2`: gear, direction (live/telemetry), motion, fault flags, speed raw, modulation, `PassOk`/`CompPhoneOk` session bits
  - `0xE8`: bus voltage, line current
  - `0xF4`: motor temperature, battery SoC (raw firmware byte — see Discovery #16 for the better estimate)
  - `0xD6`: MosFET temp, auto-learn state, hall position error, phase lost
  - `0xA0`: model name + system commands
  - `0xB8`: password status (word `0xBC`, bits 4-5)
  - `0xCA`: angle learn status byte (position confirmed, exact value meanings not independently re-verified — see context.md), Battery Signal Source chemistry selector
  - `0x08`: throttle low/high thresholds
  - `0x0B`: **packed** — BrakeConfig, TempSensor (bits 4-6), PhaseExchange (low byte); SlowDown, PC13Config, CurrAntiTheft, ParkConfig, **Direction** (bit 15, high byte)
  - `0x0C`: PID coefficients, plus `ZeroBattCoeff`/`FullBattCoeff` (SoC calibration reference voltages)
  - `0x12`: rated voltage, rated power, pole pairs
  - `0x30`: regen stop current, max regen current

  ⚠️ Byte offsets above are verified by compiling `fardriver.hpp` with g++ and reading real `offsetof()` values — the header's own inline `// 0x..` comments don't reliably match (Discovery #8).

- **Save mechanism (confirmed, Discovery #14):** write the parameter (`0xC6` word-write format), then send syscmd `0x04` (`AA C6 A0 A0 88 04 C5 0C`). The controller reboots 3-8 seconds later. `0x05` (`AA C6 A0 A0 88 05 04 CC`) reboots too, but *without* committing anything first.
- **Heartbeat (unconfirmed whether required, but replicated anyway):** `AA 13 EC 07 09 6F 28 D7`, sent once per second continuously while connected.

---

## Next steps

- [ ] Speed: get a real calibration data point (steady known speed vs. raw value shown on dashboard), compute the correct factor — current `0.0109` is provisional
- [ ] SoC: verify the new voltage-based estimate tracks sensibly over a real charge/discharge cycle; check the Battery Signal Source (chemistry) setting matches the actual pack
- [ ] Auto-learn: confirm whether the status byte ever reaches a third value after a genuine commit — current interpretation (0xAA/0x55) not independently re-verified against the real app
- [ ] Low-voltage fault: test whether a plain reset (with healthy voltage) clears the latched fault
- [ ] Confirm Direction is correct for this motor's wiring (wheel off ground)
- [ ] Increase StopBackCurr (2A→10A) and MaxBackCurr (4A→20A) for meaningful regen, now that saving actually works
- [ ] Pico W bridge firmware (MicroPython): read FarDriver serial + poll battery + emit SpeedometerRequest
- [ ] Ignition switch wiring (replace remote/key system with simple barrel switch)
- [ ] Optional: circular touchscreen for temperature + regen display via Pico W WiFi
- [ ] Add screenshots to `docs/`

---

## Resources

| Resource | Description |
|---|---|
| [SuperSoco485Monitor](https://github.com/stprograms/SuperSoco485Monitor) | C# RS485 protocol sniffer and decoder — source of the packet format and address table used here |
| [SuperSoco485](https://github.com/stprograms/SuperSoco485) | Arduino library for the Super Soco RS485 protocol |
| [jackhumbert/fardriver-controllers](https://github.com/jackhumbert/fardriver-controllers) | FarDriver serial protocol: CRC tables, address map, system commands, `fardriver.hpp`/`fardriver_message.hpp` struct definitions, translated official manual (`MANUAL.md`) |
| [bobecek79/ESP32-Fardriver-BLE-Reader](https://github.com/bobecek79/ESP32-Fardriver-BLE-Reader) | ESP32 BLE reader for FarDriver — GATT service UUIDs and BLE packet format |
| [Endless Sphere: FarDriver thread](https://endless-sphere.com/sphere/threads/fardriver-controller-serial-protocol-reverse-engineering.121825/) | Community megathread: tuning, wiring, self-learn, ND72360 builds |
| [SiAECOSYS FarDriver parameter guide](https://siaecosys.com) | English-language parameter description for FarDriver app — all settings explained |
| [supersocoforum.com](https://supersocoforum.com) | Super Soco community forum — TC-specific builds and documented issues |
| FarDriver Android app | `NanjingFardriver2_4_9.Apk`, via [Google Drive folder](https://drive.google.com/drive/folders/17K1ILh-IekDZlz2ZEMyxLLreL_TSe8y7) ("NANJING FARDRIVER Android Apps Apk and PC software with firmware") — decompiled to find the real save mechanism (Discoveries #13-14) |
| [pyxamstore](https://github.com/jakev/pyxamstore) | Unpacks Xamarin's assembly-store blob format (`assemblies.blob`) into individual .NET `.dll` files |
| `mono-utils` (`monodis`) | Ubuntu package — disassembles .NET DLLs to readable IL without needing the full .NET SDK; sufficient for everything found in Discoveries #13-14 |
| [androguard](https://github.com/androguard/androguard) | `pip install androguard` — tried early for basic APK/manifest/string inspection; superseded by `monodis` once the Xamarin/.NET structure was clear |

---

*Project by Edwin · University of Twente / FabLab Oldenzaal · 2024–2026*