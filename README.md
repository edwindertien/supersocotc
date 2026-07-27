# Super Soco TC — Controller Conversion & RS485 Bridge

> **Goal:** Replace the stock motor controller with a tunable FarDriver ND72360 to enable regenerative braking, and build a Raspberry Pi Pico W bridge that keeps the original instrument panel working via the Super Soco RS485 protocol.

*Inspired by / adapted from:*
- https://github.com/stprograms/SuperSoco485Monitor
- https://github.com/stprograms/SuperSoco485
- https://github.com/jackhumbert/fardriver-controllers
- https://github.com/bobecek79/ESP32-Fardriver-BLE-Reader
- The real FarDriver Android app itself, decompiled (see `context.md`)

---

## Background

A 2021 Super Soco TC started throwing a **motor failure (engine failure) warning** at sustained maximum speed. After a 20-minute cool-down the bike would restart, pointing to a thermal shutdown. The stock controller provides no way to tune thermal limits, reduce phase current, or enable regenerative braking.

**First problem discovered along the way:** the motor had stopped working entirely — traced to a faulty Hall sensor (yellow channel, 100Ω short to GND) in the Bosch hub motor, error code **E96**. Fixing it turned out to be two separate faults, not one: the original short, and then the *replacement* sensor fitted 180° rotated relative to the other two — which put its supply polarity backwards and broke that sensor too. Motor runs fine once fitted correctly (see Discovery #3 below).

**Primary goal:** replace the stock controller with a FarDriver ND72360 to:
- Enable regenerative braking (stock system has none — or so we thought; see §Discoveries)
- Tune phase current limits to prevent thermal overload
- Gain telemetry visibility into motor and battery state

**Longest-running sub-problem, now resolved:** parameter writes (undervoltage cutoff, temperature sensor type, direction) would update the controller's RAM immediately but never survive a reset or power cycle, across many sessions of hypothesis-testing on the serial protocol. Solved by decompiling the real FarDriver Android app and passively sniffing a real session with it — full story in `context.md`.

---

## What we built

| File | Description |
|---|---|
| `supersoco_monitor.py` | Terminal-based RS485 monitor (rich dashboard) |
| `supersoco_web.py` | **Web-based** RS485 monitor (Flask + SSE, no tkinter) |
| `fardriver_web.py` | **Web-based** FarDriver diagnostic + settings tool |
| `fardriver_silent_monitor.py` | **Passive, read-only** serial sniffer — sends nothing, ever. Used to capture and decode real traffic between the actual Android app/BT module and the controller. See `context.md` for the full strategy. |
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

*These are the hardware/mechanical findings. All the software and protocol reverse-engineering discoveries (decode bugs, the FarDriver write-persistence investigation, decompiling the Android app, the passive-sniffing strategy that found the real save command, speed/SoC calibration fixes) now live in `context.md`, which is the more detailed technical handoff document.*

### 1. Bus master is the instrument panel, not an ECU
The ECU socket on the TC wiring harness is populated with RS485 and power wires but no ECU board is fitted. The instrument panel acts as master, polling `0xDA` (controller) and `0x5A` (battery) directly.

### 2. Regenerative braking exists in the stock TC
Bus capture during a deceleration run showed the battery switching to `Activity=charging` and the controller request byte switching to `charging=yes` at speeds above ~28 km/h on throttle roll-off. Battery current goes negative (charging) during this phase.

### 3. Hall sensor failure was actually two separate faults, not one
The Bosch hub motor's yellow (center) Hall channel had a 100Ω short to GND, causing error **E96**. But replacing the sensor didn't immediately fix things — because it turned up a *second*, distinct fault:
1. **Original fault:** defective sensor, yellow channel shorted to GND.
2. **Replacement fault:** the new sensor was fitted **rotated 180° relative to the other two**. It's an easy mistake — nothing about the sensor or its mounting makes the correct orientation obvious, so it looks like it should fit either way.
3. **Consequence:** the reversed orientation put the sensor's supply polarity backwards, and that reverse voltage broke the *new* sensor too.
4. **Fix:** refit with the correct orientation (matching the other two). Motor runs fine after that.

Worth remembering for next time: a Hall sensor replacement that seems to go in fine but causes new problems is worth checking for orientation *before* assuming it's another bad part.

### 4. Open-collector Hall outputs need pull-ups
The three Hall signal wires are open-collector outputs. Pull-up resistors live on the **controller side**, not on the motor PCB. The yellow channel's 100Ω short to GND was enough to defeat the pull-up and prevent the signal from reaching the supply rail.

### 5. Battery terminal corrosion caused no-start
A dull grey (oxidised) terminal on one battery connector was causing a high-resistance junction. At low current draw (lights, controller boot) the bike appeared to start. Under the higher current demand of motor startup, the voltage drop across the corroded joint was sufficient to trigger BMS protection. Fix: clean with 600-grit sandpaper, apply dielectric grease.

---

## Hardware notes

### FarDriver ND72360
- **Voltage:** 48–72V (run at 60V with stock battery)
- **Phase current:** 190A peak
- **Position sensor:** 120° Hall (option 0 in app)
- **Bluetooth:** built-in (BLE, device name `YQxxx`)
- **Serial:** 3.3V TTL, 4-pin header, **19200 baud** — **Pin 1 (3.3V supply) must NOT be connected**
- **First run minimum wiring:** battery + 3× phase + Hall connector (6-wire) + throttle (3-wire) + brake signal

*(Protocol commands — self-learn, save/commit, reset, factory-reset — are in `context.md`, along with the rest of the serial protocol.)*

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

The full serial protocol — CRC algorithms, packet formats, the flash address map, the save/commit mechanism, the heartbeat, all of it — is documented in detail in `context.md`, cross-checked against [jackhumbert/fardriver-controllers](https://github.com/jackhumbert/fardriver-controllers) (`fardriver.hpp`, its README, and the translated official manual), the real Android app (decompiled), and a passive capture of a genuine app session. Short version: the write mechanism was correct from early on; the missing piece for months was the save command (`0x04`, not `0x05`), found by decompiling the app and sniffing a real session — see `context.md` for the full story.

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
| FarDriver Android app | `NanjingFardriver2_4_9.Apk`, via [Google Drive folder](https://drive.google.com/drive/folders/17K1ILh-IekDZlz2ZEMyxLLreL_TSe8y7) ("NANJING FARDRIVER Android Apps Apk and PC software with firmware") — decompiled to find the real save mechanism (full story in `context.md`) |
| [pyxamstore](https://github.com/jakev/pyxamstore) | Unpacks Xamarin's assembly-store blob format (`assemblies.blob`) into individual .NET `.dll` files |
| `mono-utils` (`monodis`) | Ubuntu package — disassembles .NET DLLs to readable IL without needing the full .NET SDK; sufficient for everything found decompiling the app (see `context.md`) |
| [androguard](https://github.com/androguard/androguard) | `pip install androguard` — tried early for basic APK/manifest/string inspection; superseded by `monodis` once the Xamarin/.NET structure was clear |

---

*Project by Edwin · University of Twente / FabLab Oldenzaal · 2024–2026*