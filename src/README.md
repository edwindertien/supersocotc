# SuperSoco TC ↔ FarDriver Pico W Bridge (C++ / PlatformIO)

Reads FarDriver telemetry over TTL serial, answers the Super Soco
instrument panel's RS485 polls as if it were the stock controller, and
passively eavesdrops the battery's own responses on the same bus. Status
is viewable over WiFi (the Pico hosts its own access point) from a phone
or laptop browser.

## Structure: one build, feature flags

One set of sources, one PlatformIO environment, one `main.cpp`. What
actually runs is controlled by the `ENABLE_*` flags at the top of
`include/config.h` — same pattern your `picoControl` project already
uses (`#define USE_OLED (1)` etc). Edit a flag, rebuild, upload.

An earlier version of this project used two separate PlatformIO
environments (`picow_cli` / `picow_bridge`) with `build_src_filter` to
select which files got compiled. That mechanism failed twice in a row on
real builds — `main_cli.cpp` silently not getting included, "undefined
reference to setup/loop" at link time — for no benefit over just not
compiling code you haven't enabled yet. Replaced with plain `#if` guards
instead, which is standard C preprocessor behavior with no
platform/toolchain-specific risk.

```cpp
// include/config.h
#define ENABLE_RS485_RESPOND 0   // actively answer the panel's controller (0xDA) requests
#define ENABLE_FARDRIVER     0   // read real FarDriver telemetry over SerialPIO
#define ENABLE_WIFI          0   // WiFi AP + status web page
```

RS485 passive monitoring — raw hex + interpreted printing to Serial,
battery eavesdrop — always runs regardless of these flags; it's the
foundational diagnostic, not optional. The flags layer on additional
pieces one at a time:

1. **Everything at 0** (as shipped). RS485 wiring only needs to be
   connected. Confirm frames decode correctly, checksums pass, battery
   numbers look real.
2. **`ENABLE_RS485_RESPOND 1`.** Start answering the panel's requests
   for the controller. FarDriver's still off, so responses carry
   placeholder/zero data — this step is about confirming the panel
   accepts a response shape at all, not correct values yet.
3. **`+ ENABLE_FARDRIVER 1`.** Real telemetry now flows into those
   responses instead of zeros. Wire up the FarDriver TTL link for this
   one.
4. **`+ ENABLE_WIFI 1`.** Status page comes up alongside everything else.

Build and upload the normal PlatformIO way — there's only one target now:

```bash
pio run -t upload
pio run -t monitor    # 115200 baud
```

Type `s` + Enter any time in the serial monitor for an on-demand RS485
status summary; one also prints automatically every 3 seconds.

```
[12453] REQ       dst=0xDA(controller) len=0  (request, no PDU fields to decode)  | raw: C5 5C BA DA 00 00 0D
[12890] RESP      dst=0x5A(battery) len=10  V=65V SoC=62% T=22C I=0A Cyc=103 VBreaker=0x00 Act=idle/unknown  | raw: B6 6B BA 5A 0A ...
```

If you see mostly `??? unrecognized byte` lines and few or no valid
frames, that's a wiring/polarity/baud problem to chase before anything
else — the RS485 A/B pair is the first thing to try swapping.

## Architecture

```
FarDriver ND72360  --TTL, SerialPIO-->  GP6/GP7  ---\
   (speed, gear,                                     |  (ENABLE_FARDRIVER)
    temp, faults)                                     |
                                                  Pico W, single build
                                                        |   setup() / loop()
Instrument Panel   <--RS485, Serial1-->  GP0/GP1  ----+   - rs485Poll() (always)
   (bus master)                                       |   - fardriverPoll()  (ENABLE_FARDRIVER)
        |                                             |   - webServer.loop() (ENABLE_WIFI)
        v                                             |     (RS485 DE: GP2)
Battery (0x5A)      --RS485 (same bus)----------------/

                    Phone/laptop <--WiFi AP--- Pico W's own access point
                                                (ENABLE_WIFI)
```

Single core throughout (`setup()`/`loop()` only, no `setup1()`/`loop1()`)
— matching `picoControl`'s proven pattern of running WiFi serving
alongside a timing-sensitive real-time task (there: motor control; here:
answering bus polls) entirely single-core, successfully. If RS485
responsiveness ever turns out to need tighter guarantees than this
round-robin can give it once this is on the bench and `ENABLE_WIFI` is
adding real load, moving `rs485Poll()` to a dedicated core is the
documented fallback, not built in from the start since it hasn't been
shown necessary.

## Pinout (this board)

```
RS485 (Serial1, hardware UART0):
  GP0 -> RS485 transceiver DI (TX)
  GP1 -> RS485 transceiver RO (RX)
  GP2 -> RS485 transceiver DE + /RE (tied together)

I2C, reserved for OLED (not wired in yet):
  GP4 -> SDA
  GP5 -> SCL

FarDriver (SerialPIO -- see note below):
  GP6 -> FarDriver RXD (TX from Pico)
  GP7 -> FarDriver TXD (RX to Pico)
```

**Why SerialPIO for FarDriver:** GP6/GP7 aren't hardware-UART-capable
pins on the RP2040 — UART0 lives on GP0/GP1 here (already used for
RS485), and UART1's pin options (GP4/GP5, GP8/GP9, GP20/21, GP24/25) don't
include GP6/GP7 either, and GP4/GP5 are reserved for I2C anyway. SerialPIO
is the arduino-pico core's PIO-state-machine-emulated UART — same
interface as a real Serial object, usable on any GPIO pair.

**Worth knowing before you're debugging on the bench:** SerialPIO is
generally solid but has at least one documented real-world report of data
corruption under sustained load
([earlephilhower/arduino-pico#2541](https://github.com/earlephilhower/arduino-pico/issues/2541),
a GPS module at 9600 baud). FarDriver runs at 19200 baud with a steady
~50 packets/sec — comparable, possibly heavier, load. If FarDriver
telemetry looks glitchy or CRC-fails often once `ENABLE_FARDRIVER` is on,
that issue is the first thing to check. The fallback would be freeing up
a real hardware UART1 pin pair (GP8/GP9) by moving the OLED to a
different I2C pin pair or `Wire1` instead of GP4/GP5.

## Files

```
platformio.ini          -- one environment
include/
  config.h              -- ENABLE_* feature flags, pins, baud rates, addresses, WiFi credentials
  state.h               -- shared BridgeState struct (always defined -- rs485.cpp needs it for battery data even with every flag off)
  protocol.h             -- pure protocol logic, zero Arduino dependency
  rs485.h                -- RS485 monitor + optional respond, takes Stream&
  fardriver_link.h       -- takes Stream& (compiled in only if ENABLE_FARDRIVER)
  webserver.h             -- (compiled in only if ENABLE_WIFI)
src/
  protocol.cpp            -- FarDriver CRC, Super Soco checksum, frame building -- TESTED, see below
  rs485.cpp               -- passive monitor (always) + panel-answering (#if ENABLE_RS485_RESPOND)
  fardriver_link.cpp      -- BT handshake, heartbeat, telemetry decode
  webserver.cpp           -- WiFi AP + status page, raw WiFiServer/WiFiClient (matches picoControl's WIFIremote.cpp pattern)
  main.cpp                -- single entry point, #if-guarded setup()/loop()
test/
  test_protocol.cpp       -- standalone test suite for protocol.cpp, plain g++
  arduino_stub/           -- minimal Arduino.h/WiFi.h/SerialPIO.h stand-ins used to
                             compile-check every .cpp file, in multiple flag
                             configurations, during development
```

`rs485.cpp` merges what used to be two separate files
(`rs485_monitor.cpp` + `supersoco_link.cpp`) — same underlying frame
parser either way, `#if ENABLE_RS485_RESPOND` adds the transmit path on
top rather than duplicating the parsing loop in a second file.

## Building

1. Open this folder in VSCode with the PlatformIO extension installed.
2. Edit `include/config.h` — leave the `ENABLE_*` flags at 0 for the
   first build, and change `WIFI_AP_PASSWORD` whenever you do turn
   `ENABLE_WIFI` on.
3. Build and Upload via the PlatformIO toolbar — same workflow as `picoControl`.

Uses the exact `platform`/`board`/`board_build.core`/`framework` lines
already proven working in your `picoControl` project.

## What's actually been tested

Being precise about this, since it matters for where to look first if something's wrong:

**Compiled and run with a real compiler (`test/test_protocol.cpp`, plain g++, 15 checks, all passing):**
- The FarDriver CRC, against two independently known-good real captured packets.
- The heartbeat command bytes and the confirmed real save command (syscmd `0x04`), against real captured sessions.
- FarDriver read-frame validation against a real captured frame (and confirmed it correctly *rejects* a corrupted one).
- The Super Soco checksum and full response frame construction — structure, addressing, checksum, terminator all verified byte-by-byte.
- The speed value round-trip through the panel's own `0.109` conversion factor.

**Compiled, linked, AND run against a stub covering the full API surface used by every file** — `Stream`/`HardwareSerial`/`SerialUART`, `SerialPIO`, `WiFi`/`WiFiServer`/`WiFiClient`/`IPAddress`, `String`. Every `.cpp` file compiles individually, and the complete project links into a runnable binary with `setup()`/`loop()` resolving correctly — checked in **four separate flag configurations**: all off (as shipped), `RESPOND` only, `RESPOND`+`FARDRIVER`, and all three on. All four compile, link, and run without crashing, and each one's startup output correctly reflects which flags are active. This specifically exists to catch `#if`-guard mistakes (a flag combination that compiles in isolation but breaks when combined with another) rather than just testing the two extremes.

This doesn't prove correct *behavior* on real hardware — the stub's `millis()`/`digitalWrite()`/etc. are no-ops, and WiFi/UART don't do anything real. What it rules out is the class of bugs that actually bit this project three times in a row so far: typos, wrong method names, type mismatches, a build-system mechanism that silently dropped a file. Real, if narrower, value than it might sound.

**Corrected against three real build failures (2026):**
1. `HardwareSerial` doesn't have `.setTX()/.setRX()` — those are only on the more specific `SerialUART` class. Fixed by having modules work against `Stream&` only, with pin assignment moved to `main.cpp` on the concrete `Serial1` object.
2. A `while (!Serial)` wait-for-USB pattern didn't compile (`HardwareSerial` has no `operator bool()` here) — added from general Arduino habit rather than checked against your own `picoControl` code, which doesn't do this. Removed.
3. The two-environment `build_src_filter` setup silently excluded `main_cli.cpp` from its own build. Replaced with the single-environment/feature-flag structure this README now describes.

Each time, the stub was corrected to actually *reproduce* the specific failure for the broken code before being trusted again — not just accept the fix.

**Not verified at all — no way to run this on real hardware or the actual Pico/PlatformIO toolchain from here:**
- Everything about actual runtime behavior on the Pico: real UART/SerialPIO timing, WiFi AP bring-up, RS485 transceiver DE/RE timing against real hardware. Expect genuine debugging here.
- The exact PDU field layout the panel expects (which byte is speed, temp, etc.) — sourced from the community `stprograms/SuperSoco485` library, not a confirmed live capture of *your* panel. Watch what the eavesdropped battery responses decode to and see if the numbers make sense (voltage/SoC especially, since those are easy to sanity-check by eye) — that's exactly what step 1 is for.

## Corrected against a real bus capture (2026-08)

The first real bench test (`ENABLE_RS485_RESPOND` still off, pure
monitoring) caught a genuine bug: `requests seen -> controller: 0,
battery: 0, other: 45+` — every request was being miscategorized.

Root cause: two wrong assumptions, both fixed now.

1. **The real bus master is `0xAA`, not `0xBA`.** Confirmed by a 47+
   frame capture with zero checksum failures — `0xAA` appears in *every*
   frame, `0xBA` in none. This contradicts an earlier note in this
   project's `context.md` ("the instrument panel (0xBA) is the bus
   master") — worth reconciling there too, but the real capture is what
   this code now trusts.
2. **Byte order within a telegram was backwards.** The
   `stprograms/SuperSoco485` library's `getSource()`/`getDestination()`
   method names suggested position 2 = source, position 3 = destination.
   The real data shows the opposite: a request is
   `[DST=target][SRC=master]`, a response is `[DST=master][SRC=responder]`.
   Confirmed against three real frames — a controller request, a battery
   request, and a battery response — all consistent, no exceptions.

Fixed in `config.h` (`ADDR_PANEL` corrected to `0xAA`), `protocol.cpp`
(`buildSuperSocoResponse` now writes `[dst][src]`, not `[src][dst]`), and
`rs485.cpp` (the "which device is this frame about" field is now
`DST` for requests and `SRC` for responses — they're genuinely different
fields, not the same one). `test_protocol.cpp` now includes your actual
captured bytes as regression tests — `buildSuperSocoResponse` reproduces
the real battery response frame byte-for-byte.

The decoded battery values in your capture (69V, 95% SoC, 24°C, 45
cycles) are all in completely plausible ranges for a real pack, and the
baud rate (9600) needed no changes — 47+ consecutive frames parsed with
zero checksum failures once the panel was actively polling. The dozen or
so `??? unrecognized byte` lines before that (uptime 54-87s) look like
ordinary startup noise, not a wiring problem — worth keeping an eye on
whether that count ever climbs again mid-session, but a one-time count
that stops growing isn't concerning on its own.

## Known gaps / next steps

Roughly in the order they'd naturally come up, matching the flag build-up above:

1. **Step 1 (flags all off)** — confirm RS485 wiring, valid frames decoding, battery numbers looking sane.
2. **Step 2 (`ENABLE_RS485_RESPOND`)** — confirm the panel accepts a response shape from the bridge at all.
3. **Step 3 (`+ ENABLE_FARDRIVER`)** — confirm real telemetry values look right, both in Serial output and (once wired) on the panel itself.
4. **Step 4 (`+ ENABLE_WIFI`)** — status page.
5. **OLED display**: `state.h`'s `BridgeState` struct is the single source every other module reads from, so an `oled_display.cpp` reading it directly (never writing), using the same `Adafruit_SSD1306`/`Adafruit_GFX` libraries already in `picoControl`, is a clean addition later. Uncomment the `lib_deps` lines in `platformio.ini` when you get there.
6. **Parking/kickstand signal**: not wired into the controller-response logic yet — currently inferred crudely from `motion`+`brake`.
7. **SerialPIO reliability**: worth specifically watching for once `ENABLE_FARDRIVER` is on, given the known issue linked above.