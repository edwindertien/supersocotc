#pragma once
/**
 * config.h -- all the knobs in one place.
 *
 * Feature flags -- enable incrementally to test one piece at a time
 * --------------------------------------------------------------------
 * Same pattern as picoControl's USE_OLED etc: one main.cpp, one build,
 * #if-guarded blocks turn pieces on/off. No more separate environments/
 * build_src_filter -- that mechanism failed twice in a row on real
 * builds and added complexity for no real benefit over just... not
 * compiling code you haven't enabled yet.
 *
 * Suggested order to bring things up:
 *   1. Leave everything below at 0. RS485 passive monitoring (raw +
 *      interpreted printing, battery eavesdrop) always runs regardless
 *      of these flags -- it's the foundational diagnostic, not optional.
 *   2. ENABLE_RS485_RESPOND 1 -- start answering the panel's requests
 *      for the controller. FarDriver's still off, so responses will
 *      carry placeholder/zero data -- this step is about confirming the
 *      panel accepts a response shape at all, not correct values yet.
 *   3. ENABLE_FARDRIVER 1 -- real telemetry now flows into those
 *      responses instead of zeros.
 *   4. ENABLE_WIFI 1 -- status page comes up alongside everything else.
 */
#define ENABLE_RS485_RESPOND 1   // actively answer the panel's controller (0xDA) requests
#define ENABLE_FARDRIVER     0   // read real FarDriver telemetry over SerialPIO
#define ENABLE_WIFI          0   // WiFi AP + status web page

/**
 * Wiring (this specific board's pinout)
 * --------------------------------------
 * RS485 bus (Super Soco instrument panel <-> battery -- tap in parallel
 * with the existing wiring, do not break the existing panel<->battery
 * link). Hardware UART0 (Serial1):
 *     Pico GP0 (Serial1 TX) -> RS485 transceiver DI
 *     Pico GP1 (Serial1 RX) -> RS485 transceiver RO
 *     Pico GP2              -> RS485 transceiver DE + /RE tied together
 *     RS485 transceiver A/B -> bus A/B (swap if data looks garbled)
 *
 * I2C, reserved for the OLED (not wired in by this delivery yet):
 *     Pico GP4 -> OLED SDA
 *     Pico GP5 -> OLED SCL
 *
 * FarDriver TTL header ("USB" header, 3.3V logic -- do NOT connect Pin 1 /
 * 3.3V supply). GP6/GP7 aren't hardware-UART-capable pins on the RP2040
 * (UART0 lives on GP0/GP1 here, already used for RS485; UART1's pin
 * options are GP4/GP5, GP8/GP9, GP20/21, GP24/25 -- none of which are
 * free given the I2C reservation), so this uses SerialPIO -- a PIO-state-
 * machine-emulated UART, usable on any GPIO pair, same interface as a
 * real Serial object:
 *     Pico GP6 (SerialPIO TX) -> FarDriver RXD
 *     Pico GP7 (SerialPIO RX) -> FarDriver TXD
 *     GND <-> GND
 *
 * Worth knowing: SerialPIO is generally solid but has at least one
 * documented report of data corruption under sustained load
 * (earlephilhower/arduino-pico#2541, GPS module at 9600 baud). FarDriver
 * runs at 19200 baud with a steady ~50 packets/sec -- comparable load.
 * If FarDriver telemetry looks glitchy/CRC-fails often once this is on
 * the bench, that issue is the first thing to check; the fallback would
 * be freeing up a real hardware UART1 pin pair (GP8/GP9) by moving the
 * OLED to Wire1 or a different pin pair instead of GP4/GP5.
 */

// ---- RS485 link (hardware UART0 = Serial1) ----
#define RS485_TX_PIN 0
#define RS485_RX_PIN 1
#define RS485_DE_PIN 2            // transceiver DE/RE tied together, HIGH = transmit
#define RS485_BAUD   9600         // confirmed against stprograms/SuperSoco485

// ---- I2C, reserved for OLED (not used by this delivery yet) ----
#define OLED_SDA_PIN 4
#define OLED_SCL_PIN 5

// ---- FarDriver TTL link (SerialPIO, since GP6/GP7 aren't hardware UART pins) ----
#define FARDRIVER_TX_PIN 6
#define FARDRIVER_RX_PIN 7
#define FARDRIVER_BAUD   19200    // confirmed this session -- see context.md
#define FARDRIVER_PIO_FIFO 64     // a bit more headroom than the 32-byte default

// Bus addresses. CORRECTED against a real capture (2026) -- the master
// is 0xAA, not 0xBA as context.md's earlier note said. 0xAA appears
// constant across every request AND response in a real 47+ frame
// capture with zero checksum failures; 0xBA never appears at all.
// Byte order within a telegram was ALSO corrected against that same
// capture: position 2 is DESTINATION, position 3 is SOURCE -- opposite
// of what the stprograms/SuperSoco485 library's getSource()/
// getDestination() method NAMES suggested. A request is
// [DST=target][SRC=master]; a response is [DST=master][SRC=responder].
// Trust this over both the library's naming and the older context.md
// note -- this is checked against real bytes off the actual bus.
#define ADDR_PANEL      0xAA      // instrument panel, bus master (was 0xBA -- wrong)
#define ADDR_CONTROLLER 0xDA      // what we impersonate
#define ADDR_BATTERY    0x5A      // what we eavesdrop on, never transmit to

// Super Soco telegram type markers (stprograms/SuperSoco485, TelegramParser.h)
#define SS_REQ_1 0xC5
#define SS_REQ_2 0x5C
#define SS_RESP_1 0xB6
#define SS_RESP_2 0x6B
#define SS_TERMINATOR 0x0D

// ---- WiFi AP ----
#define WIFI_AP_SSID     "SuperSoco-Bridge"
#define WIFI_AP_PASSWORD "supersoco"   // 8+ chars required for WPA2; change this
#define WIFI_AP_CHANNEL  6
#define WIFI_AP_HIDDEN   false

// ---- Web server ----
#define WEB_PORT 80

// ---- Misc ----
#define HEARTBEAT_INTERVAL_MS 1000    // matches the real FarDriver app's own cadence
#define STALE_DATA_TIMEOUT_MS 5000    // no packets for this long -> flag stale