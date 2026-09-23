/**
 * main.cpp -- single entry point, one build, one environment.
 *
 * What actually runs is controlled by the ENABLE_* flags in config.h --
 * same pattern as picoControl's USE_OLED. RS485 passive monitoring
 * (battery eavesdrop, stats) always runs; the flags layer on additional
 * pieces one at a time:
 *
 *   ENABLE_RS485_RESPOND -- answer the panel's controller requests
 *   ENABLE_FARDRIVER      -- read real FarDriver telemetry over SerialPIO
 *   ENABLE_WIFI            -- WiFi AP + status web page
 *
 * View output via the PlatformIO Serial Monitor (115200 baud). Type
 * 'help' + Enter for the full command list -- covers per-frame print
 * mode (raw/decoded/quiet), an on-demand status summary, and (with
 * ENABLE_RS485_RESPOND on) a speed/gear override for testing the
 * panel's response to the bridge before FarDriver is wired up.
 */

#include <Arduino.h>
#include <cstring>
#include <cstdlib>
#include "config.h"
#include "state.h"
#include "rs485.h"

#if ENABLE_FARDRIVER
#include <SerialPIO.h>
#include "fardriver_link.h"
#endif

#if ENABLE_WIFI
#include "webserver.h"
#endif

BridgeState state;

#if ENABLE_FARDRIVER
SerialPIO fdSerial(FARDRIVER_TX_PIN, FARDRIVER_RX_PIN, FARDRIVER_PIO_FIFO);
#endif

#if ENABLE_WIFI
WebStatusServer webServer;
#endif

namespace {

uint32_t lastStatusMs = 0;
constexpr uint32_t STATUS_INTERVAL_MS = 3000;

char cmdBuf[64];
size_t cmdLen = 0;

void printHelp() {
    Serial.println("Commands:");
    Serial.println("  status     - print RS485 status summary now");
    Serial.println("  raw        - per-frame output: timestamp/type/raw hex only, no decode");
    Serial.println("  decoded    - per-frame output: raw + full interpretation (default)");
    Serial.println("  quiet      - suppress per-frame output entirely (status only)");
#if ENABLE_RS485_RESPOND
    Serial.println("  speed <N>  - override controller response speed in km/h, e.g. 'speed 45'");
    Serial.println("  gear <N>   - override controller response gear (0-3), e.g. 'gear 2'");
    Serial.println("  temp <N>   - override controller response temp in C, e.g. 'temp 25'");
    Serial.println("  auto       - clear override, use live FarDriver data (zero if ENABLE_FARDRIVER is off)");
#endif
    Serial.println("  help       - this list");
}

void handleCommand(char *line) {
    while (*line == ' ') line++;
    size_t len = strlen(line);
    while (len > 0 && (line[len - 1] == ' ' || line[len - 1] == '\r')) {
        line[--len] = '\0';
    }
    if (len == 0) return;

    char *space = strchr(line, ' ');
    char *arg = nullptr;
    if (space) {
        *space = '\0';
        arg = space + 1;
        while (*arg == ' ') arg++;
    }

    if (strcmp(line, "status") == 0) {
        rs485PrintStatus();

    } else if (strcmp(line, "raw") == 0) {
        rs485SetVerbosity(Verbosity::RAW);
        Serial.println("-> print mode: raw");

    } else if (strcmp(line, "decoded") == 0) {
        rs485SetVerbosity(Verbosity::DECODED);
        Serial.println("-> print mode: decoded");

    } else if (strcmp(line, "quiet") == 0) {
        rs485SetVerbosity(Verbosity::QUIET);
        Serial.println("-> print mode: quiet");

    } else if (strcmp(line, "help") == 0) {
        printHelp();

#if ENABLE_RS485_RESPOND
    } else if (strcmp(line, "speed") == 0) {
        if (arg && *arg) {
            state.test_speed_kmh = atof(arg);
            state.test_override_active = true;
            Serial.printf("-> TEST OVERRIDE: gear=%u speed=%.1fkm/h temp=%dC -- 'auto' to clear\n",
                          state.test_gear, state.test_speed_kmh, state.test_temp);
        } else {
            Serial.println("usage: speed <km/h>, e.g. 'speed 45'");
        }

    } else if (strcmp(line, "gear") == 0) {
        if (arg && *arg) {
            state.test_gear = (uint8_t)atoi(arg);
            state.test_override_active = true;
            Serial.printf("-> TEST OVERRIDE: gear=%u speed=%.1fkm/h temp=%dC -- 'auto' to clear\n",
                          state.test_gear, state.test_speed_kmh, state.test_temp);
        } else {
            Serial.println("usage: gear <0-3>, e.g. 'gear 2'");
        }

    } else if (strcmp(line, "temp") == 0) {
        if (arg && *arg) {
            state.test_temp = (int8_t)atoi(arg);
            state.test_override_active = true;
            Serial.printf("-> TEST OVERRIDE: gear=%u speed=%.1fkm/h temp=%dC -- 'auto' to clear\n",
                          state.test_gear, state.test_speed_kmh, state.test_temp);
        } else {
            Serial.println("usage: temp <C>, e.g. 'temp 25'");
        }

    } else if (strcmp(line, "auto") == 0) {
        state.test_override_active = false;
        Serial.println("-> test override cleared, using live data");
#endif

    } else {
        Serial.print("unknown command: ");
        Serial.println(line);
        Serial.println("type 'help' for the list");
    }
}

void pollCommands() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            if (cmdLen > 0) {
                cmdBuf[cmdLen] = '\0';
                handleCommand(cmdBuf);
                cmdLen = 0;
            }
        } else if (cmdLen < sizeof(cmdBuf) - 1) {
            cmdBuf[cmdLen++] = c;
        }
    }
}

}  // namespace

void setup() {
    Serial.begin(115200);
    // With every ENABLE_* flag off (the default first-test config),
    // nothing slow happens between Serial.begin() and the first print --
    // unlike picoControl, which naturally has a delay here from OLED/
    // motor/WiFi init. USB CDC needs a moment to enumerate on the host
    // side; without that natural delay, early prints can be silently
    // lost before a terminal has attached. This delay is standard,
    // ordinary Arduino practice (unlike the `while (!Serial)` pattern
    // removed earlier, which doesn't compile on this core at all) --
    // safe on every board, no dependency on any Serial-specific API.
    delay(1500);

    Serial.println();
    Serial.println("=== SuperSoco/FarDriver bridge ===");
    Serial.printf("RS485: Serial1 on GP%d(TX)/GP%d(RX), %u baud, DE=GP%d\n",
                  RS485_TX_PIN, RS485_RX_PIN, RS485_BAUD, RS485_DE_PIN);
    Serial.printf("Flags: ENABLE_RS485_RESPOND=%d ENABLE_FARDRIVER=%d ENABLE_WIFI=%d\n",
                  ENABLE_RS485_RESPOND, ENABLE_FARDRIVER, ENABLE_WIFI);
    Serial.println();
    printHelp();
    Serial.println();

    // Pin assignment and begin() happen here, on the concrete Serial1
    // object -- .setTX()/.setRX() are only declared on the specific
    // SerialUART class, not the generic HardwareSerial/Stream type
    // rs485.cpp deliberately works against.
    Serial1.setTX(RS485_TX_PIN);
    Serial1.setRX(RS485_RX_PIN);
    Serial1.begin(RS485_BAUD, SERIAL_8N1);
    rs485Init(Serial1);

#if ENABLE_FARDRIVER
    fdSerial.begin(FARDRIVER_BAUD);
    fardriverInit(fdSerial);
    Serial.println("FarDriver SerialPIO ready (GP6/GP7)");
#endif

#if ENABLE_WIFI
    webServer.begin();
    Serial.print("AP up: ");
    Serial.print(WIFI_AP_SSID);
    Serial.print(" -> ");
    Serial.println(webServer.ip());
#endif

    lastStatusMs = millis();
}

void loop() {
    rs485Poll(Serial1);

#if ENABLE_FARDRIVER
    fardriverPoll(fdSerial);
#endif

#if ENABLE_WIFI
    webServer.loop();
#endif

    pollCommands();

    // Only auto-print on a timer outside quiet mode -- this was the bug:
    // quiet suppressed per-frame output but not this periodic print, so
    // it wasn't actually quiet. Now 'status' is the only thing that
    // prints anything while in quiet mode.
    if (rs485GetVerbosity() != Verbosity::QUIET &&
        millis() - lastStatusMs >= STATUS_INTERVAL_MS) {
        rs485PrintStatus();
        lastStatusMs = millis();
    }
}