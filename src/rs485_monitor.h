#pragma once
/**
 * rs485_monitor.h -- passive, read-only RS485 bus monitor for the CLI
 * step. Sends NOTHING to the bus, ever -- same philosophy as
 * fardriver_silent_monitor.py: observe and decode first, before this
 * project starts actively answering the panel's requests.
 *
 * Prints two things to Serial (USB):
 *   - Every frame, raw hex AND (where the addressing is recognized)
 *     field-by-field interpreted.
 *   - A periodic status summary: frame counts, checksum failures, and
 *     the last decoded battery/controller-request activity.
 *
 * This stays in the project as a standalone diagnostic tool even once
 * main_bridge.cpp exists -- build the `picow_cli` PlatformIO environment
 * any time you want to just watch the bus without the bridge responding
 * to anything.
 *
 * Takes Stream& rather than a concrete serial type -- deliberately.
 * Pin assignment (.setTX()/.setRX()) and .begin() happen in main_cli.cpp
 * on the concrete Serial1 object directly, BEFORE calling rs485MonitorInit
 * -- those methods live on the specific SerialUART class, not on the
 * generic HardwareSerial/Stream base, so calling them through a generic
 * reference here doesn't compile. Keeping this module's own interface
 * generic (just available/read/write/flush) avoids needing to know or
 * care which concrete serial type is being used.
 */

#include <Arduino.h>

void rs485MonitorInit(Stream &uart);
void rs485MonitorPoll(Stream &uart);

// Call periodically (main_cli.cpp does this on a timer) to print the
// aggregated status summary.
void rs485MonitorPrintStatus();

