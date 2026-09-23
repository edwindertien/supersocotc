#pragma once
/**
 * supersoco_link.h -- the Super Soco RS485 side of the bridge.
 *
 * Two jobs on one shared bus:
 *   1. Answer the instrument panel's requests for the CONTROLLER (0xDA)
 *      using FarDriver-derived data from `state`.
 *   2. Passively eavesdrop the BATTERY's (0x5A) responses to the panel's
 *      own requests -- never transmit for these, just read and record.
 *
 * Runs on hardware UART0 (Serial1, GP0/GP1) with DE/RE on GP2.
 *
 * Protocol details (envelope, checksum, addressing, PDU layout) are in
 * this project's README and context.md -- verified against the real
 * stprograms/SuperSoco485 Arduino library source, not just its README.
 *
 * Takes Stream& rather than a concrete serial type -- same reasoning as
 * rs485_monitor.h: .setTX()/.setRX() only exist on the specific
 * SerialUART class, not the generic HardwareSerial/Stream base, so pin
 * assignment and .begin() happen in main_bridge.cpp on the concrete
 * Serial1 object, before calling supersocoInit.
 */

#include <Arduino.h>

void supersocoInit(Stream &uart);
void supersocoPoll(Stream &uart);

