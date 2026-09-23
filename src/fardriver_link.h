#pragma once
/**
 * fardriver_link.h -- talks to the FarDriver over TTL serial exactly the
 * way fardriver_web.py does (same CRC, same captured BT-handshake bytes,
 * same heartbeat), but read-only: this bridge only ever consumes
 * telemetry, it never writes settings. Protocol logic itself lives in
 * protocol.cpp/h (tested standalone); this file is just the serial glue
 * around it.
 *
 * Takes a Stream& rather than a concrete HardwareSerial/SerialPIO type --
 * both implement the same Stream/Print interface (available/read/write),
 * so this works unchanged whichever one is passed in. On this board it's
 * SerialPIO on GP6/GP7 (see config.h for why).
 */

#include <Arduino.h>

// Call once from setup(): configures the given serial port's baud
// (pin assignment for SerialPIO happens at construction in main.cpp).
void fardriverInit(Stream &uart);

// Call every loop() iteration: drains available bytes, runs the BT
// handshake state machine, decodes telemetry into `state`, and sends the
// heartbeat on its own schedule.
void fardriverPoll(Stream &uart);
