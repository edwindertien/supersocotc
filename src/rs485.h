#pragma once
/**
 * rs485.h -- the Super Soco RS485 side of the bridge, all in one module.
 *
 * Always does (regardless of any flag):
 *   - Passive frame parsing, raw hex + field-interpreted printing to Serial.
 *   - Decodes eavesdropped BATTERY (0x5A) responses into `state`.
 *   - Tracks/prints a periodic status summary.
 *
 * Additionally, if ENABLE_RS485_RESPOND (config.h) is set:
 *   - Answers the panel's requests for the CONTROLLER (0xDA) using
 *     FarDriver-derived data from `state` (zeros/placeholder until
 *     ENABLE_FARDRIVER is also on).
 *
 * This replaces the earlier two-file split (rs485_monitor.cpp +
 * supersoco_link.cpp) and the two-PlatformIO-environment structure that
 * went with it -- both were more moving parts than the actual problem
 * needed, and the environment-switching mechanism (build_src_filter)
 * turned out to be a real source of build failures with no upside over
 * just not compiling code you haven't enabled yet. One set of sources,
 * one environment, #if-guarded blocks -- same pattern as picoControl's
 * USE_OLED.
 *
 * Takes Stream& rather than a concrete serial type -- pin assignment
 * (.setTX()/.setRX()) and .begin() happen in main.cpp on the concrete
 * Serial1 object, before calling rs485Init. See config.h's comment on
 * why (SerialUART vs. the generic HardwareSerial/Stream base).
 *
 * Protocol details (envelope, checksum, addressing, PDU layout) verified
 * against the real stprograms/SuperSoco485 Arduino library source, not
 * just its README -- see this project's context.md.
 *
 * Per-frame printing is controlled by a verbosity mode, settable via
 * the serial CLI (see main.cpp):
 *   QUIET   -- no per-frame printing, only stats/status
 *   RAW     -- timestamp + raw hex only, no decoded interpretation
 *   DECODED -- raw hex + full field-level interpretation (the original,
 *              always-on behavior from earlier in this project)
 * Defaults to DECODED, matching what this project has printed all along.
 */

#include <Arduino.h>

enum class Verbosity { QUIET, RAW, DECODED };

void rs485Init(Stream &uart);
void rs485Poll(Stream &uart);
void rs485PrintStatus();
void rs485SetVerbosity(Verbosity v);
Verbosity rs485GetVerbosity();
const char *rs485VerbosityName(Verbosity v);