#include "rs485.h"
#include "protocol.h"
#include "config.h"
#include "state.h"
#include <cstring>
#include <cmath>

using namespace fdproto;

namespace {

uint8_t rxBuf[128];
size_t rxLen = 0;

// ---- Running stats for the periodic status summary ----
uint32_t statTotalFrames = 0;
uint32_t statChecksumFail = 0;
uint32_t statUnrecognizedBytes = 0;
uint32_t statReqController = 0;
uint32_t statRespController = 0;
uint32_t statReqBattery = 0;
uint32_t statRespBattery = 0;
uint32_t statOther = 0;

// Last OBSERVED controller response, diagnostic only -- distinct from
// `state`, which holds OUR OWN bridge's operational data. This tracks
// "what's on the bus" even when ENABLE_RS485_RESPOND is off and nothing
// (or the real stock controller, if it's still connected) is answering.
bool ctrlObservedKnown = false;
uint8_t ctrlObservedMode = 0;
int16_t ctrlObservedCurrentMa = 0, ctrlObservedSpeedRaw = 0;
int8_t ctrlObservedTemp = 0;
bool ctrlObservedParking = false;
uint32_t ctrlObservedLastSeenMs = 0;

Verbosity verbosity = Verbosity::DECODED;  // matches original always-on behavior

void consume(size_t n) {
    if (n >= rxLen) { rxLen = 0; return; }
    memmove(rxBuf, rxBuf + n, rxLen - n);
    rxLen -= n;
}

const char *addrName(uint8_t a) {
    if (a == ADDR_PANEL) return "panel";
    if (a == ADDR_CONTROLLER) return "controller";
    if (a == ADDR_BATTERY) return "battery";
    return "unknown";
}

void printHex(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        Serial.printf("%02X ", data[i]);
    }
}

// Decodes into `state` directly -- battery data is genuinely shared
// state (useful for the status page and a future OLED later), not just
// a monitor-mode display value. Always runs regardless of verbosity;
// only the printing (below, in the caller) is conditional.
bool storeBatteryFields(const uint8_t *pdu, uint8_t len) {
    if (len < 10) return false;
    state.batt_voltage = pdu[0];
    state.batt_soc = pdu[1];
    state.batt_temp = (int8_t)pdu[2];
    state.batt_charge_current = (int8_t)pdu[3];
    state.batt_cycles = (pdu[4] << 8) | pdu[5];
    state.batt_vbreaker = pdu[8];
    state.batt_activity = pdu[9];
    state.batt_connected = true;
    state.batt_last_packet_ms = millis();
    return true;
}

void printBatteryFields() {
    const char *actStr = (state.batt_activity == 1) ? "CHARGING"
                        : (state.batt_activity == 4) ? "DISCHARGING" : "idle/unknown";
    Serial.printf("V=%uV SoC=%u%% T=%dC I=%dA Cyc=%u VBreaker=0x%02X Act=%s",
                  state.batt_voltage, state.batt_soc, state.batt_temp,
                  state.batt_charge_current, state.batt_cycles,
                  state.batt_vbreaker, actStr);
}

// Diagnostic-only decode of an OBSERVED controller response -- not
// written to `state`, since (unlike battery) this isn't data the bridge
// itself produced or should treat as authoritative.
bool storeControllerFields(const uint8_t *pdu, uint8_t len) {
    if (len < 10) return false;
    ctrlObservedMode = pdu[0];
    ctrlObservedCurrentMa = (pdu[1] << 8) | pdu[2];
    ctrlObservedSpeedRaw = (pdu[3] << 8) | pdu[4];
    ctrlObservedTemp = (int8_t)pdu[5];
    ctrlObservedParking = pdu[8] != 0;
    ctrlObservedKnown = true;
    ctrlObservedLastSeenMs = millis();
    return true;
}

void printControllerFields() {
    // Same panel-side conversion factor used when building this field --
    // see context.md "Speed factor correction".
    float kmh = ctrlObservedSpeedRaw * 0.109f;
    Serial.printf("mode=%u I=%dmA speed_raw=%d(~%.1fkm/h) T=%dC parking=%s",
                  ctrlObservedMode, ctrlObservedCurrentMa, ctrlObservedSpeedRaw,
                  kmh, ctrlObservedTemp, ctrlObservedParking ? "yes" : "no");
}

#if ENABLE_RS485_RESPOND

// Build the 10-byte controller PDU. Normally from FarDriver-derived
// `state` (zeros/placeholder until ENABLE_FARDRIVER is also on) -- but
// if a CLI test override is active (see state.h, main.cpp's command
// parser), speed/gear come from that instead, so response-building and
// the panel's reaction to it can be tested before FarDriver is wired up.
void buildControllerPdu(uint8_t pdu[10]) {
    uint8_t mode;
    uint16_t speedRaw;
    bool motionForPdu;
    uint8_t temp;

    if (state.test_override_active) {
        mode = state.test_gear & 0xFF;
        motionForPdu = state.test_speed_kmh > 0.0f;
        speedRaw = motionForPdu ? (uint16_t)(state.test_speed_kmh / 0.109f) : 0;
        temp = (uint8_t)state.test_temp;
    } else {
        mode = state.gear & 0xFF;
        motionForPdu = state.motion;
        speedRaw = motionForPdu ? (uint16_t)(state.speed_kmh / 0.109f) : 0;
        // FarDriver-derived motor temp -- zero until ENABLE_FARDRIVER is
        // on and connected. NOT the battery's temperature; state.batt_temp
        // is a separate, independently-eavesdropped field this PDU
        // doesn't currently use at all (see README "temperature source"
        // note for the planned selectable battery/motor/driver option).
        temp = (uint8_t)(state.motor_temp & 0xFF);
    }

    uint16_t currentMa = (uint16_t)(fabsf(state.line_current) * 1000.0f);
    uint8_t parking = (!motionForPdu && state.brake) ? 1 : 0;

    pdu[0] = mode;
    pdu[1] = (currentMa >> 8) & 0xFF;
    pdu[2] = currentMa & 0xFF;
    pdu[3] = (speedRaw >> 8) & 0xFF;
    pdu[4] = speedRaw & 0xFF;
    pdu[5] = temp;
    pdu[6] = 0;
    pdu[7] = 0;
    pdu[8] = parking;
    pdu[9] = 0;
}

void sendFrame(Stream &uart, const uint8_t *frame, size_t len) {
    digitalWrite(RS485_DE_PIN, HIGH);
    // Real RS485 transceivers switch receive->transmit in nanoseconds to
    // low microseconds (typical driver-enable propagation delay on parts
    // like MAX485/MAX3485) -- far faster than a single UART bit period
    // (104us at 9600 baud), so no extra delay needed here before write().
    uart.write(frame, len);
    uart.flush();
    // CORRECTED (2026): the previous fixed 200us margin here was less
    // than one-fifth of a single byte time at this baud rate. flush()
    // is only guaranteed to mean the software-visible FIFO is empty
    // (bytes handed to the UART hardware) -- not necessarily that the
    // very last bit (including the stop bit) has actually finished
    // shifting out onto the wire. With too little margin, dropping DE
    // early can clip the trailing byte(s) of the frame -- for us,
    // specifically the checksum and terminator, the two bytes that
    // matter most for the receiver to accept the frame at all. This is
    // the leading hypothesis for "gear stuck at 0 / speed not taking
    // effect": a frame with a corrupted tail gets silently rejected by
    // the panel, which falls back to a default display -- indistinguishable
    // from a firmware logic bug, even though the gear/speed injection
    // logic itself was already confirmed correct in simulation.
    // Margin is now 2 full byte-times at the configured baud, derived
    // from RS485_BAUD rather than a guessed constant -- still negligible
    // against the ~100-240ms polling cycle observed on the real bus.
    constexpr uint32_t BITS_PER_BYTE = 10;  // 8N1: start + 8 data + stop
    uint32_t byteTimeUs = (1000000UL * BITS_PER_BYTE) / RS485_BAUD;
    delayMicroseconds(byteTimeUs * 2);
    digitalWrite(RS485_DE_PIN, LOW);
}

#endif  // ENABLE_RS485_RESPOND

}  // namespace

void rs485Init(Stream &uart) {
    (void)uart;
    pinMode(RS485_DE_PIN, OUTPUT);
    digitalWrite(RS485_DE_PIN, LOW);  // always start receive-only
    rxLen = 0;
#if ENABLE_RS485_RESPOND
    Serial.println("RS485: monitoring AND actively answering controller (0xDA) requests.");
#else
    Serial.println("RS485: passive monitor only, never transmits.");
#endif
}

void rs485Poll(Stream &uart) {
    while (uart.available() && rxLen < sizeof(rxBuf)) {
        rxBuf[rxLen++] = (uint8_t)uart.read();
    }

    bool progressed = true;
    while (progressed) {
        progressed = false;
        if (rxLen < 2) break;

        bool isRequest = (rxBuf[0] == SS_REQ_1 && rxBuf[1] == SS_REQ_2);
        bool isResponse = (rxBuf[0] == SS_RESP_1 && rxBuf[1] == SS_RESP_2);

        if (!isRequest && !isResponse) {
            if (verbosity != Verbosity::QUIET) {
                Serial.printf("[%lu] ???   unrecognized byte                         | raw: %02X\n",
                              millis(), rxBuf[0]);
            }
            statUnrecognizedBytes++;
            consume(1);
            progressed = true;
            continue;
        }

        if (rxLen < 5) break;  // need DST,SRC,LEN at minimum
        uint8_t length = rxBuf[4];
        size_t total = 5 + length + 2;
        if (total > sizeof(rxBuf)) {
            if (verbosity != Verbosity::QUIET) {
                Serial.printf("[%lu] ???   implausible length byte (%u), resyncing | raw: %02X %02X\n",
                              millis(), length, rxBuf[0], rxBuf[1]);
            }
            consume(2);
            progressed = true;
            continue;
        }
        if (rxLen < total) break;  // wait for the rest of the frame

        // CORRECTED (2026) against a real capture: position 2 is
        // DESTINATION, position 3 is SOURCE. A request is addressed TO
        // the device being asked about ([DST=target][SRC=master]); a
        // response is addressed TO the master, FROM the responder
        // ([DST=master][SRC=responder]). So the "device this frame is
        // about" is DST for requests, SRC for responses -- not the same
        // field both times. See config.h's ADDR_PANEL comment.
        uint8_t frameDst = rxBuf[2];
        uint8_t frameSrc = rxBuf[3];
        uint8_t subject = isRequest ? frameDst : frameSrc;
        const uint8_t *pdu = rxBuf + 5;
        uint8_t chk = rxBuf[5 + length];
        uint8_t term = rxBuf[5 + length + 1];
        bool checksumOk = superSocoChecksum(length, pdu, length) == chk;
        bool termOk = (term == SS_TERMINATOR);

        statTotalFrames++;

        if (!checksumOk || !termOk) {
            statChecksumFail++;
            if (verbosity != Verbosity::QUIET) {
                Serial.printf("[%lu] %s BAD  subject=0x%02X(%s) len=%u %s%s | raw: ",
                              millis(), isRequest ? "REQ " : "RESP",
                              subject, addrName(subject), length,
                              !checksumOk ? "[checksum fail] " : "",
                              !termOk ? "[bad terminator] " : "");
                printHex(rxBuf, total);
                Serial.println();
            }
            consume(total);
            progressed = true;
            continue;
        }

        if (verbosity != Verbosity::QUIET) {
            Serial.printf("[%lu] %s      subject=0x%02X(%s) len=%u  ",
                          millis(), isRequest ? "REQ " : "RESP", subject, addrName(subject), length);
        }

        if (isRequest) {
            if (subject == ADDR_CONTROLLER) {
                statReqController++;
                state.panel_requests_seen++;
                state.panel_last_request_ms = millis();
                if (verbosity == Verbosity::DECODED) {
                    Serial.print("(request, no PDU fields to decode)");
                }

#if ENABLE_RS485_RESPOND
                uint8_t pduOut[10];
                buildControllerPdu(pduOut);
                uint8_t frame[19];
                size_t frameLen = buildSuperSocoResponse(ADDR_CONTROLLER, ADDR_PANEL,
                                                           pduOut, 10, frame);
                if (verbosity != Verbosity::QUIET) {
                    Serial.print("  -> TX: ");
                    printHex(frame, frameLen);
                }
                sendFrame(uart, frame, frameLen);
                state.panel_responses_sent++;
                statRespController++;
#endif
            } else if (subject == ADDR_BATTERY) {
                statReqBattery++;
                if (verbosity == Verbosity::DECODED) {
                    Serial.print("(request, no PDU fields to decode)");
                }
            } else {
                statOther++;
                if (verbosity == Verbosity::DECODED) {
                    Serial.print("(request, no PDU fields to decode)");
                }
            }
        } else {
            if (subject == ADDR_CONTROLLER) {
#if !ENABLE_RS485_RESPOND
                // Only counted here in passive mode -- when responding
                // ourselves, statRespController is counted at send time
                // above instead, and this branch would just be our own
                // echo anyway on a real bus (we wouldn't see our own TX
                // as a separate RX event in the usual half-duplex case,
                // but the check costs nothing either way).
                statRespController++;
#endif
                storeControllerFields(pdu, length);
                if (verbosity == Verbosity::DECODED) printControllerFields();
            } else if (subject == ADDR_BATTERY) {
                statRespBattery++;
                storeBatteryFields(pdu, length);
                if (verbosity == Verbosity::DECODED) printBatteryFields();
            } else {
                statOther++;
                if (verbosity == Verbosity::DECODED) {
                    Serial.print("(unrecognized address, raw PDU only)");
                }
            }
        }

        if (verbosity != Verbosity::QUIET) {
            Serial.print("  | raw: ");
            printHex(rxBuf, total);
            Serial.println();
        }

        consume(total);
        progressed = true;
    }
}

void rs485SetVerbosity(Verbosity v) { verbosity = v; }
Verbosity rs485GetVerbosity() { return verbosity; }
const char *rs485VerbosityName(Verbosity v) {
    switch (v) {
        case Verbosity::QUIET:   return "quiet";
        case Verbosity::RAW:     return "raw";
        case Verbosity::DECODED: return "decoded";
    }
    return "?";
}

void rs485PrintStatus() {
    Serial.println("---- RS485 status ----");
    Serial.printf("  uptime: %lus   print mode: %s\n",
                  millis() / 1000, rs485VerbosityName(verbosity));
    Serial.printf("  frames seen: %lu   checksum/terminator failures: %lu   unrecognized bytes: %lu\n",
                  statTotalFrames, statChecksumFail, statUnrecognizedBytes);
    Serial.printf("  requests seen  -> controller: %lu   battery: %lu   other: %lu\n",
                  statReqController, statReqBattery, statOther);
    Serial.printf("  responses seen -> controller: %lu   battery: %lu\n",
                  statRespController, statRespBattery);

#if ENABLE_RS485_RESPOND
    Serial.printf("  our own responses sent: %lu\n", (unsigned long)state.panel_responses_sent);
    if (state.test_override_active) {
        Serial.printf("  TEST OVERRIDE active: gear=%u speed=%.1fkm/h temp=%dC "
                      "(type 'auto' to use live FarDriver data instead)\n",
                      state.test_gear, state.test_speed_kmh, state.test_temp);
    }
#else
    if (statReqController > 0 && statRespController == 0) {
        Serial.println("  NOTE: panel is asking for the controller (0xDA) but nothing is "
                        "answering yet -- expected with ENABLE_RS485_RESPOND off.");
    }
#endif

    if (state.batt_connected) {
        Serial.printf("  battery (last seen %lus ago): V=%uV SoC=%u%% T=%dC I=%dA Cyc=%u Act=0x%02X\n",
                      (millis() - state.batt_last_packet_ms) / 1000,
                      state.batt_voltage, state.batt_soc, state.batt_temp,
                      state.batt_charge_current, state.batt_cycles, state.batt_activity);
    } else {
        Serial.println("  battery: no valid response decoded yet");
    }

    if (ctrlObservedKnown) {
        Serial.printf("  controller (observed on bus, last seen %lus ago): "
                      "mode=%u I=%dmA speed_raw=%d T=%dC parking=%s\n",
                      (millis() - ctrlObservedLastSeenMs) / 1000,
                      ctrlObservedMode, ctrlObservedCurrentMa, ctrlObservedSpeedRaw,
                      ctrlObservedTemp, ctrlObservedParking ? "yes" : "no");
    } else {
        Serial.println("  controller: no valid response observed yet");
    }
    Serial.println("-----------------------");
}