#include "rs485_monitor.h"
#include "protocol.h"
#include "config.h"
#include <cstring>

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

// Last decoded battery status (from eavesdropped RESPONSE frames)
bool battKnown = false;
uint8_t battVoltage = 0, battSoc = 0, battVbreaker = 0, battActivity = 0;
int8_t battTemp = 0, battCharge = 0;
uint16_t battCycles = 0;
uint32_t battLastSeenMs = 0;

// Last decoded controller status (only populated if something -- the real
// stock controller, or a bridge in a later build -- actually answers;
// expected to stay unknown during this purely-passive step unless the
// stock controller happens to still be present)
bool ctrlKnown = false;
uint8_t ctrlMode = 0;
int16_t ctrlCurrentMa = 0, ctrlSpeedRaw = 0;
int8_t ctrlTemp = 0;
bool ctrlParking = false;
uint32_t ctrlLastSeenMs = 0;

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

void printBatteryFields(const uint8_t *pdu, uint8_t len) {
    if (len < 10) { Serial.print("(pdu too short to decode)"); return; }
    uint8_t v = pdu[0], soc = pdu[1];
    int8_t t = (int8_t)pdu[2], c = (int8_t)pdu[3];
    uint16_t cyc = (pdu[4] << 8) | pdu[5];
    uint8_t vbrk = pdu[8], act = pdu[9];
    const char *actStr = (act == 1) ? "CHARGING" : (act == 4) ? "DISCHARGING" : "idle/unknown";
    Serial.printf("V=%uV SoC=%u%% T=%dC I=%dA Cyc=%u VBreaker=0x%02X Act=%s",
                  v, soc, t, c, cyc, vbrk, actStr);

    battKnown = true;
    battVoltage = v; battSoc = soc; battTemp = t; battCharge = c;
    battCycles = cyc; battVbreaker = vbrk; battActivity = act;
    battLastSeenMs = millis();
}

void printControllerFields(const uint8_t *pdu, uint8_t len) {
    if (len < 10) { Serial.print("(pdu too short to decode)"); return; }
    uint8_t mode = pdu[0];
    int16_t currentMa = (pdu[1] << 8) | pdu[2];
    int16_t speedRaw = (pdu[3] << 8) | pdu[4];
    int8_t temp = (int8_t)pdu[5];
    bool parking = pdu[8] != 0;
    // Same panel-side conversion factor as supersoco_link.cpp uses when
    // building this field -- see context.md "Speed factor correction".
    float kmh = speedRaw * 0.109f;
    Serial.printf("mode=%u I=%dmA speed_raw=%d(~%.1fkm/h) T=%dC parking=%s",
                  mode, currentMa, speedRaw, kmh, temp, parking ? "yes" : "no");

    ctrlKnown = true;
    ctrlMode = mode; ctrlCurrentMa = currentMa; ctrlSpeedRaw = speedRaw;
    ctrlTemp = temp; ctrlParking = parking;
    ctrlLastSeenMs = millis();
}

}  // namespace

void rs485MonitorInit(Stream &uart) {
    // Pin assignment (.setTX()/.setRX()) and .begin() happen in
    // main_cli.cpp on the concrete Serial1 object, before this is called
    // -- see the header comment for why. Nothing to configure here beyond
    // resetting our own parse state.
    (void)uart;
    // Deliberately NOT configuring RS485_DE_PIN at all -- this monitor
    // never transmits, so the transceiver's DE/RE should just be left low
    // (receive-only) for the whole session. If you've got DE wired to a
    // GPIO that defaults high on boot on your transceiver board, pull it
    // low externally (or add a pinMode/digitalWrite(LOW) here) so the
    // bridge doesn't accidentally drive the bus during this passive step.
    rxLen = 0;
    Serial.println("RS485 monitor: passive, read-only, never transmits.");
}

void rs485MonitorPoll(Stream &uart) {
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
            Serial.printf("[%lu] ???   unrecognized byte                         | raw: %02X\n",
                          millis(), rxBuf[0]);
            statUnrecognizedBytes++;
            consume(1);
            progressed = true;
            continue;
        }

        if (rxLen < 5) break;  // need SRC,DST,LEN at minimum
        uint8_t length = rxBuf[4];
        size_t total = 5 + length + 2;
        if (total > sizeof(rxBuf)) {
            // Implausible length -- not a real frame, resync past the marker
            Serial.printf("[%lu] ???   implausible length byte (%u), resyncing | raw: %02X %02X\n",
                          millis(), length, rxBuf[0], rxBuf[1]);
            consume(2);
            progressed = true;
            continue;
        }
        if (rxLen < total) break;  // wait for the rest of the frame

        uint8_t src = rxBuf[2], dst = rxBuf[3];
        const uint8_t *pdu = rxBuf + 5;
        uint8_t chk = rxBuf[5 + length];
        uint8_t term = rxBuf[5 + length + 1];
        bool checksumOk = superSocoChecksum(length, pdu, length) == chk;
        bool termOk = (term == SS_TERMINATOR);

        statTotalFrames++;

        if (!checksumOk || !termOk) {
            statChecksumFail++;
            Serial.printf("[%lu] %s BAD  src=0x%02X(%s) dst=0x%02X(%s) len=%u %s%s | raw: ",
                          millis(), isRequest ? "REQ " : "RESP",
                          src, addrName(src), dst, addrName(dst), length,
                          !checksumOk ? "[checksum fail] " : "",
                          !termOk ? "[bad terminator] " : "");
            printHex(rxBuf, total);
            Serial.println();
            consume(total);
            progressed = true;
            continue;
        }

        Serial.printf("[%lu] %s      src=0x%02X(%s) dst=0x%02X(%s) len=%u  ",
                      millis(), isRequest ? "REQ " : "RESP",
                      src, addrName(src), dst, addrName(dst), length);

        if (isRequest) {
            if (dst == ADDR_CONTROLLER) statReqController++;
            else if (dst == ADDR_BATTERY) statReqBattery++;
            else statOther++;
            Serial.print("(request, no PDU fields to decode)");
        } else {
            if (dst == ADDR_CONTROLLER) {
                statRespController++;
                printControllerFields(pdu, length);
            } else if (dst == ADDR_BATTERY) {
                statRespBattery++;
                printBatteryFields(pdu, length);
            } else {
                statOther++;
                Serial.print("(unrecognized address, raw PDU only)");
            }
        }

        Serial.print("  | raw: ");
        printHex(rxBuf, total);
        Serial.println();

        consume(total);
        progressed = true;
    }
}

void rs485MonitorPrintStatus() {
    Serial.println("---- RS485 monitor status ----");
    Serial.printf("  uptime: %lus\n", millis() / 1000);
    Serial.printf("  frames seen: %lu   checksum/terminator failures: %lu   unrecognized bytes: %lu\n",
                  statTotalFrames, statChecksumFail, statUnrecognizedBytes);
    Serial.printf("  requests seen  -> controller: %lu   battery: %lu   other: %lu\n",
                  statReqController, statReqBattery, statOther);
    Serial.printf("  responses seen -> controller: %lu   battery: %lu\n",
                  statRespController, statRespBattery);

    if (statReqController > 0 && statRespController == 0) {
        Serial.println("  NOTE: panel is asking for the controller (0xDA) but nothing is "
                        "answering yet -- expected in this passive-only step.");
    }

    if (battKnown) {
        Serial.printf("  battery (last seen %lus ago): V=%uV SoC=%u%% T=%dC I=%dA Cyc=%u Act=0x%02X\n",
                      (millis() - battLastSeenMs) / 1000,
                      battVoltage, battSoc, battTemp, battCharge, battCycles, battActivity);
    } else {
        Serial.println("  battery: no valid response decoded yet");
    }

    if (ctrlKnown) {
        Serial.printf("  controller (last seen %lus ago): mode=%u I=%dmA speed_raw=%d T=%dC parking=%s\n",
                      (millis() - ctrlLastSeenMs) / 1000,
                      ctrlMode, ctrlCurrentMa, ctrlSpeedRaw, ctrlTemp, ctrlParking ? "yes" : "no");
    } else {
        Serial.println("  controller: no valid response decoded yet (none expected until "
                        "something actually answers as 0xDA)");
    }
    Serial.println("-------------------------------");
}
