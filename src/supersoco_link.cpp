#include "supersoco_link.h"
#include "protocol.h"
#include "state.h"
#include "config.h"
#include <cstring>
#include <cmath>

using namespace fdproto;

namespace {

uint8_t rxBuf[128];
size_t rxLen = 0;

void consume(size_t n) {
    if (n >= rxLen) { rxLen = 0; return; }
    memmove(rxBuf, rxBuf + n, rxLen - n);
    rxLen -= n;
}

// Build the 10-byte controller PDU from current FarDriver-derived state.
void buildControllerPdu(uint8_t pdu[10]) {
    uint8_t mode = state.gear & 0xFF;
    // SuperSoco485's ECUStatus current is in mA -- FarDriver's line_current
    // is in Amps; scale up. Sign/magnitude not critical for the dashboard,
    // panel mostly cares about speed/gear/temp/parking.
    uint16_t currentMa = (uint16_t)(fabsf(state.line_current) * 1000.0f);

    // The panel expects a RAW value in the STOCK controller's own scale,
    // not real km/h -- it applies its own internal conversion on receipt.
    // This project already empirically derived that conversion factor
    // (0.109, from comparing the stock controller's own RS485 reports
    // against a real speedometer -- see context.md "Speed factor
    // correction"). That's a DIFFERENT constant from the FarDriver-side
    // one (0.0109, unrelated protocol) -- don't mix them up.
    uint16_t speedRaw = state.motion ? (uint16_t)(state.speed_kmh / 0.109f) : 0;

    uint8_t temp = (uint8_t)(state.motor_temp & 0xFF);
    uint8_t parking = (!state.motion && state.brake) ? 1 : 0;

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

// Decode an eavesdropped battery PDU straight into state -- never
// transmitted anywhere, purely observational.
void parseBatteryPdu(const uint8_t *pdu, uint8_t len) {
    if (len < 10) return;
    state.batt_voltage = pdu[0];
    state.batt_soc = pdu[1];
    state.batt_temp = (int8_t)pdu[2];
    state.batt_charge_current = (int8_t)pdu[3];
    state.batt_cycles = (pdu[4] << 8) | pdu[5];
    state.batt_vbreaker = pdu[8];
    state.batt_activity = pdu[9];
    state.batt_connected = true;
    state.batt_last_packet_ms = millis();
}

void sendFrame(Stream &uart, const uint8_t *frame, size_t len) {
    digitalWrite(RS485_DE_PIN, HIGH);
    uart.write(frame, len);
    uart.flush();  // blocks until the hardware FIFO has actually drained
    // Small fixed safety margin beyond flush() for the last stop bit /
    // transceiver propagation delay before dropping DE.
    delayMicroseconds(200);
    digitalWrite(RS485_DE_PIN, LOW);
}

}  // namespace

void supersocoInit(Stream &uart) {
    // Pin assignment (.setTX()/.setRX()) and .begin() happen in
    // main_bridge.cpp on the concrete Serial1 object, before this is
    // called -- see the header comment for why. This only handles the DE
    // pin (a plain GPIO, unrelated to the serial object's concrete type)
    // and resets our own parse state.
    (void)uart;
    pinMode(RS485_DE_PIN, OUTPUT);
    digitalWrite(RS485_DE_PIN, LOW);  // start in receive mode
    rxLen = 0;
}

void supersocoPoll(Stream &uart) {
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
            consume(1);
            progressed = true;
            continue;
        }

        if (rxLen < 5) break;  // need SRC,DST,LEN at minimum
        uint8_t length = rxBuf[4];
        size_t total = 5 + length + 2;  // header(5) + pdu + checksum + terminator
        if (rxLen < total) break;       // wait for more bytes

        uint8_t dst = rxBuf[3];
        const uint8_t *pdu = rxBuf + 5;
        uint8_t chk = rxBuf[5 + length];
        uint8_t term = rxBuf[5 + length + 1];

        bool valid = (term == SS_TERMINATOR) &&
                     (superSocoChecksum(length, pdu, length) == chk);

        if (valid && isRequest && dst == ADDR_CONTROLLER) {
            state.panel_requests_seen++;
            state.panel_last_request_ms = millis();

            uint8_t pduOut[10];
            buildControllerPdu(pduOut);
            uint8_t frame[19];
            size_t frameLen = buildSuperSocoResponse(ADDR_PANEL, ADDR_CONTROLLER,
                                                       pduOut, 10, frame);
            sendFrame(uart, frame, frameLen);
            state.panel_responses_sent++;

        } else if (valid && isResponse && dst == ADDR_BATTERY) {
            parseBatteryPdu(pdu, length);
        }

        consume(total);
        progressed = true;
    }

    if (state.battIsStale(STALE_DATA_TIMEOUT_MS)) {
        state.batt_connected = false;
    }
}
