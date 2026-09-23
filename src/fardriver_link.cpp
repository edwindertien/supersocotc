#include "fardriver_link.h"
#include "protocol.h"
#include "state.h"
#include "config.h"
#include <cstring>

using namespace fdproto;

namespace {

uint8_t rxBuf[256];
size_t rxLen = 0;

bool handshaken = false;
uint8_t pawdCount = 0;
uint32_t lastHeartbeatMs = 0;

void consume(size_t n) {
    if (n >= rxLen) { rxLen = 0; return; }
    memmove(rxBuf, rxBuf + n, rxLen - n);
    rxLen -= n;
}

int findSeq(const uint8_t *needle, size_t needleLen) {
    if (needleLen == 0 || rxLen < needleLen) return -1;
    for (size_t i = 0; i + needleLen <= rxLen; ++i) {
        if (memcmp(rxBuf + i, needle, needleLen) == 0) return (int)i;
    }
    return -1;
}

bool containsSeq(const uint8_t *needle, size_t needleLen) {
    return findSeq(needle, needleLen) >= 0;
}

// Only the fields the instrument panel bridge actually needs -- a
// deliberate subset of fardriver_web.py's full decode(). See that file
// (and context.md) if more fields are ever needed here.
void decodeFrame(uint8_t msgId, const uint8_t data[12]) {
    if (msgId >= FLASH_READ_ADDR_LEN) return;
    uint8_t addr = FLASH_READ_ADDR[msgId];

    if (addr == 0xE2) {
        uint8_t b0 = data[0];
        state.forward = b0 & 0x01;
        state.reverse = b0 & 0x02;
        state.gear = (b0 >> 2) & 0x03;
        state.motion = b0 & 0x20;
        uint8_t b2 = data[2];
        state.hall_error = b2 & 0x01;
        state.throttle_error = b2 & 0x02;
        state.motor_temp_protect = b2 & 0x40;
        state.ctrl_temp_protect = b2 & 0x80;
        state.brake = data[3] & 0x80;
        uint16_t rawSpeed = data[6] | (data[7] << 8);
        state.raw_speed_value = rawSpeed;
        // PROVISIONAL factor -- see context.md "Speed calibration". Get a
        // real reference point and correct this if it's ever meaningfully off.
        state.speed_kmh = state.motion ? (rawSpeed * 0.0109f) : 0.0f;

    } else if (addr == 0xE8) {
        int16_t v = (int16_t)(data[0] | (data[1] << 8));
        state.voltage = v / 10.0f;
        int16_t c = (int16_t)(data[4] | (data[5] << 8));
        state.line_current = c / 4.0f;

    } else if (addr == 0xF4) {
        int16_t t = (int16_t)(data[0] | (data[1] << 8));
        state.motor_temp = t;

    } else if (addr == 0xD6) {
        uint16_t s1 = data[2] | (data[3] << 8);
        state.phase_lost = (s1 & 0x0800) != 0;
        uint16_t s2 = data[4] | (data[5] << 8);
        state.low_vol_stop = (s2 & 0x0010) != 0;
        int16_t mt = (int16_t)(data[10] | (data[11] << 8));
        state.mos_temp = mt;
    }

    state.fd_connected = true;
    state.fd_last_packet_ms = millis();
}

}  // namespace

void fardriverInit(Stream &uart) {
    // Pin assignment for SerialPIO happens at construction (see main.cpp);
    // nothing to configure here beyond resetting our own parse state.
    // If a HardwareSerial is ever passed in instead, call
    // .setTX()/.setRX()/.begin() on it in main.cpp before this runs, same
    // as here -- this function assumes .begin() has already happened.
    (void)uart;
    rxLen = 0;
    handshaken = false;
    pawdCount = 0;
    lastHeartbeatMs = millis();
}

void fardriverPoll(Stream &uart) {
    // Drain available bytes into rxBuf (bounded, drop oldest if it ever
    // fills -- shouldn't happen in normal operation, but never block/overflow)
    while (uart.available() && rxLen < sizeof(rxBuf)) {
        rxBuf[rxLen++] = (uint8_t)uart.read();
    }

    // --- BT handshake state machine ---
    static const uint8_t AT_VERSION[] = "AT+VERSION";
    static const uint8_t AT_PAWD[] = "AT+PAWD";

    int idx = findSeq(AT_VERSION, sizeof(AT_VERSION) - 1);
    if (idx >= 0) {
        uart.write(VERSION_RESPONSE, sizeof(VERSION_RESPONSE));
        consume(idx + sizeof(AT_VERSION) - 1);
    }
    while ((idx = findSeq(AT_PAWD, sizeof(AT_PAWD) - 1)) >= 0) {
        uart.write(PAWD_RESPONSE, sizeof(PAWD_RESPONSE));
        consume(idx + sizeof(AT_PAWD) - 1);
        if (++pawdCount >= 3) {
            handshaken = true;
            lastHeartbeatMs = millis();
        }
    }

    // --- Data frames: AA [0x80|id] [12 data] [crc_a] [crc_b] = 16 bytes ---
    bool progressed = true;
    while (progressed) {
        progressed = false;
        if (rxLen < 16) break;

        size_t i = 0;
        while (i < rxLen && rxBuf[i] != 0xAA) i++;
        if (i > 0) { consume(i); continue; }
        if (rxLen < 16) break;

        uint8_t msgId;
        if (parseReadFrame(rxBuf, msgId)) {
            decodeFrame(msgId, rxBuf + 2);
            consume(16);
            progressed = true;
        } else {
            consume(1);  // not a valid frame here, resync
            progressed = true;
        }
    }

    // If we've clearly dropped back to AT chatter without going through
    // the handshake branch above, force a re-sync
    if (handshaken && rxLen > 64 && containsSeq((const uint8_t *)"AT+", 3)) {
        handshaken = false;
        pawdCount = 0;
        rxLen = 0;
    }

    // Heartbeat -- confirmed real app behaviour, ~once/sec while connected
    if (handshaken && (millis() - lastHeartbeatMs) >= HEARTBEAT_INTERVAL_MS) {
        uart.write(HEARTBEAT, sizeof(HEARTBEAT));
        lastHeartbeatMs = millis();
    }

    if (state.fdIsStale(STALE_DATA_TIMEOUT_MS)) {
        state.fd_connected = false;
    }
}
