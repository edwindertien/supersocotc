#include "protocol.h"

namespace fdproto {

// ---- CRC tables (two-table, verbatim from fardriver_web.py) ----
static const uint8_t CRC_LO[256] = {
    0,192,193,1,195,3,2,194,198,6,7,199,5,197,196,4,
    204,12,13,205,15,207,206,14,10,202,203,11,201,9,8,200,
    216,24,25,217,27,219,218,26,30,222,223,31,221,29,28,220,
    20,212,213,21,215,23,22,214,210,18,19,211,17,209,208,16,
    240,48,49,241,51,243,242,50,54,246,247,55,245,53,52,244,
    60,252,253,61,255,63,62,254,250,58,59,251,57,249,248,56,
    40,232,233,41,235,43,42,234,238,46,47,239,45,237,236,44,
    228,36,37,229,39,231,230,38,34,226,227,35,225,33,32,224,
    160,96,97,161,99,163,162,98,102,166,167,103,165,101,100,164,
    108,172,173,109,175,111,110,174,170,106,107,171,105,169,168,104,
    120,184,185,121,187,123,122,186,190,126,127,191,125,189,188,124,
    180,116,117,181,119,183,182,118,114,178,179,115,177,113,112,176,
    80,144,145,81,147,83,82,146,150,86,87,151,85,149,148,84,
    156,92,93,157,95,159,158,94,90,154,155,91,153,89,88,152,
    136,72,73,137,75,139,138,74,78,142,143,79,141,77,76,140,
    68,132,133,69,135,71,70,134,130,66,67,131,65,129,128,64,
};
static const uint8_t CRC_HI[256] = {
    0,193,129,64,1,192,128,65,1,192,128,65,0,193,129,64,
    1,192,128,65,0,193,129,64,0,193,129,64,1,192,128,65,
    1,192,128,65,0,193,129,64,0,193,129,64,1,192,128,65,
    0,193,129,64,1,192,128,65,1,192,128,65,0,193,129,64,
    1,192,128,65,0,193,129,64,0,193,129,64,1,192,128,65,
    0,193,129,64,1,192,128,65,1,192,128,65,0,193,129,64,
    0,193,129,64,1,192,128,65,1,192,128,65,0,193,129,64,
    1,192,128,65,0,193,129,64,0,193,129,64,1,192,128,65,
    1,192,128,65,0,193,129,64,0,193,129,64,1,192,128,65,
    0,193,129,64,1,192,128,65,1,192,128,65,0,193,129,64,
    0,193,129,64,1,192,128,65,1,192,128,65,0,193,129,64,
    1,192,128,65,0,193,129,64,0,193,129,64,1,192,128,65,
    0,193,129,64,1,192,128,65,1,192,128,65,0,193,129,64,
    1,192,128,65,0,193,129,64,0,193,129,64,1,192,128,65,
    1,192,128,65,0,193,129,64,0,193,129,64,1,192,128,65,
    0,193,129,64,1,192,128,65,1,192,128,65,0,193,129,64,
};

void crc(const uint8_t *data, size_t len, uint8_t &outA, uint8_t &outB) {
    uint8_t a = 0x3C, b = 0x7F;
    for (size_t i = 0; i < len; ++i) {
        uint8_t idx = a ^ data[i];
        a = b ^ CRC_HI[idx];
        b = CRC_LO[idx];
    }
    outA = a;
    outB = b;
}

void buildSyscmd(uint8_t cmd, uint8_t out[8]) {
    out[0] = 0xAA; out[1] = 0xC6; out[2] = 0xA0; out[3] = 0xA0;
    out[4] = 0x88; out[5] = cmd;
    uint8_t a, b;
    crc(out, 6, a, b);
    out[6] = a; out[7] = b;
}

void buildSendCmd(uint8_t command, uint8_t subCommand, uint8_t v1, uint8_t v2, uint8_t out[8]) {
    out[0] = 0xAA;
    out[1] = command;
    out[2] = (uint8_t)(~command);
    out[3] = subCommand;
    out[4] = v1;
    out[5] = v2;
    uint8_t sum = 0;
    for (int i = 0; i < 6; ++i) sum += out[i];
    out[6] = sum;
    out[7] = (uint8_t)(~sum);
}

// AA 13 EC 07 09 6F 28 D7 -- confirmed against a real captured session
const uint8_t HEARTBEAT[8] = {0xAA, 0x13, 0xEC, 0x07, 0x09, 0x6F, 0x28, 0xD7};

const uint8_t VERSION_RESPONSE[27] = {
    0x2b,0x56,0x45,0x52,0x53,0x49,0x4f,0x4e,0x3d,  // +VERSION=
    0x01,0x08,0x6b,0x19,0x39,0xa4,0xe2,0x7e,
    0x86,0x97,0xd8,0xe6,0x7b,0xfd,0xb1,0x67,
    0x0d,0x0a
};
const uint8_t PAWD_RESPONSE[22] = {
    0x2b,0x50,0x41,0x57,0x44,0x3d,                  // +PAWD=
    0xce,0x1d,0x0d,0x8d,0x39,0x9f,0xd3,0xf3,
    0x86,0x91,0xf6,0x42,0x7c,0x77,0x85,0xe1
};

const uint8_t FLASH_READ_ADDR[56] = {
    0xE2, 0xE8, 0xEE, 0x00, 0x06, 0x0C, 0x12,
    0xE2, 0xE8, 0xEE, 0x18, 0x1E, 0x24, 0x2A,
    0xE2, 0xE8, 0xEE, 0x30, 0x5D, 0x63, 0x69,
    0xE2, 0xE8, 0xEE, 0x7C, 0x82, 0x88, 0x8E,
    0xE2, 0xE8, 0xEE, 0x94, 0x9A, 0xA0, 0xA6,
    0xE2, 0xE8, 0xEE, 0xAC, 0xB2, 0xB8, 0xBE,
    0xE2, 0xE8, 0xEE, 0xC4, 0xCA, 0xD0,
    0xE2, 0xE8, 0xEE, 0xD6, 0xDC, 0xF4, 0xFA,
};

bool parseReadFrame(const uint8_t frame[16], uint8_t &msgId) {
    if (frame[0] != 0xAA) return false;
    uint8_t flagsId = frame[1];
    if ((flagsId >> 6) != 2) return false;  // flags must be 0b10
    uint8_t a, b;
    crc(frame, 14, a, b);
    if (a != frame[14] || b != frame[15]) return false;
    msgId = flagsId & 0x3F;
    return true;
}

uint8_t superSocoChecksum(uint8_t length, const uint8_t *pdu, size_t pduLen) {
    uint8_t c = length;
    for (size_t i = 0; i < pduLen; ++i) c ^= pdu[i];
    return c;
}

size_t buildSuperSocoResponse(uint8_t src, uint8_t dst,
                               const uint8_t *pdu, uint8_t pduLen,
                               uint8_t *out) {
    // CORRECTED (2026) against a real captured battery response
    // (B6 6B AA 5A 0A ...): position 2 is DESTINATION, position 3 is
    // SOURCE -- a response is [DST=master][SRC=responder]. Was
    // previously [src][dst], backwards -- see config.h's comment on
    // ADDR_PANEL for the full reasoning.
    size_t i = 0;
    out[i++] = 0xB6;
    out[i++] = 0x6B;
    out[i++] = dst;
    out[i++] = src;
    out[i++] = pduLen;
    for (uint8_t j = 0; j < pduLen; ++j) out[i++] = pdu[j];
    out[i++] = superSocoChecksum(pduLen, pdu, pduLen);
    out[i++] = 0x0D;
    return i;
}

}  // namespace fdproto