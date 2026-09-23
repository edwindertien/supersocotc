#pragma once
/**
 * protocol.h -- pure protocol logic, deliberately free of any Arduino.h
 * dependency (just <cstdint>/<cstddef>) so it can be compiled and unit
 * tested standalone with a plain g++, independent of the PlatformIO/Pico
 * toolchain. See test/test_protocol.cpp.
 *
 * Every constant here is carried over from fardriver_web.py and this
 * project's protocol.py (silent monitor), which were verified extensively
 * against real hardware -- see context.md for the full derivation of each.
 * Don't hand-edit the CRC tables or handshake bytes without re-checking
 * against that file.
 */

#include <cstdint>
#include <cstddef>

namespace fdproto {

// ---- FarDriver two-table CRC ----
void crc(const uint8_t *data, size_t len, uint8_t &outA, uint8_t &outB);

// AA C6 A0 A0 88 <cmd> <crc_a> <crc_b> -- 8 bytes
void buildSyscmd(uint8_t cmd, uint8_t out[8]);

// AA <cmd> <~cmd> <sub> <v1> <v2> <crc> <~crc> -- 8 bytes, old-style
// "Sending commands" protocol, sum-of-bytes checksum (different from the
// two-table CRC above -- verified separately, see context.md).
void buildSendCmd(uint8_t command, uint8_t subCommand, uint8_t v1, uint8_t v2, uint8_t out[8]);

// The confirmed real heartbeat, sent ~1/sec while connected (see context.md
// "syscmd 0x04 / heartbeat" discovery)
extern const uint8_t HEARTBEAT[8];

// Exact bytes from a Saleae capture of a real BT dongle (fardriver_web.py)
extern const uint8_t VERSION_RESPONSE[27];
extern const uint8_t PAWD_RESPONSE[22];

// msg_id -> flash address, cycled by the controller's own periodic
// broadcast (fardriver_web.py FLASH_READ_ADDR, unabridged so msg_id
// indices line up)
extern const uint8_t FLASH_READ_ADDR[56];
constexpr size_t FLASH_READ_ADDR_LEN = 56;

// Validate a 16-byte FarDriver READ frame (AA [0x80|id] [12 data] [crc_a]
// [crc_b]). Returns true and fills msgId if valid.
bool parseReadFrame(const uint8_t frame[16], uint8_t &msgId);

// ---- Super Soco RS485 ----
// XOR checksum: length byte XOR'd with every PDU byte
uint8_t superSocoChecksum(uint8_t length, const uint8_t *pdu, size_t pduLen);

// Build a full response telegram: [0xB6,0x6B][src][dst][len][pdu...][chk][0x0D]
// `out` must have room for pduLen + 7 bytes. Returns the total frame length.
size_t buildSuperSocoResponse(uint8_t src, uint8_t dst,
                               const uint8_t *pdu, uint8_t pduLen,
                               uint8_t *out);

}  // namespace fdproto
