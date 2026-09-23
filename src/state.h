#pragma once
/**
 * state.h -- single shared struct holding everything the bridge knows,
 * from both sides of the bridge. Every module reads/writes plain fields
 * on the one shared instance.
 *
 * Architecture note: this project runs single-core (matching the proven
 * pattern from picoControl -- one setup()/loop(), no setup1()/loop1()),
 * so there's no genuine cross-core race here. Fields are still marked
 * `volatile` as a cheap safety margin (costs nothing, guards against any
 * future interrupt-context access), not because it's currently required.
 *
 * A future OLED display module should only ever *read* from this struct.
 */

#include <Arduino.h>

struct BridgeState {
    // ---- FarDriver-derived (from TTL serial) ----
    volatile bool fd_connected = false;
    volatile uint32_t fd_last_packet_ms = 0;
    volatile uint8_t gear = 0;
    volatile bool forward = false;
    volatile bool reverse = false;
    volatile bool motion = false;
    volatile float speed_kmh = 0.0f;
    volatile uint16_t raw_speed_value = 0;
    volatile float voltage = 0.0f;
    volatile float line_current = 0.0f;
    volatile int16_t motor_temp = 0;
    volatile int16_t mos_temp = 0;
    volatile bool hall_error = false;
    volatile bool throttle_error = false;
    volatile bool motor_temp_protect = false;
    volatile bool ctrl_temp_protect = false;
    volatile bool phase_lost = false;
    volatile bool low_vol_stop = false;
    volatile bool brake = false;

    // ---- Battery-derived (eavesdropped from RS485) ----
    volatile bool batt_connected = false;
    volatile uint32_t batt_last_packet_ms = 0;
    volatile uint8_t batt_voltage = 0;
    volatile uint8_t batt_soc = 0;
    volatile int8_t batt_temp = 0;
    volatile int8_t batt_charge_current = 0;
    volatile uint16_t batt_cycles = 0;
    volatile uint8_t batt_activity = 0;   // 0=idle/unknown, 1=charging, 4=discharging
    volatile uint8_t batt_vbreaker = 0;

    // ---- Panel-facing (RS485, what we've been sending) ----
    volatile uint32_t panel_requests_seen = 0;
    volatile uint32_t panel_responses_sent = 0;
    volatile uint32_t panel_last_request_ms = 0;

    // ---- CLI test override (ENABLE_RS485_RESPOND, before FarDriver is
    // wired up -- lets you inject a speed/gear/temp value directly via
    // the serial CLI and see the panel respond, without waiting on real
    // telemetry). When active, these values are used to build the
    // controller response instead of the FarDriver-derived fields above.
    // Cleared with the `auto` CLI command. See main.cpp's command parser.
    volatile bool test_override_active = false;
    volatile float test_speed_kmh = 0.0f;
    volatile uint8_t test_gear = 0;
    volatile int8_t test_temp = 0;

    uint32_t boot_ms = 0;

    bool fdIsStale(uint32_t timeoutMs) const {
        return (millis() - fd_last_packet_ms) > timeoutMs;
    }
    bool battIsStale(uint32_t timeoutMs) const {
        return (millis() - batt_last_packet_ms) > timeoutMs;
    }
};

// Single shared instance, defined in main.cpp, declared here for every
// other file to reference
extern BridgeState state;