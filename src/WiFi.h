// Minimal stand-ins for WiFi.h / String -- same purpose as Arduino.h in
// this directory: catch real compile errors before they reach an actual
// PlatformIO build, not a functional emulation. String's API is stable
// across virtually all Arduino cores (unlike HardwareSerial/SerialUART,
// which turned out to have real RP2040-specific gaps this stub caught
// too late for) -- lower risk here, but still verify against a real
// build rather than assume this covers everything.
#pragma once
#include "Arduino.h"
#include <string>

class String {
public:
    String() {}
    String(const char *s) : _s(s ? s : "") {}
    explicit String(char c) : _s(1, c) {}
    String(int v) : _s(std::to_string(v)) {}
    String(unsigned long v) : _s(std::to_string(v)) {}
    String(long v) : _s(std::to_string(v)) {}

    size_t length() const { return _s.length(); }
    void trim() {
        size_t a = _s.find_first_not_of(" \t\r\n");
        size_t b = _s.find_last_not_of(" \t\r\n");
        _s = (a == std::string::npos) ? "" : _s.substr(a, b - a + 1);
    }
    int indexOf(char c) const {
        auto p = _s.find(c);
        return p == std::string::npos ? -1 : (int)p;
    }
    int indexOf(char c, int from) const {
        if (from < 0) from = 0;
        auto p = _s.find(c, (size_t)from);
        return p == std::string::npos ? -1 : (int)p;
    }
    String substring(int from) const {
        if (from < 0 || (size_t)from > _s.length()) return String("");
        return String(_s.substr(from).c_str());
    }
    String substring(int from, int to) const {
        if (from < 0) from = 0;
        if (to < from) return String("");
        return String(_s.substr(from, to - from).c_str());
    }
    bool operator==(const char *other) const { return _s == other; }
    bool operator==(const String &other) const { return _s == other._s; }
    bool operator!=(const char *other) const { return _s != other; }
    const char *c_str() const { return _s.c_str(); }
    int toInt() const { return _s.empty() ? 0 : std::stoi(_s); }

private:
    std::string _s;
};

class IPAddress {
public:
    IPAddress() {}
    IPAddress(uint8_t a, uint8_t b, uint8_t c, uint8_t d) : _a{a, b, c, d} {}
    uint8_t operator[](int i) const { return _a[i]; }
    String toString() const {
        char buf[16];
        snprintf(buf, sizeof(buf), "%u.%u.%u.%u", _a[0], _a[1], _a[2], _a[3]);
        return String(buf);
    }
private:
    uint8_t _a[4] = {0, 0, 0, 0};
};

inline void HardwareSerial::println(const IPAddress &ip) {
    printf("%s\n", ip.toString().c_str());
}

class WiFiClient : public Stream {
public:
    operator bool() const { return _connected; }
    bool connected() const { return _connected; }
    void setTimeout(unsigned long) {}
    String readStringUntil(char) { return String(""); }
    void stop() { _connected = false; }
    IPAddress remoteIP() const { return IPAddress(); }
    void print(const char *) {}
    void print(const String &) {}
    void println() {}
    void println(const char *) {}
    void println(const String &) {}
    int available() override { return 0; }
    int read() override { return -1; }
    size_t write(const uint8_t *buf, size_t len) override { (void)buf; return len; }
    size_t write(uint8_t) override { return 1; }
private:
    bool _connected = false;
};

class WiFiServer {
public:
    explicit WiFiServer(int) {}
    void begin() {}
    void stop() {}
    WiFiClient accept() { return WiFiClient(); }
};

enum wifi_mode_t { WIFI_OFF, WIFI_STA, WIFI_AP };

class WiFiClass {
public:
    void mode(wifi_mode_t) {}
    bool softAP(const char *, const char *, int = 1, bool = false) { return true; }
    bool softAPConfig(IPAddress, IPAddress, IPAddress) { return true; }
    bool softAPdisconnect(bool = false) { return true; }
    IPAddress softAPIP() { return IPAddress(192, 168, 4, 1); }
};

extern WiFiClass WiFi;

#define PROGMEM
inline uint8_t pgm_read_byte(const uint8_t *p) { return *p; }
