// Minimal stand-in for SerialPIO.h -- constructor signature verified
// against arduino-pico's own docs (SerialPIO(txpin, rxpin, fifosize)),
// but not compiled against the real header, so treat this as lower-
// confidence than the Stream/HardwareSerial split in Arduino.h (which
// was corrected against an actual build failure).
#pragma once
#include "Arduino.h"

class SerialPIO : public Stream {
public:
    SerialPIO(int txPin, int rxPin, int fifoSize = 32) {
        (void)txPin; (void)rxPin; (void)fifoSize;
    }
    void begin(unsigned long) {}
    int available() override { return 0; }
    int read() override { return -1; }
    size_t write(const uint8_t *buf, size_t len) override { (void)buf; return len; }
    size_t write(uint8_t) override { return 1; }
};
