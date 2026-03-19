#ifndef HardwareSerial_h
#define HardwareSerial_h

#include <stdint.h>

#include "Stream.h"

struct device;

class HardwareSerial : public Stream {
public:
    explicit HardwareSerial(const struct device *uart = nullptr, int8_t rxPin = -1, int8_t txPin = -1);

    void begin(unsigned long baud);
    void begin(unsigned long baud, uint16_t config);
    void end();
    bool setPins(int8_t rxPin, int8_t txPin);

    int available() override;
    int read() override;
    int peek() override;
    void flush() override;

    size_t write(uint8_t value) override;
    using Print::write;

    operator bool() const;
    bool isConfigured() const;
    bool usesPins(uint8_t rxPin, uint8_t txPin) const;

private:
    const struct device *_uart;
    int8_t _rxPin;
    int8_t _txPin;
    unsigned long _baud;
    uint16_t _config;
    bool _configured;
    int _peek;
};

extern HardwareSerial Serial;
extern HardwareSerial Serial1;
extern HardwareSerial &Serial2;

#endif
