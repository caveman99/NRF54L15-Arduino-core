#ifndef SPI_h
#define SPI_h

#include <stddef.h>
#include <stdint.h>

#include <Arduino.h>

#define SPI_HAS_TRANSACTION 1

#define SPI_MODE0 0x00
#define SPI_MODE1 0x01
#define SPI_MODE2 0x02
#define SPI_MODE3 0x03

class SPISettings {
public:
    SPISettings(uint32_t clock = 4000000U, uint8_t bitOrder = MSBFIRST, uint8_t dataMode = SPI_MODE0)
        : _clock(clock), _bitOrder(bitOrder), _dataMode(dataMode)
    {
    }

    uint32_t clock() const { return _clock; }
    uint8_t bitOrder() const { return _bitOrder; }
    uint8_t dataMode() const { return _dataMode; }

private:
    uint32_t _clock;
    uint8_t _bitOrder;
    uint8_t _dataMode;
};

class SPIClass {
public:
    SPIClass();

    void begin();
    void begin(uint8_t csPin);
    bool setPins(int8_t sck, int8_t miso, int8_t mosi, int8_t ss = -1);
    void end();

    void beginTransaction(const SPISettings &settings);
    void endTransaction();

    uint8_t transfer(uint8_t data);
    uint16_t transfer16(uint16_t data);
    void transfer(void *buffer, size_t length);
    void transfer(const void *txBuffer, void *rxBuffer, size_t length);

private:
    const void *_spi;
    uint32_t _frequency;
    uint16_t _operation;
    uint8_t _csPin;
    bool _hasCsPin;
    bool _inTransaction;
    int8_t _sckPin;
    int8_t _misoPin;
    int8_t _mosiPin;
    int8_t _preferredCsPin;
};

extern SPIClass SPI;

#endif
