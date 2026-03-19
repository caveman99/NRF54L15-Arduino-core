#include "SPI.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>

namespace {
const struct device *resolveSPI()
{
#if DT_NODE_HAS_STATUS(DT_ALIAS(xiao_spi), okay)
    return DEVICE_DT_GET(DT_ALIAS(xiao_spi));
#else
    return nullptr;
#endif
}

uint16_t operationFromSettings(const SPISettings &settings)
{
    uint16_t op = SPI_WORD_SET(8);

    if (settings.bitOrder() == LSBFIRST) {
        op |= SPI_TRANSFER_LSB;
    } else {
        op |= SPI_TRANSFER_MSB;
    }

    switch (settings.dataMode()) {
    case SPI_MODE1:
        op |= SPI_MODE_CPHA;
        break;
    case SPI_MODE2:
        op |= SPI_MODE_CPOL;
        break;
    case SPI_MODE3:
        op |= SPI_MODE_CPOL | SPI_MODE_CPHA;
        break;
    case SPI_MODE0:
    default:
        break;
    }

    return op;
}
} // namespace

SPIClass::SPIClass()
    : _spi(resolveSPI()),
      _frequency(4000000U),
      _operation(operationFromSettings(SPISettings())),
      _csPin(PIN_SPI_SS),
      _hasCsPin(false),
      _inTransaction(false),
      _sckPin(PIN_SPI_SCK),
      _misoPin(PIN_SPI_MISO),
      _mosiPin(PIN_SPI_MOSI),
      _preferredCsPin(PIN_SPI_SS)
{
}

void SPIClass::begin()
{
    begin((_preferredCsPin >= 0) ? static_cast<uint8_t>(_preferredCsPin) : PIN_SPI_SS);
}

void SPIClass::begin(uint8_t csPin)
{
    _csPin = csPin;
    _preferredCsPin = static_cast<int8_t>(csPin);
    _hasCsPin = true;
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
}

bool SPIClass::setPins(int8_t sck, int8_t miso, int8_t mosi, int8_t ss)
{
    if (sck < 0 || miso < 0 || mosi < 0) {
        return false;
    }

    // Zephyr pin routing is fixed by devicetree. Accept only the board-default pins.
    if (sck != PIN_SPI_SCK || miso != PIN_SPI_MISO || mosi != PIN_SPI_MOSI) {
        return false;
    }

    _sckPin = sck;
    _misoPin = miso;
    _mosiPin = mosi;
    if (ss >= 0) {
        _preferredCsPin = ss;
    }

    return true;
}

void SPIClass::end()
{
    _inTransaction = false;
}

void SPIClass::beginTransaction(const SPISettings &settings)
{
    _frequency = settings.clock();
    _operation = operationFromSettings(settings);

    _inTransaction = true;
    if (_hasCsPin) {
        digitalWrite(_csPin, LOW);
    }
}

void SPIClass::endTransaction()
{
    if (_hasCsPin) {
        digitalWrite(_csPin, HIGH);
    }
    _inTransaction = false;
}

uint8_t SPIClass::transfer(uint8_t data)
{
    const struct device *dev = static_cast<const struct device *>(_spi);
    if (dev == nullptr || !device_is_ready(dev)) {
        return 0;
    }

    bool autoTransaction = !_inTransaction;
    if (autoTransaction) {
        beginTransaction(SPISettings());
    }

    uint8_t rx = 0;
    struct spi_buf tx_buf = {
        .buf = &data,
        .len = sizeof(data),
    };
    struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    struct spi_buf rx_buf = {
        .buf = &rx,
        .len = sizeof(rx),
    };
    struct spi_buf_set rx_set = {
        .buffers = &rx_buf,
        .count = 1,
    };

    struct spi_config cfg = {
        .frequency = _frequency,
        .operation = _operation,
        .slave = 0,
        .cs = nullptr,
    };

    (void)spi_transceive(dev, &cfg, &tx, &rx_set);

    if (autoTransaction) {
        endTransaction();
    }

    return rx;
}

uint16_t SPIClass::transfer16(uint16_t data)
{
    uint8_t tx[2];
    uint8_t rx[2] = {0U, 0U};

    if ((_operation & SPI_TRANSFER_LSB) != 0U) {
        tx[0] = static_cast<uint8_t>(data & 0xFFU);
        tx[1] = static_cast<uint8_t>((data >> 8) & 0xFFU);
    } else {
        tx[0] = static_cast<uint8_t>((data >> 8) & 0xFFU);
        tx[1] = static_cast<uint8_t>(data & 0xFFU);
    }

    transfer(tx, rx, sizeof(tx));

    if ((_operation & SPI_TRANSFER_LSB) != 0U) {
        return static_cast<uint16_t>(rx[0]) | (static_cast<uint16_t>(rx[1]) << 8);
    }
    return (static_cast<uint16_t>(rx[0]) << 8) | static_cast<uint16_t>(rx[1]);
}

void SPIClass::transfer(void *buffer, size_t length)
{
    if (buffer == nullptr || length == 0) {
        return;
    }

    transfer(buffer, buffer, length);
}

void SPIClass::transfer(const void *txBuffer, void *rxBuffer, size_t length)
{
    if (length == 0) {
        return;
    }

    const uint8_t *txData = static_cast<const uint8_t *>(txBuffer);
    uint8_t *rxData = static_cast<uint8_t *>(rxBuffer);

    if (txData == nullptr && rxData == nullptr) {
        return;
    }

    if (txData == nullptr) {
        for (size_t i = 0; i < length; ++i) {
            rxData[i] = transfer(0xFFU);
        }
        return;
    }

    if (rxData == nullptr) {
        for (size_t i = 0; i < length; ++i) {
            (void)transfer(txData[i]);
        }
        return;
    }

    const struct device *dev = static_cast<const struct device *>(_spi);
    if (dev == nullptr || !device_is_ready(dev)) {
        return;
    }

    bool autoTransaction = !_inTransaction;
    if (autoTransaction) {
        beginTransaction(SPISettings());
    }

    struct spi_buf tx_buf = {
        .buf = const_cast<uint8_t *>(txData),
        .len = length,
    };
    struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    struct spi_buf rx_buf = {
        .buf = rxData,
        .len = length,
    };
    struct spi_buf_set rx_set = {
        .buffers = &rx_buf,
        .count = 1,
    };

    struct spi_config cfg = {
        .frequency = _frequency,
        .operation = _operation,
        .slave = 0,
        .cs = nullptr,
    };

    (void)spi_transceive(dev, &cfg, &tx, &rx_set);

    if (autoTransaction) {
        endTransaction();
    }
}

SPIClass SPI;
