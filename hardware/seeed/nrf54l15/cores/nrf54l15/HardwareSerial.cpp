#include "HardwareSerial.h"

#include "Arduino.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

namespace {
const struct device *resolveBridgeDevice()
{
#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart20), okay)
    return DEVICE_DT_GET(DT_NODELABEL(uart20));
#elif DT_NODE_HAS_STATUS(DT_CHOSEN(zephyr_console), okay)
    return DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
#else
    return nullptr;
#endif
}

const struct device *resolveHeaderDevice()
{
#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart21), okay)
    return DEVICE_DT_GET(DT_NODELABEL(uart21));
#elif DT_NODE_HAS_STATUS(DT_ALIAS(xiao_serial), okay)
    return DEVICE_DT_GET(DT_ALIAS(xiao_serial));
#else
    return nullptr;
#endif
}

const struct device *resolveDeviceForPins(int8_t rxPin, int8_t txPin)
{
    if (rxPin == PIN_SAMD11_RX && txPin == PIN_SAMD11_TX) {
        return resolveBridgeDevice();
    }

    if (rxPin == PIN_SERIAL_RX && txPin == PIN_SERIAL_TX) {
        return resolveHeaderDevice();
    }

    if (rxPin == PIN_SERIAL1_RX && txPin == PIN_SERIAL1_TX) {
        return resolveHeaderDevice();
    }

    return nullptr;
}

uart_config uartConfigFromArduino(unsigned long baud, uint16_t config)
{
    uart_config cfg = {
        .baudrate = static_cast<uint32_t>(baud),
        .parity = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    };

    switch (config & 0x30U) {
    case 0x20U:
        cfg.parity = UART_CFG_PARITY_EVEN;
        break;
    case 0x30U:
        cfg.parity = UART_CFG_PARITY_ODD;
        break;
    default:
        cfg.parity = UART_CFG_PARITY_NONE;
        break;
    }

    cfg.stop_bits = ((config & 0x08U) != 0U) ? UART_CFG_STOP_BITS_2 : UART_CFG_STOP_BITS_1;

    switch (config & 0x06U) {
    case 0x00U:
        cfg.data_bits = UART_CFG_DATA_BITS_5;
        break;
    case 0x02U:
        cfg.data_bits = UART_CFG_DATA_BITS_6;
        break;
    case 0x04U:
        cfg.data_bits = UART_CFG_DATA_BITS_7;
        break;
    case 0x06U:
    default:
        cfg.data_bits = UART_CFG_DATA_BITS_8;
        break;
    }

    return cfg;
}
} // namespace

HardwareSerial::HardwareSerial(const struct device *uart, int8_t rxPin, int8_t txPin)
    : _uart(uart != nullptr ? uart : resolveBridgeDevice()),
      _rxPin(rxPin),
      _txPin(txPin),
      _baud(0UL),
      _config(SERIAL_8N1),
      _configured(false),
      _peek(-1)
{
}

void HardwareSerial::begin(unsigned long baud)
{
    begin(baud, SERIAL_8N1);
}

void HardwareSerial::begin(unsigned long baud, uint16_t config)
{
    if (_uart == nullptr || !device_is_ready(_uart)) {
        return;
    }

    _baud = baud;
    _config = config;

    uart_config cfg = uartConfigFromArduino(baud, config);
    (void)uart_configure(_uart, &cfg);
    _configured = true;
}

void HardwareSerial::end()
{
    _peek = -1;
    _configured = false;
}

bool HardwareSerial::setPins(int8_t rxPin, int8_t txPin)
{
    const struct device *dev = resolveDeviceForPins(rxPin, txPin);
    if (dev == nullptr || !device_is_ready(dev)) {
        return false;
    }

    _uart = dev;
    _rxPin = rxPin;
    _txPin = txPin;

    if (_configured) {
        begin(_baud, _config);
    }

    return true;
}

int HardwareSerial::available()
{
    if (_peek >= 0) {
        return 1;
    }

    if (_uart == nullptr || !device_is_ready(_uart)) {
        return 0;
    }

    unsigned char c = 0;
    if (uart_poll_in(_uart, &c) == 0) {
        _peek = c;
        return 1;
    }

    return 0;
}

int HardwareSerial::read()
{
    if (_peek >= 0) {
        int value = _peek;
        _peek = -1;
        return value;
    }

    if (_uart == nullptr || !device_is_ready(_uart)) {
        return -1;
    }

    unsigned char c = 0;
    if (uart_poll_in(_uart, &c) == 0) {
        return c;
    }

    return -1;
}

int HardwareSerial::peek()
{
    (void)available();
    return _peek;
}

void HardwareSerial::flush()
{
    k_sleep(K_MSEC(1));
}

size_t HardwareSerial::write(uint8_t value)
{
    if (_uart == nullptr || !device_is_ready(_uart)) {
        return 0;
    }

    uart_poll_out(_uart, value);
    return 1;
}

HardwareSerial::operator bool() const
{
    return _configured && (_uart != nullptr) && device_is_ready(_uart);
}

bool HardwareSerial::isConfigured() const
{
    return _configured && (_uart != nullptr) && device_is_ready(_uart);
}

bool HardwareSerial::usesPins(uint8_t rxPin, uint8_t txPin) const
{
    return (_rxPin == static_cast<int8_t>(rxPin)) && (_txPin == static_cast<int8_t>(txPin));
}

HardwareSerial Serial(resolveBridgeDevice(), PIN_SAMD11_RX, PIN_SAMD11_TX);
HardwareSerial Serial1(resolveHeaderDevice(), PIN_SERIAL1_RX, PIN_SERIAL1_TX);
HardwareSerial &Serial2 = Serial1;
