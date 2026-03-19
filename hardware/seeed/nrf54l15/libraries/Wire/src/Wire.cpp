#include "Wire.h"

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/util.h>

namespace {
const struct device* resolveI2CForPins(uint8_t sda, uint8_t scl)
{
#if DT_NODE_HAS_STATUS(DT_ALIAS(xiao_i2c), okay)
    if (sda == PIN_WIRE_SDA && scl == PIN_WIRE_SCL) {
        return DEVICE_DT_GET(DT_ALIAS(xiao_i2c));
    }
#endif

#if DT_NODE_HAS_STATUS(DT_ALIAS(xiao_i2c1), okay)
    if (sda == PIN_WIRE1_SDA && scl == PIN_WIRE1_SCL) {
        return DEVICE_DT_GET(DT_ALIAS(xiao_i2c1));
    }
#endif

    return nullptr;
}

uint32_t speedFromClock(uint32_t clockHz)
{
    if (clockHz >= 3400000U) {
        return I2C_SPEED_SET(I2C_SPEED_HIGH);
    }
    if (clockHz >= 1000000U) {
        return I2C_SPEED_SET(I2C_SPEED_FAST_PLUS);
    }
    if (clockHz >= 400000U) {
        return I2C_SPEED_SET(I2C_SPEED_FAST);
    }
    return I2C_SPEED_SET(I2C_SPEED_STANDARD);
}

struct TargetSlot {
    TwoWire* wire;
    struct i2c_target_config config;
};

TargetSlot g_targetSlots[2] = {};

TargetSlot* allocateTargetSlot(TwoWire* wire)
{
    for (size_t i = 0; i < ARRAY_SIZE(g_targetSlots); ++i) {
        if (g_targetSlots[i].wire == wire) {
            return &g_targetSlots[i];
        }
    }

    for (size_t i = 0; i < ARRAY_SIZE(g_targetSlots); ++i) {
        if (g_targetSlots[i].wire == nullptr) {
            g_targetSlots[i].wire = wire;
            g_targetSlots[i].config = {};
            return &g_targetSlots[i];
        }
    }

    return nullptr;
}

TwoWire* resolveWireFromTarget(struct i2c_target_config* config)
{
    if (config == nullptr) {
        return nullptr;
    }

    for (size_t i = 0; i < ARRAY_SIZE(g_targetSlots); ++i) {
        if (&g_targetSlots[i].config == config) {
            return g_targetSlots[i].wire;
        }
    }

    return nullptr;
}
} // namespace

TwoWire::TwoWire(NRF_TWIM_Type* twim, uint8_t sda, uint8_t scl)
    : _i2c(resolveI2CForPins(sda, scl)),
      _twim(twim),
      _sda(sda),
      _scl(scl),
      _frequency(400000U),
      _initialized(false),
      _txBufferLength(0),
      _txAddress(0),
      _rxBufferIndex(0),
      _rxBufferLength(0),
      _peek(-1),
      _targetTxLength(0),
      _targetTxIndex(0),
      _targetAddress(0),
      _targetRegistered(false),
      _inOnRequestCallback(false),
      _targetDirection(TARGET_DIR_NONE),
      _onReceive(nullptr),
      _onRequest(nullptr),
      _pendingRepeatedStart(false)
{
    (void)allocateTargetSlot(this);
}

void TwoWire::begin()
{
    const struct device* dev = static_cast<const struct device*>(_i2c);
    if (dev == nullptr || !device_is_ready(dev)) {
        return;
    }

    if (_targetRegistered) {
        TargetSlot* slot = allocateTargetSlot(this);
        if (slot != nullptr) {
            (void)i2c_target_unregister(dev, &slot->config);
        }
        _targetRegistered = false;
        _targetDirection = TARGET_DIR_NONE;
    }

    (void)i2c_configure(dev, I2C_MODE_CONTROLLER | speedFromClock(_frequency));
    _initialized = true;
}

void TwoWire::begin(uint8_t address)
{
    const struct device* dev = static_cast<const struct device*>(_i2c);
    if (dev == nullptr || !device_is_ready(dev)) {
        return;
    }

    TargetSlot* slot = allocateTargetSlot(this);
    if (slot == nullptr) {
        return;
    }

    if (_targetRegistered && _targetAddress == address) {
        return;
    }

    static const struct i2c_target_callbacks targetCallbacks = {
        .write_requested = TwoWire::targetWriteRequestedAdapter,
        .read_requested = TwoWire::targetReadRequestedAdapter,
        .write_received = TwoWire::targetWriteReceivedAdapter,
        .read_processed = TwoWire::targetReadProcessedAdapter,
        .stop = TwoWire::targetStopAdapter,
        .error = nullptr,
    };

    if (_targetRegistered) {
        (void)i2c_target_unregister(dev, &slot->config);
        _targetRegistered = false;
    }

    clearReceiveState();
    clearTargetTxState();

    slot->config = {};
    slot->config.address = address;
    slot->config.flags = 0;
    slot->config.callbacks = &targetCallbacks;

    int err = i2c_target_register(dev, &slot->config);
    if (err == 0) {
        _targetRegistered = true;
        _targetAddress = address;
        _targetDirection = TARGET_DIR_NONE;
    }
}

void TwoWire::begin(int address)
{
    if (address < 0 || address > 0x7F) {
        return;
    }
    begin(static_cast<uint8_t>(address));
}

void TwoWire::end()
{
    const struct device* dev = static_cast<const struct device*>(_i2c);
    TargetSlot* slot = allocateTargetSlot(this);

    if (_targetRegistered && dev != nullptr && device_is_ready(dev) && slot != nullptr) {
        (void)i2c_target_unregister(dev, &slot->config);
    }

    _targetRegistered = false;
    _targetDirection = TARGET_DIR_NONE;
    _inOnRequestCallback = false;
    _pendingRepeatedStart = false;
    clearControllerTxState();
    clearReceiveState();
    clearTargetTxState();
}

void TwoWire::setClock(uint32_t freq)
{
    _frequency = freq;
    begin();
}

void TwoWire::beginTransmission(uint8_t address)
{
    _txAddress = address;
    _pendingRepeatedStart = false;
    clearControllerTxState();
}

void TwoWire::beginTransmission(int address)
{
    beginTransmission(static_cast<uint8_t>(address));
}

size_t TwoWire::write(uint8_t data)
{
    if (isTargetWriteContext()) {
        if (_targetTxLength >= sizeof(_targetTxBuffer)) {
            return 0;
        }

        _targetTxBuffer[_targetTxLength++] = data;
        return 1;
    }

    if (_txBufferLength >= sizeof(_txBuffer)) {
        return 0;
    }

    _txBuffer[_txBufferLength++] = data;
    return 1;
}

size_t TwoWire::write(const uint8_t* data, size_t quantity)
{
    if (data == nullptr || quantity == 0) {
        return 0;
    }

    size_t written = 0;
    if (isTargetWriteContext()) {
        while (written < quantity && _targetTxLength < sizeof(_targetTxBuffer)) {
            _targetTxBuffer[_targetTxLength++] = data[written++];
        }
        return written;
    }

    while (written < quantity && _txBufferLength < sizeof(_txBuffer)) {
        _txBuffer[_txBufferLength++] = data[written++];
    }

    return written;
}

uint8_t TwoWire::endTransmission(bool sendStop)
{
    const struct device* dev = static_cast<const struct device*>(_i2c);
    if (dev == nullptr || !device_is_ready(dev)) {
        clearControllerTxState();
        _pendingRepeatedStart = false;
        return 4;
    }

    if (!sendStop) {
        _pendingRepeatedStart = true;
        return 0;
    }

    int err = i2c_write(dev, _txBuffer, _txBufferLength, _txAddress);
    clearControllerTxState();
    _pendingRepeatedStart = false;

    return (err == 0) ? 0 : 4;
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop)
{
    return requestFrom(address, static_cast<size_t>(quantity), sendStop != 0);
}

uint8_t TwoWire::requestFrom(uint8_t address, size_t quantity, bool sendStop)
{
    ARG_UNUSED(sendStop);

    const struct device* dev = static_cast<const struct device*>(_i2c);
    if (dev == nullptr || !device_is_ready(dev) || quantity == 0) {
        clearReceiveState();
        clearControllerTxState();
        _pendingRepeatedStart = false;
        return 0;
    }

    clearReceiveState();
    _rxBufferLength = static_cast<uint8_t>(MIN(quantity, sizeof(_rxBuffer)));

    int err = 0;
    if (_pendingRepeatedStart && _txBufferLength > 0 && _txAddress == address) {
        err = i2c_write_read(dev, address, _txBuffer, _txBufferLength, _rxBuffer, _rxBufferLength);
        clearControllerTxState();
        _pendingRepeatedStart = false;
    } else {
        err = i2c_read(dev, _rxBuffer, _rxBufferLength, address);
    }

    if (err != 0) {
        clearReceiveState();
    }

    return _rxBufferLength;
}

uint8_t TwoWire::requestFrom(int address, int quantity)
{
    return requestFrom(address, quantity, static_cast<uint8_t>(true));
}

uint8_t TwoWire::requestFrom(int address, int quantity, uint8_t sendStop)
{
    if (address < 0 || address > 0x7F || quantity <= 0) {
        clearReceiveState();
        clearControllerTxState();
        _pendingRepeatedStart = false;
        return 0;
    }
    return requestFrom(static_cast<uint8_t>(address), static_cast<size_t>(quantity), sendStop != 0);
}

int TwoWire::available(void)
{
    const uint8_t remaining = (_rxBufferIndex < _rxBufferLength) ? (_rxBufferLength - _rxBufferIndex) : 0U;
    return static_cast<int>(remaining + (_peek >= 0 ? 1U : 0U));
}

void TwoWire::onReceive(void (*callback)(int))
{
    _onReceive = callback;
}

void TwoWire::onRequest(void (*callback)(void))
{
    _onRequest = callback;
}

int TwoWire::read(void)
{
    if (_peek >= 0) {
        int value = _peek;
        _peek = -1;
        return value;
    }

    if (_rxBufferIndex >= _rxBufferLength) {
        return -1;
    }

    return _rxBuffer[_rxBufferIndex++];
}

int TwoWire::peek(void)
{
    if (_peek >= 0) {
        return _peek;
    }

    if (_rxBufferIndex >= _rxBufferLength) {
        return -1;
    }

    _peek = _rxBuffer[_rxBufferIndex++];
    return _peek;
}

void TwoWire::flush(void)
{
    clearControllerTxState();
    clearTargetTxState();
}

bool TwoWire::isTargetWriteContext() const
{
    return _inOnRequestCallback || (_targetRegistered && _targetDirection == TARGET_DIR_READ);
}

void TwoWire::clearControllerTxState()
{
    _txBufferLength = 0;
}

void TwoWire::clearReceiveState()
{
    _rxBufferLength = 0;
    _rxBufferIndex = 0;
    _peek = -1;
}

void TwoWire::clearTargetTxState()
{
    _targetTxLength = 0;
    _targetTxIndex = 0;
}

int TwoWire::provideTargetByte(uint8_t* value)
{
    if (value == nullptr) {
        return -EINVAL;
    }

    if (_targetTxIndex < _targetTxLength) {
        *value = _targetTxBuffer[_targetTxIndex++];
    } else {
        *value = 0xFF;
    }

    return 0;
}

int TwoWire::handleTargetWriteRequested()
{
    _targetDirection = TARGET_DIR_WRITE;
    clearReceiveState();
    return 0;
}

int TwoWire::handleTargetWriteReceived(uint8_t value)
{
    if (_rxBufferLength >= sizeof(_rxBuffer)) {
        return -ENOMEM;
    }

    _rxBuffer[_rxBufferLength++] = value;
    return 0;
}

int TwoWire::handleTargetReadRequested(uint8_t* value)
{
    _targetDirection = TARGET_DIR_READ;
    clearTargetTxState();

    if (_onRequest != nullptr) {
        _inOnRequestCallback = true;
        _onRequest();
        _inOnRequestCallback = false;
    }

    return provideTargetByte(value);
}

int TwoWire::handleTargetReadProcessed(uint8_t* value)
{
    return provideTargetByte(value);
}

int TwoWire::handleTargetStop()
{
    if (_targetDirection == TARGET_DIR_WRITE && _onReceive != nullptr) {
        _rxBufferIndex = 0;
        _peek = -1;
        _onReceive(static_cast<int>(_rxBufferLength));
    }

    _targetDirection = TARGET_DIR_NONE;
    _inOnRequestCallback = false;
    clearTargetTxState();

    return 0;
}

int TwoWire::targetWriteRequestedAdapter(struct i2c_target_config* config)
{
    TwoWire* wire = resolveWireFromTarget(config);
    return (wire != nullptr) ? wire->handleTargetWriteRequested() : -EINVAL;
}

int TwoWire::targetWriteReceivedAdapter(struct i2c_target_config* config, uint8_t value)
{
    TwoWire* wire = resolveWireFromTarget(config);
    return (wire != nullptr) ? wire->handleTargetWriteReceived(value) : -EINVAL;
}

int TwoWire::targetReadRequestedAdapter(struct i2c_target_config* config, uint8_t* value)
{
    TwoWire* wire = resolveWireFromTarget(config);
    return (wire != nullptr) ? wire->handleTargetReadRequested(value) : -EINVAL;
}

int TwoWire::targetReadProcessedAdapter(struct i2c_target_config* config, uint8_t* value)
{
    TwoWire* wire = resolveWireFromTarget(config);
    return (wire != nullptr) ? wire->handleTargetReadProcessed(value) : -EINVAL;
}

int TwoWire::targetStopAdapter(struct i2c_target_config* config)
{
    TwoWire* wire = resolveWireFromTarget(config);
    return (wire != nullptr) ? wire->handleTargetStop() : -EINVAL;
}

TwoWire Wire(nullptr, PIN_WIRE_SDA, PIN_WIRE_SCL);
TwoWire Wire1(nullptr, PIN_WIRE1_SDA, PIN_WIRE1_SCL);
