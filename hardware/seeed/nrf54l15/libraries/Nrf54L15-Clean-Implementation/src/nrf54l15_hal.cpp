#include "nrf54l15_hal.h"

#include <Arduino.h>
#include <Bluetooth.h>
#include <HardwareSerial.h>
#include <PDM.h>
#include <SPI.h>
#include <Wire.h>
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <variant.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>
#include <zephyr/random/random.h>

namespace {

using namespace nrf54l15;
using namespace xiao_nrf54l15;

CpuFrequency g_idleCpuFrequency = CpuFrequency::k64MHz;
bool g_idleCpuScalingEnabled = false;
xiao_nrf54l15::I2sTx* g_activeI2sTx = nullptr;
xiao_nrf54l15::I2sRx* g_activeI2sRx = nullptr;
xiao_nrf54l15::I2sDuplex* g_activeI2sDuplex = nullptr;
void (*g_i2sRawHandler)() = nullptr;
bool g_i2sIrqConnected = false;

void i2sSharedZephyrIsr(const void*) {
  if (g_i2sRawHandler != nullptr) {
    g_i2sRawHandler();
  } else if (g_activeI2sDuplex != nullptr) {
    g_activeI2sDuplex->onIrq();
  } else if (g_activeI2sRx != nullptr) {
    g_activeI2sRx->onIrq();
  } else if (g_activeI2sTx != nullptr) {
    g_activeI2sTx->onIrq();
  }
}

void ensureI2sIrqConnected(uint8_t priority) {
  if (!g_i2sIrqConnected) {
    (void)irq_connect_dynamic(I2S20_IRQn, priority, i2sSharedZephyrIsr, nullptr, 0);
    g_i2sIrqConnected = true;
  }
  irq_enable(I2S20_IRQn);
}

NRF_GPIO_Type* gpioForPort(uint8_t port) {
  switch (port) {
    case 0:
      return NRF_P0;
    case 1:
      return NRF_P1;
    case 2:
      return NRF_P2;
    default:
      return nullptr;
  }
}

bool pinToDigitalPin(const Pin& pin, uint8_t* digitalPin) {
  if (digitalPin == nullptr) {
    return false;
  }

  if (pin.port == kPinD0.port && pin.pin == kPinD0.pin) {
    *digitalPin = PIN_D0;
  } else if (pin.port == kPinD1.port && pin.pin == kPinD1.pin) {
    *digitalPin = PIN_D1;
  } else if (pin.port == kPinD2.port && pin.pin == kPinD2.pin) {
    *digitalPin = PIN_D2;
  } else if (pin.port == kPinD3.port && pin.pin == kPinD3.pin) {
    *digitalPin = PIN_D3;
  } else if (pin.port == kPinD4.port && pin.pin == kPinD4.pin) {
    *digitalPin = PIN_D4;
  } else if (pin.port == kPinD5.port && pin.pin == kPinD5.pin) {
    *digitalPin = PIN_D5;
  } else if (pin.port == kPinD6.port && pin.pin == kPinD6.pin) {
    *digitalPin = PIN_D6;
  } else if (pin.port == kPinD7.port && pin.pin == kPinD7.pin) {
    *digitalPin = PIN_D7;
  } else if (pin.port == kPinD8.port && pin.pin == kPinD8.pin) {
    *digitalPin = PIN_D8;
  } else if (pin.port == kPinD9.port && pin.pin == kPinD9.pin) {
    *digitalPin = PIN_D9;
  } else if (pin.port == kPinD10.port && pin.pin == kPinD10.pin) {
    *digitalPin = PIN_D10;
  } else if (pin.port == kPinD11.port && pin.pin == kPinD11.pin) {
    *digitalPin = PIN_D11;
  } else if (pin.port == kPinD12.port && pin.pin == kPinD12.pin) {
    *digitalPin = PIN_D12;
  } else if (pin.port == kPinD13.port && pin.pin == kPinD13.pin) {
    *digitalPin = PIN_D13;
  } else if (pin.port == kPinD14.port && pin.pin == kPinD14.pin) {
    *digitalPin = PIN_D14;
  } else if (pin.port == kPinD15.port && pin.pin == kPinD15.pin) {
    *digitalPin = PIN_D15;
  } else if (pin.port == kPinUserLed.port && pin.pin == kPinUserLed.pin) {
    *digitalPin = PIN_LED_BUILTIN;
  } else if (pin.port == kPinUserButton.port && pin.pin == kPinUserButton.pin) {
    *digitalPin = PIN_BUTTON;
  } else if (pin.port == kPinSAMD11Tx.port && pin.pin == kPinSAMD11Tx.pin) {
    *digitalPin = PIN_SAMD11_TX;
  } else if (pin.port == kPinSAMD11Rx.port && pin.pin == kPinSAMD11Rx.pin) {
    *digitalPin = PIN_SAMD11_RX;
  } else if (pin.port == kPinImuMicPowerEnable.port &&
             pin.pin == kPinImuMicPowerEnable.pin) {
    *digitalPin = PIN_IMU_MIC_PWR;
  } else if (pin.port == kPinRfSwitchPower.port &&
             pin.pin == kPinRfSwitchPower.pin) {
    *digitalPin = PIN_RF_SW;
  } else if (pin.port == kPinRfSwitchCtl.port &&
             pin.pin == kPinRfSwitchCtl.pin) {
    *digitalPin = PIN_RF_SW_CTL;
  } else if (pin.port == kPinVbatEnable.port && pin.pin == kPinVbatEnable.pin) {
    *digitalPin = PIN_VBAT_EN;
  } else if (pin.port == kPinVbatSense.port && pin.pin == kPinVbatSense.pin) {
    *digitalPin = PIN_VBAT_READ;
  } else {
    return false;
  }

  return true;
}

bool samePin(const Pin& lhs, const Pin& rhs) {
  return lhs.port == rhs.port && lhs.pin == rhs.pin;
}

uint8_t spiModeToArduino(SpiMode mode) {
  switch (mode) {
    case SpiMode::kMode1:
      return SPI_MODE1;
    case SpiMode::kMode2:
      return SPI_MODE2;
    case SpiMode::kMode3:
      return SPI_MODE3;
    case SpiMode::kMode0:
    default:
      return SPI_MODE0;
  }
}

TwoWire* resolveWireForPins(const Pin& scl, const Pin& sda) {
  if (samePin(scl, kDefaultI2cScl) && samePin(sda, kDefaultI2cSda)) {
    return &Wire;
  }

  if (samePin(scl, kPinD11) && samePin(sda, kPinD12)) {
    return &Wire1;
  }

  return nullptr;
}

enum class ComparatorOwner : uint8_t {
  kNone = 0U,
  kComp = 1U,
  kLpcomp = 2U,
};

ComparatorOwner g_comparatorOwner = ComparatorOwner::kNone;

bool claimComparator(ComparatorOwner owner) {
  if (g_comparatorOwner != ComparatorOwner::kNone &&
      g_comparatorOwner != owner) {
    return false;
  }
  g_comparatorOwner = owner;
  return true;
}

void releaseComparator(ComparatorOwner owner) {
  if (g_comparatorOwner == owner) {
    g_comparatorOwner = ComparatorOwner::kNone;
  }
}

bool pinSupportsAnalogInput(const Pin& pin) {
  return isConnected(pin) && (xiao_nrf54l15::saadcInputForPin(pin) >= 0);
}

bool configureAnalogPeripheralPin(const Pin& pin) {
  if (!pinSupportsAnalogInput(pin)) {
    return false;
  }

  NRF_GPIO_Type* gpio = gpioForPort(pin.port);
  if (gpio == nullptr) {
    return false;
  }

  uint32_t cnf = 0U;
  cnf |= GPIO_PIN_CNF_INPUT_Disconnect;
  cnf |= GPIO_PIN_CNF_PULL_Disabled;
  cnf |= GPIO_PIN_CNF_SENSE_Disabled;
  cnf |= ((GPIO_PIN_CNF_CTRLSEL_GPIO << GPIO_PIN_CNF_CTRLSEL_Pos) &
          GPIO_PIN_CNF_CTRLSEL_Msk);
  gpio->PIN_CNF[pin.pin] = cnf;
  return true;
}

uint32_t makeCompPinSelect(const Pin& pin) {
  return ((static_cast<uint32_t>(pin.pin) << COMP_PSEL_PIN_Pos) &
          COMP_PSEL_PIN_Msk) |
         ((static_cast<uint32_t>(pin.port) << COMP_PSEL_PORT_Pos) &
          COMP_PSEL_PORT_Msk);
}

uint32_t makeLpcompPinSelect(const Pin& pin) {
  return ((static_cast<uint32_t>(pin.pin) << LPCOMP_PSEL_PIN_Pos) &
          LPCOMP_PSEL_PIN_Msk) |
         ((static_cast<uint32_t>(pin.port) << LPCOMP_PSEL_PORT_Pos) &
          LPCOMP_PSEL_PORT_Msk);
}

bool waitForNonZero(volatile uint32_t* reg, uint32_t spinLimit) {
  if (reg == nullptr) {
    return false;
  }

  while (spinLimit-- > 0U) {
    if (*reg != 0U) {
      return true;
    }
  }
  return false;
}

uint32_t makeQdecConnectedPinSelect(const Pin& pin) {
  if (!isConnected(pin)) {
    return PSEL_DISCONNECTED;
  }

  return ((static_cast<uint32_t>(pin.pin) << QDEC_PSEL_A_PIN_Pos) &
          QDEC_PSEL_A_PIN_Msk) |
         ((static_cast<uint32_t>(pin.port) << QDEC_PSEL_A_PORT_Pos) &
          QDEC_PSEL_A_PORT_Msk) |
         ((QDEC_PSEL_A_CONNECT_Connected << QDEC_PSEL_A_CONNECT_Pos) &
          QDEC_PSEL_A_CONNECT_Msk);
}

uint16_t clampPermille(uint16_t permille) {
  return (permille > 1000U) ? 1000U : permille;
}

uint32_t absDiffU32(uint32_t lhs, uint32_t rhs) {
  return (lhs >= rhs) ? (lhs - rhs) : (rhs - lhs);
}

uint32_t compThresholdCodeFromPermille(uint16_t permille) {
  const uint16_t clamped = clampPermille(permille);
  uint32_t scaled =
      (static_cast<uint32_t>(clamped) * 64U + 500U) / 1000U;
  if (scaled < 1U) {
    scaled = 1U;
  }
  if (scaled > 64U) {
    scaled = 64U;
  }
  return scaled - 1U;
}

uint32_t compThresholdRegValue(uint16_t lowPermille, uint16_t highPermille) {
  const uint32_t lowCode = compThresholdCodeFromPermille(lowPermille);
  const uint32_t highCode = compThresholdCodeFromPermille(highPermille);
  return ((lowCode << COMP_TH_THDOWN_Pos) & COMP_TH_THDOWN_Msk) |
         ((highCode << COMP_TH_THUP_Pos) & COMP_TH_THUP_Msk);
}

bool waitForCompReady(volatile uint32_t* readyEvent, uint32_t spinLimit) {
  return waitForNonZero(readyEvent, spinLimit);
}

GpioPull gpioPullFromQdecInputPull(QdecInputPull pull) {
  switch (pull) {
    case QdecInputPull::kPullDown:
      return GpioPull::kPullDown;
    case QdecInputPull::kPullUp:
      return GpioPull::kPullUp;
    case QdecInputPull::kDisabled:
    default:
      return GpioPull::kDisabled;
  }
}

LpcompReference nearestLpcompReference(uint16_t thresholdPermille) {
  struct Candidate {
    uint16_t permille;
    LpcompReference reference;
  };

  static constexpr Candidate kCandidates[] = {
      {63U, LpcompReference::k1over16Vdd},
      {125U, LpcompReference::k1over8Vdd},
      {188U, LpcompReference::k3over16Vdd},
      {250U, LpcompReference::k2over8Vdd},
      {313U, LpcompReference::k5over16Vdd},
      {375U, LpcompReference::k3over8Vdd},
      {438U, LpcompReference::k7over16Vdd},
      {500U, LpcompReference::k4over8Vdd},
      {563U, LpcompReference::k9over16Vdd},
      {625U, LpcompReference::k5over8Vdd},
      {688U, LpcompReference::k11over16Vdd},
      {750U, LpcompReference::k6over8Vdd},
      {813U, LpcompReference::k13over16Vdd},
      {875U, LpcompReference::k7over8Vdd},
      {938U, LpcompReference::k15over16Vdd},
  };

  const uint16_t clamped = clampPermille(thresholdPermille);
  uint32_t bestDiff = 0xFFFFFFFFUL;
  LpcompReference best = LpcompReference::k4over8Vdd;
  for (const Candidate& candidate : kCandidates) {
    const uint32_t diff = absDiffU32(candidate.permille, clamped);
    if (diff < bestDiff) {
      bestDiff = diff;
      best = candidate.reference;
      if (diff == 0U) {
        break;
      }
    }
  }
  return best;
}

void disableSystemOffRetention() {
#ifdef NRF_MEMCONF
  for (uint32_t i = 0; i < MEMCONF_POWER_MaxCount; ++i) {
    NRF_MEMCONF->POWER[i].RET = 0U;
    NRF_MEMCONF->POWER[i].RET2 = 0U;
  }
#endif
}

void clearSystemOffVprRetention() {
#ifdef NRF_MEMCONF
  if (MEMCONF_POWER_MaxCount > 1U) {
    NRF_MEMCONF->POWER[1U].RET &= ~MEMCONF_POWER_RET_MEM0_Msk;
  }
#endif
}

[[noreturn]] void enterSystemOff(NRF_RESET_Type* reset,
                                 NRF_REGULATORS_Type* regulators,
                                 bool disableRamRetention) {
  clearSystemOffVprRetention();
  if (disableRamRetention) {
    disableSystemOffRetention();
  }

  __asm volatile("cpsid i" ::: "memory");
  __asm volatile("dsb 0xF" ::: "memory");
  __asm volatile("isb 0xF" ::: "memory");
  reset->RESETREAS = 0xFFFFFFFFUL;
  __asm volatile("dsb 0xF" ::: "memory");

  regulators->SYSTEMOFF = REGULATORS_SYSTEMOFF_SYSTEMOFF_Enter;
  __asm volatile("dsb 0xF" ::: "memory");
  while (true) {
    __asm volatile("wfe");
  }
}

HardwareSerial* resolveSerialForBase(uint32_t base) {
  if (base == nrf54l15::UARTE20_BASE) {
    return &Serial;
  }
  return &Serial1;
}

HardwareSerial* resolveSerialForPins(const Pin& txd, const Pin& rxd) {
  if (samePin(txd, kPinSAMD11Tx) && samePin(rxd, kPinSAMD11Rx)) {
    return &Serial;
  }
  if (samePin(txd, kDefaultUartTx) && samePin(rxd, kDefaultUartRx)) {
    return &Serial1;
  }
  return nullptr;
}

uint32_t pdmRatioDivisor(uint8_t ratio) {
  return (ratio == PDM_RATIO_RATIO_Ratio80) ? 80UL : 64UL;
}

uint32_t pdmSampleRateFromConfig(uint8_t prescalerDiv, uint8_t ratio) {
  if (prescalerDiv == 0U) {
    prescalerDiv = 1U;
  }

  uint32_t sampleRate = 32000000UL / static_cast<uint32_t>(prescalerDiv);
  sampleRate /= pdmRatioDivisor(ratio);
  if (sampleRate < 8000UL) {
    sampleRate = 8000UL;
  }
  if (sampleRate > 48000UL) {
    sampleRate = 48000UL;
  }
  return sampleRate;
}

void safeCopyString(const char* source, char* destination, size_t destinationSize) {
  if (destination == nullptr || destinationSize == 0U) {
    return;
  }

  if (source == nullptr) {
    destination[0] = '\0';
    return;
  }

  strncpy(destination, source, destinationSize - 1U);
  destination[destinationSize - 1U] = '\0';
}

bool formatBleUuid16String(uint16_t uuid16, char* destination, size_t destinationSize) {
  if (destination == nullptr || destinationSize < 5U) {
    return false;
  }

  snprintf(destination, destinationSize, "%04X", uuid16);
  return true;
}

bool formatBleUuid128String(const uint8_t* uuid128,
                            char* destination,
                            size_t destinationSize) {
  if (uuid128 == nullptr || destination == nullptr || destinationSize < 37U) {
    return false;
  }

  snprintf(destination, destinationSize,
           "%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X",
           uuid128[0], uuid128[1], uuid128[2], uuid128[3], uuid128[4], uuid128[5],
           uuid128[6], uuid128[7], uuid128[8], uuid128[9], uuid128[10], uuid128[11],
           uuid128[12], uuid128[13], uuid128[14], uuid128[15]);
  return true;
}

uint8_t bleGattPropertiesToArduino(uint8_t properties) {
  uint8_t result = 0U;
  if ((properties & kBleGattPropRead) != 0U) {
    result |= BLECharacteristic::BLERead;
  }
  if ((properties & kBleGattPropWrite) != 0U) {
    result |= BLECharacteristic::BLEWrite;
  }
  if ((properties & kBleGattPropWriteNoRsp) != 0U) {
    result |= BLECharacteristic::BLEWriteWithoutResponse;
  }
  if ((properties & kBleGattPropNotify) != 0U) {
    result |= BLECharacteristic::BLENotify;
  }
  if ((properties & kBleGattPropIndicate) != 0U) {
    result |= BLECharacteristic::BLEIndicate;
  }
  return result;
}

enum class BleScanShimMode : uint8_t {
  kPassive,
  kActive,
};

struct BleScanShimContext {
  BleScanShimMode mode;
  bool ready;
  bool done;
  uint32_t deadlineMs;
  uint32_t postAdvDeadlineMs;
  BleAdvertisingChannel pseudoChannel;
  BleScanPacket passivePacket;
  uint8_t passivePayload[31];
  BleActiveScanResult activeResult;
};

BleScanShimContext* g_bleScanContext = nullptr;

constexpr size_t kBleMaxSerializedPayloadLen = 31U;
constexpr size_t kBleMaxAdStructures = 16U;
constexpr uint32_t kBleLegacyOneShotAdvInterval = 0x0020U;
constexpr uint32_t kBleLegacyOneShotWindowMs = 40U;

BleAdvertisingChannel bleAdvertisingChannelFromIndex(uint8_t index) {
  switch (index % 3U) {
    case 1U:
      return BleAdvertisingChannel::k38;
    case 2U:
      return BleAdvertisingChannel::k39;
    case 0U:
    default:
      return BleAdvertisingChannel::k37;
  }
}

uint32_t bleScanWindowMsFromSpin(uint64_t spinLimit,
                                 uint32_t minimumMs,
                                 uint32_t maximumMs = 1500U) {
  uint32_t windowMs = static_cast<uint32_t>(spinLimit / 10000ULL);
  if (windowMs < minimumMs) {
    windowMs = minimumMs;
  }
  if (windowMs > maximumMs) {
    windowMs = maximumMs;
  }
  return windowMs;
}

uint8_t bleRawPduTypeFromGapType(uint8_t advType) {
  switch (advType) {
    case BT_GAP_ADV_TYPE_ADV_IND:
      return static_cast<uint8_t>(BleAdvPduType::kAdvInd);
    case BT_GAP_ADV_TYPE_ADV_DIRECT_IND:
      return static_cast<uint8_t>(BleAdvPduType::kAdvDirectInd);
    case BT_GAP_ADV_TYPE_ADV_SCAN_IND:
      return static_cast<uint8_t>(BleAdvPduType::kAdvScanInd);
    case BT_GAP_ADV_TYPE_ADV_NONCONN_IND:
      return static_cast<uint8_t>(BleAdvPduType::kAdvNonConnInd);
    case BT_GAP_ADV_TYPE_SCAN_RSP:
      return static_cast<uint8_t>(BleAdvPduType::kScanRsp);
    default:
      return 0xFFU;
  }
}

uint8_t blePduHeaderFromScanArgs(const bt_addr_le_t* address, uint8_t advType) {
  if (address == nullptr) {
    return 0U;
  }

  uint8_t header = bleRawPduTypeFromGapType(advType);
  if (header == 0xFFU) {
    header = 0U;
  }
  if (address->type == BT_ADDR_LE_RANDOM) {
    header |= (1U << 6U);
  }
  return header;
}

uint8_t bleCopyPayloadWithAddress(uint8_t* destination,
                                  size_t destinationSize,
                                  const bt_addr_le_t* address,
                                  const net_buf_simple* buffer) {
  if (destination == nullptr || destinationSize == 0U) {
    return 0U;
  }

  size_t used = 0U;
  memset(destination, 0, destinationSize);

  if (address != nullptr && destinationSize >= 6U) {
    memcpy(destination, address->a.val, 6U);
    used = 6U;
  }

  if (buffer != nullptr && buffer->len > 0U && used < destinationSize) {
    const size_t remaining = destinationSize - used;
    const size_t copyLen = (buffer->len < remaining) ? buffer->len : remaining;
    memcpy(&destination[used], buffer->data, copyLen);
    used += copyLen;
  }

  return static_cast<uint8_t>(used);
}

bool bleScanAddressMatches(const bt_addr_le_t* address,
                           const uint8_t expected[6],
                           bool expectedRandom) {
  if (address == nullptr || expected == nullptr) {
    return false;
  }
  if ((address->type == BT_ADDR_LE_RANDOM) != expectedRandom) {
    return false;
  }
  return memcmp(address->a.val, expected, 6U) == 0;
}

void bleScanShimCallback(const bt_addr_le_t* address,
                         int8_t rssi,
                         uint8_t advType,
                         net_buf_simple* buffer) {
  BleScanShimContext* context = g_bleScanContext;
  if (context == nullptr || address == nullptr || buffer == nullptr) {
    return;
  }

  const uint8_t rawPduType = bleRawPduTypeFromGapType(advType);
  if (rawPduType == 0xFFU) {
    return;
  }

  if (context->mode == BleScanShimMode::kPassive) {
    if (rawPduType == static_cast<uint8_t>(BleAdvPduType::kScanRsp)) {
      return;
    }
    context->passivePacket.channel = context->pseudoChannel;
    context->passivePacket.rssiDbm = rssi;
    context->passivePacket.pduHeader = blePduHeaderFromScanArgs(address, advType);
    context->passivePacket.length =
        bleCopyPayloadWithAddress(context->passivePayload,
                                  sizeof(context->passivePayload),
                                  address,
                                  buffer);
    context->passivePacket.payload = context->passivePayload;
    context->ready = true;
    context->done = true;
    return;
  }

  const bool isScanResponse =
      (rawPduType == static_cast<uint8_t>(BleAdvPduType::kScanRsp));
  if (!context->activeResult.scanResponseReceived &&
      !context->activeResult.advPayloadLength) {
    if (isScanResponse) {
      return;
    }

    context->activeResult.channel = context->pseudoChannel;
    context->activeResult.advRssiDbm = rssi;
    context->activeResult.advHeader = blePduHeaderFromScanArgs(address, advType);
    context->activeResult.advertiserAddressRandom =
        (address->type == BT_ADDR_LE_RANDOM);
    memcpy(context->activeResult.advertiserAddress, address->a.val, 6U);
    context->activeResult.advPayloadLength =
        bleCopyPayloadWithAddress(context->activeResult.advPayload,
                                  sizeof(context->activeResult.advPayload),
                                  address,
                                  buffer);
    context->ready = true;

    if (rawPduType != static_cast<uint8_t>(BleAdvPduType::kAdvInd) &&
        rawPduType != static_cast<uint8_t>(BleAdvPduType::kAdvScanInd)) {
      context->done = true;
    }
    return;
  }

  if (!isScanResponse) {
    return;
  }
  if (!bleScanAddressMatches(address,
                             context->activeResult.advertiserAddress,
                             context->activeResult.advertiserAddressRandom)) {
    return;
  }

  context->activeResult.scanResponseReceived = true;
  context->activeResult.scanRspRssiDbm = rssi;
  context->activeResult.scanRspHeader = blePduHeaderFromScanArgs(address, advType);
  context->activeResult.scanRspPayloadLength =
      bleCopyPayloadWithAddress(context->activeResult.scanRspPayload,
                                sizeof(context->activeResult.scanRspPayload),
                                address,
                                buffer);
  context->done = true;
}

void extractBleLocalName(const uint8_t* payload,
                         size_t payloadLen,
                         char* destination,
                         size_t destinationSize) {
  if (destination == nullptr || destinationSize == 0U) {
    return;
  }

  size_t offset = 0U;
  while (payload != nullptr && offset < payloadLen) {
    const uint8_t fieldLen = payload[offset++];
    if (fieldLen == 0U) {
      break;
    }
    if ((offset + fieldLen) > payloadLen) {
      break;
    }

    const uint8_t fieldType = payload[offset++];
    const size_t dataLen = static_cast<size_t>(fieldLen - 1U);
    if ((fieldType == BT_DATA_NAME_COMPLETE ||
         fieldType == BT_DATA_NAME_SHORTENED) &&
        dataLen > 0U) {
      const size_t copyLen =
          (dataLen < (destinationSize - 1U)) ? dataLen : (destinationSize - 1U);
      memcpy(destination, &payload[offset], copyLen);
      destination[copyLen] = '\0';
      return;
    }
    offset += dataLen;
  }
}

bool parseBleAdvertisingPayload(const uint8_t* payload,
                                size_t payloadLen,
                                bt_data* entries,
                                size_t maxEntries,
                                size_t* entryCount) {
  if (entries == nullptr || entryCount == nullptr) {
    return false;
  }
  if (payloadLen > 0U && payload == nullptr) {
    return false;
  }

  size_t count = 0U;
  size_t offset = 0U;
  while (offset < payloadLen) {
    const uint8_t fieldLen = payload[offset++];
    if (fieldLen == 0U) {
      break;
    }
    if (fieldLen < 1U || (offset + fieldLen) > payloadLen || count >= maxEntries) {
      return false;
    }

    entries[count].type = payload[offset++];
    entries[count].data_len = static_cast<uint8_t>(fieldLen - 1U);
    entries[count].data = &payload[offset];
    offset += static_cast<size_t>(fieldLen - 1U);
    ++count;
  }

  *entryCount = count;
  return true;
}

BleAddressType bleAddressTypeFromZephyr(uint8_t type) {
  return (type == BT_ADDR_LE_RANDOM) ? BleAddressType::kRandomStatic
                                     : BleAddressType::kPublic;
}

uint8_t zephyrAddressTypeFromBle(BleAddressType type) {
  return (type == BleAddressType::kRandomStatic) ? BT_ADDR_LE_RANDOM
                                                 : BT_ADDR_LE_PUBLIC;
}

struct BleLinkShimState {
  bt_conn* conn;
  bool callbacksRegistered;
  bool connected;
  bool connectEventPending;
  bool disconnectEventPending;
  bool encrypted;
  bool parameterUpdatePending;
  uint8_t disconnectReason;
  bool disconnectReasonValid;
  bool disconnectReasonRemote;
  bt_addr_le_t peerAddress;
  uint16_t intervalUnits;
  uint16_t latency;
  uint16_t supervisionTimeoutUnits;
  uint16_t eventCounter;
  uint8_t nextDataChannel;
  uint32_t lastEventMs;
  int8_t rssiDbm;
  uint8_t pendingRxPayload[32];
  uint8_t pendingRxPayloadLen;
  uint8_t pendingRxAttOpcode;
  uint8_t pendingTxPayload[32];
  uint8_t pendingTxPayloadLen;
  uint8_t pendingTxAttOpcode;
};

BleLinkShimState g_bleLinkState = {};
bt_conn_cb g_bleConnCallbacks = {};
bt_conn_auth_cb g_bleAuthCallbacks = {};
bt_conn_auth_info_cb g_bleAuthInfoCallbacks = {};

struct BleSecurityShimState {
  bool authCallbacksRegistered;
  bool authInfoCallbacksRegistered;
  bool bondRecordValid;
  bool localIdentityValid;
  bool pairingConfirmPending;
  uint8_t localIdentityId;
  bt_conn* pairingConfirmConn;
  BleBondRecord bondRecord;
  BleEncryptionDebugCounters encDebug;
  BleBondLoadCallback bondLoadCallback;
  BleBondSaveCallback bondSaveCallback;
  BleBondClearCallback bondClearCallback;
  void* bondCallbackContext;
  BleTraceCallback traceCallback;
  void* traceContext;
};

BleSecurityShimState g_bleSecurityState = {};

constexpr uint8_t kBleDefaultChannelMap[5] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x1FU};

void bleTraceLog(const char* format, ...) {
  if (g_bleSecurityState.traceCallback == nullptr || format == nullptr) {
    return;
  }

  char buffer[160];
  va_list args;
  va_start(args, format);
  const int length = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  if (length < 0) {
    return;
  }

  buffer[sizeof(buffer) - 1U] = '\0';
  g_bleSecurityState.traceCallback(buffer, g_bleSecurityState.traceContext);
}

bool getIdentityAddressById(uint8_t identityId, bt_addr_le_t* outAddress) {
  if (outAddress == nullptr) {
    return false;
  }

  bt_addr_le_t addresses[CONFIG_BT_ID_MAX];
  memset(addresses, 0, sizeof(addresses));
  size_t count = ARRAY_SIZE(addresses);
  bt_id_get(addresses, &count);
  if (identityId >= count) {
    return false;
  }

  *outAddress = addresses[identityId];
  return true;
}

bool findIdentityIdByAddress(const uint8_t address[6], BleAddressType type,
                             uint8_t* outIdentityId) {
  if (address == nullptr || outIdentityId == nullptr) {
    return false;
  }

  bt_addr_le_t addresses[CONFIG_BT_ID_MAX];
  memset(addresses, 0, sizeof(addresses));
  size_t count = ARRAY_SIZE(addresses);
  bt_id_get(addresses, &count);

  const uint8_t zephyrType = zephyrAddressTypeFromBle(type);
  for (size_t index = 0; index < count; ++index) {
    if (addresses[index].type != zephyrType) {
      continue;
    }
    if (memcmp(addresses[index].a.val, address, 6U) == 0) {
      *outIdentityId = static_cast<uint8_t>(index);
      return true;
    }
  }

  return false;
}

void clearCachedBondRecord() {
  memset(&g_bleSecurityState.bondRecord, 0, sizeof(g_bleSecurityState.bondRecord));
  g_bleSecurityState.bondRecordValid = false;
}

void clearPendingPairingConfirm() {
  g_bleSecurityState.pairingConfirmPending = false;
  if (g_bleSecurityState.pairingConfirmConn != nullptr) {
    bt_conn_unref(g_bleSecurityState.pairingConfirmConn);
    g_bleSecurityState.pairingConfirmConn = nullptr;
  }
}

void queuePendingPairingConfirm(struct bt_conn* conn) {
  if (conn == nullptr) {
    return;
  }

  if (g_bleSecurityState.pairingConfirmConn != nullptr &&
      g_bleSecurityState.pairingConfirmConn != conn) {
    bt_conn_unref(g_bleSecurityState.pairingConfirmConn);
    g_bleSecurityState.pairingConfirmConn = nullptr;
  }

  if (g_bleSecurityState.pairingConfirmConn == nullptr) {
    g_bleSecurityState.pairingConfirmConn = bt_conn_ref(conn);
  }
  g_bleSecurityState.pairingConfirmPending = true;
}

void servicePendingBleSecurityActions() {
  if (!g_bleSecurityState.pairingConfirmPending ||
      g_bleSecurityState.pairingConfirmConn == nullptr) {
    return;
  }

  bt_conn* conn = g_bleSecurityState.pairingConfirmConn;
  g_bleSecurityState.pairingConfirmConn = nullptr;
  g_bleSecurityState.pairingConfirmPending = false;

  const int err = bt_conn_auth_pairing_confirm(conn);
  if (err != 0) {
    bleTraceLog("pairing-confirm err=%d", err);
  } else {
    bleTraceLog("pairing-confirm accepted");
  }
  bt_conn_unref(conn);
}

bool cacheBondRecord(const BleBondRecord& record, uint8_t identityId, bool invokeSave) {
  g_bleSecurityState.bondRecord = record;
  g_bleSecurityState.bondRecordValid = true;
  g_bleSecurityState.localIdentityValid = true;
  g_bleSecurityState.localIdentityId = identityId;

  if (invokeSave && g_bleSecurityState.bondSaveCallback != nullptr) {
    if (!g_bleSecurityState.bondSaveCallback(&record,
                                             g_bleSecurityState.bondCallbackContext)) {
      bleTraceLog("bond-save-callback failed");
      return false;
    }
  }

  return true;
}

bool buildBondRecordFromConn(const bt_conn* conn, BleBondRecord* outRecord,
                             uint8_t* outIdentityId = nullptr) {
  if (conn == nullptr || outRecord == nullptr) {
    return false;
  }

  bt_conn_info info;
  memset(&info, 0, sizeof(info));
  if (bt_conn_get_info(conn, &info) != 0 || info.type != BT_CONN_TYPE_LE) {
    return false;
  }

  const bt_addr_le_t* local = (info.le.src != nullptr) ? info.le.src : info.le.local;
  const bt_addr_le_t* peer = (info.le.dst != nullptr) ? info.le.dst : info.le.remote;
  if (local == nullptr || peer == nullptr) {
    return false;
  }

  memset(outRecord, 0, sizeof(*outRecord));
  memcpy(outRecord->localAddress, local->a.val, sizeof(outRecord->localAddress));
  outRecord->localAddressRandom = (local->type == BT_ADDR_LE_RANDOM) ? 1U : 0U;
  memcpy(outRecord->peerAddress, peer->a.val, sizeof(outRecord->peerAddress));
  outRecord->peerAddressRandom = (peer->type == BT_ADDR_LE_RANDOM) ? 1U : 0U;

  const uint8_t keySize = bt_conn_enc_key_size(conn);
  if (keySize >= 7U && keySize <= 16U) {
    outRecord->keySize = keySize;
  }

  if (outIdentityId != nullptr) {
    *outIdentityId = info.id;
  }

  return true;
}

bool refreshCachedBondRecordFromConn(const bt_conn* conn, bool requireBondedPeer,
                                     bool invokeSave) {
  BleBondRecord record{};
  uint8_t identityId = BT_ID_DEFAULT;
  if (!buildBondRecordFromConn(conn, &record, &identityId)) {
    return false;
  }

  bt_addr_le_t peer;
  memset(&peer, 0, sizeof(peer));
  peer.type = record.peerAddressRandom ? BT_ADDR_LE_RANDOM : BT_ADDR_LE_PUBLIC;
  memcpy(peer.a.val, record.peerAddress, sizeof(record.peerAddress));

  if (requireBondedPeer && !bt_le_bond_exists(identityId, &peer)) {
    return false;
  }

  return cacheBondRecord(record, identityId, invokeSave);
}

struct BondLookupContext {
  bool found;
  bt_addr_le_t peer;
};

void findFirstBondCallback(const bt_bond_info* info, void* userData) {
  if (info == nullptr || userData == nullptr) {
    return;
  }

  auto* context = static_cast<BondLookupContext*>(userData);
  if (context->found) {
    return;
  }

  context->found = true;
  context->peer = info->addr;
}

bool loadBondRecordFromIdentity(uint8_t identityId, BleBondRecord* outRecord) {
  if (outRecord == nullptr) {
    return false;
  }

  BondLookupContext context{};
  bt_foreach_bond(identityId, findFirstBondCallback, &context);
  if (!context.found) {
    return false;
  }

  bt_addr_le_t local;
  memset(&local, 0, sizeof(local));
  if (!getIdentityAddressById(identityId, &local)) {
    return false;
  }

  memset(outRecord, 0, sizeof(*outRecord));
  memcpy(outRecord->localAddress, local.a.val, sizeof(outRecord->localAddress));
  outRecord->localAddressRandom = (local.type == BT_ADDR_LE_RANDOM) ? 1U : 0U;
  memcpy(outRecord->peerAddress, context.peer.a.val, sizeof(outRecord->peerAddress));
  outRecord->peerAddressRandom = (context.peer.type == BT_ADDR_LE_RANDOM) ? 1U : 0U;
  return true;
}

uint16_t bleIntervalUnitsFromInfo(const bt_conn_info& info) {
  if (info.type != BT_CONN_TYPE_LE) {
    return 0U;
  }

  if (info.le.interval_us != 0U) {
    return static_cast<uint16_t>((info.le.interval_us + 1249U) / 1250U);
  }
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  return info.le.interval;
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
}

void clearBlePendingTx() {
  g_bleLinkState.pendingTxPayloadLen = 0U;
  g_bleLinkState.pendingTxAttOpcode = 0U;
}

void clearBlePendingRx() {
  g_bleLinkState.pendingRxPayloadLen = 0U;
  g_bleLinkState.pendingRxAttOpcode = 0U;
}

bool queueBleAttPayload(uint8_t* payloadBuffer,
                        uint8_t* payloadLength,
                        uint8_t* attOpcode,
                        uint8_t opcode,
                        uint16_t handle,
                        const uint8_t* value,
                        uint8_t valueLength) {
  if (payloadBuffer == nullptr || payloadLength == nullptr || attOpcode == nullptr ||
      (valueLength > 0U && value == nullptr)) {
    return false;
  }

  const uint8_t totalLength = static_cast<uint8_t>(valueLength + 7U);
  if (totalLength > 32U) {
    return false;
  }

  memset(payloadBuffer, 0, 32U);
  payloadBuffer[0] = static_cast<uint8_t>(valueLength + 3U);
  payloadBuffer[1] = 0x00U;
  payloadBuffer[2] = 0x04U;
  payloadBuffer[3] = 0x00U;
  payloadBuffer[4] = opcode;
  payloadBuffer[5] = static_cast<uint8_t>(handle & 0xFFU);
  payloadBuffer[6] = static_cast<uint8_t>((handle >> 8U) & 0xFFU);
  if (valueLength > 0U) {
    memcpy(&payloadBuffer[7], value, valueLength);
  }
  *payloadLength = totalLength;
  *attOpcode = opcode;
  return true;
}

void updateBleLinkInfo(const bt_conn* conn) {
  if (conn == nullptr) {
    return;
  }

  bt_conn_info info;
  memset(&info, 0, sizeof(info));
  if (bt_conn_get_info(conn, &info) == 0 && info.type == BT_CONN_TYPE_LE) {
    g_bleLinkState.intervalUnits = bleIntervalUnitsFromInfo(info);
    if (g_bleLinkState.intervalUnits == 0U) {
      g_bleLinkState.intervalUnits = 24U;
    }
    g_bleLinkState.latency = info.le.latency;
    g_bleLinkState.supervisionTimeoutUnits = info.le.timeout;
    g_bleLinkState.encrypted = (info.security.level > BT_SECURITY_L1);
  }

  const bt_addr_le_t* peerAddress = bt_conn_get_dst(conn);
  if (peerAddress != nullptr) {
    memcpy(&g_bleLinkState.peerAddress, peerAddress, sizeof(g_bleLinkState.peerAddress));
  } else {
    memset(&g_bleLinkState.peerAddress, 0, sizeof(g_bleLinkState.peerAddress));
  }
}

void bleLinkConnectedCallback(struct bt_conn* conn, uint8_t err) {
  if (err != 0U) {
    return;
  }

  if (g_bleLinkState.conn != nullptr && g_bleLinkState.conn != conn) {
    bt_conn_unref(g_bleLinkState.conn);
  }

  g_bleLinkState.conn = bt_conn_ref(conn);
  g_bleLinkState.connected = true;
  g_bleLinkState.connectEventPending = true;
  g_bleLinkState.disconnectEventPending = false;
  g_bleLinkState.disconnectReason = 0U;
  g_bleLinkState.disconnectReasonValid = false;
  g_bleLinkState.disconnectReasonRemote = false;
  g_bleLinkState.parameterUpdatePending = false;
  g_bleLinkState.eventCounter = 0U;
  g_bleLinkState.nextDataChannel = 0U;
  g_bleLinkState.lastEventMs = 0U;
  g_bleLinkState.rssiDbm = 0;
  clearBlePendingRx();
  clearBlePendingTx();
  updateBleLinkInfo(conn);
}

void bleLinkDisconnectedCallback(struct bt_conn* conn, uint8_t reason) {
  updateBleLinkInfo(conn);
  g_bleLinkState.connected = false;
  g_bleLinkState.disconnectEventPending = true;
  g_bleLinkState.disconnectReason = reason;
  g_bleLinkState.disconnectReasonValid = true;
  g_bleLinkState.disconnectReasonRemote =
      (reason != BT_HCI_ERR_LOCALHOST_TERM_CONN);
  g_bleLinkState.parameterUpdatePending = false;
  clearBlePendingRx();
  clearPendingPairingConfirm();

  if (g_bleLinkState.conn != nullptr) {
    bt_conn_unref(g_bleLinkState.conn);
    g_bleLinkState.conn = nullptr;
  }
}

void bleLinkLeParamUpdatedCallback(struct bt_conn* conn,
                                   uint16_t interval,
                                   uint16_t latency,
                                   uint16_t timeout) {
  (void)conn;
  g_bleLinkState.intervalUnits = interval;
  g_bleLinkState.latency = latency;
  g_bleLinkState.supervisionTimeoutUnits = timeout;
  g_bleLinkState.parameterUpdatePending = true;
}

#if defined(CONFIG_BT_SMP) || defined(CONFIG_BT_CLASSIC)
void bleLinkSecurityChangedCallback(struct bt_conn* conn,
                                    bt_security_t level,
                                    enum bt_security_err err) {
  (void)conn;
  g_bleLinkState.encrypted =
      (err == BT_SECURITY_ERR_SUCCESS) && (level > BT_SECURITY_L1);
}
#endif

void blePairingCompleteCallback(struct bt_conn* conn, bool bonded) {
  (void)conn;
  (void)bonded;
  clearPendingPairingConfirm();
}

void blePairingFailedCallback(struct bt_conn* conn, enum bt_security_err reason) {
  (void)conn;
  (void)reason;
  clearPendingPairingConfirm();
}

void bleAuthCancelCallback(struct bt_conn* conn) {
  (void)conn;
  clearPendingPairingConfirm();
}

void bleAuthPairingConfirmCallback(struct bt_conn* conn) {
  queuePendingPairingConfirm(conn);
}

void bleBondDeletedCallback(uint8_t id, const bt_addr_le_t* peer) {
  (void)peer;
  if (g_bleSecurityState.bondRecordValid && g_bleSecurityState.localIdentityValid &&
      g_bleSecurityState.localIdentityId == id) {
    clearCachedBondRecord();
  }
}

void ensureBleLinkCallbacksRegistered() {
  if (g_bleLinkState.callbacksRegistered) {
    return;
  }

  memset(&g_bleConnCallbacks, 0, sizeof(g_bleConnCallbacks));
  g_bleConnCallbacks.connected = bleLinkConnectedCallback;
  g_bleConnCallbacks.disconnected = bleLinkDisconnectedCallback;
  g_bleConnCallbacks.le_param_updated = bleLinkLeParamUpdatedCallback;
#if defined(CONFIG_BT_SMP) || defined(CONFIG_BT_CLASSIC)
  g_bleConnCallbacks.security_changed = bleLinkSecurityChangedCallback;
#endif
  if (bt_conn_cb_register(&g_bleConnCallbacks) == 0) {
    g_bleLinkState.callbacksRegistered = true;
  }
}

void ensureBleSecurityCallbacksRegistered() {
  if (g_bleSecurityState.authCallbacksRegistered &&
      g_bleSecurityState.authInfoCallbacksRegistered) {
    return;
  }

  if (!g_bleSecurityState.authCallbacksRegistered) {
    memset(&g_bleAuthCallbacks, 0, sizeof(g_bleAuthCallbacks));
    g_bleAuthCallbacks.cancel = bleAuthCancelCallback;
    g_bleAuthCallbacks.pairing_confirm = bleAuthPairingConfirmCallback;
    if (bt_conn_auth_cb_register(&g_bleAuthCallbacks) == 0) {
      g_bleSecurityState.authCallbacksRegistered = true;
    }
  }

  if (!g_bleSecurityState.authInfoCallbacksRegistered) {
    memset(&g_bleAuthInfoCallbacks, 0, sizeof(g_bleAuthInfoCallbacks));
    g_bleAuthInfoCallbacks.pairing_complete = blePairingCompleteCallback;
    g_bleAuthInfoCallbacks.pairing_failed = blePairingFailedCallback;
    g_bleAuthInfoCallbacks.bond_deleted = bleBondDeletedCallback;
    if (bt_conn_auth_info_cb_register(&g_bleAuthInfoCallbacks) == 0) {
      g_bleSecurityState.authInfoCallbacksRegistered = true;
    }
  }
}

uint32_t bleSyntheticEventIntervalMs() {
  uint16_t intervalUnits = g_bleLinkState.intervalUnits;
  if (intervalUnits == 0U) {
    intervalUnits = 24U;
  }

  uint32_t intervalMs = (static_cast<uint32_t>(intervalUnits) * 5U + 3U) / 4U;
  if (intervalMs == 0U) {
    intervalMs = 1U;
  }
  return intervalMs;
}

#if !defined(CONFIG_BT_BAS)
bool queueBleAttNotification(const struct bt_gatt_attr* attr,
                             const uint8_t* value,
                             uint8_t valueLength) {
  if (attr == nullptr || value == nullptr || valueLength == 0U) {
    return false;
  }

  const uint16_t handle = bt_gatt_attr_get_handle(attr);
  if (handle == 0U) {
    return false;
  }

  clearBlePendingTx();
  return queueBleAttPayload(g_bleLinkState.pendingTxPayload,
                            &g_bleLinkState.pendingTxPayloadLen,
                            &g_bleLinkState.pendingTxAttOpcode, 0x1BU, handle,
                            value, valueLength);
}
#endif

bool bleVisibleConnectionActive() {
  return g_bleLinkState.connected || g_bleLinkState.disconnectEventPending;
}

constexpr uint8_t kBleDataPduMaxPayload = 27U;
constexpr uint8_t kBleMicLen = 4U;

constexpr uint8_t kAesSbox[256] = {
    0x63, 0x7C, 0x77, 0x7B, 0xF2, 0x6B, 0x6F, 0xC5, 0x30, 0x01, 0x67, 0x2B,
    0xFE, 0xD7, 0xAB, 0x76, 0xCA, 0x82, 0xC9, 0x7D, 0xFA, 0x59, 0x47, 0xF0,
    0xAD, 0xD4, 0xA2, 0xAF, 0x9C, 0xA4, 0x72, 0xC0, 0xB7, 0xFD, 0x93, 0x26,
    0x36, 0x3F, 0xF7, 0xCC, 0x34, 0xA5, 0xE5, 0xF1, 0x71, 0xD8, 0x31, 0x15,
    0x04, 0xC7, 0x23, 0xC3, 0x18, 0x96, 0x05, 0x9A, 0x07, 0x12, 0x80, 0xE2,
    0xEB, 0x27, 0xB2, 0x75, 0x09, 0x83, 0x2C, 0x1A, 0x1B, 0x6E, 0x5A, 0xA0,
    0x52, 0x3B, 0xD6, 0xB3, 0x29, 0xE3, 0x2F, 0x84, 0x53, 0xD1, 0x00, 0xED,
    0x20, 0xFC, 0xB1, 0x5B, 0x6A, 0xCB, 0xBE, 0x39, 0x4A, 0x4C, 0x58, 0xCF,
    0xD0, 0xEF, 0xAA, 0xFB, 0x43, 0x4D, 0x33, 0x85, 0x45, 0xF9, 0x02, 0x7F,
    0x50, 0x3C, 0x9F, 0xA8, 0x51, 0xA3, 0x40, 0x8F, 0x92, 0x9D, 0x38, 0xF5,
    0xBC, 0xB6, 0xDA, 0x21, 0x10, 0xFF, 0xF3, 0xD2, 0xCD, 0x0C, 0x13, 0xEC,
    0x5F, 0x97, 0x44, 0x17, 0xC4, 0xA7, 0x7E, 0x3D, 0x64, 0x5D, 0x19, 0x73,
    0x60, 0x81, 0x4F, 0xDC, 0x22, 0x2A, 0x90, 0x88, 0x46, 0xEE, 0xB8, 0x14,
    0xDE, 0x5E, 0x0B, 0xDB, 0xE0, 0x32, 0x3A, 0x0A, 0x49, 0x06, 0x24, 0x5C,
    0xC2, 0xD3, 0xAC, 0x62, 0x91, 0x95, 0xE4, 0x79, 0xE7, 0xC8, 0x37, 0x6D,
    0x8D, 0xD5, 0x4E, 0xA9, 0x6C, 0x56, 0xF4, 0xEA, 0x65, 0x7A, 0xAE, 0x08,
    0xBA, 0x78, 0x25, 0x2E, 0x1C, 0xA6, 0xB4, 0xC6, 0xE8, 0xDD, 0x74, 0x1F,
    0x4B, 0xBD, 0x8B, 0x8A, 0x70, 0x3E, 0xB5, 0x66, 0x48, 0x03, 0xF6, 0x0E,
    0x61, 0x35, 0x57, 0xB9, 0x86, 0xC1, 0x1D, 0x9E, 0xE1, 0xF8, 0x98, 0x11,
    0x69, 0xD9, 0x8E, 0x94, 0x9B, 0x1E, 0x87, 0xE9, 0xCE, 0x55, 0x28, 0xDF,
    0x8C, 0xA1, 0x89, 0x0D, 0xBF, 0xE6, 0x42, 0x68, 0x41, 0x99, 0x2D, 0x0F,
    0xB0, 0x54, 0xBB, 0x16};

constexpr uint8_t kAesRcon[11] = {0x00, 0x01, 0x02, 0x04, 0x08, 0x10,
                                  0x20, 0x40, 0x80, 0x1B, 0x36};

using AesState = uint8_t[4][4];

uint8_t aesXtime(uint8_t value) {
  return static_cast<uint8_t>((value << 1U) ^
                              (((value >> 7U) & 0x01U) * 0x1BU));
}

void aesAddRoundKey(uint8_t round, AesState* state, const uint8_t* roundKey) {
  for (uint8_t row = 0U; row < 4U; ++row) {
    for (uint8_t column = 0U; column < 4U; ++column) {
      (*state)[row][column] ^= roundKey[(round * 16U) + (row * 4U) + column];
    }
  }
}

void aesSubBytes(AesState* state) {
  for (uint8_t row = 0U; row < 4U; ++row) {
    for (uint8_t column = 0U; column < 4U; ++column) {
      (*state)[column][row] = kAesSbox[(*state)[column][row]];
    }
  }
}

void aesShiftRows(AesState* state) {
  uint8_t temp = (*state)[0][1];
  (*state)[0][1] = (*state)[1][1];
  (*state)[1][1] = (*state)[2][1];
  (*state)[2][1] = (*state)[3][1];
  (*state)[3][1] = temp;

  temp = (*state)[0][2];
  (*state)[0][2] = (*state)[2][2];
  (*state)[2][2] = temp;
  temp = (*state)[1][2];
  (*state)[1][2] = (*state)[3][2];
  (*state)[3][2] = temp;

  temp = (*state)[0][3];
  (*state)[0][3] = (*state)[3][3];
  (*state)[3][3] = (*state)[2][3];
  (*state)[2][3] = (*state)[1][3];
  (*state)[1][3] = temp;
}

void aesMixColumns(AesState* state) {
  for (uint8_t row = 0U; row < 4U; ++row) {
    uint8_t first = (*state)[row][0];
    uint8_t mix = static_cast<uint8_t>((*state)[row][0] ^ (*state)[row][1] ^
                                       (*state)[row][2] ^ (*state)[row][3]);
    uint8_t partial =
        static_cast<uint8_t>((*state)[row][0] ^ (*state)[row][1]);
    partial = aesXtime(partial);
    (*state)[row][0] ^= static_cast<uint8_t>(partial ^ mix);
    partial = static_cast<uint8_t>((*state)[row][1] ^ (*state)[row][2]);
    partial = aesXtime(partial);
    (*state)[row][1] ^= static_cast<uint8_t>(partial ^ mix);
    partial = static_cast<uint8_t>((*state)[row][2] ^ (*state)[row][3]);
    partial = aesXtime(partial);
    (*state)[row][2] ^= static_cast<uint8_t>(partial ^ mix);
    partial = static_cast<uint8_t>((*state)[row][3] ^ first);
    partial = aesXtime(partial);
    (*state)[row][3] ^= static_cast<uint8_t>(partial ^ mix);
  }
}

void aesKeyExpansion128(const uint8_t key[16], uint8_t roundKey[176]) {
  for (uint8_t index = 0U; index < 16U; ++index) {
    roundKey[index] = key[index];
  }

  uint8_t temp[4] = {0U, 0U, 0U, 0U};
  uint16_t generated = 16U;
  uint8_t rconIndex = 1U;
  while (generated < 176U) {
    for (uint8_t index = 0U; index < 4U; ++index) {
      temp[index] = roundKey[static_cast<uint16_t>(generated - 4U + index)];
    }

    if ((generated % 16U) == 0U) {
      const uint8_t rotated = temp[0];
      temp[0] = temp[1];
      temp[1] = temp[2];
      temp[2] = temp[3];
      temp[3] = rotated;
      temp[0] = kAesSbox[temp[0]];
      temp[1] = kAesSbox[temp[1]];
      temp[2] = kAesSbox[temp[2]];
      temp[3] = kAesSbox[temp[3]];
      temp[0] ^= kAesRcon[rconIndex];
      ++rconIndex;
    }

    for (uint8_t index = 0U; index < 4U; ++index) {
      roundKey[generated] =
          static_cast<uint8_t>(roundKey[generated - 16U] ^ temp[index]);
      ++generated;
    }
  }
}

void aesEncryptBlockBe(const uint8_t key[16],
                       const uint8_t plaintext[16],
                       uint8_t out[16]) {
  uint8_t roundKey[176] = {0U};
  uint8_t stateBuffer[16] = {0U};
  aesKeyExpansion128(key, roundKey);
  memcpy(stateBuffer, plaintext, sizeof(stateBuffer));

  auto* state = reinterpret_cast<AesState*>(&stateBuffer[0]);
  aesAddRoundKey(0U, state, roundKey);
  for (uint8_t round = 1U; round < 10U; ++round) {
    aesSubBytes(state);
    aesShiftRows(state);
    aesMixColumns(state);
    aesAddRoundKey(round, state, roundKey);
  }
  aesSubBytes(state);
  aesShiftRows(state);
  aesAddRoundKey(10U, state, roundKey);
  memcpy(out, stateBuffer, sizeof(stateBuffer));
}

void reverseBytes(const uint8_t* input, uint8_t* output, uint8_t length) {
  if (input == nullptr || output == nullptr) {
    return;
  }

  for (uint8_t index = 0U; index < length; ++index) {
    output[index] = input[static_cast<uint8_t>(length - 1U - index)];
  }
}

void xor16(const uint8_t* left, const uint8_t* right, uint8_t* out) {
  if (left == nullptr || right == nullptr || out == nullptr) {
    return;
  }

  for (uint8_t index = 0U; index < 16U; ++index) {
    out[index] = static_cast<uint8_t>(left[index] ^ right[index]);
  }
}

bool aesEncryptCcmBlock(const uint8_t keyLe[16],
                        const uint8_t plaintext[16],
                        uint8_t out[16]) {
  if (keyLe == nullptr || plaintext == nullptr || out == nullptr) {
    return false;
  }
  aesEncryptBlockBe(keyLe, plaintext, out);
  return true;
}

void buildBleCcmNonce(const uint8_t iv[8],
                      uint64_t counter,
                      uint8_t direction,
                      uint8_t nonce[13]) {
  if (iv == nullptr || nonce == nullptr) {
    return;
  }

  const uint64_t maskedCounter = counter & 0x7FFFFFFFFFULL;
  nonce[0] = static_cast<uint8_t>(maskedCounter & 0xFFU);
  nonce[1] = static_cast<uint8_t>((maskedCounter >> 8U) & 0xFFU);
  nonce[2] = static_cast<uint8_t>((maskedCounter >> 16U) & 0xFFU);
  nonce[3] = static_cast<uint8_t>((maskedCounter >> 24U) & 0xFFU);
  nonce[4] = static_cast<uint8_t>(((maskedCounter >> 32U) & 0x7FU) |
                                  ((direction & 0x01U) << 7U));
  memcpy(&nonce[5], iv, 8U);
}

void buildCcmCtrBlock(const uint8_t nonce[13], uint16_t counter, uint8_t out[16]) {
  if (nonce == nullptr || out == nullptr) {
    return;
  }

  memset(out, 0, 16U);
  out[0] = 0x01U;
  memcpy(&out[1], nonce, 13U);
  out[14] = static_cast<uint8_t>((counter >> 8U) & 0xFFU);
  out[15] = static_cast<uint8_t>(counter & 0xFFU);
}

bool bleComputeCcmMic(const uint8_t key[16],
                      const uint8_t nonce[13],
                      uint8_t header,
                      const uint8_t* payload,
                      uint8_t payloadLen,
                      uint8_t adataMask,
                      uint8_t outMic[4]) {
  if (key == nullptr || nonce == nullptr || outMic == nullptr) {
    return false;
  }
  if (payloadLen > 0U && payload == nullptr) {
    return false;
  }

  uint8_t x[16] = {0U};
  uint8_t block[16] = {0U};
  uint8_t tmp[16] = {0U};

  block[0] = 0x49U;
  memcpy(&block[1], nonce, 13U);
  block[14] = 0x00U;
  block[15] = payloadLen;
  xor16(x, block, tmp);
  if (!aesEncryptCcmBlock(key, tmp, x)) {
    return false;
  }

  memset(block, 0, sizeof(block));
  block[0] = 0x00U;
  block[1] = 0x01U;
  block[2] = static_cast<uint8_t>(header & adataMask);
  xor16(x, block, tmp);
  if (!aesEncryptCcmBlock(key, tmp, x)) {
    return false;
  }

  uint8_t offset = 0U;
  while (offset < payloadLen) {
    memset(block, 0, sizeof(block));
    const uint8_t remaining = static_cast<uint8_t>(payloadLen - offset);
    const uint8_t chunk = (remaining < 16U) ? remaining : 16U;
    memcpy(block, &payload[offset], chunk);
    xor16(x, block, tmp);
    if (!aesEncryptCcmBlock(key, tmp, x)) {
      return false;
    }
    offset = static_cast<uint8_t>(offset + chunk);
  }

  uint8_t s0[16] = {0U};
  buildCcmCtrBlock(nonce, 0U, block);
  if (!aesEncryptCcmBlock(key, block, s0)) {
    return false;
  }

  for (uint8_t index = 0U; index < 4U; ++index) {
    outMic[index] = static_cast<uint8_t>(x[index] ^ s0[index]);
  }
  return true;
}

bool bleCcmEncryptPayload(const uint8_t key[16],
                          const uint8_t iv[8],
                          uint64_t counter,
                          uint8_t direction,
                          uint8_t header,
                          const uint8_t* plaintext,
                          uint8_t plaintextLen,
                          uint8_t adataMask,
                          uint8_t* outCipherWithMic,
                          uint8_t* outCipherWithMicLen) {
  if (key == nullptr || iv == nullptr || outCipherWithMic == nullptr ||
      outCipherWithMicLen == nullptr) {
    return false;
  }
  if (plaintextLen > 0U && plaintext == nullptr) {
    return false;
  }
  if (plaintextLen > kBleDataPduMaxPayload) {
    return false;
  }

  uint8_t nonce[13] = {0U};
  buildBleCcmNonce(iv, counter, direction, nonce);

  uint8_t offset = 0U;
  uint16_t ctr = 1U;
  while (offset < plaintextLen) {
    uint8_t stream[16] = {0U};
    uint8_t ctrBlock[16] = {0U};
    buildCcmCtrBlock(nonce, ctr, ctrBlock);
    if (!aesEncryptCcmBlock(key, ctrBlock, stream)) {
      return false;
    }

    const uint8_t remaining = static_cast<uint8_t>(plaintextLen - offset);
    const uint8_t chunk = (remaining < 16U) ? remaining : 16U;
    for (uint8_t index = 0U; index < chunk; ++index) {
      outCipherWithMic[offset + index] =
          static_cast<uint8_t>(plaintext[offset + index] ^ stream[index]);
    }
    offset = static_cast<uint8_t>(offset + chunk);
    ++ctr;
  }

  uint8_t mic[4] = {0U};
  if (!bleComputeCcmMic(key, nonce, header, plaintext, plaintextLen, adataMask,
                        mic)) {
    return false;
  }
  memcpy(&outCipherWithMic[plaintextLen], mic, sizeof(mic));
  *outCipherWithMicLen = static_cast<uint8_t>(plaintextLen + kBleMicLen);
  return true;
}

bool bleCcmDecryptPayload(const uint8_t key[16],
                          const uint8_t iv[8],
                          uint64_t counter,
                          uint8_t direction,
                          uint8_t header,
                          const uint8_t* cipherWithMic,
                          uint8_t cipherWithMicLen,
                          uint8_t adataMask,
                          uint8_t* outPlaintext,
                          uint8_t* outPlaintextLen) {
  if (key == nullptr || iv == nullptr || cipherWithMic == nullptr ||
      outPlaintext == nullptr || outPlaintextLen == nullptr) {
    return false;
  }
  if (cipherWithMicLen < kBleMicLen) {
    return false;
  }

  const uint8_t payloadLen = static_cast<uint8_t>(cipherWithMicLen - kBleMicLen);
  if (payloadLen > kBleDataPduMaxPayload) {
    return false;
  }

  uint8_t nonce[13] = {0U};
  buildBleCcmNonce(iv, counter, direction, nonce);

  uint8_t offset = 0U;
  uint16_t ctr = 1U;
  while (offset < payloadLen) {
    uint8_t stream[16] = {0U};
    uint8_t ctrBlock[16] = {0U};
    buildCcmCtrBlock(nonce, ctr, ctrBlock);
    if (!aesEncryptCcmBlock(key, ctrBlock, stream)) {
      return false;
    }

    const uint8_t remaining = static_cast<uint8_t>(payloadLen - offset);
    const uint8_t chunk = (remaining < 16U) ? remaining : 16U;
    for (uint8_t index = 0U; index < chunk; ++index) {
      outPlaintext[offset + index] =
          static_cast<uint8_t>(cipherWithMic[offset + index] ^ stream[index]);
    }
    offset = static_cast<uint8_t>(offset + chunk);
    ++ctr;
  }

  uint8_t expectedMic[4] = {0U};
  if (!bleComputeCcmMic(key, nonce, header, outPlaintext, payloadLen, adataMask,
                        expectedMic)) {
    return false;
  }

  uint8_t diff = 0U;
  for (uint8_t index = 0U; index < kBleMicLen; ++index) {
    diff |= static_cast<uint8_t>(expectedMic[index] ^
                                 cipherWithMic[payloadLen + index]);
  }
  if (diff != 0U) {
    return false;
  }

  *outPlaintextLen = payloadLen;
  return true;
}

bool computeBleAh(const uint8_t irk[16], const uint8_t prand[3], uint8_t hash[3]) {
  if (irk == nullptr || prand == nullptr || hash == nullptr) {
    return false;
  }

  uint8_t keyBe[16] = {0U};
  uint8_t plaintextBe[16] = {0U};
  uint8_t ciphertextBe[16] = {0U};
  reverseBytes(irk, keyBe, sizeof(keyBe));
  plaintextBe[13] = prand[2];
  plaintextBe[14] = prand[1];
  plaintextBe[15] = prand[0];
  aesEncryptBlockBe(keyBe, plaintextBe, ciphertextBe);
  hash[0] = ciphertextBe[15];
  hash[1] = ciphertextBe[14];
  hash[2] = ciphertextBe[13];
  return true;
}

}  // namespace

namespace xiao_nrf54l15 {

bool ClockControl::startHfxo(bool waitForTuned, uint32_t spinLimit) {
  NRF_CLOCK_Type* clock = reinterpret_cast<NRF_CLOCK_Type*>(
      static_cast<uintptr_t>(nrf54l15::CLOCK_BASE));
  if (clock == nullptr) {
    return false;
  }

  const bool alreadyRunning =
      (((clock->XO.STAT & CLOCK_XO_STAT_STATE_Msk) >> CLOCK_XO_STAT_STATE_Pos) ==
       CLOCK_XO_STAT_STATE_Running);
  if (alreadyRunning) {
    return true;
  }

  clock->EVENTS_XOSTARTED = 0U;
  clock->EVENTS_XOTUNED = 0U;
  clock->EVENTS_XOTUNEFAILED = 0U;
  clock->TASKS_XOSTART = CLOCK_TASKS_XOSTART_TASKS_XOSTART_Trigger;

  if (!waitForTuned) {
    return true;
  }

  if (spinLimit == 0U) {
    spinLimit = 1000000UL;
  }

  while (spinLimit-- > 0U) {
    const bool running =
        (((clock->XO.STAT & CLOCK_XO_STAT_STATE_Msk) >>
          CLOCK_XO_STAT_STATE_Pos) == CLOCK_XO_STAT_STATE_Running);
    if (running || (clock->EVENTS_XOSTARTED != 0U) ||
        (clock->EVENTS_XOTUNED != 0U)) {
      return true;
    }
    if (clock->EVENTS_XOTUNEFAILED != 0U) {
      return false;
    }
  }

  return (((clock->XO.STAT & CLOCK_XO_STAT_STATE_Msk) >>
           CLOCK_XO_STAT_STATE_Pos) == CLOCK_XO_STAT_STATE_Running);
}

void ClockControl::stopHfxo() {
  NRF_CLOCK_Type* clock = reinterpret_cast<NRF_CLOCK_Type*>(
      static_cast<uintptr_t>(nrf54l15::CLOCK_BASE));
  if (clock != nullptr) {
    clock->TASKS_XOSTOP = CLOCK_TASKS_XOSTOP_TASKS_XOSTOP_Trigger;
  }
}

bool ClockControl::setCpuFrequency(CpuFrequency frequency) {
  return static_cast<uint32_t>(frequency) == F_CPU;
}

CpuFrequency ClockControl::cpuFrequency() {
  return (F_CPU >= static_cast<uint32_t>(CpuFrequency::k128MHz))
             ? CpuFrequency::k128MHz
             : CpuFrequency::k64MHz;
}

bool ClockControl::enableIdleCpuScaling(CpuFrequency idleFrequency) {
  g_idleCpuFrequency = idleFrequency;
  g_idleCpuScalingEnabled = true;
  return false;
}

void ClockControl::disableIdleCpuScaling() {
  g_idleCpuScalingEnabled = false;
}

bool ClockControl::idleCpuScalingEnabled() {
  return g_idleCpuScalingEnabled;
}

CpuFrequency ClockControl::idleCpuFrequency() {
  return g_idleCpuFrequency;
}

bool Gpio::configure(const Pin& pin, GpioDirection direction, GpioPull pull) {
  uint8_t digitalPin = 0U;
  if (!pinToDigitalPin(pin, &digitalPin)) {
    return false;
  }

  if (direction == GpioDirection::kOutput) {
    pinMode(digitalPin, OUTPUT);
    return true;
  }

  switch (pull) {
    case GpioPull::kPullUp:
      pinMode(digitalPin, INPUT_PULLUP);
      break;
    case GpioPull::kPullDown:
      pinMode(digitalPin, INPUT_PULLDOWN);
      break;
    case GpioPull::kDisabled:
    default:
      pinMode(digitalPin, INPUT);
      break;
  }

  return true;
}

bool Gpio::write(const Pin& pin, bool high) {
  uint8_t digitalPin = 0U;
  if (!pinToDigitalPin(pin, &digitalPin)) {
    return false;
  }

  digitalWrite(digitalPin, high ? HIGH : LOW);
  return true;
}

bool Gpio::read(const Pin& pin, bool* high) {
  if (high == nullptr) {
    return false;
  }

  uint8_t digitalPin = 0U;
  if (!pinToDigitalPin(pin, &digitalPin)) {
    return false;
  }

  *high = (digitalRead(digitalPin) == HIGH);
  return true;
}

bool Gpio::toggle(const Pin& pin) {
  bool high = false;
  if (!read(pin, &high)) {
    return false;
  }
  return write(pin, !high);
}

bool Gpio::setDriveS0D1(const Pin& pin) {
  NRF_GPIO_Type* gpio = gpioForPort(pin.port);
  if (gpio == nullptr || pin.pin > 31U) {
    return false;
  }

  uint32_t cnf = gpio->PIN_CNF[pin.pin];
  cnf &= ~(nrf54l15::gpio::PIN_CNF_DRIVE0_Msk |
           nrf54l15::gpio::PIN_CNF_DRIVE1_Msk);
  cnf |= (nrf54l15::gpio::DRIVE0_S0 << nrf54l15::gpio::PIN_CNF_DRIVE0_Pos);
  cnf |= (nrf54l15::gpio::DRIVE1_D1 << nrf54l15::gpio::PIN_CNF_DRIVE1_Pos);
  gpio->PIN_CNF[pin.pin] = cnf;
  return true;
}

Spim::Spim(uint32_t base, uint32_t coreClockHz)
    : base_(base),
      coreClockHz_(coreClockHz),
      cs_(kPinDisconnected),
      frequencyHz_(4000000UL),
      mode_(SpiMode::kMode0),
      lsbFirst_(false),
      active_(false) {}

bool Spim::begin(const Pin& sck,
                 const Pin& mosi,
                 const Pin& miso,
                 const Pin& cs,
                 uint32_t hz,
                 SpiMode mode,
                 bool lsbFirst) {
  (void)base_;
  (void)coreClockHz_;

  uint8_t sckPin = 0U;
  uint8_t mosiPin = 0U;
  uint8_t misoPin = 0U;
  if (!pinToDigitalPin(sck, &sckPin) || !pinToDigitalPin(mosi, &mosiPin) ||
      !pinToDigitalPin(miso, &misoPin)) {
    return false;
  }

  if (!SPI.setPins(static_cast<int8_t>(sckPin), static_cast<int8_t>(misoPin),
                   static_cast<int8_t>(mosiPin))) {
    return false;
  }

  cs_ = cs;
  frequencyHz_ = (hz == 0U) ? 4000000UL : hz;
  mode_ = mode;
  lsbFirst_ = lsbFirst;

  if (active_) {
    SPI.endTransaction();
    SPI.end();
    active_ = false;
  }

  if (isConnected(cs_)) {
    uint8_t csPin = 0U;
    if (!pinToDigitalPin(cs_, &csPin)) {
      return false;
    }
    SPI.begin(csPin);
  } else {
    SPI.begin();
  }

  SPI.beginTransaction(SPISettings(frequencyHz_,
                                   lsbFirst_ ? LSBFIRST : MSBFIRST,
                                   spiModeToArduino(mode_)));
  active_ = true;
  return true;
}

bool Spim::setFrequency(uint32_t hz) {
  frequencyHz_ = (hz == 0U) ? 4000000UL : hz;
  if (!active_) {
    return true;
  }

  SPI.endTransaction();
  SPI.beginTransaction(SPISettings(frequencyHz_,
                                   lsbFirst_ ? LSBFIRST : MSBFIRST,
                                   spiModeToArduino(mode_)));
  return true;
}

void Spim::end() {
  if (!active_) {
    return;
  }

  SPI.endTransaction();
  SPI.end();
  active_ = false;
}

bool Spim::transfer(const uint8_t* tx,
                    uint8_t* rx,
                    size_t len,
                    uint32_t spinLimit) {
  (void)spinLimit;

  if (!active_ || len == 0U) {
    return false;
  }

  SPI.transfer(tx, rx, len);
  return true;
}

Spis::Spis(uint32_t base)
    : spis_(reinterpret_cast<NRF_SPIS_Type*>(static_cast<uintptr_t>(base))),
      active_(false) {}

bool Spis::begin(const Pin& sck,
                 const Pin& mosi,
                 const Pin& miso,
                 const Pin& csn,
                 SpiMode mode,
                 bool lsbFirst,
                 uint8_t defaultChar,
                 uint8_t overReadChar,
                 bool autoAcquireAfterEnd) {
  if (!isConnected(sck) || !isConnected(mosi) || !isConnected(miso) ||
      !isConnected(csn)) {
    return false;
  }

  if (!Gpio::configure(sck, GpioDirection::kInput, GpioPull::kDisabled) ||
      !Gpio::configure(mosi, GpioDirection::kInput, GpioPull::kDisabled) ||
      !Gpio::configure(miso, GpioDirection::kOutput, GpioPull::kDisabled) ||
      !Gpio::configure(csn, GpioDirection::kInput, GpioPull::kPullUp) ||
      !Gpio::write(miso, true)) {
    return false;
  }

  spis_->ENABLE = (SPIS_ENABLE_ENABLE_Disabled << SPIS_ENABLE_ENABLE_Pos);
  spis_->PSEL.SCK = make_psel(sck.port, sck.pin);
  spis_->PSEL.MOSI = make_psel(mosi.port, mosi.pin);
  spis_->PSEL.MISO = make_psel(miso.port, miso.pin);
  spis_->PSEL.CSN = make_psel(csn.port, csn.pin);

  uint32_t config = 0U;
  if (lsbFirst) {
    config |= (SPIS_CONFIG_ORDER_LsbFirst << SPIS_CONFIG_ORDER_Pos) &
              SPIS_CONFIG_ORDER_Msk;
  }
  if (mode == SpiMode::kMode1 || mode == SpiMode::kMode3) {
    config |= (SPIS_CONFIG_CPHA_Trailing << SPIS_CONFIG_CPHA_Pos) &
              SPIS_CONFIG_CPHA_Msk;
  }
  if (mode == SpiMode::kMode2 || mode == SpiMode::kMode3) {
    config |= (SPIS_CONFIG_CPOL_ActiveLow << SPIS_CONFIG_CPOL_Pos) &
              SPIS_CONFIG_CPOL_Msk;
  }

  spis_->CONFIG = config;
  spis_->DEF = ((static_cast<uint32_t>(defaultChar) << SPIS_DEF_DEF_Pos) &
                SPIS_DEF_DEF_Msk);
  spis_->ORC = ((static_cast<uint32_t>(overReadChar) << SPIS_ORC_ORC_Pos) &
                SPIS_ORC_ORC_Msk);
  spis_->SHORTS =
      autoAcquireAfterEnd
          ? ((SPIS_SHORTS_END_ACQUIRE_Enabled << SPIS_SHORTS_END_ACQUIRE_Pos) &
             SPIS_SHORTS_END_ACQUIRE_Msk)
          : 0U;
  spis_->INTENCLR = 0xFFFFFFFFUL;
  spis_->EVENTS_END = 0U;
  spis_->EVENTS_ACQUIRED = 0U;
  spis_->EVENTS_DMA.RX.END = 0U;
  spis_->EVENTS_DMA.TX.END = 0U;

  spis_->ENABLE = ((SPIS_ENABLE_ENABLE_Enabled << SPIS_ENABLE_ENABLE_Pos) &
                   SPIS_ENABLE_ENABLE_Msk);
  active_ = true;
  clearStatus();
  return true;
}

bool Spis::acquire(uint32_t spinLimit) {
  if (!active_) {
    return false;
  }
  if ((spis_->SEMSTAT & SPIS_SEMSTAT_SEMSTAT_Msk) ==
      (SPIS_SEMSTAT_SEMSTAT_CPU << SPIS_SEMSTAT_SEMSTAT_Pos)) {
    return true;
  }

  spis_->EVENTS_ACQUIRED = 0U;
  spis_->TASKS_ACQUIRE = SPIS_TASKS_ACQUIRE_TASKS_ACQUIRE_Trigger;
  while (spinLimit-- > 0U) {
    if ((spis_->SEMSTAT & SPIS_SEMSTAT_SEMSTAT_Msk) ==
        (SPIS_SEMSTAT_SEMSTAT_CPU << SPIS_SEMSTAT_SEMSTAT_Pos)) {
      return true;
    }
    if (spis_->EVENTS_ACQUIRED != 0U) {
      spis_->EVENTS_ACQUIRED = 0U;
      return true;
    }
  }

  return (spis_->SEMSTAT & SPIS_SEMSTAT_SEMSTAT_Msk) ==
         (SPIS_SEMSTAT_SEMSTAT_CPU << SPIS_SEMSTAT_SEMSTAT_Pos);
}

bool Spis::setBuffers(uint8_t* rx,
                      size_t rxLen,
                      const uint8_t* tx,
                      size_t txLen,
                      uint32_t spinLimit) {
  if (!active_ || rx == nullptr || tx == nullptr || rxLen == 0U || txLen == 0U ||
      rxLen > 0xFFFFU || txLen > 0xFFFFU) {
    return false;
  }
  if (!acquire(spinLimit)) {
    return false;
  }

  spis_->DMA.RX.PTR = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(rx));
  spis_->DMA.RX.MAXCNT =
      (static_cast<uint32_t>(rxLen) << SPIS_DMA_RX_MAXCNT_MAXCNT_Pos) &
      SPIS_DMA_RX_MAXCNT_MAXCNT_Msk;
  spis_->DMA.TX.PTR = static_cast<uint32_t>(
      reinterpret_cast<uintptr_t>(const_cast<uint8_t*>(tx)));
  spis_->DMA.TX.MAXCNT =
      (static_cast<uint32_t>(txLen) << SPIS_DMA_TX_MAXCNT_MAXCNT_Pos) &
      SPIS_DMA_TX_MAXCNT_MAXCNT_Msk;

  spis_->EVENTS_END = 0U;
  spis_->EVENTS_DMA.RX.END = 0U;
  spis_->EVENTS_DMA.TX.END = 0U;
  clearStatus();
  return true;
}

bool Spis::releaseTransaction() {
  if (!active_) {
    return false;
  }
  spis_->TASKS_RELEASE = SPIS_TASKS_RELEASE_TASKS_RELEASE_Trigger;
  return true;
}

bool Spis::pollAcquired(bool clearEventFlag) {
  const bool fired = active_ && (spis_->EVENTS_ACQUIRED != 0U);
  if (fired && clearEventFlag) {
    spis_->EVENTS_ACQUIRED = 0U;
  }
  return fired;
}

bool Spis::pollEnd(bool clearEventFlag) {
  const bool fired = active_ && (spis_->EVENTS_END != 0U);
  if (fired && clearEventFlag) {
    spis_->EVENTS_END = 0U;
  }
  return fired;
}

size_t Spis::receivedBytes() const {
  if (!active_) {
    return 0U;
  }
  return static_cast<size_t>(
      (spis_->DMA.RX.AMOUNT & SPIS_DMA_RX_AMOUNT_AMOUNT_Msk) >>
      SPIS_DMA_RX_AMOUNT_AMOUNT_Pos);
}

size_t Spis::transmittedBytes() const {
  if (!active_) {
    return 0U;
  }
  return static_cast<size_t>(
      (spis_->DMA.TX.AMOUNT & SPIS_DMA_TX_AMOUNT_AMOUNT_Msk) >>
      SPIS_DMA_TX_AMOUNT_AMOUNT_Pos);
}

bool Spis::overflowed() const {
  return active_ && ((spis_->STATUS & SPIS_STATUS_OVERFLOW_Msk) != 0U);
}

bool Spis::overread() const {
  return active_ && ((spis_->STATUS & SPIS_STATUS_OVERREAD_Msk) != 0U);
}

void Spis::clearStatus() {
  if (spis_ == nullptr) {
    return;
  }
  spis_->STATUS = SPIS_STATUS_OVERREAD_Clear | SPIS_STATUS_OVERFLOW_Clear;
}

void Spis::end() {
  if (spis_ == nullptr) {
    return;
  }

  spis_->SHORTS = 0U;
  spis_->ENABLE = (SPIS_ENABLE_ENABLE_Disabled << SPIS_ENABLE_ENABLE_Pos);
  spis_->PSEL.SCK = PSEL_DISCONNECTED;
  spis_->PSEL.MOSI = PSEL_DISCONNECTED;
  spis_->PSEL.MISO = PSEL_DISCONNECTED;
  spis_->PSEL.CSN = PSEL_DISCONNECTED;
  active_ = false;
}

Twim::Twim(uint32_t base) : base_(base), wire_(nullptr), active_(false) {}

bool Twim::begin(const Pin& scl, const Pin& sda, TwimFrequency frequency) {
  (void)base_;

  TwoWire* wire = resolveWireForPins(scl, sda);
  if (wire == nullptr) {
    return false;
  }

  wire_ = wire;
  static_cast<TwoWire*>(wire_)->setClock(static_cast<uint32_t>(frequency));
  static_cast<TwoWire*>(wire_)->begin();
  active_ = true;
  return true;
}

bool Twim::setFrequency(TwimFrequency frequency) {
  if (wire_ == nullptr) {
    return false;
  }

  static_cast<TwoWire*>(wire_)->setClock(static_cast<uint32_t>(frequency));
  return true;
}

void Twim::end() {
  if (wire_ != nullptr) {
    static_cast<TwoWire*>(wire_)->end();
  }
  active_ = false;
}

bool Twim::write(uint8_t address7,
                 const uint8_t* data,
                 size_t len,
                 uint32_t spinLimit) {
  (void)spinLimit;

  if (!active_ || wire_ == nullptr || (data == nullptr && len != 0U)) {
    return false;
  }

  auto* wire = static_cast<TwoWire*>(wire_);
  wire->beginTransmission(address7);
  if (len != 0U && wire->write(data, len) != len) {
    (void)wire->endTransmission(true);
    return false;
  }
  return wire->endTransmission(true) == 0U;
}

bool Twim::read(uint8_t address7,
                uint8_t* data,
                size_t len,
                uint32_t spinLimit) {
  (void)spinLimit;

  if (!active_ || wire_ == nullptr || data == nullptr) {
    return false;
  }

  auto* wire = static_cast<TwoWire*>(wire_);
  const size_t received = wire->requestFrom(address7, len, true);
  if (received != len) {
    return false;
  }

  for (size_t index = 0; index < len; ++index) {
    if (wire->available() <= 0) {
      return false;
    }
    data[index] = static_cast<uint8_t>(wire->read());
  }

  return true;
}

bool Twim::writeRead(uint8_t address7,
                     const uint8_t* tx,
                     size_t txLen,
                     uint8_t* rx,
                     size_t rxLen,
                     uint32_t spinLimit) {
  (void)spinLimit;

  if (!active_ || wire_ == nullptr || (tx == nullptr && txLen != 0U) ||
      (rx == nullptr && rxLen != 0U)) {
    return false;
  }

  auto* wire = static_cast<TwoWire*>(wire_);
  wire->beginTransmission(address7);
  if (txLen != 0U && wire->write(tx, txLen) != txLen) {
    (void)wire->endTransmission(true);
    return false;
  }
  if (wire->endTransmission(false) != 0U) {
    return false;
  }

  const size_t received = wire->requestFrom(address7, rxLen, true);
  if (received != rxLen) {
    return false;
  }

  for (size_t index = 0; index < rxLen; ++index) {
    if (wire->available() <= 0) {
      return false;
    }
    rx[index] = static_cast<uint8_t>(wire->read());
  }

  return true;
}

Timer::Timer(uint32_t base, uint32_t pclkHz, uint8_t channelCount)
    : base_(base),
      pclkHz_(pclkHz),
      channelCount_(channelCount),
      prescaler_(4U),
      bitWidth_(TimerBitWidth::k32bit),
      counterMode_(false),
      running_(false),
      startMicros_(0U),
      storedTicks_(0U),
      callbacks_{},
      callbackContext_{},
      compareSlots_{} {}

bool Timer::begin(TimerBitWidth bitWidth, uint8_t prescaler, bool counterMode) {
  if (channelCount_ == 0U || channelCount_ > 8U) {
    return false;
  }

  bitWidth_ = bitWidth;
  prescaler_ = (prescaler > 9U) ? 9U : prescaler;
  counterMode_ = counterMode;
  running_ = false;
  storedTicks_ = 0U;
  startMicros_ = 0U;
  for (uint8_t channel = 0U; channel < 8U; ++channel) {
    compareSlots_[channel] = {};
    callbacks_[channel] = nullptr;
    callbackContext_[channel] = nullptr;
  }
  return !counterMode_;
}

bool Timer::setFrequency(uint32_t targetHz) {
  if (targetHz == 0U || targetHz > pclkHz_) {
    return false;
  }

  uint8_t bestPrescaler = 0U;
  while (bestPrescaler < 9U &&
         (pclkHz_ / (1UL << bestPrescaler)) > targetHz) {
    ++bestPrescaler;
  }

  prescaler_ = bestPrescaler;
  return timerHz() == targetHz;
}

uint32_t Timer::timerHz() const {
  return pclkHz_ / (1UL << prescaler_);
}

uint32_t Timer::ticksFromMicros(uint32_t us) const {
  const uint64_t ticks =
      (static_cast<uint64_t>(us) * static_cast<uint64_t>(timerHz())) /
      1000000ULL;
  return static_cast<uint32_t>(ticks);
}

void Timer::start() {
  if (running_) {
    return;
  }

  startMicros_ = micros();
  running_ = true;
}

void Timer::stop() {
  if (!running_) {
    return;
  }

  storedTicks_ = currentTicks();
  running_ = false;
}

void Timer::clear() {
  storedTicks_ = 0U;
  if (running_) {
    startMicros_ = micros();
  }

  for (uint8_t channel = 0U; channel < 8U; ++channel) {
    compareSlots_[channel].eventPending = false;
    if (compareSlots_[channel].configured) {
      compareSlots_[channel].nextFireTicks =
          compareSlots_[channel].intervalTicks;
    }
  }
}

bool Timer::setCompare(uint8_t channel,
                       uint32_t ccValue,
                       bool autoClear,
                       bool autoStop,
                       bool oneShot,
                       bool enableInterrupt) {
  if (channel >= channelCount_ || channel >= 8U || ccValue == 0U) {
    return false;
  }

  compareSlots_[channel].intervalTicks = ccValue;
  compareSlots_[channel].nextFireTicks = ccValue;
  compareSlots_[channel].configured = true;
  compareSlots_[channel].autoClear = autoClear;
  compareSlots_[channel].autoStop = autoStop;
  compareSlots_[channel].oneShot = oneShot;
  compareSlots_[channel].interruptEnabled = enableInterrupt;
  compareSlots_[channel].eventPending = false;
  return true;
}

uint32_t Timer::capture(uint8_t channel) {
  (void)channel;
  return currentTicks();
}

bool Timer::pollCompare(uint8_t channel, bool clearEvent) {
  if (channel >= channelCount_ || channel >= 8U) {
    return false;
  }

  const bool fired = compareSlots_[channel].eventPending;
  if (fired && clearEvent) {
    compareSlots_[channel].eventPending = false;
  }
  return fired;
}

volatile uint32_t* Timer::publishCompareConfigRegister(uint8_t channel) const {
  (void)channel;
  return nullptr;
}

volatile uint32_t* Timer::subscribeStartConfigRegister() const {
  return nullptr;
}

volatile uint32_t* Timer::subscribeStopConfigRegister() const {
  return nullptr;
}

volatile uint32_t* Timer::subscribeClearConfigRegister() const {
  return nullptr;
}

volatile uint32_t* Timer::subscribeCaptureConfigRegister(uint8_t channel) const {
  (void)channel;
  return nullptr;
}

void Timer::enableInterrupt(uint8_t channel, bool enable) {
  if (channel >= channelCount_ || channel >= 8U) {
    return;
  }

  compareSlots_[channel].interruptEnabled = enable;
}

bool Timer::attachCompareCallback(uint8_t channel,
                                  CompareCallback callback,
                                  void* context) {
  if (channel >= channelCount_ || channel >= 8U) {
    return false;
  }

  callbacks_[channel] = callback;
  callbackContext_[channel] = context;
  return true;
}

void Timer::service() {
  if (!running_) {
    return;
  }

  const uint32_t nowTicks = currentTicks();
  for (uint8_t channel = 0U; channel < channelCount_ && channel < 8U;
       ++channel) {
    auto& slot = compareSlots_[channel];
    if (!slot.configured) {
      continue;
    }

    while (slot.configured && nowTicks >= slot.nextFireTicks &&
           slot.nextFireTicks != 0U) {
      slot.eventPending = true;
      if (callbacks_[channel] != nullptr) {
        callbacks_[channel](channel, callbackContext_[channel]);
      }

      if (slot.autoStop) {
        stop();
      }

      if (slot.autoClear) {
        slot.nextFireTicks += slot.intervalTicks;
        continue;
      }

      if (slot.oneShot || !slot.autoClear) {
        slot.configured = false;
      }
    }
  }
}

uint32_t Timer::currentTicks() const {
  if (!running_) {
    return storedTicks_;
  }

  const uint64_t elapsedUs =
      static_cast<uint64_t>(micros()) - startMicros_;
  uint64_t ticks = storedTicks_;
  ticks += (elapsedUs * static_cast<uint64_t>(timerHz())) / 1000000ULL;

  switch (bitWidth_) {
    case TimerBitWidth::k8bit:
      ticks &= 0xFFULL;
      break;
    case TimerBitWidth::k16bit:
      ticks &= 0xFFFFULL;
      break;
    case TimerBitWidth::k24bit:
      ticks &= 0xFFFFFFULL;
      break;
    case TimerBitWidth::k32bit:
    default:
      ticks &= 0xFFFFFFFFULL;
      break;
  }

  return static_cast<uint32_t>(ticks);
}

Pwm::Pwm(uint32_t base)
    : base_(base),
      outPin_(kPinDisconnected),
      frequencyHz_(1000UL),
      dutyPermille_(500U),
      activeHigh_(true),
      configured_(false),
      running_(false),
      periodPending_(false) {}

bool Pwm::beginSingle(const Pin& outPin,
                      uint32_t frequencyHz,
                      uint16_t dutyPermille,
                      bool activeHigh) {
  (void)base_;

  uint8_t digitalPin = 0U;
  if (!pinToDigitalPin(outPin, &digitalPin)) {
    return false;
  }

  outPin_ = outPin;
  frequencyHz_ = (frequencyHz == 0U) ? 1000UL : frequencyHz;
  dutyPermille_ = (dutyPermille > 1000U) ? 1000U : dutyPermille;
  activeHigh_ = activeHigh;
  configured_ = true;
  running_ = false;
  periodPending_ = false;
  pinMode(digitalPin, OUTPUT);
  digitalWrite(digitalPin, activeHigh_ ? LOW : HIGH);
  return true;
}

bool Pwm::setDutyPermille(uint16_t dutyPermille) {
  if (!configured_) {
    return false;
  }

  dutyPermille_ = (dutyPermille > 1000U) ? 1000U : dutyPermille;
  return applyOutput();
}

bool Pwm::setFrequency(uint32_t frequencyHz) {
  if (!configured_ || frequencyHz == 0U) {
    return false;
  }

  frequencyHz_ = frequencyHz;
  return applyOutput();
}

bool Pwm::start(uint8_t sequence, uint32_t spinLimit) {
  (void)sequence;
  (void)spinLimit;

  if (!configured_) {
    return false;
  }

  running_ = true;
  periodPending_ = true;
  return applyOutput();
}

bool Pwm::stop(uint32_t spinLimit) {
  (void)spinLimit;

  if (!configured_) {
    return false;
  }

  running_ = false;
  return applyOutput();
}

void Pwm::end() {
  if (!configured_) {
    return;
  }

  (void)stop();
  configured_ = false;
}

bool Pwm::pollPeriodEnd(bool clearEvent) {
  const bool pending = periodPending_;
  if (pending && clearEvent) {
    periodPending_ = false;
  }
  return pending;
}

bool Pwm::applyOutput() const {
  if (!configured_) {
    return false;
  }

  uint8_t digitalPin = 0U;
  if (!pinToDigitalPin(outPin_, &digitalPin)) {
    return false;
  }

  if (!running_) {
    digitalWrite(digitalPin, activeHigh_ ? LOW : HIGH);
    return true;
  }

  const uint16_t effectivePermille =
      activeHigh_ ? dutyPermille_ : static_cast<uint16_t>(1000U - dutyPermille_);
  if (digitalPinHasPWM(digitalPin)) {
    analogWritePinFrequency(digitalPin, frequencyHz_);
    analogWriteResolution(10U);
    const int pwmValue = static_cast<int>((effectivePermille * 1023U) / 1000U);
    analogWrite(digitalPin, pwmValue);
    return true;
  }

  digitalWrite(digitalPin, (effectivePermille >= 500U) ? HIGH : LOW);
  return true;
}

Gpiote::Gpiote(uint32_t base, uint8_t channelCount)
    : base_(base),
      channelCount_(channelCount),
      callbacks_{},
      callbackContext_{},
      eventSlots_{},
      taskSlots_{} {}

bool Gpiote::configureEvent(uint8_t channel,
                            const Pin& pin,
                            GpiotePolarity polarity,
                            bool enableInterrupt) {
  (void)base_;

  if (channel >= channelCount_ || channel >= 8U || !isConnected(pin)) {
    return false;
  }

  bool high = false;
  if (!Gpio::read(pin, &high)) {
    return false;
  }

  eventSlots_[channel].pin = pin;
  eventSlots_[channel].polarity = polarity;
  eventSlots_[channel].configured = true;
  eventSlots_[channel].interruptEnabled = enableInterrupt;
  eventSlots_[channel].lastHigh = high;
  eventSlots_[channel].eventPending = false;
  return true;
}

bool Gpiote::configureTask(uint8_t channel,
                           const Pin& pin,
                           GpiotePolarity polarity,
                           bool initialHigh) {
  if (channel >= channelCount_ || channel >= 8U || !isConnected(pin)) {
    return false;
  }

  if (!Gpio::configure(pin, GpioDirection::kOutput, GpioPull::kDisabled) ||
      !Gpio::write(pin, initialHigh)) {
    return false;
  }

  taskSlots_[channel].pin = pin;
  taskSlots_[channel].polarity = polarity;
  taskSlots_[channel].configured = true;
  return true;
}

void Gpiote::disableChannel(uint8_t channel) {
  if (channel >= channelCount_ || channel >= 8U) {
    return;
  }

  eventSlots_[channel] = {};
  taskSlots_[channel] = {};
  callbacks_[channel] = nullptr;
  callbackContext_[channel] = nullptr;
}

bool Gpiote::triggerTaskOut(uint8_t channel) {
  return triggerTask(channel, false, false);
}

bool Gpiote::triggerTaskSet(uint8_t channel) {
  return triggerTask(channel, true, false);
}

bool Gpiote::triggerTaskClr(uint8_t channel) {
  return triggerTask(channel, false, true);
}

bool Gpiote::pollInEvent(uint8_t channel, bool clearEvent) {
  if (channel >= channelCount_ || channel >= 8U) {
    return false;
  }

  const bool pending = eventSlots_[channel].eventPending;
  if (pending && clearEvent) {
    eventSlots_[channel].eventPending = false;
  }
  return pending;
}

bool Gpiote::pollPortEvent(bool clearEvent) {
  bool anyPending = false;
  for (uint8_t channel = 0U; channel < channelCount_ && channel < 8U;
       ++channel) {
    if (eventSlots_[channel].eventPending) {
      anyPending = true;
      if (clearEvent) {
        eventSlots_[channel].eventPending = false;
      }
    }
  }
  return anyPending;
}

volatile uint32_t* Gpiote::subscribeTaskOutConfigRegister(uint8_t channel) const {
  (void)channel;
  return nullptr;
}

volatile uint32_t* Gpiote::subscribeTaskSetConfigRegister(uint8_t channel) const {
  (void)channel;
  return nullptr;
}

volatile uint32_t* Gpiote::subscribeTaskClrConfigRegister(uint8_t channel) const {
  (void)channel;
  return nullptr;
}

void Gpiote::enableInterrupt(uint8_t channel, bool enable) {
  if (channel >= channelCount_ || channel >= 8U) {
    return;
  }

  eventSlots_[channel].interruptEnabled = enable;
}

bool Gpiote::attachInCallback(uint8_t channel,
                              InCallback callback,
                              void* context) {
  if (channel >= channelCount_ || channel >= 8U) {
    return false;
  }

  callbacks_[channel] = callback;
  callbackContext_[channel] = context;
  return true;
}

void Gpiote::service() {
  for (uint8_t channel = 0U; channel < channelCount_ && channel < 8U;
       ++channel) {
    auto& slot = eventSlots_[channel];
    if (!slot.configured) {
      continue;
    }

    bool high = false;
    if (!Gpio::read(slot.pin, &high)) {
      continue;
    }

    bool triggered = false;
    switch (slot.polarity) {
      case GpiotePolarity::kLoToHi:
        triggered = (!slot.lastHigh && high);
        break;
      case GpiotePolarity::kHiToLo:
        triggered = (slot.lastHigh && !high);
        break;
      case GpiotePolarity::kToggle:
        triggered = (slot.lastHigh != high);
        break;
      case GpiotePolarity::kNone:
      default:
        triggered = false;
        break;
    }

    slot.lastHigh = high;
    if (!triggered) {
      continue;
    }

    slot.eventPending = true;
    if (callbacks_[channel] != nullptr) {
      callbacks_[channel](channel, callbackContext_[channel]);
    }
  }
}

bool Gpiote::triggerTask(uint8_t channel, bool explicitHigh, bool explicitLow) {
  if (channel >= channelCount_ || channel >= 8U || !taskSlots_[channel].configured) {
    return false;
  }

  const Pin& pin = taskSlots_[channel].pin;
  if (explicitHigh) {
    return Gpio::write(pin, true);
  }
  if (explicitLow) {
    return Gpio::write(pin, false);
  }

  switch (taskSlots_[channel].polarity) {
    case GpiotePolarity::kLoToHi:
      return Gpio::write(pin, true);
    case GpiotePolarity::kHiToLo:
      return Gpio::write(pin, false);
    case GpiotePolarity::kToggle:
      return Gpio::toggle(pin);
    case GpiotePolarity::kNone:
    default:
      return true;
  }
}

Dppic::Dppic(uint32_t base)
    : dppic_(reinterpret_cast<NRF_DPPIC_Type*>(static_cast<uintptr_t>(base))) {}

bool Dppic::enableChannel(uint8_t channel, bool enable) {
  if (dppic_ == nullptr || channel >= 24U) {
    return false;
  }

  const uint32_t mask = (1UL << channel);
  if (enable) {
    dppic_->CHENSET = mask;
  } else {
    dppic_->CHENCLR = mask;
  }
  return true;
}

bool Dppic::channelEnabled(uint8_t channel) const {
  if (dppic_ == nullptr || channel >= 24U) {
    return false;
  }

  return (dppic_->CHEN & (1UL << channel)) != 0U;
}

bool Dppic::configurePublish(volatile uint32_t* publishRegister,
                             uint8_t channel,
                             bool enable) const {
  if (publishRegister == nullptr || channel >= 24U) {
    return false;
  }

  *publishRegister = enable ? (static_cast<uint32_t>(channel) | (1UL << 31U))
                            : 0U;
  return true;
}

bool Dppic::configureSubscribe(volatile uint32_t* subscribeRegister,
                               uint8_t channel,
                               bool enable) const {
  if (subscribeRegister == nullptr || channel >= 24U) {
    return false;
  }

  *subscribeRegister = enable ? (static_cast<uint32_t>(channel) | (1UL << 31U))
                              : 0U;
  return true;
}

bool Dppic::connect(volatile uint32_t* publishRegister,
                    volatile uint32_t* subscribeRegister,
                    uint8_t channel,
                    bool enableChannelFlag) const {
  if (!configurePublish(publishRegister, channel, true) ||
      !configureSubscribe(subscribeRegister, channel, true)) {
    return false;
  }

  if (enableChannelFlag && dppic_ != nullptr) {
    dppic_->CHENSET = (1UL << channel);
  }
  return true;
}

bool Dppic::disconnectPublish(volatile uint32_t* publishRegister) const {
  if (publishRegister == nullptr) {
    return false;
  }

  *publishRegister = 0U;
  return true;
}

bool Dppic::disconnectSubscribe(volatile uint32_t* subscribeRegister) const {
  if (subscribeRegister == nullptr) {
    return false;
  }

  *subscribeRegister = 0U;
  return true;
}

Saadc::Saadc(uint32_t base)
    : base_(base),
      resolution_(AdcResolution::k12bit),
      gain_(AdcGain::k2over8),
      oversample_(AdcOversample::kBypass),
      differential_(false),
      configured_(false),
      positivePin_(kPinDisconnected),
      negativePin_(kPinDisconnected) {}

bool Saadc::begin(AdcResolution resolution, uint32_t spinLimit) {
  (void)spinLimit;

  resolution_ = resolution;
  oversample_ = AdcOversample::kBypass;
  configured_ = false;
  return true;
}

bool Saadc::begin(AdcResolution resolution,
                  AdcOversample oversample,
                  uint32_t spinLimit) {
  (void)spinLimit;

  resolution_ = resolution;
  oversample_ = oversample;
  configured_ = false;
  return true;
}

bool Saadc::calibrate(uint32_t spinLimit) {
  (void)spinLimit;
  return true;
}

void Saadc::end() {
  configured_ = false;
}

bool Saadc::configureSingleEnded(uint8_t channel,
                                 const Pin& pin,
                                 AdcGain gain,
                                 uint16_t tacq,
                                 uint8_t tconv,
                                 bool burst) {
  (void)channel;
  (void)tacq;
  (void)tconv;
  (void)burst;

  if (saadcInputForPin(pin) < 0) {
    return false;
  }

  positivePin_ = pin;
  negativePin_ = kPinDisconnected;
  gain_ = gain;
  differential_ = false;
  configured_ = true;
  return true;
}

bool Saadc::configureDifferential(uint8_t channel,
                                  const Pin& positivePin,
                                  const Pin& negativePin,
                                  AdcGain gain,
                                  uint16_t tacq,
                                  uint8_t tconv,
                                  bool burst) {
  (void)channel;
  (void)tacq;
  (void)tconv;
  (void)burst;

  if (saadcInputForPin(positivePin) < 0 || saadcInputForPin(negativePin) < 0) {
    return false;
  }

  positivePin_ = positivePin;
  negativePin_ = negativePin;
  gain_ = gain;
  differential_ = true;
  configured_ = true;
  return true;
}

bool Saadc::sampleRaw(int16_t* outRaw, uint32_t spinLimit) const {
  (void)spinLimit;

  if (outRaw == nullptr || !configured_) {
    return false;
  }

  const uint32_t samples = oversampleCount(oversample_);
  int64_t accumulated = 0;
  for (uint32_t sampleIndex = 0U; sampleIndex < samples; ++sampleIndex) {
    const int positiveRaw = sampleSingleRaw(positivePin_);
    if (positiveRaw < 0) {
      return false;
    }

    if (differential_) {
      const int negativeRaw = sampleSingleRaw(negativePin_);
      if (negativeRaw < 0) {
        return false;
      }
      accumulated += static_cast<int64_t>(positiveRaw) -
                     static_cast<int64_t>(negativeRaw);
    } else {
      accumulated += positiveRaw;
    }
  }

  *outRaw = static_cast<int16_t>(accumulated / static_cast<int64_t>(samples));
  return true;
}

bool Saadc::sampleMilliVolts(int32_t* outMilliVolts, uint32_t spinLimit) const {
  return sampleMilliVoltsSigned(outMilliVolts, spinLimit);
}

bool Saadc::sampleMilliVoltsSigned(int32_t* outMilliVolts,
                                   uint32_t spinLimit) const {
  (void)spinLimit;

  if (outMilliVolts == nullptr) {
    return false;
  }

  int16_t raw = 0;
  if (!sampleRaw(&raw)) {
    return false;
  }

  uint32_t numerator = 0U;
  uint32_t denominator = 0U;
  if (!gainRatio(gain_, &numerator, &denominator) || numerator == 0U) {
    return false;
  }

  const int64_t maxCount = static_cast<int64_t>(maxCountForResolution(resolution_));
  const int64_t scaled =
      static_cast<int64_t>(raw) * 600LL * static_cast<int64_t>(denominator);
  *outMilliVolts = static_cast<int32_t>(scaled /
                                        (maxCount * static_cast<int64_t>(numerator)));
  return true;
}

int Saadc::sampleSingleRaw(const Pin& pin) const {
  const int8_t analogIndex = saadcInputForPin(pin);
  if (analogIndex < 0) {
    return -1;
  }

  analogReadResolution(static_cast<uint8_t>(resolution_));
  return analogRead(static_cast<uint8_t>(analogIndex));
}

uint32_t Saadc::oversampleCount(AdcOversample oversample) {
  switch (oversample) {
    case AdcOversample::k2x:
      return 2U;
    case AdcOversample::k4x:
      return 4U;
    case AdcOversample::k8x:
      return 8U;
    case AdcOversample::k16x:
      return 16U;
    case AdcOversample::k32x:
      return 32U;
    case AdcOversample::k64x:
      return 64U;
    case AdcOversample::k128x:
      return 128U;
    case AdcOversample::k256x:
      return 256U;
    case AdcOversample::kBypass:
    default:
      return 1U;
  }
}

uint32_t Saadc::maxCountForResolution(AdcResolution resolution) {
  switch (resolution) {
    case AdcResolution::k8bit:
      return 0xFFU;
    case AdcResolution::k10bit:
      return 0x3FFU;
    case AdcResolution::k12bit:
      return 0xFFFU;
    case AdcResolution::k14bit:
    default:
      return 0x3FFFU;
  }
}

bool Saadc::gainRatio(AdcGain gain,
                      uint32_t* numerator,
                      uint32_t* denominator) {
  if (numerator == nullptr || denominator == nullptr) {
    return false;
  }

  switch (gain) {
    case AdcGain::k2:
      *numerator = 2U;
      *denominator = 1U;
      return true;
    case AdcGain::k1:
      *numerator = 1U;
      *denominator = 1U;
      return true;
    case AdcGain::k2over3:
      *numerator = 2U;
      *denominator = 3U;
      return true;
    case AdcGain::k2over4:
      *numerator = 2U;
      *denominator = 4U;
      return true;
    case AdcGain::k2over5:
      *numerator = 2U;
      *denominator = 5U;
      return true;
    case AdcGain::k2over6:
      *numerator = 2U;
      *denominator = 6U;
      return true;
    case AdcGain::k2over7:
      *numerator = 2U;
      *denominator = 7U;
      return true;
    case AdcGain::k2over8:
      *numerator = 2U;
      *denominator = 8U;
      return true;
    default:
      return false;
  }
}

Comp::Comp(uint32_t base)
    : comp_(reinterpret_cast<NRF_COMP_Type*>(static_cast<uintptr_t>(base))),
      active_(false) {}

bool Comp::beginThreshold(const Pin& inputPin,
                          uint16_t thresholdPermille,
                          uint16_t hysteresisPermille,
                          CompReference reference,
                          CompSpeedMode speed,
                          const Pin& externalReferencePin,
                          CompCurrentSource currentSource,
                          uint32_t spinLimit) {
  return beginSingleEnded(inputPin, reference, thresholdPermille,
                          hysteresisPermille, speed, externalReferencePin,
                          currentSource, spinLimit);
}

bool Comp::beginSingleEnded(const Pin& inputPin,
                            CompReference reference,
                            uint16_t thresholdPermille,
                            uint16_t hysteresisPermille,
                            CompSpeedMode speed,
                            const Pin& externalReferencePin,
                            CompCurrentSource currentSource,
                            uint32_t spinLimit) {
  if (active_) {
    end();
  }

  if (!pinSupportsAnalogInput(inputPin)) {
    return false;
  }
  if (reference == CompReference::kExternalAref &&
      !pinSupportsAnalogInput(externalReferencePin)) {
    return false;
  }
  if (!claimComparator(ComparatorOwner::kComp)) {
    return false;
  }
  if (!configureAnalogPeripheralPin(inputPin) ||
      (reference == CompReference::kExternalAref &&
       !configureAnalogPeripheralPin(externalReferencePin))) {
    releaseComparator(ComparatorOwner::kComp);
    return false;
  }

  thresholdPermille = clampPermille(thresholdPermille);
  hysteresisPermille = clampPermille(hysteresisPermille);
  const uint16_t halfHysteresis = hysteresisPermille / 2U;
  uint16_t lowPermille = (thresholdPermille > halfHysteresis)
                             ? static_cast<uint16_t>(thresholdPermille -
                                                     halfHysteresis)
                             : 0U;
  uint16_t highPermille = static_cast<uint16_t>(
      thresholdPermille + (hysteresisPermille - halfHysteresis));
  if (highPermille > 1000U) {
    highPermille = 1000U;
  }
  if (lowPermille > highPermille) {
    lowPermille = highPermille;
  }

  comp_->TASKS_STOP = COMP_TASKS_STOP_TASKS_STOP_Trigger;
  comp_->ENABLE = (COMP_ENABLE_ENABLE_Disabled << COMP_ENABLE_ENABLE_Pos);
  comp_->INTENCLR = 0xFFFFFFFFUL;
  comp_->SHORTS = 0U;
  clearEvents();

  comp_->PSEL = makeCompPinSelect(inputPin);
  comp_->REFSEL = ((static_cast<uint32_t>(reference) << COMP_REFSEL_REFSEL_Pos) &
                   COMP_REFSEL_REFSEL_Msk);
  comp_->EXTREFSEL = (reference == CompReference::kExternalAref)
                         ? makeCompPinSelect(externalReferencePin)
                         : 0U;
  comp_->TH = compThresholdRegValue(lowPermille, highPermille);
  comp_->MODE = ((static_cast<uint32_t>(speed) << COMP_MODE_SP_Pos) &
                 COMP_MODE_SP_Msk) |
                ((COMP_MODE_MAIN_SE << COMP_MODE_MAIN_Pos) &
                 COMP_MODE_MAIN_Msk);
  comp_->HYST = ((COMP_HYST_HYST_NoHyst << COMP_HYST_HYST_Pos) &
                 COMP_HYST_HYST_Msk);
  setCurrentSource(currentSource);

  comp_->ENABLE = ((COMP_ENABLE_ENABLE_Enabled << COMP_ENABLE_ENABLE_Pos) &
                   COMP_ENABLE_ENABLE_Msk);
  clearEvents();
  comp_->TASKS_START = COMP_TASKS_START_TASKS_START_Trigger;
  if (!waitForCompReady(&comp_->EVENTS_READY, spinLimit)) {
    comp_->TASKS_STOP = COMP_TASKS_STOP_TASKS_STOP_Trigger;
    comp_->ENABLE = (COMP_ENABLE_ENABLE_Disabled << COMP_ENABLE_ENABLE_Pos);
    releaseComparator(ComparatorOwner::kComp);
    return false;
  }

  active_ = true;
  return true;
}

bool Comp::beginDifferential(const Pin& positivePin,
                             const Pin& negativePin,
                             CompSpeedMode speed,
                             bool hysteresis,
                             CompCurrentSource currentSource,
                             uint32_t spinLimit) {
  if (active_) {
    end();
  }

  if (!pinSupportsAnalogInput(positivePin) ||
      !pinSupportsAnalogInput(negativePin) ||
      samePin(positivePin, negativePin)) {
    return false;
  }
  if (!claimComparator(ComparatorOwner::kComp)) {
    return false;
  }
  if (!configureAnalogPeripheralPin(positivePin) ||
      !configureAnalogPeripheralPin(negativePin)) {
    releaseComparator(ComparatorOwner::kComp);
    return false;
  }

  comp_->TASKS_STOP = COMP_TASKS_STOP_TASKS_STOP_Trigger;
  comp_->ENABLE = (COMP_ENABLE_ENABLE_Disabled << COMP_ENABLE_ENABLE_Pos);
  comp_->INTENCLR = 0xFFFFFFFFUL;
  comp_->SHORTS = 0U;
  clearEvents();

  comp_->PSEL = makeCompPinSelect(positivePin);
  comp_->EXTREFSEL = makeCompPinSelect(negativePin);
  comp_->REFSEL = 0U;
  comp_->TH = compThresholdRegValue(500U, 500U);
  comp_->MODE = ((static_cast<uint32_t>(speed) << COMP_MODE_SP_Pos) &
                 COMP_MODE_SP_Msk) |
                ((COMP_MODE_MAIN_Diff << COMP_MODE_MAIN_Pos) &
                 COMP_MODE_MAIN_Msk);
  comp_->HYST = (((hysteresis ? COMP_HYST_HYST_Hyst40mV
                              : COMP_HYST_HYST_NoHyst)
                  << COMP_HYST_HYST_Pos) &
                 COMP_HYST_HYST_Msk);
  setCurrentSource(currentSource);

  comp_->ENABLE = ((COMP_ENABLE_ENABLE_Enabled << COMP_ENABLE_ENABLE_Pos) &
                   COMP_ENABLE_ENABLE_Msk);
  clearEvents();
  comp_->TASKS_START = COMP_TASKS_START_TASKS_START_Trigger;
  if (!waitForCompReady(&comp_->EVENTS_READY, spinLimit)) {
    comp_->TASKS_STOP = COMP_TASKS_STOP_TASKS_STOP_Trigger;
    comp_->ENABLE = (COMP_ENABLE_ENABLE_Disabled << COMP_ENABLE_ENABLE_Pos);
    releaseComparator(ComparatorOwner::kComp);
    return false;
  }

  active_ = true;
  return true;
}

bool Comp::setThresholdWindowPermille(uint16_t lowPermille,
                                      uint16_t highPermille) {
  if (comp_ == nullptr) {
    return false;
  }

  lowPermille = clampPermille(lowPermille);
  highPermille = clampPermille(highPermille);
  if (lowPermille > highPermille) {
    return false;
  }

  comp_->TH = compThresholdRegValue(lowPermille, highPermille);
  return true;
}

void Comp::setCurrentSource(CompCurrentSource currentSource) {
  if (comp_ == nullptr) {
    return;
  }

  comp_->ISOURCE = ((static_cast<uint32_t>(currentSource)
                     << COMP_ISOURCE_ISOURCE_Pos) &
                    COMP_ISOURCE_ISOURCE_Msk);
}

bool Comp::sample(uint32_t spinLimit) const {
  (void)spinLimit;
  if (!active_) {
    return false;
  }

  comp_->TASKS_SAMPLE = COMP_TASKS_SAMPLE_TASKS_SAMPLE_Trigger;
  __asm volatile("dsb 0xF" ::: "memory");
  return true;
}

bool Comp::resultAbove() const {
  return comp_ != nullptr &&
         ((comp_->RESULT & COMP_RESULT_RESULT_Msk) != 0U);
}

bool Comp::pollReady(bool clearEventFlag) {
  const bool fired = active_ && (comp_->EVENTS_READY != 0U);
  if (fired && clearEventFlag) {
    comp_->EVENTS_READY = 0U;
  }
  return fired;
}

bool Comp::pollUp(bool clearEventFlag) {
  const bool fired = active_ && (comp_->EVENTS_UP != 0U);
  if (fired && clearEventFlag) {
    comp_->EVENTS_UP = 0U;
  }
  return fired;
}

bool Comp::pollDown(bool clearEventFlag) {
  const bool fired = active_ && (comp_->EVENTS_DOWN != 0U);
  if (fired && clearEventFlag) {
    comp_->EVENTS_DOWN = 0U;
  }
  return fired;
}

bool Comp::pollCross(bool clearEventFlag) {
  const bool fired = active_ && (comp_->EVENTS_CROSS != 0U);
  if (fired && clearEventFlag) {
    comp_->EVENTS_CROSS = 0U;
  }
  return fired;
}

void Comp::clearEvents() {
  if (comp_ == nullptr) {
    return;
  }

  comp_->EVENTS_READY = 0U;
  comp_->EVENTS_UP = 0U;
  comp_->EVENTS_DOWN = 0U;
  comp_->EVENTS_CROSS = 0U;
}

void Comp::end() {
  if (comp_ == nullptr) {
    active_ = false;
    releaseComparator(ComparatorOwner::kComp);
    return;
  }

  comp_->SHORTS = 0U;
  comp_->INTENCLR = 0xFFFFFFFFUL;
  clearEvents();
  comp_->TASKS_STOP = COMP_TASKS_STOP_TASKS_STOP_Trigger;
  comp_->ENABLE = ((COMP_ENABLE_ENABLE_Disabled << COMP_ENABLE_ENABLE_Pos) &
                   COMP_ENABLE_ENABLE_Msk);
  active_ = false;
  releaseComparator(ComparatorOwner::kComp);
}

Lpcomp::Lpcomp(uint32_t base)
    : lpcomp_(reinterpret_cast<NRF_LPCOMP_Type*>(static_cast<uintptr_t>(base))),
      active_(false) {}

bool Lpcomp::begin(const Pin& inputPin,
                   LpcompReference reference,
                   bool hysteresis,
                   LpcompDetect detect,
                   const Pin& externalReferencePin,
                   uint32_t spinLimit) {
  if (active_) {
    end();
  }

  if (!pinSupportsAnalogInput(inputPin)) {
    return false;
  }
  if (reference == LpcompReference::kExternalAref &&
      !pinSupportsAnalogInput(externalReferencePin)) {
    return false;
  }
  if (!claimComparator(ComparatorOwner::kLpcomp)) {
    return false;
  }
  if (!configureAnalogPeripheralPin(inputPin) ||
      (reference == LpcompReference::kExternalAref &&
       !configureAnalogPeripheralPin(externalReferencePin))) {
    releaseComparator(ComparatorOwner::kLpcomp);
    return false;
  }

  lpcomp_->TASKS_STOP = LPCOMP_TASKS_STOP_TASKS_STOP_Trigger;
  lpcomp_->ENABLE =
      ((LPCOMP_ENABLE_ENABLE_Disabled << LPCOMP_ENABLE_ENABLE_Pos) &
       LPCOMP_ENABLE_ENABLE_Msk);
  lpcomp_->INTENCLR = 0xFFFFFFFFUL;
  lpcomp_->SHORTS = 0U;
  clearEvents();

  lpcomp_->PSEL = makeLpcompPinSelect(inputPin);
  lpcomp_->REFSEL =
      ((static_cast<uint32_t>(reference) << LPCOMP_REFSEL_REFSEL_Pos) &
       LPCOMP_REFSEL_REFSEL_Msk);
  lpcomp_->EXTREFSEL = (reference == LpcompReference::kExternalAref)
                           ? makeLpcompPinSelect(externalReferencePin)
                           : 0U;
  configureAnalogDetect(detect);
  lpcomp_->HYST =
      (((hysteresis ? LPCOMP_HYST_HYST_Enabled
                    : LPCOMP_HYST_HYST_Disabled)
        << LPCOMP_HYST_HYST_Pos) &
       LPCOMP_HYST_HYST_Msk);

  lpcomp_->ENABLE =
      ((LPCOMP_ENABLE_ENABLE_Enabled << LPCOMP_ENABLE_ENABLE_Pos) &
       LPCOMP_ENABLE_ENABLE_Msk);
  clearEvents();
  lpcomp_->TASKS_START = LPCOMP_TASKS_START_TASKS_START_Trigger;
  if (!waitForCompReady(&lpcomp_->EVENTS_READY, spinLimit)) {
    lpcomp_->TASKS_STOP = LPCOMP_TASKS_STOP_TASKS_STOP_Trigger;
    lpcomp_->ENABLE =
        ((LPCOMP_ENABLE_ENABLE_Disabled << LPCOMP_ENABLE_ENABLE_Pos) &
         LPCOMP_ENABLE_ENABLE_Msk);
    releaseComparator(ComparatorOwner::kLpcomp);
    return false;
  }

  active_ = true;
  return true;
}

bool Lpcomp::beginThreshold(const Pin& inputPin,
                            uint16_t thresholdPermille,
                            bool hysteresis,
                            LpcompDetect detect,
                            const Pin& externalReferencePin,
                            uint32_t spinLimit) {
  const uint16_t clamped = clampPermille(thresholdPermille);
  if (isConnected(externalReferencePin)) {
    if (clamped != 1000U) {
      return false;
    }
    return begin(inputPin, LpcompReference::kExternalAref, hysteresis, detect,
                 externalReferencePin, spinLimit);
  }

  return begin(inputPin, nearestLpcompReference(clamped), hysteresis, detect,
               kPinDisconnected, spinLimit);
}

void Lpcomp::configureAnalogDetect(LpcompDetect detect) {
  if (lpcomp_ == nullptr) {
    return;
  }

  lpcomp_->ANADETECT =
      ((static_cast<uint32_t>(detect) << LPCOMP_ANADETECT_ANADETECT_Pos) &
       LPCOMP_ANADETECT_ANADETECT_Msk);
}

bool Lpcomp::sample(uint32_t spinLimit) const {
  (void)spinLimit;
  if (!active_) {
    return false;
  }

  lpcomp_->TASKS_SAMPLE = LPCOMP_TASKS_SAMPLE_TASKS_SAMPLE_Trigger;
  __asm volatile("dsb 0xF" ::: "memory");
  return true;
}

bool Lpcomp::resultAbove() const {
  return lpcomp_ != nullptr &&
         ((lpcomp_->RESULT & LPCOMP_RESULT_RESULT_Msk) != 0U);
}

bool Lpcomp::pollReady(bool clearEventFlag) {
  const bool fired = active_ && (lpcomp_->EVENTS_READY != 0U);
  if (fired && clearEventFlag) {
    lpcomp_->EVENTS_READY = 0U;
  }
  return fired;
}

bool Lpcomp::pollUp(bool clearEventFlag) {
  const bool fired = active_ && (lpcomp_->EVENTS_UP != 0U);
  if (fired && clearEventFlag) {
    lpcomp_->EVENTS_UP = 0U;
  }
  return fired;
}

bool Lpcomp::pollDown(bool clearEventFlag) {
  const bool fired = active_ && (lpcomp_->EVENTS_DOWN != 0U);
  if (fired && clearEventFlag) {
    lpcomp_->EVENTS_DOWN = 0U;
  }
  return fired;
}

bool Lpcomp::pollCross(bool clearEventFlag) {
  const bool fired = active_ && (lpcomp_->EVENTS_CROSS != 0U);
  if (fired && clearEventFlag) {
    lpcomp_->EVENTS_CROSS = 0U;
  }
  return fired;
}

void Lpcomp::clearEvents() {
  if (lpcomp_ == nullptr) {
    return;
  }

  lpcomp_->EVENTS_READY = 0U;
  lpcomp_->EVENTS_UP = 0U;
  lpcomp_->EVENTS_DOWN = 0U;
  lpcomp_->EVENTS_CROSS = 0U;
}

void Lpcomp::end() {
  if (lpcomp_ == nullptr) {
    active_ = false;
    releaseComparator(ComparatorOwner::kLpcomp);
    return;
  }

  lpcomp_->SHORTS = 0U;
  lpcomp_->INTENCLR = 0xFFFFFFFFUL;
  clearEvents();
  lpcomp_->TASKS_STOP = LPCOMP_TASKS_STOP_TASKS_STOP_Trigger;
  lpcomp_->ENABLE =
      ((LPCOMP_ENABLE_ENABLE_Disabled << LPCOMP_ENABLE_ENABLE_Pos) &
       LPCOMP_ENABLE_ENABLE_Msk);
  active_ = false;
  releaseComparator(ComparatorOwner::kLpcomp);
}

Qdec::Qdec(uint32_t base)
    : qdec_(reinterpret_cast<NRF_QDEC_Type*>(static_cast<uintptr_t>(base))),
      configured_(false) {}

bool Qdec::begin(const Pin& pinA,
                 const Pin& pinB,
                 QdecSamplePeriod samplePeriod,
                 QdecReportPeriod reportPeriod,
                 bool debounce,
                 QdecInputPull inputPull,
                 const Pin& ledPin,
                 QdecLedPolarity ledPolarity,
                 uint16_t ledPreUs) {
  if (configured_) {
    end();
  }

  if (!isConnected(pinA) || !isConnected(pinB)) {
    return false;
  }
  if ((pinA.port == pinB.port) && (pinA.pin == pinB.pin)) {
    return false;
  }

  const GpioPull pull = gpioPullFromQdecInputPull(inputPull);
  if (!Gpio::configure(pinA, GpioDirection::kInput, pull) ||
      !Gpio::configure(pinB, GpioDirection::kInput, pull)) {
    return false;
  }
  if (isConnected(ledPin) &&
      !Gpio::configure(ledPin, GpioDirection::kOutput, GpioPull::kDisabled)) {
    return false;
  }

  if (ledPreUs < QDEC_LEDPRE_LEDPRE_Min) {
    ledPreUs = QDEC_LEDPRE_LEDPRE_Min;
  }
  if (ledPreUs > QDEC_LEDPRE_LEDPRE_Max) {
    ledPreUs = QDEC_LEDPRE_LEDPRE_Max;
  }

  qdec_->TASKS_STOP = QDEC_TASKS_STOP_TASKS_STOP_Trigger;
  qdec_->ENABLE = QDEC_ENABLE_ENABLE_Disabled;
  qdec_->INTENCLR = 0xFFFFFFFFUL;
  qdec_->SHORTS = 0U;
  qdec_->EVENTS_SAMPLERDY = 0U;
  qdec_->EVENTS_REPORTRDY = 0U;
  qdec_->EVENTS_ACCOF = 0U;
  qdec_->EVENTS_DBLRDY = 0U;
  qdec_->EVENTS_STOPPED = 0U;

  qdec_->PSEL.A = makeQdecConnectedPinSelect(pinA);
  qdec_->PSEL.B = makeQdecConnectedPinSelect(pinB);
  qdec_->PSEL.LED =
      isConnected(ledPin) ? makeQdecConnectedPinSelect(ledPin)
                          : PSEL_DISCONNECTED;
  qdec_->LEDPOL =
      ((static_cast<uint32_t>(ledPolarity) << QDEC_LEDPOL_LEDPOL_Pos) &
       QDEC_LEDPOL_LEDPOL_Msk);
  qdec_->SAMPLEPER =
      ((static_cast<uint32_t>(samplePeriod) << QDEC_SAMPLEPER_SAMPLEPER_Pos) &
       QDEC_SAMPLEPER_SAMPLEPER_Msk);
  qdec_->REPORTPER =
      ((static_cast<uint32_t>(reportPeriod) << QDEC_REPORTPER_REPORTPER_Pos) &
       QDEC_REPORTPER_REPORTPER_Msk);
  qdec_->DBFEN =
      (((debounce ? QDEC_DBFEN_DBFEN_Enabled : QDEC_DBFEN_DBFEN_Disabled)
        << QDEC_DBFEN_DBFEN_Pos) &
       QDEC_DBFEN_DBFEN_Msk);
  qdec_->LEDPRE = ((static_cast<uint32_t>(ledPreUs) << QDEC_LEDPRE_LEDPRE_Pos) &
                   QDEC_LEDPRE_LEDPRE_Msk);

  qdec_->ENABLE = ((QDEC_ENABLE_ENABLE_Enabled << QDEC_ENABLE_ENABLE_Pos) &
                   QDEC_ENABLE_ENABLE_Msk);
  configured_ = true;
  return true;
}

void Qdec::end() {
  qdec_->TASKS_STOP = QDEC_TASKS_STOP_TASKS_STOP_Trigger;
  qdec_->ENABLE = QDEC_ENABLE_ENABLE_Disabled;
  qdec_->PSEL.A = PSEL_DISCONNECTED;
  qdec_->PSEL.B = PSEL_DISCONNECTED;
  qdec_->PSEL.LED = PSEL_DISCONNECTED;
  qdec_->EVENTS_SAMPLERDY = 0U;
  qdec_->EVENTS_REPORTRDY = 0U;
  qdec_->EVENTS_ACCOF = 0U;
  qdec_->EVENTS_DBLRDY = 0U;
  qdec_->EVENTS_STOPPED = 0U;
  configured_ = false;
}

void Qdec::start() {
  if (configured_) {
    qdec_->TASKS_START = QDEC_TASKS_START_TASKS_START_Trigger;
  }
}

void Qdec::stop() {
  if (configured_) {
    qdec_->TASKS_STOP = QDEC_TASKS_STOP_TASKS_STOP_Trigger;
  }
}

int32_t Qdec::sampleValue() const { return qdec_->SAMPLE; }

int32_t Qdec::accumulator() const { return qdec_->ACC; }

int32_t Qdec::readAndClearAccumulator() {
  if (!configured_) {
    return 0;
  }
  qdec_->TASKS_RDCLRACC = QDEC_TASKS_RDCLRACC_TASKS_RDCLRACC_Trigger;
  __asm volatile("dsb 0xF" ::: "memory");
  return qdec_->ACCREAD;
}

uint32_t Qdec::doubleTransitions() const { return qdec_->ACCDBL; }

uint32_t Qdec::readAndClearDoubleTransitions() {
  if (!configured_) {
    return 0U;
  }
  qdec_->TASKS_RDCLRDBL = QDEC_TASKS_RDCLRDBL_TASKS_RDCLRDBL_Trigger;
  __asm volatile("dsb 0xF" ::: "memory");
  return qdec_->ACCDBLREAD;
}

bool Qdec::pollSampleReady(bool clearEventFlag) {
  const bool fired = (qdec_->EVENTS_SAMPLERDY != 0U);
  if (fired && clearEventFlag) {
    qdec_->EVENTS_SAMPLERDY = 0U;
  }
  return fired;
}

bool Qdec::pollReportReady(bool clearEventFlag) {
  const bool fired = (qdec_->EVENTS_REPORTRDY != 0U);
  if (fired && clearEventFlag) {
    qdec_->EVENTS_REPORTRDY = 0U;
  }
  return fired;
}

bool Qdec::pollOverflow(bool clearEventFlag) {
  const bool fired = (qdec_->EVENTS_ACCOF != 0U);
  if (fired && clearEventFlag) {
    qdec_->EVENTS_ACCOF = 0U;
  }
  return fired;
}

bool Qdec::pollDoubleReady(bool clearEventFlag) {
  const bool fired = (qdec_->EVENTS_DBLRDY != 0U);
  if (fired && clearEventFlag) {
    qdec_->EVENTS_DBLRDY = 0U;
  }
  return fired;
}

bool Qdec::pollStopped(bool clearEventFlag) {
  const bool fired = (qdec_->EVENTS_STOPPED != 0U);
  if (fired && clearEventFlag) {
    qdec_->EVENTS_STOPPED = 0U;
  }
  return fired;
}

bool BoardControl::setAntennaPath(BoardAntennaPath path) {
  switch (path) {
    case BoardAntennaPath::kExternal:
      xiaoNrf54l15SetAntenna(XIAO_NRF54L15_ANTENNA_EXTERNAL);
      return true;
    case BoardAntennaPath::kControlHighImpedance:
      xiaoNrf54l15SetAntenna(XIAO_NRF54L15_ANTENNA_CONTROL_HIZ);
      return true;
    case BoardAntennaPath::kCeramic:
    default:
      xiaoNrf54l15SetAntenna(XIAO_NRF54L15_ANTENNA_CERAMIC);
      return true;
  }
}

BoardAntennaPath BoardControl::antennaPath() {
  switch (xiaoNrf54l15GetAntenna()) {
    case XIAO_NRF54L15_ANTENNA_EXTERNAL:
      return BoardAntennaPath::kExternal;
    case XIAO_NRF54L15_ANTENNA_CONTROL_HIZ:
      return BoardAntennaPath::kControlHighImpedance;
    case XIAO_NRF54L15_ANTENNA_CERAMIC:
    default:
      return BoardAntennaPath::kCeramic;
  }
}

bool BoardControl::setImuMicEnabled(bool enable) {
  return arduinoXiaoNrf54l15SetImuMicEnable(enable ? 1U : 0U) != 0U;
}

bool BoardControl::imuMicEnabled() {
  return arduinoXiaoNrf54l15GetImuMicEnable() != 0U;
}

bool BoardControl::setBatterySenseEnabled(bool enable) {
  return arduinoXiaoNrf54l15SetBatteryEnable(enable ? 1U : 0U) != 0U;
}

bool BoardControl::setRfSwitchPowerEnabled(bool enable) {
  return arduinoXiaoNrf54l15SetRfSwitchPower(enable ? 1U : 0U) != 0U;
}

bool BoardControl::rfSwitchPowerEnabled() {
  return arduinoXiaoNrf54l15GetRfSwitchPower() != 0U;
}

bool BoardControl::enableRfPath(BoardAntennaPath path) {
  return setRfSwitchPowerEnabled(true) && setAntennaPath(path);
}

bool BoardControl::collapseRfPathIdle(BoardAntennaPath idlePath,
                                      bool disablePower) {
  const bool pathOk = setAntennaPath(idlePath);
  if (!disablePower) {
    return pathOk;
  }
  return setRfSwitchPowerEnabled(false) && pathOk;
}

void BoardControl::enterLowestPowerState() {
  xiaoNrf54l15EnterLowestPowerBoardState();
}

bool BoardControl::sampleBatteryMilliVolts(int32_t* outMilliVolts,
                                           uint32_t settleDelayUs,
                                           uint32_t spinLimit) {
  (void)spinLimit;

  if (outMilliVolts == nullptr) {
    return false;
  }

  if (!setBatterySenseEnabled(true)) {
    return false;
  }

  if (settleDelayUs > 0U) {
    delayMicroseconds(settleDelayUs);
  }

  analogReadResolution(12);
  const int raw = analogRead(VBAT_READ);
  (void)setBatterySenseEnabled(false);
  if (raw <= 0) {
    return false;
  }

  const int32_t dividerMilliVolts =
      static_cast<int32_t>(((static_cast<int64_t>(raw) * 3600LL) + 2047LL) /
                           4095LL);
  *outMilliVolts = dividerMilliVolts * 2;
  return true;
}

bool BoardControl::sampleBatteryPercent(uint8_t* outPercent,
                                        int32_t emptyMilliVolts,
                                        int32_t fullMilliVolts,
                                        uint32_t settleDelayUs,
                                        uint32_t spinLimit) {
  if (outPercent == nullptr || fullMilliVolts <= emptyMilliVolts) {
    return false;
  }

  int32_t batteryMilliVolts = 0;
  if (!sampleBatteryMilliVolts(&batteryMilliVolts, settleDelayUs, spinLimit)) {
    return false;
  }

  if (batteryMilliVolts <= emptyMilliVolts) {
    *outPercent = 0U;
    return true;
  }
  if (batteryMilliVolts >= fullMilliVolts) {
    *outPercent = 100U;
    return true;
  }

  const int32_t span = fullMilliVolts - emptyMilliVolts;
  const int32_t scaled =
      (batteryMilliVolts - emptyMilliVolts) * 100 + (span / 2);
  *outPercent = static_cast<uint8_t>(scaled / span);
  return true;
}

PowerManager::PowerManager(uint32_t powerBase,
                           uint32_t resetBase,
                           uint32_t regulatorsBase)
    : power_(reinterpret_cast<NRF_POWER_Type*>(
          static_cast<uintptr_t>(powerBase))),
      reset_(reinterpret_cast<NRF_RESET_Type*>(
          static_cast<uintptr_t>(resetBase))),
      regulators_(reinterpret_cast<NRF_REGULATORS_Type*>(
          static_cast<uintptr_t>(regulatorsBase))) {}

void PowerManager::setLatencyMode(PowerLatencyMode mode) {
  if (mode == PowerLatencyMode::kConstantLatency) {
    power_->TASKS_CONSTLAT = POWER_TASKS_CONSTLAT_TASKS_CONSTLAT_Trigger;
  } else {
    power_->TASKS_LOWPWR = POWER_TASKS_LOWPWR_TASKS_LOWPWR_Trigger;
  }
}

bool PowerManager::isConstantLatency() const {
  return (power_->CONSTLATSTAT & 0x1UL) != 0U;
}

bool PowerManager::setRetention(uint8_t index, uint8_t value) {
  if (index >= 2U) {
    return false;
  }
  power_->GPREGRET[index] = static_cast<uint32_t>(value);
  return true;
}

bool PowerManager::getRetention(uint8_t index, uint8_t* value) const {
  if (index >= 2U || value == nullptr) {
    return false;
  }

  *value = static_cast<uint8_t>(power_->GPREGRET[index] &
                                POWER_GPREGRET_GPREGRET_Msk);
  return true;
}

uint32_t PowerManager::resetReason() const {
  return reset_->RESETREAS;
}

void PowerManager::clearResetReason(uint32_t mask) {
  reset_->RESETREAS = mask;
}

bool PowerManager::enableMainDcdc(bool enable) {
  regulators_->VREGMAIN.DCDCEN =
      enable ? REGULATORS_VREGMAIN_DCDCEN_VAL_Enabled
             : REGULATORS_VREGMAIN_DCDCEN_VAL_Disabled;

  const bool isEnabled = (regulators_->VREGMAIN.DCDCEN &
                          REGULATORS_VREGMAIN_DCDCEN_VAL_Msk) != 0U;
  return isEnabled == enable;
}

bool PowerManager::configurePowerFailComparator(PowerFailThreshold threshold,
                                                bool enableWarningEvent) {
  uint32_t config = regulators_->POFCON;
  config &= ~(REGULATORS_POFCON_POF_Msk | REGULATORS_POFCON_THRESHOLD_Msk |
              REGULATORS_POFCON_EVENTDISABLE_Msk);
  config |= (REGULATORS_POFCON_POF_Enabled << REGULATORS_POFCON_POF_Pos) &
            REGULATORS_POFCON_POF_Msk;
  config |= ((static_cast<uint32_t>(threshold)
              << REGULATORS_POFCON_THRESHOLD_Pos) &
             REGULATORS_POFCON_THRESHOLD_Msk);
  config |= (((enableWarningEvent ? REGULATORS_POFCON_EVENTDISABLE_Enabled
                                  : REGULATORS_POFCON_EVENTDISABLE_Disabled)
              << REGULATORS_POFCON_EVENTDISABLE_Pos) &
             REGULATORS_POFCON_EVENTDISABLE_Msk);
  regulators_->POFCON = config;
  power_->EVENTS_POFWARN = POWER_EVENTS_POFWARN_EVENTS_POFWARN_NotGenerated;
  return powerFailComparatorEnabled() &&
         (powerFailThreshold() == threshold) &&
         (powerFailWarningEventEnabled() == enableWarningEvent);
}

void PowerManager::disablePowerFailComparator() {
  regulators_->POFCON &= ~REGULATORS_POFCON_POF_Msk;
  clearPowerFailWarning();
}

bool PowerManager::powerFailComparatorEnabled() const {
  return (regulators_->POFCON & REGULATORS_POFCON_POF_Msk) != 0U;
}

PowerFailThreshold PowerManager::powerFailThreshold() const {
  return static_cast<PowerFailThreshold>(
      (regulators_->POFCON & REGULATORS_POFCON_THRESHOLD_Msk) >>
      REGULATORS_POFCON_THRESHOLD_Pos);
}

bool PowerManager::powerBelowPowerFailThreshold() const {
  return (regulators_->POFSTAT & REGULATORS_POFSTAT_COMPARATOR_Msk) != 0U;
}

bool PowerManager::powerFailWarningEventEnabled() const {
  return (regulators_->POFCON & REGULATORS_POFCON_EVENTDISABLE_Msk) == 0U;
}

bool PowerManager::pollPowerFailWarning(bool clearEvent) {
  const bool fired =
      (power_->EVENTS_POFWARN & POWER_EVENTS_POFWARN_EVENTS_POFWARN_Msk) != 0U;
  if (fired && clearEvent) {
    clearPowerFailWarning();
  }
  return fired;
}

void PowerManager::clearPowerFailWarning() {
  power_->EVENTS_POFWARN = POWER_EVENTS_POFWARN_EVENTS_POFWARN_NotGenerated;
}

[[noreturn]] void PowerManager::systemOff() {
  enterSystemOff(reset_, regulators_, false);
}

[[noreturn]] void PowerManager::systemOffTimedWakeMs(uint32_t delayMs) {
  if (delayMs == 0U) {
    delayMs = 1U;
  }
  delaySystemOff(delayMs);
  while (true) {
    __asm volatile("wfe");
  }
}

[[noreturn]] void PowerManager::systemOffTimedWakeUs(uint32_t delayUs) {
  uint32_t delayMs = (delayUs + 999UL) / 1000UL;
  if (delayMs == 0U) {
    delayMs = 1U;
  }
  systemOffTimedWakeMs(delayMs);
}

[[noreturn]] void PowerManager::systemOffNoRetention() {
  enterSystemOff(reset_, regulators_, true);
}

[[noreturn]] void PowerManager::systemOffTimedWakeMsNoRetention(
    uint32_t delayMs) {
  if (delayMs == 0U) {
    delayMs = 1U;
  }
  delaySystemOffNoRetention(delayMs);
  while (true) {
    __asm volatile("wfe");
  }
}

[[noreturn]] void PowerManager::systemOffTimedWakeUsNoRetention(
    uint32_t delayUs) {
  uint32_t delayMs = (delayUs + 999UL) / 1000UL;
  if (delayMs == 0U) {
    delayMs = 1U;
  }
  systemOffTimedWakeMsNoRetention(delayMs);
}

Grtc::Grtc(uint32_t base, uint8_t compareChannelCount)
    : counter_(nullptr),
      compareChannelCount_(compareChannelCount > 12U ? 12U : compareChannelCount),
      wakeLeadLfclk_(4U),
      compareEvents_{false},
      actualChannels_{0xFF},
      interruptsEnabled_{false},
      alarmContexts_{},
      softwareRunning_(false),
      softwareBaseMicros_(0U),
      stoppedCounterUs_(0U),
      compareDeadlineUs_{0U},
      compareArmed_{false} {
  (void)base;
}

bool Grtc::begin(GrtcClockSource clockSource) {
  ARG_UNUSED(clockSource);

  for (uint8_t i = 0U; i < compareChannelCount_; ++i) {
    compareEvents_[i] = false;
    actualChannels_[i] = 0xFFU;
    interruptsEnabled_[i] = false;
    alarmContexts_[i].owner = this;
    alarmContexts_[i].logicalChannel = i;
    compareDeadlineUs_[i] = 0U;
    compareArmed_[i] = false;
  }
  stoppedCounterUs_ = 0U;
  softwareBaseMicros_ = micros();
  softwareRunning_ = true;
  return true;
}

void Grtc::end() {
  for (uint8_t i = 0U; i < compareChannelCount_; ++i) {
    actualChannels_[i] = 0xFFU;
    compareEvents_[i] = false;
    interruptsEnabled_[i] = false;
    compareDeadlineUs_[i] = 0U;
    compareArmed_[i] = false;
  }
  softwareRunning_ = false;
}

void Grtc::start() {
  if (!softwareRunning_) {
    softwareBaseMicros_ = micros();
    softwareRunning_ = true;
  }
}

void Grtc::stop() {
  if (softwareRunning_) {
    stoppedCounterUs_ += micros() - softwareBaseMicros_;
    softwareRunning_ = false;
  }
}

void Grtc::clear() {
  stoppedCounterUs_ = 0U;
  softwareBaseMicros_ = micros();
  for (uint8_t i = 0U; i < compareChannelCount_; ++i) {
    compareEvents_[i] = false;
    compareArmed_[i] = false;
    compareDeadlineUs_[i] = 0U;
  }
}

uint64_t Grtc::counter() const {
  if (!softwareRunning_) {
    return stoppedCounterUs_;
  }
  return stoppedCounterUs_ + static_cast<uint64_t>(micros() - softwareBaseMicros_);
}

bool Grtc::setWakeLeadLfclk(uint8_t cycles) {
  wakeLeadLfclk_ = (cycles == 0U) ? 1U : cycles;
  return true;
}

bool Grtc::setCompareOffsetUs(uint8_t channel,
                              uint32_t offsetUs,
                              bool enableChannel) {
  if (channel >= compareChannelCount_ || !enableChannel) {
    return channel < compareChannelCount_;
  }

  if (offsetUs == 0U) {
    offsetUs = 1U;
  }
  compareDeadlineUs_[channel] = counter() + static_cast<uint64_t>(offsetUs);
  compareEvents_[channel] = false;
  compareArmed_[channel] = true;
  actualChannels_[channel] = channel;
  interruptsEnabled_[channel] = true;
  return true;
}

bool Grtc::setCompareAbsoluteUs(uint8_t channel,
                                uint64_t timestampUs,
                                bool enableChannel) {
  if (channel >= compareChannelCount_ || !enableChannel) {
    return channel < compareChannelCount_;
  }
  compareDeadlineUs_[channel] = timestampUs;
  compareEvents_[channel] = false;
  compareArmed_[channel] = true;
  actualChannels_[channel] = channel;
  interruptsEnabled_[channel] = true;
  return true;
}

bool Grtc::enableCompareChannel(uint8_t channel, bool enable) {
  if (channel >= compareChannelCount_) {
    return false;
  }
  if (!enable) {
    compareArmed_[channel] = false;
    actualChannels_[channel] = 0xFFU;
  }
  return true;
}

void Grtc::enableCompareInterrupt(uint8_t channel, bool enable) {
  if (channel < compareChannelCount_) {
    interruptsEnabled_[channel] = enable;
  }
}

bool Grtc::pollCompare(uint8_t channel, bool clearEvent) {
  if (channel >= compareChannelCount_) {
    return false;
  }

  if (compareArmed_[channel] && counter() >= compareDeadlineUs_[channel]) {
    compareArmed_[channel] = false;
    compareEvents_[channel] = true;
  }

  const bool fired = compareEvents_[channel];
  if (fired && clearEvent) {
    compareEvents_[channel] = false;
  }
  return fired;
}

bool Grtc::clearCompareEvent(uint8_t channel) {
  if (channel >= compareChannelCount_) {
    return false;
  }

  compareEvents_[channel] = false;
  return true;
}

void Grtc::markCompareEvent(uint8_t channel) {
  if (channel < compareChannelCount_) {
    compareEvents_[channel] = true;
    compareArmed_[channel] = false;
  }
}

TempSensor::TempSensor(uint32_t base)
    : temp_(reinterpret_cast<NRF_TEMP_Type*>(static_cast<uintptr_t>(base))) {}

bool TempSensor::sampleQuarterDegreesC(int32_t* outQuarterDegreesC,
                                       uint32_t spinLimit) const {
  if (outQuarterDegreesC == nullptr) {
    return false;
  }

  temp_->EVENTS_DATARDY = 0U;
  temp_->TASKS_START = TEMP_TASKS_START_TASKS_START_Trigger;
  while (spinLimit-- > 0U) {
    if (temp_->EVENTS_DATARDY != 0U) {
      temp_->TASKS_STOP = TEMP_TASKS_STOP_TASKS_STOP_Trigger;
      *outQuarterDegreesC = temp_->TEMP;
      temp_->EVENTS_DATARDY = 0U;
      return true;
    }
  }

  temp_->TASKS_STOP = TEMP_TASKS_STOP_TASKS_STOP_Trigger;
  return false;
}

bool TempSensor::sampleMilliDegreesC(int32_t* outMilliDegreesC,
                                     uint32_t spinLimit) const {
  if (outMilliDegreesC == nullptr) {
    return false;
  }

  int32_t quarterDegrees = 0;
  if (!sampleQuarterDegreesC(&quarterDegrees, spinLimit)) {
    return false;
  }

  *outMilliDegreesC = quarterDegrees * 250;
  return true;
}

Watchdog::Watchdog(uint32_t base)
    : wdt_(reinterpret_cast<NRF_WDT_Type*>(static_cast<uintptr_t>(base))),
      defaultReloadRegister_(0U),
      allowStop_(false) {}

bool Watchdog::configure(uint32_t timeoutMs,
                         uint8_t reloadRegister,
                         bool runInSleep,
                         bool runInDebugHalt,
                         bool allowStop) {
  if (reloadRegister > 7U || timeoutMs == 0U || isRunning()) {
    return false;
  }

  uint64_t cycles64 =
      (static_cast<uint64_t>(timeoutMs) * 32768ULL + 999ULL) / 1000ULL;
  if (cycles64 < WDT_CRV_CRV_Min) {
    cycles64 = WDT_CRV_CRV_Min;
  }
  if (cycles64 > WDT_CRV_CRV_Max) {
    cycles64 = WDT_CRV_CRV_Max;
  }

  uint32_t cfg = 0U;
  cfg |= ((runInSleep ? WDT_CONFIG_SLEEP_Run : WDT_CONFIG_SLEEP_Pause)
          << WDT_CONFIG_SLEEP_Pos);
  cfg |= ((runInDebugHalt ? WDT_CONFIG_HALT_Run : WDT_CONFIG_HALT_Pause)
          << WDT_CONFIG_HALT_Pos);
  cfg |= ((allowStop ? WDT_CONFIG_STOPEN_Enable : WDT_CONFIG_STOPEN_Disable)
          << WDT_CONFIG_STOPEN_Pos);

  wdt_->CRV = static_cast<uint32_t>(cycles64);
  wdt_->RREN = (1UL << reloadRegister);
  wdt_->CONFIG = cfg;
  if (allowStop) {
    wdt_->TSEN = WDT_TSEN_TSEN_Enable;
  }

  defaultReloadRegister_ = reloadRegister;
  allowStop_ = allowStop;
  return true;
}

void Watchdog::start() {
  wdt_->TASKS_START = WDT_TASKS_START_TASKS_START_Trigger;
}

bool Watchdog::stop(uint32_t spinLimit) {
  if (!allowStop_) {
    return false;
  }

  wdt_->EVENTS_STOPPED = 0U;
  wdt_->TASKS_STOP = WDT_TASKS_STOP_TASKS_STOP_Trigger;
  while (spinLimit-- > 0U) {
    if (wdt_->EVENTS_STOPPED != 0U) {
      return true;
    }
  }
  return false;
}

bool Watchdog::feed(uint8_t reloadRegister) {
  if (reloadRegister > 7U) {
    reloadRegister = defaultReloadRegister_;
  }
  wdt_->RR[reloadRegister] = WDT_RR_RR_Reload;
  return true;
}

bool Watchdog::isRunning() const {
  return (wdt_->RUNSTATUS & WDT_RUNSTATUS_RUNSTATUSWDT_Msk) != 0U;
}

uint32_t Watchdog::requestStatus() const {
  return wdt_->REQSTATUS;
}

Uarte::Uarte(uint32_t base) : base_(base), serial_(nullptr), active_(false) {}

bool Uarte::begin(const Pin& txd,
                  const Pin& rxd,
                  UarteBaud baud,
                  bool hwFlowControl,
                  const Pin& cts,
                  const Pin& rts) {
  (void)cts;
  (void)rts;

  if (!isConnected(txd) || !isConnected(rxd) || hwFlowControl) {
    return false;
  }

  uint8_t txDigitalPin = 0U;
  uint8_t rxDigitalPin = 0U;
  if (!pinToDigitalPin(txd, &txDigitalPin) ||
      !pinToDigitalPin(rxd, &rxDigitalPin)) {
    return false;
  }

  HardwareSerial* serial = resolveSerialForPins(txd, rxd);
  if (serial == nullptr) {
    serial = resolveSerialForBase(base_);
  }
  if (serial == nullptr) {
    return false;
  }

  if (!Gpio::configure(txd, GpioDirection::kOutput, GpioPull::kDisabled) ||
      !Gpio::configure(rxd, GpioDirection::kInput, GpioPull::kDisabled)) {
    return false;
  }
  (void)Gpio::write(txd, true);

  if (!serial->setPins(static_cast<int8_t>(rxDigitalPin),
                       static_cast<int8_t>(txDigitalPin))) {
    return false;
  }

  serial->begin(static_cast<unsigned long>(baud));
  serial_ = serial;
  active_ = serial->isConfigured();
  return active_;
}

void Uarte::end() {
  auto* serial = static_cast<HardwareSerial*>(serial_);
  if (serial != nullptr) {
    serial->end();
  }
  active_ = false;
}

bool Uarte::write(const uint8_t* data, size_t len, uint32_t spinLimit) {
  (void)spinLimit;

  auto* serial = static_cast<HardwareSerial*>(serial_);
  if (!active_ || serial == nullptr || data == nullptr) {
    return false;
  }
  if (len == 0U) {
    return true;
  }

  const size_t written = serial->write(data, len);
  serial->flush();
  return written == len;
}

size_t Uarte::read(uint8_t* data, size_t len, uint32_t spinLimit) {
  auto* serial = static_cast<HardwareSerial*>(serial_);
  if (!active_ || serial == nullptr || data == nullptr || len == 0U) {
    return 0U;
  }

  size_t count = 0U;
  uint32_t deadlineStart = micros();
  while (count < len && (micros() - deadlineStart) < spinLimit) {
    const int value = serial->read();
    if (value < 0) {
      delay(1);
      continue;
    }
    data[count++] = static_cast<uint8_t>(value);
    deadlineStart = micros();
  }
  return count;
}

Pdm::Pdm(uint32_t base) : base_(base), configured_(false), mono_(true) {}

bool Pdm::begin(const Pin& clk,
                const Pin& din,
                bool mono,
                uint8_t prescalerDiv,
                uint8_t ratio,
                PdmEdge edge) {
  (void)base_;
  (void)edge;

  if (!isConnected(clk) || !isConnected(din)) {
    return false;
  }
  if (!samePin(clk, kPinMicClk) || !samePin(din, kPinMicData)) {
    return false;
  }

  mono_ = mono;
  if (!BoardControl::setImuMicEnabled(true)) {
    return false;
  }
  delay(5);

  const uint32_t sampleRate = pdmSampleRateFromConfig(prescalerDiv, ratio);
  configured_ = PDM.begin(sampleRate, mono_ ? 1U : 2U);
  if (!configured_) {
    BoardControl::setImuMicEnabled(false);
  }
  return configured_;
}

void Pdm::end() {
  PDM.end();
  configured_ = false;
  (void)BoardControl::setImuMicEnabled(false);
}

bool Pdm::capture(int16_t* samples, size_t sampleCount, uint32_t spinLimit) {
  if (!configured_ || samples == nullptr || sampleCount == 0U) {
    return false;
  }

  memset(samples, 0, sampleCount * sizeof(int16_t));
  uint8_t* buffer = reinterpret_cast<uint8_t*>(samples);
  const size_t totalBytes = sampleCount * sizeof(int16_t);
  size_t offset = 0U;
  uint32_t lastProgress = micros();
  while (offset < totalBytes && (micros() - lastProgress) < spinLimit) {
    int availableBytes = PDM.available();
    if (availableBytes <= 0) {
      delay(1);
      continue;
    }

    size_t chunk = totalBytes - offset;
    if (chunk > static_cast<size_t>(availableBytes)) {
      chunk = static_cast<size_t>(availableBytes);
    }

    const int readBytes = PDM.read(&buffer[offset], chunk);
    if (readBytes <= 0) {
      delay(1);
      continue;
    }

    offset += static_cast<size_t>(readBytes);
    lastProgress = micros();
  }

  return offset == totalBytes;
}

I2sTx::I2sTx(uint32_t base)
    : i2s_(reinterpret_cast<NRF_I2S_Type*>(static_cast<uintptr_t>(base))),
      config_(),
      buffers_{nullptr, nullptr},
      wordCount_(0U),
      refillCallback_(nullptr),
      refillContext_(nullptr),
      nextBufferIndex_(1U),
      configured_(false),
      running_(false),
      restartPending_(false),
      txPtrUpdCount_(0U),
      stoppedCount_(0U),
      restartCount_(0U),
      manualStopCount_(0U) {}

bool I2sTx::setBuffers(uint32_t* buffer0, uint32_t* buffer1, uint32_t wordCount) {
  if (buffer0 == nullptr || buffer1 == nullptr || wordCount == 0U) {
    return false;
  }
  if ((reinterpret_cast<uintptr_t>(buffer0) & 0x3U) != 0U ||
      (reinterpret_cast<uintptr_t>(buffer1) & 0x3U) != 0U) {
    return false;
  }
  if (wordCount >
      (I2S_RXTXD_MAXCNT_MAXCNT_Msk >> I2S_RXTXD_MAXCNT_MAXCNT_Pos)) {
    return false;
  }

  buffers_[0] = buffer0;
  buffers_[1] = buffer1;
  wordCount_ = wordCount;
  return true;
}

void I2sTx::setRefillCallback(RefillCallback callback, void* context) {
  refillCallback_ = callback;
  refillContext_ = context;
}

bool I2sTx::begin(const I2sTxConfig& config,
                  uint32_t* buffer0,
                  uint32_t* buffer1,
                  uint32_t wordCount) {
  if (i2s_ == nullptr || !setBuffers(buffer0, buffer1, wordCount)) {
    return false;
  }
  if (!isConnected(config.mck) || !isConnected(config.sck) ||
      !isConnected(config.lrck) || !isConnected(config.sdout)) {
    return false;
  }
  if (!Gpio::configure(config.mck, GpioDirection::kOutput, GpioPull::kDisabled) ||
      !Gpio::configure(config.sck, GpioDirection::kOutput, GpioPull::kDisabled) ||
      !Gpio::configure(config.lrck, GpioDirection::kOutput, GpioPull::kDisabled) ||
      !Gpio::configure(config.sdout, GpioDirection::kOutput, GpioPull::kDisabled)) {
    return false;
  }
  if (isConnected(config.sdin) &&
      !Gpio::configure(config.sdin, GpioDirection::kInput, GpioPull::kDisabled)) {
    return false;
  }

  config_ = config;

  i2s_->TASKS_STOP = I2S_TASKS_STOP_TASKS_STOP_Trigger;
  i2s_->ENABLE = (I2S_ENABLE_ENABLE_Disabled << I2S_ENABLE_ENABLE_Pos) &
                 I2S_ENABLE_ENABLE_Msk;

  i2s_->PSEL.MCK = make_psel(config_.mck.port, config_.mck.pin);
  i2s_->PSEL.SCK = make_psel(config_.sck.port, config_.sck.pin);
  i2s_->PSEL.LRCK = make_psel(config_.lrck.port, config_.lrck.pin);
  i2s_->PSEL.SDIN =
      isConnected(config_.sdin) ? make_psel(config_.sdin.port, config_.sdin.pin)
                                : PSEL_DISCONNECTED;
  i2s_->PSEL.SDOUT = make_psel(config_.sdout.port, config_.sdout.pin);

  i2s_->CONFIG.MODE = (I2S_CONFIG_MODE_MODE_Master << I2S_CONFIG_MODE_MODE_Pos) &
                      I2S_CONFIG_MODE_MODE_Msk;
  i2s_->CONFIG.RXEN =
      (I2S_CONFIG_RXEN_RXEN_Disabled << I2S_CONFIG_RXEN_RXEN_Pos) &
      I2S_CONFIG_RXEN_RXEN_Msk;
  i2s_->CONFIG.TXEN =
      (I2S_CONFIG_TXEN_TXEN_Enabled << I2S_CONFIG_TXEN_TXEN_Pos) &
      I2S_CONFIG_TXEN_TXEN_Msk;
  i2s_->CONFIG.MCKEN =
      ((config_.enableMasterClock ? I2S_CONFIG_MCKEN_MCKEN_Enabled
                                  : I2S_CONFIG_MCKEN_MCKEN_Disabled)
       << I2S_CONFIG_MCKEN_MCKEN_Pos) &
      I2S_CONFIG_MCKEN_MCKEN_Msk;
  i2s_->CONFIG.MCKFREQ = config_.mckFreq;
  i2s_->CONFIG.RATIO =
      (config_.ratio << I2S_CONFIG_RATIO_RATIO_Pos) & I2S_CONFIG_RATIO_RATIO_Msk;
  i2s_->CONFIG.SWIDTH =
      (config_.sampleWidth << I2S_CONFIG_SWIDTH_SWIDTH_Pos) &
      I2S_CONFIG_SWIDTH_SWIDTH_Msk;
  i2s_->CONFIG.ALIGN =
      (config_.align << I2S_CONFIG_ALIGN_ALIGN_Pos) & I2S_CONFIG_ALIGN_ALIGN_Msk;
  i2s_->CONFIG.FORMAT =
      (config_.format << I2S_CONFIG_FORMAT_FORMAT_Pos) & I2S_CONFIG_FORMAT_FORMAT_Msk;
  i2s_->CONFIG.CHANNELS =
      (config_.channels << I2S_CONFIG_CHANNELS_CHANNELS_Pos) &
      I2S_CONFIG_CHANNELS_CHANNELS_Msk;

  i2s_->RXTXD.MAXCNT = (wordCount_ << I2S_RXTXD_MAXCNT_MAXCNT_Pos) &
                       I2S_RXTXD_MAXCNT_MAXCNT_Msk;
  i2s_->INTENCLR = I2S_INTENCLR_TXPTRUPD_Msk | I2S_INTENCLR_STOPPED_Msk;
  clearEvents();
  if (refillCallback_ != nullptr) {
    refillCallback_(buffers_[0], wordCount_, refillContext_);
    refillCallback_(buffers_[1], wordCount_, refillContext_);
  }
  armBuffer(0U);

  i2s_->ENABLE = (I2S_ENABLE_ENABLE_Enabled << I2S_ENABLE_ENABLE_Pos) &
                 I2S_ENABLE_ENABLE_Msk;

  ensureI2sIrqConnected(config_.irqPriority);

  nextBufferIndex_ = 1U;
  configured_ = true;
  running_ = false;
  restartPending_ = false;
  txPtrUpdCount_ = 0U;
  stoppedCount_ = 0U;
  restartCount_ = 0U;
  manualStopCount_ = 0U;
  return true;
}

void I2sTx::end() {
  if (!configured_ || i2s_ == nullptr) {
    return;
  }

  i2s_->INTENCLR = I2S_INTENCLR_TXPTRUPD_Msk | I2S_INTENCLR_STOPPED_Msk;
  i2s_->TASKS_STOP = I2S_TASKS_STOP_TASKS_STOP_Trigger;
  waitForNonZero(&i2s_->EVENTS_STOPPED, 300000UL);
  i2s_->ENABLE = (I2S_ENABLE_ENABLE_Disabled << I2S_ENABLE_ENABLE_Pos) &
                 I2S_ENABLE_ENABLE_Msk;
  i2s_->PSEL.MCK = PSEL_DISCONNECTED;
  i2s_->PSEL.SCK = PSEL_DISCONNECTED;
  i2s_->PSEL.LRCK = PSEL_DISCONNECTED;
  i2s_->PSEL.SDIN = PSEL_DISCONNECTED;
  i2s_->PSEL.SDOUT = PSEL_DISCONNECTED;
  if (g_activeI2sTx == this) {
    g_activeI2sTx = nullptr;
  }
  configured_ = false;
  running_ = false;
  restartPending_ = false;
}

bool I2sTx::start() {
  if (!configured_ || i2s_ == nullptr) {
    return false;
  }

  clearEvents();
  nextBufferIndex_ = 1U;
  armBuffer(0U);
  i2s_->INTENSET = I2S_INTENSET_TXPTRUPD_Msk | I2S_INTENSET_STOPPED_Msk;
  running_ = true;
  restartPending_ = false;
  i2s_->TASKS_START = I2S_TASKS_START_TASKS_START_Trigger;
  return true;
}

bool I2sTx::stop() {
  if (!configured_ || !running_ || i2s_ == nullptr) {
    return false;
  }

  ++manualStopCount_;
  i2s_->TASKS_STOP = I2S_TASKS_STOP_TASKS_STOP_Trigger;
  return true;
}

void I2sTx::service() {
  if (restartPending_ && config_.autoRestart) {
    restartPending_ = false;
    ++restartCount_;
    (void)start();
  }
}

void I2sTx::onIrq() {
  if (!configured_ || i2s_ == nullptr) {
    return;
  }

  if (i2s_->EVENTS_TXPTRUPD != 0U) {
    i2s_->EVENTS_TXPTRUPD = 0U;
    const uint8_t queuedIndex = nextBufferIndex_;
    armBuffer(queuedIndex);
    nextBufferIndex_ ^= 1U;
    if (refillCallback_ != nullptr) {
      refillCallback_(buffers_[nextBufferIndex_], wordCount_, refillContext_);
    }
    ++txPtrUpdCount_;
  }

  if (i2s_->EVENTS_STOPPED != 0U) {
    i2s_->EVENTS_STOPPED = 0U;
    running_ = false;
    restartPending_ = true;
    ++stoppedCount_;
  }
}

bool I2sTx::makeActive() {
  if (!configured_) {
    return false;
  }
  g_activeI2sDuplex = nullptr;
  g_activeI2sRx = nullptr;
  g_i2sRawHandler = nullptr;
  g_activeI2sTx = this;
  return true;
}

void I2sTx::irqHandler() {
  if (g_activeI2sTx != nullptr) {
    g_activeI2sTx->onIrq();
  }
}

bool I2sTx::configured() const { return configured_; }
bool I2sTx::running() const { return running_; }
bool I2sTx::restartPending() const { return restartPending_; }
uint32_t I2sTx::txPtrUpdCount() const { return txPtrUpdCount_; }
uint32_t I2sTx::stoppedCount() const { return stoppedCount_; }
uint32_t I2sTx::restartCount() const { return restartCount_; }
uint32_t I2sTx::manualStopCount() const { return manualStopCount_; }

void I2sTx::clearEvents() {
  i2s_->EVENTS_TXPTRUPD = 0U;
  i2s_->EVENTS_STOPPED = 0U;
}

void I2sTx::armBuffer(uint8_t bufferIndex) {
  const uint8_t index = bufferIndex & 1U;
  i2s_->TXD.PTR =
      static_cast<uint32_t>(reinterpret_cast<uintptr_t>(buffers_[index])) &
      I2S_TXD_PTR_PTR_Msk;
}

I2sRx::I2sRx(uint32_t base)
    : i2s_(reinterpret_cast<NRF_I2S_Type*>(static_cast<uintptr_t>(base))),
      config_(),
      buffers_{nullptr, nullptr},
      wordCount_(0U),
      receiveCallback_(nullptr),
      receiveContext_(nullptr),
      nextBufferIndex_(1U),
      configured_(false),
      running_(false),
      restartPending_(false),
      rxPtrUpdCount_(0U),
      stoppedCount_(0U),
      restartCount_(0U),
      manualStopCount_(0U) {}

bool I2sRx::setBuffers(uint32_t* buffer0, uint32_t* buffer1, uint32_t wordCount) {
  if (buffer0 == nullptr || buffer1 == nullptr || wordCount == 0U) {
    return false;
  }
  if ((reinterpret_cast<uintptr_t>(buffer0) & 0x3U) != 0U ||
      (reinterpret_cast<uintptr_t>(buffer1) & 0x3U) != 0U) {
    return false;
  }
  if (wordCount >
      (I2S_RXTXD_MAXCNT_MAXCNT_Msk >> I2S_RXTXD_MAXCNT_MAXCNT_Pos)) {
    return false;
  }

  buffers_[0] = buffer0;
  buffers_[1] = buffer1;
  wordCount_ = wordCount;
  return true;
}

void I2sRx::setReceiveCallback(ReceiveCallback callback, void* context) {
  receiveCallback_ = callback;
  receiveContext_ = context;
}

bool I2sRx::begin(const I2sRxConfig& config,
                  uint32_t* buffer0,
                  uint32_t* buffer1,
                  uint32_t wordCount) {
  if (i2s_ == nullptr || !setBuffers(buffer0, buffer1, wordCount)) {
    return false;
  }
  if (!isConnected(config.mck) || !isConnected(config.sck) ||
      !isConnected(config.lrck) || !isConnected(config.sdin)) {
    return false;
  }
  if (!Gpio::configure(config.mck, GpioDirection::kOutput, GpioPull::kDisabled) ||
      !Gpio::configure(config.sck, GpioDirection::kOutput, GpioPull::kDisabled) ||
      !Gpio::configure(config.lrck, GpioDirection::kOutput, GpioPull::kDisabled) ||
      !Gpio::configure(config.sdin, GpioDirection::kInput, GpioPull::kDisabled)) {
    return false;
  }
  if (isConnected(config.sdout) &&
      !Gpio::configure(config.sdout, GpioDirection::kOutput, GpioPull::kDisabled)) {
    return false;
  }

  config_ = config;

  i2s_->TASKS_STOP = I2S_TASKS_STOP_TASKS_STOP_Trigger;
  i2s_->ENABLE = (I2S_ENABLE_ENABLE_Disabled << I2S_ENABLE_ENABLE_Pos) &
                 I2S_ENABLE_ENABLE_Msk;

  i2s_->PSEL.MCK = make_psel(config_.mck.port, config_.mck.pin);
  i2s_->PSEL.SCK = make_psel(config_.sck.port, config_.sck.pin);
  i2s_->PSEL.LRCK = make_psel(config_.lrck.port, config_.lrck.pin);
  i2s_->PSEL.SDIN = make_psel(config_.sdin.port, config_.sdin.pin);
  i2s_->PSEL.SDOUT =
      isConnected(config_.sdout) ? make_psel(config_.sdout.port, config_.sdout.pin)
                                 : PSEL_DISCONNECTED;

  i2s_->CONFIG.MODE = (I2S_CONFIG_MODE_MODE_Master << I2S_CONFIG_MODE_MODE_Pos) &
                      I2S_CONFIG_MODE_MODE_Msk;
  i2s_->CONFIG.RXEN =
      (I2S_CONFIG_RXEN_RXEN_Enabled << I2S_CONFIG_RXEN_RXEN_Pos) &
      I2S_CONFIG_RXEN_RXEN_Msk;
  i2s_->CONFIG.TXEN =
      (I2S_CONFIG_TXEN_TXEN_Disabled << I2S_CONFIG_TXEN_TXEN_Pos) &
      I2S_CONFIG_TXEN_TXEN_Msk;
  i2s_->CONFIG.MCKEN =
      ((config_.enableMasterClock ? I2S_CONFIG_MCKEN_MCKEN_Enabled
                                  : I2S_CONFIG_MCKEN_MCKEN_Disabled)
       << I2S_CONFIG_MCKEN_MCKEN_Pos) &
      I2S_CONFIG_MCKEN_MCKEN_Msk;
  i2s_->CONFIG.MCKFREQ = config_.mckFreq;
  i2s_->CONFIG.RATIO =
      (config_.ratio << I2S_CONFIG_RATIO_RATIO_Pos) & I2S_CONFIG_RATIO_RATIO_Msk;
  i2s_->CONFIG.SWIDTH =
      (config_.sampleWidth << I2S_CONFIG_SWIDTH_SWIDTH_Pos) &
      I2S_CONFIG_SWIDTH_SWIDTH_Msk;
  i2s_->CONFIG.ALIGN =
      (config_.align << I2S_CONFIG_ALIGN_ALIGN_Pos) & I2S_CONFIG_ALIGN_ALIGN_Msk;
  i2s_->CONFIG.FORMAT =
      (config_.format << I2S_CONFIG_FORMAT_FORMAT_Pos) & I2S_CONFIG_FORMAT_FORMAT_Msk;
  i2s_->CONFIG.CHANNELS =
      (config_.channels << I2S_CONFIG_CHANNELS_CHANNELS_Pos) &
      I2S_CONFIG_CHANNELS_CHANNELS_Msk;

  i2s_->RXTXD.MAXCNT = (wordCount_ << I2S_RXTXD_MAXCNT_MAXCNT_Pos) &
                       I2S_RXTXD_MAXCNT_MAXCNT_Msk;
  i2s_->INTENCLR = I2S_INTENCLR_RXPTRUPD_Msk | I2S_INTENCLR_STOPPED_Msk;
  clearEvents();
  armBuffer(0U);

  i2s_->ENABLE = (I2S_ENABLE_ENABLE_Enabled << I2S_ENABLE_ENABLE_Pos) &
                 I2S_ENABLE_ENABLE_Msk;

  ensureI2sIrqConnected(config_.irqPriority);

  nextBufferIndex_ = 1U;
  configured_ = true;
  running_ = false;
  restartPending_ = false;
  rxPtrUpdCount_ = 0U;
  stoppedCount_ = 0U;
  restartCount_ = 0U;
  manualStopCount_ = 0U;
  return true;
}

void I2sRx::end() {
  if (!configured_ || i2s_ == nullptr) {
    return;
  }

  i2s_->INTENCLR = I2S_INTENCLR_RXPTRUPD_Msk | I2S_INTENCLR_STOPPED_Msk;
  i2s_->TASKS_STOP = I2S_TASKS_STOP_TASKS_STOP_Trigger;
  waitForNonZero(&i2s_->EVENTS_STOPPED, 300000UL);
  i2s_->ENABLE = (I2S_ENABLE_ENABLE_Disabled << I2S_ENABLE_ENABLE_Pos) &
                 I2S_ENABLE_ENABLE_Msk;
  i2s_->PSEL.MCK = PSEL_DISCONNECTED;
  i2s_->PSEL.SCK = PSEL_DISCONNECTED;
  i2s_->PSEL.LRCK = PSEL_DISCONNECTED;
  i2s_->PSEL.SDIN = PSEL_DISCONNECTED;
  i2s_->PSEL.SDOUT = PSEL_DISCONNECTED;
  if (g_activeI2sRx == this) {
    g_activeI2sRx = nullptr;
  }
  configured_ = false;
  running_ = false;
  restartPending_ = false;
}

bool I2sRx::start() {
  if (!configured_ || i2s_ == nullptr) {
    return false;
  }

  clearEvents();
  nextBufferIndex_ = 1U;
  armBuffer(0U);
  i2s_->INTENSET = I2S_INTENSET_RXPTRUPD_Msk | I2S_INTENSET_STOPPED_Msk;
  running_ = true;
  restartPending_ = false;
  i2s_->TASKS_START = I2S_TASKS_START_TASKS_START_Trigger;
  return true;
}

bool I2sRx::stop() {
  if (!configured_ || !running_ || i2s_ == nullptr) {
    return false;
  }

  ++manualStopCount_;
  i2s_->TASKS_STOP = I2S_TASKS_STOP_TASKS_STOP_Trigger;
  return true;
}

void I2sRx::service() {
  if (restartPending_ && config_.autoRestart) {
    restartPending_ = false;
    ++restartCount_;
    (void)start();
  }
}

void I2sRx::onIrq() {
  if (!configured_ || i2s_ == nullptr) {
    return;
  }

  if (i2s_->EVENTS_RXPTRUPD != 0U) {
    i2s_->EVENTS_RXPTRUPD = 0U;
    const uint8_t queuedIndex = nextBufferIndex_;
    armBuffer(queuedIndex);
    nextBufferIndex_ ^= 1U;
    if (receiveCallback_ != nullptr) {
      receiveCallback_(buffers_[nextBufferIndex_], wordCount_, receiveContext_);
    }
    ++rxPtrUpdCount_;
  }

  if (i2s_->EVENTS_STOPPED != 0U) {
    i2s_->EVENTS_STOPPED = 0U;
    running_ = false;
    restartPending_ = true;
    ++stoppedCount_;
  }
}

bool I2sRx::makeActive() {
  if (!configured_) {
    return false;
  }
  g_activeI2sDuplex = nullptr;
  g_activeI2sTx = nullptr;
  g_i2sRawHandler = nullptr;
  g_activeI2sRx = this;
  return true;
}

void I2sRx::irqHandler() {
  if (g_activeI2sRx != nullptr) {
    g_activeI2sRx->onIrq();
  }
}

bool I2sRx::configured() const { return configured_; }
bool I2sRx::running() const { return running_; }
bool I2sRx::restartPending() const { return restartPending_; }
uint32_t I2sRx::rxPtrUpdCount() const { return rxPtrUpdCount_; }
uint32_t I2sRx::stoppedCount() const { return stoppedCount_; }
uint32_t I2sRx::restartCount() const { return restartCount_; }
uint32_t I2sRx::manualStopCount() const { return manualStopCount_; }

void I2sRx::clearEvents() {
  i2s_->EVENTS_RXPTRUPD = 0U;
  i2s_->EVENTS_STOPPED = 0U;
}

void I2sRx::armBuffer(uint8_t bufferIndex) {
  const uint8_t index = bufferIndex & 1U;
  i2s_->RXD.PTR =
      static_cast<uint32_t>(reinterpret_cast<uintptr_t>(buffers_[index])) &
      I2S_RXD_PTR_PTR_Msk;
}

I2sDuplex::I2sDuplex(uint32_t base)
    : i2s_(reinterpret_cast<NRF_I2S_Type*>(static_cast<uintptr_t>(base))),
      config_(),
      txBuffers_{nullptr, nullptr},
      rxBuffers_{nullptr, nullptr},
      wordCount_(0U),
      txRefillCallback_(nullptr),
      txRefillContext_(nullptr),
      rxReceiveCallback_(nullptr),
      rxReceiveContext_(nullptr),
      nextTxBufferIndex_(1U),
      nextRxBufferIndex_(1U),
      configured_(false),
      running_(false),
      restartPending_(false),
      txPtrUpdCount_(0U),
      rxPtrUpdCount_(0U),
      stoppedCount_(0U),
      restartCount_(0U),
      manualStopCount_(0U) {}

bool I2sDuplex::setTxBuffers(uint32_t* buffer0,
                             uint32_t* buffer1,
                             uint32_t wordCount) {
  if (buffer0 == nullptr || buffer1 == nullptr || wordCount == 0U) {
    return false;
  }
  if ((reinterpret_cast<uintptr_t>(buffer0) & 0x3U) != 0U ||
      (reinterpret_cast<uintptr_t>(buffer1) & 0x3U) != 0U) {
    return false;
  }
  if (wordCount >
      (I2S_RXTXD_MAXCNT_MAXCNT_Msk >> I2S_RXTXD_MAXCNT_MAXCNT_Pos)) {
    return false;
  }

  txBuffers_[0] = buffer0;
  txBuffers_[1] = buffer1;
  wordCount_ = wordCount;
  return true;
}

bool I2sDuplex::setRxBuffers(uint32_t* buffer0,
                             uint32_t* buffer1,
                             uint32_t wordCount) {
  if (buffer0 == nullptr || buffer1 == nullptr || wordCount == 0U) {
    return false;
  }
  if ((reinterpret_cast<uintptr_t>(buffer0) & 0x3U) != 0U ||
      (reinterpret_cast<uintptr_t>(buffer1) & 0x3U) != 0U) {
    return false;
  }
  if (wordCount >
      (I2S_RXTXD_MAXCNT_MAXCNT_Msk >> I2S_RXTXD_MAXCNT_MAXCNT_Pos)) {
    return false;
  }
  if (wordCount_ != 0U && wordCount_ != wordCount) {
    return false;
  }

  rxBuffers_[0] = buffer0;
  rxBuffers_[1] = buffer1;
  wordCount_ = wordCount;
  return true;
}

void I2sDuplex::setTxRefillCallback(TxRefillCallback callback, void* context) {
  txRefillCallback_ = callback;
  txRefillContext_ = context;
}

void I2sDuplex::setRxReceiveCallback(RxReceiveCallback callback, void* context) {
  rxReceiveCallback_ = callback;
  rxReceiveContext_ = context;
}

bool I2sDuplex::begin(const I2sDuplexConfig& config,
                      uint32_t* txBuffer0,
                      uint32_t* txBuffer1,
                      uint32_t* rxBuffer0,
                      uint32_t* rxBuffer1,
                      uint32_t wordCount) {
  if (i2s_ == nullptr ||
      !setTxBuffers(txBuffer0, txBuffer1, wordCount) ||
      !setRxBuffers(rxBuffer0, rxBuffer1, wordCount)) {
    return false;
  }
  if (!isConnected(config.mck) || !isConnected(config.sck) ||
      !isConnected(config.lrck) || !isConnected(config.sdin) ||
      !isConnected(config.sdout)) {
    return false;
  }
  if (!Gpio::configure(config.mck, GpioDirection::kOutput, GpioPull::kDisabled) ||
      !Gpio::configure(config.sck, GpioDirection::kOutput, GpioPull::kDisabled) ||
      !Gpio::configure(config.lrck, GpioDirection::kOutput, GpioPull::kDisabled) ||
      !Gpio::configure(config.sdin, GpioDirection::kInput, GpioPull::kDisabled) ||
      !Gpio::configure(config.sdout, GpioDirection::kOutput, GpioPull::kDisabled)) {
    return false;
  }

  config_ = config;

  i2s_->TASKS_STOP = I2S_TASKS_STOP_TASKS_STOP_Trigger;
  i2s_->ENABLE = (I2S_ENABLE_ENABLE_Disabled << I2S_ENABLE_ENABLE_Pos) &
                 I2S_ENABLE_ENABLE_Msk;

  i2s_->PSEL.MCK = make_psel(config_.mck.port, config_.mck.pin);
  i2s_->PSEL.SCK = make_psel(config_.sck.port, config_.sck.pin);
  i2s_->PSEL.LRCK = make_psel(config_.lrck.port, config_.lrck.pin);
  i2s_->PSEL.SDIN = make_psel(config_.sdin.port, config_.sdin.pin);
  i2s_->PSEL.SDOUT = make_psel(config_.sdout.port, config_.sdout.pin);

  i2s_->CONFIG.MODE = (I2S_CONFIG_MODE_MODE_Master << I2S_CONFIG_MODE_MODE_Pos) &
                      I2S_CONFIG_MODE_MODE_Msk;
  i2s_->CONFIG.RXEN =
      (I2S_CONFIG_RXEN_RXEN_Enabled << I2S_CONFIG_RXEN_RXEN_Pos) &
      I2S_CONFIG_RXEN_RXEN_Msk;
  i2s_->CONFIG.TXEN =
      (I2S_CONFIG_TXEN_TXEN_Enabled << I2S_CONFIG_TXEN_TXEN_Pos) &
      I2S_CONFIG_TXEN_TXEN_Msk;
  i2s_->CONFIG.MCKEN =
      ((config_.enableMasterClock ? I2S_CONFIG_MCKEN_MCKEN_Enabled
                                  : I2S_CONFIG_MCKEN_MCKEN_Disabled)
       << I2S_CONFIG_MCKEN_MCKEN_Pos) &
      I2S_CONFIG_MCKEN_MCKEN_Msk;
  i2s_->CONFIG.MCKFREQ = config_.mckFreq;
  i2s_->CONFIG.RATIO =
      (config_.ratio << I2S_CONFIG_RATIO_RATIO_Pos) & I2S_CONFIG_RATIO_RATIO_Msk;
  i2s_->CONFIG.SWIDTH =
      (config_.sampleWidth << I2S_CONFIG_SWIDTH_SWIDTH_Pos) &
      I2S_CONFIG_SWIDTH_SWIDTH_Msk;
  i2s_->CONFIG.ALIGN =
      (config_.align << I2S_CONFIG_ALIGN_ALIGN_Pos) & I2S_CONFIG_ALIGN_ALIGN_Msk;
  i2s_->CONFIG.FORMAT =
      (config_.format << I2S_CONFIG_FORMAT_FORMAT_Pos) & I2S_CONFIG_FORMAT_FORMAT_Msk;
  i2s_->CONFIG.CHANNELS =
      (config_.channels << I2S_CONFIG_CHANNELS_CHANNELS_Pos) &
      I2S_CONFIG_CHANNELS_CHANNELS_Msk;

  i2s_->RXTXD.MAXCNT = (wordCount_ << I2S_RXTXD_MAXCNT_MAXCNT_Pos) &
                       I2S_RXTXD_MAXCNT_MAXCNT_Msk;
  i2s_->INTENCLR = I2S_INTENCLR_TXPTRUPD_Msk | I2S_INTENCLR_RXPTRUPD_Msk |
                   I2S_INTENCLR_STOPPED_Msk;
  clearEvents();
  armTxBuffer(0U);
  armRxBuffer(0U);

  i2s_->ENABLE = (I2S_ENABLE_ENABLE_Enabled << I2S_ENABLE_ENABLE_Pos) &
                 I2S_ENABLE_ENABLE_Msk;

  ensureI2sIrqConnected(config_.irqPriority);

  nextTxBufferIndex_ = 1U;
  nextRxBufferIndex_ = 1U;
  configured_ = true;
  running_ = false;
  restartPending_ = false;
  txPtrUpdCount_ = 0U;
  rxPtrUpdCount_ = 0U;
  stoppedCount_ = 0U;
  restartCount_ = 0U;
  manualStopCount_ = 0U;
  return true;
}

void I2sDuplex::end() {
  if (!configured_ || i2s_ == nullptr) {
    return;
  }

  i2s_->INTENCLR = I2S_INTENCLR_TXPTRUPD_Msk | I2S_INTENCLR_RXPTRUPD_Msk |
                   I2S_INTENCLR_STOPPED_Msk;
  i2s_->TASKS_STOP = I2S_TASKS_STOP_TASKS_STOP_Trigger;
  waitForNonZero(&i2s_->EVENTS_STOPPED, 300000UL);
  i2s_->ENABLE = (I2S_ENABLE_ENABLE_Disabled << I2S_ENABLE_ENABLE_Pos) &
                 I2S_ENABLE_ENABLE_Msk;
  i2s_->PSEL.MCK = PSEL_DISCONNECTED;
  i2s_->PSEL.SCK = PSEL_DISCONNECTED;
  i2s_->PSEL.LRCK = PSEL_DISCONNECTED;
  i2s_->PSEL.SDIN = PSEL_DISCONNECTED;
  i2s_->PSEL.SDOUT = PSEL_DISCONNECTED;
  if (g_activeI2sDuplex == this) {
    g_activeI2sDuplex = nullptr;
  }
  configured_ = false;
  running_ = false;
  restartPending_ = false;
}

bool I2sDuplex::start() {
  if (!configured_ || i2s_ == nullptr) {
    return false;
  }

  clearEvents();
  nextTxBufferIndex_ = 1U;
  nextRxBufferIndex_ = 1U;
  armTxBuffer(0U);
  armRxBuffer(0U);
  i2s_->INTENSET = I2S_INTENSET_TXPTRUPD_Msk | I2S_INTENSET_RXPTRUPD_Msk |
                   I2S_INTENSET_STOPPED_Msk;
  running_ = true;
  restartPending_ = false;
  i2s_->TASKS_START = I2S_TASKS_START_TASKS_START_Trigger;
  return true;
}

bool I2sDuplex::stop() {
  if (!configured_ || !running_ || i2s_ == nullptr) {
    return false;
  }

  ++manualStopCount_;
  i2s_->TASKS_STOP = I2S_TASKS_STOP_TASKS_STOP_Trigger;
  return true;
}

void I2sDuplex::service() {
  if (restartPending_ && config_.autoRestart) {
    restartPending_ = false;
    ++restartCount_;
    (void)start();
  }
}

void I2sDuplex::onIrq() {
  if (!configured_ || i2s_ == nullptr) {
    return;
  }

  if (i2s_->EVENTS_TXPTRUPD != 0U) {
    i2s_->EVENTS_TXPTRUPD = 0U;
    const uint8_t queuedIndex = nextTxBufferIndex_;
    armTxBuffer(queuedIndex);
    nextTxBufferIndex_ ^= 1U;
    if (txRefillCallback_ != nullptr) {
      txRefillCallback_(txBuffers_[nextTxBufferIndex_], wordCount_,
                        txRefillContext_);
    }
    ++txPtrUpdCount_;
  }

  if (i2s_->EVENTS_RXPTRUPD != 0U) {
    i2s_->EVENTS_RXPTRUPD = 0U;
    const uint8_t queuedIndex = nextRxBufferIndex_;
    armRxBuffer(queuedIndex);
    nextRxBufferIndex_ ^= 1U;
    if (rxReceiveCallback_ != nullptr) {
      rxReceiveCallback_(rxBuffers_[nextRxBufferIndex_], wordCount_,
                         rxReceiveContext_);
    }
    ++rxPtrUpdCount_;
  }

  if (i2s_->EVENTS_STOPPED != 0U) {
    i2s_->EVENTS_STOPPED = 0U;
    running_ = false;
    restartPending_ = true;
    ++stoppedCount_;
  }
}

bool I2sDuplex::makeActive() {
  if (!configured_) {
    return false;
  }
  g_activeI2sTx = nullptr;
  g_activeI2sRx = nullptr;
  g_i2sRawHandler = nullptr;
  g_activeI2sDuplex = this;
  return true;
}

void I2sDuplex::irqHandler() {
  if (g_activeI2sDuplex != nullptr) {
    g_activeI2sDuplex->onIrq();
  }
}

bool I2sDuplex::configured() const { return configured_; }
bool I2sDuplex::running() const { return running_; }
bool I2sDuplex::restartPending() const { return restartPending_; }
uint32_t I2sDuplex::txPtrUpdCount() const { return txPtrUpdCount_; }
uint32_t I2sDuplex::rxPtrUpdCount() const { return rxPtrUpdCount_; }
uint32_t I2sDuplex::stoppedCount() const { return stoppedCount_; }
uint32_t I2sDuplex::restartCount() const { return restartCount_; }
uint32_t I2sDuplex::manualStopCount() const { return manualStopCount_; }

bool connectI2s20Interrupt(void (*handler)(), uint8_t priority) {
  if (handler == nullptr) {
    return false;
  }
  g_activeI2sTx = nullptr;
  g_activeI2sRx = nullptr;
  g_activeI2sDuplex = nullptr;
  g_i2sRawHandler = handler;
  ensureI2sIrqConnected(priority);
  return true;
}

void disconnectI2s20Interrupt() {
  g_i2sRawHandler = nullptr;
}

void I2sDuplex::clearEvents() {
  i2s_->EVENTS_TXPTRUPD = 0U;
  i2s_->EVENTS_RXPTRUPD = 0U;
  i2s_->EVENTS_STOPPED = 0U;
}

void I2sDuplex::armTxBuffer(uint8_t bufferIndex) {
  const uint8_t index = bufferIndex & 1U;
  i2s_->TXD.PTR =
      static_cast<uint32_t>(reinterpret_cast<uintptr_t>(txBuffers_[index])) &
      I2S_TXD_PTR_PTR_Msk;
}

void I2sDuplex::armRxBuffer(uint8_t bufferIndex) {
  const uint8_t index = bufferIndex & 1U;
  i2s_->RXD.PTR =
      static_cast<uint32_t>(reinterpret_cast<uintptr_t>(rxBuffers_[index])) &
      I2S_RXD_PTR_PTR_Msk;
}

CracenRng::CracenRng(uint32_t controlBase, uint32_t coreBase)
    : controlBase_(controlBase),
      coreBase_(coreBase),
      active_(false),
      status_(0U) {}

bool CracenRng::begin(uint32_t spinLimit) {
  (void)spinLimit;
  active_ = true;
  status_ = 0U;
  return true;
}

void CracenRng::end() {
  active_ = false;
}

bool CracenRng::fill(void* data, size_t length, uint32_t spinLimit) {
  (void)spinLimit;

  if (data == nullptr) {
    return false;
  }

  const bool temporaryBegin = !active_;
  if (temporaryBegin && !begin(spinLimit)) {
    return false;
  }

  sys_rand_get(data, length);
  status_ = 0U;

  if (temporaryBegin) {
    end();
  }
  return true;
}

bool CracenRng::randomWord(uint32_t* outWord, uint32_t spinLimit) {
  return fill(outWord, sizeof(*outWord), spinLimit);
}

uint32_t CracenRng::availableWords() const {
  return active_ ? 1U : 0U;
}

uint32_t CracenRng::status() const {
  return status_;
}

bool CracenRng::healthy() const {
  return status_ != 0xFFFFFFFFUL;
}

bool CracenRng::active() const {
  return active_;
}

void CracenRng::clearEvent() {
  status_ = 0U;
}

Aar::Aar(uint32_t base)
    : base_(base), errorStatus_(0U), resolvedAmountBytes_(0U) {}

bool Aar::resolveFirst(const uint8_t address[6],
                       const uint8_t* irks,
                       size_t irkCount,
                       bool* outResolved,
                       uint16_t* outIndex,
                       uint32_t spinLimit) {
  (void)spinLimit;

  if (outResolved == nullptr || address == nullptr || irks == nullptr ||
      irkCount == 0U) {
    errorStatus_ = 1U;
    resolvedAmountBytes_ = 0U;
    if (outResolved != nullptr) {
      *outResolved = false;
    }
    if (outIndex != nullptr) {
      *outIndex = 0xFFFFU;
    }
    return false;
  }

  *outResolved = false;
  if (outIndex != nullptr) {
    *outIndex = 0xFFFFU;
  }

  errorStatus_ = 0U;
  resolvedAmountBytes_ = 0U;
  uint8_t hash[3] = {0U};
  const uint8_t* expectedHash = &address[0];
  const uint8_t* prand = &address[3];
  for (size_t irkIndex = 0U; irkIndex < irkCount; ++irkIndex) {
    if (!computeBleAh(irks + (irkIndex * 16U), prand, hash)) {
      errorStatus_ = 1U;
      return false;
    }
    if (memcmp(hash, expectedHash, sizeof(hash)) == 0) {
      *outResolved = true;
      if (outIndex != nullptr) {
        *outIndex = static_cast<uint16_t>(irkIndex);
      }
      resolvedAmountBytes_ = sizeof(uint16_t);
      break;
    }
  }

  return true;
}

bool Aar::resolveSingle(const uint8_t address[6],
                        const uint8_t irk[16],
                        bool* outResolved,
                        uint32_t spinLimit) {
  return resolveFirst(address, irk, 1U, outResolved, nullptr, spinLimit);
}

uint32_t Aar::errorStatus() const {
  return errorStatus_;
}

uint32_t Aar::resolvedAmountBytes() const {
  return resolvedAmountBytes_;
}

void Aar::clearEvents() {
  errorStatus_ = 0U;
  resolvedAmountBytes_ = 0U;
}

Ecb::Ecb(uint32_t base) : base_(base), errorStatus_(0U) {}

bool Ecb::encryptBlock(const uint8_t key[16],
                       const uint8_t plaintext[16],
                       uint8_t ciphertext[16],
                       uint32_t spinLimit) {
  (void)spinLimit;

  if (key == nullptr || plaintext == nullptr || ciphertext == nullptr) {
    errorStatus_ = 1U;
    return false;
  }

  aesEncryptBlockBe(key, plaintext, ciphertext);
  errorStatus_ = 0U;
  return true;
}

bool Ecb::encryptBlockInPlace(const uint8_t key[16],
                              uint8_t block[16],
                              uint32_t spinLimit) {
  return encryptBlock(key, block, block, spinLimit);
}

uint32_t Ecb::errorStatus() const {
  return errorStatus_;
}

void Ecb::clearEvents() {
  errorStatus_ = 0U;
}

Ccm::Ccm(uint32_t base) : base_(base), errorStatus_(0U), macValid_(false) {}

bool Ccm::encryptBlePacket(const uint8_t key[16],
                           const uint8_t iv[8],
                           uint64_t counter,
                           uint8_t direction,
                           uint8_t header,
                           const uint8_t* plaintext,
                           uint8_t plaintextLen,
                           uint8_t* outCipherWithMic,
                           uint8_t* outCipherWithMicLen,
                           CcmBleDataRate dataRate,
                           uint8_t adataMask,
                           uint32_t spinLimit) {
  (void)dataRate;
  (void)spinLimit;

  macValid_ = false;
  if (outCipherWithMicLen != nullptr) {
    *outCipherWithMicLen = 0U;
  }
  if (key == nullptr || iv == nullptr || outCipherWithMic == nullptr ||
      outCipherWithMicLen == nullptr ||
      (plaintextLen > 0U && plaintext == nullptr) ||
      plaintextLen > kBleDataPduMaxPayload) {
    errorStatus_ = 1U;
    return false;
  }

  const bool ok =
      bleCcmEncryptPayload(key, iv, counter, direction, header, plaintext,
                           plaintextLen, adataMask, outCipherWithMic,
                           outCipherWithMicLen);
  errorStatus_ = ok ? 0U : 1U;
  macValid_ = ok;
  return ok;
}

bool Ccm::decryptBlePacket(const uint8_t key[16],
                           const uint8_t iv[8],
                           uint64_t counter,
                           uint8_t direction,
                           uint8_t header,
                           const uint8_t* cipherWithMic,
                           uint8_t cipherWithMicLen,
                           uint8_t* outPlaintext,
                           uint8_t* outPlaintextLen,
                           bool* outMacValid,
                           CcmBleDataRate dataRate,
                           uint8_t adataMask,
                           uint32_t spinLimit) {
  (void)dataRate;
  (void)spinLimit;

  if (outMacValid != nullptr) {
    *outMacValid = false;
  }
  if (outPlaintextLen != nullptr) {
    *outPlaintextLen = 0U;
  }
  if (key == nullptr || iv == nullptr || cipherWithMic == nullptr ||
      outPlaintext == nullptr || outPlaintextLen == nullptr ||
      cipherWithMicLen < kBleMicLen ||
      (cipherWithMicLen - kBleMicLen) > kBleDataPduMaxPayload) {
    errorStatus_ = 1U;
    return false;
  }

  macValid_ = bleCcmDecryptPayload(key, iv, counter, direction, header,
                                   cipherWithMic, cipherWithMicLen, adataMask,
                                   outPlaintext, outPlaintextLen);
  errorStatus_ = 0U;
  if (outMacValid != nullptr) {
    *outMacValid = macValid_;
  }
  return macValid_;
}

uint32_t Ccm::errorStatus() const {
  return errorStatus_;
}

bool Ccm::macStatus() const {
  return macValid_;
}

void Ccm::clearEvents() {
  errorStatus_ = 0U;
  macValid_ = false;
}

BleRadio::BleRadio(uint32_t radioBase, uint32_t ficrBase)
    : radioBase_(radioBase),
      ficrBase_(ficrBase),
      txPowerDbm_(-8),
      pduType_(BleAdvPduType::kAdvInd),
      addressType_(BleAddressType::kRandomStatic),
      includeAdvertisingFlags_(true),
      useChSel2_(false),
      addressConfigured_(false),
      advertisingIdentityDirty_(false),
      customAdvertisingIdentity_(false),
      batteryLevel_(100U),
      advertisingIdentityId_(BT_ID_DEFAULT),
      initialized_(false),
      address_{0},
      scanCycleStartIndex_(0U),
      passiveScanPayload_{0},
      advertisingData_{0},
      scanResponseData_{0},
      advertisingDataLen_(0U),
      scanResponseDataLen_(0U),
      advertisingName_{0},
      scanResponseName_{0},
      gattDeviceName_{0},
      batteryService_(nullptr),
      batteryLevelCharacteristic_(nullptr),
      batteryServiceAdded_(false),
      nextCustomGattHandle_(kCustomGattHandleStart),
      pendingCustomGattServiceIndex_(-1),
      customGattServiceCount_(0U),
      customGattCharacteristicCount_(0U),
      customGattWriteCallback_(nullptr),
      customGattWriteContext_(nullptr),
      customGattServices_{},
      customGattCharacteristics_{} {}

bool BleRadio::begin(int8_t txPowerDbm) {
  txPowerDbm_ = txPowerDbm;
  ensureBleLinkCallbacksRegistered();
  ensureBleSecurityCallbacksRegistered();
  if (!BLE.begin(effectiveLocalName())) {
    initialized_ = false;
    return false;
  }

  initialized_ = true;
  const char* localName = effectiveLocalName();
  if (localName != nullptr && localName[0] != '\0') {
    (void)BLE.setLocalName(localName);
  }
  return ensureBatteryService();
}

void BleRadio::end() {
  if (!initialized_) {
    return;
  }

  BLE.stopAdvertising();
  (void)bt_le_adv_stop();
  (void)clearCustomGatt();
  BLE.end();
  initialized_ = false;
  batteryServiceAdded_ = false;
}

bool BleRadio::setTxPowerDbm(int8_t dbm) {
  txPowerDbm_ = dbm;
  return true;
}

bool BleRadio::selectExternalAntenna(bool external) {
  return BoardControl::setAntennaPath(
      external ? BoardAntennaPath::kExternal : BoardAntennaPath::kCeramic);
}

bool BleRadio::loadAddressFromFicr(bool forceRandomStatic) {
  if (!initialized_ && !begin(txPowerDbm_)) {
    return false;
  }

  bt_addr_le_t address;
  size_t count = 1U;
  memset(&address, 0, sizeof(address));
  bt_id_get(&address, &count);
  if (count == 0U) {
    return false;
  }

  BleAddressType type = bleAddressTypeFromZephyr(address.type);
  if (forceRandomStatic) {
    type = BleAddressType::kRandomStatic;
  }
  return setDeviceAddress(address.a.val, type);
}

bool BleRadio::setDeviceAddress(const uint8_t address[6], BleAddressType type) {
  if (address == nullptr) {
    return false;
  }

  memcpy(address_, address, sizeof(address_));
  if (type == BleAddressType::kRandomStatic) {
    address_[5] = static_cast<uint8_t>((address_[5] & 0x3FU) | 0xC0U);
  }
  addressType_ = type;
  addressConfigured_ = true;
  advertisingIdentityDirty_ = true;
  g_bleSecurityState.localIdentityValid = false;
  clearCachedBondRecord();
  return buildAdvertisingPacket() && buildScanResponsePacket();
}

bool BleRadio::getDeviceAddress(uint8_t addressOut[6], BleAddressType* typeOut) const {
  if (addressOut == nullptr) {
    return false;
  }

  if (addressConfigured_) {
    memcpy(addressOut, address_, sizeof(address_));
    if (typeOut != nullptr) {
      *typeOut = addressType_;
    }
    return true;
  }

  if (!initialized_) {
    return false;
  }

  bt_addr_le_t address;
  size_t count = 1U;
  memset(&address, 0, sizeof(address));
  bt_id_get(&address, &count);
  if (count == 0U) {
    return false;
  }

  memcpy(addressOut, address.a.val, sizeof(address.a.val));
  if (typeOut != nullptr) {
    *typeOut = bleAddressTypeFromZephyr(address.type);
  }
  return true;
}

bool BleRadio::setAdvertisingPduType(BleAdvPduType type) {
  const uint8_t raw = static_cast<uint8_t>(type);
  if (raw > 0x0FU) {
    return false;
  }
  pduType_ = type;
  return buildAdvertisingPacket();
}

bool BleRadio::setAdvertisingChannelSelectionAlgorithm2(bool enabled) {
  useChSel2_ = enabled;
  return buildAdvertisingPacket();
}

bool BleRadio::setAdvertisingData(const uint8_t* data, size_t len) {
  if (len > sizeof(advertisingData_)) {
    return false;
  }
  if (len > 0U && data == nullptr) {
    return false;
  }

  memset(advertisingData_, 0, sizeof(advertisingData_));
  advertisingName_[0] = '\0';
  if (len > 0U) {
    memcpy(advertisingData_, data, len);
    extractBleLocalName(advertisingData_, len, advertisingName_,
                        sizeof(advertisingName_));
  }
  advertisingDataLen_ = len;
  return buildAdvertisingPacket();
}

bool BleRadio::setAdvertisingName(const char* name, bool includeFlags) {
  if (name == nullptr) {
    return false;
  }

  includeAdvertisingFlags_ = includeFlags;
  safeCopyString(name, advertisingName_, sizeof(advertisingName_));
  uint8_t payload[kBleMaxSerializedPayloadLen] = {0};
  size_t used = 0U;

  if (includeFlags) {
    payload[used++] = 2U;
    payload[used++] = BT_DATA_FLAGS;
    payload[used++] = BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR;
  }

  const size_t nameLen = strlen(name);
  if (used >= sizeof(payload) || (sizeof(payload) - used) < 2U) {
    return false;
  }

  size_t copyLen = nameLen;
  uint8_t adType = BT_DATA_NAME_COMPLETE;
  if ((copyLen + 2U) > (sizeof(payload) - used)) {
    copyLen = sizeof(payload) - used - 2U;
    adType = BT_DATA_NAME_SHORTENED;
  }

  payload[used++] = static_cast<uint8_t>(copyLen + 1U);
  payload[used++] = adType;
  if (copyLen > 0U) {
    memcpy(&payload[used], name, copyLen);
    used += copyLen;
  }

  const bool ok = setAdvertisingData(payload, used);
  if (ok && gattDeviceName_[0] == '\0') {
    (void)setGattDeviceName(name);
  }
  if (initialized_ && advertisingName_[0] != '\0') {
    return ok && BLE.setLocalName(advertisingName_);
  }
  return ok;
}

bool BleRadio::buildAdvertisingPacket() {
  if (advertisingDataLen_ > sizeof(advertisingData_)) {
    return false;
  }
  if (advertisingDataLen_ > 0U) {
    bt_data entries[kBleMaxAdStructures];
    size_t entryCount = 0U;
    if (!parseBleAdvertisingPayload(advertisingData_, advertisingDataLen_, entries,
                                    kBleMaxAdStructures, &entryCount)) {
      return false;
    }
  }
  return true;
}

bool BleRadio::setScanResponseName(const char* name) {
  if (name == nullptr) {
    return false;
  }

  safeCopyString(name, scanResponseName_, sizeof(scanResponseName_));
  uint8_t payload[kBleMaxSerializedPayloadLen] = {0};
  const size_t nameLen = strlen(name);
  size_t used = 0U;
  size_t copyLen = nameLen;
  uint8_t adType = BT_DATA_NAME_COMPLETE;
  if ((copyLen + 2U) > sizeof(payload)) {
    copyLen = sizeof(payload) - 2U;
    adType = BT_DATA_NAME_SHORTENED;
  }

  payload[used++] = static_cast<uint8_t>(copyLen + 1U);
  payload[used++] = adType;
  if (copyLen > 0U) {
    memcpy(&payload[used], name, copyLen);
    used += copyLen;
  }

  return setScanResponseData(payload, used);
}

bool BleRadio::setScanResponseData(const uint8_t* data, size_t len) {
  if (len > sizeof(scanResponseData_)) {
    return false;
  }
  if (len > 0U && data == nullptr) {
    return false;
  }

  memset(scanResponseData_, 0, sizeof(scanResponseData_));
  scanResponseName_[0] = '\0';
  if (len > 0U) {
    memcpy(scanResponseData_, data, len);
    extractBleLocalName(scanResponseData_, len, scanResponseName_,
                        sizeof(scanResponseName_));
  }
  scanResponseDataLen_ = len;
  return buildScanResponsePacket();
}

bool BleRadio::buildScanResponsePacket() {
  if (scanResponseDataLen_ > sizeof(scanResponseData_)) {
    return false;
  }
  if (scanResponseDataLen_ > 0U) {
    bt_data entries[kBleMaxAdStructures];
    size_t entryCount = 0U;
    if (!parseBleAdvertisingPayload(scanResponseData_, scanResponseDataLen_, entries,
                                    kBleMaxAdStructures, &entryCount)) {
      return false;
    }
  }
  return true;
}

void BleRadio::onCustomGattCharacteristicWritten(BLECharacteristic& characteristic) {
  auto* characteristicState = static_cast<BleCustomCharacteristicState*>(
      characteristic._userContext);
  if (characteristicState == nullptr || characteristicState->owner == nullptr) {
    return;
  }
  characteristicState->owner->handleCustomGattWrite(
      characteristicState, characteristic._lastWriteWithResponse);
}

bool BleRadio::clearCustomGatt() {
  for (uint8_t i = 0U; i < customGattServiceCount_; ++i) {
    BleCustomServiceState& serviceState = customGattServices_[i];
    if (serviceState.registered && serviceState.service != nullptr) {
      if (!BLE.removeService(*serviceState.service)) {
        return false;
      }
    }
    delete serviceState.service;
    serviceState.service = nullptr;
    memset(serviceState.uuidString, 0, sizeof(serviceState.uuidString));
  }

  for (uint8_t i = 0U; i < customGattCharacteristicCount_; ++i) {
    BleCustomCharacteristicState& characteristicState = customGattCharacteristics_[i];
    delete characteristicState.characteristic;
    characteristicState.characteristic = nullptr;
    characteristicState.owner = nullptr;
    characteristicState.writeHandler = nullptr;
    characteristicState.writeContext = nullptr;
    characteristicState.valueLength = 0U;
    memset(characteristicState.value, 0, sizeof(characteristicState.value));
    memset(characteristicState.uuidString, 0, sizeof(characteristicState.uuidString));
  }

  customGattServiceCount_ = 0U;
  customGattCharacteristicCount_ = 0U;
  pendingCustomGattServiceIndex_ = -1;
  nextCustomGattHandle_ = kCustomGattHandleStart;
  customGattWriteCallback_ = nullptr;
  customGattWriteContext_ = nullptr;
  clearBlePendingRx();
  clearBlePendingTx();
  return true;
}

bool BleRadio::addCustomGattService(uint16_t uuid16, uint16_t* outServiceHandle) {
  if (!initialized_ && !begin(txPowerDbm_)) {
    return false;
  }
  if (customGattServiceCount_ >= kCustomGattMaxServices ||
      nextCustomGattHandle_ > kCustomGattHandleEnd) {
    return false;
  }
  if (!finalizePendingCustomGattService()) {
    return false;
  }

  BleCustomServiceState& serviceState = customGattServices_[customGattServiceCount_];
  memset(&serviceState, 0, sizeof(serviceState));
  if (!formatBleUuid16String(uuid16, serviceState.uuidString,
                             sizeof(serviceState.uuidString))) {
    return false;
  }

  serviceState.serviceHandle = nextCustomGattHandle_++;
  serviceState.endHandle = serviceState.serviceHandle;
  serviceState.firstCharacteristicIndex = customGattCharacteristicCount_;
  serviceState.characteristicCount = 0U;
  serviceState.service = new BLEService(serviceState.uuidString);
  if (serviceState.service == nullptr) {
    return false;
  }

  pendingCustomGattServiceIndex_ = static_cast<int8_t>(customGattServiceCount_);
  ++customGattServiceCount_;
  if (outServiceHandle != nullptr) {
    *outServiceHandle = serviceState.serviceHandle;
  }
  return true;
}

bool BleRadio::addCustomGattService128(
    const uint8_t uuid128[kCustomGattUuid128Length], uint16_t* outServiceHandle) {
  if (!initialized_ && !begin(txPowerDbm_)) {
    return false;
  }
  if (customGattServiceCount_ >= kCustomGattMaxServices ||
      nextCustomGattHandle_ > kCustomGattHandleEnd) {
    return false;
  }
  if (!finalizePendingCustomGattService()) {
    return false;
  }

  BleCustomServiceState& serviceState = customGattServices_[customGattServiceCount_];
  memset(&serviceState, 0, sizeof(serviceState));
  if (!formatBleUuid128String(uuid128, serviceState.uuidString,
                              sizeof(serviceState.uuidString))) {
    return false;
  }

  serviceState.serviceHandle = nextCustomGattHandle_++;
  serviceState.endHandle = serviceState.serviceHandle;
  serviceState.firstCharacteristicIndex = customGattCharacteristicCount_;
  serviceState.characteristicCount = 0U;
  serviceState.service = new BLEService(serviceState.uuidString);
  if (serviceState.service == nullptr) {
    return false;
  }

  pendingCustomGattServiceIndex_ = static_cast<int8_t>(customGattServiceCount_);
  ++customGattServiceCount_;
  if (outServiceHandle != nullptr) {
    *outServiceHandle = serviceState.serviceHandle;
  }
  return true;
}

bool BleRadio::addCustomGattCharacteristic(uint16_t serviceHandle, uint16_t uuid16,
                                           uint8_t properties,
                                           const uint8_t* initialValue,
                                           uint8_t initialValueLength,
                                           uint16_t* outValueHandle,
                                           uint16_t* outCccdHandle) {
  if (!initialized_ && !begin(txPowerDbm_)) {
    return false;
  }
  if (pendingCustomGattServiceIndex_ < 0 ||
      customGattCharacteristicCount_ >= kCustomGattMaxCharacteristics ||
      initialValueLength > kCustomGattMaxValueLength ||
      (initialValueLength > 0U && initialValue == nullptr)) {
    return false;
  }

  BleCustomServiceState& serviceState =
      customGattServices_[static_cast<uint8_t>(pendingCustomGattServiceIndex_)];
  if (serviceState.serviceHandle != serviceHandle || serviceState.service == nullptr) {
    return false;
  }

  const bool hasCccd =
      ((properties & kBleGattPropNotify) != 0U) ||
      ((properties & kBleGattPropIndicate) != 0U);
  const uint16_t handlesNeeded = hasCccd ? 3U : 2U;
  if (nextCustomGattHandle_ > (kCustomGattHandleEnd + 1U - handlesNeeded)) {
    return false;
  }

  BleCustomCharacteristicState& characteristicState =
      customGattCharacteristics_[customGattCharacteristicCount_];
  memset(&characteristicState, 0, sizeof(characteristicState));
  if (!formatBleUuid16String(uuid16, characteristicState.uuidString,
                             sizeof(characteristicState.uuidString))) {
    return false;
  }

  characteristicState.owner = this;
  characteristicState.serviceHandle = serviceHandle;
  characteristicState.properties = properties;
  characteristicState.declarationHandle = nextCustomGattHandle_++;
  characteristicState.valueHandle = nextCustomGattHandle_++;
  characteristicState.cccdHandle = hasCccd ? nextCustomGattHandle_++ : 0U;
  characteristicState.characteristic =
      new BLECharacteristic(characteristicState.uuidString,
                            bleGattPropertiesToArduino(properties),
                            kCustomGattMaxValueLength);
  if (characteristicState.characteristic == nullptr) {
    return false;
  }

  characteristicState.characteristic->_userContext = &characteristicState;
  characteristicState.characteristic->setEventHandler(
      BleRadio::onCustomGattCharacteristicWritten);

  if (initialValueLength > 0U) {
    memcpy(characteristicState.value, initialValue, initialValueLength);
    characteristicState.valueLength = initialValueLength;
    memcpy(characteristicState.characteristic->_value, initialValue, initialValueLength);
    characteristicState.characteristic->_valueLength = initialValueLength;
  }

  serviceState.service->addCharacteristic(*characteristicState.characteristic);
  ++serviceState.characteristicCount;
  serviceState.endHandle =
      (characteristicState.cccdHandle != 0U) ? characteristicState.cccdHandle
                                             : characteristicState.valueHandle;

  ++customGattCharacteristicCount_;
  if (outValueHandle != nullptr) {
    *outValueHandle = characteristicState.valueHandle;
  }
  if (outCccdHandle != nullptr) {
    *outCccdHandle = characteristicState.cccdHandle;
  }
  return true;
}

bool BleRadio::addCustomGattCharacteristic128(
    uint16_t serviceHandle, const uint8_t uuid128[kCustomGattUuid128Length],
    uint8_t properties, const uint8_t* initialValue, uint8_t initialValueLength,
    uint16_t* outValueHandle, uint16_t* outCccdHandle) {
  if (!initialized_ && !begin(txPowerDbm_)) {
    return false;
  }
  if (pendingCustomGattServiceIndex_ < 0 ||
      customGattCharacteristicCount_ >= kCustomGattMaxCharacteristics ||
      initialValueLength > kCustomGattMaxValueLength ||
      (initialValueLength > 0U && initialValue == nullptr)) {
    return false;
  }

  BleCustomServiceState& serviceState =
      customGattServices_[static_cast<uint8_t>(pendingCustomGattServiceIndex_)];
  if (serviceState.serviceHandle != serviceHandle || serviceState.service == nullptr) {
    return false;
  }

  const bool hasCccd =
      ((properties & kBleGattPropNotify) != 0U) ||
      ((properties & kBleGattPropIndicate) != 0U);
  const uint16_t handlesNeeded = hasCccd ? 3U : 2U;
  if (nextCustomGattHandle_ > (kCustomGattHandleEnd + 1U - handlesNeeded)) {
    return false;
  }

  BleCustomCharacteristicState& characteristicState =
      customGattCharacteristics_[customGattCharacteristicCount_];
  memset(&characteristicState, 0, sizeof(characteristicState));
  if (!formatBleUuid128String(uuid128, characteristicState.uuidString,
                              sizeof(characteristicState.uuidString))) {
    return false;
  }

  characteristicState.owner = this;
  characteristicState.serviceHandle = serviceHandle;
  characteristicState.properties = properties;
  characteristicState.declarationHandle = nextCustomGattHandle_++;
  characteristicState.valueHandle = nextCustomGattHandle_++;
  characteristicState.cccdHandle = hasCccd ? nextCustomGattHandle_++ : 0U;
  characteristicState.characteristic =
      new BLECharacteristic(characteristicState.uuidString,
                            bleGattPropertiesToArduino(properties),
                            kCustomGattMaxValueLength);
  if (characteristicState.characteristic == nullptr) {
    return false;
  }

  characteristicState.characteristic->_userContext = &characteristicState;
  characteristicState.characteristic->setEventHandler(
      BleRadio::onCustomGattCharacteristicWritten);

  if (initialValueLength > 0U) {
    memcpy(characteristicState.value, initialValue, initialValueLength);
    characteristicState.valueLength = initialValueLength;
    memcpy(characteristicState.characteristic->_value, initialValue, initialValueLength);
    characteristicState.characteristic->_valueLength = initialValueLength;
  }

  serviceState.service->addCharacteristic(*characteristicState.characteristic);
  ++serviceState.characteristicCount;
  serviceState.endHandle =
      (characteristicState.cccdHandle != 0U) ? characteristicState.cccdHandle
                                             : characteristicState.valueHandle;

  ++customGattCharacteristicCount_;
  if (outValueHandle != nullptr) {
    *outValueHandle = characteristicState.valueHandle;
  }
  if (outCccdHandle != nullptr) {
    *outCccdHandle = characteristicState.cccdHandle;
  }
  return true;
}

bool BleRadio::setCustomGattCharacteristicValue(uint16_t valueHandle,
                                                const uint8_t* value,
                                                uint8_t valueLength) {
  if (valueLength > kCustomGattMaxValueLength ||
      (valueLength > 0U && value == nullptr)) {
    return false;
  }

  BleCustomCharacteristicState* characteristicState =
      findCustomGattCharacteristic(valueHandle);
  if (characteristicState == nullptr || characteristicState->characteristic == nullptr) {
    return false;
  }

  memset(characteristicState->value, 0, sizeof(characteristicState->value));
  characteristicState->valueLength = valueLength;
  if (valueLength > 0U) {
    memcpy(characteristicState->value, value, valueLength);
  }
  memset(characteristicState->characteristic->_value, 0,
         characteristicState->characteristic->_valueSize);
  if (valueLength > 0U) {
    memcpy(characteristicState->characteristic->_value, value, valueLength);
  }
  characteristicState->characteristic->_valueLength = valueLength;
  return true;
}

bool BleRadio::getCustomGattCharacteristicValue(uint16_t valueHandle,
                                                uint8_t* outValue,
                                                uint8_t* inOutValueLength) const {
  if (outValue == nullptr || inOutValueLength == nullptr) {
    return false;
  }

  const BleCustomCharacteristicState* characteristicState =
      findCustomGattCharacteristic(valueHandle);
  if (characteristicState == nullptr) {
    return false;
  }
  if (*inOutValueLength < characteristicState->valueLength) {
    *inOutValueLength = characteristicState->valueLength;
    return false;
  }

  if (characteristicState->valueLength > 0U) {
    memcpy(outValue, characteristicState->value, characteristicState->valueLength);
  }
  *inOutValueLength = characteristicState->valueLength;
  return true;
}

bool BleRadio::notifyCustomGattCharacteristic(uint16_t valueHandle, bool indicate) {
  if (indicate) {
    return false;
  }
  if (!ensureCustomGattRegistered() || !g_bleLinkState.connected) {
    return false;
  }

  BleCustomCharacteristicState* characteristicState =
      findCustomGattCharacteristic(valueHandle);
  if (characteristicState == nullptr || characteristicState->characteristic == nullptr ||
      (characteristicState->properties & kBleGattPropNotify) == 0U ||
      !isCustomGattCccdEnabled(valueHandle, false)) {
    return false;
  }

  const bt_gatt_attr* attr = static_cast<const bt_gatt_attr*>(
      characteristicState->characteristic->_zephyr_attr);
  if (attr == nullptr) {
    return false;
  }

  const int err = bt_gatt_notify(g_bleLinkState.conn, attr, characteristicState->value,
                                 characteristicState->valueLength);
  if (err != 0) {
    return false;
  }

  clearBlePendingTx();
  return queueBleAttPayload(g_bleLinkState.pendingTxPayload,
                            &g_bleLinkState.pendingTxPayloadLen,
                            &g_bleLinkState.pendingTxAttOpcode, 0x1BU, valueHandle,
                            characteristicState->value,
                            characteristicState->valueLength);
}

bool BleRadio::isCustomGattCccdEnabled(uint16_t valueHandle, bool indication) const {
  const BleCustomCharacteristicState* characteristicState =
      findCustomGattCharacteristic(valueHandle);
  if (characteristicState == nullptr || characteristicState->characteristic == nullptr ||
      characteristicState->cccdHandle == 0U || !g_bleLinkState.connected) {
    return false;
  }

  const bt_gatt_attr* attr = static_cast<const bt_gatt_attr*>(
      characteristicState->characteristic->_zephyr_attr);
  if (attr == nullptr) {
    return false;
  }

  const uint16_t subscribeType =
      indication ? BT_GATT_CCC_INDICATE : BT_GATT_CCC_NOTIFY;
  return bt_gatt_is_subscribed(g_bleLinkState.conn, attr, subscribeType);
}

bool BleRadio::setCustomGattWriteHandler(uint16_t valueHandle,
                                         BleGattWriteCallback callback,
                                         void* context) {
  BleCustomCharacteristicState* characteristicState =
      findCustomGattCharacteristic(valueHandle);
  if (characteristicState == nullptr) {
    return false;
  }
  characteristicState->writeHandler = callback;
  characteristicState->writeContext = context;
  return true;
}

void BleRadio::setCustomGattWriteCallback(BleGattWriteCallback callback,
                                          void* context) {
  customGattWriteCallback_ = callback;
  customGattWriteContext_ = context;
}

bool BleRadio::ensureCustomGattRegistered() {
  if (!initialized_ && !begin(txPowerDbm_)) {
    return false;
  }
  return finalizePendingCustomGattService();
}

bool BleRadio::finalizePendingCustomGattService() {
  if (pendingCustomGattServiceIndex_ < 0) {
    return true;
  }

  BleCustomServiceState& serviceState =
      customGattServices_[static_cast<uint8_t>(pendingCustomGattServiceIndex_)];
  if (serviceState.registered) {
    pendingCustomGattServiceIndex_ = -1;
    return true;
  }
  if (serviceState.service == nullptr) {
    return false;
  }

  BLE.addService(*serviceState.service);
  if (serviceState.service->_zephyr_svc == nullptr ||
      serviceState.service->_registerError != 0) {
    return false;
  }

  serviceState.registered = true;
  pendingCustomGattServiceIndex_ = -1;
  return true;
}

BleRadio::BleCustomServiceState* BleRadio::findCustomGattService(
    uint16_t serviceHandle) {
  for (uint8_t i = 0U; i < customGattServiceCount_; ++i) {
    if (customGattServices_[i].serviceHandle == serviceHandle) {
      return &customGattServices_[i];
    }
  }
  return nullptr;
}

const BleRadio::BleCustomServiceState* BleRadio::findCustomGattService(
    uint16_t serviceHandle) const {
  for (uint8_t i = 0U; i < customGattServiceCount_; ++i) {
    if (customGattServices_[i].serviceHandle == serviceHandle) {
      return &customGattServices_[i];
    }
  }
  return nullptr;
}

BleRadio::BleCustomCharacteristicState* BleRadio::findCustomGattCharacteristic(
    uint16_t valueHandle) {
  for (uint8_t i = 0U; i < customGattCharacteristicCount_; ++i) {
    if (customGattCharacteristics_[i].valueHandle == valueHandle) {
      return &customGattCharacteristics_[i];
    }
  }
  return nullptr;
}

const BleRadio::BleCustomCharacteristicState* BleRadio::findCustomGattCharacteristic(
    uint16_t valueHandle) const {
  for (uint8_t i = 0U; i < customGattCharacteristicCount_; ++i) {
    if (customGattCharacteristics_[i].valueHandle == valueHandle) {
      return &customGattCharacteristics_[i];
    }
  }
  return nullptr;
}

void BleRadio::handleCustomGattWrite(
    BleCustomCharacteristicState* characteristicState, bool withResponse) {
  if (characteristicState == nullptr || characteristicState->characteristic == nullptr) {
    return;
  }

  BLECharacteristic* characteristic = characteristicState->characteristic;
  const uint16_t writeOffset = characteristic->_lastWriteOffset;
  const uint16_t writeLength = characteristic->_lastWriteLength;
  const uint16_t valueLength = characteristic->_valueLength;
  if (valueLength > 0U) {
    const uint8_t copyLength =
        (valueLength > kCustomGattMaxValueLength) ? kCustomGattMaxValueLength
                                                  : static_cast<uint8_t>(valueLength);
    memset(characteristicState->value, 0, sizeof(characteristicState->value));
    memcpy(characteristicState->value, characteristic->_value, copyLength);
    characteristicState->valueLength = copyLength;
  } else {
    characteristicState->valueLength = 0U;
    memset(characteristicState->value, 0, sizeof(characteristicState->value));
  }

  const uint8_t* writeValue = nullptr;
  uint8_t writeValueLength = 0U;
  if (writeOffset < characteristic->_valueSize &&
      writeLength <= (characteristic->_valueSize - writeOffset)) {
    writeValue = &characteristic->_value[writeOffset];
    writeValueLength = static_cast<uint8_t>(
        (writeLength > kCustomGattMaxValueLength) ? kCustomGattMaxValueLength
                                                  : writeLength);
  }

  clearBlePendingRx();
  (void)queueBleAttPayload(g_bleLinkState.pendingRxPayload,
                           &g_bleLinkState.pendingRxPayloadLen,
                           &g_bleLinkState.pendingRxAttOpcode,
                           withResponse ? 0x12U : 0x52U,
                           characteristicState->valueHandle, writeValue,
                           writeValueLength);

  if (characteristicState->writeHandler != nullptr) {
    characteristicState->writeHandler(characteristicState->valueHandle, writeValue,
                                      writeValueLength, withResponse,
                                      characteristicState->writeContext);
  }
  if (customGattWriteCallback_ != nullptr) {
    customGattWriteCallback_(characteristicState->valueHandle, writeValue,
                             writeValueLength, withResponse,
                             customGattWriteContext_);
  }
}

bool BleRadio::setGattDeviceName(const char* name) {
  safeCopyString(name, gattDeviceName_, sizeof(gattDeviceName_));
  if (initialized_ && advertisingName_[0] == '\0' && gattDeviceName_[0] != '\0') {
    return BLE.setLocalName(gattDeviceName_);
  }
  return true;
}

bool BleRadio::setGattBatteryLevel(uint8_t percent) {
  batteryLevel_ = percent;
  if (!initialized_) {
    return true;
  }
  if (!ensureBatteryService()) {
    return false;
  }

#if defined(CONFIG_BT_BAS)
  return true;
#else
  auto* characteristic =
      static_cast<BLECharacteristic*>(batteryLevelCharacteristic_);
  if (characteristic == nullptr) {
    return false;
  }
  const uint8_t value = batteryLevel_;
  const bool ok = characteristic->writeValue(&value, sizeof(value));
  if (ok && g_bleLinkState.conn != nullptr &&
      bt_gatt_is_subscribed(g_bleLinkState.conn,
                            static_cast<const bt_gatt_attr*>(characteristic->_zephyr_attr),
                            BT_GATT_CCC_NOTIFY)) {
    (void)queueBleAttNotification(
        static_cast<const bt_gatt_attr*>(characteristic->_zephyr_attr), &value,
        sizeof(value));
  }
  return ok;
#endif
}

bool BleRadio::isConnected() const {
  servicePendingBleSecurityActions();
  return bleVisibleConnectionActive();
}

bool BleRadio::isConnectionEncrypted() const {
  servicePendingBleSecurityActions();
  return g_bleLinkState.connected && g_bleLinkState.encrypted;
}

bool BleRadio::getConnectionInfo(BleConnectionInfo* info) const {
  servicePendingBleSecurityActions();
  if (info == nullptr || !bleVisibleConnectionActive()) {
    return false;
  }

  memset(info, 0, sizeof(*info));
  memcpy(info->peerAddress, g_bleLinkState.peerAddress.a.val, sizeof(info->peerAddress));
  info->peerAddressRandom =
      (g_bleLinkState.peerAddress.type == BT_ADDR_LE_RANDOM);
  info->accessAddress = 0U;
  info->crcInit = 0U;
  info->intervalUnits =
      (g_bleLinkState.intervalUnits != 0U) ? g_bleLinkState.intervalUnits : 24U;
  info->latency = g_bleLinkState.latency;
  info->supervisionTimeoutUnits = g_bleLinkState.supervisionTimeoutUnits;
  memcpy(info->channelMap, kBleDefaultChannelMap, sizeof(info->channelMap));
  info->channelCount = 37U;
  info->hopIncrement = 5U;
  info->sleepClockAccuracy = 0U;
  return true;
}

void BleRadio::getEncryptionDebugCounters(BleEncryptionDebugCounters* out) const {
  if (out == nullptr) {
    return;
  }

  *out = g_bleSecurityState.encDebug;
}

void BleRadio::clearEncryptionDebugCounters() {
  memset(&g_bleSecurityState.encDebug, 0, sizeof(g_bleSecurityState.encDebug));
}

bool BleRadio::hasBondRecord() const {
  if (g_bleSecurityState.bondRecordValid) {
    return true;
  }

  BleBondRecord record{};
  return getBondRecord(&record);
}

bool BleRadio::getBondRecord(BleBondRecord* outRecord) const {
  servicePendingBleSecurityActions();
  if (outRecord == nullptr) {
    return false;
  }

  uint8_t identityId = BT_ID_DEFAULT;
  const bool haveIdentity = resolveLocalIdentityId(&identityId);

  if (g_bleLinkState.conn != nullptr &&
      refreshCachedBondRecordFromConn(g_bleLinkState.conn, true, false)) {
    *outRecord = g_bleSecurityState.bondRecord;
    return true;
  }

  if (g_bleSecurityState.bondRecordValid &&
      (!haveIdentity || !g_bleSecurityState.localIdentityValid ||
       (g_bleSecurityState.localIdentityId == identityId))) {
    *outRecord = g_bleSecurityState.bondRecord;
    return true;
  }

  if (g_bleSecurityState.bondLoadCallback != nullptr) {
    BleBondRecord record{};
    if (g_bleSecurityState.bondLoadCallback(&record,
                                            g_bleSecurityState.bondCallbackContext)) {
      g_bleSecurityState.bondRecord = record;
      g_bleSecurityState.bondRecordValid = true;
      *outRecord = g_bleSecurityState.bondRecord;
      return true;
    }
  }

  if (!haveIdentity) {
    if (g_bleLinkState.conn == nullptr) {
      return false;
    }

    bt_conn_info info;
    memset(&info, 0, sizeof(info));
    if (bt_conn_get_info(g_bleLinkState.conn, &info) != 0 ||
        info.type != BT_CONN_TYPE_LE) {
      return false;
    }
    identityId = info.id;
  }

  BleBondRecord record{};
  if (!loadBondRecordFromIdentity(identityId, &record)) {
    return false;
  }

  if (!cacheBondRecord(record, identityId, false)) {
    return false;
  }

  *outRecord = g_bleSecurityState.bondRecord;
  return true;
}

bool BleRadio::clearBondRecord(bool clearPersistentStorage) {
  bool callbackOk = true;
  if (g_bleSecurityState.bondClearCallback != nullptr) {
    callbackOk =
        g_bleSecurityState.bondClearCallback(g_bleSecurityState.bondCallbackContext);
  }

  if (clearPersistentStorage) {
    uint8_t identityId = BT_ID_DEFAULT;
    const bool haveIdentity = resolveLocalIdentityId(&identityId);

    bt_addr_le_t peer;
    bt_addr_le_t* peerPtr = nullptr;
    if (g_bleSecurityState.bondRecordValid) {
      memset(&peer, 0, sizeof(peer));
      peer.type = g_bleSecurityState.bondRecord.peerAddressRandom
                      ? BT_ADDR_LE_RANDOM
                      : BT_ADDR_LE_PUBLIC;
      memcpy(peer.a.val, g_bleSecurityState.bondRecord.peerAddress,
             sizeof(g_bleSecurityState.bondRecord.peerAddress));
      peerPtr = &peer;
    }

    if (haveIdentity) {
      (void)bt_unpair(identityId, peerPtr);
    } else {
      (void)bt_unpair(BT_ID_DEFAULT, peerPtr);
    }
  }

  clearCachedBondRecord();
  bleTraceLog("bond-cleared");
  return callbackOk;
}

void BleRadio::setBondPersistenceCallbacks(BleBondLoadCallback loadCallback,
                                           BleBondSaveCallback saveCallback,
                                           BleBondClearCallback clearCallback,
                                           void* context) {
  g_bleSecurityState.bondLoadCallback = loadCallback;
  g_bleSecurityState.bondSaveCallback = saveCallback;
  g_bleSecurityState.bondClearCallback = clearCallback;
  g_bleSecurityState.bondCallbackContext = context;
}

void BleRadio::setTraceCallback(BleTraceCallback callback, void* context) {
  g_bleSecurityState.traceCallback = callback;
  g_bleSecurityState.traceContext = context;
}

bool BleRadio::getLastDisconnectReason(uint8_t* outReason, bool* outRemote) const {
  servicePendingBleSecurityActions();
  if (outReason == nullptr || !g_bleLinkState.disconnectReasonValid) {
    return false;
  }

  *outReason = g_bleLinkState.disconnectReason;
  if (outRemote != nullptr) {
    *outRemote = g_bleLinkState.disconnectReasonRemote;
  }
  return true;
}

bool BleRadio::disconnect(uint32_t spinLimit) {
  servicePendingBleSecurityActions();
  if (g_bleLinkState.conn == nullptr) {
    return false;
  }

  if (bt_conn_disconnect(g_bleLinkState.conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN) != 0) {
    return false;
  }

  const uint32_t deadlineMs = millis() + bleScanWindowMsFromSpin(spinLimit, 10U, 750U);
  while (static_cast<int32_t>(millis() - deadlineMs) < 0) {
    if (!g_bleLinkState.connected) {
      return true;
    }
    delay(1);
  }

  return !g_bleLinkState.connected;
}

bool BleRadio::pollConnectionEvent(BleConnectionEvent* event, uint32_t spinLimit) {
  BleConnectionEvent scratch{};
  if (event == nullptr) {
    event = &scratch;
  }

  memset(event, 0, sizeof(*event));
  const uint32_t deadlineMs = millis() + bleScanWindowMsFromSpin(spinLimit, 10U, 750U);
  while (static_cast<int32_t>(millis() - deadlineMs) < 0) {
    servicePendingBleSecurityActions();
    if (g_bleLinkState.disconnectEventPending) {
      event->eventStarted = true;
      event->terminateInd = true;
      event->disconnectReasonValid = g_bleLinkState.disconnectReasonValid;
      event->disconnectReasonRemote = g_bleLinkState.disconnectReasonRemote;
      event->disconnectReason = g_bleLinkState.disconnectReason;
      event->eventCounter = g_bleLinkState.eventCounter++;
      event->dataChannel = g_bleLinkState.nextDataChannel++ % 37U;
      event->rssiDbm = g_bleLinkState.rssiDbm;
      g_bleLinkState.disconnectEventPending = false;
      g_bleLinkState.disconnectReasonValid = false;
      clearBlePendingRx();
      clearBlePendingTx();
      return true;
    }

    if (!g_bleLinkState.connected) {
      delay(1);
      continue;
    }

    const uint32_t intervalMs = bleSyntheticEventIntervalMs();
    const bool dueByTime =
        (g_bleLinkState.lastEventMs == 0U) ||
        ((millis() - g_bleLinkState.lastEventMs) >= intervalMs);
    if (!g_bleLinkState.connectEventPending && !g_bleLinkState.parameterUpdatePending &&
        g_bleLinkState.pendingRxPayloadLen == 0U &&
        g_bleLinkState.pendingTxPayloadLen == 0U && !dueByTime) {
      delay(1);
      continue;
    }

    event->eventStarted = true;
    event->packetReceived = (g_bleLinkState.pendingRxPayloadLen > 0U);
    event->crcOk = event->packetReceived;
    event->packetIsNew = g_bleLinkState.connectEventPending ||
                         (g_bleLinkState.pendingRxPayloadLen > 0U);
    event->peerAckedLastTx = true;
    event->freshTxAllowed = true;
    event->eventCounter = g_bleLinkState.eventCounter++;
    event->dataChannel = g_bleLinkState.nextDataChannel++ % 37U;
    event->rssiDbm = g_bleLinkState.rssiDbm;
    event->llid =
        (g_bleLinkState.pendingRxPayloadLen > 0U) ? 0x02U : 0x01U;
    event->emptyAckTransmitted =
        (g_bleLinkState.pendingTxPayloadLen == 0U);
    event->txPacketSent = (g_bleLinkState.pendingTxPayloadLen > 0U);
    event->txLlid = (g_bleLinkState.pendingTxPayloadLen > 0U) ? 0x02U : 0x01U;
    if (g_bleLinkState.pendingRxPayloadLen > 0U) {
      event->attPacket = true;
      event->attOpcode = g_bleLinkState.pendingRxAttOpcode;
      event->payloadLength = g_bleLinkState.pendingRxPayloadLen;
      event->payload = g_bleLinkState.pendingRxPayload;
    }
    if (g_bleLinkState.pendingTxPayloadLen > 0U) {
      event->attPacket = true;
      event->attOpcode = g_bleLinkState.pendingTxAttOpcode;
      event->txPayloadLength = g_bleLinkState.pendingTxPayloadLen;
      event->txPayload = g_bleLinkState.pendingTxPayload;
      event->emptyAckTransmitted = false;
    }
    g_bleLinkState.connectEventPending = false;
    g_bleLinkState.parameterUpdatePending = false;
    g_bleLinkState.lastEventMs = millis();
    if (g_bleLinkState.pendingRxPayloadLen > 0U) {
      clearBlePendingRx();
    }
    if (g_bleLinkState.pendingTxPayloadLen > 0U) {
      clearBlePendingTx();
    }
    return true;
  }

  return false;
}

bool BleRadio::scanCycle(BleScanPacket* packet, uint32_t perChannelSpinLimit) {
  if (packet == nullptr) {
    return false;
  }
  if (!initialized_ && !begin(txPowerDbm_)) {
    return false;
  }

  memset(packet, 0, sizeof(*packet));
  const BleAdvertisingChannel pseudoChannel =
      bleAdvertisingChannelFromIndex(scanCycleStartIndex_);
  scanCycleStartIndex_ = static_cast<uint8_t>((scanCycleStartIndex_ + 1U) % 3U);

  BLE.stopAdvertising();
  (void)bt_le_adv_stop();
  (void)bt_le_scan_stop();

  BleScanShimContext context{};
  context.mode = BleScanShimMode::kPassive;
  context.ready = false;
  context.done = false;
  context.deadlineMs =
      millis() + bleScanWindowMsFromSpin(static_cast<uint64_t>(perChannelSpinLimit) * 3ULL,
                                         120U);
  context.pseudoChannel = pseudoChannel;
  context.passivePacket.channel = pseudoChannel;
  context.passivePacket.payload = context.passivePayload;

  bt_le_scan_param scanParam = BT_LE_SCAN_PARAM_INIT(
      BT_LE_SCAN_TYPE_PASSIVE, 0U, BT_GAP_SCAN_FAST_INTERVAL, BT_GAP_SCAN_FAST_WINDOW);
  g_bleScanContext = &context;
  int err = bt_le_scan_start(&scanParam, bleScanShimCallback);
  if (err == -EALREADY || err == -EBUSY) {
    (void)bt_le_scan_stop();
    delay(1);
    err = bt_le_scan_start(&scanParam, bleScanShimCallback);
  }
  if (err != 0) {
    g_bleScanContext = nullptr;
    return false;
  }

  while (!context.done &&
         static_cast<int32_t>(millis() - context.deadlineMs) < 0) {
    delay(1);
  }

  (void)bt_le_scan_stop();
  g_bleScanContext = nullptr;
  if (!context.ready) {
    return false;
  }

  memcpy(passiveScanPayload_, context.passivePayload, sizeof(passiveScanPayload_));
  *packet = context.passivePacket;
  packet->payload = passiveScanPayload_;
  return true;
}

bool BleRadio::scanActiveCycle(BleActiveScanResult* result,
                               uint32_t perChannelAdvListenSpinLimit,
                               uint32_t scanRspListenSpinLimit) {
  if (result == nullptr) {
    return false;
  }
  if (!initialized_ && !begin(txPowerDbm_)) {
    return false;
  }

  memset(result, 0, sizeof(*result));
  const BleAdvertisingChannel pseudoChannel =
      bleAdvertisingChannelFromIndex(scanCycleStartIndex_);
  scanCycleStartIndex_ = static_cast<uint8_t>((scanCycleStartIndex_ + 1U) % 3U);

  BLE.stopAdvertising();
  (void)bt_le_adv_stop();
  (void)bt_le_scan_stop();

  BleScanShimContext context{};
  context.mode = BleScanShimMode::kActive;
  context.ready = false;
  context.done = false;
  context.deadlineMs =
      millis() + bleScanWindowMsFromSpin(
                      static_cast<uint64_t>(perChannelAdvListenSpinLimit) * 3ULL, 120U);
  context.postAdvDeadlineMs =
      millis() + bleScanWindowMsFromSpin(scanRspListenSpinLimit, 30U, 250U);
  context.pseudoChannel = pseudoChannel;
  context.activeResult.channel = pseudoChannel;

  bt_le_scan_param scanParam = BT_LE_SCAN_PARAM_INIT(
      BT_LE_SCAN_TYPE_ACTIVE, 0U, BT_GAP_SCAN_FAST_INTERVAL, BT_GAP_SCAN_FAST_WINDOW);
  g_bleScanContext = &context;
  int err = bt_le_scan_start(&scanParam, bleScanShimCallback);
  if (err == -EALREADY || err == -EBUSY) {
    (void)bt_le_scan_stop();
    delay(1);
    err = bt_le_scan_start(&scanParam, bleScanShimCallback);
  }
  if (err != 0) {
    g_bleScanContext = nullptr;
    return false;
  }

  while (!context.done &&
         static_cast<int32_t>(millis() - context.deadlineMs) < 0) {
    if (context.ready &&
        static_cast<int32_t>(millis() - context.postAdvDeadlineMs) >= 0) {
      context.done = true;
      break;
    }
    delay(1);
  }

  (void)bt_le_scan_stop();
  g_bleScanContext = nullptr;
  if (!context.ready) {
    return false;
  }

  *result = context.activeResult;
  return true;
}

bool BleRadio::resolveLocalIdentityId(uint8_t* outIdentityId) const {
  if (outIdentityId == nullptr) {
    return false;
  }

  if (!addressConfigured_) {
    *outIdentityId = BT_ID_DEFAULT;
    return true;
  }

  if (customAdvertisingIdentity_) {
    *outIdentityId = advertisingIdentityId_;
    return true;
  }

  return findIdentityIdByAddress(address_, addressType_, outIdentityId);
}

bool BleRadio::ensureAdvertisingIdentity() {
  if (!initialized_ && !begin(txPowerDbm_)) {
    return false;
  }
  if (!addressConfigured_) {
    advertisingIdentityId_ = BT_ID_DEFAULT;
    advertisingIdentityDirty_ = false;
    return true;
  }
  if (!advertisingIdentityDirty_) {
    return true;
  }

  uint8_t existingIdentityId = BT_ID_DEFAULT;
  if (findIdentityIdByAddress(address_, addressType_, &existingIdentityId)) {
    advertisingIdentityId_ = existingIdentityId;
    customAdvertisingIdentity_ = true;
    advertisingIdentityDirty_ = false;
    return true;
  }

  bt_addr_le_t address;
  memset(&address, 0, sizeof(address));
  address.type = zephyrAddressTypeFromBle(addressType_);
  memcpy(address.a.val, address_, sizeof(address_));

  int id = 0;
  if (customAdvertisingIdentity_) {
    id = bt_id_reset(advertisingIdentityId_, &address, nullptr);
  } else {
    id = bt_id_create(&address, nullptr);
  }
  if (id < 0) {
    return false;
  }

  advertisingIdentityId_ = static_cast<uint8_t>(id);
  customAdvertisingIdentity_ = true;
  advertisingIdentityDirty_ = false;
  return true;
}

bool BleRadio::advertiseEvent(uint32_t interChannelDelayUs, uint32_t spinLimit) {
  (void)interChannelDelayUs;
  (void)spinLimit;
  (void)useChSel2_;
  (void)radioBase_;
  (void)ficrBase_;

  servicePendingBleSecurityActions();
  if (!initialized_ && !begin(txPowerDbm_)) {
    return false;
  }
  if (!ensureCustomGattRegistered() || !buildAdvertisingPacket() ||
      !buildScanResponsePacket() ||
      !ensureAdvertisingIdentity()) {
    return false;
  }

  bt_data ad[kBleMaxAdStructures];
  size_t adCount = 0U;
  if (!parseBleAdvertisingPayload(advertisingData_, advertisingDataLen_, ad,
                                  kBleMaxAdStructures, &adCount)) {
    return false;
  }

  bt_data sd[kBleMaxAdStructures];
  size_t sdCount = 0U;
  if (!parseBleAdvertisingPayload(scanResponseData_, scanResponseDataLen_, sd,
                                  kBleMaxAdStructures, &sdCount)) {
    return false;
  }

  const char* localName = effectiveLocalName();
  if (localName != nullptr && localName[0] != '\0') {
    (void)BLE.setLocalName(localName);
  }

  BLE.stopAdvertising();
  (void)bt_le_adv_stop();

  uint32_t options = 0U;
  switch (pduType_) {
    case BleAdvPduType::kAdvInd:
      options = BT_LE_ADV_OPT_CONN;
      break;
    case BleAdvPduType::kAdvNonConnInd:
      options = 0U;
      sdCount = 0U;
      break;
    case BleAdvPduType::kAdvScanInd:
      options = BT_LE_ADV_OPT_SCANNABLE;
      break;
    default:
      return false;
  }
  if (advertisingIdentityId_ != BT_ID_DEFAULT) {
    options |= BT_LE_ADV_OPT_USE_IDENTITY;
  }

  bt_le_adv_param advParam = BT_LE_ADV_PARAM_INIT(
      options, kBleLegacyOneShotAdvInterval, kBleLegacyOneShotAdvInterval, nullptr);
  advParam.options = options;
  advParam.id = advertisingIdentityId_;

  int err = bt_le_adv_start(&advParam, (adCount > 0U) ? ad : nullptr, adCount,
                            (sdCount > 0U) ? sd : nullptr, sdCount);
  if (err == -EALREADY) {
    (void)bt_le_adv_stop();
    delay(1);
    err = bt_le_adv_start(&advParam, (adCount > 0U) ? ad : nullptr, adCount,
                          (sdCount > 0U) ? sd : nullptr, sdCount);
  }
  if (err != 0) {
    return false;
  }

  delay(kBleLegacyOneShotWindowMs);
  (void)bt_le_adv_stop();
  return true;
}

bool BleRadio::advertiseInteractEvent(BleAdvInteraction* interaction,
                                      uint32_t interChannelDelayUs,
                                      uint32_t requestListenSpinLimit,
                                      uint32_t spinLimit) {
  (void)interChannelDelayUs;
  (void)radioBase_;
  (void)ficrBase_;

  servicePendingBleSecurityActions();
  if (!initialized_ && !begin(txPowerDbm_)) {
    return false;
  }
  if (!ensureBatteryService() || !ensureCustomGattRegistered() ||
      !buildAdvertisingPacket() ||
      !buildScanResponsePacket() || !ensureAdvertisingIdentity()) {
    return false;
  }

  bt_data ad[kBleMaxAdStructures];
  size_t adCount = 0U;
  if (!parseBleAdvertisingPayload(advertisingData_, advertisingDataLen_, ad,
                                  kBleMaxAdStructures, &adCount)) {
    return false;
  }

  bt_data sd[kBleMaxAdStructures];
  size_t sdCount = 0U;
  if (!parseBleAdvertisingPayload(scanResponseData_, scanResponseDataLen_, sd,
                                  kBleMaxAdStructures, &sdCount)) {
    return false;
  }

  if (interaction != nullptr) {
    memset(interaction, 0, sizeof(*interaction));
    interaction->channel = BleAdvertisingChannel::k37;
  }

  const char* localName = effectiveLocalName();
  if (localName != nullptr && localName[0] != '\0') {
    (void)BLE.setLocalName(localName);
  }

  BLE.stopAdvertising();
  (void)bt_le_adv_stop();

  uint32_t options = 0U;
  switch (pduType_) {
    case BleAdvPduType::kAdvInd:
      options = BT_LE_ADV_OPT_CONN;
      break;
    case BleAdvPduType::kAdvNonConnInd:
      options = 0U;
      sdCount = 0U;
      break;
    case BleAdvPduType::kAdvScanInd:
      options = BT_LE_ADV_OPT_SCANNABLE;
      break;
    default:
      return false;
  }
  if (advertisingIdentityId_ != BT_ID_DEFAULT) {
    options |= BT_LE_ADV_OPT_USE_IDENTITY;
  }

  bt_le_adv_param advParam = BT_LE_ADV_PARAM_INIT(
      options, kBleLegacyOneShotAdvInterval, kBleLegacyOneShotAdvInterval, nullptr);
  advParam.options = options;
  advParam.id = advertisingIdentityId_;

  int err = bt_le_adv_start(&advParam, (adCount > 0U) ? ad : nullptr, adCount,
                            (sdCount > 0U) ? sd : nullptr, sdCount);
  if (err == -EALREADY) {
    (void)bt_le_adv_stop();
    delay(1);
    err = bt_le_adv_start(&advParam, (adCount > 0U) ? ad : nullptr, adCount,
                          (sdCount > 0U) ? sd : nullptr, sdCount);
  }
  if (err != 0) {
    return false;
  }

  const uint32_t waitMs = bleScanWindowMsFromSpin(
      static_cast<uint64_t>(requestListenSpinLimit) +
          static_cast<uint64_t>(spinLimit),
      kBleLegacyOneShotWindowMs, 2500U);
  const uint32_t startedAt = millis();
  while ((millis() - startedAt) < waitMs) {
    if (g_bleLinkState.connected || g_bleLinkState.connectEventPending) {
      break;
    }
    delay(5);
  }
  (void)bt_le_adv_stop();

  if (interaction != nullptr) {
    interaction->scanResponseTransmitted = false;
    if (g_bleLinkState.connected || g_bleLinkState.connectEventPending) {
      interaction->receivedConnectInd = true;
      interaction->connectIndChSel2 = useChSel2_;
      interaction->peerAddressRandom =
          (g_bleLinkState.peerAddress.type == BT_ADDR_LE_RANDOM);
      interaction->rssiDbm = g_bleLinkState.rssiDbm;
      memcpy(interaction->peerAddress, g_bleLinkState.peerAddress.a.val,
             sizeof(interaction->peerAddress));
    }
  }
  return true;
}

bool BleRadio::ensureBatteryService() {
  if (!initialized_) {
    return false;
  }

#if defined(CONFIG_BT_BAS)
  batteryServiceAdded_ = true;
  return bt_bas_set_battery_level(batteryLevel_) == 0;
#else
  auto* service = static_cast<BLEService*>(batteryService_);
  auto* characteristic =
      static_cast<BLECharacteristic*>(batteryLevelCharacteristic_);

  if (service == nullptr) {
    service = new BLEService("180F");
    batteryService_ = service;
  }
  if (characteristic == nullptr) {
    characteristic =
        new BLECharacteristic("2A19",
                              BLECharacteristic::BLERead |
                                  BLECharacteristic::BLENotify,
                              1U);
    batteryLevelCharacteristic_ = characteristic;
    service->addCharacteristic(*characteristic);
  }
  if (!batteryServiceAdded_) {
    BLE.addService(*service);
    (void)BLE.setAdvertisedService(*service);
    batteryServiceAdded_ = true;
  }

  const uint8_t value = batteryLevel_;
  return characteristic->writeValue(&value, sizeof(value));
#endif
}

const char* BleRadio::effectiveLocalName() const {
  if (advertisingName_[0] != '\0') {
    return advertisingName_;
  }
  if (gattDeviceName_[0] != '\0') {
    return gattDeviceName_;
  }
  return "XIAO-nRF54L15";
}

}  // namespace xiao_nrf54l15
