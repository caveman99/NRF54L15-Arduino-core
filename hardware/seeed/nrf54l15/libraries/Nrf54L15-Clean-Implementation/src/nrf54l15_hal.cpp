#include "nrf54l15_hal.h"

#include <Arduino.h>
#include <Bluetooth.h>
#include <HardwareSerial.h>
#include <PDM.h>
#include <SPI.h>
#include <Wire.h>
#include <errno.h>
#include <string.h>
#include <variant.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/random/random.h>

namespace {

using namespace xiao_nrf54l15;

CpuFrequency g_idleCpuFrequency = CpuFrequency::k64MHz;
bool g_idleCpuScalingEnabled = false;

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
  if (readyEvent == nullptr) {
    return false;
  }

  while (spinLimit-- > 0U) {
    if (*readyEvent != 0U) {
      return true;
    }
  }
  return false;
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
      includeAdvertisingFlags_(true),
      batteryLevel_(100U),
      initialized_(false),
      advertisingName_{0},
      scanResponseName_{0},
      gattDeviceName_{0},
      batteryService_(nullptr),
      batteryLevelCharacteristic_(nullptr),
      batteryServiceAdded_(false) {}

bool BleRadio::begin(int8_t txPowerDbm) {
  txPowerDbm_ = txPowerDbm;
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

bool BleRadio::setAdvertisingPduType(BleAdvPduType type) {
  pduType_ = type;
  return true;
}

bool BleRadio::setAdvertisingName(const char* name, bool includeFlags) {
  includeAdvertisingFlags_ = includeFlags;
  safeCopyString(name, advertisingName_, sizeof(advertisingName_));
  if (initialized_ && advertisingName_[0] != '\0') {
    return BLE.setLocalName(advertisingName_);
  }
  return true;
}

bool BleRadio::setScanResponseName(const char* name) {
  safeCopyString(name, scanResponseName_, sizeof(scanResponseName_));
  return true;
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

  auto* characteristic =
      static_cast<BLECharacteristic*>(batteryLevelCharacteristic_);
  if (characteristic == nullptr) {
    return false;
  }
  const uint8_t value = batteryLevel_;
  return characteristic->writeValue(&value, sizeof(value));
}

bool BleRadio::advertiseInteractEvent(BleAdvInteraction* interaction,
                                      uint32_t interChannelDelayUs,
                                      uint32_t requestListenSpinLimit,
                                      uint32_t spinLimit) {
  (void)interChannelDelayUs;
  (void)requestListenSpinLimit;
  (void)spinLimit;
  (void)pduType_;
  (void)includeAdvertisingFlags_;
  (void)radioBase_;
  (void)ficrBase_;

  if (!initialized_ && !begin(txPowerDbm_)) {
    return false;
  }
  if (!ensureBatteryService()) {
    return false;
  }
  if (advertisingName_[0] != '\0') {
    (void)BLE.setLocalName(advertisingName_);
  }

  if (interaction != nullptr) {
    memset(interaction, 0, sizeof(*interaction));
    interaction->channel = BleAdvertisingChannel::k37;
  }

  if (!BLE.advertise()) {
    return false;
  }

  const uint32_t startedAt = millis();
  while ((millis() - startedAt) < 150UL) {
    delay(10);
  }
  BLE.stopAdvertising();

  if (interaction != nullptr) {
    interaction->scanResponseTransmitted = false;
  }
  return true;
}

bool BleRadio::ensureBatteryService() {
  if (!initialized_) {
    return false;
  }

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
