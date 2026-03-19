#pragma once

#include <stdint.h>

namespace xiao_nrf54l15 {

struct Pin {
  uint8_t port;
  uint8_t pin;
};

constexpr Pin kPinDisconnected{0xFF, 0xFF};

inline constexpr bool isConnected(const Pin& p) {
  return p.port <= 2U && p.pin <= 31U;
}

constexpr Pin kPinD0{1, 4};
constexpr Pin kPinD1{1, 5};
constexpr Pin kPinD2{1, 6};
constexpr Pin kPinD3{1, 7};
constexpr Pin kPinD4{1, 10};
constexpr Pin kPinD5{1, 11};
constexpr Pin kPinD6{2, 8};
constexpr Pin kPinD7{2, 7};
constexpr Pin kPinD8{2, 1};
constexpr Pin kPinD9{2, 4};
constexpr Pin kPinD10{2, 2};
constexpr Pin kPinD11{0, 3};
constexpr Pin kPinD12{0, 4};
constexpr Pin kPinD13{2, 10};
constexpr Pin kPinD14{2, 9};
constexpr Pin kPinD15{2, 6};

constexpr Pin kPinA0 = kPinD0;
constexpr Pin kPinA1 = kPinD1;
constexpr Pin kPinA2 = kPinD2;
constexpr Pin kPinA3 = kPinD3;
constexpr Pin kPinA4 = kPinD4;
constexpr Pin kPinA5 = kPinD5;
constexpr Pin kPinA6{1, 13};
constexpr Pin kPinA7{1, 14};

constexpr Pin kPinUserLed{2, 0};
constexpr Pin kPinUserButton{0, 0};

constexpr Pin kPinSAMD11Tx{1, 8};
constexpr Pin kPinSAMD11Rx{1, 9};

constexpr Pin kPinVbatEnable{1, 15};
constexpr Pin kPinVbatSense{1, 14};

constexpr Pin kPinImuScl{0, 3};
constexpr Pin kPinImuSda{0, 4};
constexpr Pin kPinImuInt{0, 2};
constexpr Pin kPinImuMicPowerEnable{0, 1};

constexpr Pin kPinMicClk{1, 12};
constexpr Pin kPinMicData{1, 13};

constexpr Pin kPinRfSwitchPower{2, 3};
constexpr Pin kPinRfSwitchCtl{2, 5};

constexpr Pin kDefaultI2cScl = kPinD5;
constexpr Pin kDefaultI2cSda = kPinD4;
constexpr Pin kDefaultSpiSck = kPinD8;
constexpr Pin kDefaultSpiMosi = kPinD10;
constexpr Pin kDefaultSpiMiso = kPinD9;
constexpr Pin kDefaultUartTx = kPinD6;
constexpr Pin kDefaultUartRx = kPinD7;

inline constexpr int8_t saadcInputForPin(const Pin& pin) {
  if (pin.port != 1U) {
    return -1;
  }

  switch (pin.pin) {
    case 4:
      return 0;
    case 5:
      return 1;
    case 6:
      return 2;
    case 7:
      return 3;
    case 11:
      return 4;
    case 12:
      return 5;
    case 13:
      return 6;
    case 14:
      return 7;
    default:
      return -1;
  }
}

}  // namespace xiao_nrf54l15
