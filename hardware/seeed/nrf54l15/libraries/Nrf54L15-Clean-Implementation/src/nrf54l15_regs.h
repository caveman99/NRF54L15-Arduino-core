#pragma once

#include <stdint.h>
#include <nrf54l15.h>

namespace nrf54l15 {

inline volatile uint32_t& reg32(uint32_t addr) {
  return *reinterpret_cast<volatile uint32_t*>(addr);
}

#ifdef NRF_TRUSTZONE_NONSECURE
constexpr uint32_t SPIM20_BASE = 0x400C6000UL;
constexpr uint32_t DPPIC20_BASE = 0x400C2000UL;
constexpr uint32_t TWIM20_BASE = static_cast<uint32_t>(NRF_TWIM20_NS_BASE);
constexpr uint32_t UARTE20_BASE = static_cast<uint32_t>(NRF_UARTE20_NS_BASE);
constexpr uint32_t TWIM21_BASE = static_cast<uint32_t>(NRF_TWIM21_NS_BASE);
constexpr uint32_t UARTE21_BASE = static_cast<uint32_t>(NRF_UARTE21_NS_BASE);
constexpr uint32_t SPIM21_BASE = static_cast<uint32_t>(NRF_TWIM21_NS_BASE);
constexpr uint32_t SAADC_BASE = static_cast<uint32_t>(NRF_SAADC_NS_BASE);
constexpr uint32_t TIMER20_BASE = 0x400CA000UL;
constexpr uint32_t PWM20_BASE = static_cast<uint32_t>(NRF_PWM20_NS_BASE);
constexpr uint32_t GPIOTE20_BASE = 0x400DA000UL;
constexpr uint32_t PDM20_BASE = 0x400D0000UL;
constexpr uint32_t CLOCK_BASE = static_cast<uint32_t>(NRF_CLOCK_NS_BASE);
constexpr uint32_t OSCILLATORS_BASE =
    static_cast<uint32_t>(NRF_OSCILLATORS_NS_BASE);
constexpr uint32_t TEMP_BASE = static_cast<uint32_t>(NRF_TEMP_NS_BASE);
constexpr uint32_t GRTC_BASE = 0x400E2000UL;
constexpr uint32_t WDT31_BASE = 0x40109000UL;
constexpr uint32_t POWER_BASE = 0x4010E000UL;
constexpr uint32_t RESET_BASE = 0x4010E000UL;
constexpr uint32_t REGULATORS_BASE = 0x40120000UL;
constexpr uint32_t COMP_BASE = 0x40106000UL;
constexpr uint32_t LPCOMP_BASE = 0x40106000UL;
#else
constexpr uint32_t SPIM20_BASE = 0x500C6000UL;
constexpr uint32_t DPPIC20_BASE = 0x500C2000UL;
constexpr uint32_t TWIM20_BASE = static_cast<uint32_t>(NRF_TWIM20_S_BASE);
constexpr uint32_t UARTE20_BASE = static_cast<uint32_t>(NRF_UARTE20_S_BASE);
constexpr uint32_t TWIM21_BASE = static_cast<uint32_t>(NRF_TWIM21_S_BASE);
constexpr uint32_t UARTE21_BASE = static_cast<uint32_t>(NRF_UARTE21_S_BASE);
constexpr uint32_t SPIM21_BASE = static_cast<uint32_t>(NRF_TWIM21_S_BASE);
constexpr uint32_t SAADC_BASE = static_cast<uint32_t>(NRF_SAADC_S_BASE);
constexpr uint32_t TIMER20_BASE = 0x500CA000UL;
constexpr uint32_t PWM20_BASE = static_cast<uint32_t>(NRF_PWM20_S_BASE);
constexpr uint32_t GPIOTE20_BASE = 0x500DA000UL;
constexpr uint32_t PDM20_BASE = 0x500D0000UL;
constexpr uint32_t CLOCK_BASE = static_cast<uint32_t>(NRF_CLOCK_S_BASE);
constexpr uint32_t OSCILLATORS_BASE =
    static_cast<uint32_t>(NRF_OSCILLATORS_S_BASE);
constexpr uint32_t TEMP_BASE = static_cast<uint32_t>(NRF_TEMP_S_BASE);
constexpr uint32_t GRTC_BASE = 0x500E2000UL;
constexpr uint32_t WDT31_BASE = 0x50109000UL;
constexpr uint32_t POWER_BASE = 0x5010E000UL;
constexpr uint32_t RESET_BASE = 0x5010E000UL;
constexpr uint32_t REGULATORS_BASE = 0x50120000UL;
constexpr uint32_t COMP_BASE = 0x50106000UL;
constexpr uint32_t LPCOMP_BASE = 0x50106000UL;
#endif

namespace gpio {
constexpr uint32_t PIN_CNF_DIR_Pos = 0;
constexpr uint32_t PIN_CNF_INPUT_Pos = 1;
constexpr uint32_t PIN_CNF_PULL_Pos = 2;
constexpr uint32_t PIN_CNF_DRIVE0_Pos = 8;
constexpr uint32_t PIN_CNF_DRIVE1_Pos = 10;

constexpr uint32_t PIN_CNF_DIR_Msk = 0x1UL << PIN_CNF_DIR_Pos;
constexpr uint32_t PIN_CNF_INPUT_Msk = 0x1UL << PIN_CNF_INPUT_Pos;
constexpr uint32_t PIN_CNF_PULL_Msk = 0x3UL << PIN_CNF_PULL_Pos;
constexpr uint32_t PIN_CNF_DRIVE0_Msk = 0x3UL << PIN_CNF_DRIVE0_Pos;
constexpr uint32_t PIN_CNF_DRIVE1_Msk = 0x3UL << PIN_CNF_DRIVE1_Pos;

constexpr uint32_t PULL_DISABLED = 0U;
constexpr uint32_t PULL_DOWN = 1U;
constexpr uint32_t PULL_UP = 3U;
constexpr uint32_t DRIVE0_S0 = 0U;
constexpr uint32_t DRIVE1_D1 = 2U;
}  // namespace gpio

}  // namespace nrf54l15
