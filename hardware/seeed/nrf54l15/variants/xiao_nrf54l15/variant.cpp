#include "Arduino.h"
#include "variant.h"

#include <errno.h>
#include <nrf54l15.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/util.h>

#define GPIO_SPEC(port_label, pin_num) \
    { \
        .port = DEVICE_DT_GET(DT_NODELABEL(port_label)), \
        .pin = (pin_num), \
        .dt_flags = 0, \
    }

namespace {

constexpr uint8_t kRfSwitchPowerPort = 2U;
constexpr uint8_t kRfSwitchPowerPin = 3U;
constexpr uint8_t kRfSwitchSelectPort = 2U;
constexpr uint8_t kRfSwitchSelectPin = 5U;
constexpr uint8_t kBatteryEnablePort = 1U;
constexpr uint8_t kBatteryEnablePin = 15U;
constexpr uint8_t kImuMicEnablePort = 0U;
constexpr uint8_t kImuMicEnablePin = 1U;
constexpr uint8_t kSamd11TxPort = 1U;
constexpr uint8_t kSamd11TxPin = 8U;
constexpr uint8_t kSamd11RxPort = 1U;
constexpr uint8_t kSamd11RxPin = 9U;
constexpr uint8_t kRfSwitchCeramic = 0U;
constexpr uint8_t kRfSwitchExternal = 1U;
constexpr uint8_t kRfSwitchControlHiz = 2U;

uint8_t g_rf_switch_selection = kRfSwitchCeramic;

NRF_GPIO_Type *gpioForPort(uint8_t port)
{
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

void gpioSetOutput(uint8_t port, uint8_t pin, bool high)
{
    NRF_GPIO_Type *gpio = gpioForPort(port);
    if (gpio == nullptr || pin > 31U) {
        return;
    }

    const uint32_t bit = (1UL << pin);
    gpio->DIRSET = bit;
    if (high) {
        gpio->OUTSET = bit;
    } else {
        gpio->OUTCLR = bit;
    }
}

bool gpioIsOutput(uint8_t port, uint8_t pin)
{
    NRF_GPIO_Type *gpio = gpioForPort(port);
    if (gpio == nullptr || pin > 31U) {
        return false;
    }
    return (gpio->DIR & (1UL << pin)) != 0U;
}

bool gpioReadOutput(uint8_t port, uint8_t pin)
{
    NRF_GPIO_Type *gpio = gpioForPort(port);
    if (gpio == nullptr || pin > 31U) {
        return false;
    }
    return (gpio->OUT & (1UL << pin)) != 0U;
}

void gpioSetInputHighZ(uint8_t port, uint8_t pin)
{
    NRF_GPIO_Type *gpio = gpioForPort(port);
    if (gpio == nullptr || pin > 31U) {
        return;
    }

    const uint32_t bit = (1UL << pin);
    uint32_t cnf = gpio->PIN_CNF[pin];
    cnf &= ~(GPIO_PIN_CNF_DIR_Msk |
             GPIO_PIN_CNF_INPUT_Msk |
             GPIO_PIN_CNF_PULL_Msk |
             GPIO_PIN_CNF_SENSE_Msk);
    cnf |= (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
    cnf |= GPIO_PIN_CNF_INPUT_Disconnect;
    cnf |= GPIO_PIN_CNF_PULL_Disabled;
    cnf |= GPIO_PIN_CNF_SENSE_Disabled;
    gpio->DIRCLR = bit;
    gpio->PIN_CNF[pin] = cnf;
}

void applyRfSwitchPower(bool enable)
{
    gpioSetOutput(kRfSwitchPowerPort, kRfSwitchPowerPin, enable);
}

void applyBatteryEnable(bool enable)
{
    gpioSetOutput(kBatteryEnablePort, kBatteryEnablePin, enable);
}

void applyImuMicEnable(bool enable)
{
    gpioSetOutput(kImuMicEnablePort, kImuMicEnablePin, enable);
}

void applyRfSwitchSelection(uint8_t selection)
{
    const uint8_t normalized_selection =
        (selection == kRfSwitchExternal) ? kRfSwitchExternal : kRfSwitchCeramic;

    applyRfSwitchPower(true);
    gpioSetOutput(kRfSwitchSelectPort,
                  kRfSwitchSelectPin,
                  normalized_selection == kRfSwitchExternal);
    g_rf_switch_selection = normalized_selection;
}

} // namespace

extern "C" uint8_t arduinoXiaoNrf54l15SetAntenna(uint8_t selection)
{
    if (selection == kRfSwitchControlHiz) {
        gpioSetInputHighZ(kRfSwitchSelectPort, kRfSwitchSelectPin);
        g_rf_switch_selection = kRfSwitchControlHiz;
        return 1U;
    }

    applyRfSwitchSelection(selection);
    return 1U;
}

extern "C" uint8_t arduinoXiaoNrf54l15GetAntenna(void)
{
    return g_rf_switch_selection;
}

extern "C" uint8_t arduinoXiaoNrf54l15SetRfSwitchPower(uint8_t enabled)
{
    applyRfSwitchPower(enabled != 0U);
    return 1U;
}

extern "C" uint8_t arduinoXiaoNrf54l15GetRfSwitchPower(void)
{
    return (gpioIsOutput(kRfSwitchPowerPort, kRfSwitchPowerPin) &&
            gpioReadOutput(kRfSwitchPowerPort, kRfSwitchPowerPin))
               ? 1U
               : 0U;
}

extern "C" uint8_t arduinoXiaoNrf54l15SetBatteryEnable(uint8_t enabled)
{
    applyBatteryEnable(enabled != 0U);
    return 1U;
}

extern "C" uint8_t arduinoXiaoNrf54l15GetBatteryEnable(void)
{
    return (gpioIsOutput(kBatteryEnablePort, kBatteryEnablePin) &&
            gpioReadOutput(kBatteryEnablePort, kBatteryEnablePin))
               ? 1U
               : 0U;
}

extern "C" uint8_t arduinoXiaoNrf54l15SetImuMicEnable(uint8_t enabled)
{
    applyImuMicEnable(enabled != 0U);
    return 1U;
}

extern "C" uint8_t arduinoXiaoNrf54l15GetImuMicEnable(void)
{
    return (gpioIsOutput(kImuMicEnablePort, kImuMicEnablePin) &&
            gpioReadOutput(kImuMicEnablePort, kImuMicEnablePin))
               ? 1U
               : 0U;
}

extern "C" void xiaoNrf54l15SetAntenna(xiao_nrf54l15_antenna_t antenna)
{
    switch (antenna) {
        case XIAO_NRF54L15_ANTENNA_EXTERNAL:
            (void)arduinoXiaoNrf54l15SetAntenna(kRfSwitchExternal);
            break;
        case XIAO_NRF54L15_ANTENNA_CONTROL_HIZ:
            (void)arduinoXiaoNrf54l15SetAntenna(kRfSwitchControlHiz);
            break;
        case XIAO_NRF54L15_ANTENNA_CERAMIC:
        default:
            (void)arduinoXiaoNrf54l15SetAntenna(kRfSwitchCeramic);
            break;
    }
}

extern "C" xiao_nrf54l15_antenna_t xiaoNrf54l15GetAntenna(void)
{
    if (!gpioIsOutput(kRfSwitchSelectPort, kRfSwitchSelectPin)) {
        return XIAO_NRF54L15_ANTENNA_CONTROL_HIZ;
    }

    return gpioReadOutput(kRfSwitchSelectPort, kRfSwitchSelectPin)
               ? XIAO_NRF54L15_ANTENNA_EXTERNAL
               : XIAO_NRF54L15_ANTENNA_CERAMIC;
}

extern "C" void xiaoNrf54l15EnterLowestPowerBoardState(void)
{
    applyImuMicEnable(false);
    applyBatteryEnable(false);
    applyRfSwitchPower(false);

    gpioSetInputHighZ(kRfSwitchSelectPort, kRfSwitchSelectPin);
    gpioSetInputHighZ(kSamd11TxPort, kSamd11TxPin);
    gpioSetInputHighZ(kSamd11RxPort, kSamd11RxPin);

    g_rf_switch_selection = kRfSwitchControlHiz;
}

extern "C" {

extern const struct gpio_dt_spec g_pin_map[] = {
    GPIO_SPEC(gpio1, 4),   // D0
    GPIO_SPEC(gpio1, 5),   // D1
    GPIO_SPEC(gpio1, 6),   // D2
    GPIO_SPEC(gpio1, 7),   // D3
    GPIO_SPEC(gpio1, 10),  // D4
    GPIO_SPEC(gpio1, 11),  // D5
    GPIO_SPEC(gpio2, 8),   // D6
    GPIO_SPEC(gpio2, 7),   // D7
    GPIO_SPEC(gpio2, 1),   // D8
    GPIO_SPEC(gpio2, 4),   // D9
    GPIO_SPEC(gpio2, 2),   // D10
    GPIO_SPEC(gpio0, 3),   // D11
    GPIO_SPEC(gpio0, 4),   // D12
    GPIO_SPEC(gpio2, 10),  // D13
    GPIO_SPEC(gpio2, 9),   // D14
    GPIO_SPEC(gpio2, 6),   // D15
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios),
    GPIO_SPEC(gpio1, 9),   // PIN_SAMD11_RX (P1.09)
    GPIO_SPEC(gpio1, 8),   // PIN_SAMD11_TX (P1.08)
    GPIO_SPEC(gpio0, 1),   // IMU_MIC_EN / PIN_IMU_MIC_PWR (P0.01)
    GPIO_SPEC(gpio2, 3),   // RF_SW / PIN_RF_SW (P2.03)
    GPIO_SPEC(gpio2, 5),   // RF_SW_CTL / PIN_RF_SW_CTL (P2.05)
    GPIO_SPEC(gpio1, 15),  // VBAT_EN / PIN_VBAT_EN (P1.15)
};

extern const size_t g_pin_map_size = ARRAY_SIZE(g_pin_map);

#if DT_NODE_HAS_STATUS(DT_NODELABEL(adc), okay)
extern const struct adc_dt_spec g_adc_map[] = {
    ADC_DT_SPEC_STRUCT(DT_NODELABEL(adc), 0),
    ADC_DT_SPEC_STRUCT(DT_NODELABEL(adc), 1),
    ADC_DT_SPEC_STRUCT(DT_NODELABEL(adc), 2),
    ADC_DT_SPEC_STRUCT(DT_NODELABEL(adc), 3),
    ADC_DT_SPEC_STRUCT(DT_NODELABEL(adc), 4),
    ADC_DT_SPEC_STRUCT(DT_NODELABEL(adc), 5),
    ADC_DT_SPEC_STRUCT(DT_NODELABEL(adc), 6),
    ADC_DT_SPEC_STRUCT(DT_NODELABEL(adc), 7),
};
extern const size_t g_adc_map_size = ARRAY_SIZE(g_adc_map);
#else
extern const struct adc_dt_spec g_adc_map[] = {};
extern const size_t g_adc_map_size = 0;
#endif

#define ARDUINO_PWM_SPEC_EMPTY { .dev = NULL, .channel = 0, .period = 0, .flags = 0 }

#if DT_NODE_HAS_STATUS(DT_ALIAS(pwm0), okay)
#define ARDUINO_PWM0_SPEC PWM_DT_SPEC_GET(DT_ALIAS(pwm0))
#else
#define ARDUINO_PWM0_SPEC ARDUINO_PWM_SPEC_EMPTY
#endif

#if DT_NODE_HAS_STATUS(DT_ALIAS(pwm1), okay)
#define ARDUINO_PWM1_SPEC PWM_DT_SPEC_GET(DT_ALIAS(pwm1))
#else
#define ARDUINO_PWM1_SPEC ARDUINO_PWM_SPEC_EMPTY
#endif

#if DT_NODE_HAS_STATUS(DT_ALIAS(pwm2), okay)
#define ARDUINO_PWM2_SPEC PWM_DT_SPEC_GET(DT_ALIAS(pwm2))
#else
#define ARDUINO_PWM2_SPEC ARDUINO_PWM_SPEC_EMPTY
#endif

#if DT_NODE_HAS_STATUS(DT_ALIAS(pwm3), okay)
#define ARDUINO_PWM3_SPEC PWM_DT_SPEC_GET(DT_ALIAS(pwm3))
#else
#define ARDUINO_PWM3_SPEC ARDUINO_PWM_SPEC_EMPTY
#endif

extern const uint8_t g_pwm_pins[] = { PIN_D6, PIN_D7, PIN_D8, PIN_D9 };

extern const struct pwm_dt_spec g_pwm_map[] = {
    ARDUINO_PWM0_SPEC,
    ARDUINO_PWM1_SPEC,
    ARDUINO_PWM2_SPEC,
    ARDUINO_PWM3_SPEC,
};

extern const size_t g_pwm_map_size = ARRAY_SIZE(g_pwm_map);

void initVariant(void)
{
    applyImuMicEnable(false);
    applyBatteryEnable(false);

#if defined(ARDUINO_XIAO_NRF54L15_EXT_ANTENNA)
    xiaoNrf54l15SetAntenna(XIAO_NRF54L15_ANTENNA_EXTERNAL);
#else
    xiaoNrf54l15SetAntenna(XIAO_NRF54L15_ANTENNA_CERAMIC);
#endif
}

} // extern "C"
