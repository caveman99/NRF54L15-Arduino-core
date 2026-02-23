#ifndef XIAO_NRF54L15_LIBRARY_H
#define XIAO_NRF54L15_LIBRARY_H

#include <Arduino.h>

enum XiaoAntennaMode {
    XIAO_ANTENNA_CERAMIC = 0,
    XIAO_ANTENNA_EXTERNAL = 1,
};

enum XiaoPeripheral {
    XIAO_PERIPHERAL_UART0 = 0,
    XIAO_PERIPHERAL_UART1 = 1,
    XIAO_PERIPHERAL_I2C0 = 2,
    XIAO_PERIPHERAL_SPI0 = 3,
    XIAO_PERIPHERAL_ADC = 4,
    XIAO_PERIPHERAL_PWM0 = 5,
};

enum XiaoPowerProfile {
    XIAO_POWER_PROFILE_PERFORMANCE = 0,
    XIAO_POWER_PROFILE_BALANCED = 1,
    XIAO_POWER_PROFILE_ULTRA_LOW_POWER = 2,
};

class XiaoNrf54L15Class {
public:
    enum Antenna : uint8_t {
        CERAMIC = XIAO_ANTENNA_CERAMIC,
        EXTERNAL = XIAO_ANTENNA_EXTERNAL,
    };

    enum RadioProfile : uint8_t {
        RADIO_BLE_ONLY = 0,
        RADIO_DUAL = 1,
        RADIO_802154_ONLY = 2,
        RADIO_UNKNOWN = 255,
    };

    // Compatibility API retained for existing sketches.
    void setAntenna(XiaoAntennaMode antenna);
    XiaoAntennaMode antenna() const;
    bool usingExternalAntenna() const;

    // Preferred API used by bundled board-test examples.
    bool setAntenna(Antenna antenna);
    Antenna getAntenna() const;
    bool useCeramicAntenna();
    bool useExternalAntenna();
    RadioProfile getRadioProfile() const;
    int getBtTxPowerDbm() const;
    bool isExternalAntennaBuild() const;
    bool setRfSwitchEnabled(bool enabled = true) const;
    bool rfSwitchEnabled() const;

    // VBAT helpers for XIAO nRF54L15 battery monitor path.
    bool setBatteryMeasurementEnabled(bool enabled = true) const;
    bool batteryMeasurementEnabled() const;
    int readBatteryRaw(uint8_t samples = 1) const;
    float readBatteryVoltage(float dividerRatio = 2.0f,
                             float referenceVoltage = 3.3f,
                             uint8_t samples = 4) const;

    const char* radioProfileName() const;
    bool bleEnabled() const;
    bool ieee802154Enabled() const;

    void sleepMs(uint32_t ms) const;
    void sleepUs(uint32_t us) const;
    bool requestSystemOff() const;
    void systemOff() const;
    bool watchdogStart(uint32_t timeoutMs,
                       bool pauseInSleep = false,
                       bool pauseInDebug = true) const;
    bool watchdogFeed() const;
    bool watchdogStop() const;
    bool watchdogActive() const;
    int watchdogLastError() const;

    uint32_t resetCause() const;
    void clearResetCause() const;
    bool resetWasWatchdog() const;

    uint32_t cpuFrequencyHz() const;
    uint32_t cpuFrequencyFromToolsHz() const;
    bool setCpuFrequencyHz(uint32_t hz) const;
    bool setPeripheralEnabled(XiaoPeripheral peripheral, bool enabled) const;
    bool peripheralEnabled(XiaoPeripheral peripheral) const;
    int peripheralLastError() const;
    bool applyPowerProfile(XiaoPowerProfile profile) const;
    XiaoPowerProfile powerProfile() const;

    bool channelSoundingEnabled() const;
    bool ble6FeatureSetRequested() const;
    bool ble6FeatureSetEnabled() const;
};

extern XiaoNrf54L15Class XiaoNrf54L15;

#endif
