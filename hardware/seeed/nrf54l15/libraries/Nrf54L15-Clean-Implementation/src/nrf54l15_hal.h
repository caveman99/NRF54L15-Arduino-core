#pragma once

#include <stddef.h>
#include <stdint.h>

#include "nrf54l15_regs.h"
#include "xiao_nrf54l15_pins.h"

class BLEService;
class BLECharacteristic;

namespace xiao_nrf54l15 {

enum class CpuFrequency : uint32_t {
  k64MHz = 64000000UL,
  k128MHz = 128000000UL,
};

enum class GpioDirection : uint8_t {
  kInput = 0,
  kOutput = 1,
};

enum class GpioPull : uint8_t {
  kDisabled = 0,
  kPullDown = 1,
  kPullUp = 3,
};

class ClockControl {
 public:
  static bool startHfxo(bool waitForTuned = true,
                        uint32_t spinLimit = 1000000UL);
  static void stopHfxo();
  static bool setCpuFrequency(CpuFrequency frequency);
  static CpuFrequency cpuFrequency();
  static bool enableIdleCpuScaling(
      CpuFrequency idleFrequency = CpuFrequency::k64MHz);
  static void disableIdleCpuScaling();
  static bool idleCpuScalingEnabled();
  static CpuFrequency idleCpuFrequency();
};

class Gpio {
 public:
  static bool configure(const Pin& pin,
                        GpioDirection direction,
                        GpioPull pull = GpioPull::kDisabled);
  static bool write(const Pin& pin, bool high);
  static bool read(const Pin& pin, bool* high);
  static bool toggle(const Pin& pin);
  static bool setDriveS0D1(const Pin& pin);
};

enum class SpiMode : uint8_t {
  kMode0 = 0,
  kMode1 = 1,
  kMode2 = 2,
  kMode3 = 3,
};

class Spim {
 public:
  explicit Spim(uint32_t base = nrf54l15::SPIM21_BASE,
                uint32_t coreClockHz = 128000000UL);

  bool begin(const Pin& sck,
             const Pin& mosi,
             const Pin& miso,
             const Pin& cs = kPinDisconnected,
             uint32_t hz = 4000000UL,
             SpiMode mode = SpiMode::kMode0,
             bool lsbFirst = false);
  bool setFrequency(uint32_t hz);
  void end();

  bool transfer(const uint8_t* tx,
                uint8_t* rx,
                size_t len,
                uint32_t spinLimit = 2000000UL);

 private:
  uint32_t base_;
  uint32_t coreClockHz_;
  Pin cs_;
  uint32_t frequencyHz_;
  SpiMode mode_;
  bool lsbFirst_;
  bool active_;
};

class Spis {
 public:
  explicit Spis(uint32_t base = nrf54l15::SPIS20_BASE);

  bool begin(const Pin& sck,
             const Pin& mosi,
             const Pin& miso,
             const Pin& csn,
             SpiMode mode = SpiMode::kMode0,
             bool lsbFirst = false,
             uint8_t defaultChar = 0xFFU,
             uint8_t overReadChar = 0xFFU,
             bool autoAcquireAfterEnd = true);

  bool acquire(uint32_t spinLimit = 2000000UL);
  bool setBuffers(uint8_t* rx,
                  size_t rxLen,
                  const uint8_t* tx,
                  size_t txLen,
                  uint32_t spinLimit = 2000000UL);
  bool releaseTransaction();

  bool pollAcquired(bool clearEvent = true);
  bool pollEnd(bool clearEvent = true);
  size_t receivedBytes() const;
  size_t transmittedBytes() const;
  bool overflowed() const;
  bool overread() const;
  void clearStatus();
  void end();

 private:
  NRF_SPIS_Type* spis_;
  bool active_;
 };

enum class TwimFrequency : uint32_t {
  k100k = 100000UL,
  k250k = 250000UL,
  k400k = 400000UL,
  k1000k = 1000000UL,
};

class Twim {
 public:
  explicit Twim(uint32_t base = nrf54l15::TWIM21_BASE);

  bool begin(const Pin& scl,
             const Pin& sda,
             TwimFrequency frequency = TwimFrequency::k400k);
  bool setFrequency(TwimFrequency frequency);
  void end();

  bool write(uint8_t address7,
             const uint8_t* data,
             size_t len,
             uint32_t spinLimit = 2000000UL);
  bool read(uint8_t address7,
            uint8_t* data,
            size_t len,
            uint32_t spinLimit = 2000000UL);
  bool writeRead(uint8_t address7,
                 const uint8_t* tx,
                 size_t txLen,
                 uint8_t* rx,
                 size_t rxLen,
                 uint32_t spinLimit = 2000000UL);

 private:
  uint32_t base_;
  void* wire_;
  bool active_;
};

enum class UarteBaud : uint32_t {
  k9600 = 9600UL,
  k115200 = 115200UL,
  k1000000 = 1000000UL,
};

class Uarte {
 public:
  explicit Uarte(uint32_t base = nrf54l15::UARTE21_BASE);

  bool begin(const Pin& txd,
             const Pin& rxd,
             UarteBaud baud = UarteBaud::k115200,
             bool hwFlowControl = false,
             const Pin& cts = kPinDisconnected,
             const Pin& rts = kPinDisconnected);
  void end();

  bool write(const uint8_t* data,
             size_t len,
             uint32_t spinLimit = 2000000UL);
  size_t read(uint8_t* data,
              size_t len,
              uint32_t spinLimit = 2000000UL);

 private:
  uint32_t base_;
  void* serial_;
  bool active_;
};

enum class TimerBitWidth : uint8_t {
  k16bit = 0,
  k8bit = 1,
  k24bit = 2,
  k32bit = 3,
};

class Timer {
 public:
  using CompareCallback = void (*)(uint8_t channel, void* context);

  explicit Timer(uint32_t base = nrf54l15::TIMER20_BASE,
                 uint32_t pclkHz = 16000000UL,
                 uint8_t channelCount = 6);

  bool begin(TimerBitWidth bitWidth = TimerBitWidth::k32bit,
             uint8_t prescaler = 4,
             bool counterMode = false);
  bool setFrequency(uint32_t targetHz);
  uint32_t timerHz() const;
  uint32_t ticksFromMicros(uint32_t us) const;

  void start();
  void stop();
  void clear();

  bool setCompare(uint8_t channel,
                  uint32_t ccValue,
                  bool autoClear = false,
                  bool autoStop = false,
                  bool oneShot = false,
                  bool enableInterrupt = false);
  uint32_t capture(uint8_t channel);
  bool pollCompare(uint8_t channel, bool clearEvent = true);
  volatile uint32_t* publishCompareConfigRegister(uint8_t channel) const;
  volatile uint32_t* subscribeStartConfigRegister() const;
  volatile uint32_t* subscribeStopConfigRegister() const;
  volatile uint32_t* subscribeClearConfigRegister() const;
  volatile uint32_t* subscribeCaptureConfigRegister(uint8_t channel) const;

  void enableInterrupt(uint8_t channel, bool enable = true);
  bool attachCompareCallback(uint8_t channel,
                             CompareCallback callback,
                             void* context = nullptr);
  void service();

 private:
  struct CompareSlot {
    uint32_t intervalTicks;
    uint32_t nextFireTicks;
    bool configured;
    bool autoClear;
    bool autoStop;
    bool oneShot;
    bool interruptEnabled;
    bool eventPending;
  };

  uint32_t currentTicks() const;

  uint32_t base_;
  uint32_t pclkHz_;
  uint8_t channelCount_;
  uint8_t prescaler_;
  TimerBitWidth bitWidth_;
  bool counterMode_;
  bool running_;
  uint64_t startMicros_;
  uint32_t storedTicks_;
  CompareCallback callbacks_[8];
  void* callbackContext_[8];
  CompareSlot compareSlots_[8];
};

class Pwm {
 public:
  explicit Pwm(uint32_t base = nrf54l15::PWM20_BASE);

  bool beginSingle(const Pin& outPin,
                   uint32_t frequencyHz = 1000UL,
                   uint16_t dutyPermille = 500,
                   bool activeHigh = true);
  bool setDutyPermille(uint16_t dutyPermille);
  bool setFrequency(uint32_t frequencyHz);

  bool start(uint8_t sequence = 0, uint32_t spinLimit = 2000000UL);
  bool stop(uint32_t spinLimit = 2000000UL);
  void end();

  bool pollPeriodEnd(bool clearEvent = true);

 private:
  bool applyOutput() const;

  uint32_t base_;
  Pin outPin_;
  uint32_t frequencyHz_;
  uint16_t dutyPermille_;
  bool activeHigh_;
  bool configured_;
  bool running_;
  bool periodPending_;
};

enum class GpiotePolarity : uint8_t {
  kNone = 0,
  kLoToHi = 1,
  kHiToLo = 2,
  kToggle = 3,
};

class Gpiote {
 public:
  using InCallback = void (*)(uint8_t channel, void* context);

  explicit Gpiote(uint32_t base = nrf54l15::GPIOTE20_BASE,
                  uint8_t channelCount = 8);

  bool configureEvent(uint8_t channel,
                      const Pin& pin,
                      GpiotePolarity polarity,
                      bool enableInterrupt = false);
  bool configureTask(uint8_t channel,
                     const Pin& pin,
                     GpiotePolarity polarity,
                     bool initialHigh = false);
  void disableChannel(uint8_t channel);

  bool triggerTaskOut(uint8_t channel);
  bool triggerTaskSet(uint8_t channel);
  bool triggerTaskClr(uint8_t channel);

  bool pollInEvent(uint8_t channel, bool clearEvent = true);
  bool pollPortEvent(bool clearEvent = true);
  volatile uint32_t* subscribeTaskOutConfigRegister(uint8_t channel) const;
  volatile uint32_t* subscribeTaskSetConfigRegister(uint8_t channel) const;
  volatile uint32_t* subscribeTaskClrConfigRegister(uint8_t channel) const;

  void enableInterrupt(uint8_t channel, bool enable = true);
  bool attachInCallback(uint8_t channel,
                        InCallback callback,
                        void* context = nullptr);
  void service();

 private:
  struct EventSlot {
    Pin pin;
    GpiotePolarity polarity;
    bool configured;
    bool interruptEnabled;
    bool lastHigh;
    bool eventPending;
  };

  struct TaskSlot {
    Pin pin;
    GpiotePolarity polarity;
    bool configured;
  };

  bool triggerTask(uint8_t channel, bool explicitHigh, bool explicitLow);

  uint32_t base_;
  uint8_t channelCount_;
  InCallback callbacks_[8];
  void* callbackContext_[8];
  EventSlot eventSlots_[8];
  TaskSlot taskSlots_[8];
};

class Dppic {
 public:
  explicit Dppic(uint32_t base = nrf54l15::DPPIC20_BASE);

  bool enableChannel(uint8_t channel, bool enable = true);
  bool channelEnabled(uint8_t channel) const;
  bool configurePublish(volatile uint32_t* publishRegister,
                        uint8_t channel,
                        bool enable = true) const;
  bool configureSubscribe(volatile uint32_t* subscribeRegister,
                          uint8_t channel,
                          bool enable = true) const;
  bool connect(volatile uint32_t* publishRegister,
               volatile uint32_t* subscribeRegister,
               uint8_t channel,
               bool enableChannel = true) const;
  bool disconnectPublish(volatile uint32_t* publishRegister) const;
  bool disconnectSubscribe(volatile uint32_t* subscribeRegister) const;

 private:
  NRF_DPPIC_Type* dppic_;
};

enum class AdcResolution : uint8_t {
  k8bit = 8,
  k10bit = 10,
  k12bit = 12,
  k14bit = 14,
};

enum class AdcGain : uint8_t {
  k2 = 0,
  k1 = 1,
  k2over3 = 2,
  k2over4 = 3,
  k2over5 = 4,
  k2over6 = 5,
  k2over7 = 6,
  k2over8 = 7,
};

enum class AdcOversample : uint8_t {
  kBypass = 0,
  k2x = 1,
  k4x = 2,
  k8x = 3,
  k16x = 4,
  k32x = 5,
  k64x = 6,
  k128x = 7,
  k256x = 8,
};

class Saadc {
 public:
  explicit Saadc(uint32_t base = nrf54l15::SAADC_BASE);

  bool begin(AdcResolution resolution = AdcResolution::k12bit,
             uint32_t spinLimit = 2000000UL);
  bool begin(AdcResolution resolution,
             AdcOversample oversample,
             uint32_t spinLimit = 2000000UL);
  bool calibrate(uint32_t spinLimit = 2000000UL);
  void end();

  bool configureSingleEnded(uint8_t channel,
                            const Pin& pin,
                            AdcGain gain = AdcGain::k2over8,
                            uint16_t tacq = 159,
                            uint8_t tconv = 4,
                            bool burst = false);
  bool configureDifferential(uint8_t channel,
                             const Pin& positivePin,
                             const Pin& negativePin,
                             AdcGain gain = AdcGain::k1,
                             uint16_t tacq = 159,
                             uint8_t tconv = 4,
                             bool burst = false);

  bool sampleRaw(int16_t* outRaw, uint32_t spinLimit = 2000000UL) const;
  bool sampleMilliVolts(int32_t* outMilliVolts,
                        uint32_t spinLimit = 2000000UL) const;
  bool sampleMilliVoltsSigned(int32_t* outMilliVolts,
                              uint32_t spinLimit = 2000000UL) const;

 private:
  int sampleSingleRaw(const Pin& pin) const;
  static uint32_t oversampleCount(AdcOversample oversample);
  static uint32_t maxCountForResolution(AdcResolution resolution);
  static bool gainRatio(AdcGain gain,
                        uint32_t* numerator,
                        uint32_t* denominator);

  uint32_t base_;
  AdcResolution resolution_;
  AdcGain gain_;
  AdcOversample oversample_;
  bool differential_;
  bool configured_;
  Pin positivePin_;
  Pin negativePin_;
};

enum class CompReference : uint8_t {
  kInt1V2 = COMP_REFSEL_REFSEL_Int1V2,
  kVdd = COMP_REFSEL_REFSEL_VDD,
  kExternalAref = COMP_REFSEL_REFSEL_ARef,
};

enum class CompSpeedMode : uint8_t {
  kLowPower = COMP_MODE_SP_Low,
  kNormal = COMP_MODE_SP_Normal,
  kHighSpeed = COMP_MODE_SP_High,
};

enum class CompCurrentSource : uint8_t {
  kDisabled = COMP_ISOURCE_ISOURCE_Off,
  k2uA5 = COMP_ISOURCE_ISOURCE_Ien2uA5,
  k5uA = COMP_ISOURCE_ISOURCE_Ien5uA,
  k10uA = COMP_ISOURCE_ISOURCE_Ien10uA,
};

class Comp {
 public:
  explicit Comp(uint32_t base = nrf54l15::COMP_BASE);

  bool beginThreshold(const Pin& inputPin,
                      uint16_t thresholdPermille = 500U,
                      uint16_t hysteresisPermille = 0U,
                      CompReference reference = CompReference::kVdd,
                      CompSpeedMode speed = CompSpeedMode::kLowPower,
                      const Pin& externalReferencePin = kPinDisconnected,
                      CompCurrentSource currentSource =
                          CompCurrentSource::kDisabled,
                      uint32_t spinLimit = 200000UL);
  bool beginSingleEnded(const Pin& inputPin,
                        CompReference reference = CompReference::kVdd,
                        uint16_t thresholdPermille = 500U,
                        uint16_t hysteresisPermille = 0U,
                        CompSpeedMode speed = CompSpeedMode::kLowPower,
                        const Pin& externalReferencePin = kPinDisconnected,
                        CompCurrentSource currentSource =
                            CompCurrentSource::kDisabled,
                        uint32_t spinLimit = 200000UL);
  bool beginDifferential(const Pin& positivePin,
                         const Pin& negativePin,
                         CompSpeedMode speed = CompSpeedMode::kLowPower,
                         bool hysteresis = false,
                         CompCurrentSource currentSource =
                             CompCurrentSource::kDisabled,
                         uint32_t spinLimit = 200000UL);
  bool setThresholdWindowPermille(uint16_t lowPermille,
                                  uint16_t highPermille);
  void setCurrentSource(CompCurrentSource currentSource);
  bool sample(uint32_t spinLimit = 200000UL) const;
  bool resultAbove() const;
  bool pollReady(bool clearEvent = true);
  bool pollUp(bool clearEvent = true);
  bool pollDown(bool clearEvent = true);
  bool pollCross(bool clearEvent = true);
  void clearEvents();
  void end();

 private:
  NRF_COMP_Type* comp_;
  bool active_;
};

enum class LpcompReference : uint8_t {
  k1over8Vdd = LPCOMP_REFSEL_REFSEL_Ref1_8Vdd,
  k2over8Vdd = LPCOMP_REFSEL_REFSEL_Ref2_8Vdd,
  k3over8Vdd = LPCOMP_REFSEL_REFSEL_Ref3_8Vdd,
  k4over8Vdd = LPCOMP_REFSEL_REFSEL_Ref4_8Vdd,
  k5over8Vdd = LPCOMP_REFSEL_REFSEL_Ref5_8Vdd,
  k6over8Vdd = LPCOMP_REFSEL_REFSEL_Ref6_8Vdd,
  k7over8Vdd = LPCOMP_REFSEL_REFSEL_Ref7_8Vdd,
  kExternalAref = LPCOMP_REFSEL_REFSEL_ARef,
  k1over16Vdd = LPCOMP_REFSEL_REFSEL_Ref1_16Vdd,
  k3over16Vdd = LPCOMP_REFSEL_REFSEL_Ref3_16Vdd,
  k5over16Vdd = LPCOMP_REFSEL_REFSEL_Ref5_16Vdd,
  k7over16Vdd = LPCOMP_REFSEL_REFSEL_Ref7_16Vdd,
  k9over16Vdd = LPCOMP_REFSEL_REFSEL_Ref9_16Vdd,
  k11over16Vdd = LPCOMP_REFSEL_REFSEL_Ref11_16Vdd,
  k13over16Vdd = LPCOMP_REFSEL_REFSEL_Ref13_16Vdd,
  k15over16Vdd = LPCOMP_REFSEL_REFSEL_Ref15_16Vdd,
};

enum class LpcompDetect : uint8_t {
  kCross = LPCOMP_ANADETECT_ANADETECT_Cross,
  kUp = LPCOMP_ANADETECT_ANADETECT_Up,
  kDown = LPCOMP_ANADETECT_ANADETECT_Down,
};

class Lpcomp {
 public:
  explicit Lpcomp(uint32_t base = nrf54l15::LPCOMP_BASE);

  bool begin(const Pin& inputPin,
             LpcompReference reference = LpcompReference::k4over8Vdd,
             bool hysteresis = false,
             LpcompDetect detect = LpcompDetect::kCross,
             const Pin& externalReferencePin = kPinDisconnected,
             uint32_t spinLimit = 200000UL);
  bool beginThreshold(const Pin& inputPin,
                      uint16_t thresholdPermille,
                      bool hysteresis = false,
                      LpcompDetect detect = LpcompDetect::kCross,
                      const Pin& externalReferencePin = kPinDisconnected,
                      uint32_t spinLimit = 200000UL);
  void configureAnalogDetect(LpcompDetect detect);
  bool sample(uint32_t spinLimit = 200000UL) const;
  bool resultAbove() const;
  bool pollReady(bool clearEvent = true);
  bool pollUp(bool clearEvent = true);
  bool pollDown(bool clearEvent = true);
  bool pollCross(bool clearEvent = true);
  void clearEvents();
  void end();

 private:
  NRF_LPCOMP_Type* lpcomp_;
  bool active_;
};

enum class QdecSamplePeriod : uint8_t {
  k128us = QDEC_SAMPLEPER_SAMPLEPER_128us,
  k256us = QDEC_SAMPLEPER_SAMPLEPER_256us,
  k512us = QDEC_SAMPLEPER_SAMPLEPER_512us,
  k1024us = QDEC_SAMPLEPER_SAMPLEPER_1024us,
  k2048us = QDEC_SAMPLEPER_SAMPLEPER_2048us,
  k4096us = QDEC_SAMPLEPER_SAMPLEPER_4096us,
  k8192us = QDEC_SAMPLEPER_SAMPLEPER_8192us,
  k16384us = QDEC_SAMPLEPER_SAMPLEPER_16384us,
  k32ms = QDEC_SAMPLEPER_SAMPLEPER_32ms,
  k65ms = QDEC_SAMPLEPER_SAMPLEPER_65ms,
  k131ms = QDEC_SAMPLEPER_SAMPLEPER_131ms,
};

enum class QdecReportPeriod : uint8_t {
  k10Samples = QDEC_REPORTPER_REPORTPER_10Smpl,
  k40Samples = QDEC_REPORTPER_REPORTPER_40Smpl,
  k80Samples = QDEC_REPORTPER_REPORTPER_80Smpl,
  k120Samples = QDEC_REPORTPER_REPORTPER_120Smpl,
  k160Samples = QDEC_REPORTPER_REPORTPER_160Smpl,
  k200Samples = QDEC_REPORTPER_REPORTPER_200Smpl,
  k240Samples = QDEC_REPORTPER_REPORTPER_240Smpl,
  k280Samples = QDEC_REPORTPER_REPORTPER_280Smpl,
  k1Sample = QDEC_REPORTPER_REPORTPER_1Smpl,
};

enum class QdecLedPolarity : uint8_t {
  kActiveLow = QDEC_LEDPOL_LEDPOL_ActiveLow,
  kActiveHigh = QDEC_LEDPOL_LEDPOL_ActiveHigh,
};

enum class QdecInputPull : uint8_t {
  kDisabled = GPIO_PIN_CNF_PULL_Disabled,
  kPullDown = GPIO_PIN_CNF_PULL_Pulldown,
  kPullUp = GPIO_PIN_CNF_PULL_Pullup,
};

class Qdec {
 public:
  explicit Qdec(uint32_t base = nrf54l15::QDEC20_BASE);

  bool begin(const Pin& pinA,
             const Pin& pinB,
             QdecSamplePeriod samplePeriod = QdecSamplePeriod::k1024us,
             QdecReportPeriod reportPeriod = QdecReportPeriod::k1Sample,
             bool debounce = true,
             QdecInputPull inputPull = QdecInputPull::kPullUp,
             const Pin& ledPin = kPinDisconnected,
             QdecLedPolarity ledPolarity = QdecLedPolarity::kActiveLow,
             uint16_t ledPreUs = 16U);
  void end();
  void start();
  void stop();

  int32_t sampleValue() const;
  int32_t accumulator() const;
  int32_t readAndClearAccumulator();
  uint32_t doubleTransitions() const;
  uint32_t readAndClearDoubleTransitions();

  bool pollSampleReady(bool clearEvent = true);
  bool pollReportReady(bool clearEvent = true);
  bool pollOverflow(bool clearEvent = true);
  bool pollDoubleReady(bool clearEvent = true);
  bool pollStopped(bool clearEvent = true);

 private:
  NRF_QDEC_Type* qdec_;
  bool configured_;
};

enum class BoardAntennaPath : uint8_t {
  kCeramic = 0,
  kExternal = 1,
  kControlHighImpedance = 2,
};

class BoardControl {
 public:
  static bool setAntennaPath(BoardAntennaPath path);
  static BoardAntennaPath antennaPath();

  static bool setImuMicEnabled(bool enable);
  static bool imuMicEnabled();

  static bool setBatterySenseEnabled(bool enable);

  static bool setRfSwitchPowerEnabled(bool enable);
  static bool rfSwitchPowerEnabled();

  static bool enableRfPath(BoardAntennaPath path = BoardAntennaPath::kCeramic);
  static bool collapseRfPathIdle(
      BoardAntennaPath idlePath = BoardAntennaPath::kControlHighImpedance,
      bool disablePower = true);

  static void enterLowestPowerState();

  static bool sampleBatteryMilliVolts(int32_t* outMilliVolts,
                                      uint32_t settleDelayUs = 5000U,
                                      uint32_t spinLimit = 500000UL);
  static bool sampleBatteryPercent(uint8_t* outPercent,
                                   int32_t emptyMilliVolts = 3300,
                                   int32_t fullMilliVolts = 4200,
                                   uint32_t settleDelayUs = 5000U,
                                   uint32_t spinLimit = 500000UL);
};

enum class PowerLatencyMode : uint8_t {
  kLowPower = 0,
  kConstantLatency = 1,
};

enum class PowerFailThreshold : uint8_t {
  k1V7 = REGULATORS_POFCON_THRESHOLD_V17,
  k1V8 = REGULATORS_POFCON_THRESHOLD_V18,
  k1V9 = REGULATORS_POFCON_THRESHOLD_V19,
  k2V0 = REGULATORS_POFCON_THRESHOLD_V20,
  k2V1 = REGULATORS_POFCON_THRESHOLD_V21,
  k2V2 = REGULATORS_POFCON_THRESHOLD_V22,
  k2V3 = REGULATORS_POFCON_THRESHOLD_V23,
  k2V4 = REGULATORS_POFCON_THRESHOLD_V24,
  k2V5 = REGULATORS_POFCON_THRESHOLD_V25,
  k2V6 = REGULATORS_POFCON_THRESHOLD_V26,
  k2V7 = REGULATORS_POFCON_THRESHOLD_V27,
  k2V8 = REGULATORS_POFCON_THRESHOLD_V28,
  k2V9 = REGULATORS_POFCON_THRESHOLD_V29,
  k3V0 = REGULATORS_POFCON_THRESHOLD_V30,
  k3V1 = REGULATORS_POFCON_THRESHOLD_V31,
  k3V2 = REGULATORS_POFCON_THRESHOLD_V32,
};

class PowerManager {
 public:
  explicit PowerManager(uint32_t powerBase = nrf54l15::POWER_BASE,
                        uint32_t resetBase = nrf54l15::RESET_BASE,
                        uint32_t regulatorsBase = nrf54l15::REGULATORS_BASE);

  void setLatencyMode(PowerLatencyMode mode);
  bool isConstantLatency() const;

  bool setRetention(uint8_t index, uint8_t value);
  bool getRetention(uint8_t index, uint8_t* value) const;

  uint32_t resetReason() const;
  void clearResetReason(uint32_t mask);

  bool enableMainDcdc(bool enable);
  bool configurePowerFailComparator(
      PowerFailThreshold threshold = PowerFailThreshold::k2V8,
      bool enableWarningEvent = true);
  void disablePowerFailComparator();
  bool powerFailComparatorEnabled() const;
  PowerFailThreshold powerFailThreshold() const;
  bool powerBelowPowerFailThreshold() const;
  bool powerFailWarningEventEnabled() const;
  bool pollPowerFailWarning(bool clearEvent = true);
  void clearPowerFailWarning();

  [[noreturn]] void systemOff();
  [[noreturn]] void systemOffTimedWakeMs(uint32_t delayMs);
  [[noreturn]] void systemOffTimedWakeUs(uint32_t delayUs);
  [[noreturn]] void systemOffNoRetention();
  [[noreturn]] void systemOffTimedWakeMsNoRetention(uint32_t delayMs);
  [[noreturn]] void systemOffTimedWakeUsNoRetention(uint32_t delayUs);

 private:
  NRF_POWER_Type* power_;
  NRF_RESET_Type* reset_;
  NRF_REGULATORS_Type* regulators_;
};

enum class GrtcClockSource : uint8_t {
  kLfxo = GRTC_CLKCFG_CLKSEL_LFXO,
  kSystemLfclk = GRTC_CLKCFG_CLKSEL_SystemLFCLK,
  kLflprc = GRTC_CLKCFG_CLKSEL_LFLPRC,
};

class Grtc {
 public:
  struct AlarmContext {
    Grtc* owner;
    uint8_t logicalChannel;
  };

  explicit Grtc(uint32_t base = nrf54l15::GRTC_BASE,
                uint8_t compareChannelCount = 12);

  bool begin(GrtcClockSource clockSource = GrtcClockSource::kSystemLfclk);
  void end();
  void start();
  void stop();
  void clear();

  uint64_t counter() const;
  bool setWakeLeadLfclk(uint8_t cycles);

  bool setCompareOffsetUs(uint8_t channel,
                          uint32_t offsetUs,
                          bool enableChannel = true);
  bool setCompareAbsoluteUs(uint8_t channel,
                            uint64_t timestampUs,
                            bool enableChannel = true);
  bool enableCompareChannel(uint8_t channel, bool enable = true);
  void enableCompareInterrupt(uint8_t channel, bool enable = true);
  bool pollCompare(uint8_t channel, bool clearEvent = true);
  bool clearCompareEvent(uint8_t channel);
  void markCompareEvent(uint8_t channel);

 private:
  const void* counter_;
  uint8_t compareChannelCount_;
  uint8_t wakeLeadLfclk_;
  bool compareEvents_[12];
  uint8_t actualChannels_[12];
  bool interruptsEnabled_[12];
  AlarmContext alarmContexts_[12];
  bool softwareRunning_;
  uint64_t softwareBaseMicros_;
  uint64_t stoppedCounterUs_;
  uint64_t compareDeadlineUs_[12];
  bool compareArmed_[12];
};

class TempSensor {
 public:
  explicit TempSensor(uint32_t base = nrf54l15::TEMP_BASE);

  bool sampleQuarterDegreesC(int32_t* outQuarterDegreesC,
                             uint32_t spinLimit = 200000UL) const;
  bool sampleMilliDegreesC(int32_t* outMilliDegreesC,
                           uint32_t spinLimit = 200000UL) const;

 private:
  NRF_TEMP_Type* temp_;
};

class Watchdog {
 public:
  explicit Watchdog(uint32_t base = nrf54l15::WDT31_BASE);

  bool configure(uint32_t timeoutMs,
                 uint8_t reloadRegister = 0,
                 bool runInSleep = true,
                 bool runInDebugHalt = false,
                 bool allowStop = false);
  void start();
  bool stop(uint32_t spinLimit = 200000UL);
  bool feed(uint8_t reloadRegister = 0xFFU);
  bool isRunning() const;
  uint32_t requestStatus() const;

 private:
  NRF_WDT_Type* wdt_;
  uint8_t defaultReloadRegister_;
  bool allowStop_;
};

enum class PdmEdge : uint8_t {
  kLeftRising = PDM_MODE_EDGE_LeftRising,
  kLeftFalling = PDM_MODE_EDGE_LeftFalling,
};

class Pdm {
 public:
  explicit Pdm(uint32_t base = nrf54l15::PDM20_BASE);

  bool begin(const Pin& clk,
             const Pin& din,
             bool mono = true,
             uint8_t prescalerDiv = 40,
             uint8_t ratio = PDM_RATIO_RATIO_Ratio64,
             PdmEdge edge = PdmEdge::kLeftRising);
  void end();

  bool capture(int16_t* samples,
               size_t sampleCount,
               uint32_t spinLimit = 4000000UL);

 private:
 uint32_t base_;
 bool configured_;
 bool mono_;
};

bool connectI2s20Interrupt(void (*handler)(), uint8_t priority = 3U);
void disconnectI2s20Interrupt();

struct I2sTxConfig {
  Pin mck = kPinDisconnected;
  Pin sck = kPinDisconnected;
  Pin lrck = kPinDisconnected;
  Pin sdin = kPinDisconnected;
  Pin sdout = kPinDisconnected;
  uint32_t mckFreq = I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV8;
  uint32_t ratio = I2S_CONFIG_RATIO_RATIO_256X;
  uint32_t sampleWidth = I2S_CONFIG_SWIDTH_SWIDTH_16Bit;
  uint32_t align = I2S_CONFIG_ALIGN_ALIGN_Left;
  uint32_t format = I2S_CONFIG_FORMAT_FORMAT_I2S;
  uint32_t channels = I2S_CONFIG_CHANNELS_CHANNELS_Stereo;
  uint8_t irqPriority = 3U;
  bool enableMasterClock = true;
  bool autoRestart = true;
};

class I2sTx {
 public:
  using RefillCallback = void (*)(uint32_t* buffer,
                                  uint32_t wordCount,
                                  void* context);

  explicit I2sTx(uint32_t base = nrf54l15::I2S20_BASE);

  bool begin(const I2sTxConfig& config,
             uint32_t* buffer0,
             uint32_t* buffer1,
             uint32_t wordCount);
  void end();

  bool start();
  bool stop();
  void service();
  void onIrq();

  bool setBuffers(uint32_t* buffer0, uint32_t* buffer1, uint32_t wordCount);
  void setRefillCallback(RefillCallback callback, void* context = nullptr);
  bool makeActive();
  static void irqHandler();

  bool configured() const;
  bool running() const;
  bool restartPending() const;
  uint32_t txPtrUpdCount() const;
  uint32_t stoppedCount() const;
  uint32_t restartCount() const;
  uint32_t manualStopCount() const;

 private:
  void clearEvents();
  void armBuffer(uint8_t bufferIndex);

  NRF_I2S_Type* i2s_;
  I2sTxConfig config_;
  uint32_t* buffers_[2];
  uint32_t wordCount_;
  RefillCallback refillCallback_;
  void* refillContext_;
  uint8_t nextBufferIndex_;
  bool configured_;
  bool running_;
  bool restartPending_;
  uint32_t txPtrUpdCount_;
  uint32_t stoppedCount_;
  uint32_t restartCount_;
  uint32_t manualStopCount_;
};

struct I2sRxConfig {
  Pin mck = kPinDisconnected;
  Pin sck = kPinDisconnected;
  Pin lrck = kPinDisconnected;
  Pin sdin = kPinDisconnected;
  Pin sdout = kPinDisconnected;
  uint32_t mckFreq = I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV8;
  uint32_t ratio = I2S_CONFIG_RATIO_RATIO_256X;
  uint32_t sampleWidth = I2S_CONFIG_SWIDTH_SWIDTH_16Bit;
  uint32_t align = I2S_CONFIG_ALIGN_ALIGN_Left;
  uint32_t format = I2S_CONFIG_FORMAT_FORMAT_I2S;
  uint32_t channels = I2S_CONFIG_CHANNELS_CHANNELS_Stereo;
  uint8_t irqPriority = 3U;
  bool enableMasterClock = true;
  bool autoRestart = true;
};

class I2sRx {
 public:
  using ReceiveCallback = void (*)(uint32_t* buffer,
                                   uint32_t wordCount,
                                   void* context);

  explicit I2sRx(uint32_t base = nrf54l15::I2S20_BASE);

  bool begin(const I2sRxConfig& config,
             uint32_t* buffer0,
             uint32_t* buffer1,
             uint32_t wordCount);
  void end();

  bool start();
  bool stop();
  void service();
  void onIrq();

  bool setBuffers(uint32_t* buffer0, uint32_t* buffer1, uint32_t wordCount);
  void setReceiveCallback(ReceiveCallback callback, void* context = nullptr);
  bool makeActive();
  static void irqHandler();

  bool configured() const;
  bool running() const;
  bool restartPending() const;
  uint32_t rxPtrUpdCount() const;
  uint32_t stoppedCount() const;
  uint32_t restartCount() const;
  uint32_t manualStopCount() const;

 private:
  void clearEvents();
  void armBuffer(uint8_t bufferIndex);

  NRF_I2S_Type* i2s_;
  I2sRxConfig config_;
  uint32_t* buffers_[2];
  uint32_t wordCount_;
  ReceiveCallback receiveCallback_;
  void* receiveContext_;
  uint8_t nextBufferIndex_;
  bool configured_;
  bool running_;
  bool restartPending_;
  uint32_t rxPtrUpdCount_;
  uint32_t stoppedCount_;
  uint32_t restartCount_;
  uint32_t manualStopCount_;
};

struct I2sDuplexConfig {
  Pin mck = kPinDisconnected;
  Pin sck = kPinDisconnected;
  Pin lrck = kPinDisconnected;
  Pin sdin = kPinDisconnected;
  Pin sdout = kPinDisconnected;
  uint32_t mckFreq = I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV8;
  uint32_t ratio = I2S_CONFIG_RATIO_RATIO_256X;
  uint32_t sampleWidth = I2S_CONFIG_SWIDTH_SWIDTH_16Bit;
  uint32_t align = I2S_CONFIG_ALIGN_ALIGN_Left;
  uint32_t format = I2S_CONFIG_FORMAT_FORMAT_I2S;
  uint32_t channels = I2S_CONFIG_CHANNELS_CHANNELS_Stereo;
  uint8_t irqPriority = 3U;
  bool enableMasterClock = true;
  bool autoRestart = true;
};

class I2sDuplex {
 public:
  using TxRefillCallback = void (*)(uint32_t* buffer,
                                    uint32_t wordCount,
                                    void* context);
  using RxReceiveCallback = void (*)(uint32_t* buffer,
                                     uint32_t wordCount,
                                     void* context);

  explicit I2sDuplex(uint32_t base = nrf54l15::I2S20_BASE);

  bool begin(const I2sDuplexConfig& config,
             uint32_t* txBuffer0,
             uint32_t* txBuffer1,
             uint32_t* rxBuffer0,
             uint32_t* rxBuffer1,
             uint32_t wordCount);
  void end();

  bool start();
  bool stop();
  void service();
  void onIrq();

  bool setTxBuffers(uint32_t* buffer0, uint32_t* buffer1, uint32_t wordCount);
  bool setRxBuffers(uint32_t* buffer0, uint32_t* buffer1, uint32_t wordCount);
  void setTxRefillCallback(TxRefillCallback callback, void* context = nullptr);
  void setRxReceiveCallback(RxReceiveCallback callback, void* context = nullptr);
  bool makeActive();
  static void irqHandler();

  bool configured() const;
  bool running() const;
  bool restartPending() const;
  uint32_t txPtrUpdCount() const;
  uint32_t rxPtrUpdCount() const;
  uint32_t stoppedCount() const;
  uint32_t restartCount() const;
  uint32_t manualStopCount() const;

 private:
  void clearEvents();
  void armTxBuffer(uint8_t bufferIndex);
  void armRxBuffer(uint8_t bufferIndex);

  NRF_I2S_Type* i2s_;
  I2sDuplexConfig config_;
  uint32_t* txBuffers_[2];
  uint32_t* rxBuffers_[2];
  uint32_t wordCount_;
  TxRefillCallback txRefillCallback_;
  void* txRefillContext_;
  RxReceiveCallback rxReceiveCallback_;
  void* rxReceiveContext_;
  uint8_t nextTxBufferIndex_;
  uint8_t nextRxBufferIndex_;
  bool configured_;
  bool running_;
  bool restartPending_;
  uint32_t txPtrUpdCount_;
  uint32_t rxPtrUpdCount_;
  uint32_t stoppedCount_;
  uint32_t restartCount_;
  uint32_t manualStopCount_;
};

class CracenRng {
 public:
  explicit CracenRng(uint32_t controlBase = 0U, uint32_t coreBase = 0U);

  bool begin(uint32_t spinLimit = 400000UL);
  void end();
  bool fill(void* data, size_t length, uint32_t spinLimit = 400000UL);
  bool randomWord(uint32_t* outWord, uint32_t spinLimit = 400000UL);
  uint32_t availableWords() const;
  uint32_t status() const;
  bool healthy() const;
  bool active() const;
  void clearEvent();

 private:
  uint32_t controlBase_;
  uint32_t coreBase_;
  bool active_;
  uint32_t status_;
};

class Aar {
 public:
  explicit Aar(uint32_t base = 0U);

  bool resolveFirst(const uint8_t address[6],
                    const uint8_t* irks,
                    size_t irkCount,
                    bool* outResolved,
                    uint16_t* outIndex = nullptr,
                    uint32_t spinLimit = 200000UL);
  bool resolveSingle(const uint8_t address[6],
                     const uint8_t irk[16],
                     bool* outResolved,
                     uint32_t spinLimit = 200000UL);
  uint32_t errorStatus() const;
  uint32_t resolvedAmountBytes() const;
  void clearEvents();

 private:
  uint32_t base_;
  uint32_t errorStatus_;
  uint32_t resolvedAmountBytes_;
};

class Ecb {
 public:
  explicit Ecb(uint32_t base = 0U);

  bool encryptBlock(const uint8_t key[16],
                    const uint8_t plaintext[16],
                    uint8_t ciphertext[16],
                    uint32_t spinLimit = 200000UL);
  bool encryptBlockInPlace(const uint8_t key[16],
                           uint8_t block[16],
                           uint32_t spinLimit = 200000UL);
  uint32_t errorStatus() const;
  void clearEvents();

 private:
  uint32_t base_;
  uint32_t errorStatus_;
};

enum class CcmBleDataRate : uint8_t {
  k125Kbit = 0,
  k250Kbit = 1,
  k500Kbit = 2,
  k1Mbit = 3,
  k2Mbit = 4,
  k4Mbit = 5,
};

class Ccm {
 public:
  explicit Ccm(uint32_t base = 0U);

  bool encryptBlePacket(const uint8_t key[16],
                        const uint8_t iv[8],
                        uint64_t counter,
                        uint8_t direction,
                        uint8_t header,
                        const uint8_t* plaintext,
                        uint8_t plaintextLen,
                        uint8_t* outCipherWithMic,
                        uint8_t* outCipherWithMicLen,
                        CcmBleDataRate dataRate = CcmBleDataRate::k125Kbit,
                        uint8_t adataMask = 0xE3U,
                        uint32_t spinLimit = 200000UL);
  bool decryptBlePacket(const uint8_t key[16],
                        const uint8_t iv[8],
                        uint64_t counter,
                        uint8_t direction,
                        uint8_t header,
                        const uint8_t* cipherWithMic,
                        uint8_t cipherWithMicLen,
                        uint8_t* outPlaintext,
                        uint8_t* outPlaintextLen,
                        bool* outMacValid = nullptr,
                        CcmBleDataRate dataRate = CcmBleDataRate::k125Kbit,
                        uint8_t adataMask = 0xE3U,
                        uint32_t spinLimit = 200000UL);
  uint32_t errorStatus() const;
  bool macStatus() const;
  void clearEvents();

 private:
  uint32_t base_;
  uint32_t errorStatus_;
  bool macValid_;
};

enum class BleAddressType : uint8_t {
  kPublic = 0,
  kRandomStatic = 1,
};

enum class BleAdvPduType : uint8_t {
  kAdvInd = 0x00,
  kAdvDirectInd = 0x01,
  kAdvNonConnInd = 0x02,
  kScanReq = 0x03,
  kScanRsp = 0x04,
  kConnectInd = 0x05,
  kAdvScanInd = 0x06,
};

enum class BleAdvertisingChannel : uint8_t {
  k37 = 37,
  k38 = 38,
  k39 = 39,
};

enum BleGattCharacteristicProperty : uint8_t {
  kBleGattPropRead = 0x02U,
  kBleGattPropWriteNoRsp = 0x04U,
  kBleGattPropWrite = 0x08U,
  kBleGattPropNotify = 0x10U,
  kBleGattPropIndicate = 0x20U,
};

struct BleScanPacket {
  BleAdvertisingChannel channel;
  int8_t rssiDbm;
  uint8_t pduHeader;
  uint8_t length;
  const uint8_t* payload;
};

struct BleActiveScanResult {
  BleAdvertisingChannel channel;
  int8_t advRssiDbm;
  uint8_t advHeader;
  uint8_t advPayloadLength;
  bool advertiserAddressRandom;
  uint8_t advertiserAddress[6];
  uint8_t advPayload[31];
  bool scanResponseReceived;
  int8_t scanRspRssiDbm;
  uint8_t scanRspHeader;
  uint8_t scanRspPayloadLength;
  uint8_t scanRspPayload[31];
};

struct BleAdvInteraction {
  BleAdvertisingChannel channel;
  bool receivedScanRequest;
  bool scanResponseTransmitted;
  bool receivedConnectInd;
  bool connectIndChSel2;
  bool peerAddressRandom;
  int8_t rssiDbm;
  uint8_t peerAddress[6];
};

struct BleConnectionInfo {
  uint8_t peerAddress[6];
  bool peerAddressRandom;
  uint32_t accessAddress;
  uint32_t crcInit;
  uint16_t intervalUnits;
  uint16_t latency;
  uint16_t supervisionTimeoutUnits;
  uint8_t channelMap[5];
  uint8_t channelCount;
  uint8_t hopIncrement;
  uint8_t sleepClockAccuracy;
};

struct BleConnectionEvent {
  bool eventStarted;
  bool packetReceived;
  bool crcOk;
  bool emptyAckTransmitted;
  bool packetIsNew;
  bool peerAckedLastTx;
  bool freshTxAllowed;
  bool implicitEmptyAck;
  bool terminateInd;
  bool disconnectReasonValid;
  bool disconnectReasonRemote;
  bool llControlPacket;
  bool attPacket;
  bool txPacketSent;
  uint16_t eventCounter;
  uint8_t dataChannel;
  int8_t rssiDbm;
  uint8_t llid;
  uint8_t rxNesn;
  uint8_t rxSn;
  uint8_t disconnectReason;
  uint8_t llControlOpcode;
  uint8_t attOpcode;
  uint8_t payloadLength;
  uint8_t txLlid;
  uint8_t txNesn;
  uint8_t txSn;
  uint8_t txPayloadLength;
  const uint8_t* payload;
  const uint8_t* txPayload;
};

using BleGattWriteCallback = void (*)(uint16_t valueHandle, const uint8_t* value,
                                      uint8_t valueLength, bool withResponse,
                                      void* context);

class BleRadio {
 public:
  static constexpr uint8_t kCustomGattMaxServices = 4U;
  static constexpr uint8_t kCustomGattMaxCharacteristics = 8U;
  static constexpr uint8_t kCustomGattMaxValueLength = 20U;
  static constexpr uint8_t kCustomGattUuid128Length = 16U;

  explicit BleRadio(uint32_t radioBase = 0U, uint32_t ficrBase = 0U);

  bool begin(int8_t txPowerDbm = -8);
  void end();

  bool setTxPowerDbm(int8_t dbm);
  bool selectExternalAntenna(bool external);
  bool loadAddressFromFicr(bool forceRandomStatic = true);
  bool setDeviceAddress(const uint8_t address[6],
                        BleAddressType type = BleAddressType::kRandomStatic);
  bool getDeviceAddress(uint8_t addressOut[6],
                        BleAddressType* typeOut = nullptr) const;
  bool setAdvertisingPduType(BleAdvPduType type);
  bool setAdvertisingChannelSelectionAlgorithm2(bool enabled);
  bool setAdvertisingData(const uint8_t* data, size_t len);
  bool setAdvertisingName(const char* name, bool includeFlags = true);
  bool buildAdvertisingPacket();
  bool setScanResponseName(const char* name);
  bool setScanResponseData(const uint8_t* data, size_t len);
  bool buildScanResponsePacket();
  bool setGattDeviceName(const char* name);
  bool setGattBatteryLevel(uint8_t percent);
  bool clearCustomGatt();
  bool addCustomGattService(uint16_t uuid16,
                            uint16_t* outServiceHandle = nullptr);
  bool addCustomGattService128(
      const uint8_t uuid128[kCustomGattUuid128Length],
      uint16_t* outServiceHandle = nullptr);
  bool addCustomGattCharacteristic(uint16_t serviceHandle, uint16_t uuid16,
                                   uint8_t properties,
                                   const uint8_t* initialValue = nullptr,
                                   uint8_t initialValueLength = 0U,
                                   uint16_t* outValueHandle = nullptr,
                                   uint16_t* outCccdHandle = nullptr);
  bool addCustomGattCharacteristic128(
      uint16_t serviceHandle,
      const uint8_t uuid128[kCustomGattUuid128Length],
      uint8_t properties,
      const uint8_t* initialValue = nullptr,
      uint8_t initialValueLength = 0U,
      uint16_t* outValueHandle = nullptr,
      uint16_t* outCccdHandle = nullptr);
  bool setCustomGattCharacteristicValue(uint16_t valueHandle,
                                        const uint8_t* value,
                                        uint8_t valueLength);
  bool getCustomGattCharacteristicValue(uint16_t valueHandle,
                                        uint8_t* outValue,
                                        uint8_t* inOutValueLength) const;
  bool notifyCustomGattCharacteristic(uint16_t valueHandle,
                                      bool indicate = false);
  bool isCustomGattCccdEnabled(uint16_t valueHandle,
                               bool indication = false) const;
  bool setCustomGattWriteHandler(uint16_t valueHandle,
                                 BleGattWriteCallback callback,
                                 void* context = nullptr);
  void setCustomGattWriteCallback(BleGattWriteCallback callback,
                                  void* context = nullptr);
  bool isConnected() const;
  bool isConnectionEncrypted() const;
  bool getConnectionInfo(BleConnectionInfo* info) const;
  bool scanCycle(BleScanPacket* packet, uint32_t perChannelSpinLimit = 300000UL);
  bool scanActiveCycle(BleActiveScanResult* result,
                       uint32_t perChannelAdvListenSpinLimit = 300000UL,
                       uint32_t scanRspListenSpinLimit = 300000UL);
  bool advertiseEvent(uint32_t interChannelDelayUs = 350U,
                      uint32_t spinLimit = 600000UL);
  bool advertiseInteractEvent(BleAdvInteraction* interaction,
                              uint32_t interChannelDelayUs = 350U,
                              uint32_t requestListenSpinLimit = 250000UL,
                              uint32_t spinLimit = 700000UL);
  bool pollConnectionEvent(BleConnectionEvent* event,
                           uint32_t spinLimit = 450000UL);

 private:
  static constexpr uint16_t kCustomGattHandleStart = 0x0020U;
  static constexpr uint16_t kCustomGattHandleEnd = 0x00FFU;

  struct BleCustomServiceState {
    uint16_t serviceHandle;
    uint16_t endHandle;
    uint8_t firstCharacteristicIndex;
    uint8_t characteristicCount;
    bool registered;
    char uuidString[37];
    BLEService* service;
  };

  struct BleCustomCharacteristicState {
    BleRadio* owner;
    uint16_t serviceHandle;
    uint8_t properties;
    uint16_t declarationHandle;
    uint16_t valueHandle;
    uint16_t cccdHandle;
    uint8_t valueLength;
    uint8_t value[kCustomGattMaxValueLength];
    char uuidString[37];
    BleGattWriteCallback writeHandler;
    void* writeContext;
    BLECharacteristic* characteristic;
  };

  static void onCustomGattCharacteristicWritten(BLECharacteristic& characteristic);
  bool ensureCustomGattRegistered();
  bool finalizePendingCustomGattService();
  BleCustomServiceState* findCustomGattService(uint16_t serviceHandle);
  const BleCustomServiceState* findCustomGattService(uint16_t serviceHandle) const;
  BleCustomCharacteristicState* findCustomGattCharacteristic(uint16_t valueHandle);
  const BleCustomCharacteristicState* findCustomGattCharacteristic(
      uint16_t valueHandle) const;
  void handleCustomGattWrite(BleCustomCharacteristicState* characteristicState,
                             bool withResponse);
  bool ensureAdvertisingIdentity();
  bool ensureBatteryService();
  const char* effectiveLocalName() const;

  uint32_t radioBase_;
  uint32_t ficrBase_;
  int8_t txPowerDbm_;
  BleAdvPduType pduType_;
  BleAddressType addressType_;
  bool includeAdvertisingFlags_;
  bool useChSel2_;
  bool addressConfigured_;
  bool advertisingIdentityDirty_;
  bool customAdvertisingIdentity_;
  uint8_t batteryLevel_;
  uint8_t advertisingIdentityId_;
  bool initialized_;
  uint8_t address_[6];
  uint8_t scanCycleStartIndex_;
  uint8_t passiveScanPayload_[31];
  uint8_t advertisingData_[31];
  uint8_t scanResponseData_[31];
  size_t advertisingDataLen_;
  size_t scanResponseDataLen_;
  char advertisingName_[32];
  char scanResponseName_[32];
  char gattDeviceName_[32];
  void* batteryService_;
  void* batteryLevelCharacteristic_;
  bool batteryServiceAdded_;
  uint16_t nextCustomGattHandle_;
  int8_t pendingCustomGattServiceIndex_;
  uint8_t customGattServiceCount_;
  uint8_t customGattCharacteristicCount_;
  BleGattWriteCallback customGattWriteCallback_;
  void* customGattWriteContext_;
  BleCustomServiceState customGattServices_[kCustomGattMaxServices];
  BleCustomCharacteristicState
      customGattCharacteristics_[kCustomGattMaxCharacteristics];
};

}  // namespace xiao_nrf54l15
