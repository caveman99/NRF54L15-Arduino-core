/*
  XIAO nRF54L15 battery measurement example.

  Board schematic path:
    - VBAT_EN -> P1.15 (must be HIGH while sampling)
    - BAT_V   -> P1.14 (A7 / ADC channel 7)
    - BAT_V is a /2 divider from the battery rail.
*/

#include <XiaoNrf54L15.h>

static const uint8_t kSampleCount = 8;
static const uint32_t kSettleDelayMs = 5;
static const uint32_t kAdcRefMv = 3300UL;
static const uint32_t kAdcMax = 4095UL;
static const uint32_t kDividerNumerator = 2UL;

static void printMilliVolts(uint32_t mv)
{
  Serial.print(mv / 1000UL);
  Serial.print('.');
  uint32_t frac = mv % 1000UL;
  if (frac < 100UL) {
    Serial.print('0');
  }
  if (frac < 10UL) {
    Serial.print('0');
  }
  Serial.print(frac);
}

void setup()
{
  Serial.begin(115200);
  delay(1200);
  analogReadResolution(12);
  (void)XiaoNrf54L15.setBatteryMeasurementEnabled(false);
  Serial.println("XIAO nRF54L15 BatteryMeasure");
}

void loop()
{
  if (!XiaoNrf54L15.setBatteryMeasurementEnabled(true)) {
    Serial.println("VBAT_EN control failed");
    delay(1000);
    return;
  }

  delay(kSettleDelayMs);
  const int raw = XiaoNrf54L15.readBatteryRaw(kSampleCount);
  const uint32_t pinMv = (static_cast<uint32_t>(raw) * kAdcRefMv + (kAdcMax / 2UL)) / kAdcMax;
  const uint32_t batteryMv = pinMv * kDividerNumerator;
  (void)XiaoNrf54L15.setBatteryMeasurementEnabled(false);

  Serial.print("raw=");
  Serial.print(raw);
  Serial.print("  BAT_V=");
  printMilliVolts(pinMv);
  Serial.print(" V  VBAT~=");
  printMilliVolts(batteryMv);
  Serial.println(" V");
  delay(1500);
}
