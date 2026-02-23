/*
  XIAO nRF54L15 battery monitor sample.

  XIAO nRF54L15 routes battery monitor through:
    - VBAT_EN: P1.15 (must be HIGH)
    - ADC input: A7 (AIN7 on P1.14)

  Override BATTERY_ADC_PIN and divider ratio if your hardware differs.
*/

#include <XiaoNrf54L15.h>

#ifndef BATTERY_ADC_PIN
#define BATTERY_ADC_PIN A7
#endif

#ifndef BATTERY_DIVIDER_NUM
#define BATTERY_DIVIDER_NUM 2UL
#endif

#ifndef BATTERY_DIVIDER_DEN
#define BATTERY_DIVIDER_DEN 1UL
#endif

#ifndef ADC_REF_MV
#define ADC_REF_MV 3300UL
#endif

static void printMilliVolts(uint32_t mv) {
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

void setup() {
  Serial.begin(115200);
  delay(1000);
  (void)XiaoNrf54L15.setBatteryMeasurementEnabled(true);
  delay(5);
  analogReadResolution(12);
  Serial.println("XIAO nRF54L15 battery monitor sample");
}

void loop() {
  int raw = 0;
  if (BATTERY_ADC_PIN == A7) {
    raw = XiaoNrf54L15.readBatteryRaw(4);
  } else {
    raw = analogRead(BATTERY_ADC_PIN);
  }

  const uint32_t pinMv = (static_cast<uint32_t>(raw) * ADC_REF_MV + 2047UL) / 4095UL;
  const uint32_t batteryMv = (pinMv * BATTERY_DIVIDER_NUM + (BATTERY_DIVIDER_DEN / 2UL)) /
                             BATTERY_DIVIDER_DEN;

  Serial.print("ADC raw=");
  Serial.print(raw);
  Serial.print("  pin=");
  printMilliVolts(pinMv);
  Serial.print(" V  battery~=");
  printMilliVolts(batteryMv);
  Serial.println(" V");
  delay(1000);
}
