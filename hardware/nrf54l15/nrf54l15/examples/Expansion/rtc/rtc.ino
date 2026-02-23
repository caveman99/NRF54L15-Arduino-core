#include <Arduino.h>
#include <Wire.h>

/*
  Expansion-base RTC sample for DS3231/DS1307-like I2C RTC (default 0x68).
*/

#ifndef RTC_ADDR
#define RTC_ADDR 0x68
#endif

struct RtcDateTime {
  uint8_t sec;
  uint8_t min;
  uint8_t hour;
  uint8_t day;
  uint8_t month;
  uint16_t year;
};

static uint8_t bcdToDec(uint8_t v) {
  return static_cast<uint8_t>((v >> 4) * 10 + (v & 0x0F));
}

static uint8_t decToBcd(uint8_t v) {
  return static_cast<uint8_t>(((v / 10U) << 4) | (v % 10U));
}

static bool readRtcRegs(uint8_t startReg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(RTC_ADDR, len, true) != len) {
    return false;
  }

  for (uint8_t i = 0; i < len; ++i) {
    buf[i] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

static bool writeRtcRegs(uint8_t startReg, const uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(startReg);
  for (uint8_t i = 0; i < len; ++i) {
    Wire.write(buf[i]);
  }
  return Wire.endTransmission() == 0;
}

static uint8_t buildMonth() {
  const char *m = __DATE__;
  if (m[0] == 'J' && m[1] == 'a') return 1;
  if (m[0] == 'F') return 2;
  if (m[0] == 'M' && m[2] == 'r') return 3;
  if (m[0] == 'A' && m[1] == 'p') return 4;
  if (m[0] == 'M' && m[2] == 'y') return 5;
  if (m[0] == 'J' && m[2] == 'n') return 6;
  if (m[0] == 'J' && m[2] == 'l') return 7;
  if (m[0] == 'A' && m[1] == 'u') return 8;
  if (m[0] == 'S') return 9;
  if (m[0] == 'O') return 10;
  if (m[0] == 'N') return 11;
  return 12;
}

static RtcDateTime buildTimeDate() {
  RtcDateTime dt{};
  dt.month = buildMonth();
  dt.day = static_cast<uint8_t>((__DATE__[4] == ' ' ? 0 : (__DATE__[4] - '0')) * 10 + (__DATE__[5] - '0'));
  dt.year = static_cast<uint16_t>((__DATE__[7] - '0') * 1000 + (__DATE__[8] - '0') * 100 +
                                  (__DATE__[9] - '0') * 10 + (__DATE__[10] - '0'));
  dt.hour = static_cast<uint8_t>((__TIME__[0] - '0') * 10 + (__TIME__[1] - '0'));
  dt.min = static_cast<uint8_t>((__TIME__[3] - '0') * 10 + (__TIME__[4] - '0'));
  dt.sec = static_cast<uint8_t>((__TIME__[6] - '0') * 10 + (__TIME__[7] - '0'));
  return dt;
}

static bool readRtcDateTime(RtcDateTime *out) {
  uint8_t r[7];
  if (!readRtcRegs(0x00, r, sizeof(r))) {
    return false;
  }

  const uint8_t sec = bcdToDec(r[0] & 0x7F);
  const uint8_t min = bcdToDec(r[1] & 0x7F);
  uint8_t hour = 0;
  if ((r[2] & 0x40U) != 0U) {
    // 12-hour mode.
    const uint8_t hour12 = bcdToDec(r[2] & 0x1FU);
    const bool pm = (r[2] & 0x20U) != 0U;
    hour = static_cast<uint8_t>((hour12 % 12U) + (pm ? 12U : 0U));
  } else {
    hour = bcdToDec(r[2] & 0x3FU);
  }

  const uint8_t day = bcdToDec(r[4] & 0x3F);
  const uint8_t month = bcdToDec(r[5] & 0x1F);
  const uint16_t year = static_cast<uint16_t>(2000U + bcdToDec(r[6]) + ((r[5] & 0x80U) ? 100U : 0U));

  out->sec = sec;
  out->min = min;
  out->hour = hour;
  out->day = day;
  out->month = month;
  out->year = year;
  return true;
}

static bool setRtcDateTime(const RtcDateTime &dt) {
  if (dt.year < 2000U || dt.year > 2199U) {
    return false;
  }

  uint8_t regs[7];
  regs[0] = decToBcd(dt.sec % 60U);
  regs[1] = decToBcd(dt.min % 60U);
  regs[2] = decToBcd(dt.hour % 24U); // force 24-hour mode
  regs[3] = 1; // day-of-week placeholder (1..7)
  regs[4] = decToBcd(dt.day == 0 ? 1 : dt.day);
  const uint16_t yearOffset = static_cast<uint16_t>(dt.year - 2000U);
  regs[5] = decToBcd(dt.month == 0 ? 1 : dt.month);
  if (yearOffset >= 100U) {
    regs[5] |= 0x80U; // century bit for years >= 2100
  }
  regs[6] = decToBcd(static_cast<uint8_t>(yearOffset % 100U));
  return writeRtcRegs(0x00, regs, sizeof(regs));
}

static bool clearOscillatorStopFlag() {
  uint8_t status = 0;
  if (!readRtcRegs(0x0F, &status, 1)) {
    return false;
  }
  status &= static_cast<uint8_t>(~0x80U);
  return writeRtcRegs(0x0F, &status, 1);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin();
  Wire.setClock(100000U);

  Wire.beginTransmission(RTC_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("RTC not found at 0x68");
    return;
  }

  Serial.println("XIAO Expanded RTC sample");

  RtcDateTime current{};
  bool validNow = readRtcDateTime(&current);
  bool needsInit = !validNow ||
                   (current.year < 2024U || current.month == 0U || current.month > 12U ||
                    current.day == 0U || current.day > 31U || current.hour > 23U ||
                    current.min > 59U || current.sec > 59U);

  uint8_t status = 0;
  if (readRtcRegs(0x0F, &status, 1) && ((status & 0x80U) != 0U)) {
    needsInit = true;
  }

  if (needsInit) {
    const RtcDateTime buildDt = buildTimeDate();
    if (setRtcDateTime(buildDt)) {
      (void)clearOscillatorStopFlag();
      Serial.println("RTC time initialized from sketch build time.");
    } else {
      Serial.println("RTC initialization failed.");
    }
  }
}

void loop() {
  RtcDateTime dt{};
  if (!readRtcDateTime(&dt)) {
    Serial.println("RTC read failed");
    delay(1000);
    return;
  }

  if (dt.year < 1000U) {
    Serial.print('0');
  }
  if (dt.year < 100U) {
    Serial.print('0');
  }
  if (dt.year < 10U) {
    Serial.print('0');
  }
  Serial.print(dt.year);
  Serial.print('-');
  if (dt.month < 10U) {
    Serial.print('0');
  }
  Serial.print(dt.month);
  Serial.print('-');
  if (dt.day < 10U) {
    Serial.print('0');
  }
  Serial.print(dt.day);
  Serial.print(' ');
  if (dt.hour < 10U) {
    Serial.print('0');
  }
  Serial.print(dt.hour);
  Serial.print(':');
  if (dt.min < 10U) {
    Serial.print('0');
  }
  Serial.print(dt.min);
  Serial.print(':');
  if (dt.sec < 10U) {
    Serial.print('0');
  }
  Serial.println(dt.sec);

  delay(1000);
}
