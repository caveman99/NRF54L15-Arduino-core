#include <Arduino.h>

#include "SenseImuBackend.h"

#if defined(__has_include)
#if __has_include(<generated/zephyr/autoconf.h>)
#include <generated/zephyr/autoconf.h>
#endif
#if __has_include(<zephyr/device.h>) && __has_include(<zephyr/devicetree.h>) && __has_include(<zephyr/drivers/sensor.h>)
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#define XIAO_SENSE_HAS_IMU_API 1
#else
#define XIAO_SENSE_HAS_IMU_API 0
#endif
#else
#define XIAO_SENSE_HAS_IMU_API 0
#endif

#if XIAO_SENSE_HAS_IMU_API
#if DT_NODE_EXISTS(DT_ALIAS(imu0))
#define XIAO_SENSE_IMU_NODE_PRESENT 1
static const struct device *const g_imu = DEVICE_DT_GET(DT_ALIAS(imu0));
#else
#define XIAO_SENSE_IMU_NODE_PRESENT 0
#endif
#else
#define XIAO_SENSE_IMU_NODE_PRESENT 0
#endif

namespace {
bool g_imuReady = false;
bool g_missingMessagePrinted = false;

#if XIAO_SENSE_HAS_IMU_API && XIAO_SENSE_IMU_NODE_PRESENT
float sensorValueToFloat(const struct sensor_value &value)
{
  return static_cast<float>(value.val1) + (static_cast<float>(value.val2) / 1000000.0f);
}

bool readSensorTriplet(
  const struct device *device,
  sensor_channel chanXyz,
  sensor_channel chanX,
  sensor_channel chanY,
  sensor_channel chanZ,
  float &x,
  float &y,
  float &z)
{
  int ret = sensor_sample_fetch_chan(device, chanXyz);
  if (ret < 0) {
    Serial.print("sensor_sample_fetch_chan failed: ");
    Serial.println(ret);
    return false;
  }

  struct sensor_value rawX;
  struct sensor_value rawY;
  struct sensor_value rawZ;

  ret = sensor_channel_get(device, chanX, &rawX);
  if (ret < 0) {
    Serial.print("sensor_channel_get(X) failed: ");
    Serial.println(ret);
    return false;
  }

  ret = sensor_channel_get(device, chanY, &rawY);
  if (ret < 0) {
    Serial.print("sensor_channel_get(Y) failed: ");
    Serial.println(ret);
    return false;
  }

  ret = sensor_channel_get(device, chanZ, &rawZ);
  if (ret < 0) {
    Serial.print("sensor_channel_get(Z) failed: ");
    Serial.println(ret);
    return false;
  }

  x = sensorValueToFloat(rawX);
  y = sensorValueToFloat(rawY);
  z = sensorValueToFloat(rawZ);
  return true;
}
#endif

void printMissingSupportOnce(const char *message)
{
  if (g_missingMessagePrinted) {
    return;
  }
  g_missingMessagePrinted = true;
  Serial.println(message);
}
}  // namespace

void senseImuSetup()
{
#if !XIAO_SENSE_HAS_IMU_API
  printMissingSupportOnce("IMU support unavailable: missing Zephyr sensor headers.");
#elif !XIAO_SENSE_IMU_NODE_PRESENT
  printMissingSupportOnce("IMU support unavailable: devicetree alias imu0 is missing.");
#else
  if (!device_is_ready(g_imu)) {
    printMissingSupportOnce("IMU device is not ready.");
    return;
  }

  struct sensor_value odr;
  odr.val1 = 12;
  odr.val2 = 500000;

  int ret = sensor_attr_set(g_imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
  if (ret < 0) {
    Serial.print("Failed to set accelerometer ODR: ");
    Serial.println(ret);
  }

  ret = sensor_attr_set(g_imu, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
  if (ret < 0) {
    Serial.print("Failed to set gyroscope ODR: ");
    Serial.println(ret);
  }

  g_imuReady = true;
  Serial.println("IMU ready (LSM6DS3TR-C, imu0).");
#endif
}

void senseImuLoop()
{
#if XIAO_SENSE_HAS_IMU_API && XIAO_SENSE_IMU_NODE_PRESENT
  if (!g_imuReady) {
    delay(1000);
    return;
  }

  float accelX = 0.0f;
  float accelY = 0.0f;
  float accelZ = 0.0f;
  if (!readSensorTriplet(
        g_imu,
        SENSOR_CHAN_ACCEL_XYZ,
        SENSOR_CHAN_ACCEL_X,
        SENSOR_CHAN_ACCEL_Y,
        SENSOR_CHAN_ACCEL_Z,
        accelX,
        accelY,
        accelZ)) {
    delay(300);
    return;
  }

  float gyroX = 0.0f;
  float gyroY = 0.0f;
  float gyroZ = 0.0f;
  if (!readSensorTriplet(
        g_imu,
        SENSOR_CHAN_GYRO_XYZ,
        SENSOR_CHAN_GYRO_X,
        SENSOR_CHAN_GYRO_Y,
        SENSOR_CHAN_GYRO_Z,
        gyroX,
        gyroY,
        gyroZ)) {
    delay(300);
    return;
  }

  Serial.print("accel[m/s^2] ");
  Serial.print(accelX, 3);
  Serial.print(", ");
  Serial.print(accelY, 3);
  Serial.print(", ");
  Serial.print(accelZ, 3);
  Serial.print(" | gyro[rad/s] ");
  Serial.print(gyroX, 3);
  Serial.print(", ");
  Serial.print(gyroY, 3);
  Serial.print(", ");
  Serial.println(gyroZ, 3);
#else
  printMissingSupportOnce("Sense IMU support is unavailable for this build.");
  delay(1000);
#endif
}
