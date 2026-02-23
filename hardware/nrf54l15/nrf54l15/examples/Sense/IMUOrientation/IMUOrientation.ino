/*
  XIAO nRF54L15 Sense IMU orientation sample.
*/

#include "SenseImuBackend.h"

void setup()
{
  Serial.begin(115200);
  delay(1200);
  Serial.println("XIAO nRF54L15 Sense IMUOrientation");
  senseImuSetup();
}

void loop()
{
  senseImuLoop();
  delay(500);
}
