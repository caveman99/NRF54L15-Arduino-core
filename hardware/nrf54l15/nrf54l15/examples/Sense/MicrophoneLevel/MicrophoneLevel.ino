/*
  XIAO nRF54L15 Sense microphone level sample.
*/

#include "SenseMicBackend.h"

void setup()
{
  Serial.begin(115200);
  delay(1200);
  Serial.println("XIAO nRF54L15 Sense MicrophoneLevel");
  senseMicSetup();
}

void loop()
{
  senseMicLoop();
  delay(100);
}
