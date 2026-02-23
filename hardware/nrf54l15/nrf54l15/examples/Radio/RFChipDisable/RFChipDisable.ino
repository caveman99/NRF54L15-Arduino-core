/*
  XIAO nRF54L15 RF switch chip control example.

  Schematic control lines:
    - RF_SW_PWR -> P2.03  (powers RF switch chip)
    - RF_SW_SEL -> P2.05  (0 = ceramic, 1 = external U.FL)

  Serial commands:
    0: disable RF switch chip (power OFF)
    1: enable RF switch chip  (power ON)
    c: select ceramic antenna
    e: select external antenna
    s: print status
*/

#include <XiaoNrf54L15.h>

static const uint32_t kStatusIntervalMs = 5000UL;
static uint32_t g_lastStatusMs = 0UL;

static const char *antennaName()
{
  return XiaoNrf54L15.getAntenna() == XiaoNrf54L15Class::EXTERNAL
             ? "external U.FL"
             : "on-board ceramic";
}

static void printStatus()
{
  Serial.print("RF switch power: ");
  Serial.println(XiaoNrf54L15.rfSwitchEnabled() ? "ON" : "OFF");
  Serial.print("Antenna select: ");
  Serial.println(antennaName());
  Serial.print("Radio profile: ");
  Serial.println(XiaoNrf54L15.radioProfileName());
}

static void printMenu()
{
  Serial.println();
  Serial.println("Commands: [0]=rf off [1]=rf on [c]=ceramic [e]=external [s]=status");
}

void setup()
{
  Serial.begin(115200);
  delay(1200);

  (void)XiaoNrf54L15.setRfSwitchEnabled(true);
  (void)XiaoNrf54L15.useCeramicAntenna();

  Serial.println("XIAO nRF54L15 RFChipDisable example");
#if defined(ARDUINO_XIAO_NRF54L15_RADIO_DISABLED)
  Serial.println("Tools radio profile: disabled");
#else
  Serial.println("Tools radio profile: enabled");
#endif
  printStatus();
  printMenu();
}

void loop()
{
  while (Serial.available() > 0) {
    const char cmd = static_cast<char>(Serial.read());
    switch (cmd) {
    case '0':
      (void)XiaoNrf54L15.setRfSwitchEnabled(false);
      printStatus();
      break;
    case '1':
      (void)XiaoNrf54L15.setRfSwitchEnabled(true);
      printStatus();
      break;
    case 'c':
    case 'C':
      (void)XiaoNrf54L15.useCeramicAntenna();
      printStatus();
      break;
    case 'e':
    case 'E':
      (void)XiaoNrf54L15.useExternalAntenna();
      printStatus();
      break;
    case 's':
    case 'S':
      printStatus();
      break;
    case '\r':
    case '\n':
      break;
    default:
      printMenu();
      break;
    }
  }

  const uint32_t now = millis();
  if (now - g_lastStatusMs >= kStatusIntervalMs) {
    g_lastStatusMs = now;
    printStatus();
  }
}
