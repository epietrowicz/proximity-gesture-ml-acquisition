#include "Particle.h"

#include <Adafruit_SH110X.h>
#include <Adafruit_VCNL4040.h>

#define SAMPLING_FREQ_HZ 50                        // Sampling frequency (Hz)
#define SAMPLING_PERIOD_MS 1000 / SAMPLING_FREQ_HZ // Sampling period (ms)
#define NUM_SAMPLES 100                            // 100 samples at 50 Hz is 2 sec window

#define BUTTON_A D4
#define BUTTON_B D3
#define BUTTON_C D22

Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);
Adafruit_VCNL4040 vcnl4040 = Adafruit_VCNL4040();

float ambientLightMin = 0.0;
float ambientLightMax = 0.0; // dynamically determined based on training environment

float proximityMin = 0.0;
float proximityMax = 1000.0; // experimentally determined

Variant samplesArray; // array of samples to be sent to the cloud
CloudEvent event;

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

// Show system, cloud connectivity, and application logs over USB
// View logs with CLI using 'particle serial monitor --follow'
SerialLogHandler logHandler(LOG_LEVEL_INFO);

void setup()
{
  SystemPowerConfiguration powerConfig = System.getPowerConfiguration();
  powerConfig.auxiliaryPowerControlPin(D23).interruptPin(A6);
  System.setPowerConfiguration(powerConfig);

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  if (!vcnl4040.begin())
  {
    Log.error("Couldn't find VCNL4040 chip");
  }
  vcnl4040.setProximityIntegrationTime(VCNL4040_PROXIMITY_INTEGRATION_TIME_8T);
  vcnl4040.setAmbientIntegrationTime(VCNL4040_AMBIENT_INTEGRATION_TIME_80MS);
  vcnl4040.setProximityLEDCurrent(VCNL4040_LED_CURRENT_120MA);
  vcnl4040.setProximityLEDDutyCycle(VCNL4040_LED_DUTY_1_40);
  // Put initialization like pinMode and begin functions here
  display.begin(0x3C, true); // Address 0x3C default

  // Clear the buffer.
  display.clearDisplay();
  display.display();
  display.setRotation(1);
}
void loop()
{
  unsigned long timestamp;
  unsigned long startTimestamp;
  while (digitalRead(BUTTON_A) == 1)
  {
    uint16_t proximity = vcnl4040.getProximity();
    uint16_t ambientLight = vcnl4040.getAmbientLight();

    if (ambientLight > ambientLightMax)
      ambientLightMax = ambientLight;

    float normProximity =
        proximity < proximityMin   ? 0.0
        : proximity > proximityMax ? 1.0
                                   : (float)(proximity - proximityMin) / (proximityMax - proximityMin);

    float normAmbientLight =
        ambientLight < ambientLightMin   ? 0.0
        : ambientLight > ambientLightMax ? 1.0
                                         : (float)(ambientLight - ambientLightMin) / (ambientLightMax - ambientLightMin);

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.print("Norm Prox: ");
    display.println(normProximity);
    display.print("Norm Light: ");
    display.println(normAmbientLight);
    display.println("------");
    display.print("Proximity: ");
    display.println(proximity);
    display.print("Light: ");
    display.println(ambientLight);
    display.display();
  }

  // Record samples in buffer
  startTimestamp = millis();
  Variant samplesArray;
  for (int i = 0; i < NUM_SAMPLES; i++)
  {
    // Take timestamp so we can hit our target frequency
    timestamp = millis();
    unsigned long timeDelta = timestamp - startTimestamp;

    uint16_t proximity = vcnl4040.getProximity();
    uint16_t ambientLight = vcnl4040.getAmbientLight();

    float normProximity =
        proximity < proximityMin   ? 0.0
        : proximity > proximityMax ? 1.0
                                   : (float)(proximity - proximityMin) / (proximityMax - proximityMin);

    float normAmbientLight =
        ambientLight < ambientLightMin   ? 0.0
        : ambientLight > ambientLightMax ? 1.0
                                         : (float)(ambientLight - ambientLightMin) / (ambientLightMax - ambientLightMin);
    Log.info("Time: %ld, Proximity: %f, ambientLight: %f", timeDelta, normProximity, normAmbientLight);

    Variant entryArray;
    // entryArray.append((int)timeDelta);
    entryArray.append(normProximity);
    entryArray.append(normAmbientLight);
    samplesArray.append(entryArray);
    // Wait just long enough for our sampling period
    while (millis() < timestamp + SAMPLING_PERIOD_MS)
      ;
  }

  // Make sure the button has been released for a few milliseconds
  while (digitalRead(BUTTON_A) == 0)
    ;

  // sessionData.set("samples", samplesArray);
  event.name("samples");
  event.data(samplesArray);
  Particle.publish(event);

  waitForNot(event.isSending, 60000);

  if (event.isSent())
  {
    Log.info("publish succeeded");
    event.clear();
  }
  else if (!event.isOk())
  {
    Log.info("publish failed error=%d", event.error());
    event.clear();
  }
}
