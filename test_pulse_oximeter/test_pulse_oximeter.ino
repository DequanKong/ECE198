#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE] = {0}; // init to zero
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute = 0;
int beatAvg = 0;

void setup() {
  Serial.begin(9600);                  // ✅ start serial so you can see prints
  delay(50);

  Wire.begin();
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX3010x not found. Check wiring/power.");
    while (1) { delay(10); }
  }

  // Basic HR settings; adjust as needed for your sensor/placement
  particleSensor.setup();                // default config is OK to start
  particleSensor.setPulseAmplitudeRed(0x0A);   // Red LED on (low)
  particleSensor.setPulseAmplitudeIR(0x0A);    // IR LED on (low)
  particleSensor.setPulseAmplitudeGreen(0);    // Green off (not needed)
}

void loop() {
  long irValue = particleSensor.getIR(); // ✅ read IR level

  // Optional: simple "finger present" check; tune this threshold for your setup
  bool fingerPresent = irValue > 50000;

  if (fingerPresent && checkForBeat(irValue)) {   // ✅ only when a beat is detected
    long delta = millis() - lastBeat;             // ms between beats
    lastBeat = millis();

    if (delta > 0) {
      beatsPerMinute = 60.0 / (delta / 1000.0);   // BPM
    }

    if (beatsPerMinute > 20 && beatsPerMinute < 255) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      int sum = 0;
      for (byte i = 0; i < RATE_SIZE; i++) sum += rates[i];
      beatAvg = sum / RATE_SIZE;
    }

    // Optional: beep here if you wire a buzzer
  }

  // Basic status prints (adjust as you like)
  Serial.print("IR: "); Serial.print(irValue);
  Serial.print("  BPM: "); Serial.print(beatsPerMinute, 1);
  Serial.print("  Avg: "); Serial.println(beatAvg);

  // If no finger detected for a while, you can zero the average:
  // if (!fingerPresent) beatAvg = 0;

  delay(20);  // small delay; avoid big delays that distort beat timing
}
