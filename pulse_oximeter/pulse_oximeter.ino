#include <DFRobot_MAX30102.h>
#include <Wire.h>
// Create an object from the DFRobot_MAX30102 class
DFRobot_MAX30102 particleSensor; // Allows us to call methods

// Put your setup code here, to run once:
void setup() {
  Serial.begin(115200); // Initialize Serial Monitor at 115200 baud
  Wire.begin(); // Enable I2C (SDA: A4, SCL: A5)
// If sensor fails to initialize, print an error message
  while (!particleSensor.begin()) { //
  Serial.println("MAX30102 not found!");
  delay(1000); // 2 sec delay before printing the message again
  }
// Configuration: (Led Brightness (0-255), Sample Average, LED Mode
// Sample Rate (Hz), Pulse Width (LED pulse width), ADC Range)
  particleSensor.sensorConfiguration(60, SAMPLEAVG_8, MODE_MULTILED,
  SAMPLERATE_400, PULSEWIDTH_411,
  ADCRANGE_16384);
}
// Put your main code here, to run repeatedly:
void loop() {
  int IRvalue = particleSensor.getIR();
  Serial.println(IRvalue); // Print the IR values to Serial Monitor
}
