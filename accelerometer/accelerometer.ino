// Accelerometer.ino
// Accelerometer.ino
// Accelerometer constants
const int zPin = A3;
const int xPin = A0;
const int yPin = A1;
const int SAMPLES = 10;
const float RestVoltage = 1.641; // Voltage at 0 gravity (g) at 3.3V
const float sensitivity = 0.3; // Sensitivity in V/gravity (g) (1 g = 9.81
ms2)
float zSmoothedVoltage = RestVoltage;
double AlphaAcc = 0.5;
//----------------
// Supporting Functions
// read and average the voltage measured from accelerometer. Averaging is done
// to remove the noise
float readAveragedVoltage(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(500);
  }
//10-bit value converted to voltage between 0 and 5 volts
  return (sum / (float)samples) * (5.0 / 1023.0);
}
//-----
// smooth out measured value for plotting
float smooth(float newVal, float prevVal, double alpha) {
  return alpha * prevVal + (1 - alpha) * newVal;
// return newVal;
}
// Main Functions ... starts with setup() then does loop()
void setup() {
  Serial.begin(9600);
  delay(1000);
}
// main functional block
void loop() {
  double zVoltage;
  float g_z; //acceleration in gravities
  float g_z_smoothed;
  float accelZ; //acceleration in ms^2
  float accelZ_smoothed;
// accelerometer data
  zVoltage = readAveragedVoltage(zPin, SAMPLES);
  zSmoothedVoltage = smooth(zVoltage, zSmoothedVoltage, AlphaAcc);
  g_z = (zVoltage - RestVoltage) / sensitivity;
  g_z_smoothed = (zSmoothedVoltage - RestVoltage) / sensitivity;
  accelZ = g_z * 9.81; //convert gravities to ms^2
  accelZ_smoothed = g_z_smoothed * 9.81;

  Serial.print(accelZ);
  Serial.print(",");
  Serial.println(accelZ_smoothed);
  delay(500); // Delay for readability
}