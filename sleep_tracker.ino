#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;

long last_beat = 0;
float beats_per_minute = 0;
int beat_avg = 0;

const int zPin = A3;
const int xPin = A0;
const int yPin = A1;
const int SAMPLES = 10;
const float RestVoltage = 1.641; // Voltage at 0 gravity (g) at 3.3V
const float sensitivity = 0.3; // Sensitivity in V/gravity (g) (1 g = 9.81
double zVoltage, xVoltage, yVoltage;
float g_z; //acceleration in gravities
float g_z_smoothed;
float accelZ; //acceleration in ms^2
float accelZ_smoothed;
float diffZ, diffZ_smoothed;
float zSmoothedVoltage = RestVoltage;

float g_x, g_x_smoothed, accelX, accelX_smoothed, diffX, diffX_smoothed;
float xSmoothedVoltage = RestVoltage;

float g_y, g_y_smoothed, accelY, accelY_smoothed, diffY, diffY_smoothed;
float ySmoothedVoltage = RestVoltage;
double AlphaAcc = 0.5;

bool calibrating = true;
long calibration_start = 0;
long calibration_heart_rate_sum = 0;
long normal_heart_rate = 0;
const int CALIBRATION_SECONDS = 30;
const int DELAY_TIME = 20; 
const int button_pin = 0;
int button_pressed = 0;

bool sleeping = false; 
long sleep_start = 0;
const int CHECK_INTERVAL = 10; 
double acc_sum = 0;
long test_heart_rate_sum = 0;
long last_check = 0;
long current_heart_rate = 0;
long current_avg_acc = 0;
double acc_mag = 0; 

const double SLEEP_HEART_RATE_RATIO = 0.8;
const double SLEEP_ACC = 0.5;
const double WAKE_HEART_RATE_RATIO = 1.1;
const double WAKE_ACC = 1;

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

void setup() {
  Serial.begin(9600);                 
  delay(50);

  Wire.begin();
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX3010x not found. Check wiring/power.");
    while (1) { delay(10); }
  }

  particleSensor.setup();                
  particleSensor.setPulseAmplitudeRed(0x0A);   // Red LED on
  particleSensor.setPulseAmplitudeIR(0x0A);    // IR LED on 
  particleSensor.setPulseAmplitudeGreen(0);    // Green off
}

void loop() {
  long ir_value = particleSensor.getIR(); 

  // Check if there is actually a finger on the sensor 
  bool finger_present = ir_value > 5000;

  // When a beat is detected
  if (finger_present && checkForBeat(ir_value)) { 
    long delta = millis() - last_beat;             // ms between beats
    last_beat = millis();
    if (delta > 0) {
      beats_per_minute = 60.0 / (delta / 1000.0);   // BPM
    }
  }
  if (calibrating){
    beats_per_minute = max(min(beats_per_minute, 255), 40); // Correct values if they are obviously nonsensical
    calibration_heart_rate_sum += beats_per_minute;
    if (millis() - calibration_start >= CALIBRATION_SECONDS*1000){
      normal_heart_rate = calibration_heart_rate_sum/(CALIBRATION_SECONDS*1000/DELAY_TIME);
      calibration_heart_rate_sum = 0; 
      calibrating = false;
      Serial.print("Their normal heart rate is: ");
      Serial.println(normal_heart_rate;)
    }
  }
  else{
    // accelerometer data
    zVoltage = readAveragedVoltage(zPin, SAMPLES);
    zSmoothedVoltage = smooth(zVoltage, zSmoothedVoltage, AlphaAcc);
    g_z = (zVoltage - RestVoltage) / sensitivity;
    g_z_smoothed = (zSmoothedVoltage - RestVoltage) / sensitivity;
    // Serial.print(g_z_smoothed*9.81);
    // Serial.print(",");
    diffZ = g_z*9.81 - accelZ;
    diffZ_smoothed = g_z_smoothed * 9.81 - accelZ_smoothed;
    accelZ = g_z * 9.81; //convert gravities to ms^2
    accelZ_smoothed = g_z_smoothed * 9.81;


    xVoltage = readAveragedVoltage(xPin, SAMPLES);
    xSmoothedVoltage = smooth(xVoltage, xSmoothedVoltage, AlphaAcc);
    g_x = (xVoltage - RestVoltage) / sensitivity;
    g_x_smoothed = (xSmoothedVoltage - RestVoltage) / sensitivity;
    diffX = g_x*9.81 - accelX;
    diffX_smoothed = g_x_smoothed * 9.81 - accelX_smoothed;
    accelX = g_x * 9.81; //convert gravities to ms^2
    accelX_smoothed = g_x_smoothed * 9.81;

    yVoltage = readAveragedVoltage(yPin, SAMPLES);
    ySmoothedVoltage = smooth(yVoltage, ySmoothedVoltage, AlphaAcc);
    g_y = (yVoltage - RestVoltage) / sensitivity;
    g_y_smoothed = (ySmoothedVoltage - RestVoltage) / sensitivity;
    diffY = g_y*9.81 - accelY;
    diffY_smoothed = g_y_smoothed * 9.81 - accelY_smoothed;
    accelY = g_y * 9.81; //convert gravities to ms^2
    accelY_smoothed = g_y_smoothed * 9.81;

    acc_mag = sqrt(diffX^2 + diffY^2 + diffZ^2);
    test_heart_rate_sum += beats_per_minute;
    if (millis() - last_check >= CHECK_INTERVAL*1000){
      current_heart_rate = test_heart_rate_sum/(CHECK_INTERVAL*1000/DELAY_TIME);
      test_heart_rate_sum = 0;
      last_check = millis();
    }
    acc_sum += acc_mag;
    if (millis() - last_check >= CHECK_INTERVAL*1000){
      current_avg_acc = acc_sum/(CHECK_INTERVAL*1000/DELAY_TIME);
      acc_sum = 0;
      last_check = millis();
    }

    if (!sleeping){
      if (current_heart_rate < SLEEP_HEART_RATE_RATIO*normal_heart_rate && curr_avg_acc < SLEEP_ACC){
        sleeping = true;
        sleep_start = millis();
      }
    }
    else{
      if (curr_heart_rate > WAKE_HEART_RATE_RATIO*normal_heart_rate && curr_avg_acc > WAKE_ACC){
        sleeping = false;
        Serial.print("They have been sleeping for ");
        Serial.println((millis() - sleep_start)/(3600*1000));
      }
    }
  }

  // Basic status prints (adjust as you like)
  Serial.print("IR: "); Serial.print(ir_value);
  Serial.print("  BPM: "); Serial.print(beats_per_minute, 1);
  Serial.print("  Avg: "); Serial.println(beat_avg);

  // If no finger detected for a while, you can zero the average:
  // if (!finger_present) beat_avg = 0;

  //rising edge
  if (digitalRead(button_pin) == 1 && button_pressed == 0){
    if (calibrating){
      normal_heart_rate = calibration_heart_rate_sum/((millis() - calibration_start)/DELAY_TIME);
      Serial.print("Their normal heart rate is: ");
      Serial.println(normal_heart_rate;)
    }
    else{
      sleeping = false;
      Serial.print("They have been sleeping for ");
      Serial.println((millis() - sleep_start)/(3600*1000));
    }
    calibrating ^= 1; //flip calibration
    calibration_heart_rate_sum = 0;
    test_heart_rate_sum = 0;
    acc_sum = 0;
  
  }
  button_pressed = digitalRead(button_pin);
  

  delay(20);  // small delay; avoid big delays that distort beat timing
}