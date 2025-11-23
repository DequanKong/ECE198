#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;

// heart rate variables
long last_beat = 0;
float beats_per_minute = 0;
int beat_avg = 0;

// Accelerometer variables
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

//Calibration variables
const int CALIBRATION_SECONDS = 15;
const int DELAY_TIME = 20; 
const int button_pin = 2;
bool calibrating = true;
long calibration_start = 0;
double calibration_heart_rate_sum = 0;
double normal_heart_rate = 0;
int button_pressed = 0;
long num_of_calibration_checks = 0;

//sleep state variables
bool sleeping = false; 
long sleep_start = 0;
double acc_sum = 0;
double test_heart_rate_sum = 0;
long last_check = 0;
double current_heart_rate = 0;
double current_avg_acc = 0;
double acc_mag = 0; 
long test_num_heart_rate_checks = 0;
long test_num_acc_checks = 0;

// Constants impacting sleep check
const int CHECK_INTERVAL = 10; // Checks once in a ten second interval
const double SLEEP_HEART_RATE_RATIO = 0.9;
const double SLEEP_ACC = 0.2;
const double WAKE_HEART_RATE_RATIO = 1.1;
const double WAKE_ACC = 0.5;

// read and average samples number of accelerometer inputs
// Averaging is done to remove noise
float readAveragedVoltage(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(500);
  }
  //convert 10-bit num`to a voltage between 0 and 5 volts
  return (sum / (float)samples) * (5.0 / 1023.0);
}

// smooth the values to prevent abrupt changes
float smooth(float newVal, float prevVal, double alpha) {
  return alpha * prevVal + (1 - alpha) * newVal;
}
// Initializes the system
void setup() {
  Serial.begin(9600);                 
  delay(50);

  Wire.begin();
  // prevents program from running if the sensor is not found
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX3010x not found. Check wiring/power.");
    while (1) { delay(10); }
  }
  // initializes sensor
  particleSensor.setup();                
  particleSensor.setPulseAmplitudeRed(0x0A);   // Red LED on
  particleSensor.setPulseAmplitudeIR(0x0A);    // IR LED on 
  particleSensor.setPulseAmplitudeGreen(0);    // Green off
}

// main loop of the code, where recurring events take place
void loop() {

  // calibration state 
  if (calibrating){
    long ir_value = particleSensor.getIR(); 

    // Check if there is actually a finger on the sensor 
    bool finger_present = ir_value > 5000;

    // When a beat is detected
    if (finger_present && checkForBeat(ir_value)) { 
      long delta = millis() - last_beat; // ms between beats
      last_beat = millis(); // update beat time
      // if actually a new beat, check to ensure something hasn't gone wrong
      if (delta > 0) {
        beats_per_minute = 60.0 / (delta / 1000.0); // BPM
      }
      beats_per_minute = max(min(beats_per_minute, 255), 40); // Correct values if they are obviously nonsensical
      // Adds BPM value to a sum so we can calculate the average later
      calibration_heart_rate_sum += beats_per_minute;
      num_of_calibration_checks++;
    }


    // If the calibration is finished
    if (millis() - calibration_start >= CALIBRATION_SECONDS*1000){
      // We find the number of samples within the time interval
      // and divide the total by that amount
      // to find the average heart rate

      // handle possible division by 0
      if(num_of_calibration_checks != 0){
        normal_heart_rate = calibration_heart_rate_sum/num_of_calibration_checks;
      }
      else{
        normal_heart_rate = 60;
      }
      // reset variables
      calibration_heart_rate_sum = 0; 
      calibrating = false;
      last_check = millis();
      sleeping = false;
      //temporary, before first measurement
      current_heart_rate = normal_heart_rate;
      current_avg_acc = 0;
      // Display this average heart rate 
      Serial.print("Their normal heart rate is: ");
      Serial.println(normal_heart_rate);
    }
  }
  // not calibrating
  else{
    // accelerometer data
    // get for each of x, y, and z 
    // take difference between consecutive accelerometer data
    // to accommodate for our accelerometer outputting large negative values
    // at certain orientations
    // z data  
    zVoltage = readAveragedVoltage(zPin, SAMPLES);
    zSmoothedVoltage = smooth(zVoltage, zSmoothedVoltage, AlphaAcc);
    g_z = (zVoltage - RestVoltage) / sensitivity;
    g_z_smoothed = (zSmoothedVoltage - RestVoltage) / sensitivity;
    diffZ = g_z*9.81 - accelZ;
    diffZ_smoothed = g_z_smoothed * 9.81 - accelZ_smoothed;
    accelZ = g_z * 9.81; //convert gravities to ms^2
    accelZ_smoothed = g_z_smoothed * 9.81;

    // x data
    xVoltage = readAveragedVoltage(xPin, SAMPLES);
    xSmoothedVoltage = smooth(xVoltage, xSmoothedVoltage, AlphaAcc);
    g_x = (xVoltage - RestVoltage) / sensitivity;
    g_x_smoothed = (xSmoothedVoltage - RestVoltage) / sensitivity;
    diffX = g_x*9.81 - accelX;
    diffX_smoothed = g_x_smoothed * 9.81 - accelX_smoothed;
    accelX = g_x * 9.81; //convert gravities to ms^2
    accelX_smoothed = g_x_smoothed * 9.81;

    // y data
    yVoltage = readAveragedVoltage(yPin, SAMPLES);
    ySmoothedVoltage = smooth(yVoltage, ySmoothedVoltage, AlphaAcc);
    g_y = (yVoltage - RestVoltage) / sensitivity;
    g_y_smoothed = (ySmoothedVoltage - RestVoltage) / sensitivity;
    diffY = g_y*9.81 - accelY;
    diffY_smoothed = g_y_smoothed * 9.81 - accelY_smoothed;
    accelY = g_y * 9.81; //convert gravities to ms^2
    accelY_smoothed = g_y_smoothed * 9.81;

    // find the magnitude of the observed acceleration by taking the norm of the vector
    acc_mag = sqrt(diffX*diffX + diffY*diffY + diffZ*diffZ);
    // we also keep an acceleration sum for the same purpose
    acc_sum += acc_mag;
    test_num_acc_checks++;
    // after we have gathered data for 10 seconds, we use it to get an average data.
    long ir_value = particleSensor.getIR(); 

    // Check if there is actually a finger on the sensor 
    bool finger_present = ir_value > 5000;

    // When a beat is detected
    if (finger_present && checkForBeat(ir_value)) { 
      long delta = millis() - last_beat; // ms between beats
      last_beat = millis(); // update beat time
      // if actually a new beat, check to ensure something hasn't gone wrong
      if (delta > 0) {
        beats_per_minute = 60.0 / (delta / 1000.0); // BPM
      }
      // keep a heart rate sum, which we will use to calculate the average heart rate in the interval
      beats_per_minute = max(min(beats_per_minute, 255), 40); // Correct values if they are obviously nonsensical
      test_heart_rate_sum += beats_per_minute;
      test_num_heart_rate_checks++;
    }
    if (millis() - last_check >= CHECK_INTERVAL*1000){
      // calculate average heart rate on interval
      if (test_num_heart_rate_checks != 0){
        current_heart_rate = test_heart_rate_sum/(test_num_heart_rate_checks);
      }
      else{
        current_heart_rate = 60;
      }
      // calculate average acceleration on the interval
      if (test_num_acc_checks != 0){
        current_avg_acc = acc_sum/test_num_acc_checks;
      }
      else{
        current_avg_acc = 0;
      }
      // reset variables
      test_heart_rate_sum = 0;
      test_num_heart_rate_checks = 0;
      test_num_acc_checks = 0;
      acc_sum = 0;
      last_check = millis();
    }
    // waking state
    if (!sleeping){
      // check conditions for transition to sleep state
      // here it's a lower heart rate and a low amount of movement
      if (current_heart_rate < SLEEP_HEART_RATE_RATIO*normal_heart_rate && current_avg_acc < SLEEP_ACC){
        Serial.println("They have started sleeping.");
        sleeping = true;
        sleep_start = millis();
      }
    }
    // sleeping state
    else{
      // check conditions for waking
      // here it is a higher heart rate and a decent amount of movement
      if (current_heart_rate > WAKE_HEART_RATE_RATIO*normal_heart_rate || current_avg_acc > WAKE_ACC){
        sleeping = false;
        // print out the amount of time they've been sleeping
        Serial.print("They have been sleeping for ");
        Serial.print((millis() - sleep_start)/(3600*1000));
        Serial.println(" hours");
      }
    }
  }

  //rising edge signal for button press
  if (digitalRead(button_pin) == 1 && button_pressed == 0){
    // currently in calibration state
    if (calibrating){
      // calculate heart rate with current data
      if (test_num_heart_rate_checks != 0){
        normal_heart_rate = calibration_heart_rate_sum/(test_num_heart_rate_checks);
      }
      else{
        normal_heart_rate = 60;
      }
      // output calibrated heart rate
      Serial.print("Their normal heart rate is: ");
      Serial.println(normal_heart_rate);
    }
    // not in calibrating state, either in sleeping or waking state 
    else{
      // if in sleeping state, forcibly turn off that state
      // as pushing the calibration button means they're awake 
      if (sleeping){
        sleeping = false;
        Serial.print("They have been sleeping for ");
        Serial.print((millis() - sleep_start)/(3600*1000.0));
        Serial.println(" hours.");
      }
    }
    calibrating ^= 1; //flip calibration
    // reset variables
    calibration_heart_rate_sum = 0;
    test_heart_rate_sum = 0;
    test_num_heart_rate_checks = 0;
    test_num_acc_checks = 0;
    acc_sum = 0;
    last_check = millis();
    calibration_start = millis();
    num_of_calibration_checks = 0;
    //temporary, before first measurement
    current_heart_rate = normal_heart_rate;
    current_avg_acc = 0;

  }
  // read button input
  button_pressed = digitalRead(button_pin);
  

  delay(DELAY_TIME);  // small delay to avoid disrupting heart rate measurement
}