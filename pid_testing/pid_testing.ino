/*
   The value of Vout is obtained by averaging 1024 values reading  the voltage on pin A0. which is connected to the sensor output. 
   The value of temeprature  ta, read by the sensor is shown on the serial monitor.
*/

#include "pidautotuner.h"
#include "PID_v1.h"

// Pin definition
#define TEMPERATURE_PIN A0
#define MOSFET_PIN 11

// Parameter constants
const float TARGET_TEMPERATURE = 13.5f;
const long CONTROL_LOOP_INTERVAL_MICROSEC = 20000; // 20ms or 50Hz control loop
const long PRINT_INTERVAL = 500; // 0.5 second interval

// Calibration constants for temperature sensor, consult datasheet and adjust accordingly
const float CALIB_VOUT0 = 0.64;   // Output voltage in V when sensor is at 0˚C
const float CALIB_TC = 0.01;   // V/°C temperature coefficient for the MCP9701/A

// Variables
PIDAutotuner tuner = PIDAutotuner();
unsigned long time_start = millis();
unsigned long delta_time;
unsigned long previousMillis = 0;
double temperature_initial = 25;
double temperature_err;
double temperature_last = 25;
double temperature_diff;

// MAIN PROGRAM
void setup() {
  analogReference(EXTERNAL);
  Serial.begin(9600);
  pinMode(MOSFET_PIN, OUTPUT);

  temperature_initial = senseTemperature();
  double temperature_target = temperature_initial - TARGET_TEMPERATURE;

  tuner.setTargetInputValue(temperature_target);
  tuner.setLoopInterval(CONTROL_LOOP_INTERVAL_MICROSEC); // in microseconds!!!
  tuner.setOutputRange(0.0f, 12.0f);
  tuner.setZNMode(PIDAutotuner::ZNModeBasicPID);
  delay(3000);
  Serial.println("PID Tuning Started!");
  tuner.startTuningLoop(micros());
  // Run a loop until tuner.isFinished() returns true
  long microseconds;
  while (!tuner.isFinished()) {
      // This loop must run at the same speed as the PID control loop being tuned
      long prevMicroseconds = microseconds;
      microseconds = micros();

      // Get input value here (temperature, encoder position, velocity, etc)
      float input = abs(temperature_initial - senseTemperature());
      // Call tunePID() with the input value and current time in microseconds
      float output = tuner.tunePID(input, microseconds);

      // Set the output - tunePid() will return values within the range configured
      // by setOutputRange(). Don't change the value or the tuning results will be
      // incorrect.
      setVoltage(output);

      // This loop must run at the same speed as the PID control loop being tuned
      while (micros() - microseconds < CONTROL_LOOP_INTERVAL_MICROSEC) delayMicroseconds(1);
  }

  // Turn off output, tuning is complete
  setVoltage(0);
  Serial.print("P:");
  Serial.println(tuner.getKp());
  Serial.print("I:");
  Serial.println(tuner.getKi());
  Serial.print("D:");
  Serial.println(tuner.getKd());
}


float senseTemperature() {
  // For every run of the loop, compute a temperature average of 128 values
  float voltage_average_sum = 0.0f;
  float voltage = 0.0f;
  for (int i = 0; i < 128; i++) {
    float sensor_raw_value = analogRead(TEMPERATURE_PIN);
    voltage = (float) sensor_raw_value * (3.315 / 1023.0f);
    voltage_average_sum = voltage_average_sum + voltage;
  }
  voltage = voltage_average_sum / 127;
  float temperature = (voltage - CALIB_VOUT0) / CALIB_TC;

  // Calculate temperature parameters that might be useful in doing PID control later
  temperature_err = temperature - TARGET_TEMPERATURE;
  temperature_diff = temperature - temperature_last;
  temperature_last = temperature;
  return temperature;
}

// Only input 0 to 12 for the voltage!
void setVoltage(float voltage) {
  // Serial.print("Voltage set to: ");
  // Serial.println(voltage);
  analogWrite(MOSFET_PIN, (voltage/12.0f)*255);
}

// Task: find out how long each loop takes because PID is dependent on time taken per loop

void loop() {
  // senseTemperature();

  // // Below is a simple implementation of an on-off controller with a differential of 17 to 19
  // int pin_state = LOW;
  // if (temperature_last >= 14) {
  //   pin_state = HIGH;
  // }
  // else if (temperature_last > 13 && temperature_last < 14 && temperature_diff < 0) {
  //   pin_state = HIGH;
  // }
  // else {
  //   pin_state = LOW;
  // }
  // // digitalWrite(MOSFET_PIN, pin_state); // uncomment this if you want on-off control
  // analogWrite(MOSFET_PIN, (8.0f/12.0f)*255); // uncomment this if you want voltage control (change 8 to whatever voltage is wanted)
  
  // // Print current temperature and time in miliseconds since start of program
  // // Open the serial plotter to see the variables in real time
  // unsigned long currentMillis = millis();
  // if (currentMillis - previousMillis >= PRINT_INTERVAL) {
  //   previousMillis = currentMillis;
  //   delta_time = millis() - time_start;
  //   // Serial.print("current temperature:");
  //   Serial.print(temperature_last);
  //   Serial.print("," );
  //   // Serial.print("temperature error:");
  //   Serial.print(temperature_err);
  //   Serial.print("," );
  //   // Serial.print("MOSFET state:");
  //   Serial.print(pin_state*100);
  //   Serial.print("\n");
  // }
}
