#include "PID_v1.h"

// Pin definition
#define TEMPERATURE_PIN A0
#define MOSFET_PIN 11

// Parameter constants
const double TARGET_TEMPERATURE = 13.5;
const long CONTROL_LOOP_INTERVAL_MICROSEC = 100000; // 100ms or 10Hz control loop
const long PRINT_INTERVAL = 500; // 0.5 second interval
// const double Kp=149.2088, Ki=1.8315, Kd=20.367; // PID version
// const double Kp=111.9065, Ki=1.0989, Kd=0;   // PI version
const double Kp=8, Ki=0.1, Kd=0; // Hand tuned version

// Calibration constants for temperature sensor, consult datasheet and adjust accordingly
const double CALIB_VOUT0 = 0.64;   // Output voltage in V when sensor is at 0˚C
const double CALIB_TC = 0.01;   // V/°C temperature coefficient for the MCP9701/A

// Variables
unsigned long previousMillis = 0UL;
double temperature_err = -11.5;
double temperature_last = 25;
double voltage_output = 0.0;
double start_time_seconds = 0.0;

// input, output, setpoint, target
PID myPID(&temperature_last, &voltage_output, &TARGET_TEMPERATURE, Kp, Ki, Kd, REVERSE);

void senseTemperature() {
  // For every run of the loop, compute a temperature average of 511 values
  double voltage_average_sum = 0.0;
  double voltage = 0.0;
  for (int i = 0; i < 512; i++) {
    double sensor_raw_value = analogRead(TEMPERATURE_PIN);
    voltage = (double) sensor_raw_value * (1.05 / 1023.0);
    voltage_average_sum = voltage_average_sum + voltage;
  }
  voltage = voltage_average_sum / 511;
  double temperature = (voltage - CALIB_VOUT0) / CALIB_TC;

  // Calculate temperature parameters that might be useful in doing PID control later
  temperature_err = TARGET_TEMPERATURE - temperature;
  temperature_last = temperature;
}

// Only input 0 to 12 for the voltage!
void setVoltage(double voltage) {
  // Serial.print("Voltage set to: ");
  // Serial.println(voltage);
  analogWrite(MOSFET_PIN, (voltage/12.0)*255);
}

// MAIN PROGRAM
void setup() {
  analogReference(INTERNAL);
  Serial.begin(9600);
  pinMode(MOSFET_PIN, OUTPUT);
  
  delay(2000);
  myPID.SetOutputLimits(0.0,12.0);
  myPID.SetSampleTime(CONTROL_LOOP_INTERVAL_MICROSEC/1000); // in milliseconds
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetMode(AUTOMATIC); // turn PID on
  Serial.println("Started!");
  start_time_seconds = millis()/1000.0;
}

// Task: find out how long each loop takes because PID is dependent on time taken per loop

void loop() {
  senseTemperature();
  myPID.Compute(); // Note: Compute() will automatically consider the loop, so no need to put this in a millis loop
  setVoltage(voltage_output);

  // Print current temperature
  // Open the serial plotter to see the variables in real time
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= PRINT_INTERVAL) {
    previousMillis = currentMillis;
    // char strbuffer[8];
    // dtostrf(, 7, 1, strbuffer);
    Serial.print(millis()/1000.0 - start_time_seconds); // Current time in seconds
    Serial.print("," );
    Serial.print(temperature_last); // Last temperature reading
    Serial.print("," );
    Serial.print(temperature_err); // Temperature error
    Serial.print("," );
    // Serial.print("voltage:");
    Serial.print(voltage_output); // Voltage of the peltier from 0 to 12V
    Serial.print("\n");
  }
}
