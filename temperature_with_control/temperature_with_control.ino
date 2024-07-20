/*
   The value of Vout is obtained by averaging 1024 values reading  the voltage on pin A0. which is connected to the sensor output. 
   The value of temeprature  ta, read by the sensor is shown on the serial monitor.
*/

// Pin definition
#define TEMPERATURE_PIN A0
#define MOSFET_PIN 11

// Parameter constants
const int TARGET_TEMPERATURE = 13.5;
const long interval = 50; // 0.05 second interval

// Calibration constants for temperature sensor, consult datasheet and adjust accordingly
const float CALIB_VOUT0 = 0.64;   // Output voltage in V when sensor is at 0˚C
const float CALIB_TC = 0.01;   // V/°C temperature coefficient for the MCP9701/A

// Variables
unsigned long time_start = millis();
unsigned long delta_time;
unsigned long previousMillis = 0;
double temperature_err;
double temperature_last = 20;
double temperature_diff;

long loop_time = 0L;

// MAIN PROGRAM
void setup() {
  analogReference(EXTERNAL);
  Serial.begin(9600);
  pinMode(MOSFET_PIN, OUTPUT);
}

void loop() {
  unsigned long start_microseconds = micros();

  // For every run of the loop, compute a temperature average of 1023 values
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

  // Below is a simple implementation of an on-off controller with a differential of 17 to 19
  // int pin_state = LOW;
  // if (temperature >= 14) {
  //   pin_state = HIGH;
  // }
  // else if (temperature > 13 && temperature < 14 && temperature_diff < 0) {
  //   pin_state = HIGH;
  // }
  // else {
  //   pin_state = LOW;
  // }
  // digitalWrite(MOSFET_PIN, pin_state); // uncomment this if you want on-off control
  // analogWrite(MOSFET_PIN, (8.0f/12.0f)*255); // uncomment this if you want voltage control (change 8 to whatever voltage is wanted)


  int pin_state = LOW;
  if (temperature > 13.5) {
    pin_state = HIGH;
  } else if (temperature < 13.5) {
    pin_state = LOW;
  }
  digitalWrite(MOSFET_PIN, pin_state);
  
  // Print current temperature and time in miliseconds since start of program
  // Open the serial plotter to see the variables in real time
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    delta_time = millis() - time_start;
    // Serial.print("current temperature:");
    Serial.print(temperature);
    Serial.print(',');
    // Serial.print("temperature error:");
    Serial.print(temperature_err);
    Serial.print(',');
    // Serial.print("MOSFET state:");
    Serial.print(pin_state*100);
    Serial.print(',');
    Serial.print(loop_time);
    Serial.print("\n");
  }
  loop_time = micros() - start_microseconds;
}
