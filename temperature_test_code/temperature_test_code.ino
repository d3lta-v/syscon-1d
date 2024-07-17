// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  analogReference(EXTERNAL);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  // Using 5V default: 0.00488V
  // Using 3.3V external: 0.00323V (Note: actual voltage is 3.315V)
  // Using 1.1V internal: 0.00107V
  float voltage = sensorValue * (3.315 / 1023.0);
  // Convert the voltage to temperature in degrees Celsius
  float temperature = voltage - 0.55;
  temperature = temperature / 0.01;
  // print out the value you read:
  Serial.println(temperature);
}
