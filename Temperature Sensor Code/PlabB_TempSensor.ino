#define GPIO_PIN 5  // Data wire is connected to digital pin 5 or any GPIO

void setup() {
  Serial.begin(9600);  // Start serial communication
}

void loop() {
  float temperatureCelsius = getTemperature();
  
  // Print the temperature to the serial monitor
  Serial.print("Temperature: ");
  Serial.print(temperatureCelsius);
  Serial.println(" °C");

  delay(1000);  // Wait for a moment before reading the temperature again
}

float getTemperature() {
  byte data[9];
  
  // Reset OneWire bus
  pinMode(GPIO_PIN, OUTPUT);
  digitalWrite(GPIO_PIN, LOW);
  delayMicroseconds(480);
  pinMode(GPIO_PIN, INPUT_PULLUP);
  delayMicroseconds(60);
  
  // Check presence pulse
  if (!digitalRead(GPIO_PIN)) {
    return -999.0;  // Sensor not detected
  }
  
  delayMicroseconds(420);
  
  // Read temperature data
  pinMode(GPIO_PIN, OUTPUT);
  digitalWrite(GPIO_PIN, LOW);
  delayMicroseconds(480);
  pinMode(GPIO_PIN, INPUT_PULLUP);
  delayMicroseconds(60);
  
  // Check if DS18B20 is transmitting data
  if (digitalRead(GPIO_PIN)) {
    return -999.0;  // Error: no response from sensor
  }
  
  delayMicroseconds(420);
  
  // Read 9 bytes of data from sensor
  for (int i = 0; i < 9; i++) {
    data[i] = shiftIn(GPIO_PIN);
  }
  
  // Calculate temperature from the data received
  int16_t rawTemperature = (data[1] << 8) | data[0];
  float temperatureCelsius = rawTemperature / 16.0;
  
  return temperatureCelsius;
}

byte shiftIn(int pin) {
  byte value = 0;
  for (int i = 0; i < 8; ++i) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(10);
    value |= digitalRead(pin) << i;
    digitalWrite(pin, LOW);
    delayMicroseconds(10);
  }
  return value;
}
