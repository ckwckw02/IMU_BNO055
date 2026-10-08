#include <Wire.h>

// Define ESP32-C3 default I2C pins
#define I2C_SDA D4
#define I2C_SCL D5

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  // Initialize I2C with custom pins
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("\nI2C Scanner initialized for ESP32-C3");
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning I2C bus...");

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println("No I2C devices found.\n");
  } else {
    Serial.println("Scan complete.\n");
  }

  delay(500);  // Wait 5 seconds before next scan
}
