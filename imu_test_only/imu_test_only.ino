// Simple IMU-only test for MPU6050
// Tests MPU6050 connection without motor driver interference

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu6050;

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial connection

  Serial.println("=== MPU6050 Standalone Test ===");
  Serial.println("Testing MPU6050 without motor driver...");

  // Initialize I2C
  Wire.begin();
  Wire.setClock(100000); // Start slow
  delay(100);

  Serial.println("Initializing MPU6050...");
  mpu6050.initialize();

  Serial.print("Testing connection...");
  bool connected = false;

  for (int attempt = 1; attempt <= 5; attempt++) {
    Serial.print(" Attempt ");
    Serial.print(attempt);
    Serial.print("...");

    if (mpu6050.testConnection()) {
      Serial.println("SUCCESS!");
      connected = true;

      // Get some basic readings
      int16_t ax, ay, az, gx, gy, gz;
      mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

      Serial.println("Sample readings:");
      Serial.print("Accel: X=");
      Serial.print(ax);
      Serial.print(" Y=");
      Serial.print(ay);
      Serial.print(" Z=");
      Serial.println(az);
      Serial.print("Gyro: X=");
      Serial.print(gx);
      Serial.print(" Y=");
      Serial.print(gy);
      Serial.print(" Z=");
      Serial.println(gz);

      break;
    } else {
      Serial.println("FAILED!");
      delay(200);
    }
  }

  if (!connected) {
    Serial.println("MPU6050 connection failed!");
    Serial.println("Check:");
    Serial.println("- 3.3V power supply (NOT 5V!)");
    Serial.println("- SDA/SCL connections to D20/D21");
    Serial.println("- I2C pull-up resistors");
    Serial.println("- IMU module functionality");

    // Manual I2C test
    Serial.println("Manual I2C test...");
    Wire.beginTransmission(0x68);
    byte error = Wire.endTransmission();
    Serial.print("I2C transmission result: ");
    Serial.println(error);
  }

  Serial.println("Test complete.");
}

void loop() {
  delay(1000);
}
