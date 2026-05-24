#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
Adafruit_ICM20649 icm;
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing


void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit ICM20649");

  // Try to initialize!
  if (!icm.begin_I2C()) {

    Serial.println("Failed to find ICM20649 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("ICM20649 Found!");
  icm.setAccelRange(ICM20649_ACCEL_RANGE_16_G);  //ACCEL RANGE

  //JUST FOR PRINTING NOT IMPORTANT
  Serial.print("Accelerometer range set to: ");
  switch (icm.getAccelRange()) {
  case ICM20649_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case ICM20649_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case ICM20649_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  case ICM20649_ACCEL_RANGE_30_G:
    Serial.println("+-30G");
    break;
  }

  icm.setGyroRange(ICM20649_GYRO_RANGE_500_DPS); //gyro range

  //JUST FOR PRINTING NOT IMPORTANT
  Serial.print("Gyro range set to: ");
  switch (icm.getGyroRange()) {
  case ICM20649_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case ICM20649_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case ICM20649_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ICM20649_GYRO_RANGE_4000_DPS:
    Serial.println("4000 degrees/s");
    break;
  }

  icm.setAccelRateDivisor(4095);
  uint16_t accel_divisor = icm.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);  //you can change accel rate, change divisor


  //PRINTING NOT IMPORTANT
  Serial.print("Accelerometer data rate divisor set to: ");
  Serial.println(accel_divisor);
  Serial.print("Accelerometer data rate (Hz) is approximately: ");
  Serial.println(accel_rate);

  icm.setGyroRateDivisor(255);
  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor); //you can change gyro rate change divisor

  Serial.print("Gyro data rate divisor set to: ");
  Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");
  Serial.println(gyro_rate);
  Serial.println();
}

unsigned long lastTime = 0;
sensors_event_t accel, gyro, temp;
float ICM20649::getTemperature() { return temp; }

void ICM20649::read() {
  icm.getEvent(&accel, &gyro, &temp);
}

float ICM20649::getAccelX() { return accel.acceleration.x; }
float ICM20649::getAccelY() { return accel.acceleration.y; }
float ICM20649::getAccelZ() { return accel.acceleration.z; }

float ICM20649::getGyroX() { return gyro.gyro.x; }
float ICM20649::getGyroY() { return gyro.gyro.y; }
float ICM20649::getGyroZ() { return gyro.gyro.z; }