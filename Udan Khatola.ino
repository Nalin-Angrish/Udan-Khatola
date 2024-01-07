#include <Arduino.h>
#include <Servo.h>

#include "lib/GyroSensor.h"

// ==================== All Constants ==================== //
const int rudderPin = 9;
const int aileronPin = 10;
const int elevatorPin = 11;

const int rudderDefault = 90;
const int aileronDefault = 90;
const int elevatorDefault = 90;

// ==================== Servo motors for plane control ==================== //
Servo rudder, aileron, elevator;

// ==================== Gyro and Accelerometer data ==================== //
GyroSensor gyro;
float roll, pitch, yaw;

void setup(){
  // ==================== Enable Serial Communication to obtain debugging data ==================== //
  Serial.begin(115200);

  // ==================== Setup Servo Motors and set them to default position ==================== //
  rudder.attach(rudderPin);
  aileron.attach(aileronPin);
  elevator.attach(elevatorPin);

  rudder.write(rudderDefault);
  aileron.write(aileronDefault);
  elevator.write(elevatorDefault);

  // ==================== Setup MPU6050 Module ==================== //
  Serial.println("Initializing I2C devices...");
  gyro.setup();
  gyro.calibrate();

  // ==================== Setup Done ==================== //
  Serial.println("Setup Done");
}

void loop(){
  // Use acceleromter and gyro angle values to obtain the roll, pitch and yaw values
  roll = gyro.roll();
  pitch = gyro.pitch();
  yaw = gyro.yaw();

  // ==================== Print the values ==================== //
  Serial.print(pitch);
  Serial.print(" ");
  Serial.print(roll);
  Serial.print(" ");
  Serial.println(yaw);
  delay(200);
}
