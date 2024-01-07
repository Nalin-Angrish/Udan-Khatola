#include <Arduino.h>
#include <Servo.h>

#include "lib/GyroSensor.h"
#include "lib/PID.h"

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
float roll, pitch;
float targetRoll, targetPitch;

// ==================== PID Controller ==================== //
// TODO: Tune the PID values

void setup(){
  // ==================== Enable Serial Communication to obtain debugging data ==================== //
  Serial.begin(115200);
  Serial.println("Starting Setup");

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

  // ==================== Set Target Values ==================== //
  // For now, we need to keep the aircraft stable. For this, we need
  // to keep the roll and pitch values to 0. We will use the gyro
  // sensor to obtain the roll and pitch values and then use the
  // servo motors to adjust the actual roll and pitch values to 0.
  targetRoll = 0;
  targetPitch = 0;
  // In real use cases, these values will be obtained from the
  // ground station / remote control.

  Serial.println("Setup Done");
}

void loop(){
  // Use acceleromter and gyro angle values to obtain the roll and pitch values
  roll = gyro.roll();
  pitch = gyro.pitch();

  // ==================== Print the values ==================== //
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(roll);
  delay(200);
}
