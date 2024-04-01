#include <Arduino.h>
#include <ESP32PWM.h>
#include <ESP32Servo.h>

#include "lib/GyroSensor.h"
#include "lib/PID.h"

// ==================== All Constants ==================== //
const int aileronPin = 10;
const int elevatorPin = 9;
const int rudderPin = 8;
const int aileronDefault = 90;  // Depends on the servo motor orientation
const int elevatorDefault = 90;
const int rudderDefault = 90;
const int aileronDirection = 1; // Depends on the servo motor orientation... again
const int elevatorDirection = -1;
const int rudderDirection = 1;

// ==================== Gyro and Barometer data ==================== //
GyroSensor gyro;
float roll, pitch, yaw;
float targetRoll, targetPitch;

// ==================== PID and Servo Controller ==================== //
Servo aileron, elevator, rudder;
PIDController PIDAilerons(5, 0, 0, -30, 30);
PIDController PIDElevators(5, 0, 0, -60, 60);
PIDController PIDRudder(5, 0, 0, -30, 30);
float aileronValue, elevatorValue, rudderValue;

void setup()
{
  // ==================== Enable Serial Communication to obtain debugging data ==================== //
  Serial.begin(115200);
  Serial.println("Starting Setup");

  // ==================== Setup Servo Motors and set them to default position ==================== //
  aileron.attach(aileronPin);
  elevator.attach(elevatorPin);
  rudder.attach(rudderPin);

  aileron.write(aileronDefault);
  elevator.write(elevatorDefault);
  rudder.write(rudderDefault);

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
  // Set the PID setpoints to the target values
  PIDAilerons.setSetpoint(targetRoll);
  PIDElevators.setSetpoint(targetPitch);
  PIDRudder.setSetpoint(0);
  // In real use cases, these values will be obtained from the
  // ground station / remote control and will be updated in loop.

  Serial.println("Setup Done");
}

void loop()
{
  unsigned long currentTime = millis();          // Get the current time in milliseconds
  // Use acceleromter and gyro angle values and barometer data to obtain the roll and pitch values
  roll = gyro.roll();
  pitch = gyro.pitch();
  yaw = gyro.yaw();
  // Obtain the aileron and elevator deflection from the PID controller
  aileronValue = PIDAilerons.compute(roll);
  elevatorValue = PIDElevators.compute(pitch);
  rudderValue = PIDRudder.compute(yaw);
  Serial.print("Aileron Value: ");
  Serial.println(aileronValue);
  Serial.print("Elevator Value: ");
  Serial.println(elevatorValue);
  Serial.print("Rudder Value: ");
  Serial.println(rudderValue);
  // Set the servo motors to the deflected position
  aileron.write(aileronDefault + aileronValue*aileronDirection);
  elevator.write(elevatorDefault + elevatorValue*elevatorDirection);
  rudder.write(rudderDefault + rudderValue*rudderDirection);

  // Efficiently maintain a 50Hz frequency using a delay.
  // Instead of a fixed delay(20), dynamically adjust for program execution time.
  unsigned long endTime = millis();             // Record current time.
  unsigned long loopEndTime = currentTime + 20; // Calculate the expected end time of the loop.
  if (loopEndTime < endTime) {
    delay(loopEndTime - endTime);               // If program already exceeded 20ms, no need to delay.  
  }
}
