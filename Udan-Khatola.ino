#include <Arduino.h>
#include <Servo.h>

#include "lib/GyroSensor.h"
#include "lib/Barometer.h"
#include "lib/PID.h"

// ==================== All Constants ==================== //
const int aileronPin = 10;
const int elevatorPin = 9;
const int aileronDefault = 90;  // Depends on the servo motor orientation
const int elevatorDefault = 90;
const int aileronDirection = 1; // Depends on the servo motor orientation... again
const int elevatorDirection = -1;

// ==================== Gyro and Barometer data ==================== //
GyroSensor gyro;
Barometer barometer;
float roll, pitch;
float targetRoll, targetPitch;
float altitude, targetAltitude;

// ==================== PID and Servo Controller ==================== //
Servo aileron, elevator;
PIDController PIDAilerons(5, 0, 0, -30, 30);
PIDController PIDElevators(5, 0, 0, -60, 60);
PIDController PIDPitch(5, 0, 0, -10, 20); // For altitude control

void setup()
{
  // ==================== Enable Serial Communication to obtain debugging data ==================== //
  Serial.begin(115200);
  Serial.println("Starting Setup");

  // ==================== Setup Servo Motors and set them to default position ==================== //
  aileron.attach(aileronPin);
  elevator.attach(elevatorPin);

  aileron.write(aileronDefault);
  elevator.write(elevatorDefault);

  // ==================== Setup MPU6050 Module ==================== //
  Serial.println("Initializing I2C devices...");
  gyro.setup();
  gyro.calibrate();
  barometer.setup();

  // ==================== Set Target Values ==================== //
  // For now, we need to keep the aircraft stable. For this, we need
  // to keep the roll and pitch values to 0. We will use the gyro
  // sensor to obtain the roll and pitch values and then use the
  // servo motors to adjust the actual roll and pitch values to 0.
  targetRoll = 0;
  targetPitch = 0;
  targetAltitude = barometer.getRelativeAltitude();
  // Set the PID setpoints to the target values
  PIDAilerons.setSetpoint(targetRoll);
  PIDElevators.setSetpoint(targetPitch);
  PIDPitch.setSetpoint(targetAltitude);
  // In real use cases, these values will be obtained from the
  // ground station / remote control and will be updated in loop.

  Serial.println("Setup Done");
}

void loop()
{
  // Use acceleromter and gyro angle values and barometer data to obtain the roll, pitch and altitude values
  roll = gyro.roll();
  pitch = gyro.pitch();
  altitude = barometer.getRelativeAltitude();
  // Obtain target pitch based on what altitude we have to achieve
  targetPitch = PIDPitch.compute(altitude);
  PIDElevators.setSetpoint(targetPitch);
  // Obtain the aileron and elevator deflection from the PID controller
  double aileronValue = PIDAilerons.compute(roll);
  double elevatorValue = PIDElevators.compute(pitch);
  Serial.print("Aileron Value: ");
  Serial.println(aileronValue);
  Serial.print("Elevator Value: ");
  Serial.println(elevatorValue);
  // Set the servo motors to the deflected position
  aileron.write(aileronDefault + aileronValue*aileronDirection);
  elevator.write(elevatorDefault + elevatorValue*elevatorDirection);
}
