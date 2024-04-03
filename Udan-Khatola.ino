#include <Arduino.h>
#include <ESP32PWM.h>
#include <ESP32Servo.h>

#include "lib/GyroSensor.h"
#include "lib/PID.h"
#include "lib/RFController.h"

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

// ==================== Loop Timing ==================== //
unsigned long loopEndTime, endTime, currentTime;

// ==================== Gyro and Barometer data ==================== //
GyroSensor gyro;
float roll, pitch;
float targetRoll, targetPitch;

// ==================== PID and Servo Controller ==================== //
Servo aileron, elevator, rudder;
PIDController PIDAilerons(5, 0, 0, -30, 30);
PIDController PIDElevators(5, 0, 0, -60, 60);
float aileronValue, elevatorValue;

// ==================== Radio Control ==================== //
RFController controller;
ControlProps controls;

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
  PIDAilerons.setSetpoint(0);
  PIDElevators.setSetpoint(0);
  // In the main program loop, these values will be obtained from the
  // ground station / remote control and will be updated in loop.

  Serial.println("Setup Done");
}

void loop()
{
  currentTime = millis();          // Get the current time in milliseconds

  // Read the control values from the radio controller and set the PID setpoints
  controls = controller.readControls(); 
  Serial.print("Roll: ");
  Serial.println(controls.roll);
  Serial.print("Pitch: ");
  Serial.println(controls.pitch);
  Serial.print("Yaw: ");
  Serial.println(controls.yaw);
  PIDAilerons.setSetpoint(controls.roll);
  PIDElevators.setSetpoint(controls.pitch);

  // Use acceleromter and gyro values to obtain the roll, pitch and yaw values
  roll = gyro.roll();
  pitch = gyro.pitch();

  // Obtain the aileron and elevator deflection from the PID controller
  aileronValue = PIDAilerons.compute(roll);
  elevatorValue = PIDElevators.compute(pitch);
  Serial.print("Aileron Value: ");
  Serial.println(aileronValue);
  Serial.print("Elevator Value: ");
  Serial.println(elevatorValue);
  Serial.print("Rudder Value: ");
  Serial.println(controls.yaw);

  // Set the servo motors to the deflected position
  aileron.write(aileronDefault + aileronValue*aileronDirection);
  elevator.write(elevatorDefault + elevatorValue*elevatorDirection);
  rudder.write(rudderDefault + controls.yaw*rudderDirection);

  // Efficiently maintain a 50Hz frequency using a delay.
  // Instead of a fixed delay(20), dynamically adjust for program execution time.
  endTime = millis();             // Record current time.
  loopEndTime = currentTime + 20; // Calculate the expected end time of the loop.
  if (loopEndTime < endTime) {
    delay(loopEndTime - endTime);               // If program already exceeded 20ms, no need to delay.  
  }
}
