#include <Arduino.h>
#include <ESP32PWM.h>
#include <ESP32Servo.h>

#include "lib/GyroSensor.h"
#include "lib/PID.h"

// ==================== All Constants ==================== //
const int aileronPin1 = 10;
const int aileronPin2 = 9;
const int elevatorPin = 8;
const int rudderPin = 7;
const int aileron1Default = 90;  // Depends on the servo motor orientation
const int aileron2Default = 90;
const int elevatorDefault = 90;
const int rudderDefault = 90;
const int aileron1Direction = 1; // Depends on the servo motor orientation... again
const int aileron2Direction = 1; 
const int elevatorDirection = -1;
const int yawDirection = 1;

// ==================== Gyro and Barometer data ==================== //
GyroSensor gyro;
float roll, pitch, yaw;
float targetRoll, targetPitch;
float altitude, targetAltitude;

// ==================== PID and Servo Controller ==================== //
Servo aileron1, aileron2, elevator, rudder;
PIDController PIDAilerons(5, 0, 0, -30, 30);
PIDController PIDElevators(5, 0, 0, -60, 60);

void setup()
{
  // ==================== Enable Serial Communication to obtain debugging data ==================== //
  Serial.begin(115200);
  Serial.println("Starting Setup");

  // ==================== Setup Servo Motors and set them to default position ==================== //
  aileron1.attach(aileronPin1);
  aileron2.attach(aileronPin2);
  elevator.attach(elevatorPin);
  rudder.attach(rudderPin);

  aileron1.write(aileron1Default);
  aileron2.write(aileron2Default);
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
  // In real use cases, these values will be obtained from the
  // ground station / remote control and will be updated in loop.

  Serial.println("Setup Done");
}

void loop()
{
  unsigned long currentTime = millis();          // Get the current time in milliseconds
  // Use acceleromter and gyro angle values and barometer data to obtain the roll, pitch and altitude values
  roll = gyro.roll();
  pitch = gyro.pitch();
  // Obtain the aileron1 and elevator deflection from the PID controller
  double aileronValue = PIDAilerons.compute(roll);
  double elevatorValue = PIDElevators.compute(pitch);
  Serial.print("Aileron Value: ");
  Serial.println(aileronValue);
  Serial.print("Elevator Value: ");
  Serial.println(elevatorValue);
  // Set the servo motors to the deflected position
  aileron1.write(aileron1Default + aileronValue*aileron1Direction);
  aileron2.write(aileron2Default - aileronValue*aileron2Direction);
  elevator.write(elevatorDefault + elevatorValue*elevatorDirection);
  //For yaw no need of PID, so feeding raw value from reciever to rudder servo motor
  yaw = constrain(yaw, -30, 30);                //constraining raw values so as to prevent extreme servo movements
  rudder.write(rudderDefault + yaw*yawDirection);

  // Efficiently maintain a 50Hz frequency using a delay.
  // Instead of a fixed delay(20), dynamically adjust for program execution time.
  unsigned long endTime = millis();             // Record current time.
  unsigned long loopEndTime = currentTime + 20; // Calculate the expected end time of the loop.
  if (loopEndTime < endTime) {
    delay(loopEndTime - endTime);               // If program already exceeded 20ms, no need to delay.  
  }
}
