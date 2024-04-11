#include <Arduino.h>
#include <Servo.h>

#include "lib/GyroSensor.h"
#include "lib/Kalman.h"
#include "lib/PID.h"
#include "lib/RFController.h"

// ==================== All Constants ==================== //
#define controllerPin 5 // 6 done (arduino side)
#define throttlePin 9 // 5 done
#define aileronPinL 3 // 1 done (power side)
#define aileronPinR 6 // 2 done
#define elevatorPin 10 // 3 done
#define rudderPin 11 // 4 done

#define aileronLDefault 135  // Depends on the servo motor orientation
#define aileronRDefault 120
#define elevatorDefault 70
#define rudderDefault 80
#define aileronLDirection 1 // Depends on the servo motor orientation... again
#define aileronRDirection 1 
#define elevatorDirection -1
#define rudderDirection 1

// ==================== Loop Timing ==================== //
unsigned long loopEndTime, endTime, currentTime;

// ==================== Gyro data with filters ==================== //
GyroSensor gyro;
GyroSensor::GyroscopeData gyroData;
Kalman kalmanX(0.1), kalmanY(0.1);
float speed;

// ==================== PID and Servo Controller ==================== //
Servo aileronL, aileronR, elevator, rudder;
CascadingPID PIDAilerons(5, 0, 0, 5, 0, 0, -45, 45, -5, 5);
CascadingPID PIDElevators(5, 0, 0, 5, 0, 0, -50, 50, -5, 5);
// PIDController PIDAilerons(5, 0, 0, -45, 45);
// PIDController PIDElevators(5, 0, 0, -45, 45);
float aileronValue, elevatorValue;

// ==================== Radio Control ==================== //
RFController controller(controllerPin);
ControlProps controls;

void setup()
{
  // ==================== Enable Serial and RF Communication to communicate data ==================== //
  Serial.begin(115200);
  Serial.println("Starting Setup");
  controller.begin();

  // ==================== Setup Servo Motors and set them to default position ==================== //
  aileronL.attach(aileronPinL);
  aileronR.attach(aileronPinR);
  elevator.attach(elevatorPin);
  rudder.attach(rudderPin);

  aileronL.write(aileronLDefault);
  aileronR.write(aileronRDefault);
  elevator.write(elevatorDefault);
  rudder.write(rudderDefault);

  //===================== Throttle Pin Setup ==========================//
  // pinMode(throttlePin, OUTPUT);
  // digitalWrite(throttlePin, LOW);


  // ==================== Setup MPU6050 Module ==================== //
  Serial.println("Initializing MPU6050 Module");
  gyro.setup();
  gyro.calibrate();

  Serial.println("Setup Done");
}

void loop()
{
  currentTime = millis();          // Get the current time in milliseconds

  // Read the control values from the radio controller and set the PID setpoints
  controls = controller.readControls();

  // Use acceleromter and gyro values to obtain the roll, pitch and yaw values
  // and use them to obtain the aileron and elevator deflection from the PID controller
  gyroData = gyro.getGyroscopeData();
  Serial.print("Roll: ");
  Serial.println(gyro.roll());
  Serial.print("Pitch: ");
  Serial.println(gyro.pitch());
  Serial.print("Yaw: ");
  Serial.println(gyro.yaw());
  aileronValue = PIDAilerons.compute(
    controls.roll,
    kalmanX.compute(gyroData.rateX, gyro.roll()), 
    gyroData.rateX
  );
  elevatorValue = PIDElevators.compute(
    controls.pitch,
    kalmanY.compute(gyroData.rateY, gyro.pitch()), 
    gyroData.rateY
  );
  // aileronValue = controls.roll;
  // elevatorValue = controls.pitch;
  Serial.print("Aileron Value: ");
  Serial.println(aileronValue);
  Serial.print("Elevator Value: ");
  Serial.println(elevatorValue);
  Serial.print("Rudder Value: ");
  Serial.println(controls.yaw);
  Serial.print("Throttle: ");
  Serial.println(controls.throttle);

  // Control propeller speed using the throttle value
  // speed = map(controls.throttle, 0, 100, 1000, 2000);
  // analogWrite(throttlePin, speed);

  // Set the servo motors to the deflected position
  aileronL.write(aileronLDefault + aileronValue*aileronLDirection);
  aileronR.write(aileronRDefault - aileronValue*aileronRDirection);
  elevator.write(elevatorDefault + elevatorValue*elevatorDirection);
  rudder.write(rudderDefault + controls.yaw*rudderDirection);

  // Efficiently maintain a 50Hz frequency using a delay.
  // Instead of a fixed delay(20), dynamically adjust for program execution time.
  endTime = millis();             // Record current time.
  loopEndTime = currentTime + 1000; // Calculate the expected end time of the loop.
  if (loopEndTime > endTime) {
    delay(loopEndTime - endTime);               // If program already exceeded 20ms, no need to delay.  
  }
}
