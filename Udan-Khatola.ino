#include <Arduino.h>
#include <Servo.h>

#include "lib/GyroSensor.h"
#include "lib/Kalman.h"
#include "lib/PID.h"
#include "lib/RFController.h"

// ==================== All Constants ==================== //
#define rudderPin 9
#define controllerPin 11
#define aileronPinL 5
#define aileronPinR 6
#define elevatorPin 10
#define throttlePin 3

// ==================== Motor Ranges ==================== //
// Rudder:
//    85 -> 0
//    130 (+45) -> 30
//    20 (-65) -> -30
int rudderFilter(int value) {
  // return 0;
  if (value >= 0) {
    return map(value, 0, 30, 85, 130);
  } else {
    return map(value, -30, 0, 20, 85);
  }
}
// Elevator:
//    70 -> 0
//    20 (-50) -> 40
//    150 (+80) -> -40
int elevatorFilter(int value) {
  if (value >= 0) {
    return map(value, 0, 40, 70, 20);
  } else {
    return map(value, -40, 0, 150, 70);
  }
}
// Aileron Right:
//    120 -> 0
//    165 (+45) -> 30
//    70 (-50) -> -30
int aileronRightFilter(int value){
  if (value >= 0) {
    return map(value, 0, 30, 120, 165);
  } else {
    return map(value, -30, 0, 70, 120);
  }
}
// Aileron Left:
//    120 -> 0
//    40 (-60) -> 30
//    170 (+50) -> -30
int aileronLeftFilter(int value){
  if (value >= 0) {
    return map(value, 0, 30, 120, 80);
  } else {
    return map(value, -30, 0, 170, 120);
  }
}
#define aileronLDefault 120  // Depends on the servo motor orientation
#define aileronRDefault 120
#define elevatorDefault 70
#define rudderDefault 85
#define aileronLDirection 1 // Depends on the servo motor orientation... again
#define aileronRDirection -1 
#define elevatorDirection 1
#define rudderDirection 1

// ==================== Loop Timing ==================== //
unsigned long loopEndTime, endTime, currentTime;

// ==================== Gyro data with filters ==================== //
GyroSensor gyro;
GyroSensor::GyroscopeData gyroData;
Kalman kalmanX(0.1), kalmanY(0.1);

// ==================== PID and Servo Controller ==================== //
Servo aileronL, aileronR, elevator, rudder, ESC;
CascadingPID PIDAilerons(6, 0.0, 0.0, 0.1, 0.0, 0.0, -30, 30, -5, 5);
CascadingPID PIDElevators(8, 0.0, 0.0, 0.15, 0.0, 0.0, -40, 40, -5, 5);
// PIDController PIDAilerons(5, 0, 0, -45, 45);
// PIDController PIDElevators(5, 0, 0, -45, 45);
float aileronValue, elevatorValue;
float throttle;

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
  ESC.attach(3,1000,2000);

  aileronL.write(aileronLDefault);
  aileronR.write(aileronRDefault);
  elevator.write(elevatorDefault);
  rudder.write(rudderDefault);

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
  // Serial.print("Roll: ");
  // Serial.println(gyro.roll());
  // Serial.print("Pitch: ");
  // Serial.println(gyro.pitch());
  // Serial.print("Yaw: ");
  // Serial.println(gyro.yaw());
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
  // Serial.print("Aileron Value: ");
  // Serial.println(aileronValue);
  // Serial.print("Elevator Value: ");
  // Serial.println(elevatorValue);
  // Serial.print("Rudder Value: ");
  // Serial.println(controls.yaw);
  Serial.print("Throttle: ");
  Serial.println(controls.throttle);

  // Control propeller speed using the throttle value
  throttle = map(controls.throttle, 0, 100, 0, 180);
  Serial.print("Throttle: ");
  Serial.println(throttle);
  ESC.write(throttle);

  // Set the servo motors to the deflected position
  aileronL.write(aileronLeftFilter(aileronValue));
  aileronR.write(aileronRightFilter(aileronValue));
  elevator.write(elevatorFilter(elevatorValue));
  rudder.write(rudderFilter(controls.yaw));

  // Efficiently maintain a 50Hz frequency using a delay.
  // Instead of a fixed delay(20), dynamically adjust for program execution time.
  endTime = millis();             // Record current time.
  loopEndTime = currentTime + 20; // Calculate the expected end time of the loop.
  if (loopEndTime > endTime) {
    delay(loopEndTime - endTime);               // If program already exceeded 20ms, no need to delay.  
  }
}
