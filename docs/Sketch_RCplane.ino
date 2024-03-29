// Including libraries
#include <ppm.h>       //this is required to receive PPM signals from transmitter there are alternate options too to read from transmitter but since it reads from ppm pin only as mentioned in group, we have used this method
#include <Servo.h>      //For controlling servo motors, (Ailerons rudder and elevator)
#include <MPU6050.h>    //For MPU6050

#define CHANNEL_COUNT 6  //As we have 6 channels in Flysky transmitter
#define PPM_PIN 3     //Receiver module attached to pin 3

// Defining global variables
Servo aileronServo;    //For Ailerons
Servo elevatorServo;   //For Elevators
Servo rudderServo;     //For Rudder

MPU6050 mpu;          //For MPU6050

//Defining an array for storing sata received from channels
int channel[CHANNEL_COUNT]; 

// Values Recieved from Transmitter
int pitchInput = 0;   // Target pitch angle (level)
int rollInput = 0;    // Target roll angle (level) 
int yawInput = 0;     // Target yaw angle (level)

// PID Parameters (will change accordingly once we get hands on it physically, subject to change)
float Kp = 1.0 ;   // Proportional Constant
float Ki = 1.0 ;   // Integral Constant
float Kd = 1.0 ;   // Derivative Constant

// Values from MPU6050 within our RC plane itself
float pitch, roll, yaw;    // Pitch, roll, and yaw angles


// Error in pitch, roll
float pitchError=0.0, rollError=0.0;
float prevPitchError = 0.0, prevRollError = 0.0;

// Variables for time-based PID control
unsigned long prevTime = 0;     //unsigned because we can use a longer set of values if you ignore the negative numbers.
float dt = 0.0;

// Integral terms for pitch and roll
float pitchIntegral = 0.0, rollIntegral = 0.0;

// Derivative terms for pitch, roll
float pitchDerivative, rollDerivative; 

// Servo outputs
float elevatorOutput, aileronOutput;     //No rudderOutput required since yaw doesn't require PID  

// Variables for propellers
float throttleInput;            
int propeller = 6;                       // Propellor motor pin attached to pin 6 pwm enabled              



void setup() {

  Serial.begin(115200); // Higher Baud rate as information input is high, therefore we didn't use 9600 Baud rate

  ppm.begin(PPM_PIN,false);          //Initializing to read PPM signals from receiver momdule, the second parameter is for the inversion of the ppm signal which we don't wish for therefore it is false 
  mpu.initialize();                 // For initializing Accelerometer/Gyroscope
 
 //defining pin for servos
  elevatorServo.attach(9); // Elevator servo attached to pin 9 pwm enabled
  aileronServo.attach(10); // Aileron servo attached to pin 10 pwm enabled
  rudderServo.attach(11);  // Rudder servo attached to pin 11 pwm enabled
  pinMode(propeller, OUTPUT);   //for popeller motor output
}



//Assuming transmitter sends as such, the reciever code is as follows:
void readReceiver() {

    // Read PPM channel values
    for (int i = 0; i < CHANNEL_COUNT; i++) {
      channel[i] = ppm.read_channel(i);                   // Reads the signals from channels and stores it in an array
    }

  throttleInput = map(channel[0], 1000, 2000, 0, 255);    // Map the value in Microseconds to analog values (0 to 255)

   // Map the value in Microseconds to angle between -30° to 30°, We mapped the data so as to move 30° only in each side so that the chances of toppling are reduced, may change accordingly after getting it physically
  pitchInput = map(channel[1], 1000, 2000, -30, 30);  
  rollInput = map(channel[2], 1000, 2000, -30, 30);
  yawInput = map(channel[3], 1000, 2000, -30, 30);

}


// A function to get three values from MPU6050 along all axes
void readMPU() { 

  int16_t accelX, accelY, accelZ;              // Making 2 bytes integer pointers so that we can pass them by reference in the below function
  mpu.getAcceleration(accelX, accelY, accelZ); //To get acceleration along axes and store it in the variables.

  // Directly use raw accelerometer values for angle calculations
  float accel_x = accelX;
  float accel_y = accelY;
  float accel_z = accelZ;

  // Calculate pitch and roll angles
  pitch = atan2( - accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * RAD_TO_DEG; //RAD_TO_DEG converts angles in radian to degrees
  roll = atan2(accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * RAD_TO_DEG;

}



//Proportional (P), Integral (I), and Derivative (D) Algorithm
void pidControl(){
 // Calculate errors for pitch and roll
  pitchError = pitchInput - pitch;
  rollError = rollInput - roll;

  // Calculate integral terms for pitch and roll
  pitchIntegral += pitchError * dt;  // Scale by time step
  rollIntegral += rollError * dt;    // Scale by time step

  // Calculate derivative terms for pitch and roll
  pitchDerivative = (pitchError - prevPitchError) / dt;  // Scale by time step
  rollDerivative = (rollError - prevRollError) / dt;     // Scale by time step

  // PID computation for pitch and roll
  elevatorOutput = Kp * pitchError + Ki * pitchIntegral + Kd * pitchDerivative;
  aileronOutput = Kp * rollError + Ki * rollIntegral + Kd * rollDerivative;

  // Limit the output to prevent extreme servo movements, and to reduce the chances of toppling
  elevatorOutput = constrain(elevatorOutput, -30, 30);
  aileronOutput = constrain(aileronOutput, -30, 30);
}



//function to operate servos and propeller
void controlServosAndThrottle() {

// A function to control the three servos, 90 represents the initial state here, servo alligned just to the centre as it can move from (0-180) positions
  elevatorServo.write(90 + elevatorOutput);
  aileronServo.write(90 + aileronOutput);         
  rudderServo.write(90 + yawInput);                    // No PID for yaw, direct control

  // Motor control based on throttle input
  analogWrite(propeller, throttleInput);

  // Store current errors for the next iteration
  prevPitchError = pitchError;
  prevRollError = rollError;
}


void loop(){

unsigned long currentTime = millis();          // Get the current time in milliseconds
dt = (currentTime - prevTime) / 1000.0;        // Calculate time step in seconds
prevTime = currentTime;                        // Update previous time

readReceiver();                                //Calling readReceiver function
readMPU();                                     //Calling readMPU function
pidControl();                                  //Calling pidControl function
controlServosAndThrottle();                    //Calling controlServosAndThrottle function

// explained in pdf in detail
unsigned long endTime = millis();
unsigned long loopEndTime = currentTime + 20;
  if (loopEndTime < endTime){
  delay(loopEndTime - endTime);
}

}