
    // servo program - declaring servo pins 

    #include <Servo.h>
    Servo s1; // for right aileron
    Servo s2; // for left aileron
    Servo s3; // for elevator



    // PID program - it is a control algorithm that maintains a desired output by continuously adjusting difference between desired value and actual value

    // defining constants (to be determined on physical testing)
    double Kp = 1; 
    double Ki = 1;
    double Kd = 1;

    // defining some initial parameters
    double error; // difference between current parameter and desired parameter
    double lasterror = 0; // error of previous cycle
    double integral = 0; 
    unsigned long currentTime = 0, lastTime = 0; 
    double deltaTime;
    double P, I, D;
    double output;

    // PID function
    double PIDfunc(double a, double b){ // a is the current position, b is the position needed to be reached
      
      // calculating error
      error = b - a;

      // setting delta time for derivative
      currentTime = millis(); // returns time passed since starting of program in milliseconds
      deltaTime = currentTime - lastTime; 
      lastTime = currentTime; 

      // updating the integral
      integral += error * deltaTime;

      // calculating PID terms
      P = Kp * error; // propotional term
      I = Ki * integral; // integral term
      D = Kd * (error - lasterror) / deltaTime; // differential term
      output = P + I + D;

      lasterror = error; // updating error

      return output;
    }



    // mpu6050 program - this is used to return real time acceleration and rotational values of the plane

    #include <Wire.h> // including library for serial communication
    #include <MPU6050.h> // including library for mpu6050

    MPU6050 mpu; 
    int16_t  ax, ay, az; // defining variables for accelerometer readings for three axis
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16); // sets full scale reading from -16g to +16g



    // remote control program

    // defining pins able for hardware interrupt of arduino, so that we have less lagging in the signal and the code runs faster
    #define RCPin2 2 // to control roll
    #define RCPin3 3 // to control pitch
    #define RCPin4 4 // to control throttle (motor speed for propulsion)
    // defining variables required to measure the pulsewidth
    volatile long StartTime2 = 0; // start time of the width2
    volatile long CurrentTime2 = 0; // measured time from pin 2
    volatile long Pulses2 = 0; // CurrentTime2 - StartTime2
    int PulseWidth2 = 0; //width of pulses without noise

    volatile long StartTime3 = 0; // start time of the width3
    volatile long CurrentTime3 = 0; // measured time from pin3 
    volatile long Pulses3= 0; 
    int PulseWidth3 = 0;

    int RCValue; // defining variable to store the throttle input

    void PulseTimer2(){ // interrupt service routine function, this function is called when there is any change in the pwm signal from pin 2
      CurrentTime2 = micros();
      if (currentTime2 > StartTime2){
        pulses2 = CurrentTime2 - StartTime2; 
      }
    }

    void PulseTimer3(){ // interrupt service routine function, this function is called when there is any change in the pwm signal from pin 3 
      CurrentTime3 = micros();
      if (currentTime3 > StartTime3){
        pulses3 = CurrentTime3 - StartTime3; 
      }
    } 



    // master program

    double desiredpitch, desiredroll; // values received from receiver for pitch and roll
    double currentroll, currentpitch; // defining current pitch and roll parameter variables received from the mpu6050
    double PITCH; // output from PID for pitch
    double ROLL; // output from PID for roll
    double s1Roll; // degrees for roll servo
    double s2Roll; // degrees for roll servo
    double s3Pitch; // degrees for pitch servo
    int Motor = 8; // defining throttle motor pin
    int motorspeed; // declaring motor speed variable 


    void setup(){
      s1.attach(9); // attaching servos to arduino pins
      s2.attach(10);
      s3.attach(11);
      s1.write(90); // moving servos to default position at 90 degrees
      s2.write(90);
      s3.write(90);

      Wire.begin(); // begins serial communication
      mpu.initialize(); // initializes mpu chip

      //taking input for the throttle bdlc motor
      pinMode(RCpin4, INPUT);
      // setting the hardware interrupt programm for two functions , PulseTimer2 and PulseTimer3
      pinMode(RCPin2, INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(RCpin2),PulseTimer2,Change);
      pinMode(RCPin3, INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(RCpin3),PulseTimer3,Change);
      delay(2000);

      pinMode(Motor, OUTPUT); // defining throttle motor pin
    }


    void loop(){
      // assuming plane moving in direction of x-axis and gravity acting on z-axis
      mpu.getAcceleration(&ax, &ay, &az); // gets raw values  of acceleration in three axis from mpu6050
      currentpitch = atan2(ax, az) * RAD_TO_DEG; // returns pitch value in degrees
      currentroll = atan2(az, ay) * RAD_TO_DEG; // returns roll value in degrees

      
      if (Pulses2 < 2000){ // removing noise from receiver signal  
        PulseWidth2 = Pulses2;
      }

      if (Pulses3 < 2000){ 
        PulseWidth3 = Pulses3;
      }
      //taking input from the throttle motor
      RCValue = pulseIn(RCpin, HIGH);

      desiredroll = map(PulseWidth2, 0, 2000, 0, 180); // maps roll value to 0-180 from receiver value
      desiredpitch = map(PulseWidth3, 0, 2000, 0, 180); // maps pitch value to 0-180 from receiver value
      ROLL = PIDfunc(currentroll, desiredroll); // returns value after PID
      PITCH = PIDfunc(currentpitch, desiredpitch); // returns value after PID
      s1Roll = map(ROLL, 0, 180, 0, 180); // maps function from PID to degrees for servo 
      s2Roll = -s1Roll;
      s3Pitch = map(PITCH, 0, 180, 0, 180);

      s1.write(s1Roll); // moves roll servo 1
      s2.write(s2Roll); // moves roll servo 2
      s3.write(s3Pitch); // moves pitch servo

      motorspeed = map(RCValue, 0, 2000, 0, 255); // maps motor speed to 0-255 from throttle value
      analogWrite(Motor, motorspeed); // moves propulsion motor
      delay(10);
    }


