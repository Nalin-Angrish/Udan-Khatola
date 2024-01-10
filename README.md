# Udan Khatola
This project is our submission for a competition organized by the Aeromodelling Club, Indian Institute of Technology Ropar from December 2023 to January 2024.

## The Problem Statement
We were given the task of coding a PID control system for self-leveling of the plane in the pitch and roll axis, as well maintaining altitude when given command from the transmitter. It was assumed to use the Arduino Uno microcontroller.

## The Solution
For this project, we implemented I2C communication for obtaining accelerometer data from the MPU6050 Module. This data was used to turn the servos for ailerons and elevators appropriately in order to correct the pitch and roll caused by external factors. The servos also employ a PID controller to regulate these values.  
  
For the second part of the project we used the MS5611 module to obtain the altitude relative to the altitude at the time of arduino boot. We were planning to implement Kalman filter or some other complementary filter to bring more precision in this but due to time constraints were unable to do this.  
  
The complete submission document is also included in the repository.

## File Structure
The root folder of the project consists of the following files:
- `docs`:
  - `Aero_assign_dec.pdf`: The problem statement for the competition.
  - `Altitute Measurement from Pressure Sensor.pdf`: A reference document for studying how we could use the MS5611 module to measure relative altitude.
  - `MPU-6050-Datasheet-Register-Map.pdf`: Datasheet for the MPU6050 module.
- `lib`:
  - `Barometer.h`: Abstraction layer for using the barometer module.
  - `Gyrosensor.h`: Abstraction layer for using the MPU6050 Accelerometer/Gyroscope module.
  - `PID.h`: Custom implementation of PID Controller.
- `.gitignore`: A list of files to exclude from git. These are just vscode configuration files as of 10 Jan, 2024.
- `Udan Khatola.ino`: The main code for running on the Arduino board.

## Code explanation
The code is very intuitive and well commented so that it is easy to understand. Here's a summary of the code and workflow:
#### lib/Barometer.h
![barometer](https://github.com/Nalin-Angrish/Udan-Khatola/assets/54469875/35d22c62-60c8-426e-a516-38f640cde2f5)
#### lib/GyroSensor.h
![gyrosensor](https://github.com/Nalin-Angrish/Udan-Khatola/assets/54469875/b97d3513-9059-40fa-8a93-c0c6339b286e)
#### lib/PID.h
![pid](https://github.com/Nalin-Angrish/Udan-Khatola/assets/54469875/7f5af4f1-6dc9-444a-804e-a2a3cd98af0b)
#### Udan Khatola.ino
![arduino](https://github.com/Nalin-Angrish/Udan-Khatola/assets/54469875/c81448eb-c0b9-4d79-a6ec-cf4cec96f3c6)

## Third Party Libraries used
The code uses mostly the standard arduino library for all its tasks, but a few features could be better implemented using libraries available on arduino's library manager.
- The communication with the MPU6050 module has been established through the inbuilt `Wire.h` library.
- Servo motors are controlled using the `Servo.h` library by `Michael Margolis, Arduino`.
- The communication with the MS5611 module has been made using the `MS5611.h` library by `Rob Tillaart`.
  
## Learning resources
We were all beginners and had very limited knowledge in this field. For learning all we needed to build this project, we used the following resources:
- [DIY Gimbal | Arduino and MPU6050 Tutorial](https://youtu.be/UxABxSADZ6U?si=g_daL5vl091i2JGe)
- [What Is PID Control? | Understanding PID Control, Part 1](https://youtu.be/wkfEZmsQqiA?si=tAEa2qYFN7nJQBxK)
- [Anti-windup for PID control | Understanding PID Control, Part 2](https://youtu.be/NVLXCwc8HzM?si=QW_AkcbqYHsvaD8n)
- [Improving the Beginner’s PID – Introduction](http://tinyurl.com/pidtut)

## The Team
The team was of 3 first year undergraduate students at Indian Institute of Technology Ropar:
- Nalin Angrish (2023MEB1360)    
    GitHub: [Nalin-Angrish](https://github.com/Nalin-Angrish)
- Somya Katoch (2023MEB1384)    
    GitHub: [somyakatoch](https://github.com/somyakatoch)
- Abhigyan Singh (2023MEB1320)    
    GitHub: [Abhigyann-Singh](https://github.com/Abhigyann-Singh)
