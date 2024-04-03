# Udan Khatola
This project is being coordinated by the Aeromodelling Club, Indian Institute of Technology Ropar. The aim of this project is to design and build an RC plane from scratch. This documentation might not be up-to-date with the code for some time till the project has been successfully completed.

For this project, we are using the ESP32 Dev board as our flight controller. We implemented I2C communication for obtaining accelerometer data from the MPU6050 Module. This data was used to turn the servos for ailerons and elevators appropriately in order to correct the pitch and roll caused by external factors. The servos also employ a PID controller to regulate these values.  

This project is based on our submission for a competition organized by the Aeromodelling Club, Indian Institute of Technology Ropar from December 2023 to January 2024. The submission has been preserved in another branch of its own. To view the submission code and documentation, check out the [submission](https://github.com/Nalin-Angrish/Udan-Khatola/tree/submission) branch.

## File Structure
The root folder of the project consists of the following files:
- `.vscode/arduino.json`: Configuration for the Arduino extension on VS Code.
- `docs`: References
- `lib`:
  - `Gyrosensor.h`: Abstraction layer for using the MPU6050 Accelerometer/Gyroscope module.
  - `Kalman.h`: The kalman filter.
  - `PID.h`: Custom implementation of PID Controller.
  - `RFController.h`: Abstraction layer for reading and interpreting controller signals.
- `model`:
  - `Model.blend`: Blender model for the skeletal structure of our RC Plane design.
  - `Model.png`: A rendered image of the design.
- `.gitignore`: A list of files to exclude from git. These are just VS Code configuration files.
- `TODO.md`: The task planning and distribution.
- `Udan-Khatola.ino`: The main code for running on the ESP32 board.

## Code explanation
The code is very intuitive and well commented so that it is easy to understand. Here's a summary of the code and workflow:
#### lib/GyroSensor.h
![gyrosensor](https://github.com/Nalin-Angrish/Udan-Khatola/assets/54469875/b97d3513-9059-40fa-8a93-c0c6339b286e)
#### lib/PID.h
![pid](https://github.com/Nalin-Angrish/Udan-Khatola/assets/54469875/7f5af4f1-6dc9-444a-804e-a2a3cd98af0b)
#### Udan Khatola.ino
![arduino](https://github.com/Nalin-Angrish/Udan-Khatola/assets/54469875/c81448eb-c0b9-4d79-a6ec-cf4cec96f3c6)

## Third Party Libraries used
The code uses mostly the standard arduino library for all its tasks, but a few features could be better implemented using libraries available on arduino's library manager.
- The communication with the MPU6050 module has been established through the inbuilt `Wire.h` library.
- Servo motors are controlled using the `ESP32Servo.h` library by `John K. Bennett, Kevin Harrington`.
- PPM signal is taken from FS-i6X receiver module using the `PPM-reader` library by `Aapo Nikkilä, Dmitry Grigoryev`.
  
## The Team
The team consisted of first year undergraduate students at Indian Institute of Technology Ropar.  
Code contributors:
- Nalin Angrish ([Nalin-Angrish](https://github.com/Nalin-Angrish))
- Abhigyan Singh ([Abhigyann-Singh](https://github.com/Abhigyann-Singh))
- Aman Mittal ([SikandarAman](https://github.com/SikandarAman))
- Pradhyuman ([Pradhyu18](https://github.com/Pradhyu18))  
  
Design and Assembly contributors:
- Arjun Maindarkar ([programmingmaster69](https://github.com/programmingmaster69))
- Ponnathavan S ([Ponnathavan](https://github.com/Ponnathavan))
- Kshitij Kumar ([Kshitijsqdn303](https://github.com/Kshitijsqdn303))  
- Sarthak Gupta ([sarthakGIT0305](https://github.com/sarthakGIT0305))  
- All code contributors

And a previous contributor:
- Somya Katoch ([somyakatoch](https://github.com/somyakatoch))