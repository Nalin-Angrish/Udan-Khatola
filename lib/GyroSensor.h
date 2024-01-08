#include<Wire.h>

class GyroSensor{
private:
  // MPU 6050 Register
  const int MPU = 0x68;
  // Error values for the accelerometer
  float AccErrorX, AccErrorY;
  // Time variables
  float elapsedTime, currentTime, previousTime;

public:
  struct AccelerometerData{
    // ==================== Accelerometer Data ==================== //
    float angleX;
    float angleY;
  };

  void setup(){
    // ==================== Initialize the MPU6050 Module ==================== //
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0);    // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
  }

  void calibrate(){
    // ==================== Calibrate the MPU6050 Module ==================== //
    // Setup the error values for the accelerometer
    AccErrorX = 0;
    AccErrorY = 0;
    for (int c=0; c < 200; c++){
      // Sum all readings
      AccelerometerData accData = getAccelerometerData();
      AccErrorX += accData.angleX;
      AccErrorY += accData.angleY;
    }
    // Take average and use negative sign to bring readings to 0 on ground
    AccErrorX /= -200;
    AccErrorY /= -200;
    Serial.print("AccErrorX: ");
    Serial.println(AccErrorX);
    Serial.print("AccErrorY: ");
    Serial.println(AccErrorY);
  }

  AccelerometerData getAccelerometerData(){
    // ==================== Obtain Accelerometer data from MPU6050 ==================== //
    Wire.beginTransmission(MPU);    // Begin transmission to the MPU-6050
    Wire.write(0x3B);               // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);    // Restart the transmission before reading the data
    Wire.requestFrom(MPU, 6, true); // Request 6 registers in total

    // Divide the raw values by 16384 for a range of +-2g, according to the datasheet
    float AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
    float AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
    float AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value

    // Calculating Roll and Pitch from the accelerometer data and the calculated error values
    AccelerometerData acc;
    acc.angleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI);
    acc.angleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI);
    return acc;
  }

  float pitch(){
    // ==================== Obtain Pitch from MPU6050 ==================== //
    return getAccelerometerData().angleX + AccErrorX;
  }

  float roll(){
    // ==================== Obtain Roll from MPU6050 ==================== //
    return getAccelerometerData().angleY + AccErrorY;
  }
};