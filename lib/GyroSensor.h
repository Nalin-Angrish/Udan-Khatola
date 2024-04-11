#include<Wire.h>
// MPU 6050 Register
#define MPU 0x68

class GyroSensor{
private:
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

  struct GyroscopeData{
    // ==================== Gyroscope Data ==================== //
    float rateX;
    float rateY;
    float rateZ;
  };

  void setup(){
    // ==================== Initialize the MPU6050 Module ==================== //
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0);    // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);

    // Set the range of the gyroscope to +- 250 degrees per second
    Wire.beginTransmission(MPU);
    Wire.write(0x1B); // GYRO_CONFIG register
    Wire.write(0b00011000); // Set the range to +- 250 degrees per second
    Wire.endTransmission(true);
  }

  void calibrate(){
    // ==================== Calibrate the MPU6050 Module ==================== //
    // Setup the error values for the accelerometer
    AccErrorX = 0;
    AccErrorY = 0;
    AccelerometerData accData;
    for (int c=0; c < 200; c++){
      // Sum all readings
      accData = getAccelerometerData();
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

  GyroscopeData getGyroscopeData(){
    // ==================== Obtain Gyroscope data from MPU6050 ==================== //
    Wire.beginTransmission(MPU);    // Begin transmission to the MPU-6050
    Wire.write(0x43);               // Start with register 0x43 (GYRO_XOUT_H)
    Wire.endTransmission(false);    // Restart the transmission before reading the data
    Wire.requestFrom(MPU, 6, true); // Request 6 registers in total

    // Divide the raw values by 131 for a range of +-250 degrees per second, according to the datasheet
    GyroscopeData gyro;
    gyro.rateX = (Wire.read() << 8 | Wire.read()) / 131.0; // X-axis value
    gyro.rateY = (Wire.read() << 8 | Wire.read()) / 131.0; // Y-axis value
    gyro.rateZ = (Wire.read() << 8 | Wire.read()) / 131.0; // Z-axis value
    return gyro;
  }

  float pitch(){
    // ==================== Obtain Pitch from MPU6050 ==================== //
    return getAccelerometerData().angleX + AccErrorX;
  }

  float roll(){
    // ==================== Obtain Roll from MPU6050 ==================== //
    return getAccelerometerData().angleY + AccErrorY;
  }

  float yaw(){
    // ==================== Obtain Yaw from MPU6050 ==================== //
    return getGyroscopeData().rateZ;
  }
};