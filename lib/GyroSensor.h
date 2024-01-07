#include<Wire.h>


class GyroSensor{
private:
  const int MPU = 0x68;
  float AccErrorX, AccErrorY, GyroErrorZ;
  float gyroAngleZ;
  float elapsedTime, currentTime, previousTime;

public:
  struct AccelerometerData{
    float angleX;
    float angleY;
  };

  void setup(){
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0);    // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
  }

  void calibrate(){
    AccErrorX = 0;
    AccErrorY = 0;
    GyroErrorZ = 0;
    int c=0;
    while (c < 200){
      AccelerometerData accData = getAccelerometerData();
      // Sum all readings
      AccErrorX += accData.angleX;
      AccErrorY += accData.angleY;
      c++;
    }
    c = 0;
    while (c < 200){
      GyroErrorZ += getGyroscopeData(false);
      c++;
    }
    AccErrorX /= 200;
    AccErrorY /= 200;
    GyroErrorZ /= 200;
    Serial.print("AccErrorX: ");
    Serial.println(AccErrorX);
    Serial.print("AccErrorY: ");
    Serial.println(AccErrorY);
    Serial.print("GyroErrorZ: ");
    Serial.println(GyroErrorZ);
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

  float getGyroscopeData(bool considerError){
    // ==================== Obtain Gyro data (only z axis) from MPU6050 ==================== //
    previousTime = currentTime;                        // Previous time is stored before the actual time read
    currentTime = millis();                            // Current time actual time read
    elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

    Wire.beginTransmission(MPU); // Begin transmission to the MPU-6050
    Wire.write(0x47);            // Start with register 0x47 (GYRO_ZOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 2, true); // Request 2 registers in total

    // Divide the raw values by 131 for a range of +-250deg/s, according to the datasheet
    // Also adjust the calculated error values
    float errorToConsider = 0;
    if(considerError) errorToConsider = GyroErrorZ;
    float GyroZ = ((Wire.read() << 8 | Wire.read()) / 131.0) + errorToConsider; // Z-axis value

    // Get the new values for the angles from the gyro
    gyroAngleZ = gyroAngleZ + GyroZ * elapsedTime;
    return gyroAngleZ;
  }

  float pitch(){
    return getAccelerometerData().angleX + AccErrorX;
  }

  float roll(){
    return getAccelerometerData().angleY + AccErrorY;
  }
  
  float yaw(){
    return getGyroscopeData(true);
  }
};