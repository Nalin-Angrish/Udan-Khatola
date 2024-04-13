#include <ppm.h>

class ControlProps {
  public:
    float throttle = 0;
    float roll = 0;
    float pitch = 0;
    float yaw = 0;
    ControlProps(){}
    ControlProps(float t, float r, float p, float y){
      throttle = t;
      roll = r;
      pitch = p;
      yaw = y;
    }
};

class RFController
{
private:
  int PORT;
  float throttle = 0, roll = 0, pitch = 0, yaw = 0;
  float buffer;
public:
  RFController(float port){
    PORT = port;
  }
  void begin()
  {
    ppm.begin(PORT, false);
  }
  ControlProps readControls()
  {
    buffer = (ppm.read_channel(3) - 1000.0)/10;
    // if(buffer > 0.1 || buffer < -0.1) // Necessary to remove the noise
    //   throttle += buffer;
    throttle = constrain(buffer, 0, 100);

    buffer = map(ppm.read_channel(1), 1000, 2000, -90, 90);
    if (buffer-roll > 1 || buffer-roll < -1) // Necessary to remove the noise
      roll = buffer;
    buffer = map(ppm.read_channel(2), 1000, 2000, -40, 40);
    if (buffer-pitch > 1 || buffer-pitch < -1) // Necessary to remove the noise
      pitch = buffer;
    yaw = map(ppm.read_channel(4), 1000, 2000, -30, 30);
    return ControlProps(throttle, roll, pitch, yaw);
    // return ControlProps(ppm.read_channel(3), ppm.read_channel(1), ppm.read_channel(2), ppm.read_channel(4));
  }
};