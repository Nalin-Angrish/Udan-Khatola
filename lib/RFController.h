#include <PPMReader.h>

struct ControlProps {
  float throttle = 0;
  float roll = 0;
  float pitch = 0;
  float yaw = 0;
};

PPMReader ppm(34, 8);

class RFController
{
  private:
    const int PORT = 34;
    const int CHANNEL_COUNT = 8;
  
  public:
    // RFController()
    // {
    //   ppm = PPMReader(PORT, CHANNEL_COUNT);
    // }
    ControlProps readControls()
    {
      ControlProps props;
      props.throttle = ppm.latestValidChannelValue(1, 0);
      props.roll = ppm.latestValidChannelValue(2, 0);
      props.pitch = ppm.latestValidChannelValue(3, 0);
      props.yaw = ppm.latestValidChannelValue(4, 0);
      return props;
    }
};