class Kalman
{   
private:
  float kalmanState;
  float kalmanUncertainty;
  float kalmanInputRate;
  float kalmanMeasurement;
  float kalmanControlMatrix;
  float kalmanGain;
public:
  Kalman(float kalmanUncertainty=0, float kalmanState=0, float kalmanControlMatrix=0.04)
  {
      kalmanState = kalmanState;
      kalmanUncertainty = kalmanUncertainty;
      kalmanControlMatrix = kalmanControlMatrix;
  }
  double compute(float kalmanInputRate, float kalmanMeasurement){
      kalmanState = kalmanState + (kalmanControlMatrix * kalmanInputRate);
      kalmanUncertainty = kalmanUncertainty + 0.04*0.04*4*4;
      kalmanGain = kalmanUncertainty/(kalmanUncertainty + 3*3);
      kalmanState = kalmanState + kalmanGain*(kalmanMeasurement - kalmanState);
      kalmanUncertainty = (1-kalmanGain)*kalmanUncertainty;
      return kalmanState;
  }
};
