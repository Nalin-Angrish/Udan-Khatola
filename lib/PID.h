class PIDController
{
private:
  unsigned long lastTime;
  double Setpoint;
  double integral, lastInput;
  double kp, ki, kd;
  double outMin, outMax;

public:
  PIDController(){};
  PIDController(double Kp, double Ki , double Kd , double outputMin , double outputMax )
  {// defining constructor for PID function
    outMin = outputMin;
    outMax = outputMax;
    kp = Kp;
    ki = Ki;
    kd = Kd;
    lastTime = millis();
  }
  void setLimit(double outputMin, double outputMax)
  {// defining function to set limits for PID function to prevent anti windup
    outMin = outputMin;
    outMax = outputMax;
  }
  void setTunings(double Kp, double Ki, double Kd)
  {// defining function to set tunings for PID function
    kp = Kp;
    ki = Ki;
    kd = Kd;
  }
  void setSetpoint(double setpoint)
  {// defining function to set target point for PID function
    Setpoint = setpoint;
  }
  double compute(double input)
  {
    //How long since we last calculated
    unsigned long now = millis();
    double timeChange = (double)(now - lastTime);

    //Compute all the working error variables
    double error = Setpoint - input;
    integral += (ki * error * timeChange);
    // integral windup guard
    if(integral> outMax) integral = outMax;
      else if(integral< outMin) integral = outMin;
    double dInput = (input - lastInput) / timeChange;

    //Remember some variables for next time
    lastInput = input;
    lastTime = now;

    //Compute PID Output
    double Output = kp * error + integral - kd * dInput;
    // PID output limit
    if(Output > outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;
    return Output;
  }
};

class CascadingPID
{
private:
  PIDController innerPID;
  PIDController outerPID;

public:
  CascadingPID(double innerKp, double innerKi, double innerKd, double outerKp, double outerKi, double outerKd, double innerOutputMin, double innerOutputMax, double outerOutputMin, double outerOutputMax)
  {
      innerPID = PIDController(innerKp, innerKi, innerKd, innerOutputMin, innerOutputMax);
      outerPID = PIDController(outerKp, outerKi, outerKd, outerOutputMin, outerOutputMax);
  }
  void setInnerTunings(double innerKp, double innerKi, double innerKd)
  {
      innerPID.setTunings(innerKp, innerKi, innerKd);
  }
  void setOuterTunings(double outerKp, double outerKi, double outerKd)
  {
      outerPID.setTunings(outerKp, outerKi, outerKd);
  }
  double compute(double outerSetpoint, double KalmanInput,double GyroInput)
  {   
      outerPID.setSetpoint(outerSetpoint);
      innerPID.setSetpoint(outerPID.compute(KalmanInput));
      return innerPID.compute(GyroInput);

  }
};