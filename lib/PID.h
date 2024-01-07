class PIDController
{
private:
  unsigned long lastTime;
  double Setpoint;
  double integral, lastInput;
  double kp, ki, kd;
  double outMin, outMax;

public:
  PIDController(double Kp, double Ki, double Kd, double outputMin, double outputMax)
  {
    outMin = outputMin;
    outMax = outputMax;
    kp = Kp;
    ki = Ki;
    kd = Kd;
    lastTime = millis();
  }
  void setLimit(double outputMin, double outputMax)
  {
    outMin = outputMin;
    outMax = outputMax;
  }
  void setTunings(double Kp, double Ki, double Kd)
  {
    kp = Kp;
    ki = Ki;
    kd = Kd;
  }
  void setSetpoint(double setpoint)
  {
    Setpoint = setpoint;
  }
  double compute(double input)
  {
    /*How long since we last calculated*/
    unsigned long now = millis();
    double timeChange = (double)(now - lastTime);

    /*Compute all the working error variables*/
    double error = Setpoint - input;
    integral += (ki * error * timeChange);
    if(integral> outMax) integral = outMax;
      else if(integral< outMin) integral = outMin;
    double dInput = (input - lastInput) / timeChange;

    /*Remember some variables for next time*/
    lastInput = input;
    lastTime = now;

    /*Compute PID Output*/
    double Output = kp * error + integral - kd * dInput;
    if(Output > outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;
    return Output;
  }
};