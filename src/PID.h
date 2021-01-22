#ifndef PID_h
#define PID_h

#include "Arduino.h"

class PID
{
private:
    double Kp, Ki, Kd;
    double sampling_time = 0.0;
    double currentTime = 0;
    double lastTime = 0;
    double setPoint = 0;

    double error = 0;
    double lastError = 0;
    
    double deltaTime = 0;
    double deltaError = 0;

    double errorArea = 0;
    double integratorWindUp = 50;

    double maxOut = 100;
    double minOut = -100;

public:
    PID(double Kp, double Ki, double Kd);
    void set_SetPoint(double newSetPoint);
    void setKp(double Kp);
    void setKi(double Ki);
    void setKd(double Kd);
    double compute(double current); 
    void setSamplingTime(double sampleTime);
    double getLastError(void);
    void setMinOut(double newMin);
    void setMaxOut(double newMax);
    void setWindUp(double newWindUp);
    void clearParam(void);
    void initParam(double integratorWindUp, double maxOut, double minOut);
    
    
    
};



#endif // PIDLibrary
