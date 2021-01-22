# Arduino PID controller library
# @author : Ali Unsal 

This library help to you use PID controller with Arduino Boards
## Usage

```c++
#include <PID.h>

double Kp = 2, Ki = 5, Kd = 1;
PID myPID(Kp, Ki, Kd);

// Defined Functions: 
// void set_SetPoint(double newSetPoint);
// void setKp(double Kp);
// void setKi(double Ki);
// void setKd(double Kd);
// double compute(double current); 
// void setSamplingTime(double sampleTime);
// double getLastError(void);
// void setMinOut(double newMin);
// void setMaxOut(double newMax);
// void setWindUp(double newWindUp);
// void clearParam(void);
// void initParam(double integratorWindUp, double maxOut, double minOut);

void setup() {

}

void loop() {
  
}
```

I've added a few things, but code written with the old library should still work
after changing `#include` line.

## Authors

* Ali Unsal
