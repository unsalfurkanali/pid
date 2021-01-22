/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID.h>

double Kp = 2, Ki = 5, Kd = 1;
PID myPID(Kp, Ki, Kd);

void setup() {
             // setpoint
}

void loop() {
  
}
