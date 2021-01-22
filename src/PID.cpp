// !/usr/bin/g++

// Copyright (C) 2021 Ali Unsal

// title           :pidLibrary.cpp
// description     :PID Controller Algorithm on C++ for Arduino Boards
// author          :Ali Unsal
// date            :20210122
// version         :0.1
// notes           :

// ==============================================================================


#include "PID.h"


PID::PID(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;


    this->currentTime = millis();
    this->lastTime = this->currentTime;

}

void PID::set_SetPoint(double newSetPoint) {
    this->setPoint = newSetPoint;
}

/*  compute(...)**************************************************************
 *  This function will be calculate the new value for the control with respect
 *  to current value, proportional gain, integral gain and derivative gain. 
 *  These calculations will look like the equations below
 *  error = setPoint - currentPoint
 *  slope = dError / dTime
 *  Area = Area + error * dt
 *  New Val = Kp * error + Ki * [Area + error] + Kd * [slope]
 ****************************************************************************/
double PID::compute(double current) {
    this->currentTime = millis();
    this->deltaTime = this->currentTime - this->lastTime;
    this->lastError = this->error;
    this->error = this->setPoint - current;
    this->deltaError = this->error - this->lastError;
    if (this->deltaTime >= this->sampling_time) {
        double errorSlope = (this->deltaError)/(this->deltaTime);
        this->errorArea = this->errorArea + this->error * this->deltaTime;
        
        //Integrator windup control
        if (this->errorArea > this->integratorWindUp ) this->errorArea = this->integratorWindUp;
        else if (this->errorArea < -this->integratorWindUp ) this->errorArea = -this->integratorWindUp;


        double computed = this->Kp * this->error + this->Ki * this->errorArea + this->Kd * errorSlope;

        //Max and Min out limit control
        if (computed > this->maxOut) computed = this->maxOut;
        else if (computed < this->minOut) computed = this->minOut;

        return computed;
    }
    else return false;

}

/*  setKp(...)****************************************************************
 *  This function will be change the Proportional gain coefficient. 
 ****************************************************************************/
void PID::setKp(double Kp) {
    this->Kp = Kp;
}

/*  setKi(...)****************************************************************
 *  This function will be change the Integral gain coefficient. 
 ****************************************************************************/
void PID::setKi(double Ki) {
    this->Ki = Ki;
}

/*  setKd(...)****************************************************************
 *  This function will be change the Derivative gain coefficient. 
 ****************************************************************************/
void PID::setKd(double Kd) {
    this->Kd = Kd;
}


/*  setSamplingTime(...)******************************************************
 *  This function will be change sampling time or delta time limit.  
 ****************************************************************************/
void PID::setSamplingTime(double sampleTime) {
    this->sampling_time = sampleTime;
}

/*  getLastError(...)*********************************************************
 *  This function will be change sampling time or delta time limit.  
 ****************************************************************************/
double PID::getLastError(void) {
    return this->error;
}

void PID::setWindUp(double newWindUp) {
    this->integratorWindUp = newWindUp;
}

void PID::setMaxOut(double newMax) {
    this->maxOut = newMax;
}

void PID::setMinOut(double newMin) {
    this->minOut = newMin;
}

void PID::clearParam(void) {
    this->error = 0;
    this->lastError = 0;
    this->deltaTime = 0;
    this->deltaError = 0;
    this->currentTime = millis();
}

void PID::initParam(double integratorWindUp, double maxOut, double minOut) {
    this->maxOut = maxOut;
    this->minOut = minOut;
    this->integratorWindUp = integratorWindUp;
}




