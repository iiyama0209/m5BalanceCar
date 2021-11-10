#ifndef myDC_MOTOR
#define myDC_MOTOR

#include <Arduino.h>
#include<M5Stack.h>

class myDCMotor{
  public:
    myDCMotor(int IN1_left,int IN2_left,int IN1_right,int IN2_right);
    void Drive(float power);
};


#endif
