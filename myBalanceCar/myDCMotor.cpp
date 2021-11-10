#include"myDCMotor.h"

#define MOTOR_POWER_MAX    255
#define MOTOR_POWER_MIN    50

const double PWM_Hz = 312500;
const uint8_t PWM_level = 8;
const uint8_t PWM_CH1 = 1;
const uint8_t PWM_CH2 = 2;
const uint8_t PWM_CH3 = 3;
const uint8_t PWM_CH4 = 4;

myDCMotor::myDCMotor(int IN1_left,int IN2_left,int IN1_right,int IN2_right){
  pinMode(IN1_right,OUTPUT);
  pinMode(IN2_right,OUTPUT);
  pinMode(IN1_left,OUTPUT);
  pinMode(IN2_left,OUTPUT);
  ledcSetup(PWM_CH1,PWM_Hz,PWM_level);
  ledcSetup(PWM_CH2,PWM_Hz,PWM_level);
  ledcSetup(PWM_CH3,PWM_Hz,PWM_level);
  ledcSetup(PWM_CH4,PWM_Hz,PWM_level);
  ledcAttachPin(IN1_right,PWM_CH1);
  ledcAttachPin(IN2_right,PWM_CH2);
  ledcAttachPin(IN1_left,PWM_CH3);
  ledcAttachPin(IN2_left,PWM_CH4);  
}

void myDCMotor::Drive(float power){
    if(power > 0){
    power += MOTOR_POWER_MIN;
    if(power > MOTOR_POWER_MAX)power = 255;
    ledcWrite(PWM_CH1,power);
    ledcWrite(PWM_CH2,0);
    ledcWrite(PWM_CH3,power);
    ledcWrite(PWM_CH4,0);       
  }else if(power < 0){
    power *= -1;
    power += MOTOR_POWER_MIN;
    if(power > MOTOR_POWER_MAX)power = 255;
    ledcWrite(PWM_CH1,0);
    ledcWrite(PWM_CH2,power);
    ledcWrite(PWM_CH3,0);
    ledcWrite(PWM_CH4,power);    
  }else if(power == 0){
    ledcWrite(PWM_CH1,255);
    ledcWrite(PWM_CH2,255);
    ledcWrite(PWM_CH3,255);
    ledcWrite(PWM_CH4,255);      
  }else if(power == -1){
    ledcWrite(PWM_CH1,0);
    ledcWrite(PWM_CH2,0);
    ledcWrite(PWM_CH3,0);
    ledcWrite(PWM_CH4,0);     
  }
}
