#define M5STACK_MPU6886 

#include<M5Stack.h>
#include<Kalman.h>
#include <BlynkSimpleEsp32_BLE.h>
#include"myDCMotor.h"
#include"StackFace.h"

char blynkAuth[] = "bOXpYvGF9jdvCKSCoyoHviOhijrOQQaT";

#define MOTOR_POWER_MAX    255
#define MOTOR_POWER_MIN    50

//x, y, zの順
float acc[3];
float accOffset[3];
float gyro[3];
float gyroOffset[3];
float kalAngleX;
float dt;
Kalman kalmanX;
long lastMs = 0;
long tick = 0;

//moter
const int IN1_left = 3;
const int IN2_left = 5;
const int IN1_right = 26;
const int IN2_right = 25;
myDCMotor DCmotor(IN1_left,IN2_left,IN1_right,IN2_right);


//encoder
const uint8_t ENC_IN1 = 16;
const uint8_t ENC_IN2 = 17;
const uint8_t ENC_IN3 = 35;
const uint8_t ENC_IN4 = 36;

volatile int  enc_count1;
volatile int  enc_count2;
byte pos1;
byte pos2;


//IMU
float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;
float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;

//PID
float KP = 50.0;
float KI = 4.0;
float KD = 2.0;

float power=0,I=0,preP=0,preTime,Target=90,Dire;

bool start = false;
StackFace face;

BLYNK_WRITE(V0){
  KP = param.asInt();
}

BLYNK_WRITE(V1){
  KI = param.asInt();
}

BLYNK_WRITE(V2){
  KD = param.asInt();
}

/*BLYNK_WRITE(V3){
  MOTOR_POWER_MIN = param.asInt();
}*/

void calibration(){
  //補正値を求める
  float gyroSum[3];
  float accSum[3];
  for(int i = 0; i < 500; i++){
    readGyro();
    gyroSum[0] += gyro[0];
    gyroSum[1] += gyro[1];
    gyroSum[2] += gyro[2];
    accSum[0] += acc[0];
    accSum[1] += acc[1];
    accSum[2] += acc[2];
    delay(2);
  }
  gyroOffset[0] = gyroSum[0]/500;
  gyroOffset[1] = gyroSum[1]/500;
  gyroOffset[2] = gyroSum[2]/500;
  accOffset[0] = accSum[0]/500;
  accOffset[1] = accSum[1]/500;
  accOffset[2] = accSum[2]/500 - 1.0;//重力加速度1G
}

void readGyro(){
  M5.IMU.getGyroData(&gyro[0], &gyro[1], &gyro[2]);
  M5.IMU.getAccelData(&acc[0], &acc[1], &acc[2]);
}
void applyCalibration(){
  gyro[0] -= gyroOffset[0];
  gyro[1] -= gyroOffset[1];
  gyro[2] -= gyroOffset[2];
  acc[0] -= accOffset[0];
  acc[1] -= accOffset[1];
  acc[2] -= accOffset[2];
}
float getRoll(){
  return atan2(acc[1], acc[2]) * RAD_TO_DEG;
}
float getPitch(){
  return atan(-acc[0] / sqrt(acc[1]*acc[1] + acc[2]*acc[2])) * RAD_TO_DEG;
}

void setup() {
  // put your setup code here, to run once:
  M5.begin();
  M5.Power.begin();
  M5.Lcd.setTextSize(3);
  //IMUキャリブレーション＆設定
  M5.IMU.Init();
  M5.Lcd.println("Calibrating...");  
  calibration();
  readGyro();
  kalmanX.setAngle(getRoll());
  delay(500);
  Serial.begin(9600);

  //ロータリーエンコーダーPinの設定
  pinMode(ENC_IN1, INPUT_PULLUP);
  pinMode(ENC_IN2, INPUT_PULLUP);
  pinMode(ENC_IN3, INPUT_PULLUP);
  pinMode(ENC_IN4, INPUT_PULLUP);
  
  //エンコーダーの割込み設定
  /*attachInterrupt(digitalPinToInterrupt(ENC_IN1), ENC_READ1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_IN2), ENC_READ1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_IN3), ENC_READ2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_IN4), ENC_READ2, CHANGE);*/

  M5.Lcd.setTextSize(3);

  preTime = micros();
  //Blynk設定
  Blynk.setDeviceName("M5Stack");
  Blynk.begin(blynkAuth);
  face.normalFace();
}

void loop() {
  // put your main code here, to run repeatedly:
  M5.update();
  Blynk.run();
  float pitch,roll,yaw,Duty,P,D,now,dt,Time;
  /*m5.Lcd.clear();
  m5.Lcd.setCursor(10,10);
  m5.Lcd.print(enc_count1);
  m5.Lcd.print(':');
  m5.Lcd.println(enc_count2);*/

  readGyro();
  applyCalibration();
  roll = getRoll();
  pitch = getPitch();

  Time=micros();
  dt = (Time - preTime) / 1000000.0;
  preTime = Time;
  kalAngleX  = kalmanX.getAngle(roll,gyro[0],dt);
  now = Target - kalAngleX;
  //m5.Lcd.print("roll:");
  //m5.Lcd.println(kalAngleX);
  Serial.println(kalAngleX);


   if(-30 < now && now < 30 && start){
    P = now/90;
    I += P*dt;
    D = (P - preP)/dt;
    preP = P;

    if(100 < abs(I*KI)){
      I = 0;  
    }
    power = KP * P + KI * I + D * KD;

    Duty = (int)(MOTOR_POWER_MAX - MOTOR_POWER_MIN)* power/10; 
    DCmotor.Drive(Duty);
  } else {  // +-20度を越えたら倒れたとみなす
    power = 0;
    I = 0;
    DCmotor.Drive(-1);
  }

  if(m5.BtnA.wasPressed()){
    if(!start)start = true;
    else start = false;
      
  }
}

void ENC_READ1() {
  byte cur = (!digitalRead(ENC_IN2) << 1) + !digitalRead(ENC_IN1);
  byte old = pos1 & B00000011;
  byte dir = (pos1 & B00110000) >> 4;
 
  if (cur == 3) cur = 2;
  else if (cur == 2) cur = 3;
 
  if (cur != old)
  {
    if (dir == 0)
    {
      if (cur == 1 || cur == 3) dir = cur;
    } else {
      if (cur == 0)
      {
        if (dir == 1 && old == 3) enc_count1+= 36;
        else if (dir == 3 && old == 1) enc_count1-=36;
        dir = 0;
      }
    }
 
    bool rote = 0;
    if (cur == 3 && old == 0) rote = 0;
    else if (cur == 0 && old == 3) rote = 1;
    else if (cur > old) rote = 1;
 
    pos1 = (dir << 4) + (old << 2) + cur;
  }
}

void ENC_READ2() {
  byte cur = (!digitalRead(ENC_IN3) << 1) + !digitalRead(ENC_IN4);
  byte old = pos2 & B00000011;
  byte dir = (pos2 & B00110000) >> 4;
 
  if (cur == 3) cur = 2;
  else if (cur == 2) cur = 3;
 
  if (cur != old)
  {
    if (dir == 0)
    {
      if (cur == 1 || cur == 3) dir = cur;
    } else {
      if (cur == 0)
      {
        if (dir == 1 && old == 3) enc_count2+= 36;
        else if (dir == 3 && old == 1) enc_count2-=36;
        dir = 0;
      }
    }
 
    bool rote = 0;
    if (cur == 3 && old == 0) rote = 0;
    else if (cur == 0 && old == 3) rote = 1;
    else if (cur > old) rote = 1;
 
    pos2 = (dir << 4) + (old << 2) + cur;
  }
}
