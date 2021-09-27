#define M5STACK_MPU6886 

#include<M5Stack.h>
#include<Kalman.h>
#include <BlynkSimpleEsp32_BLE.h>

char blynkAuth[] = "bOXpYvGF9jdvCKSCoyoHviOhijrOQQaT";

#define MOTOR_POWER_MIN    50
#define MOTOR_POWER_MAX    255

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
const int IN1_right = 25;
const int IN2_right = 26;

const double PWM_Hz = 312500;
const uint8_t PWM_level = 8;
const uint8_t PWM_CH1 = 1;
const uint8_t PWM_CH2 = 2;
const uint8_t PWM_CH3 = 3;
const uint8_t PWM_CH4 = 4;

//encoder
const uint8_t ENC_IN1 = 16;
const uint8_t ENC_IN2 = 17;
const uint8_t ENC_IN3 = 35;
const uint8_t ENC_IN4 = 36;

volatile byte pos1;
volatile int  enc_count1;
volatile byte pos2;
volatile int enc_count2;

int count_temp1;
int count_temp2;

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
float KP = 200.0;
float KI = 4.0;
float KD = 2.0;

float power=0,I=0,preP=0,preTime,Target=0,Dire;


//TDT_sSprite sprite(&M5.Lcd);



void DCmoter(float power){
  if(power > 0){
    power += MOTOR_POWER_MIN;
    if(power > 255)power = 255;
    ledcWrite(PWM_CH1,0);
    ledcWrite(PWM_CH2,power);
    ledcWrite(PWM_CH3,0);
    ledcWrite(PWM_CH4,power);       
  }else if(power < 0){
    power *= -1;
    power += MOTOR_POWER_MIN;
    if(power > 255)power = 255;
    ledcWrite(PWM_CH1,power);
    ledcWrite(PWM_CH2,0);
    ledcWrite(PWM_CH3,power);
    ledcWrite(PWM_CH4,0);    
  }else if(power == 0){
    ledcWrite(PWM_CH1,255);
    ledcWrite(PWM_CH2,255);
    ledcWrite(PWM_CH3,255);
    ledcWrite(PWM_CH4,255);      
  }
}

BLYNK_WRITE(V0){
  KP = param.asInt();
}

BLYNK_WRITE(V1){
  KI = param.asInt();
}

BLYNK_WRITE(V2){
  KD = param.asInt();
}

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
  delay(500);  
  calibration();
  readGyro();
  kalmanX.setAngle(getRoll());
  lastMs = micros();
  Serial.begin(9600);
  
  //DCモーターPinの設定
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
  
  //ロータリーエンコーダーPinの設定
  pinMode(ENC_IN1, INPUT_PULLUP);
  pinMode(ENC_IN2, INPUT_PULLUP);
  pinMode(ENC_IN3, INPUT_PULLUP);
  pinMode(ENC_IN4, INPUT_PULLUP);
  
  //エンコーダーの割込み設定
  attachInterrupt(digitalPinToInterrupt(ENC_IN1), ENC_READ1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_IN2), ENC_READ1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_IN3), ENC_READ2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_IN4), ENC_READ2, CHANGE);

  M5.Lcd.setTextSize(3);

  preTime = micros();
  //Blynk設定
  Blynk.setDeviceName("M5Stack");
  Blynk.begin(blynkAuth);
  M5.Lcd.clear();
}

void loop() {
  // put your main code here, to run repeatedly:
  M5.update();
  Blynk.run();
  float pitch,roll,yaw,Duty,P,D,now,dt,Time;
  m5.Lcd.clear();
  m5.Lcd.setCursor(10,10);
  m5.Lcd.print(enc_count1);
  m5.Lcd.print(':');
  m5.Lcd.println(enc_count2);

  readGyro();
  applyCalibration();
  dt = (micros() - lastMs) / 1000000.0;
  Time=micros();
  preTime = Time;
  lastMs = micros();
  roll = getRoll();
  pitch = getPitch();
  
  now = Target - roll;
  m5.Lcd.print("Roll:");
  m5.Lcd.println(roll);

  if(-40 < now && now < 40){
    P = now/90;
    I += P*dt;
    D = (P - preP)/dt;
    preP = P;

    if(100 < abs(I*KI)){
      I = 0;  
    }
    power = KP * P + D * KD + KI * I;

    Duty = (int)(MOTOR_POWER_MAX - MOTOR_POWER_MIN)* power/10; 
    DCmoter(Duty);
  } else {  // +-20度を越えたら倒れたとみなす
    power = 0;
    I = 0;
    DCmoter(0);
  }
  m5.Lcd.print("Power:");
  m5.Lcd.println(power);
  m5.Lcd.print("KP:");
  m5.Lcd.println(KP);
  m5.Lcd.print("KD:");
  m5.Lcd.println(KD);
  m5.Lcd.print("KI:");
  m5.Lcd.println(KI);
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
