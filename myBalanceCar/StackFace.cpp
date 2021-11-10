#include"StackFace.h"

StackFace::StackFace(){
  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.setBrightness(200);
  width = 320;
  height = 240;
}

void StackFace::normalFace(){
  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.fillCircle((int)width/4,(int)height/4,30,BLACK);
  M5.Lcd.fillCircle((int)width/4*3,(int)height/4,30,BLACK);
  M5.Lcd.fillRect((int)width/4-30,(int)height/4,61,(int)height/2, BLACK);
  M5.Lcd.fillRect((int)width/4*3-30,(int)height/4,61,(int)height/2, BLACK);
  M5.Lcd.fillCircle((int)width/4,(int)height/4*3,30,BLACK);
  M5.Lcd.fillCircle((int)width/4*3,(int)height/4*3,30,BLACK);
}

void StackFace::sleepFace(){
  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.fillRect((int)width/4-30,(int)height/2-10,61,20, BLACK);
  M5.Lcd.fillRect((int)width/4*3-30,(int)height/2-10,61,20, BLACK);  
}

void StackFace::eyeBlink(){
  sleepFace();
  delay(200);
  normalFace();
  delay(200);
  sleepFace();
  delay(200);
  normalFace();
}

void StackFace::wink(){
  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.fillCircle((int)width/4,(int)height/4,30,BLACK);
  M5.Lcd.fillCircle((int)width/4,(int)height/4*3,30,BLACK);
  M5.Lcd.fillRect((int)width/4-30,(int)height/4,61,(int)height/2, BLACK);
  M5.Lcd.fillRect((int)width/4*3-30,(int)height/2-10,61,20, BLACK);  
  delay(1000);
  normalFace();
}

void StackFace::halloween(){
  M5.Lcd.fillScreen(0xFDA0);//オレンジ色
  M5.Lcd.fillTriangle((int)width/4,(int)height/4-30,(int)width/8,(int)height/2-30,(int)width/8*3,(int)height/2-30,BLACK);
  M5.Lcd.fillTriangle((int)width/4*3,(int)height/4-30,(int)width/8*5,(int)height/2-30,(int)width/8*7,(int)height/2-30,BLACK);
  M5.Lcd.fillTriangle((int)width/2,(int)height/2-30,(int)width/2-30,(int)height/2,(int)width/2+30,(int)height/2,BLACK);
  M5.Lcd.fillTriangle((int)width/12,(int)height/2,(int)width/4,(int)height/8*7,(int)width/10*4,(int)height/4*3,BLACK);
  M5.Lcd.fillTriangle((int)width/12*11,(int)height/2,(int)width/4*3,(int)height/8*7,(int)width/10*6,(int)height/4*3,BLACK);
  M5.Lcd.fillTriangle((int)width/8*3,(int)height/2+20,(int)width/4,(int)height/8*7,(int)width/4*3,(int)height/8*7,BLACK);
  M5.Lcd.fillTriangle((int)width/8*5,(int)height/2+20,(int)width/4*3,(int)height/8*7,(int)width/4,(int)height/8*7,BLACK);
  M5.Lcd.fillTriangle((int)width/8*3+10,(int)height/8*6,(int)width/4,(int)height/8*7,(int)width/2+10,(int)height/8*7,0xFDA0);
  M5.Lcd.fillTriangle((int)width/8*5-10,(int)height/8*6,(int)width/4*3,(int)height/8*7,(int)width/2-10,(int)height/8*7,0xFDA0);
}
