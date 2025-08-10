#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define Y0 16
#define step 250

//servos
#define front_right_up 11
#define front_right_down 14

#define front_left_up 10
#define front_left_down 15

#define back_right_up 7
#define back_right_down 2

#define back_left_up 6
#define back_left_down 3

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setServo(int servo, int angle){
  pwm.setPWM(servo, 0, map(angle, 0, 180, 102, 512));
}

void moveLeg_back_left(float x, float y){

  float a = 8;
  float b = 10;
  float y0 = Y0-2;
  float c = x*x + (y0 - y)*(y0 - y);
  float alfa = acos((c + a*a - b*b)/2/a/sqrt(c)) + acos(x/sqrt(c));
  float beta = acos((a*a + b*b - c)/2/a/b);

  setServo(back_left_up, alfa*180/PI);
  setServo(back_left_down, beta*180/PI);
}

void moveLeg_front_left(float x, float y){

  float a = 8;
  float b = 10;
  float y0 = Y0;
  float c = x*x + (y0 - y)*(y0 - y);
  float alfa = acos((c + a*a - b*b)/2/a/sqrt(c)) + acos(x/sqrt(c));
  float beta = acos((a*a + b*b - c)/2/a/b);

  setServo(front_left_up, alfa*180/PI);
  setServo(front_left_down, beta*180/PI);
}

void moveLeg_front_right(float x, float y){

  float a = 8;
  float b = 10;
  float y0 = Y0;
  float c = x*x + (y0 - y)*(y0 - y);
  float alfa = acos((c + a*a - b*b)/2/a/sqrt(c)) + acos(x/sqrt(c));
  float beta = acos((a*a + b*b - c)/2/a/b);

  setServo(front_right_up, 180 - alfa*180/PI);
  setServo(front_right_down, 180 - beta*180/PI);
}

void moveLeg_back_right(float x, float y){

  float a = 8;
  float b = 10;
  float y0 = Y0-2;
  float c = x*x + (y0 - y)*(y0 - y);
  float alfa = acos((c + a*a - b*b)/2/a/sqrt(c)) + acos(x/sqrt(c));
  float beta = acos((a*a + b*b - c)/2/a/b);

  setServo(back_right_up, 180 - alfa*180/PI);
  setServo(back_right_down, 180 - beta*180/PI);
}

void setup() {
//servo
  pwm.begin(); 
  pwm.setPWMFreq(50);
  delay(10);

  moveLeg_front_left(0,0);
  moveLeg_back_right(0,0);
  moveLeg_back_left(0,0);
  moveLeg_front_right(0,0);

  delay(3000);
}

void loop() {
  moveLeg_front_left(4,0);
  moveLeg_back_right(4,0);
  moveLeg_back_left(-4,4);
  moveLeg_front_right(-4,4);
  delay(step);
  moveLeg_front_left(4,4);
  moveLeg_back_right(4,4);
  moveLeg_back_left(-4,0);
  moveLeg_front_right(-4,0);
  delay(step);
  moveLeg_front_left(-4, 4);
  moveLeg_back_right(-4,4);
  moveLeg_back_left(4,0);
  moveLeg_front_right(4,0);
  delay(step);
  moveLeg_front_left(-4,0);
  moveLeg_back_right(-4,0);
  moveLeg_back_left(4,4);
  moveLeg_front_right(4,4);
  delay(step);



}
