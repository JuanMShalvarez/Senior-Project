#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define Y0 16
#define step 250

//servos
#define front_left_up 12
#define front_left_down 13

#define front_right_up 1
#define front_right_down 0

#define back_right_up 14
#define back_right_down 15

#define back_left_up 3
#define back_left_down 2

Adafruit_PWMServoDriver pwm0 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);

//dc motors
#define RPWM1 25
#define LPWM1 26
#define REN1  27
#define LEN1  14

#define RPWM2 33
#define LPWM2 32
#define REN2  12
#define LEN2  13

void setServo(int servo, int angle, int address){
  if(address == 0)
    pwm0.setPWM(servo, 0, map(angle, 0, 180, 102, 512));
  if(address == 1)
    pwm1.setPWM(servo, 0, map(angle, 0, 180, 102, 512));
}

void setMotor(int rpwm, int lpwm, int ren, int len, int speed) {
  // speed: -255 to 255
  digitalWrite(ren, HIGH);
  digitalWrite(len, HIGH);
  if (speed > 0) {
    analogWrite(rpwm, speed);
    analogWrite(lpwm, 0);
  } else {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, -speed);
  }
}

void moveLeg_back_left(float x, float y){

  float a = 8;
  float b = 10;
  float y0 = Y0;
  float c = x*x + (y0 - y)*(y0 - y);
  float alfa = acos((c + a*a - b*b)/2/a/sqrt(c)) + acos(x/sqrt(c));
  float beta = acos((a*a + b*b - c)/2/a/b);

  setServo(back_left_up, alfa*180/PI, 0);
  setServo(back_left_down, beta*180/PI, 0);
}

void moveLeg_front_right(float x, float y){

  float a = 8;
  float b = 10;
  float y0 = Y0-2;
  float c = x*x + (y0 - y)*(y0 - y);
  float alfa = acos((c + a*a - b*b)/2/a/sqrt(c)) + acos(x/sqrt(c));
  float beta = acos((a*a + b*b - c)/2/a/b);

  setServo(front_right_up, alfa*180/PI, 1);
  setServo(front_right_down, beta*180/PI, 1);
}

void moveLeg_front_left(float x, float y){

  float a = 8;
  float b = 10;
  float y0 = Y0-2;
  float c = x*x + (y0 - y)*(y0 - y);
  float alfa = acos((c + a*a - b*b)/2/a/sqrt(c)) + acos(x/sqrt(c));
  float beta = acos((a*a + b*b - c)/2/a/b);

  setServo(front_left_up, 180 - alfa*180/PI, 0);
  setServo(front_left_down, 180 - beta*180/PI, 0);
}

void moveLeg_back_right(float x, float y){

  float a = 8;
  float b = 10;
  float y0 = Y0;
  float c = x*x + (y0 - y)*(y0 - y);
  float alfa = acos((c + a*a - b*b)/2/a/sqrt(c)) + acos(x/sqrt(c));
  float beta = acos((a*a + b*b - c)/2/a/b);

  setServo(back_right_up, 180 - alfa*180/PI, 1);
  setServo(back_right_down, 180 - beta*180/PI, 1);
}


void setup() {
  //servo
  pwm0.begin(); 
  pwm0.setPWMFreq(50);
  pwm1.begin(); 
  pwm1.setPWMFreq(50);
  delay(10);

  // Motor 1 enable pins
  pinMode(REN1, OUTPUT);
  pinMode(LEN1, OUTPUT);
  digitalWrite(REN1, HIGH);
  digitalWrite(LEN1, HIGH);

  // Motor 2 enable pins
  pinMode(REN2, OUTPUT);
  pinMode(LEN2, OUTPUT);
  digitalWrite(REN2, HIGH);
  digitalWrite(LEN2, HIGH);

  moveLeg_front_left(-5,0);
  moveLeg_front_right(-2,0);
  moveLeg_back_right(-5,0);
  moveLeg_back_left(-5,0);

  delay(1000);
}

void loop() {

  setMotor(RPWM1, LPWM1, REN1, LEN1, 100);
  setMotor(RPWM2, LPWM2, REN2, LEN2, 100);

  moveLeg_front_left(1,0);
  moveLeg_back_right(-10,2);
  moveLeg_back_left(-2,8);
  moveLeg_front_right(-7,4);
  delay(step);
  moveLeg_front_left(1, 4);
  moveLeg_back_right(-10,8);
  moveLeg_back_left(-2,2);
  moveLeg_front_right(-1,0);
  delay(step);
  moveLeg_front_left(-7,4);
  moveLeg_back_right(-2,8);
  moveLeg_back_left(-10,2);
  moveLeg_front_right(1,0);
  delay(step);
  moveLeg_front_left(-7,0);
  moveLeg_back_right(-2,2);
  moveLeg_back_left(-10,8);
  moveLeg_front_right(1,4);
  delay(step);




  // threeMoveFL();
  // delay(1000);
  // threeMoveFR();
  // delay(1000);
  // threeMoveBL();
  // delay(1000);
  // threeMoveBR();
  // delay(1000);
  // displace();
  // delay(1000);
}

void threeMoveFL(){
  // moveLeg_front_left(1,0);
  // delay(step);
  moveLeg_front_left(1, 4);
  delay(step);
  moveLeg_front_left(-10, 4);
  delay(step);
  moveLeg_front_left(-10, 0);
  delay(step);
}

void threeMoveFR(){
  // moveLeg_front_right(1,0);
  // delay(step);
  moveLeg_front_right(1,4);
  delay(step);
  moveLeg_front_right(-7,4);
  delay(step);
  moveLeg_front_right(-7,0);
  delay(step);
}

void threeMoveBL(){
  moveLeg_back_left(-8, 8);
  delay(step);
  moveLeg_back_left(0, 8);
  delay(step);
  moveLeg_back_left(0, 0);
  delay(step);
}

void threeMoveBR(){
  moveLeg_back_right(-8,8);
  delay(step);
  moveLeg_back_right(0, 8);
  delay(step);
  moveLeg_back_right(0, 0);
  delay(step);
}

void displace(){
  moveLeg_front_left(-5,0);
  moveLeg_front_right(-2,0);
  moveLeg_back_left(-5, 0);
  moveLeg_back_right(-5, 0);
}
