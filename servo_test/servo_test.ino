#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define Y0 15
#define step 350

int frontXinit = 0;
int frontYinit = 4;

int frontYstep = 6;
int frontXstep = 12;

int backXinit = 0;
int backYinit = 2;

int backYstep = 4;
int backXstep = 10;

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
  float y0 = Y0;
  float c = x*x + (y0 - y)*(y0 - y);
  float alfa = acos((c + a*a - b*b)/2/a/sqrt(c)) + acos(x/sqrt(c));
  float beta = acos((a*a + b*b - c)/2/a/b);

  setServo(front_right_up, alfa*180/PI, 1);
  setServo(front_right_down, beta*180/PI, 1);
}

void moveLeg_front_left(float x, float y){

  float a = 8;
  float b = 10;
  float y0 = Y0;
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

  moveLeg_front_left(frontXinit, frontYinit);
  moveLeg_front_right(frontXinit, frontYinit);
  moveLeg_back_right(backXinit, backYinit);
  moveLeg_back_left(backXinit, backYinit);

  // setMotor(RPWM1, LPWM1, REN1, LEN1, 50);
  // setMotor(RPWM2, LPWM2, REN2, LEN2, 50);
  delay(1000);

  // delay(500);
  // moveLeg_front_left(-5, 0);
  // moveLeg_back_right(-5, 0);
  // delay(500);
  // moveLeg_front_right(-2, 7);
  // moveLeg_back_left(-5, 7);

  delay(1000);
}

void loop() {

  // delay(200);
  // moveLeg_front_left(-5, 0);
  // moveLeg_back_right(-5, 0);
  // delay(200);
  // moveLeg_front_right(-2, 7);
  // moveLeg_back_left(-5, 7);

  // delay(200);
  // moveLeg_front_right(-2, 0);
  // moveLeg_back_left(-5, 0);
  // delay(200);
  // moveLeg_front_left(-5, 7);
  // moveLeg_back_right(-5, 7);

  moveLeg_front_left(frontXinit, frontYinit);
  moveLeg_back_right(backXinit - backXstep, backYinit);
  moveLeg_back_left(backXinit, backYinit + backYstep);
  moveLeg_front_right(frontXinit - frontXstep, frontYinit + frontYstep);
  delay(step);
  moveLeg_front_left(frontXinit, frontYinit + frontYstep);
  moveLeg_back_right(backXinit - backXstep, backYinit + backYstep);
  moveLeg_back_left(backXinit, backYinit);
  moveLeg_front_right(frontXinit - frontXstep, frontYinit);
  delay(step);
  moveLeg_front_left(frontXinit - frontXstep, frontYinit + frontYstep);
  moveLeg_back_right(backXinit, backYinit + backYstep);
  moveLeg_back_left(backXinit - backXstep, backYinit);
  moveLeg_front_right(frontXinit, frontYinit);
  delay(step);
  moveLeg_front_left(frontXinit - frontXstep, frontYinit);
  moveLeg_back_right(backXinit, backYinit);
  moveLeg_back_left(backXinit - backXstep, backYinit + backYstep);
  moveLeg_front_right(frontXinit, frontYinit + frontYstep);
  delay(step);



  // threeMoveFL();
  // delay(step);
  // threeMoveFR();
  // delay(step);
  // threeMoveBL();
  // delay(step);
  // threeMoveBR();
  // delay(step);
  // displace();
  // delay(step);
}

void threeMoveFL(){
  // moveLeg_front_left(1,0);
  // delay(step);
  moveLeg_front_left(1, 4);
  delay(step);
  moveLeg_front_left(-10, 4);
  delay(step);
  // moveLeg_front_left(-5, 0);
  // delay(step);
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
  moveLeg_back_right(-6, 8);
  delay(step);
  moveLeg_back_right(-6, 6);
  delay(step);
}

void displace(){
  moveLeg_front_left(-5,0);
  moveLeg_front_right(-2,0);
  moveLeg_back_left(-11, 6);
  moveLeg_back_right(-11, 6);
}
