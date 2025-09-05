#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define Y0 16
#define step 200

//servos
#define front_left_up 12
#define front_left_down 13

#define front_right_up 1
#define front_right_down 0

#define back_right_up 14
#define back_right_down 15

#define back_left_up 3
#define back_left_down 2

//SDA (default is GPIO 21)
//SCL (default is GPIO 22)

//dc motors
#define RPWM1 25
#define LPWM1 26
#define REN1  27
#define LEN1  14

#define RPWM2 33
#define LPWM2 32
#define REN2  12
#define LEN2  13

//reciver
#define CH1 4 
#define CH2 15 
#define CH3 34 
#define CH4 35 
#define CH5 39 
#define CH6 36

Adafruit_PWMServoDriver pwm0 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);

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

void walk(){
  moveLeg_front_left(-3,0);
  moveLeg_back_right(-3,0);
  moveLeg_back_left(0,3);
  moveLeg_front_right(0,3);
  delay(step);
  moveLeg_front_left(-3, 0);
  moveLeg_back_right(-3,0);
  moveLeg_back_left(3,0);
  moveLeg_front_right(3,0);
  delay(step);
  moveLeg_front_left(0,3);
  moveLeg_back_right(0,3);
  moveLeg_back_left(-3,0);
  moveLeg_front_right(-3,0);
  delay(step);
  moveLeg_front_left(3,0);
  moveLeg_back_right(3,0);
  moveLeg_back_left(-3,0);
  moveLeg_front_right(-3,0);
  delay(step);
}

void drive(){
  int rightPulse = pulseIn(CH3, HIGH, 25000);
  int leftPulse  = pulseIn(CH2, HIGH, 25000);

  int rightSpeed = map(rightPulse, 1000, 2000, -255, 255);
  int leftSpeed  = map(leftPulse, 1000, 2000, -255, 255);

  setMotor(RPWM1, LPWM1, REN1, LEN1, rightSpeed);
  setMotor(RPWM2, LPWM2, REN2, LEN2, leftSpeed);

  delay(20);
}

void setup() {
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

  //Set up PWM (8-bit, 1kHz)
  // ledcSetup(0, 1000, 8); ledcAttachPin(RPWM1, 0);
  // ledcSetup(1, 1000, 8); ledcAttachPin(LPWM1, 1);
  // ledcSetup(2, 1000, 8); ledcAttachPin(RPWM2, 2);
  // ledcSetup(3, 1000, 8); ledcAttachPin(LPWM2, 3);

  //servo
  pwm0.begin(); 
  pwm0.setPWMFreq(50);
  pwm1.begin(); 
  pwm1.setPWMFreq(50);
  delay(10);
  moveLeg_front_left(0,0);
  moveLeg_back_right(0,0);
  moveLeg_back_left(0,0);
  moveLeg_front_right(0,0);


  //reciver
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);
  pinMode(CH5, INPUT);
  pinMode(CH6, INPUT);


  delay(3000);
}

void loop() {
  walk();
}
