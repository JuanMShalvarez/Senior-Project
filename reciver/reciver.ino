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
#define CH1 35
#define CH2 34
#define CH3 15
#define CH4 36
#define CH5 4
#define CH6 39

void setup() {
  Serial.begin(9600);
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

  //reciver
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);
  pinMode(CH5, INPUT);
  pinMode(CH6, INPUT);
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

void loop() {
  int rightPulse = pulseIn(CH3, HIGH, 25000);
  int leftPulse  = pulseIn(CH2, HIGH, 25000);
    
    Serial.print(rightPulse);
    Serial.print(", ");
    Serial.println(leftPulse);

    int rightSpeed = map(rightPulse, 1000, 2000, -255, 255);
    int leftSpeed  = map(leftPulse, 1000, 2000, -255, 255);

    setMotor(RPWM1, LPWM1, REN1, LEN1, rightSpeed);
    setMotor(RPWM2, LPWM2, REN2, LEN2, leftSpeed);

  delay(20);  // loop rate ~50Hz
}
