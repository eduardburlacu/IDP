#include <Adafruit_MotorShield.h>

Adafruit_MotorShield  AFMS   = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft  = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);

#define rotTime 150
#define moveTime 100

int speedLeft = 0;
int speedRight = 0;
int TOPSPEED = 255;
int l = 0;
int r = 0;
int SLOWDOWN = TOPSPEED * 0.6;
bool LEFT;
bool RIGHT;

void setup() {
  delay(3000);
  Serial.begin(9600);
  AFMS.begin();
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  motorLeft -> setSpeed(0);
  motorRight -> setSpeed(0);
  motorLeft -> run(FORWARD);
  motorRight -> run(FORWARD);
}

void lineFollowing() {
  l = digitalRead(5);
  r = digitalRead(6);
  //Serial.print(l);
  //Serial.println(r);
  if (l == 1) {
    motorLeft -> setSpeed(SLOWDOWN);
    motorRight -> setSpeed(SLOWDOWN);
    motorLeft -> run(BACKWARD);
    motorRight -> run(FORWARD);
    delay(rotTime);
    motorLeft -> run(FORWARD);
    delay(moveTime);
    Serial.println("left");
  }
  if (r == 1) {
    motorLeft -> setSpeed(SLOWDOWN);
    motorRight -> setSpeed(SLOWDOWN);
    motorLeft -> run(FORWARD);
    motorRight -> run(BACKWARD);
    delay(rotTime);
    motorRight -> run(FORWARD);
    delay(moveTime);
    Serial.println("right");
  }
  if (l == 0 && r == 0) {
    if (speedLeft != TOPSPEED && speedRight != TOPSPEED){
      motorLeft -> setSpeed(TOPSPEED);
      motorRight -> setSpeed(TOPSPEED);
      motorLeft -> run(FORWARD);
      motorRight -> run(FORWARD);
      speedLeft = TOPSPEED;
      speedRight = TOPSPEED;
    }
    Serial.println("straight");
    Serial.print(l);
    Serial.println(r);
  }
}

void loop() {
  while(true) {
   lineFollowing();
   delay(50);
  }
}
