#include <Adafruit_MotorShield.h>
#include <NewPing.h>

// Pin numbers
#define pinLL 2
#define pinL 7
#define pinR 6
#define pinRR 5
#define pinSideTrig 8
#define pinSideEcho 11
#define pinMoving A0
#define pinButton A1
#define pinBlueDetector 1
#define pinRedDetector 0
#define pinBlue A2
#define pinRed A3

// Global constants
#define ROT_TIME 140
#define MOVE_TIME 100
#define TUNNEL_DISTANCE 8
#define INTERVAL 1000
#define TURN_DELAY 1050
#define MOVE_DELAY 2000
#define TOPSPEED 255
const int SLOWDOWN = TOPSPEED * 0.6;

// Initialize side ultrasound
NewPing sonarSide(pinSideTrig, pinSideEcho, 15);

// Initialize motors and sensors
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft = AFMS.getMotor(3);
Adafruit_DCMotor *motorRight = AFMS.getMotor(4);

// Start state
int state = 0;

// Button state
bool button = false;

// State debounce reference
unsigned long start = 0;

// Motor speeds
int speedLeft = 0;
int speedRight = 0;

// Line sensor variables
int ll = 0;
int l = 0;
int r = 0;
int rr= 0;

// Turn left 90 degrees
void turnLeft() {
  delay(TURN_DELAY);
  motorLeft -> setSpeed(TOPSPEED);
  motorRight-> setSpeed(TOPSPEED);
  motorLeft -> run(BACKWARD);
  motorRight-> run(FORWARD);
  delay(MOVE_DELAY);
  motorLeft -> run(FORWARD);
  speedLeft=TOPSPEED;
  speedRight=TOPSPEED;
  motorLeft -> setSpeed(speedLeft);
  motorRight -> setSpeed(speedRight);
}

// Turn right 90 degrees
void turnRight() {
  delay(TURN_DELAY);
  motorLeft -> setSpeed(TOPSPEED);
  motorRight-> setSpeed(TOPSPEED);
  motorLeft -> run(FORWARD);
  motorRight-> run(BACKWARD);
  delay(MOVE_DELAY);
  motorRight -> run(FORWARD);
  speedLeft=TOPSPEED;
  speedRight=TOPSPEED;
  motorLeft -> setSpeed(speedLeft);
  motorRight -> setSpeed(speedRight);
}

// Decision making when junctions are reached
void junctionDetector() {
  switch (state) {
    case 0:
        if (ll == 1 || rr == 1) {
          state = 1;
          start = millis();
        } 
      break;

    case 1:
      if ((ll == 1 || rr == 1) && (millis() > start + INTERVAL)) {
        
        turnRight();
        start = millis();
        state = 2;
        Serial.println("State 2");
      }
      break;
    case 2:
      if ((ll == 1 || rr == 1) && (millis() > start + INTERVAL)) {
        
        start = millis();
        state = 3;
        Serial.println("State 3 (Yay!!)");
      }
      break;
    case 3:
      if ((ll == 1 || rr == 1) && (millis() > start + INTERVAL)) {
        
        start = millis();
        state = 4;
        Serial.println("State 4 (More Yay!!)");
      }
      break;

    case 4:
      if ((ll == 1 || rr == 1) && (millis() > start + INTERVAL)) {
        
        start = millis();
        state = 5;
        Serial.println("State 5 (More Yay!!)");
      }
      break;
    case 5:
      if ((ll == 1 || rr == 1) && (millis() > start + INTERVAL)) {
        start = millis();
        state = 6;
        Serial.println("State 6 (More Yay!!)");
      }
      break;
    case 6:
      if ((ll == 1 || rr == 1) && (millis() > start + INTERVAL)) {
        Serial.println("State 7 (Parking Yay!!)");
        turnRight();
        motorLeft -> setSpeed(TOPSPEED);
        motorRight -> setSpeed(TOPSPEED);
        motorLeft -> run(FORWARD);
        motorRight -> run(FORWARD);
        speedLeft = TOPSPEED;
        speedRight = TOPSPEED;
        delay(3000);
        motorLeft -> setSpeed(0);
        motorRight -> setSpeed(0);
        delay(2000000);
        start = millis();
      }
      break;
  }
}

// Tunnel navigation using ultrasound
void tunnelNavigation() {
  int diff = sonarSide.ping_cm() - TUNNEL_DISTANCE;
  if (diff > 0) {
    if (speedLeft != SLOWDOWN || speedRight != SLOWDOWN) {
      motorLeft -> setSpeed(SLOWDOWN);
      motorRight -> setSpeed(SLOWDOWN);
      speedLeft = SLOWDOWN;
      speedRight = SLOWDOWN;
    }
    motorLeft -> run(BACKWARD);
    motorRight -> run(FORWARD);
    delay(ROT_TIME * 0.6);
    motorLeft -> run(FORWARD);
    delay(MOVE_TIME * 2);
  }
  else if (diff < 0) {
    if (speedLeft != SLOWDOWN || speedRight != SLOWDOWN) {
      motorLeft -> setSpeed(SLOWDOWN);
      motorRight -> setSpeed(SLOWDOWN);
      speedLeft = SLOWDOWN;
      speedRight = SLOWDOWN;
    }
    motorLeft -> run(FORWARD);
    motorRight -> run(BACKWARD);
    delay(ROT_TIME * 0.6);
    motorRight -> run(FORWARD);
    delay(MOVE_TIME * 2);
  }
  else {
    if (speedLeft != TOPSPEED || speedRight != TOPSPEED) {
      motorLeft -> setSpeed(TOPSPEED);
      motorRight -> setSpeed(TOPSPEED);
      motorLeft -> run(FORWARD);
      motorRight -> run(FORWARD);
      speedLeft = TOPSPEED;
      speedRight = TOPSPEED;
    }
  }
}

// Line following
void lineFollowing() {
  ll = digitalRead(pinLL);
  rr = digitalRead(pinRR);
  if (ll == 1 || rr == 1) {
    l = 0;
    r = 0;
  }
  else {
    l = digitalRead(pinL);
    r = digitalRead(pinR);
  }
   junctionDetector();
  if (l == 1) {
    if (speedLeft != SLOWDOWN || speedRight != SLOWDOWN) {
      motorLeft -> setSpeed(SLOWDOWN);
      motorRight -> setSpeed(SLOWDOWN);
      speedLeft = SLOWDOWN;
      speedRight = SLOWDOWN;
    }
    motorLeft -> run(BACKWARD);
    motorRight -> run(FORWARD);
    delay(ROT_TIME);
    motorLeft -> run(FORWARD);
    delay(MOVE_TIME);
  }
  if (r == 1) {
    if (speedLeft != SLOWDOWN || speedRight != SLOWDOWN) {
      motorLeft -> setSpeed(SLOWDOWN);
      motorRight -> setSpeed(SLOWDOWN);
      speedLeft = SLOWDOWN;
      speedRight = SLOWDOWN;
    }
    motorLeft -> run(FORWARD);
    motorRight -> run(BACKWARD);
    delay(ROT_TIME);
    motorRight -> run(FORWARD);
    delay(MOVE_TIME);
  }
  if (l == 0 && r == 0) {
    if (speedLeft != TOPSPEED || speedRight != TOPSPEED){
      motorLeft -> setSpeed(TOPSPEED);
      motorRight -> setSpeed(TOPSPEED);
      motorLeft -> run(FORWARD);
      motorRight -> run(FORWARD);
      speedLeft = TOPSPEED;
      speedRight = TOPSPEED;
    }
  }
}

void setup() {
  Serial.begin(9600);
  AFMS.begin();
  pinMode(pinMoving, OUTPUT);
  pinMode(pinLL, INPUT);
  pinMode(pinL, INPUT);
  pinMode(pinR, INPUT);
  pinMode(pinRR, INPUT);
  pinMode(pinSideTrig, OUTPUT);
  pinMode(pinSideEcho, INPUT);
  pinMode(pinButton,INPUT);
  pinMode(pinBlueDetector, INPUT); 
  pinMode(pinRedDetector, INPUT); 
  pinMode(pinBlue, OUTPUT); 
  pinMode(pinRed, OUTPUT);
  motorLeft -> setSpeed(speedLeft) ;
  motorRight -> setSpeed(speedRight);
  motorLeft -> run(FORWARD);
  motorRight -> run(FORWARD);
  digitalWrite(pinMoving, HIGH);
  start = millis();
}

void loop() {
  if (digitalRead(pinButton) == HIGH && !button) {
    button = true;
    speedLeft = 255;
    speedRight = 255;
    motorLeft -> setSpeed(speedLeft) ;
    motorRight -> setSpeed(speedRight);
    motorLeft -> run(FORWARD);
    motorRight -> run(FORWARD);
    digitalWrite(pinMoving, LOW);
    delay(300);
  }
  if (button) {
    if (sonarSide.ping_cm() != 0) {
      tunnelNavigation();
    }
    else {
      lineFollowing();
    }
    delay(50);
  }
}