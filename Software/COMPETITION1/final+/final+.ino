#include <Adafruit_MotorShield.h>
#include <NewPing.h>
#include <Servo.h>

// Pin numbers
#define pinLL 2
#define pinL 7
#define pinR 6
#define pinRR 5
#define pinFrontTrig 13
#define pinFrontEcho 12
#define pinSideTrig 8
#define pinSideEcho 11
#define pinMoving A0
#define pinButton A1
#define pinBlueDetector 1
#define pinRedDetector 0
#define pinBlue A2
#define pinRed A3

// Global constants
#define ROT_TIME 160
#define MOVE_TIME 70
#define TUNNEL_DISTANCE 8
#define INTERVAL 1000
#define INTERVAL_CORNER_AVOID_2 30000
#define INTERVAL_CORNER_AVOID_6 30000
#define BLUE_TIME 0
#define RED_TIME 0
#define TURN_DELAY 1050
#define MOVE_DELAY 2000
#define TOPSPEED 255
const int SLOWDOWN = TOPSPEED * 0.6;

bool is_returning=false;
bool colorRed=true;
bool tunnel_passed = false;

Servo servoArm;
Servo servoClaw;

uint8_t pickup_distance = 5; // change it !!!!!!!!!!!!!1
const int arm_down = 0;
const int arm_up = 0;
const int claw_close = 45;
const int claw_open = 0;

// Initialize ultrasound
NewPing sonarFront(pinFrontTrig, pinFrontEcho, 6); //change range
NewPing sonarSide(pinSideTrig, pinSideEcho, 15);

// Initialize motors and sensors
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft = AFMS.getMotor(3);
Adafruit_DCMotor *motorRight = AFMS.getMotor(4);

// Start state
int state = 0;

bool set = false;
bool setStop = false;
bool endRed = false;

// Button state
bool button = false;

// State debounce reference
unsigned long start = 0;
unsigned long tunnel_start = millis() + 1000000;

// Motor speeds
int speedLeft = 0;
int speedRight = 0;

// Line sensor variables
int ll = 0;
int l = 0;
int r = 0;
int rr= 0;

void detectColour(void) {

  if(digitalRead(pinBlueDetector)){
    is_returning=true;
    if (digitalRead(pinRedDetector)){
      colorRed=true;
      digitalWrite(pinRed, HIGH);
      digitalWrite(pinBlue, LOW);
    } else {
      colorRed=false;
      digitalWrite(pinRed, LOW);
      digitalWrite(pinBlue, HIGH);
    }
  } else {
    is_returning=false;
    digitalWrite(pinRed, LOW);
    digitalWrite(pinBlue, LOW);
  }
}

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

void turnLeftNoDelay() {
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

void turnRightNoDelay() {
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

void turnRightEnd() {
  motorLeft -> setSpeed(TOPSPEED);
  motorRight-> setSpeed(TOPSPEED);
  motorLeft -> run(FORWARD);
  motorRight-> run(BACKWARD);
  delay(MOVE_DELAY*1.2);
  motorRight -> run(FORWARD);
  speedLeft=TOPSPEED;
  speedRight=TOPSPEED;
  motorLeft -> setSpeed(speedLeft);
  motorRight -> setSpeed(speedRight);
}

void turnLeftEnd() {
  motorLeft -> setSpeed(TOPSPEED);
  motorRight-> setSpeed(TOPSPEED);
  motorLeft -> run(BACKWARD);
  motorRight-> run(FORWARD);
  delay(MOVE_DELAY * 1.2);
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
        if (endRed == false && setStop == false) {
          turnRight();
        }
        else if (endRed == false) {
          //delay(500);
          turnRight();
        }
        else {
          turnLeft();
        }
        if (endRed == false && setStop == true) {
          motorLeft -> run(FORWARD);
          motorRight -> run(FORWARD);
          speedLeft = TOPSPEED;
          speedRight = TOPSPEED;
          delay(3000);
          speedLeft = 0;
          speedRight = 0;
          motorLeft -> setSpeed(speedLeft);
          motorRight -> setSpeed(speedRight);
          delay(2000000);
        }
        if (endRed) {
          motorLeft -> run(FORWARD);
          motorRight -> run(FORWARD);
          speedLeft = TOPSPEED;
          speedRight = TOPSPEED;
          delay(3000);
          speedLeft = 0;
          speedRight = 0;
          motorLeft -> setSpeed(speedLeft);
          motorRight -> setSpeed(speedRight);
          delay(2000000);
        }
        start = millis();
        state = 2;
        //Serial.println("State 2");
      }
      break;
    case 2:
      if ((ll == 1 || rr == 1) && (millis() > start + INTERVAL_CORNER_AVOID_2)) {
        
        start = millis();
        state = 3;
        //Serial.println("State 3 (Yay!!)");
      }
      break;
    case 3:
      if ((ll == 1 || rr == 1) && (millis() > start + INTERVAL)) {
        
        start = millis();
        state = 4;
        //Serial.println("State 4 (More Yay!!)");
      }
      break;

    case 4:
      if ((ll == 1 || rr == 1) && (millis() > start + INTERVAL)) {
        
        start = millis();
        state = 5;
        //Serial.println("State 5 (More Yay!!)");
      }
      break;
    case 5:
      if ((ll == 1 || rr == 1) && (millis() > start + INTERVAL)) {
        start = millis();
        if(is_returning && !colorRed){
          turnRight();
        }
        else{
          state = 6;
        //Serial.println("State 6 (More Yay!!)");}
        }
      }
      break;
    /*case 6:    
      if ((ll == 1 || rr == 1) && (millis() > start + INTERVAL_CORNER_AVOID_6)) {
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
      break;*/
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
    delay(ROT_TIME * 0.4);
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
    delay(ROT_TIME * 0.4);
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
  if (state > 1 && !tunnel_passed) {
    if(l == 0 && ll == 1) {
      l = 1;
    }
    if(r == 0 && rr == 1) {
      r = 1;
    }
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
/*
void block_retrieval(void){
  while (sonarFront.ping_cm() > pickup_distance){ lineFollowing(); }
  motorLeft -> setSpeed(0);
  motorRight-> setSpeed(0);
  servoClaw.write(claw_open);
  delay(1000);
  servoArm.write(arm_down);
  delay(1000);
  servoClaw.write(claw_close);
  delay(1000);
  servoArm.write(arm_up);
  delay(1000);
  detectColour();

}
*/
void block_placement(void){
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
      tunnel_passed = true;
    }
    else if (tunnel_passed) {
      if (set == false) {
        tunnel_start = millis();
        set = true;
      }
      lineFollowing();
      if (tunnel_passed && !colorRed && millis() > tunnel_start + 12000) {
        turnRight();
        motorLeft -> setSpeed(TOPSPEED);
        motorRight -> setSpeed(TOPSPEED);
        motorLeft -> run(FORWARD);
        motorRight -> run(FORWARD);
        delay(1500);
        motorLeft -> run(BACKWARD);
        motorRight -> run(BACKWARD);
        motorLeft -> setSpeed(TOPSPEED);
        motorRight -> setSpeed(TOPSPEED);
        speedLeft = TOPSPEED;
        speedRight = TOPSPEED;
        delay(1500);
        motorLeft -> run(FORWARD);
        motorRight -> run(FORWARD);
        turnLeftEnd();
        state = 1;
        endRed = false;
        tunnel_passed=false;
        start = millis();
      }
      else if (tunnel_passed && millis() > tunnel_start + 27000) {
        turnRight();
        motorLeft -> setSpeed(TOPSPEED);
        motorRight -> setSpeed(TOPSPEED);
        motorLeft -> run(FORWARD);
        motorRight -> run(FORWARD);
        delay(1500);
        motorLeft -> run(BACKWARD);
        motorRight -> run(BACKWARD);
        motorLeft -> setSpeed(TOPSPEED);
        motorRight -> setSpeed(TOPSPEED);
        speedLeft = TOPSPEED;
        speedRight = TOPSPEED;
        delay(1500);
        motorLeft -> run(FORWARD);
        motorRight -> run(FORWARD);
        turnRightNoDelay();
        state = 1;
        endRed = true;
        tunnel_passed = false;
        start = millis();
      }
    }
    else {
      lineFollowing();
    }
    if (sonarFront.ping_cm() != 0 && setStop == false) {
      setStop = true;
      //Serial.println("stop");
      delay(1500);
      motorLeft -> setSpeed(0);
      motorRight -> setSpeed(0);
      for(int i=0;i<10;i++){
        if(digitalRead(pinRedDetector) == HIGH) {
          colorRed = false;
          endRed = false;
        }
        delay(50);
      }
      if (colorRed) {
        digitalWrite(pinRed, HIGH);
      }
      else {
        digitalWrite(pinBlue, HIGH);
      }
      delay(4500);
      digitalWrite(pinRed, LOW);
      digitalWrite(pinBlue, LOW);
      motorLeft -> setSpeed(TOPSPEED);
      motorRight -> setSpeed(TOPSPEED);
    }
    delay(50);
  }
      if (speedLeft!=0 || speedRight!=0){
      digitalWrite(pinMoving,LOW);  
    } 
    else {
      digitalWrite(pinMoving, HIGH);
    }
}