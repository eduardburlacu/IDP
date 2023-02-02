#include <Adafruit_MotorShield.h>
#include <NewPing.h>

#define rotTime  150
#define moveTime 100
int state=2;
unsigned long TAU=300;

Adafruit_MotorShield  AFMS   = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft  = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);


bool cube_to_destination=false;
bool cubeDetectedFront=false;
bool cubeDetectedSide= false;
int speedLeft = 0;
int speedRight = 0;
int TOPSPEED = 255;
int ll =0;
int l = 0;
int r = 0;
int rr= 0;

int SLOWDOWN = TOPSPEED * 0.6;
bool LEFT;
bool RIGHT;


void setup() {
  delay(3000);
  Serial.begin(9600);
  AFMS.begin();
  pinMode(4,INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  motorLeft -> setSpeed(0);
  motorRight -> setSpeed(0);
  motorLeft -> run(FORWARD);
  motorRight -> run(FORWARD);
}

void turnLeft()
{
  motorLeft -> setSpeed(TOPSPEED);
  motorRight-> setSpeed(TOPSPEED);
  motorLeft -> run(BACKWARD);
  motorRight-> run(FORWARD);
  delay(TAU);
  motorLeft  -> run(FORWARD);
  motorLeft  -> setSpeed(TOPSPEED);
  motorRight -> setSpeed(TOPSPEED);
  speedLeft = TOPSPEED;
  speedRight = TOPSPEED;
}

void turnRight()
{
  motorLeft -> setSpeed(TOPSPEED);
  motorRight-> setSpeed(TOPSPEED);
  motorLeft -> run(FORWARD);
  motorRight-> run(BACKWARD);
  delay(TAU);
  motorRight  -> run(FORWARD);
  motorLeft  -> setSpeed(TOPSPEED);
  motorRight -> setSpeed(TOPSPEED);
}

void junction_detector()
{
  switch(state)
  {
    case 0:
      if(ll==1 && rr==1){
        state=1;
      }
      else if (ll != rr)
      {  
        Serial.print("ERROR IN JUNCTION DETECTOR SWITCH case0. JUNCTION= ");
        Serial.print(ll);
        Serial.println(rr);
      }
      break;

    case 1:
      if(ll==1 && rr==1)
      {
        turnRight();
        state=2;
      }
      else if( ll!=rr){
        Serial.print("ERROR IN JUNCTION_DETECTOR SWITCH case1. JUNCTION= ");
        Serial.print(ll);
        Serial.println(rr);
      }  
      break;

    case 2:
      if(ll==0 && rr==1){
        if (cube_to_destination){turnRight();} //code to be added once navigaion works
        else{
          l=0;
          r=0;  
          state=3;        
          }
      } else if( ll!=rr){
        Serial.print("ERROR IN JUNCTION_DETECTOR SWITCH case2. JUNCTION= ");
        Serial.print(ll);
        Serial.println(rr);
      }  
      break;
    
    case 3:        
      if(ll==1 && rr==0){
        if(cubeDetectedSide){
          turnLeft();
        } 
        else{
          l=0;
          state=4;
        }
      } else if( ll!=rr){
        Serial.print("ERROR IN JUNCTION_DETECTOR SWITCH case3. JUNCTION= ");
        Serial.print(ll);
        Serial.println(rr);
      }  
      break;

    case 4:
      if (ll==1 && rr==1){
        l=0;
        r=0;
        state=5;
      } else if( ll!=rr){
        Serial.print("ERROR IN JUNCTION_DETECTOR SWITCH case4. JUNCTION= ");
        Serial.print(ll);
        Serial.println(rr);
      }  
      break;
    
    case 5:
      if(ll==1 && rr==0){
        //Code to be added to take the cube after we check the navigation works.
        l=0;
        state=6;
        } else if( ll!=rr){
        Serial.print("ERROR IN JUNCTION_DETECTOR SWITCH case5. JUNCTION= ");
        Serial.print(ll);
        Serial.println(rr);
      } 
      break;

    case 6:
      if(ll==0 && rr==1){
        if (cube_to_destination){turnRight();}
        else{
          r=1;
          state=1;
        }
      }else if( ll!=rr){
        Serial.print("ERROR IN JUNCTION_DETECTOR SWITCH case6. JUNCTION= ");
        Serial.print(ll);
        Serial.println(rr);
      } 
      break;
  }
}

void lineFollowing() {

  ll = digitalRead(4);
  l = digitalRead(5);
  r = digitalRead(6);
  rr=digitalRead(7);
  junction_detector();
  //Serial.print(l);
  //Serial.println(r);

  if (l == 1) {
    if (ll!=1){
    motorLeft -> setSpeed(SLOWDOWN);
    motorRight -> setSpeed(SLOWDOWN);
    motorLeft -> run(BACKWARD);
    motorRight -> run(FORWARD);
    delay(rotTime);
    motorLeft -> run(FORWARD);
    delay(moveTime);
    Serial.println("left");
    }
  }
  if (r == 1) {
    if (rr!=1){
    motorLeft -> setSpeed(SLOWDOWN);
    motorRight -> setSpeed(SLOWDOWN);
    motorLeft -> run(FORWARD);
    motorRight -> run(BACKWARD);
    delay(rotTime);
    motorRight -> run(FORWARD);
    delay(moveTime);
    Serial.println("right");
    }
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