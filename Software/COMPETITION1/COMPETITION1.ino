#include <Adafruit_MotorShield.h>
#include <NewPing.h>

#define rotTime  120
#define moveTime 100
#define pinMoving A0
#define pinButton A1

//Color detection pins. !!!!!!!! CHECK TOMORROW
#define pinBlueDetector 0
#define pinRedDetector 1
#define pinBlue A2
#define pinRed A3

NewPing sonarSide(8,11, 15);
//NewPing sonarFront(,,);

// Initialize motors and sensors
Adafruit_MotorShield  AFMS   = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft  = AFMS.getMotor(3);
Adafruit_DCMotor *motorRight = AFMS.getMotor(4);

// Boolean start button variable
bool parked=false;
bool button = false;
bool START = false;

int state=1;
int speedLeft = 0;
int speedRight = 0;
const int TOPSPEED = 255;
const int SLOWDOWN = TOPSPEED * 0.6;

int ll = 0;
int l = 0;
int r = 0;
int rr= 0;


unsigned long duration; //useless
const unsigned long PARK_TIME = 4000;
const int TunnelDistance = 9;
const int rangeSide = 12;
bool in_tunnel = false;
bool is_returning=false;
unsigned long INTERVAL = 1000;
unsigned long start = 0;
unsigned long TURN_DELAY=1050;
unsigned long TAU=2000;

void turnLeft() {
  delay(TURN_DELAY);
  motorLeft -> setSpeed(TOPSPEED);
  motorRight-> setSpeed(TOPSPEED);
  motorLeft -> run(BACKWARD);
  motorRight-> run(FORWARD);
  delay(TAU);
  motorLeft  -> run(FORWARD);
  speedLeft=TOPSPEED;
  speedRight=TOPSPEED;
  motorLeft  -> setSpeed(speedLeft);
  motorRight -> setSpeed(speedRight);
}

void turnRight() {
  delay(TURN_DELAY);
  motorLeft -> setSpeed(TOPSPEED);
  motorRight-> setSpeed(TOPSPEED);
  motorLeft -> run(FORWARD);
  motorRight-> run(BACKWARD);
  delay(TAU);
  motorRight  -> run(FORWARD);
  speedLeft=TOPSPEED;
  speedRight=TOPSPEED;
  motorLeft  -> setSpeed(speedLeft);
  motorRight -> setSpeed(speedRight);
}

void turn180(void){
  motorLeft -> setSpeed(TOPSPEED);
  motorRight-> setSpeed(TOPSPEED);
  motorLeft -> run(FORWARD);
  motorRight-> run(BACKWARD);
  delay( 2 * TAU);
  motorRight  -> run(FORWARD);
  speedLeft=TOPSPEED;
  speedRight=TOPSPEED;
  motorLeft  -> setSpeed(speedLeft);
  motorRight -> setSpeed(speedRight);

}

void detectColour(void){

  if(digitalRead(pinBlueDetector)){
    if (digitalRead(pinRedDetector)){
      digitalWrite(pinRed, HIGH);
      digitalWrite(pinBlue, LOW);
    } else {
      digitalWrite(pinRed, LOW);
      digitalWrite(pinBlue, HIGH);
    }
  } else {
    digitalWrite(pinRed, LOW);
    digitalWrite(pinBlue, LOW);
  }
}

void junction_detector() 
{
  switch(state)
  {
    case 0:
        if(ll == 1 || rr == 1){
          state = 1;
          start = millis();} 
        else if (ll != rr)
        { 
          Serial.print("ERROR IN JUNCTION DETECTOR SWITCH case0. JUNCTION= ");
          Serial.print(ll);
          Serial.println(rr);
        }
          break;

    case 1:
      if( ll==0 && rr==1 && millis() > start + INTERVAL && is_returning)
      {
        Serial.println("In case 1, current state is: ");
        Serial.println(state);
        turnRight();
        state=0;
        start=millis();
      } else if((ll==1 || rr==1) && millis() > start + INTERVAL)
      {
        turnRight();
        state = 2;
        start = millis();
      }
      break;       
    
    case 2:
      if(rr == 1 && millis() > start + INTERVAL){
      start = millis();
      l=0;
      r=0;
      state=3;   
      } else if( ll!=rr){
        Serial.print("ERROR IN JUNCTION_DETECTOR SWITCH case2. JUNCTION= ");
        Serial.print(ll);
        Serial.println(rr);
      }
      break;

    case 3:    
      if(ll==1 && millis() > start + INTERVAL)
      {
        start = millis();
        l=0;
        state=4;
      } else if( ll!=rr){
        Serial.print("ERROR IN JUNCTION_DETECTOR SWITCH case3. JUNCTION= ");
        Serial.print(ll);
        Serial.println(rr);
      }  
      break;

    case 4:
      if (ll==1 && rr==1 && millis() > start + INTERVAL){  //if (ll==1 && rr==1 && millis() > start + INTERVAL){ is the real code, changed with || because one sensor does not work
        start = millis();
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
      if(ll==1 && millis() > start + INTERVAL){    
        start = millis();
        l=0;
        state=6;
      } else if( ll!=rr){
        Serial.print("ERROR IN JUNCTION_DETECTOR SWITCH case5. JUNCTION= ");
        Serial.print(ll);
        Serial.println(rr);
      } 
      break;

    case 6:
      is_returning=true;
      if(rr==1 && millis() > start + INTERVAL){
        start = millis();
        r=1;
        state=1;
        Serial.println("in case 6, now state is:");
        Serial.println(state);
        }else if( ll!=rr){
        Serial.print("ERROR IN JUNCTION_DETECTOR SWITCH case6. JUNCTION= ");
        Serial.print(ll);
        Serial.println(rr);
      } 
      break;

    default:
            Serial.print("ERROR IN JUNCTION_DETECTOR SWITCH. It entered default state ");
            break; 
  }  
}

void lineFollowing() {

  ll = digitalRead(2);  //it was 5 before
  rr = digitalRead(5);  //it was 2 before
  l = digitalRead(7);
  r = digitalRead(6);
  junction_detector();

  if (l == 1) {
    if (speedLeft != SLOWDOWN || speedRight != SLOWDOWN) {
      motorLeft -> setSpeed(SLOWDOWN);
      motorRight -> setSpeed(SLOWDOWN);
      speedLeft = SLOWDOWN;
      speedRight = SLOWDOWN;
    }
    motorLeft -> run(BACKWARD);
    motorRight -> run(FORWARD);
    delay(rotTime);
    motorLeft -> run(FORWARD);
    delay(moveTime);
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
    delay(rotTime);
    motorRight -> run(FORWARD);
    delay(moveTime);
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
unsigned long readUltrasonicDistance(int inPin, int outPin) {
  //Send out trigger pulse
  digitalWrite(outPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(outPin, LOW);

  //Read echo pulse length
  duration = pulseIn(inPin, HIGH);
  return duration/58;
} */

// Triggered when abs(sonarSideSideSide.ping_cm() - TunnelDistance) < criticalValue
void tunnelNavigation() {
  Serial.println(sonarSide.ping_cm());
  int diff = sonarSide.ping_cm() - TunnelDistance;
  if (diff > 0) {
    if (speedLeft != SLOWDOWN || speedRight != SLOWDOWN) {
      motorLeft -> setSpeed(SLOWDOWN);
      motorRight -> setSpeed(SLOWDOWN);
      speedLeft = SLOWDOWN;
      speedRight = SLOWDOWN;
    }
    motorLeft -> run(BACKWARD);
    motorRight -> run(FORWARD);
    delay(rotTime * 0.6);
    motorLeft -> run(FORWARD);
    delay(moveTime * 2);
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
    delay(rotTime * 0.6);
    motorRight -> run(FORWARD);
    delay(moveTime * 2);
  }
  else {
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
  delay(3000);
  Serial.begin(9600);
  AFMS.begin();
  pinMode(pinMoving,OUTPUT);
  pinMode(6,INPUT);
  pinMode(7, INPUT);
  pinMode(5, INPUT);
  pinMode(2, INPUT);
  pinMode(8, OUTPUT);
  pinMode(11, INPUT);
  pinMode(pinButton,INPUT);
  //Color pins
  pinMode(pinBlueDetector, INPUT); 
  pinMode(pinRedDetector, INPUT); 
  pinMode(pinBlue,OUTPUT); 
  pinMode(pinRed, OUTPUT); 
  speedLeft=0;
  speedRight=0;
  motorLeft  -> setSpeed(speedLeft) ;
  motorRight -> setSpeed(speedRight);
  motorLeft -> run(FORWARD);
  motorRight -> run(FORWARD);
}

void loop() {
  
  if ( digitalRead(pinButton) == HIGH && !button)
  {
    button = true;
    speedLeft=255;
    speedRight=255;
    motorLeft  -> setSpeed(speedLeft) ;
    motorRight -> setSpeed(speedRight);
    motorLeft -> run(FORWARD);
    motorRight -> run(FORWARD);
    digitalWrite(pinMoving,HIGH);
    delay(300);
  }

if (button && !parked)
{
  if ( sonarSide.ping_cm()  != 0 ) {
    in_tunnel = true;
    tunnelNavigation();
  }
  else {
    in_tunnel = false;
    lineFollowing(); 
    if (state==0 && is_returning){
      start=millis();
      while(millis()<start+PARK_TIME){
        lineFollowing();
      }
      speedLeft=0;
      speedRight=0;
      motorLeft-> setSpeed(speedLeft);
      motorRight-> setSpeed(speedRight);
      digitalWrite(pinMoving,HIGH);
      parked=true;
    }
  }

  if (speedLeft!=0 || speedRight!=0){
  digitalWrite(pinMoving,LOW);  
  } else{digitalWrite(pinMoving, HIGH);}
  detectColour();
}
  delay(50);
}