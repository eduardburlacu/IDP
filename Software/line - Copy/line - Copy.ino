#include <Adafruit_MotorShield.h>
#include "Wire.h"
#include <Servo.h>
#include <NewPing.h>

// Define the relevant state variables
 bool colorRed  = false;         //True if the detected block is red -- from electrical team
 bool colorDetected = false;     //True if a color is detected -- from electrical team
 bool cubeDetectedFront = false; //True if the front ultrasonic sensor detects a cube in front of the robot.
 bool cubeDetectedSide  = false; //True if the side ultrasonic sensor detects a cube on the left side of the robot
 bool in_junction = false;       //True if a junction is detected
 bool in_tunnel = false;         //True if the cube is in tunnel.
 bool is_in_box = true;          //True if the robot is inside one of the start/end boxes.
 bool is_region = false;         //True if the robot has reached the region where cubes are expected(regions 3 to 5)
 bool cube_to_destination=false; //True if the cube has been taken and is on its way to drop off.
 bool destination_reached=false; //True if the robot has made the final turn right to the path to cube placement.
 
 unsigned int cubeDistance=0;   
 unsigned int distance_front;  // Get average distance reading (discard anomalies)

 uint8_t JUNCTION[]= {0, 0};   // Variable encoding the state of the 2 IR sensors for junction detection. 
 uint8_t LINE[]    = {0, 0};   // Variable encoding the state of the 2 IR sensors for line following.                 
 uint8_t speedLeft=0;          // Variable for the speed of the left  DC Motor, range 0->255
 uint8_t speedRight=0;         // Variable for the speed of the right DC Motor, range 0->255 
 uint8_t state=0;              // Stores the current state of the robot(see the state diagram on github/Software)

 const float TunnelKd=0.1, TunnelKp=1, TunnelKi=0.2; // Tunnel PID controller parameters. Need to be found analitically after the mechanics is done!!
 const uint8_t TunnelDistance = 0; // desired distance from tunnel wall
 const uint8_t TunnelExit = 0; // distance reading found at tunnel exit
 uint8_t claw_open = 0;       // Servo motor angle for open claw
 uint8_t claw_close = 0;      // Servo motor angle for closed claw
 uint8_t arm_up = 0;          // Servo motor angle for elevated arm
 uint8_t arm_down = 0;        // Servo motor angle for lowered arm
 uint8_t pickup_distance = 0; // Distance from ultrasound where robot stops
 const uint8_t truck_distance = 0; // Distance from truck where robot stops

 float THETA = 0.0;			       // Is 1.0 if the robots goes too much to left
 float PREV_THETA = 0.0;       // Previous step
 float TOTAL_THETA = 0.0;		   // Discrete integral of THETA.


//       !!!!! PARAMETERS TO BE TUNED BY EXPERIMENT !!!!!!

const uint8_t DELTA=        30;  // Compensation caused by the imbalance in weight of distribution in the robot. Make it positive if it drifts to the left!!
const uint8_t NUM_READ=      8;  // Number of points to use for average
const uint8_t RANGE_FRONT = 10;  // Maximum distance allowed to the frontal ultrasonic sensor.
const uint8_t RANGE_SIDE=   10;  // Maximum distance allowed to the side ultrasonic sensor.
const uint8_t REFRESH_TIME= 10;  // Time in miliseconds after which a new update is provided
const unsigned long TAU=   200;  // The required time(ms) to perform a 90 deg turn.
const uint8_t TOP_SPEED=   120;  // The final speed of the robot as ratio to the maximum available input from the DC motor.
const uint8_t ROT_SPEED=   150;  // The wheel's complementary speeds when performing a rotation

const float Kd=20;            // PID controller parameters            
const float Kp=45;            // Need to be found experimentally 
const float Ki=10;            // after the mechanics is done!!


// Define the pin numbers for sensors and motors. To be filled in with Andrew after they finish the electrical circuits!!

const uint8_t pinLL=  4;
const uint8_t pinL =  5;
const uint8_t pinR =  6;
const uint8_t pinRR=  7;

const uint8_t pinColorRed = 14;
const uint8_t pinColorTrig= 15;
const uint8_t pinMoving= 14;
const uint8_t pinMotorLeft;
const uint8_t pinMotorRight;
const uint8_t pinFrontTrig= 10 ; // Digital pin used to connect the receiver    of the HC-SR04 ultrasonic front sensor
const uint8_t pinFrontEcho= 11 ; // Digital pin used to connect the transmitter of the HC-SR04 ultrasonic front sensor
const uint8_t pinSideTrig = 12 ; // Digital pin used to connect the receiver    of the HC-SR04 ultrasonic side  sensor
const uint8_t pinSideEcho=  13 ; // Digital pin used to connect the transmitter of the HC-SR04 ultrasonic side  sensor   
const uint8_t pinArm = 2;
const uint8_t pinClaw= 3;


// Functions for the line following loop  

void get_state(void)
{
  LINE[0]     = digitalRead(pinL);
  LINE[1]     = digitalRead(pinR);
  JUNCTION[0] = digitalRead(pinLL);
  JUNCTION[0] = digitalRead(pinRR);
  delay(2);
}

void get_error(void)
{ 
  /*
  Convention: shift to left leads to positive error.
  */
  PREV_THETA = THETA;
  if (LINE=="01"){ THETA = 1.0; }
  else if( LINE=="10" ){THETA = -1.0; }
  else {THETA = 0.0; }
  TOTAL_THETA += THETA;
}

float get_control_signal(void)
{ 
  // Used for the feedback loop of the parameter. The new value of the motor speed is set accordingly.
  return Kp * THETA + Ki * TOTAL_THETA + Kd * (THETA - PREV_THETA);
}

float get_control_signal_tunnel(int diff, int totalDiff, int prevDiff)
{ 
  // Used for the feedback loop of the parameter. The new value of the motor speed is set accordingly.
  return TunnelKp * diff + TunnelKi * totalDiff + TunnelKd * (diff - prevDiff);
}

void update_region(void)
{
   if (state == 3 || state == 4 || state == 5) {
    is_region = true;
  } else {  is_region = false;  }
}

// Create objects for motor control (DC and Servo)

Adafruit_MotorShield  AFMS   = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft  = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);
NewPing sonarFront(pinFrontTrig, pinFrontEcho,RANGE_FRONT);
NewPing sonarSide(pinSideTrig, pinSideEcho, RANGE_SIDE);
Servo servoArm;
Servo servoClaw;

// Movement commands.

void turnLeft()
{
  motorLeft -> setSpeed(ROT_SPEED);
  motorRight-> setSpeed(ROT_SPEED);
  motorLeft -> run(BACKWARD);
  motorRight-> run(FORWARD);
  delay(TAU);
  motorLeft  -> run(FORWARD);
  motorLeft  -> setSpeed(TOP_SPEED);
  motorRight -> setSpeed(TOP_SPEED);
  speedLeft = TOP_SPEED;
  speedRight = TOP_SPEED;
}

void turnLeft(uint8_t angle)
{
  motorLeft -> setSpeed(ROT_SPEED);
  motorRight-> setSpeed(ROT_SPEED);
  motorLeft -> run(BACKWARD);
  motorRight-> run(FORWARD);
  double revolve_time;
  if(angle==180){ revolve_time = 2 * TAU;} else{ revolve_time = TAU * angle / 90.0;  }
  delay(revolve_time);
  motorLeft  -> run(FORWARD);
  motorLeft  -> setSpeed(TOP_SPEED);
  motorRight -> setSpeed(TOP_SPEED);
  speedLeft = TOP_SPEED;
  speedRight = TOP_SPEED;
}

void turnRight()
{
  motorLeft -> setSpeed(ROT_SPEED);
  motorRight-> setSpeed(ROT_SPEED);
  motorLeft -> run(FORWARD);
  motorRight-> run(BACKWARD);
  delay(TAU);
  motorRight  -> run(FORWARD);
  motorLeft  -> setSpeed(TOP_SPEED);
  motorRight -> setSpeed(TOP_SPEED);
}

void turnRight(uint8_t angle)
{
  motorLeft -> setSpeed(ROT_SPEED);
  motorRight-> setSpeed(ROT_SPEED);
  motorLeft -> run(FORWARD);
  motorRight-> run(BACKWARD);
  double revolve_time;
  if(angle==180){
    revolve_time = 2 * TAU;
  }
  else{
    revolve_time = TAU * angle / 90.0;  
  }
  delay(revolve_time);
  motorRight -> run(FORWARD);
  motorLeft  -> setSpeed(TOP_SPEED);
  motorRight -> setSpeed(TOP_SPEED);
  speedLeft = TOP_SPEED;
  speedRight = TOP_SPEED;
}


//Deal with any junction in the track using this
void junction_detector(void)
{
  if( !in_junction){
    switch(state)
    {
      case 0:
        if(JUNCTION[0]==1 && JUNCTION[1]==1){
          state=1;

        } else if(JUNCTION[0] != JUNCTION[1])
          {  
        Serial.print("ERROR IN JUNCTION_DETECTOR SWITCH case0. JUNCTION= ");
        Serial.print(JUNCTION[0]);
        Serial.println(JUNCTION[1]);
          }
        break;
      
      case 1:
        if(JUNCTION[0]==1 && JUNCTION[1]==1)
        {
        motorLeft -> setSpeed(0);
        motorRight-> setSpeed(0);
        turnRight();
        state=2;

        } else if(JUNCTION[0] != JUNCTION[1]){
        Serial.print("ERROR IN JUNCTION_DETECTOR SWITCH case1. JUNCTION= ");
        Serial.print(JUNCTION[0]);
        Serial.print(JUNCTION[1]);
        }
        break;
      
      case 2:
        if (JUNCTION[0]==0 && JUNCTION[1]==1)
        {
          if(cube_to_destination && !colorRed)
          {
          motorLeft -> setSpeed(0);
          motorRight-> setSpeed(0);
          turnRight();
          destination_reached=true;
          }
          else{ state=3; update_region(); }
        }
        else if(JUNCTION[0] != JUNCTION[1]){
          Serial.print("ERROR IN JUNCTION_DETECTOR SWITCH case2. JUNCTION= ");
          Serial.print(JUNCTION[0]);
          Serial.print(JUNCTION[1]);     
        }
        break;
      
      case 3:
        if (JUNCTION[0]==1 && JUNCTION[1]==0)
        {
          motorLeft -> setSpeed(0);
          motorRight ->setSpeed(0);
          ultrasonic_lookup("side");
          if (cubeDetectedSide){
            turnLeft();
            in_junction=true;
          } else{
            motorLeft -> setSpeed(TOP_SPEED);
            motorRight-> setSpeed(TOP_SPEED);
            state=4;
            update_region();
            }
        }else if(JUNCTION[0] != JUNCTION[1]){
          Serial.print("ERROR IN JUNCTION_DETECTOR SWITCH case3. JUNCTION= ");
          Serial.print(JUNCTION[0]);
          Serial.print(JUNCTION[1]);
        }
        break;

      case 4:
        if(JUNCTION[0]==1 && JUNCTION[1]==1)
        { state=5; 
          update_region();
        } else if(JUNCTION[0] != JUNCTION[1]){
          Serial.print("ERROR IN JUNCTION_DETECTOR SWITCH case4. JUNCTION= ");
          Serial.print(JUNCTION[0]);
          Serial.print(JUNCTION[1]);
        }
        break;
      
      case 5:
        if(JUNCTION[0]==1 && JUNCTION[1]==0)
        {
        motorLeft -> setSpeed(0);
        motorRight-> setSpeed(0);
        ultrasonic_lookup("side");
        if (cubeDetectedSide){
          turnLeft();
          in_junction=true;
        } else{
            motorLeft -> setSpeed(TOP_SPEED);
            motorRight-> setSpeed(TOP_SPEED);
            state=6;
            update_region();
            }
        }
        else if(JUNCTION[0] != JUNCTION[1]){
        Serial.print("ERROR IN JUNCTION_DETECTOR SWITCH case5. JUNCTION= ");
        Serial.print(JUNCTION[0]);
        Serial.print(JUNCTION[1]);
        }
        break;
      
      case 6:
        if(JUNCTION[0]==0 && JUNCTION[1]==1)
        {
          if (cube_to_destination && colorRed){
            motorLeft -> setSpeed(0);
            motorRight-> setSpeed(0);
            turnRight();
            destination_reached=true;
          } else{state=1; update_region();}
        }
        else if(JUNCTION[0] != JUNCTION[1]){
        Serial.print("ERROR IN JUNCTION_DETECTOR SWITCH case6. JUNCTION= ");
        Serial.print(JUNCTION[0]);
        Serial.print(JUNCTION[1]);
        }
        unsigned long time = sonarSide.ping_median(NUM_READ);
        if (time!=0){in_tunnel=true;}
        else{in_tunnel=false;}
        break;
    }
  } else
      {
        if(JUNCTION[0]==1 && JUNCTION[1]==1)
        {
          motorLeft -> setSpeed(0);
          motorRight-> setSpeed(0);
          turnLeft();
          state++;
          update_region();
          in_junction=false;
        }
      }
  Serial.print("exiting junction detector");

}

void ultrasonic_lookup(char* sensor)
{ 
    /*
    Performs the search for cubes in the robot is in the important region. First, it searches with the frontal sensor. If it is a cube frontways, 
    it will stop at the corresponding (distance to the cube) - (pickup_distance) for the collection mechanism.
    */

    if(sensor=="front")  
    {
      unsigned long time = sonarFront.ping_median(NUM_READ);
      if(time != 0){
        unsigned int  dist = sonarFront.convert_cm(time);
        cubeDetectedFront=true;
        cubeDistance=dist;
      } else{cubeDetectedFront=false;}
    } 
    else if(sensor=="side")
    {
      unsigned long time = sonarSide.ping_median(NUM_READ);
      if (time != 0){
        unsigned int dist = sonarSide.convert_cm(time);
        cubeDetectedSide=true;
        cubeDistance=dist;} else{cubeDetectedSide=false;}
    }
    else{Serial.println("Error in ultrasonic lookup, invalid key. Valid keys are: front , side ."); }
}

void leave_box(void)
{

  if (is_in_box == false){ Serial.println("Error in leave_box, the state of the robot is_in_box is not true as expected"); }
  motorLeft  -> setSpeed(TOP_SPEED);
  motorRight -> setSpeed(TOP_SPEED);
  motorLeft  -> run(FORWARD);
  motorRight -> run(FORWARD);
  digitalWrite(pinMoving, LOW);
  do{
    get_state();
    junction_detector(); 
  } while( JUNCTION[0]=='0' && JUNCTION[1]=='0' );
  Serial.println("Found line");
  is_in_box=false;
}

void line_follower(void)
{
  /*
  In an iteration, this pipeline function does the following: 
    1. setting the state of the IR sensors
    2. checks if there is a junction.                           
    3. computes the required action based on the current output relative to the desired state ( THETA=0 state ).
  */
  get_state();
  junction_detector();
  //if (is_region && !cube_to_destination){
  //  ultrasonic_lookup("front");
  //}
  get_error();
  float x = get_control_signal();
  Serial.println(x);
  if (speedLeft!=TOP_SPEED + DELTA - x && speedRight!=TOP_SPEED - DELTA - x)
    {
    speedLeft = TOP_SPEED + DELTA - x;
    speedRight= TOP_SPEED - DELTA - x;
    motorLeft  -> setSpeed(speedLeft);
    motorRight -> setSpeed(speedRight); 
    }  
  delay(REFRESH_TIME);
}

//triggered when abs(sonarSide.ping_cm() - TunnelDistance) < criticalValue
void tunnel_navigation() 
{
  int diff = sonarSide.ping_cm() - TunnelDistance;
  int prevDiff, totalDiff = diff;
  float output = 0;
  while ((diff + TunnelDistance) <= TunnelExit) {
    output = get_control_signal_tunnel(diff, totalDiff, prevDiff);
    if ((speedLeft != TOP_SPEED + DELTA - output) && (speedRight != TOP_SPEED - DELTA - output)){
      motorLeft  -> setSpeed(TOP_SPEED + DELTA - output);
      motorRight -> setSpeed(TOP_SPEED - DELTA - output);  
    }
    prevDiff = diff;
    diff = sonarSide.ping_cm() - TunnelDistance;
    totalDiff += diff;
  }
}

// to be called when robot is set distance from block
void block_retrieval() {
  //move towards block until critical distance is reached (possibly slow down?)
  while (sonarFront.ping_cm() > pickup_distance) {
    line_follower();
  }
  //stop robot
  motorLeft  -> setSpeed(0);
  motorRight -> setSpeed(0);
  speedLeft = 0;
  speedRight = 0;
  digitalWrite(pinMoving, HIGH);
  //detect color
  colorDetected = digitalRead(pinColorTrig);
  if (colorDetected){colorRed = digitalRead(pinColorRed);
  } else{ Serial.println("Color detector/ Daniel's algorithm for pickup/ the chassis design is broke, or maybe all :(");}

  //descend claw
  servoArm.write(arm_down);
  //close claw 
  servoClaw.write(claw_close);
  //lift claw
  servoArm.write(arm_up);
  cube_to_destination = true;
  cubeDetectedFront = false;
  cubeDetectedSide  = false;
  digitalWrite(pinMoving, LOW);
  turnRight(180);
}

// to be called when robot is set distance from truck
void block_placement() {
  //move towards truck until critical distance is reached (possibly slow down?)
  while (sonarFront.ping_cm() > truck_distance) {
    line_follower();
  }
  //stop robot
  motorLeft  -> setSpeed(0);
  motorRight -> setSpeed(0);
  speedLeft = 0;
  speedRight = 0;
  digitalWrite(pinMoving, HIGH);  
  //open claw 
  servoClaw.write(claw_open);
  //reverse to give space to turn
  digitalWrite(pinMoving, LOW);
  motorLeft -> run(BACKWARD);
  motorRight -> run(BACKWARD);
  motorLeft -> setSpeed(TOP_SPEED);
  motorRight -> setSpeed(TOP_SPEED);
  speedLeft = -1 * TOP_SPEED;
  speedRight = -1 * TOP_SPEED;
  delay(1500); // reverses for 1.5 seconds
  motorLeft  -> setSpeed(0);
  motorRight -> setSpeed(0);
  motorLeft -> run(FORWARD);
  motorRight -> run(FORWARD);
  //turn 180 degrees
  turnRight(180);
  cube_to_destination=false;
  destination_reached=false;
}

void setup() {
  delay(2000);
  Serial.begin(9600);
  AFMS.begin();
  //Sensors setup step
    //IR sensors for line following
  pinMode(pinL, INPUT);
  pinMode(pinR, INPUT);
  pinMode(pinLL,INPUT);
  pinMode(pinRR,INPUT);

    // Pins for color/velocity sensing
  pinMode(pinMoving, OUTPUT);
  digitalWrite(pinMoving,HIGH);
  pinMode(pinColorRed,     INPUT);
  pinMode(pinColorTrig, INPUT);

      // Ultrasonic sensors for object detection.INPUT
  pinMode(pinFrontTrig, OUTPUT);
  pinMode(pinFrontEcho, INPUT);
  pinMode(pinSideTrig,  OUTPUT);
  pinMode(pinSideEcho,  INPUT);
      //Motors setup step
  servoArm.attach(pinArm);
  servoClaw.attach(pinClaw);
  motorLeft -> setSpeed(0);
  motorRight-> setSpeed(0);
  motorLeft -> run(FORWARD);
  motorRight-> run(FORWARD);
}

void loop() 
{
//Leave the box at start and after the cube has been placed correctly. !!!!!!!!!!!!NB turn 180 after the cube has been placed successfully
  if(is_in_box && !cube_to_destination )
  { leave_box(); }

  do{ 
    line_follower();
    }while(!is_region);
  is_region=true;
  
  do{  
    line_follower();
    if(cubeDetectedFront && !cubeDetectedSide)
    { block_retrieval(); }
  } while(!in_tunnel);
  is_region=false;
  while(in_tunnel){
    tunnel_navigation();
    delay(REFRESH_TIME);
  }
  do{
    line_follower();
  }while(cube_to_destination && !destination_reached);
  block_retrieval(); 
}
