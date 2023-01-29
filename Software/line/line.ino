#include "header.h"

 // Create objects for motor control (DC and Servo)

Adafruit_MotorShield  AFMS   = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft  = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);
NewPing sonarFront(pinFrontTrig, pinFrontEcho,RANGE_FRONT);
NewPing sonarSide(pinSideTrig, pinSideEcho, RANGE_SIDE);
Servo servoArm;
Servo servoClaw;

void accelerate(uint8_t initial_speed=0, uint8_t final_speed= TOP_SPEED, uint8_t step_size=5, bool reverse=false)
{
  motorLeft -> setSpeed(initial_speed);
  motorRight-> setSpeed(initial_speed);
  if (!reverse){
    motorLeft -> run(FORWARD);
    motorRight-> run(FORWARD);  }
    else {
    motorLeft -> run(BACKWARD);
    motorRight-> run(BACKWARD); }
  uint8_t current=initial_speed;
  while(current!=final_speed) {
    motorLeft  -> setSpeed(current);
    motorRight -> setSpeed(current); 
    current += step_size;
    delay(10);  }
}

void turnLeft()
{
  motorLeft -> setSpeed(ROT_SPEED);
  motorRight-> setSpeed(ROT_SPEED);
  motorLeft -> run(BACKWARD);
  motorRight-> run(FORWARD);
  unsigned long start_time=millis();
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
  if(angle==180){
    revolve_time = 2 * TAU;
  }
  else{
    revolve_time = TAU * angle / 90.0;  
  }
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

void junction_detector(void)
{
  if( !in_junction){
    switch(state)
    {
      case 0:
        if(JUNCTION[0]==1 && JUNCTION[1]==1){
          state=1;
          update_region();
        } else if(JUNCTION[0] != JUNCTION[1])
          {  
        Serial.print("ERROR IN JUNCTION_DETECTOR SWITCH case0. JUNCTION= ");
        Serial.print(JUNCTION[0]);
        Serial.print(JUNCTION[1]);
          }
        break;
      
      case 1:
        if(JUNCTION[0]==1 && JUNCTION[1]==1)
        {
        motorLeft -> setSpeed(0);
        motorRight-> setSpeed(0);
        turnRight();
        state=2;
        update_region();
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
          } else{state=3; update_region();}
        }else if(JUNCTION[0] != JUNCTION[1]){
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
  uint8_t current=0;
  motorLeft  -> setSpeed(TOP_SPEED);
  motorRight -> setSpeed(TOP_SPEED);
  motorLeft  -> run(FORWARD);
  motorRight -> run(FORWARD);
  digitalWrite(pinMoving, LOW);
  do{
    junction_detector(); 
  } while( JUNCTION[0]=='0' && JUNCTION[1]=='0' );
  is_in_box=false;
}

void line_follower(void)
{
  /*
  In an iteration, this pipeline function does the following: 
    1. setting the state of the IR sensors
    2. checks if there is a junction.                           
    3. computes the required action based on the current output relative to the desired state ( e=0 state ).
  */
  get_state();
  junction_detector();
  if (is_region && !cube_to_destination){
    ultrasonic_lookup("front");
  }
  get_error();
  float x = get_control_signal();

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
  if(is_in_box && (!cube_to_destination) )
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
