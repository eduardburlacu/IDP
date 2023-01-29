#include <Adafruit_MotorShield.h>
#include "Wire.h"
#include <Servo.h>
#include <NewPing.h>

// Define the relevant state variables
extern bool colorRed  = false;
extern bool colorDetected = false;
extern bool cubeDetectedFront = false;
extern bool cubeDetectedSide  = false;
extern bool in_junction = false;
extern bool in_tunnel = false;
extern bool is_in_box = true;        //V True if the robot is inside the start/end point.
extern bool is_region = false;       // True if the robot has reached the region where cubes are expected.        !!!!!!! ADD STH SO THAT IT GETS TRIGGERED !!!!!!!!!!!
extern bool cube_to_destination=false;
extern bool destination_reached=false;
extern unsigned int cubeDistance=0;
extern unsigned int distance_front;  // Get average distance reading (discard anomalies)

extern uint8_t JUNCTION[]= {0, 0};   // Variable encoding the state of the 2 IR sensors for junction detection. 
extern uint8_t LINE[]    = {0, 0};   // Variable encoding the state of the 2 IR sensors for line following.                 
extern uint8_t speedLeft=0;          // Variable for the speed of the left  DC Motor, range 0->255
extern uint8_t speedRight=0;         // Variable for the speed of the right DC Motor, range 0->255 
extern uint8_t state=0;

extern float TunnelKd=0.1, TunnelKp=1, TunnelKi=0.2; // Tunnel PID controller parameters. Need to be found analitically after the mechanics is done!!
extern const uint8_t TunnelDistance = 0; // desired distance from tunnel wall
extern const uint8_t TunnelExit = 0; // distance reading found at tunnel exit
extern uint8_t claw_open = 0;       // Servo motor angle for open claw
extern uint8_t claw_close = 0;      // Servo motor angle for closed claw
extern uint8_t arm_up = 0;          // Servo motor angle for elevated arm
extern uint8_t arm_down = 0;        // Servo motor angle for lowered arm
extern uint8_t pickup_distance = 0; // Distance from ultrasound where robot stops
extern const uint8_t truck_distance = 0; // Distance from truck where robot stops

extern float THETA = 0.0;			       // Angle of rotation relative to the line to be followed.
extern float PREV_THETA = 0.0;       // Previous step -> angle of rotation relative to the line to be followed.
extern float TOTAL_THETA = 0.0;		   // Discrete integral of rotation relative to the line to be followed.


//       !!!!! PARAMETERS TO BE TUNED BY EXPERIMENT !!!!!!

#define DELTA        0    // Compensation caused by the imbalance in weight of distribution in the robot. Make it positive if it drifts to the left!!
#define NUM_READ     8  	// Number of points to use for average
#define RANGE_FRONT  100  // Maximum distance allowed to the frontal ultrasonic sensor.
#define RANGE_SIDE   100  // Maximum distance allowed to the side ultrasonic sensor.
#define REFRESH_TIME 10 	// Time in miliseconds after which a new update is provided
#define TAU           100 // The required time(ms) to perform a 90 deg turn.
#define TOP_SPEED     150 // The final speed of the robot as ratio to the maximum available input from the DC motor.
#define ROT_SPEED     100 // The wheel's complementary speeds when performing a rotation

#define Kd 0.1            // PID controller parameters            
#define Kp 1              // Need to be found analitically 
#define Ki 0.2            // after the mechanics is done!!


// Define the pin numbers for sensors and motors. To be filled in with Andrew after they finish the electrical circuits!!


#define pinL  1
#define pinR  2
#define pinLL 3
#define pinRR 4
#define pinColorRed 20
#define pinColorTrig 21
#define pinMoving 10
#define pinMotorLeft
#define pinMotorRight
#define pinPotServo      // Analog  pin used to connect the potentiometer
#define pinFrontTrig 6   // Digital pin used to connect the receiver    of the HC-SR04 ultrasonic front sensor
#define pinFrontEcho 7   // Digital pin used to connect the transmitter of the HC-SR04 ultrasonic front sensor
#define pinSideTrig  8   // Digital pin used to connect the receiver    of the HC-SR04 ultrasonic side  sensor
#define pinSideEcho  9   // Digital pin used to connect the transmitter of the HC-SR04 ultrasonic side  sensor   
#define pinArm  5
#define pinClaw 6

// Prototypes for functions
float get_control_signal(void);
float get_control_signal_tunnel(int diff, int totalDiff, int prevDiff);
float moving_avg(int N, int len,float v[]);
float moving_avg(int len, float v[]);

uint8_t measure(int pin);
unsigned int final_distance_reading(void);

void accelerate(uint8_t initial_speed, uint8_t final_speed, uint8_t step_size, bool reverse);
void blinkLed(void);
void cube_retrieval(void);
void get_error(void);
void get_state(void);
void junction_search_cube(void);
void junction_detector(void);
void leave_box(void);
void lift_cube(void);
void line_follower(void);
void block_retrieval(void);
void block_placement();
void tunel_navigation(void);
void search_cube(void);
void turnLeft(void);
void turnLeft(uint8_t angle);
void turnRight(void);
void turnRight(uint8_t angle);
void ultrasonic_lookup(void);
void update_region(void);
// Some general purpose functions

void update_region(void)
{
   if (state == 3 || state == 4 || state == 5) {
    is_region = true;
  } else {
    is_region = false;
}
}

float moving_avg(int N, int len, float v[])
{
    /*
    Use it to get rid of random fluctuations in reading.
    N is the number of points considered to the average.
    len is the length of v
    v[] stores the last recorded values of a variable to be averaged.
    if N < len , only considers the last N readings in v[].
    */
    float average_total = 0.0;
    for (int index = len- N; index < len; index++) {
        average_total += v[index];
    }
    average_total /= N;
    return average_total;
}

float moving_avg(int len, float v[])
{
    float average_total = 0.0;
    for (int index = 0; index < len; index++) {
        average_total += v[index];
    }
    average_total /= len;
    return average_total;
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

void get_state(void)
{
  uint8_t v[4]={0,0,0,0};
  for (int i=0;i<NUM_READ;i++)
  {
  LINE[0]     += digitalRead(pinL);
  LINE[1]     += digitalRead(pinR);
  JUNCTION[0] += digitalRead(pinLL);
  JUNCTION[0] += digitalRead(pinRR);
  bool is_region;
  delay(2);
  }

  for (int j=0;j<2;j++)
  {  
    LINE[j]     = (LINE[j]    + NUM_READ - 1)/NUM_READ;
    JUNCTION[j] = (JUNCTION[j]+ NUM_READ - 1)/NUM_READ;
  } 
}
