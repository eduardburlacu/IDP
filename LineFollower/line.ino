#include <Adafruit_MotorShield.h>
#include "Wire.h"
#include <Servo.h>
// Define the relevant state variables

#define REFRESH_TIME 10 		// Time in miliseconds after which a new update is provided
#define NUM_READ 8				  // Number of points to use for average
char  LINE[] = "00";				// Variable encoding the state of the 2 IR sensors for line following.
char  JUNCTION[] = "00";	  // Variable encoding the state of the 2 IR sensors for junction detection.
int   speedLeft = 0;        // Variable for the speed of the left  DC Motor, range 0->255
int   speedRight =0;        // Variable for the speed of the right DC Motor, range 0->255
float THETA = 0.0;			    // Angle of rotation relative to the line to be followed.
float PREV_THETA = 0.0;     // Previous step -> angle of rotation relative to the line to be followed.
float TOTAL_THETA = 0.0;		// Discrete integral of rotation relative to the line to be followed.
float Kd, Kp, Ki;           // PID controller parameters. Need to be found analitically after the mechanics is done!!

// Define the pin numbers for sensors and motors. To be filled in with Andrew after they finish the electrical circuits!!

#define pinSensorL
#define pinSensorR
#define pinSensorLL
#define pinSensorRR
#define pinRedLed
#define pinBlueLed

// Prototypes for functions

float moving_avg(int N, int len,float v[]);
float moving_avg(int len, float v[]);
float control(float v[], float iv[]);
float get_error(void);
void search_cube(void);
void junction_search_cube(void);
void lift_cube(void);
void turnLeft(void);
void turnLeft(int angle);
void turnRight(void);
void turnRight(int angle);
void blinkLed(void);

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

float get_error(void){
  if ( (LINE=="01") || (LINE=="10") ){return 1.0;}
  else {return 0.0;}
}

float control(void){ 
  // Used for the feedback loop of the parameter. The new value of the motor speed is set accordingly.
  return Kp * THETA + Ki * TOTAL_THETA + Kd * (THETA - PREV_THETA); 
  }

Adafruit_MotorShield  AFMS   = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft  = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);

void setup() {
  //AFMS.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(2);
}
