#include <iostream>
#include <assert.h>

// Define the relevant state variables

#define REFRESH_TIME 100.0		// Time in miliseconds after which a new update is provided
#define NUM_READ 10				// Number of points to use for average
char LINE[] = "00";				// Variable encoding the state of the 2 IR sensors for line following.
char JUNCTION[] = "00";			// Variable encoding the state of the 2 IR sensors for junction detection.
float THETA = 0.0;				// Angle of rotation relative to the line to be followed.
float TOTAL_THETA = 0.0;		// Discrete integral of rotation relative to the line to be followed.

// Define the pin numbers for sensors and motors. To be filled in with Andrew after they finish the electrical circuits!!

#define pinSensorLeft
#define pinSensorRight
#define pinJunctionLeft
#define pinJunctionRight
#define pinMotorLeft
#define pinMotorRight
#define pinRedLed
#define pinBlueLed

// Prototypes for functions

float moving_avg(int N, int len,float v[]);
float moving_avg(int len, float v[]);
float control(float v[], float iv[]);
void update_time(void);
void lift_cube(void);


