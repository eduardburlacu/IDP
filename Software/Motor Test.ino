#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

void setup() {
  Servo servoA;                                     //Create a servo object; need one per servo
  
  Adafruit_MotorShield MS = Adafruit_MotorShield(); 
  Adafruit_DCMotor *motorA = MS.getMotor(1);        //Need one of these lines per motor, #s 1-4 
  Adafruit_DCMotor *motorB = MS.getMotor(2);        
  MS.begin();
  
  motorA->setSpeed(150);                            //Sets the speed of the DC motor, must be done before the motors can work
  motorB->setSpeed(150);
  motorA->run(FORWARD);                             //Modifies the state of the motor , pass FORWARD, BACKWARD or RELEASE which cuts the power to the motor
  motorB->run(FORWARD); 

  servoA.attach(9);                                  //Needed to initalise the servos, it wants the pin number which is 10 for Servo1 and 9 for Servo2 in our case
  servoA.write(0);                                   //It wants the angle in degrees of the position to move to
}

void loop() {
  // put your main code here, to run repeatedly:

}
