#include <Adafruit_MotorShield.h>
Servo servoArm;
Servo servoClaw;

const int arm_down = 0;
const int arm_up = 0;
const int claw_close = 45;
const int claw_open = 0;


void setup() {
  servoArm.attach(9); // TBD
  servoClaw.attach(10); // TBD
}

void loop() {
  //descend claw
  servoArm.write(arm_down);
  delay(1000);
  //lift claw
  servoArm.write(arm_up);
  delay(1000);
  //close claw 
  servoClaw.write(claw_close);
  delay(1000);
  //open claw
  servoArm.write(claw_open);
  delay(1000);
}