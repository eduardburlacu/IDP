#include <Adafruit_MotorShield.h>

Adafruit_MotorShield  AFMS   = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft  = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set the speed to start, from 0 (off) to 255 (max speed)
  motorLeft -> setSpeed(150);
  motorRight-> setSpeed(150);
  motorLeft -> run(FORWARD);
  motorRight-> run(FORWARD);
  // turn on motor
  motorLeft -> run(RELEASE);
  motorRight-> run(RELEASE);
}

void loop() {
  uint8_t i;
  Serial.print("tick");

  motorLeft  -> run(FORWARD);
  motorRight -> run(FORWARD);
  for (i=0; i<255; i++) {
    motorLeft -> setSpeed(i);
    motorRight-> setSpeed(i);
    delay(10);
  }
  for (i=255; i!=0; i--) {
    motorLeft -> setSpeed(i);
    motorRight-> setSpeed(i);
    delay(10);
  }

  Serial.print("tock");
  motorLeft  -> run(BACKWARD);
  motorRight -> run(BACKWARD);
  for (i=0; i<255; i++) {
    motorLeft -> setSpeed(i);
    motorRight-> setSpeed(i);
    delay(10);
  }
  for (i=255; i!=0; i--) {
    motorLeft -> setSpeed(i);
    motorRight-> setSpeed(i);
    delay(10);
  }
  Serial.print("tech");
  motorLeft  -> run(RELEASE);
  motorRight -> run(RELEASE);
  delay(1000);
}
