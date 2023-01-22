const int analoguePin = A0;
int val = 0;
float sensorInput = 0.0;
void setup() {
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  val = analogRead(analoguePin);
  sensorInput = val/1023.0 * 4.42;
  Serial.println(sensorInput);
  delay(500);
}
