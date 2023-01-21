const int trigPin = 8;
const int echoPin = 9;
unsigned long distance;
unsigned long duration;

void setup() {
  // put your setup code here, to run once:
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  distance = readUltrasonicDistance();
  Serial.print(distance, DEC);
  Serial.print("\r\n");
  delay(60);
}

unsigned long readUltrasonicDistance(){
  //Send out trigger pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  //Read echo pulse length
  duration = pulseIn(echoPin, HIGH);
  return duration/58;

}