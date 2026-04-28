int led = 13;

void setup() {
  pinMode(led, OUTPUT);
}

void loop() {
  digitalWrite(led, HIGH);
  //delayMicroseconds(1);
  //delay(1);
  digitalWrite(led, LOW);
  delay(998);
  delayMicroseconds(1420);
}
