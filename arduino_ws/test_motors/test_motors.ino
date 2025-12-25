const int enA = 7;
const int in1 = 22;
const int in2 = 23;
const int enB = 6;
const int in3 = 24;
const int in4 = 25;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Set direction forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void loop() {
  // Test at 100% PWM
  analogWrite(enA, 255);
  analogWrite(enB, 255);
  delay(3000);

  // Stop
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  delay(2000);
}
