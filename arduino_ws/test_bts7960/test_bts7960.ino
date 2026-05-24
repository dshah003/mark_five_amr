/*
 * test_bts7960.ino
 *
 * Standalone hardware test for 2x BTS7960 motor driver modules.
 * NO serial commands or ROS2 needed — just upload and watch the motors.
 *
 * Test sequence (repeats forever):
 *   1. Both motors forward  at 40% — 2 seconds
 *   2. Stop                          — 1 second
 *   3. Both motors reverse  at 40% — 2 seconds
 *   4. Stop                          — 1 second
 *   5. Spin in place left            — 1.5 seconds
 *   6. Stop                          — 1 second
 *   7. Spin in place right           — 1.5 seconds
 *   8. Stop                          — 2 seconds
 *
 * Pin mapping must match Robot_Node_BTS7960.ino exactly.
 *
 * Serial monitor (115200 baud) shows what each step is doing.
 */

// Left BTS7960
#define RPWM_LEFT   4
#define LPWM_LEFT   5
#define R_EN_LEFT   6
#define L_EN_LEFT   7

// Right BTS7960
#define RPWM_RIGHT  8
#define LPWM_RIGHT  9
#define R_EN_RIGHT  10
#define L_EN_RIGHT  11

// Test speed: 40% duty cycle (0-255)
#define TEST_PWM 100

void motor_left(int pwm) {
  // pwm > 0 → forward, < 0 → reverse, 0 → stop
  int mag = constrain(abs(pwm), 0, 255);
  if (pwm > 0) {
    analogWrite(RPWM_LEFT, mag);
    analogWrite(LPWM_LEFT, 0);
  } else if (pwm < 0) {
    analogWrite(RPWM_LEFT, 0);
    analogWrite(LPWM_LEFT, mag);
  } else {
    analogWrite(RPWM_LEFT, 0);
    analogWrite(LPWM_LEFT, 0);
  }
}

void motor_right(int pwm) {
  int mag = constrain(abs(pwm), 0, 255);
  if (pwm > 0) {
    analogWrite(RPWM_RIGHT, mag);
    analogWrite(LPWM_RIGHT, 0);
  } else if (pwm < 0) {
    analogWrite(RPWM_RIGHT, 0);
    analogWrite(LPWM_RIGHT, mag);
  } else {
    analogWrite(RPWM_RIGHT, 0);
    analogWrite(LPWM_RIGHT, 0);
  }
}

void stop_all() {
  motor_left(0);
  motor_right(0);
}

void setup() {
  Serial.begin(115200);

  // Enable both modules
  pinMode(R_EN_LEFT,  OUTPUT); digitalWrite(R_EN_LEFT,  HIGH);
  pinMode(L_EN_LEFT,  OUTPUT); digitalWrite(L_EN_LEFT,  HIGH);
  pinMode(R_EN_RIGHT, OUTPUT); digitalWrite(R_EN_RIGHT, HIGH);
  pinMode(L_EN_RIGHT, OUTPUT); digitalWrite(L_EN_RIGHT, HIGH);

  stop_all();

  Serial.println("BTS7960 test starting in 2 seconds...");
  Serial.println("Pins: LEFT RPWM=4 LPWM=5 EN=6,7 | RIGHT RPWM=8 LPWM=9 EN=10,11");
  delay(2000);
}

void loop() {
  Serial.println("--- FORWARD ---");
  motor_left(TEST_PWM);
  motor_right(TEST_PWM);
  delay(2000);

  Serial.println("--- STOP ---");
  stop_all();
  delay(1000);

  Serial.println("--- REVERSE ---");
  motor_left(-TEST_PWM);
  motor_right(-TEST_PWM);
  delay(2000);

  Serial.println("--- STOP ---");
  stop_all();
  delay(1000);

  Serial.println("--- SPIN LEFT (left back, right forward) ---");
  motor_left(-TEST_PWM);
  motor_right(TEST_PWM);
  delay(1500);

  Serial.println("--- STOP ---");
  stop_all();
  delay(1000);

  Serial.println("--- SPIN RIGHT (left forward, right back) ---");
  motor_left(TEST_PWM);
  motor_right(-TEST_PWM);
  delay(1500);

  Serial.println("--- STOP --- (2s pause before repeat) ---");
  stop_all();
  delay(2000);
}
