/*
 * test_bts7960.ino — Command-driven hardware test for 2x BTS7960 modules.
 *
 * Waits for single-character commands over Serial (115200 baud).
 * Motors hold their last command until you send a new one.
 * No automatic loop, no timeout.
 *
 * Commands:
 *   f  — Forward
 *   b  — Backward
 *   l  — Spin left  (left back, right forward)
 *   r  — Spin right (left forward, right back)
 *   s  — Stop
 *
 * Pin mapping (must match Robot_Node_BTS7960.ino):
 *   Left  BTS7960: RPWM=4, LPWM=5, R_EN=6, L_EN=7
 *   Right BTS7960: RPWM=8, LPWM=9, R_EN=10, L_EN=11
 */

#define RPWM_LEFT   5
#define LPWM_LEFT   4
#define R_EN_LEFT   6
#define L_EN_LEFT   7

#define RPWM_RIGHT  8
#define LPWM_RIGHT  9
#define R_EN_RIGHT  10
#define L_EN_RIGHT  11

#define TEST_PWM 100  // 0-255, ~40% duty cycle

void motor(int rpwm, int lpwm, int pwm) {
  // pwm > 0 = forward, < 0 = reverse, 0 = stop
  int mag = constrain(abs(pwm), 0, 255);
  if (pwm > 0) {
    analogWrite(rpwm, mag);
    analogWrite(lpwm, 0);
  } else if (pwm < 0) {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, mag);
  } else {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, 0);
  }
}

void setup() {
  pinMode(R_EN_LEFT,  OUTPUT); digitalWrite(R_EN_LEFT,  HIGH);
  pinMode(L_EN_LEFT,  OUTPUT); digitalWrite(L_EN_LEFT,  HIGH);
  pinMode(R_EN_RIGHT, OUTPUT); digitalWrite(R_EN_RIGHT, HIGH);
  pinMode(L_EN_RIGHT, OUTPUT); digitalWrite(L_EN_RIGHT, HIGH);

  motor(RPWM_LEFT,  LPWM_LEFT,  0);
  motor(RPWM_RIGHT, LPWM_RIGHT, 0);

  Serial.begin(115200);
  Serial.println("Ready. Commands: f b l r s");
}

void loop() {
  if (Serial.available() == 0) return;

  char cmd = Serial.read();

  switch (cmd) {
    case 'f':
      motor(RPWM_LEFT,  LPWM_LEFT,   TEST_PWM);
      motor(RPWM_RIGHT, LPWM_RIGHT,  TEST_PWM);
      Serial.println("Forward");
      break;
    case 'b':
      motor(RPWM_LEFT,  LPWM_LEFT,  -TEST_PWM);
      motor(RPWM_RIGHT, LPWM_RIGHT, -TEST_PWM);
      Serial.println("Backward");
      break;
    case 'l':
      motor(RPWM_LEFT,  LPWM_LEFT,  -TEST_PWM);
      motor(RPWM_RIGHT, LPWM_RIGHT,  TEST_PWM);
      Serial.println("Spin left");
      break;
    case 'r':
      motor(RPWM_LEFT,  LPWM_LEFT,   TEST_PWM);
      motor(RPWM_RIGHT, LPWM_RIGHT, -TEST_PWM);
      Serial.println("Spin right");
      break;
    case 's':
      motor(RPWM_LEFT,  LPWM_LEFT,  0);
      motor(RPWM_RIGHT, LPWM_RIGHT, 0);
      Serial.println("Stop");
      break;
  }
}
