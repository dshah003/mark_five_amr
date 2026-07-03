/*
 * Robot_Node_BTS7960.ino  —  Closed-loop differential drive
 *
 * Per-wheel PI velocity control (encoder feedback) with feedforward,
 * PWM slew-rate limiting, and matched 3.9 kHz PWM on all motor pins.
 *
 * Serial Protocol:
 *   RX: "v,<linear_x>,<angular_z>\n"   (from ROS serial_bridge)
 *   TX: "t,<left_ticks>,<right_ticks>\n" (to ROS serial_bridge)
 *
 * Wiring — REV B (pins 4 and 7 SWAPPED vs REV A):
 *   Left  BTS7960: RPWM→7, LPWM→5, R_EN→6,  L_EN→4
 *   Right BTS7960: RPWM→8, LPWM→9, R_EN→10, L_EN→11
 *   Left  encoder: A→21, B→20
 *   Right encoder: A→3,  B→2
 *
 * Why the swap: pin 4 is on Timer0, whose prescaler can't change without
 * breaking millis(). With RPWM on 7, all four PWM pins sit on Timers 2/3/4,
 * which are retimed to 3.92 kHz (default 490 Hz is rough and audible;
 * BTS7960 is rated to 25 kHz).
 */

#define SERIAL_BAUD      115200
#define LOOP_INTERVAL    30     // ms between control updates + tick publishes

// ── Pin definitions (REV B) ───────────────────────────────────────────────────

#define RPWM_LEFT   7
#define LPWM_LEFT   5
#define R_EN_LEFT   6
#define L_EN_LEFT   4

#define RPWM_RIGHT  8
#define LPWM_RIGHT  9
#define R_EN_RIGHT  10
#define L_EN_RIGHT  11

#define ENC_IN_LEFT_A   21
#define ENC_IN_LEFT_B   20
#define ENC_IN_RIGHT_A  3
#define ENC_IN_RIGHT_B  2

// ── Physical parameters ───────────────────────────────────────────────────────

const double WHEEL_BASE = 0.36;   // meters, center-to-center of wheels

// Per-wheel calibration (must match odometry.yaml — 2x A-channel CHANGE decoder)
// The ~8% L/R difference is real hardware asymmetry in the right encoder.
const double TICKS_PER_METER_LEFT  = 4594.0;
const double TICKS_PER_METER_RIGHT = 4255.0;

// ── Control parameters ────────────────────────────────────────────────────────

// Feedforward: PWM ≈ K_FF * |vel| + FF_OFFSET (gets the output near the right
// value immediately; the PI loop corrects the residual). Stall is around
// PWM 60 unloaded — the integrator winds through stiction if FF undershoots.
const double K_FF      = 278.0;
const double FF_OFFSET = 52.0;

// PI gains (PWM counts per m/s of error, PWM counts per m of integrated error)
const double KP_VEL = 150.0;
const double KI_VEL = 600.0;
const double INTEGRAL_MAX = 0.1;  // clamp so KI * integral ≤ 60 PWM counts

// Low-pass filter on measured wheel velocity (1 tick / 30 ms ≈ 0.007 m/s
// quantization). alpha = weight of the newest sample.
const double VEL_FILTER_ALPHA = 0.4;

const int PWM_MAX  = 160;  // ~63% duty; v ≈ 0.39 m/s open-loop equivalent
const int SLEW_MAX = 10;   // max PWM change per 30 ms loop (0→160 in ~0.5 s)

const double V_DEADBAND = 0.01;  // m/s; below this, target is treated as stop

// ── Encoder state (modified by ISRs) ─────────────────────────────────────────

volatile int16_t left_wheel_tick_count  = 0;
volatile int16_t right_wheel_tick_count = 0;

// ── Runtime state ─────────────────────────────────────────────────────────────

double velLeftTarget  = 0.0;   // m/s, positive = forward
double velRightTarget = 0.0;

double velLeftMeas  = 0.0;     // filtered measured wheel velocity, m/s
double velRightMeas = 0.0;

double integralLeft  = 0.0;    // integrated velocity error, m
double integralRight = 0.0;

int pwmLeftOut  = 0;           // signed PWM currently applied (after slew)
int pwmRightOut = 0;

int16_t prevTicksLeft  = 0;    // tick snapshot from previous control loop
int16_t prevTicksRight = 0;

unsigned long lastCmdVelReceived = 0;
unsigned long previousMillis     = 0;

char    cmdBuf[48];
uint8_t cmdLen = 0;

// ── Encoder ISRs ──────────────────────────────────────────────────────────────
// CHANGE on A channel only. When A changes, B has been stable for ~90° of
// rotation (mid-phase), so direction reads are reliable with no timing races.
// The B channel is not used as an interrupt source — its short pulses caused
// missed edges and asymmetric counts on the right encoder.
// Forward motion DECREMENTS the raw count; publishTicks() and the control
// loop both negate, so ROS and the PI loop see forward as positive.

void right_wheel_tick() {
  boolean a = digitalRead(ENC_IN_RIGHT_A);
  boolean b = digitalRead(ENC_IN_RIGHT_B);
  // A leads B during physical forward: A rising + B low, or A falling + B high
  boolean forward = a ? !b : b;
  if (forward) {
    right_wheel_tick_count = (right_wheel_tick_count == -32768) ? 32767  : right_wheel_tick_count - 1;
  } else {
    right_wheel_tick_count = (right_wheel_tick_count == 32767)  ? -32768 : right_wheel_tick_count + 1;
  }
}

void left_wheel_tick() {
  boolean a = digitalRead(ENC_IN_LEFT_A);
  boolean b = digitalRead(ENC_IN_LEFT_B);
  // Left motor inverted — B leads A during physical forward: A rising + B high, or A falling + B low
  boolean forward = a ? b : !b;
  if (forward) {
    left_wheel_tick_count = (left_wheel_tick_count == -32768) ? 32767  : left_wheel_tick_count - 1;
  } else {
    left_wheel_tick_count = (left_wheel_tick_count == 32767)  ? -32768 : left_wheel_tick_count + 1;
  }
}

// ── Motor helpers ─────────────────────────────────────────────────────────────

void set_motor(int rpwm_pin, int lpwm_pin, int signed_pwm) {
  int mag = constrain(abs(signed_pwm), 0, 255);
  if (signed_pwm >= 0) {
    analogWrite(rpwm_pin, mag);
    analogWrite(lpwm_pin, 0);
  } else {
    analogWrite(rpwm_pin, 0);
    analogWrite(lpwm_pin, mag);
  }
}

// ── Control ───────────────────────────────────────────────────────────────────

double feedforward(double vel) {
  if (vel > 0)  return  K_FF * vel + FF_OFFSET;
  if (vel < 0)  return  K_FF * vel - FF_OFFSET;
  return 0.0;
}

// One PI update for one wheel. Returns the new signed PWM (pre-slew).
int velocityPi(double target, double measured, double dt, double &integral) {
  if (target == 0.0) {
    // Commanded stop: no PI hunting around zero, just ramp the PWM down
    // (the slew limiter provides the ramp). Reset integrator for a clean
    // next start.
    integral = 0.0;
    return 0;
  }

  double error = target - measured;
  double u = feedforward(target) + KP_VEL * error + KI_VEL * integral;

  // Anti-windup: integrate only if output is unsaturated, or the error is
  // pulling the output back out of saturation.
  bool saturated = (u > PWM_MAX) || (u < -PWM_MAX);
  if (!saturated || (error > 0) != (u > 0)) {
    integral = constrain(integral + error * dt, -INTEGRAL_MAX, INTEGRAL_MAX);
  }

  return constrain((int)u, -PWM_MAX, PWM_MAX);
}

int slewLimit(int current, int requested) {
  return constrain(requested, current - SLEW_MAX, current + SLEW_MAX);
}

void controlUpdate(double dt) {
  noInterrupts();
  int16_t lt = left_wheel_tick_count;
  int16_t rt = right_wheel_tick_count;
  interrupts();

  // int16 subtraction handles counter wraparound; negate so forward = positive
  double dLeft  = -(double)((int16_t)(lt - prevTicksLeft));
  double dRight = -(double)((int16_t)(rt - prevTicksRight));
  prevTicksLeft  = lt;
  prevTicksRight = rt;

  double vLeftRaw  = dLeft  / TICKS_PER_METER_LEFT  / dt;
  double vRightRaw = dRight / TICKS_PER_METER_RIGHT / dt;
  velLeftMeas  += VEL_FILTER_ALPHA * (vLeftRaw  - velLeftMeas);
  velRightMeas += VEL_FILTER_ALPHA * (vRightRaw - velRightMeas);

  int pwmLeftReq  = velocityPi(velLeftTarget,  velLeftMeas,  dt, integralLeft);
  int pwmRightReq = velocityPi(velRightTarget, velRightMeas, dt, integralRight);

  pwmLeftOut  = slewLimit(pwmLeftOut,  pwmLeftReq);
  pwmRightOut = slewLimit(pwmRightOut, pwmRightReq);

  set_motor(RPWM_LEFT,  LPWM_LEFT,  pwmLeftOut);
  set_motor(RPWM_RIGHT, LPWM_RIGHT, pwmRightOut);
}

// ── Velocity command ──────────────────────────────────────────────────────────

void processCmdVel(double linear_x, double angular_z) {
  lastCmdVelReceived = millis();

  // Standard differential-drive kinematics.
  // angular_z > 0 = CCW = turn left → left wheel backward, right wheel forward.
  double leftVel  = linear_x - (WHEEL_BASE / 2.0) * angular_z;
  double rightVel = linear_x + (WHEEL_BASE / 2.0) * angular_z;

  velLeftTarget  = (fabs(leftVel)  < V_DEADBAND) ? 0.0 : leftVel;
  velRightTarget = (fabs(rightVel) < V_DEADBAND) ? 0.0 : rightVel;
}

// ── Serial ────────────────────────────────────────────────────────────────────

void parseCommand(const char *s) {
  if (s[0] != 'v' || s[1] != ',') return;
  char *end;
  double linear_x = strtod(s + 2, &end);
  if (*end != ',') return;
  double angular_z = strtod(end + 1, NULL);
  processCmdVel(linear_x, angular_z);
}

// Fixed buffer, parsed as soon as a full line arrives — no String heap churn,
// and back-to-back commands in one serialEvent() can't clobber each other.
void serialEvent() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      cmdBuf[cmdLen] = '\0';
      parseCommand(cmdBuf);
      cmdLen = 0;
    } else if (c != '\r') {
      if (cmdLen < sizeof(cmdBuf) - 1) {
        cmdBuf[cmdLen++] = c;
      } else {
        cmdLen = 0;  // overlong garbage line: discard and resync at next '\n'
      }
    }
  }
}

void publishTicks() {
  noInterrupts();
  int16_t lt = left_wheel_tick_count;
  int16_t rt = right_wheel_tick_count;
  interrupts();
  Serial.print("t,");
  Serial.print(-lt);
  Serial.print(",");
  Serial.println(-rt);
}

// ── Setup / Loop ──────────────────────────────────────────────────────────────

void setup() {
  // Enable both half-bridges on each BTS7960
  pinMode(R_EN_LEFT,  OUTPUT); digitalWrite(R_EN_LEFT,  HIGH);
  pinMode(L_EN_LEFT,  OUTPUT); digitalWrite(L_EN_LEFT,  HIGH);
  pinMode(R_EN_RIGHT, OUTPUT); digitalWrite(R_EN_RIGHT, HIGH);
  pinMode(L_EN_RIGHT, OUTPUT); digitalWrite(L_EN_RIGHT, HIGH);

  set_motor(RPWM_LEFT,  LPWM_LEFT,  0);
  set_motor(RPWM_RIGHT, LPWM_RIGHT, 0);

  // Retime motor PWM from the 490 Hz default to 3.92 kHz (phase-correct,
  // prescaler 64 → 8). Timer0 (millis) is untouched — no motor pin uses it.
  TCCR2B = (TCCR2B & 0b11111000) | 0x02;  // pin 9  (right LPWM)
  TCCR3B = (TCCR3B & 0b11111000) | 0x02;  // pin 5  (left LPWM)
  TCCR4B = (TCCR4B & 0b11111000) | 0x02;  // pins 7, 8 (left/right RPWM)

  // INPUT_PULLUP on all encoder pins reduces electrical noise
  pinMode(ENC_IN_LEFT_A,  INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B,  INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B, INPUT_PULLUP);

  // A-channel CHANGE only — B read inside ISR, not used as interrupt source
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A),  left_wheel_tick,  CHANGE);

  Serial.begin(SERIAL_BAUD);
}

void loop() {
  unsigned long now = millis();
  if (now - previousMillis >= LOOP_INTERVAL) {
    double dt = (now - previousMillis) / 1000.0;
    previousMillis = now;

    // Safety stop: zero targets if no command received for 1 second
    if (now - lastCmdVelReceived > 1000) {
      velLeftTarget  = 0.0;
      velRightTarget = 0.0;
    }

    controlUpdate(dt);
    publishTicks();
  }
}
