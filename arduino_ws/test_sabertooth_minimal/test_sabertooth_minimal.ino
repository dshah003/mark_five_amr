/*
 * Sabertooth Motor Test - Independent Motor Control
 *
 * Based on TankStyleSweep example but using ST.motor() for
 * independent left/right control (better for ROS differential drive).
 *
 * DIP Switches: OFF OFF ON ON ON ON (Packetized Serial, Address 128)
 *
 * Wiring (Arduino Mega):
 *   Pin 18 (TX1) --> Sabertooth S1
 *   GND --> 0V
 *
 * Test Sequence:
 *   1. Motor 1 forward (3s)
 *   2. Stop (2s)
 *   3. Motor 2 forward (3s)
 *   4. Stop (2s)
 *   5. Both forward (3s)
 *   6. Both reverse (3s)
 *   7. Stop (3s)
 */

#include <Sabertooth.h>

// Address 128, using Serial1 (Pin 18 TX on Mega)
Sabertooth ST(128, Serial1);

void setup() {
  Serial1.begin(9600);
  ST.autobaud();

  delay(1000);

  // Stop both motors
  ST.motor(1, 0);
  ST.motor(2, 0);
  delay(1000);
}

void loop() {
  // Phase 1: Motor 1 forward only
  ST.motor(1, 60);
  ST.motor(2, 0);
  delay(3000);

  // Stop
  ST.motor(1, 0);
  ST.motor(2, 0);
  delay(2000);

  // Phase 2: Motor 2 forward only
  ST.motor(1, 0);
  ST.motor(2, 60);
  delay(3000);

  // Stop
  ST.motor(1, 0);
  ST.motor(2, 0);
  delay(2000);

  // Phase 3: Both forward
  ST.motor(1, 60);
  ST.motor(2, 60);
  delay(3000);

  // Phase 4: Both reverse
  ST.motor(1, -60);
  ST.motor(2, -60);
  delay(3000);

  // Stop
  ST.motor(1, 0);
  ST.motor(2, 0);
  delay(3000);
}
