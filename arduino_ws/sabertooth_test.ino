/*
 * Sabertooth Test Sketch
 * Upload this to verify Sabertooth communication
 */

#include <Sabertooth.h>

Sabertooth ST(128, Serial1);

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);

  Serial.println("Sabertooth Test Starting...");
  ST.autobaud();
  delay(2000);
  Serial.println("Autobaud complete. Testing motors...");
}

void loop() {
  Serial.println("Motor 1 forward 50");
  ST.motor(1, 50);
  delay(2000);

  Serial.println("Motor 1 stop");
  ST.motor(1, 0);
  delay(1000);

  Serial.println("Motor 2 forward 50");
  ST.motor(2, 50);
  delay(2000);

  Serial.println("Motor 2 stop");
  ST.motor(2, 0);
  delay(1000);

  Serial.println("Both motors forward 50");
  ST.motor(1, 50);
  ST.motor(2, 50);
  delay(2000);

  Serial.println("Both motors stop");
  ST.motor(1, 0);
  ST.motor(2, 0);
  delay(3000);
}
