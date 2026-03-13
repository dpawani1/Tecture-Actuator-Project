#include <Stepper.h>

const int stepsPerRevolution = 2048;
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);

const int potPin = A0;
int potValue;
int centered;
int speedValue;

const int deadZone = 80; // Increase this to increase the deadzone in the middle.

void setup() {
  Serial.begin(9600);
}

void loop() {
  potValue = analogRead(potPin);    // 0–1023
  centered = potValue - 512;        // -512 → +511

  // Check if pot is near middle
  if (abs(centered) < deadZone) {
    return;
  }

  // Outside dead zone: compute speed and direction
  speedValue = map(abs(centered), deadZone, 511, 1, 10);
  myStepper.setSpeed(speedValue);

  int stepSize = stepsPerRevolution / 100; // change 100 lower to reduce vibration

  if (centered > 0) {
    myStepper.step(stepSize);  // forward
  } else {
    myStepper.step(-stepSize); // reverse
  }

  delay(5);
}

