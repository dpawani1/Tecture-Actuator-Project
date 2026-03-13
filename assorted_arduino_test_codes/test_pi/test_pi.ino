#include <Servo.h>

Servo lin;

const int SERVO_PIN = 9;

// Your old pulse endpoints (keep these unless you know your actuator needs different)
const int MIN_US = 1390;
const int MAX_US = 1940;

const int SPEED_US_PER_SEC = 130;
const int UPDATE_MS = 15;

const unsigned long HOLD_AT_TOP_MS = 500;

enum Phase { IDLE, EXTEND, HOLD_TOP, RETRACT };
Phase phase = IDLE;

int current_us = MIN_US;
int target_us  = MIN_US;

unsigned long lastUpdate = 0;
unsigned long phaseStart = 0;

void enterPhase(Phase p) {
  phase = p;
  phaseStart = millis();
}

bool stepTowardTarget() {
  unsigned long now = millis();
  if (now - lastUpdate < (unsigned long)UPDATE_MS) return false;
  lastUpdate = now;

  int step = (SPEED_US_PER_SEC * UPDATE_MS) / 1000;
  if (step < 1) step = 1;

  if (current_us < target_us) {
    current_us += step;
    if (current_us > target_us) current_us = target_us;
    lin.writeMicroseconds(current_us);
  } else if (current_us > target_us) {
    current_us -= step;
    if (current_us < target_us) current_us = target_us;
    lin.writeMicroseconds(current_us);
  }

  return (current_us == target_us);
}

void setup() {
  Serial.begin(9600);

  lin.attach(SERVO_PIN);
  lin.writeMicroseconds(MIN_US);

  Serial.println("READY");
}

void loop() {
  unsigned long now = millis();

  // Pi sends:
  // 'T' = trigger (extend -> hold -> retract)
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'T' && phase == IDLE) {
      target_us = MAX_US;
      enterPhase(EXTEND);
    }
  }

  switch (phase) {
    case IDLE:
      break;

    case EXTEND:
      if (stepTowardTarget()) enterPhase(HOLD_TOP);
      break;

    case HOLD_TOP:
      if (now - phaseStart >= HOLD_AT_TOP_MS) {
        target_us = MIN_US;
        enterPhase(RETRACT);
      }
      break;

    case RETRACT:
      if (stepTowardTarget()) enterPhase(IDLE);
      break;
  }
}