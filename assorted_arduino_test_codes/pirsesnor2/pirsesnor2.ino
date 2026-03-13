#include <Servo.h>

#define SERVO_PIN 9
#define PIR_PIN 2

const int MIN_US = 1390;
const int MAX_US = 1940;

const int SPEED_US_PER_SEC = 130;
const int UPDATE_MS = 15;

const unsigned long COOLDOWN_MS = 5000;   // ignore new triggers for 5s
const unsigned long HOLD_AT_TOP_MS = 500; // pause at top

Servo actuator;

int current_us = MIN_US;
unsigned long lastUpdate = 0;

bool wasHigh = false;
unsigned long lastTriggerTime = 0;

void moveToward(int target_us) {
  unsigned long now = millis();
  if (now - lastUpdate < (unsigned long)UPDATE_MS) return;
  lastUpdate = now;

  int step = (SPEED_US_PER_SEC * UPDATE_MS) / 1000;
  if (step < 1) step = 1;

  if (current_us < target_us) {
    current_us += step;
    if (current_us > target_us) current_us = target_us;
    actuator.writeMicroseconds(current_us);
  } else if (current_us > target_us) {
    current_us -= step;
    if (current_us < target_us) current_us = target_us;
    actuator.writeMicroseconds(current_us);
  }
}

void setup() {
  pinMode(PIR_PIN, INPUT);
  actuator.attach(SERVO_PIN);
  actuator.writeMicroseconds(current_us);
}

void loop() {
  int pir = digitalRead(PIR_PIN);

  // Rising edge detect (LOW -> HIGH)
  bool rising = (pir == HIGH && !wasHigh);
  wasHigh = (pir == HIGH);

  unsigned long now = millis();

  if (rising && (now - lastTriggerTime > COOLDOWN_MS)) {
    lastTriggerTime = now;

    // Extend smoothly
    while (current_us != MAX_US) {
      moveToward(MAX_US);
    }
    delay(HOLD_AT_TOP_MS);

    // Retract smoothly
    while (current_us != MIN_US) {
      moveToward(MIN_US);
    }
  }
}

