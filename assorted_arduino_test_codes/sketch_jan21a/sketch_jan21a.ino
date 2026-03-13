#include <Servo.h>

#define SERVO_PIN 9

// Your calibrated endpoints
const int MIN_US = 1390;
const int MAX_US = 1940;

Servo actuator;

bool running = false;
int current_us = MIN_US;

// ===== SPEED KNOB =====
// How fast to change the command, in microseconds per second.
// Smaller = slower/quieter. Try: 30, 60, 120, 200
const int SPEED_US_PER_SEC = 130;

// Update interval (ms). 20ms matches typical servo frame rate nicely.
const int UPDATE_MS = 15;

unsigned long lastUpdate = 0;
bool goingOut = true; // extend then retract

void setup() {
  Serial.begin(9600);
  Serial.println("SPACE = start/stop");
  actuator.attach(SERVO_PIN);
  actuator.writeMicroseconds(current_us);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == ' ') {
      running = !running;
      Serial.println(running ? "Running" : "Stopped");
    }
  }

  if (!running) return;

  unsigned long now = millis();
  if (now - lastUpdate < (unsigned long)UPDATE_MS) return;
  lastUpdate = now;

  // compute how many microseconds to step this tick
  // step = (us/sec) * (sec/tick) = SPEED_US_PER_SEC * UPDATE_MS/1000
  int step = (SPEED_US_PER_SEC * UPDATE_MS) / 1000;
  if (step < 1) step = 1;

  if (goingOut) {
    current_us += step;
    if (current_us >= MAX_US) {
      current_us = MAX_US;
      goingOut = false;
    }
  } else {
    current_us -= step;
    if (current_us <= MIN_US) {
      current_us = MIN_US;
      goingOut = true;
    }
  }

  actuator.writeMicroseconds(current_us);
}
