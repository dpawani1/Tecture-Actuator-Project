#include <ServoEasing.hpp>   // NOTE: .hpp for this library

#define SERVO_PIN 9

// your endpoints
#define MIN_US 1090
#define MAX_US 1940

ServoEasing actuator;  // create object

bool running = false;

// Map microseconds to degrees for ServoEasing (0..180)
int usToDeg(int us) {
  long deg = (long)(us - MIN_US) * 180L / (MAX_US - MIN_US);
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  return (int)deg;
}

void setup() {
  Serial.begin(9600);
  Serial.println("SPACE = start/stop smooth loop");

  // attach servo
  actuator.attach(SERVO_PIN);

  // easing curve (try EASE_SINE_IN_OUT too)
  actuator.setEasingType(EASE_CUBIC_IN_OUT);

  // speed in degrees/second (lower = slower/quieter)
  actuator.setSpeed(15);

  // start at retracted
  actuator.write(usToDeg(MIN_US));
  delay(300);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == ' ') {
      running = !running;
      Serial.println(running ? "Running" : "Stopped");
    }
  }

  if (running) {
    actuator.startEaseTo(usToDeg(MAX_US));
    while (actuator.isMoving()) {
      actuator.update();
    }
    delay(400);

    actuator.startEaseTo(usToDeg(MIN_US));
    while (actuator.isMoving()) {
      actuator.update();
    }
    delay(400);
  }
}
