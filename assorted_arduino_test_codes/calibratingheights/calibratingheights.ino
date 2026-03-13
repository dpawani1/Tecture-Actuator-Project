#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const uint8_t N = 9;
const uint8_t CH[N] = {0,1,2,3,4,5,6,7,8};

const int MIN_US = 1000;
const int MAX_US = 1940;

int current_us = MIN_US;

uint16_t usToCounts(int us) {
  long counts = (long)us * 4096L / 20000L;
  if (counts < 0) counts = 0;
  if (counts > 4095) counts = 4095;
  return (uint16_t)counts;
}

void moveAll(int us) {
  for (uint8_t i = 0; i < N; i++) {
    pwm.setPWM(CH[i], 0, usToCounts(us));
  }
}

void setup() {
  Serial.begin(9600);

  pwm.begin();
  pwm.setPWMFreq(50);

  moveAll(current_us);

  Serial.println("Type a height (microseconds) and press Send.");
  Serial.print("Valid range: ");
  Serial.print(MIN_US);
  Serial.print(" to ");
  Serial.println(MAX_US);
  Serial.print("Current height: ");
  Serial.println(current_us);
}

void loop() {
  if (!Serial.available()) return;

  int us = Serial.parseInt();
  while (Serial.available()) Serial.read();

  if (us == 0) return;

  if (us < MIN_US) us = MIN_US;
  if (us > MAX_US) us = MAX_US;

  current_us = us;
  moveAll(current_us);

  Serial.print("Height: ");
  Serial.println(current_us);
}