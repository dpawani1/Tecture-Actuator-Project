#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const uint8_t N = 9;
const uint8_t CH[N] = {0,1,2,3,4,5,6,7,8};

const int MIN_US = 1390;
const int MAX_US = 1940;

const int SPEED_US_PER_SEC = 130;
const int UPDATE_MS = 15;

int current_us[N];
int target_us[N];

unsigned long lastUpdate = 0;

uint16_t usToCounts(int us) {
  long counts = (long)us * 4096L / 20000L; // 20ms at 50Hz
  if (counts < 0) counts = 0;
  if (counts > 4095) counts = 4095;
  return (uint16_t)counts;
}

void writeChannel(uint8_t ch, int us) {
  pwm.setPWM(ch, 0, usToCounts(us));
}

bool stepAllTowardTargets() {
  unsigned long now = millis();
  if (now - lastUpdate < (unsigned long)UPDATE_MS) return false;
  lastUpdate = now;

  int step = (SPEED_US_PER_SEC * UPDATE_MS) / 1000;
  if (step < 1) step = 1;

  for (uint8_t i = 0; i < N; i++) {
    if (current_us[i] < target_us[i]) {
      current_us[i] += step;
      if (current_us[i] > target_us[i]) current_us[i] = target_us[i];
      writeChannel(CH[i], current_us[i]);
    } else if (current_us[i] > target_us[i]) {
      current_us[i] -= step;
      if (current_us[i] < target_us[i]) current_us[i] = target_us[i];
      writeChannel(CH[i], current_us[i]);
    }
  }
  return true;
}

// Apply mask bits to targets:
// bit i = 1 => MAX_US, bit i = 0 => MIN_US
void applyMaskToTargets(uint16_t mask) {
  for (uint8_t i = 0; i < N; i++) {
    if (mask & (1 << i)) target_us[i] = MAX_US;
    else                target_us[i] = MIN_US;
  }
}

void setup() {
  Serial.begin(9600);

  pwm.begin();
  pwm.setPWMFreq(50);

  for (uint8_t i = 0; i < N; i++) {
    current_us[i] = MIN_US;
    target_us[i]  = MIN_US;
    writeChannel(CH[i], current_us[i]);
  }

  Serial.println("READY");
}

void loop() {
  // Expect a line like: 0..511 then '\n'
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      long m = line.toInt(); // 0..511
      if (m < 0) m = 0;
      if (m > 511) m = 511;
      applyMaskToTargets((uint16_t)m);
    }
  }

  // Always ease toward latest targets
  stepAllTowardTargets();
}