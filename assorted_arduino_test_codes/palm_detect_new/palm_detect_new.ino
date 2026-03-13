#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const uint8_t N = 9;
const uint8_t CH[N] = {0,1,2,3,4,5,6,7,8};

const int MIN_US = 1150;
const int MAX_US = 1900;

const int SPEED_US_PER_SEC = 140;
const int UPDATE_MS = 10;

int current_us[N];
int target_us[N];

unsigned long lastUpdate = 0;

// -------- SAFETY SETTINGS ----------
const unsigned long STARTUP_DELAY_MS = 10000;   // 10 sec delay before accepting commands
// -----------------------------------
unsigned long bootMs = 0;

uint16_t usToCounts(int us) {
  long counts = (long)us * 4096L / 20000L; // 20ms period @ 50Hz
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
    }
    else if (current_us[i] > target_us[i]) {
      current_us[i] -= step;
      if (current_us[i] < target_us[i]) current_us[i] = target_us[i];
      writeChannel(CH[i], current_us[i]);
    }
  }
  return true;
}

/*
  Mapping from GRID bit index (0..8) to ACTUATOR index (0..8).
  If your physical layout doesn't match the PCA9685 channel order,
  change this mapping ONLY (not wiring).
*/
const uint8_t GRID_TO_ACT[N] = {
  0, 1, 2,
  3, 4, 5,
  6, 7, 8
};

// bit g = 1 => actuator GRID_TO_ACT[g] goes to MAX_US
// others go to MIN_US
void applyMaskToTargets(uint16_t mask) {
  for (uint8_t a = 0; a < N; a++) {
    target_us[a] = MIN_US;
  }

  for (uint8_t g = 0; g < N; g++) {
    if (mask & (1 << g)) {
      uint8_t a = GRID_TO_ACT[g];
      if (a < N) target_us[a] = MAX_US;
    }
  }
}

/*
  FIXED: Always treat incoming number as a MASK (0..511).
  This avoids the ambiguity that caused:
    4 (top-right bit) being misread as index 4 => (1<<4)=16 (center)
*/
uint16_t decodeIncomingToMask(long v) {
  if (v < 0) v = 0;
  if (v > 511) v = 511;
  return (uint16_t)v;
}

void setup() {
  Serial.begin(9600);   // KEEP AT 9600

  pwm.begin();
  pwm.setPWMFreq(50);

  // Immediately home everything
  for (uint8_t i = 0; i < N; i++) {
    current_us[i] = MIN_US;
    target_us[i]  = MIN_US;
    writeChannel(CH[i], current_us[i]);
  }

  bootMs = millis();

  Serial.println("READY (HOMED)");
  Serial.println("Protocol: send MASK 0..511 (e.g., 4=bit2, 16=bit4, 0=all home)");
  Serial.println("Waiting 10 seconds...");
}

void loop() {
  unsigned long now = millis();

  // Hold home for first 10 seconds
  if (now - bootMs < STARTUP_DELAY_MS) {
    applyMaskToTargets(0);
    stepAllTowardTargets();
    return;
  }

  // Read serial mask
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      long v = line.toInt();
      uint16_t mask = decodeIncomingToMask(v);

      // Debug print so you can see exactly what Arduino received
      Serial.print("RX=");
      Serial.print(v);
      Serial.print("  MASK=");
      Serial.println(mask);

      applyMaskToTargets(mask);
    }
  }

  // Smooth motion always active
  stepAllTowardTargets();
}