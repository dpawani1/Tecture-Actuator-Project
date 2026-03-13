#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define PIR_PIN 2

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const uint8_t N = 9;
const uint8_t CH[N] = {0,1,2,3,4,5,6,7,8};

const int MIN_US = 1420;
const int MAX_US = 1940;

const int SPEED_US_PER_SEC = 300;
const int UPDATE_MS = 5;

const unsigned long HOLD_AT_TOP_MS = 500;
const unsigned long HOLD_AT_BOT_MS = 200;

const unsigned long COOLDOWN_MS = 5000;

enum Phase {
  IDLE,
  ALL_EXTEND,
  ALL_HOLD_TOP,
  ALL_RETRACT,
  ALL_HOLD_BOT,
  SEQ_EXTEND,
  SEQ_HOLD_TOP,
  SEQ_RETRACT,
  SEQ_HOLD_BOT
};

Phase phase = IDLE;

int current_us[N];
int target_us[N];

uint8_t seqIndex = 0;

unsigned long lastUpdate = 0;
unsigned long phaseStart = 0;

bool wasHigh = false;
unsigned long lastTriggerTime = 0;

uint16_t usToCounts(int us) {
  long counts = (long)us * 4096L / 20000L;
  if (counts < 0) counts = 0;
  if (counts > 4095) counts = 4095;
  return (uint16_t)counts;
}

void writeChannel(uint8_t ch, int us) {
  pwm.setPWM(ch, 0, usToCounts(us));
}

void setAllTargets(int us) {
  for (uint8_t i = 0; i < N; i++) target_us[i] = us;
}

void setOnlyOneTarget(uint8_t idx, int idxTarget) {
  for (uint8_t i = 0; i < N; i++) {
    target_us[i] = (i == idx) ? idxTarget : MIN_US;
  }
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

  for (uint8_t i = 0; i < N; i++) {
    if (current_us[i] != target_us[i]) return false;
  }
  return true;
}

void enterPhase(Phase p) {
  phase = p;
  phaseStart = millis();
}

void resetToMin() {
  for (uint8_t i = 0; i < N; i++) {
    current_us[i] = MIN_US;
    target_us[i]  = MIN_US;
    writeChannel(CH[i], current_us[i]);
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(PIR_PIN, INPUT);

  pwm.begin();
  pwm.setPWMFreq(50);

  resetToMin();

  delay(30000);
}

void loop() {
  unsigned long now = millis();

  int pir = digitalRead(PIR_PIN);
  bool rising = (pir == HIGH && !wasHigh);
  wasHigh = (pir == HIGH);

  if (phase == IDLE && rising && (now - lastTriggerTime > COOLDOWN_MS)) {
    lastTriggerTime = now;
    Serial.println("detected motion");
    setAllTargets(MAX_US);
    enterPhase(ALL_EXTEND);
  }

  switch (phase) {
    case IDLE:
      break;

    case ALL_EXTEND:
      if (stepAllTowardTargets()) enterPhase(ALL_HOLD_TOP);
      break;

    case ALL_HOLD_TOP:
      if (now - phaseStart >= HOLD_AT_TOP_MS) {
        setAllTargets(MIN_US);
        enterPhase(ALL_RETRACT);
      }
      break;

    case ALL_RETRACT:
      if (stepAllTowardTargets()) enterPhase(ALL_HOLD_BOT);
      break;

    case ALL_HOLD_BOT:
      if (now - phaseStart >= HOLD_AT_BOT_MS) {
        seqIndex = 0;
        setOnlyOneTarget(seqIndex, MAX_US);
        enterPhase(SEQ_EXTEND);
      }
      break;

    case SEQ_EXTEND:
      if (stepAllTowardTargets()) enterPhase(SEQ_HOLD_TOP);
      break;

    case SEQ_HOLD_TOP:
      if (now - phaseStart >= HOLD_AT_TOP_MS) {
        setOnlyOneTarget(seqIndex, MIN_US);
        enterPhase(SEQ_RETRACT);
      }
      break;

    case SEQ_RETRACT:
      if (stepAllTowardTargets()) enterPhase(SEQ_HOLD_BOT);
      break;

    case SEQ_HOLD_BOT:
      if (now - phaseStart >= HOLD_AT_BOT_MS) {
        seqIndex++;
        if (seqIndex >= N) {
          enterPhase(IDLE);
        } else {
          setOnlyOneTarget(seqIndex, MAX_US);
          enterPhase(SEQ_EXTEND);
        }
      }
      break;
  }
}
