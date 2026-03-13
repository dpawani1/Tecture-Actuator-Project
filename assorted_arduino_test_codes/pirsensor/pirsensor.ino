#define PIR_PIN 2

bool lastState = LOW;

void setup() {
  Serial.begin(9600);
  pinMode(PIR_PIN, INPUT);

  Serial.println("PIR warming up...");
  delay(30000);   // PIR needs ~30s to stabilize
  Serial.println("PIR ready");
}

void loop() {
  bool currentState = digitalRead(PIR_PIN);

  // Only print when the state changes
  if (currentState != lastState) {
    if (currentState == HIGH) {
      Serial.println("detected motion");
    } else {
      Serial.println("not detecting motion");
    }
    lastState = currentState;
  }
}
