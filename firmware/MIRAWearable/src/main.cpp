#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#define MOTOR_PIN 9
// State
// LOST = 0
// GUIDE = 1
// CLOSE = 2
// REACHED = 3
// 
typedef struct __attribute__((packed)) {
  float magnitude;   
  int8_t state;
} FeedbackPacketSmall;

// Latest received packet (copied from callback)
volatile bool fbReady = false;
FeedbackPacketSmall fbLatest;

// ---------- MOTOR CONTROL ----------
static inline void motorWrite(int strength) {
  strength = constrain(strength, 0, 255);
  analogWrite(MOTOR_PIN, strength);
}

// ---------- HAPTIC PATTERN STATE MACHINE ----------
enum HapticMode { MODE_GUIDANCE, MODE_PATTERN };
HapticMode hMode = MODE_GUIDANCE;

struct Step {
  uint16_t onMs;
  uint16_t offMs;
  uint8_t  strength;
};

enum PatternType { PAT_NONE, PAT_TARGET };

// On-target heartbeat: double pulse
const Step TARGET_PATTERN[] = {
  {45, 120, 210},
  {45, 260, 210},
};


const int TARGET_LEN = sizeof(TARGET_PATTERN) / sizeof(TARGET_PATTERN[0]);


PatternType activePattern = PAT_NONE;
int patternIdx = 0;
bool stepIsOn = false;
uint32_t stepDeadlineMs = 0;

uint32_t lastTargetMs = 0;
const uint32_t TARGET_COOLDOWN_MS = 900;  // leave a clear gap between heartbeat cycles



void startPattern(PatternType p, uint32_t nowMs) {
  if (p != PAT_TARGET) return;
  lastTargetMs = nowMs;

  hMode = MODE_PATTERN;
  activePattern = p;
  patternIdx = 0;
  stepIsOn = true;

  motorWrite(TARGET_PATTERN[0].strength);
  stepDeadlineMs = nowMs + TARGET_PATTERN[0].onMs;
}

void servicePattern(uint32_t nowMs) {
  if (hMode != MODE_PATTERN) return;
  if ((int32_t)(nowMs - stepDeadlineMs) < 0) return;

  if (activePattern != PAT_TARGET) {
    hMode = MODE_GUIDANCE;
    motorWrite(0);
    return;
  }

  const Step &s = TARGET_PATTERN[patternIdx];

  if (stepIsOn) {
    // OFF phase
    motorWrite(0);
    stepIsOn = false;
    stepDeadlineMs = nowMs + s.offMs;
  } else {
    // NEXT step
    patternIdx++;
    if (patternIdx >= TARGET_LEN) {
      motorWrite(0);
      hMode = MODE_GUIDANCE;
      activePattern = PAT_NONE;
      return;
    }
    const Step &n = TARGET_PATTERN[patternIdx];
    motorWrite(n.strength);
    stepIsOn = true;
    stepDeadlineMs = nowMs + n.onMs;
  }
}

// ---------- ESP-NOW CALLBACK ----------
void onDataReceived(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len < (int)sizeof(FeedbackPacketSmall)) return;
  memcpy((void*)&fbLatest, incomingData, sizeof(FeedbackPacketSmall));
  fbReady = true; // keep callback fast
}

// ---------- MAIN LOGIC ----------
void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_PIN, OUTPUT);
  motorWrite(0);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW failed");
    return;
  }
  esp_now_register_recv_cb(onDataReceived);
}

void loop() {
  const uint32_t nowMs = millis();

  // Always service pattern (non-blocking)
  servicePattern(nowMs);

  // ----- THRESHOLDS (tune) -----
  // distance is in "tiles" because your dx/dy are cell deltas
  const float TARGET_THRESH = 0.05f;   // basically "at target"

  // Pull latest packet safely
  FeedbackPacketSmall fb;
  bool got = false;
  if (fbReady) {
    noInterrupts();
    fb = fbLatest;
    fbReady = false;
    interrupts();
    got = true;
  }

  if (got) {
    // Debug 
    // Serial.print("Mag: "); Serial.print(fb.magnitude);
    // Serial.print("  dx: "); Serial.print(fb.dx);
    // Serial.print("  dy: "); Serial.print(fb.dy);
    // Serial.println();

    // negative magnitude: no tile
    if (fb.magnitude < 0.0f) {
      if (hMode == MODE_GUIDANCE) motorWrite(0);
      return;
    }

    // 1) TARGET pattern (highest priority)
    if (fb.magnitude <= TARGET_THRESH &&
        (nowMs - lastTargetMs) > TARGET_COOLDOWN_MS) {
      startPattern(PAT_TARGET, nowMs);
      Serial.println("Target reached!");
      return;
    }

    // 2) GUIDANCE vibration (continuous when not playing target pattern)
    if (hMode == MODE_GUIDANCE) {
      const float maxMag = 10.0f;
      float m = constrain(fb.magnitude, 0.0f, maxMag);
      float t = 1.0f - (m / maxMag);
      int strength = (int)(t * 255.0f);
      motorWrite(strength);
    }
  }

  // No delays. loop spins fast.
}