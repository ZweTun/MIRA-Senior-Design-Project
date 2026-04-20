// #include <Arduino.h>
// #include <esp_now.h>
// #include <WiFi.h>

// #define MOTOR_PIN 9
// // State
// // LOST = 0
// // GUIDE = 1
// // CLOSE = 2
// // REACHED = 3
// // 
// typedef struct __attribute__((packed)) {
//   float magnitude;   
//   int8_t state;
// } FeedbackPacketSmall;

// // Latest received packet (copied from callback)
// volatile bool fbReady = false;
// FeedbackPacketSmall fbLatest;

// // ---------- MOTOR CONTROL ----------
// static inline void motorWrite(int strength) {
//   strength = constrain(strength, 0, 255);
//   analogWrite(MOTOR_PIN, strength);
// }

// // ---------- HAPTIC PATTERN STATE MACHINE ----------
// enum HapticMode { MODE_GUIDANCE, MODE_PATTERN };
// HapticMode hMode = MODE_GUIDANCE;

// struct Step {
//   uint16_t onMs;
//   uint16_t offMs;
//   uint8_t  strength;
// };

// enum PatternType { PAT_NONE, PAT_TARGET };

// // On-target heartbeat: double pulse
// const Step TARGET_PATTERN[] = {
//   {45, 120, 210},
//   {45, 260, 210},
// };



// const int TARGET_LEN = sizeof(TARGET_PATTERN) / sizeof(TARGET_PATTERN[0]);


// PatternType activePattern = PAT_NONE;
// int patternIdx = 0;
// bool stepIsOn = false;
// uint32_t stepDeadlineMs = 0;

// uint32_t lastTargetMs = 0;
// const uint32_t TARGET_COOLDOWN_MS = 900;  // leave a clear gap between heartbeat cycles



// void startPattern(PatternType p, uint32_t nowMs) {
//   if (p != PAT_TARGET) return;
//   lastTargetMs = nowMs;

//   hMode = MODE_PATTERN;
//   activePattern = p;
//   patternIdx = 0;
//   stepIsOn = true;

//   motorWrite(TARGET_PATTERN[0].strength);
//   stepDeadlineMs = nowMs + TARGET_PATTERN[0].onMs;
// }

// void servicePattern(uint32_t nowMs) {
//   if (hMode != MODE_PATTERN) return;
//   if ((int32_t)(nowMs - stepDeadlineMs) < 0) return;

//   if (activePattern != PAT_TARGET) {
//     hMode = MODE_GUIDANCE;
//     motorWrite(0);
//     return;
//   }

//   const Step &s = TARGET_PATTERN[patternIdx];

//   if (stepIsOn) {
//     // OFF phase
//     motorWrite(0);
//     stepIsOn = false;
//     stepDeadlineMs = nowMs + s.offMs;
//   } else {
//     // NEXT step
//     patternIdx++;
//     if (patternIdx >= TARGET_LEN) {
//       motorWrite(0);
//       hMode = MODE_GUIDANCE;
//       activePattern = PAT_NONE;
//       return;
//     }
//     const Step &n = TARGET_PATTERN[patternIdx];
//     motorWrite(n.strength);
//     stepIsOn = true;
//     stepDeadlineMs = nowMs + n.onMs;
//   }
// }

// // ---------- ESP-NOW CALLBACK ----------
// void onDataReceived(const uint8_t *mac, const uint8_t *incomingData, int len) {
//   if (len < (int)sizeof(FeedbackPacketSmall)) return;
//   memcpy((void*)&fbLatest, incomingData, sizeof(FeedbackPacketSmall));
//   fbReady = true; // keep callback fast
// }

// // ---------- MAIN LOGIC ----------
// void setup() {
//   Serial.begin(115200);
//   pinMode(MOTOR_PIN, OUTPUT);
//   motorWrite(0);

//   WiFi.mode(WIFI_STA);
//   if (esp_now_init() != ESP_OK) {
//     Serial.println("ESP-NOW failed");
//     return;
//   }
//   esp_now_register_recv_cb(onDataReceived);
// }

// void loop() {
//   const uint32_t nowMs = millis();

//   // Always service pattern (non-blocking)
//   servicePattern(nowMs);

//   // ----- THRESHOLDS (tune) -----
//   // distance is in "tiles" because your dx/dy are cell deltas
//   const float TARGET_THRESH = 0.05f;   // basically "at target"

//   // Pull latest packet safely
//   FeedbackPacketSmall fb;
//   bool got = false;
//   if (fbReady) {
//     noInterrupts();
//     fb = fbLatest;
//     fbReady = false;
//     interrupts();
//     got = true;
//   }

//   if (got) {
//     // Debug 
//     // Serial.print("Mag: "); Serial.print(fb.magnitude);
//     // Serial.print("  dx: "); Serial.print(fb.dx);
//     // Serial.print("  dy: "); Serial.print(fb.dy);
//     // Serial.println();

//     // negative magnitude: no tile
//     if (fb.magnitude < 0.0f) {
//       if (hMode == MODE_GUIDANCE) motorWrite(0);
//       return;
//     }

//     // 1) TARGET pattern (highest priority)
//     if (fb.magnitude <= TARGET_THRESH &&
//         (nowMs - lastTargetMs) > TARGET_COOLDOWN_MS) {
//       startPattern(PAT_TARGET, nowMs);
//       Serial.println("Target reached!");
//       return;
//     }

//     // 2) GUIDANCE vibration (continuous when not playing target pattern)
//     if (hMode == MODE_GUIDANCE) {
//       const float maxMag = 10.0f;
//       float m = constrain(fb.magnitude, 0.0f, maxMag);
//       float t = 1.0f - (m / maxMag);
//       int strength = (int)(t * 255.0f);
//       motorWrite(strength);
//     }
//   }

//   // No delays. loop spins fast.
// }








#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#define MOTOR_PIN 9
// State
// LOST = 0
// GUIDE = 1
// CLOSE = 2
// REACHED = 3   
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

enum PatternType { PAT_NONE, PAT_CLOSE, PAT_TARGET };

// Close pattern: “tick…tick” (slow heartbeat feel)
const Step CLOSE_PATTERN[] = {
  {45, 120, 210},
  {45, 260, 210},
};
const int CLOSE_LEN = sizeof(CLOSE_PATTERN) / sizeof(CLOSE_PATTERN[0]);

// “da-da-DAAA” success jingle
const Step TARGET_PATTERN[] = {
  {70,  80, 180},
  {70,  80, 220},
  {220, 50, 255},
};
const int TARGET_LEN = sizeof(TARGET_PATTERN) / sizeof(TARGET_PATTERN[0]);


PatternType activePattern = PAT_NONE;
int patternIdx = 0;
bool stepIsOn = false;
uint32_t stepDeadlineMs = 0;

uint32_t lastCloseMs  = 0;
uint32_t lastTargetMs = 0;
const uint32_t CLOSE_COOLDOWN_MS  = 500;   // can repeat occasionally
const uint32_t TARGET_COOLDOWN_MS = 1200;  // don’t spam



void startPattern(PatternType p, uint32_t nowMs) {
  // Target overrides close, close does NOT override target
  if (activePattern == PAT_TARGET && p != PAT_TARGET) return;

  const Step* steps = nullptr;
  int len = 0;

  if (p == PAT_CLOSE)  { steps = CLOSE_PATTERN;  len = CLOSE_LEN;  lastCloseMs = nowMs; }
  if (p == PAT_TARGET) { steps = TARGET_PATTERN; len = TARGET_LEN; lastTargetMs = nowMs; }

  if (!steps || len == 0) return;

  hMode = MODE_PATTERN;
  activePattern = p;
  patternIdx = 0;
  stepIsOn = true;

  motorWrite(steps[0].strength);
  stepDeadlineMs = nowMs + steps[0].onMs;
}

void servicePattern(uint32_t nowMs) {
  if (hMode != MODE_PATTERN) return;
  if ((int32_t)(nowMs - stepDeadlineMs) < 0) return;

  const Step* steps = (activePattern == PAT_TARGET) ? TARGET_PATTERN : CLOSE_PATTERN;
  int len = (activePattern == PAT_TARGET) ? TARGET_LEN : CLOSE_LEN;

  const Step &s = steps[patternIdx];

  if (stepIsOn) {
    // OFF phase
    motorWrite(0);
    stepIsOn = false;
    stepDeadlineMs = nowMs + s.offMs;
  } else {
    // NEXT step
    patternIdx++;
    if (patternIdx >= len) {
      motorWrite(0);
      hMode = MODE_GUIDANCE;
      activePattern = PAT_NONE;
      return;
    }
    const Step &n = steps[patternIdx];
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
  // Motor test
  // motorWrite(0);
  // delay(500);
  // Always service pattern (non-blocking)
  servicePattern(nowMs);

  // ----- THRESHOLDS (tune) -----
  // distance is in "tiles" because your dx/dy are cell deltas
  const float TARGET_THRESH = 0.05f;   // basically "at target"
  const float CLOSE_ENTER   = 1.42f;   // include all 8 neighbors around target (sqrt(2) tiles)
  const float CLOSE_EXIT    = 1.77f;   // hysteresis so it doesn't chatter

  static bool inCloseZone = false;

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
      inCloseZone = false;
      return;
    }

    // 1) TARGET pattern (highest priority)
    if (fb.magnitude <= TARGET_THRESH &&
        hMode != MODE_PATTERN && 
        (nowMs - lastTargetMs) > CLOSE_COOLDOWN_MS) {
      startPattern(PAT_CLOSE, nowMs);
      Serial.println("Target reached!");
      inCloseZone = false;
      return;
    }

    // 2) CLOSE pattern (when within 1 tile but not at target)
    // if (!inCloseZone && fb.magnitude <= CLOSE_ENTER) inCloseZone = true;
    // if ( inCloseZone && fb.magnitude >= CLOSE_EXIT)  inCloseZone = false;

    // if (inCloseZone &&
    //     hMode != MODE_PATTERN &&                         // don’t interrupt active pattern
    //     (nowMs - lastCloseMs) > CLOSE_COOLDOWN_MS) {
    //   startPattern(PAT_CLOSE, nowMs);
    //   // don't return; guidance will resume after the close pattern ends
    // }

    // 3) GUIDANCE vibration (only when not playing a pattern)
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