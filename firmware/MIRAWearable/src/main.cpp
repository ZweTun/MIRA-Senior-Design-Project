#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#define MOTOR_PIN 9   // vibration motor pin

// typedef struct __attribute__((packed)){
//   float magnitude;   // 0.0  1.0 distance
//   int8_t dx;         // -1, 0, 1
//   int8_t dy;         // -1, 0, 1
// } FeedbackPacket;
// FeedbackPacket fb;


// // ---------------- HAPTIC HELPERS ----------------
// void vibrate(int duration, int strength) {
//   strength = constrain(strength, 0, 255);
//   analogWrite(MOTOR_PIN, strength);
//   delay(duration);
//   analogWrite(MOTOR_PIN, 0);
// }



// void hapticTargetReached() {
//   vibrate(200, 255);
//   delay(100);
//   vibrate(200, 255);
//   Serial.println("Target reached!");
// }

// // ---------------- ESP-NOW CALLBACK ----------------
// void onDataReceived(const uint8_t * mac, const uint8_t *incomingData, int len) {
//   memcpy(&fb, incomingData, sizeof(fb));

//   Serial.print("dx: "); Serial.print(fb.dx);
//   Serial.print(" dy: "); Serial.print(fb.dy);
//   Serial.print(" mag: "); Serial.println(fb.magnitude);

//   if (fb.magnitude < 0.0) {
//     vibrate(200, 255);
//     delay(100);
//     vibrate(200, 255);
//     Serial.println("No tile detected, ignoring.");
//     return;
//   }
 
//   // If magnitude is within threshold, consider target reached
//   // if (fb.magnitude < 1.0) {
//   //   hapticTargetReached();
//   //   return;
//   // }

//   //Flipped mapping: stronger vibration for smaller magnitude (closer to target)
//   int strength = map(fb.magnitude, 1.0, 0.0, 0, 255);
//   strength = constrain(strength, 0, 255);
//   analogWrite(MOTOR_PIN, strength);


// }


// void setup() {
//   Serial.begin(115200);
//   pinMode(MOTOR_PIN, OUTPUT);

//   WiFi.mode(WIFI_STA);
//   if (esp_now_init() != ESP_OK) {
//     Serial.println("ESP-NOW failed");
//     return;
//   }

//   esp_now_register_recv_cb(onDataReceived);
// }

// void loop() {}


void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_PIN, OUTPUT);


}

void loop() {
  analogWrite(MOTOR_PIN, 255);
  Serial.println("Vibrating at full strength...");
}



