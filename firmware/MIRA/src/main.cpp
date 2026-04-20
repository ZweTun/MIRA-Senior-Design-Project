// Ensure Arduino core is included first so `Serial` is declared
#include <Arduino.h>



#include <vector>
#include <map>


#include <esp_now.h>
#include <esp_err.h>
#include <WiFi.h>

#include "rfid.h"
#include "mosiac.h"
#include <set>


// Button map 
const int NUM_ROWS = 6;
const int NUM_COLS = 6;

// Put all ROWS on one side of the dev board
const int rowPins[NUM_ROWS] = {13, 14, 27, 26, 25, 33};

// Put all COLUMNS on the other side of the dev board
const int colPins[NUM_COLS] = {23, 22, 21, 19, 18, 32};

//NOTE: on physical board, wire the rows to the assigned column pins, wire the columns to assigned row pins... will fix later, dont fix rn

// Store previous button states for simple debounce / change detection
bool stableState[NUM_ROWS][NUM_COLS];
bool lastReading[NUM_ROWS][NUM_COLS];
unsigned long lastDebounceTime[NUM_ROWS][NUM_COLS];

const unsigned long debounceDelay = 30;   // ms
const unsigned long printInterval = 250;  // ms
unsigned long lastPrintTime = 0;




// ESP now receiver's MAC
uint8_t receiverMAC[] = {0x64, 0xE8, 0x33, 0x51, 0xCD, 0xFC};
// 64:E8:33:51:CD:FC

// State
// STOP = -1 
// LOST = 0
// GUIDE = 1
// CLOSE = 2
// REACHED = 3
typedef struct {
  float magnitude;
  int16_t dx;
  int16_t dy;
  int8_t state;
  CellPos currentPos;
} FeedbackPacket;

typedef struct {
  float magnitude;
  int8_t state;
} FeedbackPacketSmall;

FeedbackPacket currFeedback;


//INIT PARAMETERS 
int const offset = 200; 
int const gridRows = 8; 
int const gridCols = 8;
float const epsilon = 0.0; 
int const buzzer = 9;
int const threshold = 100;

unsigned long lastPacketMs = 0;
bool hasFeedback = false;
const unsigned long FEEDBACK_TIMEOUT_MS = 300;
bool newFeedback = false;
bool hasFirstModelUpdate = false;

volatile bool espNowSendInFlight = false;
unsigned long nextSendAllowedMs = 0;
const unsigned long MIN_SEND_INTERVAL_MS = 25;
const unsigned long SEND_ERROR_BACKOFF_MS = 100;

const int8_t STOP_STATE = -1;
const int8_t LOST_STATE = 0;

void onEspNowSend(const uint8_t *mac_addr, esp_now_send_status_t status) {
  (void)mac_addr;
  espNowSendInFlight = false;
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("ESP-NOW delivery failed at MAC layer");
  }
}






// April Tag localization 

// char buffer[64]; // Buffer for serial input
// void debug_tag_locations() {
//   if (Serial.available()) {
//     int len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
//     buffer[len] = '\0';  // terminate string

//     Serial.print("RX: ");
//     Serial.println(buffer);
//   }

// }





void giveFeedback(float distance, int state) {


  
  // Print 
  // Serial.print("Distance to target: ");
  // Serial.println(distance);

  FeedbackPacketSmall packet;
  packet.magnitude = distance;
  packet.state = state;

  unsigned long now = millis();
  if (espNowSendInFlight || now < nextSendAllowedMs) {
    return;
  }

  espNowSendInFlight = true;
  nextSendAllowedMs = now + MIN_SEND_INTERVAL_MS;

  esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&packet, sizeof(packet));

  if (result != ESP_OK) {
    espNowSendInFlight = false;
    nextSendAllowedMs = now + SEND_ERROR_BACKOFF_MS;
    Serial.print("ESP-NOW send failed: ");
    Serial.print(esp_err_to_name(result));
    Serial.print(" (");
    Serial.print(result);
    Serial.println(")");
  }
}

void sendStopCommand() {
  giveFeedback(-1.0f, STOP_STATE);
}



// Target guidance loop 
void guideToTarget(const FeedbackPacket& feedbackPacket) {

    //Find current cell user is hovering over 
    //SenseCell* current = localize();
    //display(targets);
    
    if (feedbackPacket.state == STOP_STATE) {
      sendStopCommand();
      return;
    }

    if (feedbackPacket.currentPos.row < 0 || feedbackPacket.currentPos.row >= gridRows || feedbackPacket.currentPos.col < 0 || feedbackPacket.currentPos.col >= gridCols) {
      // Send empty feedback or some signal that no tile movement is detected
      giveFeedback(-1.0f, LOST_STATE); // Indicate no tile detected
      return;
    }
  


    
    //display();


    // Serial.print("Position: (");
    // Serial.print(feedbackPacket.currentPos.row);
    // Serial.print(", ");
    // Serial.print(feedbackPacket.currentPos.col);
    // Serial.print(") ");
    // Hardcoded for now, will replace with actual distance calculation later
    giveFeedback(feedbackPacket.magnitude, feedbackPacket.state);


   //delay(300);
  
}








// Read or write mode
bool readMode = true; // Set to false for write mode

// HSPI wiring 
static constexpr uint8_t RFID_SS   = 5;
static constexpr uint8_t RFID_RST  = 27;
static constexpr uint8_t RFID_SCK  = 14;
static constexpr uint8_t RFID_MISO = 12;
static constexpr uint8_t RFID_MOSI = 13;

static uint32_t lastUid = 0;
static uint32_t lastActionMs = 0;




RFID rfid(RFID_SS, RFID_RST, RFID_SCK, RFID_MISO, RFID_MOSI);

void initializeRFID() {
  if (!rfid.begin()) {
    Serial.println("RFID init FAILED. Check 3.3V, GND, and SPI wiring.");
    while (true) { delay(1000); }
  }
  Serial.println("RFID init OK. Tap a tag...");
}



void setup() {
  Serial.begin(115200);
  delay(200);
  


  // Initialize WiFi in station mode for ESP-NOW
  WiFi.mode(WIFI_STA);   // ESP-NOW works in station mode
  WiFi.setSleep(false);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  if (esp_now_register_send_cb(onEspNowSend) != ESP_OK) {
    Serial.println("Failed to register ESP-NOW send callback");
    return;
  }

  // Register peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Safety: command stop before the model starts streaming updates.
  sendStopCommand();

  // Intitalize button map 
   for (int r = 0; r < NUM_ROWS; r++) {
    pinMode(rowPins[r], OUTPUT);
    digitalWrite(rowPins[r], HIGH); // inactive row = HIGH
  }

  for (int c = 0; c < NUM_COLS; c++) {
    pinMode(colPins[c], INPUT_PULLUP);
  }

  for (int r = 0; r < NUM_ROWS; r++) {
    for (int c = 0; c < NUM_COLS; c++) {
      stableState[r][c] = false;
      lastReading[r][c] = false;
      lastDebounceTime[r][c] = 0;
    }
  }


  // Initialize RFID reader
  //initializeRFID();
  // Setup mux
  // Setup 4 pins that feed into hall effect sensors
  // Serial.println("Initializing sensors...");
  // for (int i = 0; i < gridRows; i++) {
  //   for (int j = 0; j < gridCols; j++) {
  //     //pinMode(pins[i][j], INPUT);
  //    senseMat[i][j] = SenseCell(i, j);
  //   }
  // }


  Serial.println("System initialized.\n");

}

// returns color id 
int handleRead() {
  BlockInfo info = rfid.readBlock();
  if (info.valid) {
    // Serial.print("Read block: UID32=");
    // Serial.print(info.uid32, HEX);
    // Serial.print(", Color=");
    // Serial.println(id_to_color[info.colorIndex]);
    return info.colorIndex;
  } else {
    //Serial.println("Failed to read block.");
    return -1;
  }
} 


// returns color string
String handleWrite() {
  int colorIndex = 1; // Example: write "Green" (index 1)
  bool success = rfid.writeBlock(colorIndex);
  if (success) {
    Serial.print("Wrote block with color: ");


  } else {
    Serial.println("Failed to write block.");
    return "";
  }
}


// Targets is a set 


// Update current target and current position
void update() {
  newFeedback = false;


  if (Serial.available()) {
    
    String line = Serial.readStringUntil('\n');
    line.trim();
    float magnitude = 0.0f;
    int dx = 0;
    int dy = 0;
    int state = 0;
    int currentX = -1;
    int currentY = -1;
    int parsed = sscanf(
      line.c_str(),
      "%f,%d,%d,%d,%d,%d",
      &magnitude,
      &dx,
      &dy,
      &state,
      &currentX,
      &currentY
    );


    if (parsed == 6) {
      // Send ACK back to Python
      Serial.print("ACK,");
      Serial.print(magnitude, 2); Serial.print(",");
      Serial.print(dx); Serial.print(",");
      Serial.print(dy); Serial.print(",");
      Serial.print(state); Serial.print(",");
      Serial.print(currentX); Serial.print(",");
      Serial.println(currentY);
      currFeedback.magnitude = magnitude;
      currFeedback.dx = dx;
      currFeedback.dy = dy;
      currFeedback.state = state;
      currFeedback.currentPos = CellPos(currentX, currentY);

      // Send packet back to TX to debug

    
      lastPacketMs = millis();
      hasFeedback = true;
      newFeedback = true;
      hasFirstModelUpdate = true;
    } else {
      Serial.print("Bad packet: ");
      Serial.println(line);
    }
  }
}



void scanMatrix() {
  for (int r = 0; r < NUM_ROWS; r++) {
    for (int i = 0; i < NUM_ROWS; i++) {
      digitalWrite(rowPins[i], HIGH);
    }

    digitalWrite(rowPins[r], LOW);
    delayMicroseconds(50);

    for (int c = 0; c < NUM_COLS; c++) {
      bool pressed = (digitalRead(colPins[c]) == LOW);

      if (pressed != lastReading[r][c]) {
        lastDebounceTime[r][c] = millis();
        lastReading[r][c] = pressed;
      }

      if ((millis() - lastDebounceTime[r][c]) > debounceDelay) {
        stableState[r][c] = pressed;
      }
    }
  }

  for (int i = 0; i < NUM_ROWS; i++) {
    digitalWrite(rowPins[i], HIGH);
  }
}

void printReadableMatrix() {
  Serial.println("Current switch grid:");
  Serial.println();

  Serial.print("      ");
  for (int c = 0; c < NUM_COLS; c++) {
    Serial.print("C");
    Serial.print(c + 1);
    Serial.print(" ");
  }
  Serial.println();

  for (int r = 0; r < NUM_ROWS; r++) {
    Serial.print("R");
    Serial.print(r + 1);
    Serial.print(" ->  ");

    for (int c = 0; c < NUM_COLS; c++) {
      Serial.print(stableState[r][c] ? " X " : " . ");
    }
    Serial.println();
  }
}

void printPressedSummary() {
  bool anyPressed = false;

  // Serial.println();
  // Serial.println("Pressed switches:");
  // Package format is list of CellPos 
 
  if (!anyPressed) {
    Serial.println("None");
  } else {
    Serial.print("PRESSED:");

    for (int r = 0; r < NUM_ROWS; r++) {
      for (int c = 0; c < NUM_COLS; c++) {
        if (stableState[r][c]) {
          Serial.print(r);
          Serial.print(",");
          Serial.print(c);
          Serial.print(";");

          anyPressed = true;
        }
      }
    }

    Serial.println();  
  }


}

void loop() {
  //RFIDRead r = rfid.poll();
  // if (!r.present) return;
  // debug_tag_locations();
  static bool timeoutSent = false;

  update();

  // Do not drive haptics/motor until the first valid model update arrives.
  if (!hasFirstModelUpdate) {
    Serial.println("Waiting for first model update...");
    return;
  }
  //giveFeedback(currFeedback.magnitude, 2);
  guideToTarget(currFeedback);

  // if (newFeedback) {
  //   guideToTarget(currFeedback);
  //   timeoutSent = false;
  // } else if (hasFeedback && millis() - lastPacketMs > FEEDBACK_TIMEOUT_MS) {
  //   if (!timeoutSent) {
  //     sendStopCommand();
  //     timeoutSent = true;
  //   }
  // }

  // scanMatrix();
  // if (millis() - lastPrintTime >= printInterval) {
  //   lastPrintTime = millis();
  //   // printReadableMatrix();
  //   printPressedSummary();
  // }

  // Recieved feedback packet and its fresh 
  // if (hasFeedback && millis() - lastPacketMs <= FEEDBACK_TIMEOUT_MS) {
  //   if (newFeedback) {
  //     guideToTarget(currFeedback);
  //     timeoutSent = false;
  //   }
  // } else {
  //   if (!timeoutSent) { 
  //     giveFeedback(-1.0f, 0);
  //     timeoutSent = true;
  //   }
  // }
    
  // uint32_t now = millis();
  // if (r.uid32 == lastUid && (now - lastActionMs) < 800) {
  //   return;
  // }
  // lastUid = r.uid32;
  // lastActionMs = now;
    



  // if (r.present && readMode) {
  //   // If it's the same tag still sitting there, don't keep re-reading
  //   int colorIndex = handleRead();
  //   //Serial.println("Color read: " + id_to_color[colorIndex]);
  //   if (colorIndex >= 0) {
  //     auto positions = mosiac.getTargetCells(colorIndex);
  //     currTarget = positions; // Update global target set
  //   }
  //   // if (!positions.empty()) {
    //   Serial.print("New target color: ");
    //   Serial.println(id_to_color[colorIndex]);
    //   Serial.println("Target cells:");
    //   for (const CellPos& pos : positions) {
    //     Serial.print("(");
    //     Serial.print(pos.posX);
    //     Serial.print(", ");
    //     Serial.print(pos.posY);
    //     Serial.println(")");
    //   }
    // } else {
    //    Serial.println("Read color has no target cells.");
    // }

  // } else if (r.present && !readMode) {
  //   String color = handleWrite();
  // //  Serial.println("Color written: " + color);
  // }










}

