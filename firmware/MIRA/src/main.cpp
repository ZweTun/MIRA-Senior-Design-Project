// Ensure Arduino core is included first so `Serial` is declared
#include <Arduino.h>





#include <vector>
#include <map>
#include <Arduino.h>

#include <esp_now.h>
#include <WiFi.h>


#include <Arduino.h>
#include "rfid.h"
#include "mosiac.h"

// ESP now receiver's MAC
uint8_t receiverMAC[] = {0x64, 0xE8, 0x33, 0x51, 0xCD, 0xFC};
// 64:E8:33:51:CD:FC

typedef struct __attribute__((packed)){
  float magnitude;   // 0.0 → 1.0 normalized distance
  int8_t dx;         // -1, 0, 1
  int8_t dy;         // -1, 0, 1
} FeedbackPacket;
using namespace std;

//INIT PARAMETERS 
int const offset = 200; 
int const gridRows = 4; 
int const gridCols = 4;
float const epsilon = 0.0; 
int const buzzer = 9;
int const threshold = 100;

// Color map dictionary 
String id_to_color[4] = {
  "Red", // 0
  "Green", // 1
  "Cyan", 
  "Magenta"
};

int color_to_id(String color) {
  if (color == "Red") return 0;
  else if (color == "Green") return 1;
  else if (color == "Cyan") return 2;
  else if (color == "Magenta") return 3;
  return 0; 
}


// Mux address pins

const int MUX_S0 = 25; // LSB
const int MUX_S1 = 26;
const int MUX_S2 = 33;
const int MUX_S3 = 15;   // MSB


// // Mux common signal -> ESP32 ADC pin
const int MUX_SIG_PIN = 32;  // ADC1_CH4

// Buffer to hold current sensor readings for heatmap display
int sensorBuffer[gridRows][gridCols];


// Test mosaic (4x4)
String mosaic[gridRows][gridCols] = {
  {"Red",     "Red",   "Red",     "Red"},
  {"Green",   "Green",    "Green",  "Green"},
  {"Cyan",    "Cyan", "Cyan",      "Cyan"},
  {"Magenta", "Magenta",     "Magenta",    "Magenta"}
};

int baseline[gridRows][gridCols];



Mosiac mosiac;

// A struct representing one hall-effect sensing cell
struct SenseCell {
  CellPos cellPosData; 
  int sensorVal;
 // int sensorPin;
  bool occupied = false; 

  // Constructor
  SenseCell(int x = 0, int y = 0)
      :  cellPosData( CellPos(x, y)), sensorVal(0) { }

  void debug() const {
    Serial.print("Position: (");
    Serial.print(cellPosData.posX);
    Serial.print(", ");
    Serial.print(cellPosData.posY);
    Serial.print("), Sensor Pin: ");
    //Serial.print(sensorPin);
    Serial.print(", Value: ");
    Serial.println(sensorVal);
  }


  // int getSensorPin() const {
  //   return sensorPin;
  // }

  int getSensorVal() const {
    return sensorVal;
  }

  // Update with calibrated baseline
  void updateVal(float val) {
    sensorVal = abs(val - baseline[cellPosData.posX][cellPosData.posY]);
    // To check for occupancy, we can set a threshold above baseline. 
    occupied = (sensorVal > threshold);

  }
};
  

// matrix of sensors and sensemat data structure 
SenseCell senseMat[gridRows][gridCols];
// Select channel 0..15 on the mux
void setMuxChannel(uint8_t ch) {
  digitalWrite(MUX_S0, (ch & 0x01) ? HIGH : LOW);
  digitalWrite(MUX_S1, (ch & 0x02) ? HIGH : LOW);
  digitalWrite(MUX_S2, (ch & 0x04) ? HIGH : LOW);
  digitalWrite(MUX_S3, (ch & 0x08) ? HIGH : LOW);
  delayMicroseconds(5);  // settle
}

// Return the cell with the highest sensor reading
// Mux localization 

SenseCell* localize() {
  SenseCell* maxCell = nullptr;
  int maxVal = -1;
  for (int ch = 0; ch < gridRows * gridCols; ch++) { // 0..15
    setMuxChannel(ch);
    int raw = analogRead(MUX_SIG_PIN);  // 0..4095

    float val = (float) raw;

    int physRow = ch / gridCols;   // physical row from mux order
    int physCol = ch % gridCols;   // physical col from mux order
    
    // Flip vertically: logical row 0 = top, 3 = bottom
    int row = gridRows - 1 - physRow;
    int col = physCol;
    senseMat[row][col].updateVal(val);
    sensorBuffer[row][col] = senseMat[row][col].getSensorVal();
    // Only look at unoccupied cells  
    if (senseMat[row][col].getSensorVal() > maxVal ) {
      maxVal = senseMat[row][col].getSensorVal();
      maxCell = &senseMat[row][col];
    }
  
  }

  // No moving magnet tile detected, return null
  if (maxVal < threshold) {
   // Serial.println("No tile detected. Please place a tile.");
    return nullptr;
  }

  //Serial.print("Max Cell Sensor: ");
  //maxCell->debug();
  return maxCell;

}


//CSV format for heatmap 
void display(const std::set<CellPos>& targets) {     
  // Send CSV rows
  for (int i = 0; i < gridRows; i++) {
    for (int j = 0; j < gridCols; j++) {
      Serial.print(sensorBuffer[i][j]);
      if (j < gridCols - 1) {
        Serial.print(",");
      }
    }
    Serial.println();
  }

  // Send target marker: T,row,col
  for (const CellPos& target : targets) {
    Serial.print("T,");
    Serial.print(target.posX); // row
    Serial.print(",");
    Serial.println(target.posY); // col
  }

  Serial.println("---"); // frame separator
  delay(50);
}


// Compute absolute distance vector between current and target cells
// Calculate vector to closest target if multiple targets 
CellPos calculateVector(const std::set<CellPos>& targets, const SenseCell* current) {
  CellPos closestTarget = *targets.begin();
  int minDistance = INT_MAX;
  for (const CellPos& target : targets) {
    int distance = abs(current->cellPosData.posX - target.posX) + abs(current->cellPosData.posY - target.posY);
    if (distance < minDistance) {
      minDistance = distance;
      closestTarget = target;
    }
  }
  return CellPos(
    abs(current->cellPosData.posX - closestTarget.posX),
    abs(current->cellPosData.posY - closestTarget.posY)
  );
}



void giveFeedback(const CellPos& diff) {
  float distance = sqrt(diff.posX * diff.posX + diff.posY * diff.posY);

  // float maxDistance = 2.0;
  // float magnitude = min(distance / maxDistance, 1.0f);

  // Normalize direction (+1, 0, -1)
  // int8_t dx = (diff.posX > 0) - (diff.posX < 0);
  // int8_t dy = (diff.posY > 0) - (diff.posY < 0);

  // if (dx == 0 && dy == 0) {
  //   // No movement needed
  //   magnitude = 0.0;
  // }

  FeedbackPacket packet;
  packet.magnitude = distance;
  packet.dx = diff.posX;
  packet.dy = diff.posY;
  
  esp_now_send(receiverMAC, (uint8_t *)&packet, sizeof(packet));
}

// Target guidance loop 
void guideToTarget(const std::set<CellPos>& targets) {

    //Find current cell user is hovering over 
    SenseCell* current = localize();
    display(targets);
    
    if (current == nullptr) {
      // Send empty feedback or some signal that no tile is detected
      FeedbackPacket packet;
      packet.magnitude = -1.0; // Use -1 to indicate no tile detected
      packet.dx = 0;
      packet.dy = 0;
      esp_now_send(receiverMAC, (uint8_t *)&packet, sizeof(packet));
      
      //Serial.println("No tile detected. Please place a tile.");
      //delay(1000);
      return;
    }
  


    
    //display();
    //Calculate absolute vector difference magnitude 
    CellPos diff = calculateVector(targets, current);

    // Serial.print("Position: (");
    // Serial.print(current->cellPosData.posX);
    // Serial.print(", ");
    // Serial.print(current->cellPosData.posY);
    // Serial.print(") ");
    giveFeedback(diff);

    // Exit when user reaches target ish area 
    // if (diff.posX <= epsilon && diff.posY <= epsilon) {
    //   break;
    // }
 

   //delay(300);
  
}

// Setup the mapping from color list of positions
void initializeMosaicMap() {
  for (int i = 0; i < gridRows; i++) {
    for (int j = 0; j < gridCols; j++) {
      String color = mosaic[i][j];
      // mosaicMap[color].push_back(CellPos(i, j));
      int colorIndex = color_to_id(color);
      mosiac.addCellToColor(colorIndex, CellPos(i, j));
    }
  }
}




// Use MUX 
void calibrateSensors() {



  for (int ch = 0; ch < gridRows * gridCols; ch++) { // 0..15
      long sum = 0;
      for (int k = 0; k < 400; k++) {
        setMuxChannel(ch);
        int raw = analogRead(MUX_SIG_PIN);  // 0..4095
        sum += raw; // raw read
        delay(2);
      }
    int physRow = ch / gridCols;   // physical row from mux order
    int physCol = ch % gridCols;   // physical col from mux order
    
    // Flip vertically: logical row 0 = top, 3 = bottom
    int row = gridRows - 1 - physRow;
    int col = physCol;
    baseline[row][col] = sum / 400; // average baseline
  }
  Serial.println("MUX Calibration complete.");
   
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

void setup() {
  Serial.begin(250000);
  delay(2000);
  // Initialize mosaic map
  initializeMosaicMap();

  mosiac.debug();

  // Initialize WiFi in station mode for ESP-NOW
  WiFi.mode(WIFI_STA);   // ESP-NOW works in station mode
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
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


  // Initialize RFID reader
  if (!rfid.begin()) {
    Serial.println("RFID init FAILED. Check 3.3V, GND, and SPI wiring.");
    while (true) { delay(1000); }
  }

  //Serial.println("RFID init OK. Tap a tag...");

  // Setup mux
  // Setup 4 pins that feed into hall effect sensors
  Serial.println("Initializing sensors...");
  for (int i = 0; i < gridRows; i++) {
    for (int j = 0; j < gridCols; j++) {
      //pinMode(pins[i][j], INPUT);
     senseMat[i][j] = SenseCell(i, j);
    }
  }

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);

  pinMode(MUX_SIG_PIN, INPUT);


  calibrateSensors();
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
    Serial.println(id_to_color[colorIndex]);
    return id_to_color[colorIndex];
  } else {
    Serial.println("Failed to write block.");
    return "";
  }
}

CellPos colorToCellPos(const String& color) {
  if (color == "Red") return CellPos(0, 0);
  if (color == "Green") return CellPos(0, 1);
  if (color == "Cyan") return CellPos(1, 0);
  if (color == "Magenta") return CellPos(1, 1);
  return CellPos(-1, -1); // Invalid color
}


// Targets is a set 
std::set<CellPos> currTarget; 
void loop() {
  RFIDRead r = rfid.poll();
  // if (!r.present) return;

  guideToTarget(currTarget);
  
  uint32_t now = millis();
  if (r.uid32 == lastUid && (now - lastActionMs) < 800) {
    return;
  }
  lastUid = r.uid32;
  lastActionMs = now;
    


  if (r.present && readMode) {
    // If it's the same tag still sitting there, don't keep re-reading
    int colorIndex = handleRead();
    //Serial.println("Color read: " + id_to_color[colorIndex]);
    if (colorIndex >= 0) {
      auto positions = mosiac.getTargetCells(colorIndex);
      currTarget = positions; // Update global target set
    }
    // if (!positions.empty()) {
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

  } else if (r.present && !readMode) {
    String color = handleWrite();
  //  Serial.println("Color written: " + color);
  }

  
 
  delay(50);

}

