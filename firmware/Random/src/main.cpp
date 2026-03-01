// Ensure Arduino core is included first so `Serial` is declared
#include <Arduino.h>
// #include <vector>
// #include <map>
// (other includes are below)

// #include <esp_now.h>
// #include <WiFi.h>

// // ESP now receiver's MAC
// uint8_t receiverMAC[] = {0xB8, 0xF8, 0x62, 0xF8, 0x6C, 0x40};


// typedef struct __attribute__((packed)){
//   float magnitude;   // 0.0 → 1.0 normalized distance
//   int8_t dx;         // -1, 0, 1
//   int8_t dy;         // -1, 0, 1
// } FeedbackPacket;
// using namespace std;

// //INIT PARAMETERS 
// int const offset = 200; 
// int const gridRows = 2; 
// int const gridCols = 2;
// float const epsilon = 0.0; 
// int const buzzer = 9;
// int NOISE_THRESHOLD = 120;   // tune this experimentally



// // Set up 16 ADC pins for hall effect sensors (4x4 grid)
// // static const int pins[gridRows][gridCols] = {
// //   {11, 12, 14, 15},
// //   {13, 10, 9, 8},
// //   {7, 6, 5, 4},
// //   {3, 2, 1, 0}
// // };

// // 2x2 test
// static const int pins[gridRows][gridCols] = {
//  {12, 11},
//  {10, 9}
// };

// // Test mosaic (4x4)
// // String mosaic[gridRows][gridCols] = {
// //   {"Red", "Green", "Blue", "Yellow"},
// //   {"Cyan", "Magenta", "Orange", "Purple"},
// //   {"White", "Black", "Gray", "Brown"},
// //   {"Pink", "Lime", "Teal", "Navy"}
// // };

// // Test Mosaic (2x2) 
// String mosaic[gridRows][gridCols] = {
//   {"Red", "Green"},
//   {"Blue", "Yellow"}
// };

// struct CellPos {
//   int posX;
//   int posY;
//   CellPos(int x = 0, int y = 0) : posX(x), posY(y) {}
// };

// // Define target mosaic colors 
// std::map<String, vector<CellPos>> mosaicMap;




// // A struct representing one hall-effect sensing cell
// struct SenseCell {
//   CellPos cellPosData; 
//   int sensorVal;
//   int sensorPin;

//   // Constructor
//   SenseCell(int x = 0, int y = 0, int pin = -1)
//       :  cellPosData( CellPos(x, y)), sensorVal(0), sensorPin(pin) { }

//   void debug() const {
//     Serial.print("Position: (");
//     Serial.print(cellPosData.posX);
//     Serial.print(", ");
//     Serial.print(cellPosData.posY);
//     Serial.print("), Sensor Pin: ");
//     Serial.print(sensorPin);
//     Serial.print(", Value: ");
//     Serial.println(sensorVal);
//   }

//   int getSensorPin() const {
//     return sensorPin;
//   }

//   int getSensorVal() const {
//     return sensorVal;
//   }

//   void updateVal() {
//     // Read analog value and compute absolute difference from offset
//     sensorVal = abs(200 - analogRead(sensorPin)) ;
//   }
// };





// // matrix of sensors and sensemat data structure 
// SenseCell senseMat[gridRows][gridCols];
// int sensorBuffer[gridRows][gridCols];


// // Return the cell with the highest sensor reading
// // Absolute value 
// // Mux localization 
// SenseCell* localize() {
//   SenseCell* maxCell = nullptr;
//   int maxVal = -1;

//   for (int i = 0; i < gridRows; i++) {
//     for (int j = 0; j < gridCols; j++) {
//       senseMat[i][j].updateVal();
//       sensorBuffer[i][j] = senseMat[i][j].getSensorVal();

//       if (sensorBuffer[i][j] > maxVal) {
//         maxVal = sensorBuffer[i][j];
//         maxCell = &senseMat[i][j];
//       }
//     }
//   }

//   // If magnet not present return NULL
//   // if (maxVal < NOISE_THRESHOLD) {
//   //   return nullptr;
//   // }

//   return maxCell;
// }


// //CSV format for heatmap 
// void display() {
//   // Send CSV rows
//   for (int i = 0; i < gridRows; i++) {
//     for (int j = 0; j < gridCols; j++) {
//       Serial.print(sensorBuffer[i][j]);
//       if (j < gridCols - 1) {
//         Serial.print(",");  // separate columns
//       }
//     }
//     Serial.println(); // new line after each row
//   }

//   Serial.println("---"); // row separator for Python parser
//   delay(1000); // 5 updates/sec
// }


// // Compute absolute distance vector between current and target cells
// CellPos calculateVector(const CellPos& target, const SenseCell* current) {
//   return CellPos(
//     abs(current->cellPosData.posX - target.posX),
//     abs(current->cellPosData.posY - target.posY)
//   );
// }



// void giveFeedback(const CellPos& diff) {
//   float distance = sqrt(diff.posX * diff.posX + diff.posY * diff.posY);

//   float maxDistance = 2.0;
//   float magnitude = min(distance / maxDistance, 1.0f);

//   // Normalize direction (+1, 0, -1)
//   int8_t dx = (diff.posX > 0) - (diff.posX < 0);
//   int8_t dy = (diff.posY > 0) - (diff.posY < 0);

//   if (dx == 0 && dy == 0) {
//     // No movement needed
//     magnitude = 0.0;
//   }

//   FeedbackPacket packet;
//   packet.magnitude = magnitude;
//   packet.dx = dx;
//   packet.dy = dy;

//   esp_now_send(receiverMAC, (uint8_t *)&packet, sizeof(packet));
// }

// // Target guidance loop 
// void guideToTarget(const CellPos& target) {
//   while (true) {

//     //Find current cell user is hovering over 
//     SenseCell* current = localize();
//     display();


    
//     //display();
//     //Calculate absolute vector difference magnitude 
//     CellPos diff = calculateVector(target, current);

//     // Serial.print("Position: (");
//     // Serial.print(current->cellPosData.posX);
//     // Serial.print(", ");
//     // Serial.print(current->cellPosData.posY);
//     // Serial.print(") ");
//     giveFeedback(diff);

//     // Exit when user reaches target ish area 
//     // if (diff.posX <= epsilon && diff.posY <= epsilon) {
//     //   break;
//     // }
 

//     delay(200);
//   }
// }

// // Setup the mapping from color list of positions
// void initializeMosaicMap() {
//   for (int i = 0; i < gridRows; i++) {
//     for (int j = 0; j < gridCols; j++) {
//       String color = mosaic[i][j];
//       mosaicMap[color].push_back(CellPos(i, j));
//     }
//   }
// }



// //SETUP-------------------------------------------------------------------
// void setup() {
//   Serial.begin(115200);
//   delay(1000);

//   // ESP Now initialization
//   WiFi.mode(WIFI_STA);   // ESP-NOW works in station mode

//   if (esp_now_init() != ESP_OK) {
//     Serial.println("Error initializing ESP-NOW");
//     return;
//   }

//   // Register peer
//   esp_now_peer_info_t peerInfo = {};
//   memcpy(peerInfo.peer_addr, receiverMAC, 6);
//   peerInfo.channel = 0;  
//   peerInfo.encrypt = false;

//   if (esp_now_add_peer(&peerInfo) != ESP_OK) {
//     Serial.println("Failed to add peer");
//     return;
//   }


//   pinMode(buzzer, OUTPUT);

//   // Setup 4 pins that feed into hall effect sensors
//   for (int i = 0; i < gridRows; i++) {
//     for (int j = 0; j < gridCols; j++) {
//       pinMode(pins[i][j], INPUT);
//       senseMat[i][j] = SenseCell(i, j, pins[i][j]);
//     }
//   }

//   Serial.println("System initialized.\n");
// }



// void loop() {

//   //Example test for esp now communication
//   // SenseCell* current = &senseMat[1][1];
//   // CellPos target(0, 0);
//   // CellPos diff = calculateVector(target, current);
//   // giveFeedback(diff);
//   // delay(2000);

//   //Red tile for example 
//   //CellPos target(0, 0);


//   //Serial.println("Please place the RED tile.");
//   //guideToTarget(target);


//   // Heat map test 
//   SenseCell* current = localize();  
//   display();


//   //display();
  
//   //Serial.println("Done.\n");
//   //delay(1000);

//   // const char *msg = "Hello from ESP32-S3!";
//   // esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)msg, strlen(msg));

//   // if (result == ESP_OK) {
//   //   Serial.println("Message sent successfully");
//   // } else {
//   //   Serial.println("Error sending message");
//   // }

//   delay(100);
// }





#include <vector>
#include <map>
#include <Arduino.h>

#include <esp_now.h>
#include <WiFi.h>

// ESP now receiver's MAC
uint8_t receiverMAC[] = {0xB8, 0xF8, 0x62, 0xF8, 0x6C, 0x40};



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


// Set up 16 ADC pins for hall effect sensors (4x4 grid)
// NOTE: Replace the GPIO numbers below with the actual ADC-capable
// GPIOs on your ESP32-S3 dev board. The example below uses GPIO0..15
// as placeholders and will compile, but you should verify each pin's
// ADC capability and wiring before relying on measurements.
static const int pins[gridRows][gridCols] = {
  {9, 10, 13, 16}, 
  {11, 12, 15, 7},
  {3, 8, 6, 5},
  {18, 17, 4, 19}
  
};


// Test mosaic (4x4)
String mosaic[gridRows][gridCols] = {
  {"Red", "Green"},
  {"Cyan", "Magenta"}
};

struct CellPos {
  int posX;
  int posY;
  CellPos(int x = 0, int y = 0) : posX(x), posY(y) {}
};

// Define target mosaic colors 
std::map<String, vector<CellPos>> mosaicMap;




// A struct representing one hall-effect sensing cell
struct SenseCell {
  CellPos cellPosData; 
  int sensorVal;
  int sensorPin;

  // Constructor
  SenseCell(int x = 0, int y = 0, int pin = -1)
      :  cellPosData( CellPos(x, y)), sensorVal(0), sensorPin(pin) { }

  void debug() const {
    Serial.print("Position: (");
    Serial.print(cellPosData.posX);
    Serial.print(", ");
    Serial.print(cellPosData.posY);
    Serial.print("), Sensor Pin: ");
    Serial.print(sensorPin);
    Serial.print(", Value: ");
    Serial.println(sensorVal);
  }

  int getSensorPin() const {
    return sensorPin;
  }

  int getSensorVal() const {
    return sensorVal;
  }

  void updateVal() {
    sensorVal = abs(200 - analogRead(sensorPin)) ;
  }
};





// matrix of sensors and sensemat data structure 
SenseCell senseMat[gridRows][gridCols];
int sensorBuffer[gridRows][gridCols];


// Return the cell with the highest sensor reading
// Mux localization 
SenseCell* localize() {
  SenseCell* maxCell = nullptr;
  int maxVal = -1;

  for (int i = 0; i < gridRows; i++) {
    for (int j = 0; j < gridCols; j++) {
      senseMat[i][j].updateVal();
      sensorBuffer[i][j] = senseMat[i][j].getSensorVal();
      if (senseMat[i][j].getSensorVal() > maxVal) {
        maxVal = senseMat[i][j].getSensorVal();
        maxCell = &senseMat[i][j];
      }
    }
  }

  //Serial.print("Max Cell Sensor: ");
  //maxCell->debug();
  return maxCell;



  // For 16 x 16 grid we drive one row at a time
  // and read all columns to find max


}

//CSV format for heatmap 
void display() {
  // Send CSV rows
  for (int i = 0; i < gridRows; i++) {
    for (int j = 0; j < gridCols; j++) {
      Serial.print(sensorBuffer[i][j]);
      if (j < gridCols - 1) {
        Serial.print(",");  // separate columns
      }
    }
    Serial.println(); // new line after each row
  }

  Serial.println("---"); // row separator for Python parser
  delay(1000); // 5 updates/sec
}


// Compute absolute distance vector between current and target cells
CellPos calculateVector(const CellPos& target, const SenseCell* current) {
  return CellPos(
    abs(current->cellPosData.posX - target.posX),
    abs(current->cellPosData.posY - target.posY)
  );
}



void giveFeedback(const CellPos& diff) {
  float distance = sqrt(diff.posX * diff.posX + diff.posY * diff.posY);

  float maxDistance = 2.0;
  float magnitude = min(distance / maxDistance, 1.0f);

  // Normalize direction (+1, 0, -1)
  int8_t dx = (diff.posX > 0) - (diff.posX < 0);
  int8_t dy = (diff.posY > 0) - (diff.posY < 0);

  if (dx == 0 && dy == 0) {
    // No movement needed
    magnitude = 0.0;
  }

  FeedbackPacket packet;
  packet.magnitude = magnitude;
  packet.dx = dx;
  packet.dy = dy;

  esp_now_send(receiverMAC, (uint8_t *)&packet, sizeof(packet));
}

// Target guidance loop 
void guideToTarget(const CellPos& target) {
  while (true) {

    //Find current cell user is hovering over 
    SenseCell* current = localize();
    display();


    
    //display();
    //Calculate absolute vector difference magnitude 
    CellPos diff = calculateVector(target, current);

    // Serial.print("Position: (");
    // Serial.print(current->cellPosData.posX);
    // Serial.print(", ");
    // Serial.print(current->cellPosData.posY);
    // Serial.print(") ");
    giveFeedback(diff);

    // Exit when user reaches target ish area 
    if (diff.posX <= epsilon && diff.posY <= epsilon) {
      break;
    }
 

    delay(300);
  }
}

// Setup the mapping from color list of positions
void initializeMosaicMap() {
  for (int i = 0; i < gridRows; i++) {
    for (int j = 0; j < gridCols; j++) {
      String color = mosaic[i][j];
      mosaicMap[color].push_back(CellPos(i, j));
    }
  }
}



//SETUP-------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  // ESP Now initialization
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


  pinMode(buzzer, OUTPUT);

  // Setup 4 pins that feed into hall effect sensors
  for (int i = 0; i < gridRows; i++) {
    for (int j = 0; j < gridCols; j++) {
      pinMode(pins[i][j], INPUT);
      senseMat[i][j] = SenseCell(i, j, pins[i][j]);
    }
  }

  Serial.println("System initialized.\n");
}



void loop() {

  //Example test for esp now communication
  // SenseCell* current = &senseMat[1][1];
  // CellPos target(0, 0);
  // CellPos diff = calculateVector(target, current);
  // giveFeedback(diff);
  // delay(2000);

  //Red tile for example 
  CellPos target(0, 0);

  //tone(buzzer, 1000);
  //Serial.println("Please place the RED tile.");
  guideToTarget(target);


  // Heat map test 
  // SenseCell* current = localize();  
  //display();


  //display();
  
  //Serial.println("Done.\n");
  //delay(1000);

  // const char *msg = "Hello from ESP32-S3!";
  // esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)msg, strlen(msg));

  // if (result == ESP_OK) {
  //   Serial.println("Message sent successfully");
  // } else {
  //   Serial.println("Error sending message");
  // }

  delay(200);
}


