// #include <vector>
// #include <map>
// #include <Arduino.h>

// using namespace std;

// //INIT PARAMETERS 
// int const offset = 200; 
// int const gridRows = 2; 
// int const gridCols = 3;
// float const epsilon = 0.0; 
// int const buzzer = 9;


// //Test mosiac 
// String mosaic[gridRows][gridCols] = {
//   {"Red", "Green"},
//   {"Blue", "Red"}
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
//     sensorVal = abs(200 - analogRead(sensorPin)) ;
//   }
// };





// // matrix of sensors and sensemat data structure 
// SenseCell senseMat[gridRows][gridCols];
// int sensorBuffer[gridRows][gridCols];


// // Return the cell with the highest sensor reading
// SenseCell* localize() {
//   SenseCell* maxCell = nullptr;
//   int maxVal = -1;

//   for (int i = 0; i < gridRows; i++) {
//     for (int j = 0; j < gridCols; j++) {
//       senseMat[i][j].updateVal();
//       sensorBuffer[i][j] = senseMat[i][j].getSensorVal();
//       if (senseMat[i][j].getSensorVal() > maxVal) {
//         maxVal = senseMat[i][j].getSensorVal();
//         maxCell = &senseMat[i][j];
//       }
//     }
//   }
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




// // Feedback based on direction and distance
// void giveFeedback(const CellPos& diff) {
//   // Compute magnitude for scaling tone frequency
//   float distance = sqrt(diff.posX * diff.posX + diff.posY * diff.posY);

//   // If we're close enough to the target
//   if (distance <= epsilon) {
//     noTone(buzzer);
//     Serial.println("Tile correctly placed!");
//     return;
//   }

//   //Buzzer tone insteand of LRA vibration motor for testing purposes 
//   float maxDistance = 2.0;           // maximum expected distance
//   float normDistance = min(distance, maxDistance) / maxDistance; 
//   int minFreq = 400;                 // low tone when close
//   int maxFreq = 800;                 // high tone when far
//   int toneFreq = minFreq + (int)((maxFreq - minFreq) * normDistance);

//   // Determine direction and print + play tone
//   Serial.print("Move ");
//   if (diff.posX < -epsilon) {
//     Serial.print("DOWN ");
//     tone(buzzer, toneFreq);
//   } else if (diff.posX > epsilon) {
//     Serial.print("UP ");
//     tone(buzzer, toneFreq);
//   }

//   if (diff.posY < -epsilon) {
//     Serial.print("RIGHT ");
//     tone(buzzer, toneFreq);
//   } else if (diff.posY > epsilon) {
//     Serial.print("LEFT ");
//     tone(buzzer, toneFreq);
//   }

//   Serial.println();
// }


// // Target guidance loop 
// void guideToTarget(const CellPos& target) {
//   while (true) {

//     //Find current cell user is hovering over 
//     SenseCell* current = localize();
    
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
//     if (diff.posX <= epsilon && diff.posY <= epsilon) {
//       break;
//     }
 

//     delay(300);
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


//   pinMode(buzzer, OUTPUT);

//   // Setup 4 pins that feed into hall effect sensors
//   int pins[gridRows][gridCols] = {{A1, A0, A4}, {A3, A2, A5}};
//   for (int i = 0; i < gridRows; i++) {
//     for (int j = 0; j < gridCols; j++) {
//       pinMode(pins[i][j], INPUT);
//       senseMat[i][j] = SenseCell(i, j, pins[i][j]);
//     }
//   }

//   Serial.println("System initialized.\n");
// }



// void loop() {
//   //Red tile for example 
//  // CellPos target(0, 0);

//   //tone(buzzer, 1000);
//   //Serial.println("Please place the RED tile.");
//  // guideToTarget(target);

//   SenseCell* current = localize();
    
//   display();


//   //display();
  
//   //Serial.println("Done.\n");
//   //delay(1000);
// }



void setup() {
   pinMode(9, OUTPUT); // Set pin 9 as output
}
void loop() {
   digitalWrite(9, HIGH); // Turn LED on
   delay(500); // Wait for 0.5 seconds
   digitalWrite(9, LOW); // Turn LED off
   delay(500); // Wait for 0.5 seconds
}
