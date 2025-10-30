// A struct representing one hall-effect sensing cell
struct SenseCell {
  int posX;
  int posY;
  int sensorVal;
  int sensorPin;

  // Constructor
  SenseCell(int x = 0, int y = 0, int pin = -1)
      : posX(x), posY(y), sensorVal(0), sensorPin(pin) {}

  void debug() const {
    Serial.print("Position: (");
    Serial.print(posX);
    Serial.print(", ");
    Serial.print(posY);
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
    sensorVal = analogRead(sensorPin);
  }
};

// 2x2 matrix of sensors
SenseCell senseMat[2][2];

// Return the cell with the highest sensor reading
SenseCell* localize() {
  SenseCell* maxCell = nullptr;
  int maxVal = -1;

  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      senseMat[i][j].updateVal();
      if (senseMat[i][j].getSensorVal() > maxVal) {
        maxVal = senseMat[i][j].getSensorVal();
        maxCell = &senseMat[i][j];
      }
    }
  }
  return maxCell;
}

// For now just print the matrix
void display() {
  Serial.println("Sensor Matrix:");
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      Serial.print(senseMat[i][j].getSensorVal());
      Serial.print("\t");
    }
    Serial.println();
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Setup 4 pins that feed into hall effect sensors
  int pins[2][2] = {{A0, A1}, {A2, A3}};
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      pinMode(pins[i][j], INPUT);
      senseMat[i][j] = SenseCell(i, j, pins[i][j]);
    }
  }

  Serial.println("System initialized.\n");
}

void loop() {
  SenseCell* strongest = localize();

  if (strongest != nullptr) {
    Serial.print("Strongest field at: (");
    Serial.print(strongest->posX);
    Serial.print(", ");
    Serial.print(strongest->posY);
    Serial.print(") with value ");
    Serial.println(strongest->getSensorVal());
  }

  display();
  delay(1000);
}
