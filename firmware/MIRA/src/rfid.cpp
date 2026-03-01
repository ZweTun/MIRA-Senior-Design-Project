#include "rfid.h"
#include <SPI.h>
#include <MFRC522.h>


static MFRC522* g_rfid = nullptr;
static bool readBlock16(MFRC522& r, byte blockAddr, byte out[16]);
static MFRC522::MIFARE_Key defaultKey();
static bool writeBlock16(MFRC522& r, byte blockAddr,  byte data[16]);
static bool ntagWriteColor(MFRC522& r, uint8_t colorIndex);
static bool ntagReadColor(MFRC522& r, byte& out);



RFID::RFID(uint8_t ssPin, uint8_t rstPin,
           uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin)
  : _ss(ssPin), _rst(rstPin), _sck(sckPin), _miso(misoPin), _mosi(mosiPin) {}


bool RFID::begin() {
  // Start SPI bus with explicit pins (ESP32 best practice)
  SPI.begin(_sck, _miso, _mosi, _ss);

  if (g_rfid) {
    delete g_rfid;
    g_rfid = nullptr;
  }
  g_rfid = new MFRC522(_ss, _rst);
  g_rfid->PCD_Init();

  // Basic “is it alive?” check: read VersionReg
  byte v = g_rfid->PCD_ReadRegister(g_rfid->VersionReg);
  if (v == 0x00 || v == 0xFF) {
    // Often indicates wiring/power problem
    return false;
  }
  return true;
}

uint32_t RFID::uidToU32(const uint8_t* uidBytes, uint8_t uidSize) const {
  // Works for 4/7/10-byte UIDs: fold into a 32-bit hash-ish value.
  // (Good enough for demos; collisions unlikely at small scale.)
  uint32_t h = 2166136261u; // FNV offset basis
  for (uint8_t i = 0; i < uidSize; i++) {
    h ^= uidBytes[i];
    h *= 16777619u;         // FNV prime
  }
  return h;
}

/**
 * Helper routine to dump a byte array as hex values to Serial.
 */
void RFID::dump_byte_array(byte *buffer, byte bufferSize) {
    for (byte i = 0; i < bufferSize; i++) {
        Serial.print(buffer[i] < 0x10 ? " 0" : " ");
        Serial.print(buffer[i], HEX);
    }
}

RFIDRead RFID::poll() {
  if (!g_rfid) return {false, 0};

  // If no new card, optionally “hold” last UID for a short time
  if (!g_rfid->PICC_IsNewCardPresent() || !g_rfid->PICC_ReadCardSerial()) {
    if (_lastUid != 0 && (millis() - _lastSeenMs) < _holdMs) {
      return {true, _lastUid}; // keep stable for UI/haptics
    }
    return {false, 0};
  }

  uint32_t uid32 = uidToU32(g_rfid->uid.uidByte, g_rfid->uid.size);
  // Serial.print("Card detected: UID=");
  // dump_byte_array(g_rfid->uid.uidByte, g_rfid->uid.size);
  Serial.println();

  _lastUid = uid32;
  _lastSeenMs = millis();

  // Stop talking to the card cleanly
//halt();

  return {true, uid32};
}

void RFID::halt() {
  if (!g_rfid) return;
  // g_rfid->PICC_HaltA();
  g_rfid->PCD_StopCrypto1();
}

bool RFID::writeBlock(const int colorIndex) {

  // if (!g_rfid) {
  //  Serial.println("RFID not initialized");
  //  halt();
  //  return false;   // in writeBlock
  // }
  // // Encode color as int 
  // // Set valid to true 
  // byte data[16] = {0};
  // String title = "BLOCK";
  // for (size_t i = 0; i < 5; i++) {
  //   data[i] = (byte)title.charAt(i);
  // }


  // if (colorIndex < 0 ) return false;
  // data[5] = (byte)colorIndex; // Store color index at byte 5
  // data[6] = 1; // Set valid flag at byte 6

  // bool success = writeBlock16(*g_rfid, 4, data);
  // if (success) {
  //   Serial.println("Block written successfully.");
  // } else {
  //   Serial.println("Failed to write block.");
  // }
  // halt();
  // return success;



   if (!g_rfid) return false;
  if (colorIndex < 0 || colorIndex > 255) return false;

  bool ok = ntagWriteColor(*g_rfid, (uint8_t)colorIndex);

  halt(); // ok to halt after op
  return ok;
}

BlockInfo RFID::readBlock() {
  // BlockInfo info;
  // if (!g_rfid) {
  //   halt();
  //   return info;    // in readBlock
  // }

  // byte blockAddr = 4; // where we store ouur info 
  // byte data[16];
  // if (!readBlock16(*g_rfid, blockAddr, data)) {
  //   return info;
  // }

  // if (!(data[0]=='B' && data[1]=='L' && data[2]=='O' && data[3]=='C' && data[4]=='K')) {
  //   info.valid = false;
  //   return info;
  // }

  // // Extract color index from data[5]
  // int colorIndex = data[5];


  // info.uid32 = uidToU32(g_rfid->uid.uidByte, g_rfid->uid.size);
  // info.colorIndex = colorIndex;
  // info.valid = data[6] == 1; // Check valid flag at byte 6

  // halt();
  // return info;


  BlockInfo info;
  if (!g_rfid) return info;

  uint8_t colorIdx;
  bool ok = ntagReadColor(*g_rfid, colorIdx);

  info.uid32 = uidToU32(g_rfid->uid.uidByte, g_rfid->uid.size);
  info.valid = ok;
  info.colorIndex = ok ? colorIdx : 255;

  halt();
  return info;
}


static MFRC522::MIFARE_Key defaultKey() {
  MFRC522::MIFARE_Key key;
  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;
  return key;
}

// Reads 16 bytes from blockAddr into out[16]. Returns true on success.
static bool readBlock16(MFRC522& r, byte blockAddr, byte out[16]) {
  // Must have read a card already: r.uid must be valid

  MFRC522::MIFARE_Key key = defaultKey();

  // Authenticate (Key A)
  MFRC522::StatusCode status =
      r.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, blockAddr, &key, &(r.uid));
  if (status != MFRC522::STATUS_OK) {
    Serial.print("Auth failed: ");
    Serial.println(r.GetStatusCodeName(status));
    return false;
  }

  // Read returns 16 bytes + 2 CRC bytes = 18 total in buffer
  byte buffer[18];
  byte size = sizeof(buffer);

  status = r.MIFARE_Read(blockAddr, buffer, &size);
  if (status != MFRC522::STATUS_OK) {
    Serial.print("Read failed: ");
    Serial.println(r.GetStatusCodeName(status));
    r.PCD_StopCrypto1();
    return false;
  }

  // Copy first 16 bytes (data)
  for (byte i = 0; i < 16; i++) out[i] = buffer[i];

  r.PCD_StopCrypto1();
  return true;
}


// Returns true on success
static bool writeBlock16(MFRC522& r, byte blockAddr,  byte data[16]) {
  // Reject trailer blocks: 3, 7, 11, 15, ...
  if ((blockAddr + 1) % 4 == 0) {
    Serial.println("Refusing to write trailer block.");
    return false;
  }

  // Must have read a card already: r.uid must be valid
  MFRC522::MIFARE_Key key = defaultKey();

  // Authenticate for that block using Key A
  MFRC522::StatusCode status =
      r.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, blockAddr, &key, &(r.uid));
  if (status != MFRC522::STATUS_OK) {
    Serial.print("Auth failed: ");
    Serial.println(r.GetStatusCodeName(status));
    return false;
  }

  // Write 16 bytes
  status = r.MIFARE_Write(blockAddr, data, 16);
  if (status != MFRC522::STATUS_OK) {
    Serial.print("Write failed: ");
    Serial.println(r.GetStatusCodeName(status));
    r.PCD_StopCrypto1();
    return false;
  }

  r.PCD_StopCrypto1(); // stop encryption on PCD
  return true;
}





// NTAG White 
static constexpr byte TILE_PAGE = 4; // good default for NTAG21x user memory

static bool ntagWriteColor(MFRC522& r, uint8_t colorIndex) {
  byte p[4] = { 'T', 'I', 'L', (byte)colorIndex };
  MFRC522::StatusCode s = r.MIFARE_Ultralight_Write(TILE_PAGE, p, 4);
  if (s != MFRC522::STATUS_OK) {
    Serial.print("NTAG write failed: ");
    Serial.println(r.GetStatusCodeName(s));
    return false;
  }
  return true;
}

static bool ntagReadColor(MFRC522& r, uint8_t &colorIndexOut) {
  // MIFARE_Read(page, ...) reads 16 bytes = 4 pages starting at `page`
  byte buffer[18];
  byte size = sizeof(buffer);

  MFRC522::StatusCode s = r.MIFARE_Read(TILE_PAGE, buffer, &size);
  if (s != MFRC522::STATUS_OK) {
   // Serial.print("NTAG read failed: ");
    //Serial.println(r.GetStatusCodeName(s));
    return false;
  }

  // first 4 bytes correspond to TILE_PAGE
  if (buffer[0] != 'T' || buffer[1] != 'I' || buffer[2] != 'L') {
    return false; // not our data
  }

  colorIndexOut = buffer[3];
  return true;
}
