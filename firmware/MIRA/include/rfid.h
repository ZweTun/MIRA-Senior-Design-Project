#pragma once
#include <Arduino.h>
#include <stdint.h>

// Simple result struct
struct RFIDRead {
  bool present;
  uint32_t uid32;     // hashed / packed UID for easy mapping
};

// Block info struct
struct BlockInfo {
  uint32_t uid32;
  int colorIndex;
  bool valid = false; 
};

class RFID {
public:
  // Pass pins + SPI pins explicitly (ESP32)
  RFID(uint8_t ssPin, uint8_t rstPin,
       uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin);

  bool begin();
  RFIDRead poll();          // non-blocking: returns {false,0} if no new tag
  void halt();              // optional: stop crypto / halt tag


  BlockInfo readBlock();
  bool writeBlock(const int colorIndex); // writes to block 1, returns success 

private:
  uint8_t _ss, _rst;
  uint8_t _sck, _miso, _mosi;

  // “Debounce” / stability
  uint32_t _lastUid = 0;
  uint32_t _lastSeenMs = 0;
  uint32_t _holdMs = 400;   // how long to keep reporting same UID after last seen

  uint32_t uidToU32(const uint8_t* uidBytes, uint8_t uidSize) const;
  void dump_byte_array(byte *buffer, byte bufferSize);
};