// OneWire object, includes DS18B20 temperature reading

#include "Ports.h"

#define TEMP_ERROR -1000

class OneWire {
  public:    
    OneWire();
    bool setup(uint8_t);
    uint8_t reset();
    bool startConversion();
    int16_t readTemperature();
    bool setResolution(uint8_t);
    
  private:
    uint8_t pinBitmask, portID, dataBytes[9];
    const uint8_t readROM = 0x33;
    const uint8_t matchROM = 0x55;
    const uint8_t skipROM = 0xCC;
    const uint8_t convertT = 0x44;
    const uint8_t copyScratchpad = 0x48;
    const uint8_t writeScratchpad = 0x4E;
    const uint8_t readScratchpad = 0xBE;
    const uint8_t res9bit = 0x1F;
    const uint8_t res10bit = 0x3F;
    const uint8_t res11bit = 0x5F;
    const uint8_t res12bit = 0x7F;
    enum class timing { RESETLOW = 480, RESETREL = 70, RESETDELAY = 410, READLOW = 6, READREL = 9, READDELAY = 55,
                        WRITE0LOW = 60, WRITE0REL = 10, WRITE1LOW = 6, WRITE1REL = 64 };
    void write(uint8_t);
    uint8_t read();
    void readBytes(uint8_t);
    inline void pinLow();
    inline void pinRelease();
    inline uint8_t pinRead();
    void lowRelease(uint16_t, uint16_t);
    uint8_t CRC(uint8_t);
};

OneWire::OneWire() {}

bool OneWire::setup(uint8_t owPin) {
  pinBitmask = digitalPinToBitMask(owPin);
  portID = digitalPinToPort(owPin);

  if (portID == NOT_A_PORT) return false;
  port[portID]->DIRCLR = pinBitmask;
  return true;
}

inline void OneWire::pinLow() {
  port[portID]->DIRSET = pinBitmask;
  port[portID]->OUTCLR = pinBitmask;
}

inline void OneWire::pinRelease() {
  port[portID]->DIRCLR = pinBitmask;
}

inline uint8_t OneWire::pinRead () {
  return (port[portID]->IN & pinBitmask) ? 1 : 0;
}

void OneWire::lowRelease(uint16_t low, uint16_t high) {
  pinLow();
  delayMicroseconds(low);
  pinRelease();
  delayMicroseconds(high);
}

// bus reset, returns 0 if device is present
uint8_t OneWire::reset() {
  uint8_t data = 1;

  lowRelease((uint16_t)timing::RESETLOW, (uint8_t)timing::RESETREL);
  data = pinRead();
  delayMicroseconds((uint16_t)timing::RESETDELAY);
  return data;
}

void OneWire::write(uint8_t data) {
  for (uint8_t i = 0; i < 8; i++) {
    if ((data & 1) == 1) lowRelease((uint16_t)timing::WRITE1LOW, (uint16_t)timing::WRITE1REL);
    else lowRelease ((uint16_t)timing::WRITE0LOW, (uint16_t)timing::WRITE0REL);
    data = data >> 1;
  }
}

uint8_t OneWire::read() {
  uint8_t data = 0;
  for (uint8_t i = 0; i < 8; i++) {
    lowRelease((uint16_t)timing::READLOW, (uint16_t)timing::READREL);
    data = data | pinRead() << i;
    delayMicroseconds((uint16_t)timing::READDELAY);
  }
  return data;
}

// read bytes into array, least significant byte first
void OneWire::readBytes(uint8_t bytes) {
  for (uint8_t i = 0; i < bytes; i++) {
    dataBytes[i] = read();
  }
}

// calculate CRC over buffer - 0x00 is correct
uint8_t OneWire::CRC(uint8_t bytes) {
  uint8_t crc = 0;
  for (uint8_t j = 0; j < bytes; j++) {
    crc = crc ^ dataBytes[j];
    for (uint8_t i = 0; i < 8; i++) crc = crc >> 1 ^ ((crc & 1) ? 0x8c : 0);
  }
  return crc;
}

// start conversion
bool OneWire::startConversion() {
  if (reset() != 0) {
    return false;
  } else {
    write(skipROM);
    write(convertT);
  }
  return true;
}

// read temperature of a single DS18B20 on the bus
// returns degrees multiplied by 100 (integer only)
int16_t OneWire::readTemperature() {
  int16_t rawTemp;
  
  if (reset() != 0) {
    return TEMP_ERROR;
  } else {
    write(skipROM);
    write(readScratchpad);
    readBytes(9);
    if (CRC(9) == 0) {
      rawTemp = (((int16_t)dataBytes[1]) << 8) | dataBytes[0];
      return (rawTemp * 25 + 2) / 4;
    }
  }
  return TEMP_ERROR;
}

bool OneWire::setResolution(uint8_t resolution) {
  uint8_t res;
  
  switch(resolution) {
    case 12: res = res12bit; break;
    case 11: res = res11bit; break;
    case 10: res = res10bit; break;
    default: res = res9bit; break;
  }

  if (reset() != 0) {
    write(skipROM);
    write(writeScratchpad);
    write(0);
    write(0);
    write(res);
    reset();
    return true;
  }
  return false;
}
