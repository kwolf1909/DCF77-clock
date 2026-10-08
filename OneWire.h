// OneWire object, includes DS18B20 temperature reading

#if defined (__AVR_ATtiny1614__)
PORT_t* const portOW[] = { &PORTA, &PORTB };
#endif
#if defined (__AVR_ATmega4809__)
PORT_t* const portOW[] = { &PORTA, &PORTB, &PORTC, &PORTD, &PORTE, &PORTF };
#endif

class OneWire {
  public:    
    OneWire(uint8_t);
    void setup();
    uint8_t reset();
    void write(uint8_t);
    uint8_t read();
    void readBytes(uint8_t);
    bool startConversion();
    int16_t readTemperature();
    bool setResolution(uint8_t);
    
  private:
    uint8_t pinBitmask, pinBit, portID;
    uint8_t dataBytes[9];
    const uint8_t ReadROM = 0x33;
    const uint8_t MatchROM = 0x55;
    const uint8_t SkipROM = 0xCC;
    const uint8_t ConvertT = 0x44;
    const uint8_t WriteScratchpad = 0x4E;
    const uint8_t ReadScratchpad = 0xBE;
    const uint8_t res9bit = 0x1F;
    const uint8_t res10bit = 0x3F;
    const uint8_t res11bit = 0x5F;
    const uint8_t res12bit = 0x7F;
    enum class timing { RESETLOW = 480, RESETREL = 70, RESETDELAY = 410, READLOW = 6, READREL = 9, READDELAY = 55,
                        WRITE0LOW = 60, WRITE0REL = 10, WRITE1LOW = 6, WRITE1REL = 64 };
    
    inline void delayMicros(uint16_t);
    inline void pinLow();
    inline void pinRelease();
    inline uint8_t pinRead();
    void lowRelease(uint16_t, uint16_t);
    uint8_t CRC(uint8_t);
};

OneWire::OneWire(uint8_t pin) {
  pinBitmask = digitalPinToBitMask(pin);
  pinBit = digitalPinToBitPosition(pin);
  portID = digitalPinToPort(pin);
}

void OneWire::setup(void) {

  if (portID == NOT_A_PORT) return;
  portOW[portID]->DIRCLR = pinBitmask;
  
  TCB0.CNT = 0;
  TCB0.CCMP = 0xFFFF;

  // Enable timer with CLK_PER/2 as source in periodic interrupt mode
  TCB0.CTRLB = TCB_CNTMODE_INT_gc;
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;
  //TCB0.INTCTRL = TCB_CAPT_bm;
}

inline void OneWire::delayMicros(uint16_t micro) {
  const uint16_t factor = F_CPU / 2000000;
  
  TCB0.CCMP = (micro - 4) * factor;    // F_PER / 2, substract 4µs for register setup time
  TCB0.CNT = 0;
  TCB0.INTFLAGS = TCB_CAPT_bm;
  while (!(TCB0.INTFLAGS & TCB_CAPT_bm));
}

inline void OneWire::pinLow() {
  portOW[portID]->DIRSET = pinBitmask;
  portOW[portID]->OUTCLR = pinBitmask;
}

inline void OneWire::pinRelease() {
  portOW[portID]->DIRCLR = pinBitmask;
}

inline uint8_t OneWire::pinRead () {
  return (portOW[portID]->IN & pinBitmask) ? 1 : 0;
}

void OneWire::lowRelease(uint16_t low, uint16_t high) {
  pinLow();
  delayMicros(low);
  pinRelease();
  delayMicros(high);
}

// bus reset, returns 0 if device is present
uint8_t OneWire::reset() {
  uint8_t data = 1;

  lowRelease((uint16_t)timing::RESETLOW, (uint8_t)timing::RESETREL);
  data = pinRead();
  delayMicros((uint16_t)timing::RESETDELAY);
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
    delayMicros((uint16_t)timing::READDELAY);
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
    write(SkipROM);
    write(ConvertT);
  }
  return true;
}

#define TEMP_ERROR -1000

// read temperature of a single DS18B20 on the bus
// returns degrees multiplied by 100 (integer only)
int16_t OneWire::readTemperature() {
  int16_t rawTemp;
  
  if (reset() != 0) {
    return TEMP_ERROR;
  } else {
    write(SkipROM);
    write(ReadScratchpad);
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
    write(SkipROM);
    write(WriteScratchpad);
    write(0);
    write(100);
    write(res);
    reset();
    return true;
  }
  return false;
}
