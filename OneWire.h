// OneWire object, includes DS18B20 temperature reading

class OneWire {
  public:
    enum class resolution { RES9BIT = 0x1F, RES10BIT = 0x3F, RES11BIT = 0x5F, RES12BIT = 0x7F };
    
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
    uint8_t oneWirePin;
    uint8_t dataBytes[9];
    const uint8_t ReadROM = 0x33;
    const uint8_t MatchROM = 0x55;
    const uint8_t SkipROM = 0xCC;
    const uint8_t ConvertT = 0x44;
    const uint8_t WriteScratchpad = 0x4E;
    const uint8_t ReadScratchpad = 0xBE;
    
    void delayMicros(uint16_t);
    inline void pinLow();
    inline void pinRelease();
    inline uint8_t pinRead();
    void lowRelease(uint16_t, uint16_t);
    uint8_t CRC(uint8_t);
};

OneWire::OneWire(uint8_t pin) {
  oneWirePin = pin;
}

void OneWire::setup(void) {

  PORTA.DIRCLR = 1 << oneWirePin;
  
  TCB0.CNT = 0;
  TCB0.CCMP = 0xFFFF;

  // Enable the timer with CLK_PER/2 as source in periodic interrupt mode
  TCB0.CTRLB = TCB_CNTMODE_INT_gc;
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;
  //TCB0.INTCTRL = TCB_CAPT_bm;

  //TCCR1 = 0<<CTC1 | 0<<PWM1A | 5<<CS10;  // CTC mode, 500kHz clock
  //GTCCR = 0<<PWM1B;
}

void OneWire::delayMicros(uint16_t micro) {
  TCB0.CCMP = micro * 4;    // F_PER / 2 = 4 MHz
  TCB0.CNT = 0;
  TCB0.INTFLAGS = TCB_CAPT_bm;
  while (!(TCB0.INTFLAGS & TCB_CAPT_bm));

  //TCNT1 = 0; TIFR = 1<<OCF1A;
  //OCR1A = (micro>>1) - 1;
  //while ((TIFR & 1<<OCF1A) == 0);
}

inline void OneWire::pinLow() {
  PORTA.DIRSET = 1 << oneWirePin;
  PORTA.OUTCLR = 1 << oneWirePin;
}

inline void OneWire::pinRelease() {
  PORTA.DIRCLR = 1 << oneWirePin;
}

// Returns 0 or 1
inline uint8_t OneWire::pinRead () {
  return (PORTA.IN >> oneWirePin) & 1;
}

void OneWire::lowRelease(uint16_t low, uint16_t high) {
  pinLow();
  delayMicros(low);
  pinRelease();
  delayMicros(high);
}

uint8_t OneWire::reset() {
  uint8_t data = 1;

  lowRelease(480, 70);
  data = pinRead();
  delayMicros(410);
  return data;   // 0 = device present
}

void OneWire::write(uint8_t data) {
  int del;
  for (uint8_t i = 0; i < 8; i++) {
    if ((data & 1) == 1) del = 6; else del = 60;
    lowRelease(del, 70 - del);
    data = data >> 1;
  }
}

uint8_t OneWire::read() {
  uint8_t data = 0;
  for (uint8_t i = 0; i < 8; i++) {
    lowRelease(6, 9);
    data = data | pinRead() << i;
    delayMicros(55);
  }
  return data;
}

// Read bytes into array, least significant byte first
void OneWire::readBytes(uint8_t bytes) {
  for (uint8_t i = 0; i < bytes; i++) {
    dataBytes[i] = read();
  }
}

// Calculate CRC over buffer - 0x00 is correct
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

// Read temperature of a single DS18B20 on the bus
// Returns degrees multiplied by 100 (integer only)
int16_t OneWire::readTemperature() {
  int16_t rawTemp;
  
  if (reset() != 0) {
    return -100;
  } else {
    write(SkipROM);
    write(ReadScratchpad);
    readBytes(9);
    if (CRC(9) == 0) {
      rawTemp = (((int16_t)dataBytes[1]) << 8) | dataBytes[0];
      return (rawTemp * 25 + 2) / 4;
    }
  }
  return -100;
}

bool OneWire::setResolution(uint8_t res) {
  OneWire::resolution res2;
  
  switch(res) {
    case 12: res2 = resolution::RES12BIT; break;
    case 11: res2 = resolution::RES11BIT; break;
    case 10: res2 = resolution::RES10BIT; break;
    default: res2 = resolution::RES9BIT;  break;
  }

  if (reset() != 0) {
    write(SkipROM);
    write(WriteScratchpad);
    write(0);
    write(100);
    write((uint8_t)res2);
    reset();
    return true;
  }
  return false;
}
