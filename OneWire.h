#include <avr/io.h>
#include <avr/interrupt.h>

// device resolution
#define TEMP_9_BIT    0x1F    //  9 bit
#define TEMP_10_BIT   0x3F    // 10 bit
#define TEMP_11_BIT   0x5F    // 11 bit
#define TEMP_12_BIT   0x7F    // 12 bit

//--------------- One Wire Protocol -----------------

// buffer to read data or ROM code
static union {
  uint8_t dataBytes[9];
  uint16_t dataWords[4];
};

uint8_t oneWirePin;
const uint8_t ReadROM = 0x33;
const uint8_t MatchROM = 0x55;
const uint8_t SkipROM = 0xCC;
const uint8_t ConvertT = 0x44;
const uint8_t WriteScratchpad = 0x4E;
const uint8_t ReadScratchpad = 0xBE;

void oneWireSetup(uint8_t pin) {

  oneWirePin = pin;
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

void delayMicros(uint16_t micro) {
  TCB0.CCMP = micro * 4;    // F_PER / 2 = 4 MHz
  TCB0.CNT = 0;
  TCB0.INTFLAGS = TCB_CAPT_bm;
  while (!(TCB0.INTFLAGS & TCB_CAPT_bm));

  //TCNT1 = 0; TIFR = 1<<OCF1A;
  //OCR1A = (micro>>1) - 1;
  //while ((TIFR & 1<<OCF1A) == 0);
}

inline void pinLow() {
  PORTA.DIRSET = 1 << oneWirePin;
  PORTA.OUTCLR = 1 << oneWirePin;
}

inline void pinRelease() {
  PORTA.DIRCLR = 1 << oneWirePin;
}

// Returns 0 or 1
inline uint8_t pinRead () {
  return (PORTA.IN >> oneWirePin) & 1;
}

void lowRelease(uint16_t low, uint16_t high) {
  pinLow();
  delayMicros(low);
  pinRelease();
  delayMicros(high);
}

uint8_t oneWireReset() {
  uint8_t data = 1;

  lowRelease(480, 70);
  data = pinRead();
  delayMicros(410);
  return data;   // 0 = device present
}

void oneWireWrite(uint8_t data) {
  int del;
  for (uint8_t i = 0; i < 8; i++) {
    if ((data & 1) == 1) del = 6; else del = 60;
    lowRelease(del, 70 - del);
    data = data >> 1;
  }
}

uint8_t oneWireRead() {
  uint8_t data = 0;
  for (uint8_t i = 0; i < 8; i++) {
    lowRelease(6, 9);
    data = data | pinRead() << i;
    delayMicros(55);
  }
  return data;
}

// Read bytes into array, least significant byte first
void oneWireReadBytes(uint8_t bytes) {
  for (uint8_t i = 0; i < bytes; i++) {
    dataBytes[i] = oneWireRead();
  }
}

// Calculate CRC over buffer - 0x00 is correct
uint8_t oneWireCRC(uint8_t bytes) {
  uint8_t crc = 0;
  for (uint8_t j = 0; j < bytes; j++) {
    crc = crc ^ dataBytes[j];
    for (uint8_t i = 0; i < 8; i++) crc = crc >> 1 ^ ((crc & 1) ? 0x8c : 0);
  }
  return crc;
}

// start conversion
bool startConversion() {
  if (oneWireReset() != 0) {
    return false;
  } else {
    oneWireWrite(SkipROM);
    oneWireWrite(ConvertT);
  }
  return true;
}

// Read temperature of a single DS18B20 or MAX31820 on the bus
int16_t readTemperature() {
  if (oneWireReset() != 0) {
    return -99;
  } else {
    oneWireWrite(SkipROM);
    oneWireWrite(ReadScratchpad);
    oneWireReadBytes(9);
    if (oneWireCRC(9) == 0) return dataWords[0];
  }
  return -99;
}

bool setResolution(uint8_t resolution) {
  uint8_t res;
  
  switch(resolution) {
    case 12: res = TEMP_12_BIT; break;
    case 11: res = TEMP_11_BIT; break;
    case 10: res = TEMP_10_BIT; break;
    default: res = TEMP_9_BIT;   break;
  }

  if (oneWireReset() != 0) {
    //_oneWire->select(_deviceAddress);
    oneWireWrite(SkipROM);
    oneWireWrite(WriteScratchpad);
    oneWireWrite(0);
    oneWireWrite(100);
    oneWireWrite(res);
    oneWireReset();
    return true;
  }
  return false;
}
