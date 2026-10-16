// DCF77 receiving object - interface to all DCF-functions and interrupt-handler

#include "Ports.h"

const uint16_t timerFreq = 200;                                 // 1 / 0.005s
const uint16_t bit0MinDuration = (timerFreq * 2) / 100;         // 20 ms
const uint16_t bit0DurationLow = (timerFreq * 7) / 100;         // 70 ms
const uint16_t bit0DurationHigh = (timerFreq * 15) / 100;       // 150 ms
const uint16_t bit1DurationLow = (timerFreq * 17) / 100;        // 170 ms
const uint16_t bit1DurationHigh = (timerFreq * 25) / 100;       // 250 ms
const uint16_t pauseDurationLow = (timerFreq * 150) / 100;      // 1500 ms
const uint16_t pauseDurationHigh = (timerFreq * 220) / 100;     // 2200 ms

#define DCF_SIZE          59
#define NOSIGNAL_COUNTER  10

struct timeStampDCF77
{
  // raw DCF77 values are always in two digits
  uint8_t minute;
  uint8_t hour;
  uint8_t day;
  uint8_t weekday;
  uint8_t month;
  uint8_t year;
  uint8_t A1; // change from CET to CEST or vice-versa.
  uint8_t CEST;
  uint8_t CET;
  int8_t transmitter_fault;  // only relevant with very good signal
};

class dcf77 {
  public:
    enum class result { SUCCESS = 0, INVALID = -1 };
    enum class dcfState { IDLE = 1, DETECT, STARTBIT, STARTBITCOMPL, MINUTEMARKER, RECEIVING };
    enum class dcfPulse { NONE = 1, PAUSE, BIT0, BIT1, INVALID };
    enum class dcfPulseType { START = 1, END };
    enum class dcfSignal { WAIT = 1, GOOD, NOSIGNAL };
    bool pulseReceivedAnim, noSignalMarker;
    uint8_t pinBitmaskDcf, pinBitmaskLed, portIdLed, noSignalCounter;
    volatile uint16_t timerTicks;

    dcf77();
    void begin(uint8_t, uint8_t);
    void handle();
    void timerSetup();
    void request();
    bool getRequestState();
    dcf77::dcfState getState();
    uint8_t getPos();
    bool getBit(uint8_t);
    dcf77::dcfPulse getLastPulse();
    dcf77::dcfPulseType getLastPulseType();
    uint16_t getLastPulseLength();
    bool checkSignal();
    bool checkPulseReceived();
    bool checkComplete();
    bool checkStartBit();
    bool checkMinuteMarker();
    bool checkReceiveBit();
    bool checkRestart();
    dcf77::dcfSignal getSignalStatus();
    dcf77::result decode(timeStampDCF77 *);
    void handleInt(bool);

  private:
    bool dcfReq, minuteMarker, restartMarker, startBit, receiveBit, dcfComplete;
    uint8_t noSignalCnt, validPulses;
    uint8_t bitArray[DCF_SIZE], dcfPos;
    dcf77::dcfState state;
    dcf77::dcfSignal signalStatus;

    dcf77::result checkParity();
    uint8_t bitScale(uint8_t *, uint8_t);

    volatile bool pulseReceivedDcf;
    volatile uint16_t lastPulseLength;
    volatile dcf77::dcfPulse lastPulse;
    volatile dcf77::dcfPulseType lastPulseType;
};

// constructor
dcf77::dcf77() {
  state = dcfState::IDLE;
  dcfPos = 0;
  dcfReq = false;
  dcfComplete = false;
  minuteMarker = false;
  restartMarker = false;
  startBit = false;
  signalStatus = dcfSignal::WAIT;
  noSignalCounter = 0;
  validPulses = 0;
  lastPulse = dcfPulse::NONE;
  lastPulseType = dcfPulseType::START;
}

void dcf77::begin(uint8_t pin1, uint8_t pin2) {
  pinBitmaskDcf = digitalPinToBitMask(pin1);
  pinBitmaskLed = digitalPinToBitMask(pin2);
  portIdLed = digitalPinToPort(pin2);

  // DCF signal input with pullup
  register8_t *pinCtrlReg = &PORTA.PIN0CTRL + digitalPinToBitPosition(pin1);
  *pinCtrlReg = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;

#ifdef LED
  port[portIdLed]->DIRSET = pinBitmaskLed;
  port[portIdLed]->OUTCLR = pinBitmaskLed;
#endif
  timerSetup();
}

void dcf77::handle(void) {

#ifdef SERIALDEBUG
  char buffer[40];
#endif
  if (checkSignal() == false) {
    state = dcfState::IDLE;
    return;
  }
  if (pulseReceivedDcf == false) return;
  pulseReceivedDcf = false;
  pulseReceivedAnim = true;

  switch (state) {
    case dcfState::IDLE:
      if (dcfReq) {
        dcfComplete = false;
        validPulses = 0;
        dcfPos = 0;
        signalStatus = dcfSignal::WAIT;
        state = dcfState::DETECT;
      }
      break;

    case dcfState::DETECT:
      // wait for first five valid pulses
      if (lastPulseType == dcfPulseType::END) {
        if (lastPulse == dcfPulse::BIT0 || lastPulse == dcfPulse::BIT1) {
          // valid pulse detected
#ifdef SERIALDEBUG
          Serial.println("DCF: Valid pulse detected");
#endif
          noSignalCnt = 0;
          if(++validPulses == 5) {
#ifdef SERIALDEBUG
            Serial.println("DCF: Wait for minute marker");
#endif
            signalStatus = dcfSignal::GOOD;
            state = dcfState::MINUTEMARKER;
          }
        }
      }
      break;

    case dcfState::MINUTEMARKER:
      if (lastPulseType == dcfPulseType::START) {
#ifdef SERIALDEBUG
        sprintf(buffer, "DCF: lastPulseLength Start %u", lastPulseLength);
        Serial.println(buffer);
#endif
        if (lastPulse == dcfPulse::PAUSE) {
          // minute marker detected
#ifdef SERIALDEBUG
          Serial.println("DCF: Minute marker detected");
#endif
          minuteMarker = true;
          state = dcfState::STARTBIT;
        }
      }
#ifdef SERIALDEBUG
      if (lastPulseType == dcfPulseType::END) {
        sprintf(buffer, "DCF: lastPulseLength End   %u", lastPulseLength);
        Serial.println(buffer);
      }
#endif
      break;

    case dcfState::STARTBIT:
      if (lastPulseType == dcfPulseType::END) {
        if (lastPulse == dcfPulse::BIT0) {
#ifdef SERIALDEBUG
          Serial.println("DCF: Startbit detected, receiving now...");
#endif
          bitArray[0] = 0;
          dcfPos = 0;
          startBit = true;
          state = dcfState::RECEIVING;
        }
        if (lastPulse == dcfPulse::INVALID) {
          // receive signal error
#ifdef SERIALDEBUG
          Serial.println("\r\nDCF: Receive signal error");
#endif
          restartMarker = true;
          state = dcfState::IDLE;
        }
      }
      break;

    case dcfState::STARTBITCOMPL:
//    if (lastPulseType == dcfPulseType::END) {
//      if (lastPulse == dcfPulse::BIT0) {
      if (lastPulseType == dcfPulseType::START) {
        if (lastPulse == dcfPulse::PAUSE) {

#ifdef SERIALDEBUG
          Serial.println("\r\nDCF: Receive complete");
#endif
          dcfComplete = true;
          dcfReq = false;
          state = dcfState::IDLE;
        }
        else {
          restartMarker = true;
          state = dcfState::IDLE;
        }
      }
      break;

    case dcfState::RECEIVING:
      if (lastPulseType == dcfPulseType::END) {
        if (lastPulse == dcfPulse::BIT0 || lastPulse == dcfPulse::BIT1) {
          dcfPos++;
          bitArray[dcfPos] = (lastPulse == dcfPulse::BIT0) ? 0 : 1;
          receiveBit = true;
#ifdef SERIALDEBUG
          if (bitArray[dcfPos]) Serial.print("1"); else Serial.print("0");
#endif
        }
        if (lastPulse == dcfPulse::INVALID) {
#ifdef SERIALDEBUG
          Serial.println("\r\nDCF: Invalid pulse detected, restart");
#endif          
          // receive signal error
          restartMarker = true;
          state = dcfState::IDLE;
        }
        // finally wait for next startbit to complete
        if (dcfPos == DCF_SIZE - 1) state = dcfState::STARTBITCOMPL;
      }
      break;
  }
  return;
}

void dcf77::timerSetup(void) {
  cli();
  // use TCB0 in periodic interrupt mode (5 ms)
  TCB0.CTRLA = 0x00;
  TCB0.CTRLB = TCB_CNTMODE_INT_gc;
  TCB0.CCMP = (F_CPU / 400) - 1;
  TCB0.INTCTRL = TCB_CAPT_bm;
    
  // start timer with CLK_PER/2
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;
  sei();
}

void dcf77::request(void) {
  dcfPos = 0;
  dcfReq = true;
  dcfComplete = false;
  minuteMarker = false;
  startBit = false;
  receiveBit = false;
  restartMarker = false;

  // clear receive data buffer
  //for (uint8_t i = 0; i < DCF_SIZE; i++) bitArray[i] = 0;
}

bool dcf77::getRequestState(void) {
  return dcfReq;
}

dcf77::dcfState dcf77::getState(void) {
  return state;
}

uint8_t dcf77::getPos(void) {
  return dcfPos;
}

bool dcf77::getBit(uint8_t pos) {
  return bitArray[pos] ? true : false;
}

dcf77::dcfPulse dcf77::getLastPulse(void) {
  return lastPulse;
}

dcf77::dcfPulseType dcf77::getLastPulseType(void) {
  return lastPulseType;
}

uint16_t dcf77::getLastPulseLength(void) {
  return lastPulseLength;
}


dcf77::dcfSignal dcf77::getSignalStatus(void) {
  return signalStatus;
}

bool dcf77::checkSignal(void) {
  if (noSignalCounter >= NOSIGNAL_COUNTER) {
#ifdef SERIALDEBUG
    Serial.println("DCF: No signal");
#endif
    signalStatus = dcfSignal::NOSIGNAL;
    restartMarker = true;
    noSignalCounter = 0;
    return false;
  }
  if (timerTicks > pauseDurationHigh) {
#ifdef SERIALDEBUG
    Serial.println("DCF: timerTicks > pauseDurationHigh");
#endif
    timerTicks = 0;
    noSignalCounter++;
  }
  return true;
}

bool dcf77::checkPulseReceived(void) {
  bool pr = pulseReceivedAnim;
  pulseReceivedAnim = false;
  return pr;
}

bool dcf77::checkComplete(void) {
  return dcfComplete;
}

bool dcf77::checkStartBit(void) {
  bool sb = startBit;
  startBit = false;
  return sb;
}

bool dcf77::checkMinuteMarker(void) {
  bool mm = minuteMarker;
  minuteMarker = false;
  return mm;
}

bool dcf77::checkReceiveBit(void) {
  bool rb = receiveBit;
  receiveBit = false;
  return rb;
}

bool dcf77::checkRestart(void) {
  bool rs = restartMarker;
  restartMarker = false;
  return rs;
}

uint8_t dcf77::bitScale(uint8_t *bitstring, uint8_t len) {
  static const uint8_t weights[] = {1, 2, 4, 8, 10, 20, 40, 80};
  static const uint8_t weightsLen = 8;
  uint8_t value = 0;

  for (uint8_t i = 0; i < len && i < weightsLen; i++) value += weights[i] * bitstring[i];

  return value;
}

dcf77::result dcf77::checkParity(void) {
  uint8_t minuteParity = 0;
  uint8_t hourParity = 0;
  uint8_t dateParity = 0;

  // calculate parity for minute
  for (uint8_t i = 21; i < 28; ++i) minuteParity ^= bitArray[i];

  // calculate parity for hour
  for (uint8_t i = 29; i < 35; ++i) hourParity ^= bitArray[i];

  // calculate parity for date
  for (uint8_t i = 36; i < 58; ++i) dateParity ^= bitArray[i];

  // check the parity bits for minutes and hours - DCF77 uses even parity
  if ((minuteParity != bitArray[28]) || (hourParity != bitArray[35]) || (dateParity != bitArray[58]))
    return result::INVALID; // Parity error

  return result::SUCCESS;
}

// extracts and interprets the date and time from the binary DCF77 string and writes them into a timeStampDCF77 structure
dcf77::result dcf77::decode(timeStampDCF77 *dcf) {
  // decode the bit strings according to the DCF77 specification
  dcf->hour = bitScale(bitArray + 29, 6);
  dcf->minute = bitScale(bitArray + 21, 7);
  dcf->day = bitScale(bitArray + 36, 6);
  dcf->weekday = bitScale(bitArray + 42, 3);
  dcf->month = bitScale(bitArray + 45, 5);
  dcf->year = bitScale(bitArray + 50, 8);
  dcf->transmitter_fault = bitScale(bitArray + 15, 1);
  dcf->A1 = bitScale(bitArray + 16, 1);
  dcf->CEST = bitScale(bitArray + 17, 1);
  dcf->CET = bitScale(bitArray + 18, 1);

  if (checkParity() == result::INVALID) {
#ifdef SERIALDEBUG
    Serial.println("Decode: Parity error in hour or minute.");
#endif
    return result::INVALID;
  }

  // check if day, month, or year have invalid (00) values
  if (dcf->day == 0 || dcf->month == 0 || dcf->year == 0 || (dcf->CEST == dcf->CET)) {
#ifdef SERIALDEBUG
    Serial.println("Decode: Invalid date received.");
#endif
    return result::INVALID;
  }
  return result::SUCCESS;
}

// interrupt handler for pin change interrupt / signal detection
void dcf77::handleInt(bool signalLevel) {

  uint16_t pulseLength = timerTicks;

  timerTicks = 0;

  // filter noise
  if (pulseLength < bit0MinDuration) return;

#ifdef LED
  if (signalLevel)
    port[portIdLed]->OUTCLR = pinBitmaskLed;
  else {
    if (getRequestState()) port[portIdLed]->OUTSET = pinBitmaskLed;
  }
#endif

  lastPulse = dcfPulse::NONE;
  if (signalLevel == false) {
    // end of pause
    lastPulseType = dcfPulseType::START;
    if (pulseLength >= pauseDurationLow && pulseLength <= pauseDurationHigh) lastPulse = dcfPulse::PAUSE;
  }
  else {
    // end of pulse
    lastPulseType = dcfPulseType::END;
    if (pulseLength >= bit0DurationLow && pulseLength <= bit0DurationHigh) lastPulse = dcfPulse::BIT0;
    if (pulseLength >= bit1DurationLow && pulseLength <= bit1DurationHigh) lastPulse = dcfPulse::BIT1;
    if (pulseLength < bit0DurationLow || pulseLength > bit1DurationHigh) lastPulse = dcfPulse::INVALID;
  }
  lastPulseLength = pulseLength;
  pulseReceivedDcf = true;
  return;
}
