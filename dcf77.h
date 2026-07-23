// DCF77 receiving object - interface to all DCF-functions and interrupt-handler

#define TIMER_FREQ              7812  // 8 Mhz / 1024 = 7812
#define TIMER_CMPMATCH          15625 // timer ticks (2000 ms)
#define TIMER_TOP               32767

#define BIT_0_MIN_DURATION      156   // timer ticks (20 ms)
#define BIT_0_DURATION_LOW      625   // timer ticks (80 ms)
#define BIT_0_DURATION_HIGH     938   // timer ticks (120 ms)
#define BIT_1_DURATION_LOW      1406  // timer ticks (180 ms)
#define BIT_1_DURATION_HIGH     1719  // timer ticks (220 ms)
#define TIMEOUT_DURATION_LOW    12000 // timer ticks (1700 ms)
#define TIMEOUT_DURATION_HIGH   16000 // timer ticks (1900 ms)
//#define TIMEOUT_DURATION_LOW  13280 // timer ticks (1700 ms)
//#define TIMEOUT_DURATION_HIGH 14843 // timer ticks (1900 ms)

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
    enum class dcfState { IDLE = 1, DETECT, STARTBIT, MINUTEMARKER, MINUTEMARKERCOMPL, RECEIVING };
    enum class pulseType { NONE = 0, START = 1, END = 2 };

    dcf77();
    void timerSetup();
    void request();
    bool getRequestState();
    dcf77::dcfState getState();
    uint8_t getPos();
    bool getBit(uint8_t);
    dcf77::pulseType getLastPulseType();
    uint16_t getLastPulseLen();
    bool checkComplete();
    bool checkStartBit();
    bool checkMinuteMarker();
    bool checkRestart();
    void noSignal();
    bool getSignalStatus();
    dcf77::result decode(timeStampDCF77 *);
    void handleInt(dcf77::pulseType, uint16_t);

  private:
    uint8_t bitArray[DCF_SIZE], dcfPos, noSignalCnt;
    uint16_t lastPulseLen;
    dcf77::dcfState state;
    dcf77::pulseType lastPulseType;

    dcf77::result checkParity();
    uint8_t bitScale(uint8_t *, uint8_t);

    volatile bool dcfReq, minuteMarker, restartMarker, startBit, receiveBit, dcfComplete;
};

// constructor
dcf77::dcf77()
{
  state = dcfState::IDLE;
  dcfPos = 0;
  dcfReq = false;
  dcfComplete = false;
  minuteMarker = false;
  restartMarker = false;
  startBit = false;
  noSignalCnt = 0;
  lastPulseType = pulseType::NONE;
}

void dcf77::timerSetup(void)
{
  cli();
  TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc;
  TCA0.SINGLE.CTRLD = 0;
  TCA0.SINGLE.CTRLECLR = TCA_SINGLE_DIR_bm;
  TCA0.SINGLE.CMP0 = TIMER_CMPMATCH;
  TCA0.SINGLE.PER = TIMER_TOP;
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP0EN_bm;
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1024_gc | TCA_SINGLE_ENABLE_bm;
  sei();
}

void dcf77::request(void) {
  dcfPos = 0;
  dcfReq = true;
  dcfComplete = false;
  noSignalCnt = 0;

  // clear receive data buffer
  for (uint8_t i = 0; i < DCF_SIZE; i++) bitArray[i] = 0;
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

dcf77::pulseType dcf77::getLastPulseType(void) {
  return lastPulseType;
}

uint16_t dcf77::getLastPulseLen(void) {
  return lastPulseLen;
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

bool dcf77::checkRestart(void) {
  bool rs = restartMarker;
  restartMarker = false;
  return rs;
}

void dcf77::noSignal(void) {
  if (noSignalCnt < NOSIGNAL_COUNTER) noSignalCnt++;

  // receive signal error
  if (state == dcfState::RECEIVING) {
    state = dcfState::DETECT;
    dcfPos = 0;
  }
  return;
}

bool dcf77::getSignalStatus(void) {
  if (noSignalCnt >= NOSIGNAL_COUNTER) return false;
  return true;
}

uint8_t dcf77::bitScale(uint8_t *bitstring, uint8_t len)
{
  static const uint8_t weights[] = {1, 2, 4, 8, 10, 20, 40, 80};
  static const uint8_t weights_len = 8;
  uint8_t value = 0;

  for (uint8_t i = 0; i < len && i < weights_len; i++) value += weights[i] * bitstring[i];

  return value;
}

dcf77::result dcf77::checkParity(void)
{
  //DCF77 uses even parity
  uint8_t minuteParity = 0;
  uint8_t hourParity = 0;
  uint8_t dateParity = 0;

  // Calculate parity for minute
  for (uint8_t i = 21; i < 28; ++i) minuteParity ^= bitArray[i];

  // Calculate parity for hour
  for (uint8_t i = 29; i < 35; ++i) hourParity ^= bitArray[i];

  // Calculate parity for date
  for (uint8_t i = 36; i < 58; ++i) dateParity ^= bitArray[i];

  // Check the parity bits for minutes and hours
  if ((minuteParity != bitArray[28]) || (hourParity != bitArray[35]) || (dateParity != bitArray[58]))
    return result::INVALID; // Parity error

  return result::SUCCESS;
}

// Extracts and interprets the date and time from the binary DCF77 string and writes them into a timeStampDCF77 structure.
dcf77::result dcf77::decode(timeStampDCF77 *dcf)
{
  // Decode the bit strings according to the DCF77 specification
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
    Serial.println("\r\nParity error in hour or minute.");
#endif
    return result::INVALID;
  }

  // Check if day, month, or year have invalid (00) values
  if (dcf->day == 0 || dcf->month == 0 || dcf->year == 0 || (dcf->CEST == dcf->CET)) {
#ifdef SERIALDEBUG
    Serial.println("\r\nInvalid date received.");
#endif
    return result::INVALID; // The date is not plausible
  }
  return result::SUCCESS;
}

void dcf77::handleInt(dcf77::pulseType type, uint16_t lenSignal)
{
  lastPulseType = type;
  lastPulseLen = lenSignal;
  noSignalCnt = 0;

  switch (state) {
    case dcfState::IDLE:
      if (dcfReq) {
        dcfComplete = false;
        state = dcfState::DETECT;
      }
      break;

    case dcfState::DETECT:
      // wait for first valid pulse
      if (type == pulseType::END) {
        if ((lenSignal >= BIT_0_DURATION_LOW && lenSignal <= BIT_0_DURATION_HIGH) ||
            (lenSignal >= BIT_1_DURATION_LOW && lenSignal <= BIT_1_DURATION_HIGH)) {
          // first valid pulse detected
          state = dcfState::MINUTEMARKER;
        }
      }
      break;

    case dcfState::MINUTEMARKER:
      if (type == pulseType::START) {
        if (lenSignal >= TIMEOUT_DURATION_LOW && lenSignal <= TIMEOUT_DURATION_HIGH) {
          // minute marker detected
          minuteMarker = true;
          dcfPos = 0;
          state = dcfState::STARTBIT;
        }
      }
      break;

    case dcfState::MINUTEMARKERCOMPL:
      if (type == pulseType::START) {
        if (lenSignal >= TIMEOUT_DURATION_LOW && lenSignal <= TIMEOUT_DURATION_HIGH) {
          dcfComplete = true;
          dcfReq = false;
          state = dcfState::IDLE;
        }
        else {
          dcfPos = 0;
          restartMarker = true;
          state = dcfState::MINUTEMARKER;          
        }
      }
      break;
    
    case dcfState::STARTBIT:
      if (type == pulseType::END) {
        if (lenSignal >= BIT_0_DURATION_LOW && lenSignal <= BIT_0_DURATION_HIGH) {
          bitArray[0] = 0;
          startBit = true;
          dcfPos = 1;
          state = dcfState::RECEIVING;
        }
        if (lenSignal < BIT_0_DURATION_LOW || lenSignal > BIT_1_DURATION_HIGH) {
          // receive signal error
          dcfPos = 0;
          state = dcfState::MINUTEMARKER;
        }
      }
      break;

    case dcfState::RECEIVING:
      if (type == pulseType::END) {
        if (lenSignal >= BIT_0_DURATION_LOW && lenSignal <= BIT_0_DURATION_HIGH) {
          bitArray[dcfPos] = 0;
          receiveBit = true;
        }
        if (lenSignal >= BIT_1_DURATION_LOW && lenSignal <= BIT_1_DURATION_HIGH) {
          bitArray[dcfPos] = 1;
          receiveBit = true;
        }
        if (lenSignal < BIT_0_DURATION_LOW || lenSignal > BIT_1_DURATION_HIGH) {
          // receive signal error
          dcfPos = 0;
          restartMarker = true;
          state = dcfState::MINUTEMARKER;
        }
        // finally weit for next minute marker to complete
        if (++dcfPos == DCF_SIZE) state = dcfState::MINUTEMARKERCOMPL;
      }
      break;
  }
}
