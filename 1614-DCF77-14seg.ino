/*******************************************************************************/
/*
  DCF77-7seg.ino

  This program receives the DCF77 time signal and syncs it with the external
  RTC-clock. It is displayed on a 8-digit 7-segment display with HT16K33
  controller.
  The MCU used is an ATtiny 814 or 1614. Without serial debugging and OneWire,
  an Attiny 414 oder 412 (with minor modifications) can be used as well.
  Optionally the OneWire part can be included for an additional temperature
  display.

  MCU-clock: 8 MHz
  Timers used: TCA0 (DCF signal measurement), TCB0 for OneWire implementation,
               TCD0 (millis), RTC (2 Hz generation via interrupt)
  External RTC: DS3231 with battery backup, supplies 32K clock for internal RTC

  Author: K. Wolf
  Date: Apr 17th 2026
*/
/*******************************************************************************/

//#define TINY_412
#define TINY_1614

#ifdef TINY_412
#include <TinyI2CMaster.h>
#include "DateTime.h"
#define TINYWIRE
#endif
#ifdef TINY_1614
#include <Wire.h>
#include <RTClib.h>
#include "OneWire.h"
#define RTC_AVAIL
#endif
#define SEG14
#include "AlphaDisplay.h"

//#define SERIALDEBUG
#define ONEWIRE
#define LED

#define SUCCESS               0
#define ERROR_INVALID_VALUE  -1
#define NO_SENSOR            -99
#define DCF77_SIZE            59

#define TIMER_FREQ            7812  // 8 Mhz / 1024 = 7812
#define TIMER_CMPMATCH        15625 // timer ticks (2000 ms)
#define TIMER_TOP             32767
#define BIT_0_MIN_DURATION    156   // timer ticks (20 ms)
#define BIT_0_DURATION_LOW    625   // timer ticks (80 ms)
#define BIT_0_DURATION_HIGH   938   // timer ticks (120 ms)
#define BIT_1_DURATION_LOW    1406  // timer ticks (180 ms)
#define BIT_1_DURATION_HIGH   1719  // timer ticks (220 ms)
//#define TIMEOUT_DURATION_LOW  13280 // timer ticks (1700 ms)
//#define TIMEOUT_DURATION_HIGH 14843 // timer ticks (1900 ms)
#define TIMEOUT_DURATION_LOW  12000 // timer ticks (1700 ms)
#define TIMEOUT_DURATION_HIGH 16000 // timer ticks (1900 ms)

// ----------------- 7-segment digit definitions -----------------------------------

#define LINE_RIGHTUP    0x02
#define LINE_RIGHTDOWN  0x04
#define LINE_TOP        0x01
#define LINE_BOTTOM     0x08
#define LINE_LEFTUP     0x20
#define LINE_LEFTDOWN   0x10
#define LINE_MIDDLE     0x40
#define COLON           0x80

#define HT16K33_BLINK_CMD       0x80
#define HT16K33_BLINK_DISPLAYON 0x01
#define HT16K33_BLINK_OFF       0
#define HT16K33_BLINK_2HZ       1
#define HT16K33_BLINK_1HZ       2
#define HT16K33_BLINK_HALFHZ    3

#define DISPLAY_ADDRESS         0x70
#define DISPLAY_DIGITS          8
#define DISPLAY_BRIGHTNESS      4

#define ONEWIRE_PIN             4
#define ONEWIRE_RESOLUTION      11

#ifdef MILLIS_USE_TIMERA0
#error "This sketch takes over TCA0 - please use a different timer for millis"
#endif

// ----------------- hardware definitions ------------------------------------------

#ifdef TINY_412
const uint8_t pinLed = PIN6_bm;
const uint8_t pinDcf = PIN3_bm;
const uint8_t pinButton = PIN7_bm;
#endif
#ifdef TINY_1614
const uint8_t pinLed = PIN5_bm;
const uint8_t pinDcf = PIN3_bm;
const uint8_t pinButton = PIN6_bm;
const uint8_t pinPullup = PIN7_bm;
#endif

const uint32_t resyncDelay = 10 * 60 * 1000L;
const uint32_t receiveTimeout = 5 * 60 * 1000L;
const uint32_t buttonDelay = 500;

struct TimeStampDCF77
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

uint8_t bitArray[DCF77_SIZE];  // memory location for received DCF77 bit string
TimeStampDCF77 dCF77time;             // data type for decoded DCF77 string

bool      prevtock, lastDCFDecode, syncReq, syncTimeout, syncStatus, sens;
uint8_t   timeState, showState, DCFstate, syncPos, prevPos, prevPulse, vcc[2];
int16_t   tempRaw;
uint32_t  currentTime, resyncTime, lastButtonTime, lastReceiveTime;

volatile bool minuteMarker, startBit, receiveBit, receiveComplete, ticktock, button;
volatile uint8_t receiveState, pulseType, DCFpos;
volatile uint16_t lengthPulse, lengthPause;

AlphaDisplay alpha;

DateTime dt(2026, 1, 1, 0, 0, 0);
#ifdef RTC_AVAIL
RTC_DS3231 rtc;
#endif

enum { TIME_NOTIME = 1, TIME_RTC, TIME_SYNC, TIME_SYNCED, TIME_RESYNC };
enum { DCF_RECEIVING = 1, DCF_RECEIVECOMPLETE, DCF_SYNCTIME };
enum { RECEIVE_MINUTEMARKER = 1, RECEIVE_DETECT, RECEIVE_STARTBIT, RECEIVE_RECEIVING };
enum { PULSE_START = 1, PULSE_END };
enum { SHOW_TIMEDATE = 1, SHOW_TIMEFULL, SHOW_TIMETEMP, SHOW_LOWBATT };
enum { SHOWSYNC_MINUTEMARKER = 1, SHOWSYNC_RECEIVING };

//----------------------------------------------------------------------------------

void setup() {
#ifdef SERIALDEBUG
  Serial.swap(1);         // use PA1(Tx) and PA2 (Rx)
  Serial.begin(115200);
  Serial.println("\r\nInit...");
#endif

  // DCF signal input with pullup
  PORTA.DIRCLR = pinDcf;
  PORTA.PIN3CTRL = PORT_PULLUPEN_bm;
  PORTA.PIN3CTRL |= PORT_ISC_BOTHEDGES_gc;

  // LED
  PORTA.DIRSET = pinLed;
  PORTA.OUTCLR = pinLed;

  // Button
  PORTA.DIRCLR = pinButton;
  PORTA.PIN6CTRL = PORT_PULLUPEN_bm;
  PORTA.PIN6CTRL |= PORT_ISC_FALLING_gc;

#ifdef TINY_1614
  // Pullup for 32K CLK-signal
  PORTA.DIRSET = pinPullup;
  PORTA.OUTSET = pinPullup;
#endif

#ifdef TINYWIRE
  TinyI2C.init();
#else
  Wire.begin();
#endif

  alpha.init(DISPLAY_ADDRESS, DISPLAY_DIGITS, DISPLAY_BRIGHTNESS);
  alpha.print("INIT");
  delay(1000);

  sens = false;
#ifdef ONEWIRE
  oneWireSetup(ONEWIRE_PIN);
  setResolution(ONEWIRE_RESOLUTION);
  if (startConversion()) {
    sens = true;
    delay(500);
    tempRaw = readTemperature();
    
    alpha.print("TEMP OK ");
    delay(1000);
  }
#endif
  
  timeState = TIME_NOTIME;
#ifdef RTC_AVAIL
  // initialize external RTC
  if (rtc.begin()) {
    if (rtc.lostPower()) {
      rtc.adjust(DateTime(2026, 1, 1, 12, 0, 0));
    }
    else {
      dt = rtc.now();
      timeState = TIME_RTC;
    }
    rtc.enable32K();

    alpha.print("RTC OK  ");
    delay(1000);
  }
#endif
  alpha.clear();
  
  RTCinit();
  ADCinit();
  delay(50);
  measureVoltage();
  setupDCF77();
  
  DCFstate = DCF_RECEIVING;
  showState = SHOW_TIMEDATE;
  receiveState = RECEIVE_DETECT;
  prevPulse = PULSE_END;
  DCFpos = 0;
  syncPos = 0;
  prevPos = 0;
  minuteMarker = false;
  startBit = false;
  receiveBit = false;
  lastDCFDecode = false;
  receiveComplete = false;
  ticktock = false;
  prevtock = false;
  button = false;

  lastReceiveTime = millis();
}

//----------------------------------------------------------------------------------

void loop() {
  currentTime = millis();

#ifdef SERIALDEBUG
  // debug DCF77 signal
  if (minuteMarker) {
    minuteMarker = false;
    Serial.println("\n\rMinute marker detected");
  }
  if (startBit) {
    startBit = false;
    Serial.println("Startbit detected");
  }
#endif
  if (receiveBit && DCFpos) {
    receiveBit = false;
#ifdef SERIALDEBUG
    if (bitArray[DCFpos - 1] == 0) Serial.print("0");
    if (bitArray[DCFpos - 1] == 1) Serial.print("1");
#endif
#ifdef ONEWIRE
    if (sens) {
      if (DCFpos == 1) startConversion();
      if (DCFpos == 2) tempRaw = readTemperature();
    }
#endif
  }

  // show sync animation during first sync
  if (syncReq && timeState == TIME_SYNC) {
    // show receive pulse animation
    if (receiveState == RECEIVE_MINUTEMARKER && DCFpos < DCF77_SIZE) {
      if (prevPulse != pulseType) {
        prevPulse = pulseType;
        if (pulseType == PULSE_START) showSync(SHOWSYNC_MINUTEMARKER, syncPos, true);
        if (pulseType == PULSE_END) showSync(SHOWSYNC_MINUTEMARKER, syncPos++, false);
        if (syncPos > 3) syncPos = 0;
      }
    }
    // show counter
    if (receiveState == RECEIVE_RECEIVING && DCFpos < DCF77_SIZE) {
      if (DCFpos != prevPos) {
        prevPos = DCFpos;
        showSync(SHOWSYNC_RECEIVING, DCF77_SIZE - DCFpos, false);
      }
    }
    // show final counter value
    if (receiveState == RECEIVE_MINUTEMARKER && DCFpos == DCF77_SIZE) showSync(SHOWSYNC_RECEIVING, 0, false);
  }

#ifdef TINY1614
  // read button and debounce
  if (button) {
    button = false;
    if (currentTime - lastButtonTime > buttonDelay) {
      lastButtonTime = currentTime;
      switch (showState) {
        case SHOW_TIMEDATE:
          showState = SHOW_TIMEFULL;
          break;
        case SHOW_TIMEFULL:
          if (sens) showState = SHOW_TIMETEMP;
          else showState = SHOW_LOWBATT;
          break;
        case SHOW_TIMETEMP:
          showState = SHOW_LOWBATT;
          break;
        case SHOW_LOWBATT:
          showState = SHOW_TIMEDATE;
          break;
      }
    }
  }
#endif

  // DCF77 processing
  switch (DCFstate) {
    case DCF_RECEIVING:
      if (receiveComplete) {
#ifdef SERIALDEBUG
        Serial.println("DCF77 receive complete");
#endif
        receiveComplete = false;
        lastReceiveTime = currentTime;
        DCFstate = DCF_RECEIVECOMPLETE;
      }
      if (currentTime - lastReceiveTime > resyncDelay) {
        // not received time info for longer time
        syncTimeout = true;
      }
      break;

    case DCF_RECEIVECOMPLETE:
      if (decodeDCF77(bitArray, &dCF77time) == SUCCESS) {
#ifdef SERIALDEBUG
        Serial.println("New DCF77 decode successful");
#endif  
        syncTimeout = false;
        lastDCFDecode = true;
        DCFstate = DCF_SYNCTIME;
      }
      else {
        lastDCFDecode = false;
        DCFstate = DCF_RECEIVING;
      }
      break;

    case DCF_SYNCTIME:
      if (syncReq) {
        // update local time
        dt = DateTime(dCF77time.year, dCF77time.month, dCF77time.day, dCF77time.hour, dCF77time.minute, 0);
#ifdef RTC_AVAIL
        rtc.adjust(dt);
#endif
        // clear RTC clock counter
        while (RTC.STATUS > 0);
        RTC.CNT = 0;

        syncReq = false;
      }
      lastDCFDecode = false;
      DCFstate = DCF_RECEIVING;
      break;
  }

  // state machine for display handling
  if (ticktock != prevtock) {
    prevtock = ticktock;

    measureVoltage();
    if (vcc[0] < 3) showState = SHOW_LOWBATT;

    switch (timeState) {
      case TIME_NOTIME:
        // with no RTC-time, wait for DCF-time
#ifdef SERIALDEBUG
        Serial.println("Waiting for DCF77 time...");
#endif
        syncReq = true;
        syncStatus = false;
        syncTimeout = true;
        timeState = TIME_SYNC;
        showSync(SHOWSYNC_MINUTEMARKER, 0, false);
        break;

      case TIME_RTC:
        // on initial start with RTC time, trigger resync
        syncReq = true;
        syncStatus = false;
        syncTimeout = true;
        timeState = TIME_RESYNC;
        break;
        
      case TIME_SYNC:
        // on initial sync, wait for first time arrival
        if (syncReq == false) {
#ifdef SERIALDEBUG
          Serial.println("Sync: Time is updated");
#endif
          resyncTime = currentTime;
          timeState = TIME_SYNCED;
          syncStatus = true;
          showTime(showState, dt.hour(), dt.minute(), dt.second(), dt.month(), dt.day(), ticktock, syncStatus);
        }
        break;

      case TIME_RESYNC:
        // resync after startup with RTC time
        if (syncReq == false) {
#ifdef SERIALDEBUG
          Serial.println("Resync: Time is updated");
#endif
          resyncTime = currentTime;
          timeState = TIME_SYNCED;
          syncStatus = true;
        }
        showTime(showState, dt.hour(), dt.minute(), dt.second(), dt.month(), dt.day(), ticktock, syncStatus);
        break;
      
      case TIME_SYNCED:
        if (currentTime - resyncTime > resyncDelay && syncReq == false) {
#ifdef SERIALDEBUG
          Serial.println("Resync: Resync started...");
#endif
          // request DCF update
          resyncTime = currentTime;
          syncReq = true;
        }
        // calculate new sync status
        if (syncReq && syncTimeout) syncStatus = false;

        showTime(showState, dt.hour(), dt.minute(), dt.second(), dt.month(), dt.day(), ticktock, syncStatus);
        break;
    }
  }
  delay(10);
}

//----------------------------------------------------------------------------------

void setupDCF77(void)
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

int BitScaleDCF77(uint8_t *bitstring, uint8_t length)
{
  static const int weights[] = {1, 2, 4, 8, 10, 20, 40, 80};
  int value = 0;
  static const int weights_len = 8;

  for (int i = 0; i < length && i < weights_len; i++) value += weights[i] * bitstring[i];

  return value;
}

int checkParity(uint8_t *bitArray)
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
    return ERROR_INVALID_VALUE; // Parity error

  return SUCCESS;
}

//Extracts and interprets the date and time from the binary DCF77 string and writes them into a TimeStampDCF77 structure.
int decodeDCF77(uint8_t *bitArray, TimeStampDCF77 * time)
{
  // Decode the bit strings according to the DCF77 specification
  time->hour = BitScaleDCF77(bitArray + 29, 6);
  time->minute = BitScaleDCF77(bitArray + 21, 7);
  time->day = BitScaleDCF77(bitArray + 36, 6);
  time->weekday = BitScaleDCF77(bitArray + 42, 3);
  time->month = BitScaleDCF77(bitArray + 45, 5);
  time->year = BitScaleDCF77(bitArray + 50, 8);
  time->transmitter_fault = BitScaleDCF77(bitArray + 15, 1);
  time->A1 = BitScaleDCF77(bitArray + 16, 1);
  time->CEST = BitScaleDCF77(bitArray + 17, 1);
  time->CET = BitScaleDCF77(bitArray + 18, 1);

  if (checkParity(bitArray) == ERROR_INVALID_VALUE) {
#ifdef SERIALDEBUG
    Serial.println("\nParity error in hour or minute.");
#endif
    return ERROR_INVALID_VALUE;
  }

  // Check if day, month, or year have invalid (00) values
  if (time->day == 0 || time->month == 0 || time->year == 0 || (time->CEST == time->CET)) {
#ifdef SERIALDEBUG
    Serial.println("\nInvalid date received.");
#endif
    return ERROR_INVALID_VALUE; // The date is not plausible
  }
  return SUCCESS;
}

//----------------------------------------------------------------------------------

ISR(PORTA_PORT_vect) {

  // detect DCF77 signal
  if (PORTA.INTFLAGS & pinDcf) {
    PORTA.INTFLAGS = pinDcf;

    // filter noise
    if (TCA0.SINGLE.CNT < BIT_0_MIN_DURATION) return;

    if (PORTA.IN & pinDcf) {
      // edge low to high
#ifdef LED
      PORTA.OUTCLR = pinLed;
#endif
      lengthPulse = TCA0.SINGLE.CNT;
      TCA0.SINGLE.CNT = 0;
      pulseType = PULSE_END;
    } else {
      // edge high to low
#ifdef LED
      PORTA.OUTSET = pinLed;
#endif
      lengthPause = TCA0.SINGLE.CNT;
      TCA0.SINGLE.CNT = 0;
      pulseType = PULSE_START;
    }

    switch (receiveState) {
      case RECEIVE_MINUTEMARKER:
        if (pulseType == PULSE_START) {
          if (lengthPause >= TIMEOUT_DURATION_LOW && lengthPause <= TIMEOUT_DURATION_HIGH) {
            // minute marker detected
            minuteMarker = true;
            // this triggers data processing
            if (DCFpos == DCF77_SIZE) receiveComplete = true;
            DCFpos = 0;
            receiveState = RECEIVE_STARTBIT;
          }
        }
        break;

      case RECEIVE_STARTBIT:
        if (pulseType == PULSE_END) {
          if (lengthPulse >= BIT_0_DURATION_LOW && lengthPulse <= BIT_0_DURATION_HIGH) {
            bitArray[0] = 0;
            startBit = true;
            DCFpos = 1;
            receiveState = RECEIVE_RECEIVING;
          }
          if (lengthPulse < BIT_0_DURATION_LOW || lengthPulse > BIT_1_DURATION_HIGH) {
            // receive signal error
            DCFpos = 0;
            receiveState = RECEIVE_MINUTEMARKER;
          }
        }
        break;

      case RECEIVE_DETECT:
        // wait for first valid pulse
        if (pulseType == PULSE_END) {
          if ((lengthPulse >= BIT_0_DURATION_LOW && lengthPulse <= BIT_0_DURATION_HIGH) ||
              (lengthPulse >= BIT_1_DURATION_LOW && lengthPulse <= BIT_1_DURATION_HIGH)) {
            // first valid pulse detected 
            receiveState = RECEIVE_MINUTEMARKER;
          }
        }
        break;
      
      case RECEIVE_RECEIVING:
        if (pulseType == PULSE_END) {
          if (lengthPulse >= BIT_0_DURATION_LOW && lengthPulse <= BIT_0_DURATION_HIGH) {
            bitArray[DCFpos] = 0;
            receiveBit = true;
          }
          if (lengthPulse >= BIT_1_DURATION_LOW && lengthPulse <= BIT_1_DURATION_HIGH) {
            bitArray[DCFpos] = 1;
            receiveBit = true;
          }
          if (lengthPulse < BIT_0_DURATION_LOW || lengthPulse > BIT_1_DURATION_HIGH) {
            // receive signal error
            DCFpos = 0;
            receiveState = RECEIVE_MINUTEMARKER;
          }
          if (++DCFpos == DCF77_SIZE) receiveState = RECEIVE_MINUTEMARKER;
        }
        break;
    }
  }

  // detect button press
  if (PORTA.INTFLAGS & pinButton) {
    PORTA.INTFLAGS = pinButton;
    button = true;
  }
}

ISR(TCA0_CMP0_vect) {
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_CMP0_bm;
  TCA0.SINGLE.CNT = 0;

  //PORTA.OUTTGL = pinLed;
}

// the RTC interrupt is called twice a seconds
ISR(RTC_PIT_vect) {
  RTC.PITINTFLAGS = RTC_PI_bm;
  
  ticktock = !ticktock;
  if (ticktock == true) dt = dt + 1;
  
  //PORTA.OUTTGL = pinLed;
}

//----------------------------------------------------------------------------------

void showSync(uint8_t showMode, uint8_t pos, bool sync) {
  char buffer[9];

  buffer[0] = 'S';
  buffer[1] = 'Y';
  buffer[2] = 'N';
  buffer[3] = 'C';
  buffer[4] = buffer[5] = buffer[6] = buffer[7] = ' ';

  switch (showMode) {
    case SHOWSYNC_MINUTEMARKER:
      if (pos > 3) return;
      if (sync) buffer[4 + pos] = ' ' | COLON;
      break;

    case SHOWSYNC_RECEIVING:
      if (pos >= 10) buffer[6] = '0' + pos / 10;
      buffer[7] = '0' + pos % 10;
  }
  buffer[8] = 0;
  alpha.print(buffer);
}

void showTime(uint8_t mode, uint8_t hr, uint8_t min, uint8_t sec, uint8_t m, uint8_t d, bool colon, bool sync) {
  char buffer[9];

  // show time
  buffer[0] = '0' + hr / 10;
  buffer[1] = ('0' + hr % 10) | (colon ? COLON : 0);
  buffer[2] = '0' + min / 10;
  buffer[3] = ('0' + min % 10) | (sync ? 0 : COLON);

  switch (mode) {
    case SHOW_TIMEDATE:
      if (d >= 10 && m >= 10) {
        buffer[4] = '0' + d / 10;
        buffer[5] = ('0' + d % 10) | COLON;
        buffer[6] = '0' + m / 10;
        buffer[7] = ('0' + m % 10) | COLON;
      }
      if (d >= 10 && m < 10) {
        buffer[4] = ' ';
        buffer[5] = '0' + d / 10;
        buffer[6] = ('0' + d % 10) | COLON;
        buffer[7] = ('0' + m) | COLON;
      }
      if (d < 10 && m >= 10) {
        buffer[4] = ' ';
        buffer[5] = ('0' + d) | COLON;
        buffer[6] = '0' + m / 10;
        buffer[7] = ('0' + m % 10) | COLON;
      }
      if (d < 10 && m < 10) {
        buffer[4] = ' ';
        buffer[5] = ' ';
        buffer[6] = ('0' + d) | COLON;
        buffer[7] = ('0' + m) | COLON;
      }
      break;

    case SHOW_TIMEFULL:
      buffer[4] = '0' + sec / 10;
      buffer[5] = '0' + sec % 10;
      buffer[6] = ' ';
      buffer[7] = ' ';
      break;

#ifdef ONEWIRE
    case SHOW_TIMETEMP:
      int8_t temp;
      uint8_t tenth;
      temp = tempRaw >> 4;
      tenth = ((tempRaw & 0x0F) * 10) >> 4;
      // check for invalid range
      if (temp < -9 || temp > 50) {
        buffer[4] = '-';
        buffer[5] = '-' | COLON;
        buffer[6] = '-';
        buffer[7] = 'C';
        break;
      }
      if (temp < 0) {
        temp = -temp;
        buffer[4] = '-';
        buffer[5] = ('0' + temp) | COLON;
      }
      else {
        buffer[4] = '0' + temp / 10;
        buffer[5] = ('0' + temp % 10) | COLON;
      }
      buffer[6] = '0' + tenth;
      buffer[7] = 'C';
      break;
#endif

    case SHOW_LOWBATT:
      buffer[4] = ' ';
      buffer[5] = ' ';
      buffer[6] = ('0' + vcc[0]) | COLON;
      buffer[7] = '0' + vcc[1];
      break;
  }
  buffer[8] = 0;
  alpha.print(buffer);
}

void RTCinit(void) {

// enable external 32K clock when external RTC available
#ifdef RTC_AVAIL
  uint8_t temp = CLKCTRL.XOSC32KCTRLA & ~CLKCTRL_ENABLE_bm;
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.XOSC32KCTRLA = temp;
  while (CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm);

  temp = CLKCTRL.XOSC32KCTRLA | CLKCTRL_SEL_bm;
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.XOSC32KCTRLA = temp;

  temp = CLKCTRL.XOSC32KCTRLA | CLKCTRL_ENABLE_bm;
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.XOSC32KCTRLA = temp;

  while (RTC.STATUS > 0);
  RTC.CLKSEL = RTC_CLKSEL_TOSC32K_gc;
#else
  // use internal 32K clock source
  while (RTC.STATUS > 0);
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;
#endif
  
  while (RTC.PITSTATUS > 0);
  RTC.PITCTRLA = RTC_PERIOD_CYC16384_gc | RTC_PITEN_bm;
  while (RTC.PITSTATUS > 0);
  RTC.PITINTCTRL = RTC_PI_bm;
}

void ADCinit(void) {
  VREF.CTRLA = VREF_ADC0REFSEL_1V1_gc;
  ADC0.CTRLC = ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV256_gc;
  ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;
  ADC0.CTRLA = ADC_ENABLE_bm;
}

void measureVoltage(void) {
  ADC0.COMMAND = ADC_STCONV_bm;
  while (ADC0.COMMAND & ADC_STCONV_bm);
  uint16_t adc_reading = ADC0.RES;
  uint16_t voltage = 11264 / adc_reading;
  vcc[0] = voltage / 10;
  vcc[1] = voltage % 10;
}
