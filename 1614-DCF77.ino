//------------------------------------------------------------------------------------------------
//  1614-DCF77.ino
//
//  This program receives the european DCF77 time signal and syncs it with the external RTC-clock.
//  It is displayed on an 8-digit alphanumeric- or 7-segment display with display-controller HT16K33.
//  The MCU used is an ATtiny 814 or 1614. Without serial debugging and OneWire, an ATtiny 412
//  can be used. The MCU used is selected by setting the appropriate #define statement.
//  The circuit can be powered by a LiPo-cell. If the voltage drops below 3.0 V,
//  the voltage is displayed as a low voltage indicator.
//  A push button switches between different display modes:
//  Time with date, time with seconds, time with temperature, time with battery voltage
//
//  MCU-clock: 8 MHz
//  Timers used: TCA0 (DCF signal measurement), TCB0 (OneWire timing),
//               TCD0 (millis), RTC (2 Hz generation via interrupt)
//  External RTC: DS3231 with battery backup, supplies 32K clock for internal RTC
//
//  Author: Klaus Wolf
//  Date: June 14th 2026
//------------------------------------------------------------------------------------------------

#if defined(__AVR_ATtiny412__)
#include <TinyI2CMaster.h>
#include "DateTime.h"
#define TINYWIRE
#endif
#if defined(__AVR_ATtiny814__) || defined(__AVR_ATtiny1614__)
#include <Wire.h>
#include <RTClib.h>
#include "OneWire.h"
#define RTC_AVAIL
#define ONEWIRE
#define BUTTON
//#define SERIALDEBUG
#endif
#define SEG14
#include "AlphaDisplay.h"
#include "dcf77.h"

#define LED

#define DISPLAY_ADDRESS         0x70
#define DISPLAY_DIGITS          8
#define DISPLAY_BRIGHTNESS      4

#define ONEWIRE_PIN             4
#define ONEWIRE_RESOLUTION      11

#ifdef MILLIS_USE_TIMERA0
#error "This sketch takes over TCA0 - please use a different timer for millis"
#endif

const uint8_t pinDcf = PIN3_bm;
const uint8_t pinLed = PIN6_bm;
const uint8_t pinButton = PIN7_bm;

const uint32_t syncDelay = 30 * 60 * 1000L;
const uint32_t tempDelay = 20 * 1000L;
const uint32_t buttonDelay = 500L;

bool      syncReq, syncComplete, sens;
uint8_t   timeState, showState, receiveState, syncState;
int16_t   temp;
uint16_t  vcc;
uint32_t  currentTime, lastSyncTime, lastTempTime, lastButtonTime;

volatile bool tickTock, button;

AlphaDisplay alpha;
dcf77 dcf;
timeStampDCF77 dcfTime;

DateTime dt(2026, 1, 1, 0, 0, 0);

#ifdef RTC_AVAIL
RTC_DS3231 rtc;
#endif

#ifdef ONEWIRE
OneWire ow(ONEWIRE_PIN);
#endif

enum { TIME_NOTIME = 1, TIME_RTC, TIME_SYNC, TIME_SYNCED };
enum { RECEIVE_INIT = 1, RECEIVE_IDLE, RECEIVE_RECEIVING, RECEIVE_COMPLETE };
enum { SHOW_TIMEDATE = 1, SHOW_TIMEFULL, SHOW_TIMETEMP, SHOW_LOWBATT };
enum { SHOWSYNC_INIT = 1, SHOWSYNC_IDLE, SHOWSYNC_MINUTEMARKER, SHOWSYNC_STARTBIT,
       SHOWSYNC_RECEIVING, SHOWSYNC_RECEIVECOMPL, SHOWSYNC_NOSIGNAL, SHOWSYNC_WAITSIGNAL };

//----------------------------------------------------------------------------------

void setup() {
#ifdef SERIALDEBUG
  Serial.swap(1);         // use PA1(TXD) and PA2 (RXD)
  Serial.begin(115200);
  Serial.println("\r\nInit...");
#endif

  // DCF signal input with pullup
  PORTA.DIRCLR = pinDcf;
  PORTA.PIN3CTRL = PORT_PULLUPEN_bm;
  PORTA.PIN3CTRL |= PORT_ISC_BOTHEDGES_gc;

#ifdef LED
  PORTA.DIRSET = pinLed;
  PORTA.OUTSET = pinLed;
#endif

#ifdef BUTTON
  PORTA.DIRCLR = pinButton;
  PORTA.PIN7CTRL = PORT_PULLUPEN_bm;
  PORTA.PIN7CTRL |= PORT_ISC_FALLING_gc;
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
  ow.setup();
  delay(100);
  ow.setResolution(ONEWIRE_RESOLUTION);
  delay(100);
  if (ow.startConversion()) {
    sens = true;
    delay(500);
    temp = ow.readTemperature();

    alpha.print("TEMP OK ");
    delay(1500);
  }
#endif

  timeState = TIME_NOTIME;
#ifdef RTC_AVAIL
  // initialize external RTC
  if (rtc.begin()) {
    if (rtc.lostPower()) {
#ifdef SERIALDEBUG
      Serial.println("RTC power loss!");
#endif
      rtc.adjust(DateTime(2026, 1, 1, 12, 0, 0));
    }
    else timeState = TIME_RTC;
    
    rtc.enable32K();
    alpha.print("RTC OK  ");
  }
  else {
#ifdef SERIALDEBUG
    Serial.println("RTC failed!");
#endif
    alpha.print("RTC FAIL");
  }
  delay(1500);
#endif

  RTCinit();
  ADCinit();

  // setup timer TCA0 for DCF signal processing
  dcf.timerSetup();

  receiveState = RECEIVE_INIT;
  syncState = SHOWSYNC_INIT;
  showState = SHOW_TIMEDATE;
  tickTock = false;
  button = false;

  lastButtonTime = lastTempTime = millis();

  alpha.print("--------");
  
  // setup finished
#ifdef LED
  PORTA.OUTCLR = pinLed;
#endif
}

//----------------------------------------------------------------------------------

void loop() {
  currentTime = millis();

  // show sync animation and countdown
  syncState = handleAnimation(syncState);

  // state machine for receive data handling
  receiveState = handleReceive(receiveState);

  // state machine for time and display handling
  timeState = handleTime(timeState);

#ifdef BUTTON
  // switching display modes
  showState = handleButton(showState);
#endif

  delay(20);
}

//----------------------------------------------------------------------------------

uint8_t handleAnimation(uint8_t state) {

  uint8_t pos;
  dcf77::dcfState dcfState;
  static uint8_t syncPos, prevPos;
  static dcf77::pulseType pulse, prevPulse;

  if (timeState != TIME_SYNC) return SHOWSYNC_IDLE;
  if (dcf.getSignalStatus() == false) state = SHOWSYNC_NOSIGNAL;
  
  dcfState = dcf.getState();
  
  switch (state) {
    case SHOWSYNC_INIT:
      // wait for first signal
      showSync(SHOWSYNC_WAITSIGNAL, false);
      return SHOWSYNC_WAITSIGNAL;
      
    case SHOWSYNC_IDLE:
      if (dcfState == dcf77::dcfState::MINUTEMARKER) {
        prevPulse = dcf77::pulseType::END;
        return SHOWSYNC_MINUTEMARKER;
      }
      break;

    case SHOWSYNC_MINUTEMARKER:
#ifdef SERIALDEBUG
      if (dcf.checkMinuteMarker()) Serial.println("\r\nMinute marker detected");
#endif
      if (dcfState == dcf77::dcfState::STARTBIT) {
        syncPos = 0;
        prevPos = 0;
        return SHOWSYNC_STARTBIT;
      }
      // show receive pulse animation
      pulse = dcf.getLastPulseType();
      if (prevPulse != pulse) {
        prevPulse = pulse;
        if (pulse == dcf77::pulseType::START) showSync(state, true);
        if (pulse == dcf77::pulseType::END) showSync(state, false);
        if (syncPos > 3) syncPos = 0;
#ifdef SERIALDEBUG
        if (pulse == dcf77::pulseType::END) Serial.printf("PulseLen: %u\r\n", dcf.getLastPulseLen());
#endif
      }
      break;

    case SHOWSYNC_STARTBIT:
#ifdef SERIALDEBUG
      if (dcf.checkStartBit()) Serial.println("Startbit detected");
#endif
      if (dcfState == dcf77::dcfState::RECEIVING) return SHOWSYNC_RECEIVING;
      break;

    case SHOWSYNC_RECEIVING:
      if (dcf.checkComplete()) return SHOWSYNC_RECEIVECOMPL;
      if (dcf.checkRestart()) return SHOWSYNC_IDLE;
      
      pos = dcf.getPos();
      if (pos != prevPos) {
        prevPos = pos;
#ifdef SERIALDEBUG
        if (dcf.getBit(pos - 1)) Serial.print("1"); else Serial.print("0");
#endif
        showSync(state, false);
      }
      break;

    case SHOWSYNC_RECEIVECOMPL:
      return SHOWSYNC_IDLE;

    case SHOWSYNC_NOSIGNAL:
      showSync(state, false);
      return SHOWSYNC_WAITSIGNAL;
    
    case SHOWSYNC_WAITSIGNAL:
      if (dcf.getSignalStatus()) return SHOWSYNC_IDLE;
  }
  return syncState;
}

//----------------------------------------------------------------------------------

uint8_t handleReceive(uint8_t state) {

  switch (state) {
    case RECEIVE_INIT:
      lastSyncTime = currentTime;
      syncReq = true;
      syncComplete = false;
      return RECEIVE_IDLE;

    case RECEIVE_IDLE:
      if (syncReq) {
#ifdef SERIALDEBUG
        Serial.println("\r\nSync: Started...");
#endif
        dcf.request();
        return RECEIVE_RECEIVING;
      }
      if (currentTime - lastSyncTime > syncDelay && dt.second() == 50) syncReq = true;
      break;

    case RECEIVE_RECEIVING:
      if (dcf.checkComplete()) {
#ifdef SERIALDEBUG
        Serial.println("\r\nDCF77 receive complete");
#endif
        return RECEIVE_COMPLETE;
      }
      break;

    case RECEIVE_COMPLETE:
      if (dcf.decode(&dcfTime) == dcf77::result::SUCCESS) {
#ifdef SERIALDEBUG
        Serial.println("DCF77 decode successful");
        //Serial.printf("DCF77-Time: %02u:%02u\r\n", dcfTime.hour, dcfTime.minute);
#endif
        // update local time
        dt = DateTime(dcfTime.year, dcfTime.month, dcfTime.day, dcfTime.hour, dcfTime.minute, 0);
#ifdef RTC_AVAIL
        rtc.adjust(dt);
#endif
        // clear RTC clock counter
        while (RTC.STATUS > 0);
        RTC.CNT = 0;

        // job done
        syncComplete = true;
        syncReq = false;
        lastSyncTime = currentTime;
        return RECEIVE_IDLE;
      }
      else {
#ifdef SERIALDEBUG
        Serial.println("DCF77 decode failed!");
#endif
        // trigger new DCF receiving cycle
        dcf.request();
        return RECEIVE_RECEIVING;
      }
  }
  return state;
}

//----------------------------------------------------------------------------------

uint8_t handleTime(uint8_t state) {

  static bool prevTock = false;

  if (tickTock != prevTock) {
    prevTock = tickTock;

    // show voltage if low (< 3V)
    vcc = measureVoltage();
    if ((vcc >> 8) < 3) showState = SHOW_LOWBATT;

    // read temperature
#ifdef ONEWIRE
    readTemp(tickTock);
#endif

    switch (state) {
      case TIME_NOTIME:
        // with no RTC-time, wait for DCF-time
        return TIME_SYNC;

#ifdef RTC_AVAIL
      case TIME_RTC:
        // initial start with RTC time
        dt = rtc.now();
        return TIME_SYNCED;
#endif
      case TIME_SYNC:
        if (syncComplete) {
          syncComplete = false;
          return TIME_SYNCED;
        }
        break;

      case TIME_SYNCED:
        showTime(showState, dt.hour(), dt.minute(), dt.second(), dt.month(), dt.day(), tickTock, syncReq);
        break;
    }
  }
  return state;
}

//----------------------------------------------------------------------------------

#ifdef BUTTON
uint8_t handleButton(uint8_t state) {

  if (button) {
    button = false;
    if (currentTime - lastButtonTime > buttonDelay) {
      lastButtonTime = currentTime;
      switch (state) {
        case SHOW_TIMEDATE:
          return SHOW_TIMEFULL;

        case SHOW_TIMEFULL:
          if (sens) return SHOW_TIMETEMP;
          else return SHOW_LOWBATT;

        case SHOW_TIMETEMP:
          return SHOW_LOWBATT;

        case SHOW_LOWBATT:
          return SHOW_TIMEDATE;
      }
    }
  }
  return state;
}
#endif

//----------------------------------------------------------------------------------

void showSync(uint8_t mode, bool colon) {
  char buffer[9];
  static uint8_t pos = 0;

  switch (mode) {
    case SHOWSYNC_WAITSIGNAL:
      strcpy(buffer, "WAIT    ");
      pos = 0;
      break;
    
    case SHOWSYNC_MINUTEMARKER:
      strcpy(buffer, "SYNC    ");
      if (colon) buffer[4 + pos] |= COLON;
      if (!colon) pos++;
      if (pos > 3) pos = 0;
      break;

    case SHOWSYNC_RECEIVING:
    case SHOWSYNC_RECEIVECOMPL:
      pos = DCF_SIZE - dcf.getPos();
      strcpy(buffer, "RECV    ");
      if (pos >= 10) buffer[6] = '0' + pos / 10;
      buffer[7] = '0' + pos % 10;
      break;
    
    case SHOWSYNC_NOSIGNAL:
      strcpy(buffer, "NOSIGNAL");
  }
  buffer[8] = 0;
  alpha.print(buffer);
}

//----------------------------------------------------------------------------------

void showTime(uint8_t mode, uint8_t hr, uint8_t min, uint8_t sec, uint8_t m, uint8_t d, bool colon, bool sync) {
  char buffer[9];
  
  // show time
  buffer[0] = hr >= 10 ? ('0' + hr / 10) : ' ';
  buffer[1] = ('0' + hr % 10) | (colon ? COLON : 0);
  buffer[2] = '0' + min / 10;
  buffer[3] = ('0' + min % 10) | (sync ? COLON : 0);
  
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
      buffer[3] |= COLON;
      buffer[4] = '0' + sec / 10;
      buffer[5] = ('0' + sec % 10) | (sync ? COLON : 0);
      buffer[6] = ' ';
      buffer[7] = ' ';
      break;

#ifdef ONEWIRE
    case SHOW_TIMETEMP:
      bool negative;
      int16_t temp2;

      if (temp < 0) {
        negative = true;
        temp2 = -temp;
      }
      else {
        negative = false;
        temp2 = temp;
      }

      if (negative) {
        buffer[4] = '-';
        buffer[5] = ('0' + temp2 / 100) | COLON;
      }
      else {
        buffer[4] = '0' + temp2 / 1000;
        buffer[5] = ('0' + ((temp2 / 100) % 10)) | COLON;
      }
      buffer[6] = '0' + (temp2 / 10) % 10;
      buffer[7] = 'C';
      break;
#endif

    case SHOW_LOWBATT:
      buffer[4] = ' ';
      buffer[5] = ('0' + (vcc >> 8)) | COLON;
      buffer[6] = '0' + (vcc & 0x0F);
      buffer[7] = 'V';
      break;
  }
  buffer[8] = 0;
  alpha.print(buffer);
}

//----------------------------------------------------------------------------------

#ifdef ONEWIRE
bool readTemp(bool tick) {

  static bool conv = false;

  dcf77::dcfState dcfState = dcf.getState();
  
  if (!sens || dcfState != dcf77::dcfState::IDLE) return false;

  if (tick) {
    // read temperature
    if (conv) {
#ifdef SERIALDEBUG
      Serial.println("Reading temperature...");
#endif
      temp = ow.readTemperature();
      conv = false;
      return true;
    }
    if (currentTime - lastTempTime > tempDelay) {
      lastTempTime = currentTime;
#ifdef SERIALDEBUG
      Serial.println("Starting conversion...");
#endif
      ow.startConversion();
      conv = true;
    }
  }
  return false;
}
#endif

//----------------------------------------------------------------------------------

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

uint16_t measureVoltage(void) {
  ADC0.COMMAND = ADC_STCONV_bm;
  while (ADC0.COMMAND & ADC_STCONV_bm);
  uint16_t adc_reading = ADC0.RES;
  uint16_t voltage = 11264 / adc_reading;
  return (voltage / 10) << 8 | (voltage % 10);
}

//---------------------------------------- ISR ---------------------------------------

ISR(PORTA_PORT_vect) {

  uint16_t lengthSignal;

  // detect DCF77 signal
  if (PORTA.INTFLAGS & pinDcf) {
    PORTA.INTFLAGS = pinDcf;

    // filter noise
    if (TCA0.SINGLE.CNT < BIT_0_MIN_DURATION) {
      TCA0.SINGLE.CNT = 0;
      return;
    }

    if (PORTA.IN & pinDcf) {
      // edge low to high
#ifdef LED
      PORTA.OUTCLR = pinLed;
#endif
      lengthSignal = TCA0.SINGLE.CNT;
      TCA0.SINGLE.CNT = 0;
      dcf.handleInt(dcf77::pulseType::END, lengthSignal);
    } else {
      // edge high to low
#ifdef LED
      //PORTA.OUTSET = pinLed;
      if (dcf.getRequestState()) PORTA.OUTSET = pinLed;
#endif
      lengthSignal = TCA0.SINGLE.CNT;
      TCA0.SINGLE.CNT = 0;
      dcf.handleInt(dcf77::pulseType::START, lengthSignal);
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

  dcf.noSignal();

  //PORTA.OUTTGL = pinLed;
}

// the RTC interrupt is called twice a seconds
ISR(RTC_PIT_vect) {
  RTC.PITINTFLAGS = RTC_PI_bm;

  tickTock = !tickTock;
  if (tickTock) dt = dt + 1;

  //PORTA.OUTTGL = pinLed;
}
