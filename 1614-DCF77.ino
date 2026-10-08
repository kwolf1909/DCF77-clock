//------------------------------------------------------------------------------------------------
//  1614-DCF77.ino
//
//  This program receives the european DCF77 time signal and syncs it with the external RTC-clock.
//  It is displayed on an 8-digit alphanumeric- or 7-segment display with display-controller HT16K33.
//  The MCU used is an ATtiny 1614 or an ATmega 4809. With reduced features, an ATtiny 412 can be used.
//  The circuit can be powered by a Li-Ion cell. If the voltage drops below 3.0 V,
//  the voltage is displayed as a low voltage indicator.
//  A push button switches between different display modes:
//  Time with date, time with seconds, time with temperature, time with battery voltage
//
//  MCU-clock: 8 MHz (recommended)
//  Timers used: TCA0 (DCF pulse width measurement),
//               TCB0 (OneWire timing)
//               TCB3/TCD0 (millis), RTC (2 Hz generation via interrupt)
//  External RTC: DS3231 with battery backup, supplies 32K clock for internal RTC
//
//  Author: Klaus Wolf
//  Date: Oct 08 2026
//------------------------------------------------------------------------------------------------

#if defined(__AVR_ATtiny412__)
#include <TinyI2CMaster.h>
#include "DateTime.h"
#define TINYWIRE
#define LED
#define DCF_PIN                 4   // PA3
#define LED_PIN                 1   // PA7
#endif
#if defined(__AVR_ATtiny1614__)
#include <Wire.h>
#include <RTClib.h>
#include <OneButtonTiny.h>
#include "OneWire.h"
#define RTCEXT
#define ONEWIRE
#define BUTTON
#define VOLTAGE
#define SEG14
#define LED
#define DCF_PIN                 10  // PA3
#define LED_PIN                 2   // PA6
#define BUTTON_PIN              3   // PA7
#define ONEWIRE_PIN             0   // PA4
#endif
#if defined(__AVR_ATmega4809__)
#include <Wire.h>
#include <RTClib.h>
#include <OneButtonTiny.h>
#include "OneWire.h"
#define RTCEXT
#define ONEWIRE
#define BUTTON
//#define VOLTAGE
#define SEG14
#define LED
#define DCF_PIN                 2   // PA0
#define LED_PIN                 13  // PE2
#define BUTTON_PIN              17  // PD0
#define ONEWIRE_PIN             16  // PD1
#endif

//#define SERIALDEBUG

#include "AlphaDisplay.h"
#include "DCF77.h"

#define DISPLAY_ADDRESS         0x70
#define DISPLAY_DIGITS          8
#define DISPLAY_BRIGHTNESS      4
#define ONEWIRE_RESOLUTION      11

#ifdef MILLIS_USE_TIMERA0
#error "This sketch takes over TCA0 - please use a different timer for millis"
#endif

const bool      syncAfterStart = true;
const uint8_t   readDelay = 10;
const uint16_t  voltageLimit = 3000;
const uint32_t  syncDelay = 30 * 60 * 1000L;
const uint32_t  tempDelay = 20 * 1000L;
const uint32_t  buttonDelay = 250L;

char      buffer[40];
bool      syncReq, syncComplete, sens, buttonPressed, buttonLongPressed;
uint8_t   timeState, showState, receiveState, syncState, readState;
int16_t   temp;
uint16_t  vcc;
uint32_t  currentTime, lastSyncTime;
volatile bool tickTock;

AlphaDisplay alpha;
dcf77 dcf;
timeStampDCF77 dcfTime;

DateTime dt(2026, 1, 1, 0, 0, 0);

#ifdef RTCEXT
RTC_DS3231 rtc;
#endif

#ifdef ONEWIRE
OneWire ow(ONEWIRE_PIN);
#endif

#ifdef BUTTON
OneButtonTiny button(BUTTON_PIN, true, true);
#endif

enum { TIME_NOTIME = 1, TIME_RTC, TIME_SYNC, TIME_SYNCED };
enum { RECEIVE_INIT = 1, RECEIVE_IDLE, RECEIVE_RECEIVING, RECEIVE_COMPLETE };
enum { SHOW_TIMEDATE = 1, SHOW_TIMEFULL, SHOW_TIMETEMP, SHOW_LOWBATT };
enum { SHOWSYNC_IDLE = 1, SHOWSYNC_MINUTEMARKER, SHOWSYNC_STARTBIT,
       SHOWSYNC_RECEIVING, SHOWSYNC_RECEIVECOMPL, SHOWSYNC_NOSIGNAL, SHOWSYNC_WAITSIGNAL
     };
enum { READ_IDLE = 1, READ_STARTCONV, READ_TEMP, READ_VCC};

//----------------------------------------------------------------------------------

void setup() {
#ifdef SERIALDEBUG
#if defined(__AVR_ATtiny1614__)
  Serial.swap(1); // PA1(TX)
  Serial.begin(115200, SERIAL_TX_ONLY);
#endif
#if defined(__AVR_ATmega4809__)
  Serial.begin(115200); // PA0(TX)
#endif
  Serial.println("\r\nInit...");
  sprintf(buffer, "F_CPU=%lu", F_CPU);
  Serial.println(buffer);
#endif

#ifdef BUTTON
  button.attachClick([]() {
    buttonPressed = true;
  });
  button.attachLongPressStart([]() {
    buttonLongPressed = true;
  });
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
  if (!ow.reset()) {
    sens = true;
    ow.setResolution(ONEWIRE_RESOLUTION);
    alpha.print("TEMP OK ");
    delay(1000);
  }
#endif

  timeState = TIME_NOTIME;
#ifdef RTCEXT
  // initialize external RTC
  if (rtc.begin()) {
    if (rtc.lostPower()) {
#ifdef SERIALDEBUG
      Serial.println("RTC power loss!");
#endif
      rtc.adjust(DateTime(2026, 1, 1, 0, 0, 0));
    }
    else timeState = TIME_RTC;

    rtc.enable32K();
    alpha.print("RTC OK  ");
    delay(1000);
  }
  else alpha.print("RTC FAIL");
#endif

  // use external RTC for 32K clock if available, or use internal 32K clock source
  RTCinit();
#ifdef VOLTAGE
  ADCinit();
#endif
  // start DCF signal processing
  dcf.begin(DCF_PIN, LED_PIN);

  receiveState = RECEIVE_INIT;
  syncState = SHOWSYNC_IDLE;
  showState = SHOW_TIMEDATE;
  readState = READ_IDLE;
  tickTock = false;
  buttonPressed = false;
  buttonLongPressed = false;
  vcc = voltageLimit;

  alpha.print("--------");
}

//----------------------------------------------------------------------------------

void loop() {

  dcf.handle();

  // show sync animation and countdown
  syncState = handleAnimation(syncState);

  // state machine for receive data handling
  receiveState = handleReceive(receiveState);

  // state machine for time and display handling
  timeState = handleTime(timeState);

  // handle temp and voltage reading
  readState = handleReadTempVcc(readState);

#ifdef BUTTON
  // switching display modes
  showState = handleButton(showState);
#endif
}

//----------------------------------------------------------------------------------

uint8_t handleAnimation(uint8_t state) {

  if (timeState != TIME_SYNC || dcf.getRequestState() == false) return SHOWSYNC_IDLE;

  switch (state) {
    case SHOWSYNC_IDLE:
      // wait for first signal
      showSync(SHOWSYNC_WAITSIGNAL, 0, false);
      return SHOWSYNC_WAITSIGNAL;

    case SHOWSYNC_WAITSIGNAL:
      if (dcf.getSignalStatus() == dcf77::dcfSignal::GOOD) return SHOWSYNC_MINUTEMARKER;
      if (dcf.getSignalStatus() == dcf77::dcfSignal::NOSIGNAL) {
        showSync(SHOWSYNC_NOSIGNAL, 0, false);
        return SHOWSYNC_NOSIGNAL;
      }
      break;

    case SHOWSYNC_NOSIGNAL:
      if (dcf.getSignalStatus() == dcf77::dcfSignal::WAIT) {
        showSync(SHOWSYNC_WAITSIGNAL, 0, false);
        return SHOWSYNC_WAITSIGNAL;
      }
      break;

    case SHOWSYNC_MINUTEMARKER:
      if (dcf.checkMinuteMarker()) return SHOWSYNC_STARTBIT;
      if (dcf.checkPulseReceived()) showSync(state, 0, dcf.getLastPulseType() == dcf77::dcfPulseType::START ? true : false);
      break;

    case SHOWSYNC_STARTBIT:
      if (dcf.checkStartBit()) {
        showSync(state, 0, false);
        return SHOWSYNC_RECEIVING;
      }
      break;

    case SHOWSYNC_RECEIVING:
      if (dcf.checkComplete()) return SHOWSYNC_RECEIVECOMPL;
      if (dcf.checkRestart()) return SHOWSYNC_IDLE;
      if (dcf.checkReceiveBit()) showSync(state, dcf.getPos(), false);
      break;

    case SHOWSYNC_RECEIVECOMPL:
      return SHOWSYNC_IDLE;
  }
  return state;
}

//----------------------------------------------------------------------------------

uint8_t handleReceive(uint8_t state) {

  switch (state) {
    case RECEIVE_INIT:
      lastSyncTime = currentTime;
      syncComplete = false;
      return RECEIVE_IDLE;

    case RECEIVE_IDLE:
      if (syncReq) {
        syncReq = false;
        syncComplete = false;
#ifdef SERIALDEBUG
        Serial.println("\r\nSync: Started");
#endif
        dcf.request();
        return RECEIVE_RECEIVING;
      }
      if (currentTime - lastSyncTime > syncDelay && dt.second() == 50) syncReq = true;
      break;

    case RECEIVE_RECEIVING:
      if (dcf.checkComplete()) {
#ifdef SERIALDEBUG
        Serial.println("Receive: Receive complete");
#endif
        return RECEIVE_COMPLETE;
      }
      break;

    case RECEIVE_COMPLETE:
      if (dcf.decode(&dcfTime) == dcf77::result::SUCCESS) {
#ifdef SERIALDEBUG
        Serial.println("Receive: Decode successful");
#endif
        // update local time
        dt = DateTime(dcfTime.year, dcfTime.month, dcfTime.day, dcfTime.hour, dcfTime.minute, 0);
#ifdef RTCEXT
        rtc.adjust(dt);
#endif
#ifdef SERIALDEBUG
        sprintf(buffer, "Receive: RTC synced to DCF-time %02u:%02u:00", dcfTime.hour, dcfTime.minute);
        Serial.println(buffer);
#endif
        // clear RTC clock counter
        while (RTC.STATUS > 0);
        RTC.CNT = 0;

        // job done
        syncComplete = true;
        lastSyncTime = currentTime;
        return RECEIVE_IDLE;
      }
      else {
#ifdef SERIALDEBUG
        Serial.println("Receive: Decode failed!");
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

  bool tt = tickTock;
  static bool prevtt = false;

  if (prevtt == tt) return state;
  prevtt = tt;

  // increase local time by one second
  if (tt) dt = dt + 1;

  switch (state) {
    case TIME_NOTIME:
      // with no RTC-time, wait for DCF-time
      syncReq = true;
      return TIME_SYNC;

#ifdef RTCEXT
    case TIME_RTC:
      // initial start with RTC time
      dt = rtc.now();
      if (syncAfterStart) syncReq = true;
      return TIME_SYNCED;
#endif
    case TIME_SYNC:
      if (syncComplete) {
        syncComplete = false;
        return TIME_SYNCED;
      }
      break;

    case TIME_SYNCED:
      showTime(showState, dt.hour(), dt.minute(), dt.second(), dt.month(), dt.day(), tt, dcf.getRequestState());
      break;
  }
  return state;
}

//----------------------------------------------------------------------------------

uint8_t handleReadTempVcc(uint8_t state) {

  bool tt = tickTock;
  static bool prevtt = false;
  static uint8_t delayCounter = 0;

  if (prevtt == tt) return state;
  prevtt = tt;

  // don't read values during DCF signal processing
  if (dcf.getRequestState()) return READ_IDLE;

  if (prevtt) {
    switch (state) {
      case READ_IDLE:
        if (delayCounter++ >= readDelay) {
          if (sens) return READ_STARTCONV;
          else return READ_VCC;
        }
        break;

      case READ_STARTCONV:
#ifdef ONEWIRE
        ow.startConversion();
#endif
        return READ_TEMP;

      case READ_TEMP:
#ifdef ONEWIRE
        temp = ow.readTemperature();
#ifdef SERIALDEBUG
        sprintf(buffer, "DS18B20: Reading temperature: %u", temp);
        Serial.println(buffer);
#endif
#endif
        return READ_VCC;

      case READ_VCC:
#ifdef VOLTAGE
        vcc = measureVoltage();
        if (vcc < voltageLimit) showState = SHOW_LOWBATT;
#ifdef SERIALDEBUG
        sprintf(buffer, "Voltage: %u", vcc);
        Serial.println(buffer);
#endif
#endif
        delayCounter = 0;
        return READ_IDLE;
    }
  }
  return state;
}

//----------------------------------------------------------------------------------

#ifdef BUTTON
uint8_t handleButton(uint8_t state) {

  button.tick();

  if (buttonPressed) {
    buttonPressed = false;
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
  if (buttonLongPressed) {
    buttonLongPressed = false;
    if (timeState == TIME_SYNCED) {
      timeState = TIME_SYNC;
      syncState = SHOWSYNC_IDLE;
      syncReq = true;
    }
  }
  return state;
}
#endif

//----------------------------------------------------------------------------------

void showSync(uint8_t mode, uint8_t pos, bool dot) {
  char buffer[DISPLAY_DIGITS + 1];
  uint8_t posrev;
  static uint8_t posdot = 0;

  switch (mode) {
    case SHOWSYNC_MINUTEMARKER:
      strcpy(buffer, "SYNC    ");
      if (dot) {
        buffer[4 + posdot] |= DOT;
        posdot++;
      }
      if (posdot > 3) posdot = 0;
      break;

    case SHOWSYNC_STARTBIT:
      strcpy(buffer, "START   ");
      break;

    case SHOWSYNC_RECEIVING:
    case SHOWSYNC_RECEIVECOMPL:
      posdot = 0;
      posrev = DCF_SIZE - pos - 1;
      strcpy(buffer, "RECV    ");
      if (posrev >= 10) buffer[6] = '0' + posrev / 10;
      buffer[7] = '0' + posrev % 10;
      break;

    case SHOWSYNC_WAITSIGNAL:
      strcpy(buffer, "WAIT    ");
      pos = 0;
      break;

    case SHOWSYNC_NOSIGNAL:
      strcpy(buffer, "NOSIGNAL");
  }
  buffer[8] = 0;
  alpha.print(buffer);
}

//----------------------------------------------------------------------------------

void showTime(uint8_t mode, uint8_t hr, uint8_t min, uint8_t sec, uint8_t m, uint8_t d, bool dot, bool sync) {
  char buffer[DISPLAY_DIGITS + 1];

  // show time
  buffer[0] = hr >= 10 ? ('0' + hr / 10) : ' ';
  buffer[1] = ('0' + hr % 10) | (dot ? DOT : 0);
  buffer[2] = '0' + min / 10;
  buffer[3] = ('0' + min % 10) | (sync ? DOT : 0);

  switch (mode) {
    case SHOW_TIMEDATE:
      if (d >= 10 && m >= 10) {
        buffer[4] = '0' + d / 10;
        buffer[5] = ('0' + d % 10) | DOT;
        buffer[6] = '0' + m / 10;
        buffer[7] = ('0' + m % 10) | DOT;
      }
      if (d >= 10 && m < 10) {
        buffer[4] = ' ';
        buffer[5] = '0' + d / 10;
        buffer[6] = ('0' + d % 10) | DOT;
        buffer[7] = ('0' + m) | DOT;
      }
      if (d < 10 && m >= 10) {
        buffer[4] = ' ';
        buffer[5] = ('0' + d) | DOT;
        buffer[6] = '0' + m / 10;
        buffer[7] = ('0' + m % 10) | DOT;
      }
      if (d < 10 && m < 10) {
        buffer[4] = ' ';
        buffer[5] = ' ';
        buffer[6] = ('0' + d) | DOT;
        buffer[7] = ('0' + m) | DOT;
      }
      break;

    case SHOW_TIMEFULL:
      buffer[3] |= DOT;
      buffer[4] = '0' + sec / 10;
      buffer[5] = ('0' + sec % 10) | (sync ? DOT : 0);
      buffer[6] = ' ';
      buffer[7] = ' ';
      break;

#ifdef ONEWIRE
    case SHOW_TIMETEMP:
      bool neg;
      int16_t temp2;

      if (temp == TEMP_ERROR) {
        // temp reading error
        buffer[4] = '-';
        buffer[5] = '-' | DOT;
        buffer[6] = '-';
        buffer[7] = 'C';
        break;
      }
      if (temp < 0) {
        neg = true;
        temp2 = -temp;
      }
      else {
        neg = false;
        temp2 = temp;
      }
      if (neg) {
        buffer[4] = '-';
        buffer[5] = ('0' + temp2 / 100) | DOT;
      }
      else {
        if (temp2 >= 1000) buffer[4] = '0' + temp2 / 1000;
        else buffer[4] = ' ';
        buffer[5] = ('0' + ((temp2 / 100) % 10)) | DOT;
      }
      buffer[6] = '0' + (temp2 / 10) % 10;
      buffer[7] = 'C';
      break;
#endif

#ifdef VOLTAGE
    case SHOW_LOWBATT:
      buffer[4] = ' ';
      buffer[5] = ('0' + (vcc / 1000)) | DOT;
      buffer[6] = '0' + ((vcc / 100) % 10);
      buffer[7] = 'V';
      break;
#endif
  }
  buffer[8] = 0;
  alpha.print(buffer);
}

//----------------------------------------------------------------------------------

void RTCinit(void) {

  // enable external 32K clock when external RTC available
#if defined(__AVR_ATtiny1614__) && defined(RTCEXT)
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

#ifdef VOLTAGE
// configure internal ADC for reading own supply voltage
void ADCinit(void) {
#if defined(__AVR_ATtiny1614__)
  VREF.CTRLA = (VREF.CTRLA & ~VREF_ADC0REFSEL_gm) | VREF_ADC0REFSEL_1V1_gc;
  ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;
  ADC0.CTRLB = ADC_SAMPNUM_ACC64_gc;
  ADC0.CTRLC = ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV16_gc | ADC_SAMPCAP_bm;
  ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;
#endif
#if defined(__AVR_ATmega4809__)
  VREF.CTRLA = VREF_ADC0REFSEL_1V1_gc;
  //ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;
  ADC0.MUXPOS = 0x1C;
  ADC0.CTRLB = ADC_SAMPNUM_ACC64_gc;
  ADC0.CTRLC = ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV16_gc;
  ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc;
#endif
}

// the voltage is read by accumulating 64 samples
uint16_t measureVoltage(void) {
#if defined(__AVR_ATtiny1614__)
  ADC0.COMMAND = ADC_STCONV_bm;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
  ADC0.INTFLAGS = ADC_RESRDY_bm;
  uint16_t accumulatedVal = ADC0.RES;
  if (accumulatedVal == 0) return 0;
  return (uint16_t)(72019200UL / accumulatedVal);
#endif
#if defined(__AVR_ATmega4809__)
  ADC0.COMMAND = ADC_STCONV_bm;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
  ADC0.INTFLAGS = ADC_RESRDY_bm;
  uint16_t adcVal = ADC0.RES;
  if (adcVal == 0) return 0;
  //return 1125300 / adcVal;
  return (uint16_t)(72019200UL / adcVal);
#endif
}
#endif

// the RTC interrupt is called twice a seconds, increases internal time every second
ISR(RTC_PIT_vect) {
  RTC.PITINTFLAGS = RTC_PI_bm;
  tickTock = !tickTock;
}

// DCF77 interrupt handler, is called on every pulse edge
ISR(PORTA_PORT_vect) {
  if (PORTA.INTFLAGS & dcf.pinBitmaskDcf) {
    PORTA.INTFLAGS = dcf.pinBitmaskDcf;
    dcf.handleInt((PORTA.IN & dcf.pinBitmaskDcf) ? true : false);
  }
}

// on timer compare match, no DCF-signal is received, increase no-signal counter
ISR(TCA0_CMP0_vect) {
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_CMP0_bm;
  TCA0.SINGLE.CNT = 0;
  dcf.noSignal();
}
