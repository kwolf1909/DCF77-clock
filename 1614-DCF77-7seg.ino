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
  Timers used: TCA0 (DCF signal processing), RTC (2 Hz generation via interrupt),
               TCD0 (millis)
  External RTC: DS3231 with battery backup, supplies 32K clock for internal RTC

  Author: K. Wolf
  Date: Feb 27th 2026
*/
/*******************************************************************************/

#include <Wire.h>
#include <RTClib.h>
#include "7segfonttable.h"
#include "OneWire.h"

//#define SERIALDEBUG         1
//#define ONEWIRE             1
#define LED                   1
#define SUCCESS               0
#define ERROR_INVALID_VALUE  -1
#define TIMER_FREQ            7812  // 8 Mhz / 1024 = 7812
#define TIMER_CMPMATCH        15625 // timer ticks (2000 ms)
#define TIMER_TOP             32767
#define BIT_0_MIN_DURATION    156   // timer ticks (20 ms)
#define BIT_0_DURATION_LOW    625   // timer ticks (80 ms)
#define BIT_0_DURATION_HIGH   938   // timer ticks (120 ms)
#define BIT_1_DURATION_LOW    1406  // timer ticks (180 ms)
#define BIT_1_DURATION_HIGH   1719  // timer ticks (220 ms)
#define TIMEOUT_DURATION_LOW  13280 // timer ticks (1700 ms)
#define TIMEOUT_DURATION_HIGH 14843 // timer ticks (1900 ms)
#define DCF77_STRING_SIZE     59

// ----------------- 7-segment digit definitions --------------------------------------

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

#define NUM_DIGITS              8
#define DISPLAY_BRIGHTNESS      8

#ifdef MILLIS_USE_TIMERA0
#error "This sketch takes over TCA0 - please use a different timer for millis"
#endif

const uint8_t displayAddress = 0x70;
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

uint8_t bitArray[DCF77_STRING_SIZE];  // memory location for received DCF77 bit string
TimeStampDCF77 dCF77time;             // data type for decoded DCF77 string

bool      prevtock, lastDCFDecode, syncReq, syncTimeout, syncStatus;
uint8_t   timeState, showState, DCFstate, syncPos, prevPulse, vcc[2];
int16_t   tempRaw;
uint32_t  currentTime, resyncTime, lastButtonTime, lastReceiveTime;

volatile bool minuteMarker, startBit, receiveBit, receiveComplete, ticktock, button;
volatile uint8_t receiveState, pulseType, DCFpos;
volatile uint16_t lengthPulse, lengthPause;

DateTime dt(2026, 1, 1, 0, 0, 0);
RTC_DS3231 rtc;

enum { TIME_NOTIME = 1, TIME_RTC, TIME_SYNC, TIME_SYNCED, TIME_RESYNC };
enum { DCF_RECEIVING = 1, DCF_RECEIVECOMPLETE, DCF_SYNCTIME };
enum { RECEIVE_MINUTEMARKER = 1, RECEIVE_STARTBIT, RECEIVE_RECEIVING };
enum { PULSE_START = 1, PULSE_END };
enum { SHOW_TIMEDATE = 1, SHOW_TIMEFULL, SHOW_TIMETEMP, SHOW_LOWBATT };

//----------------------------------------------------------------------------------

void setup() {
#ifdef SERIALDEBUG
  Serial.swap(1);         // use PA1(Tx) and PA2 (Rx)
  Serial.begin(115200);
  Serial.println("\r\nInit...");
#endif

  // DCF signal input with pullup on PA3
  PORTA.DIRCLR = PIN3_bm;
  PORTA.PIN3CTRL = PORT_PULLUPEN_bm;
  PORTA.PIN3CTRL |= PORT_ISC_BOTHEDGES_gc;

  // LED on PA5
  PORTA.DIRSET = PIN5_bm;
  PORTA.OUTCLR = PIN5_bm;

  // Button on PA6
  PORTA.DIRCLR = PIN6_bm;
  PORTA.PIN6CTRL = PORT_PULLUPEN_bm;
  PORTA.PIN6CTRL |= PORT_ISC_FALLING_gc;

  // Pullup for XOSC1
  PORTA.DIRSET = PIN7_bm;
  PORTA.OUTSET = PIN7_bm;

  Wire.begin();

  delay(50);
  initDisplay(DISPLAY_BRIGHTNESS);
  printDisplay("INIT    ");
  delay(1000);

#ifdef ONEWIRE
  // OneWire on pin PA4
  oneWireSetup(4);
  setResolution(11);
  startConversion();
  delay(400);
  tempRaw = readTemperature();
#endif
  
  timeState = TIME_NOTIME;
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
  }

  RTCinit();
  ADCinit();
  delay(50);
  measureVoltage();
  setupDCF77();
  
  DCFstate = DCF_RECEIVING;
  showState = SHOW_TIMEDATE;
  receiveState = RECEIVE_MINUTEMARKER;
  prevPulse = PULSE_END;
  DCFpos = 0;
  syncPos = 0;
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
    Serial.println("\n\rMinute Marker detected");
    minuteMarker = false;
  }
  if (startBit) {
    Serial.println("Startbit detected");
    startBit = false;
  }
  if (receiveBit && DCFpos) {
    if (bitArray[DCFpos - 1] == 0) Serial.print("0");
    if (bitArray[DCFpos - 1] == 1) Serial.print("1");
    receiveBit = false;
  }
#endif

  // show sync animation during first sync
  if (timeState == TIME_SYNC) {
    if (prevPulse != pulseType) {
      prevPulse = pulseType;
      if (pulseType == PULSE_START) showSync(syncPos, true);
      if (pulseType == PULSE_END) showSync(syncPos++, false);
      if (syncPos > 3) syncPos = 0;
    }
  }

  // read button and debounce
  if (button) {
    button = false;
    if (currentTime - lastButtonTime > buttonDelay) {
      lastButtonTime = currentTime;
#ifdef SERIALDEBUG
      Serial.println("Button detected");
#endif
      switch (showState) {
        case SHOW_TIMEDATE: showState = SHOW_TIMEFULL; break;
        case SHOW_TIMEFULL: showState = SHOW_LOWBATT; break;
        case SHOW_LOWBATT: showState = SHOW_TIMEDATE; break;
      }
    }
  }

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
        rtc.adjust(dt);

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

    //  measureVoltage();
    //  if (vcc[0] < 3) showState = SHOW_LOWBATT;

    switch (timeState) {
      case TIME_NOTIME:
        // with no RTC-time, wait for DCF-time
#ifdef SERIALDEBUG
        Serial.println("Waiting for DCF77 time...");
#endif
        syncReq = true;
        syncStatus = false;
        timeState = TIME_SYNC;
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
        }
        showTime(showState, dt.hour(), dt.minute(), dt.second(), dt.month(), dt.day(), ticktock, syncStatus);
        break;
      
      case TIME_SYNCED:
        if (currentTime - resyncTime > resyncDelay && syncReq == false) {
          // request DCF update
          resyncTime = currentTime;
          syncReq = true;
        }
        // calculate new sync status
        if (syncReq && syncTimeout) syncStatus = false;
        else syncStatus = true;

        showTime(showState, dt.hour(), dt.minute(), dt.second(), dt.month(), dt.day(), ticktock, syncStatus);
        break;
    }
  }
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
  if (PORTA.INTFLAGS & PIN3_bm) {
    PORTA.INTFLAGS = PIN3_bm;

    // filter noise
    if (TCA0.SINGLE.CNT < BIT_0_MIN_DURATION) return;

    if (PORTA.IN & PIN3_bm) {
      // edge low to high
#if LED
      PORTA.OUTCLR = PIN5_bm;
#endif
      lengthPulse = TCA0.SINGLE.CNT;
      TCA0.SINGLE.CNT = 0;
      pulseType = PULSE_END;
    } else {
      // edge high to low
#if LED
      PORTA.OUTSET = PIN5_bm;
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
            if (DCFpos == DCF77_STRING_SIZE) receiveComplete = true;
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
          if (++DCFpos == DCF77_STRING_SIZE) receiveState = RECEIVE_MINUTEMARKER;
        }
        break;
    }
  }

  // detect button press
  if (PORTA.INTFLAGS & PIN6_bm) {
    PORTA.INTFLAGS = PIN6_bm;
    button = true;
  }
}

ISR(TCA0_CMP0_vect) {
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_CMP0_bm;
  TCA0.SINGLE.CNT = 0;

  //PORTA.OUTTGL = PIN5_bm;
}

// the RTC interrupt is called twice a seconds
ISR(RTC_PIT_vect) {
  RTC.PITINTFLAGS = RTC_PI_bm;
  ticktock = !ticktock;
  if (ticktock == true) dt = dt + 1;

  //PORTA.OUTTGL = PIN5_bm;
}

//----------------------------------------------------------------------------------

void initDisplay (uint8_t bright) {

  Wire.beginTransmission(displayAddress);
  Wire.write(0x21);
  Wire.endTransmission();

  Wire.beginTransmission(displayAddress);
  Wire.write(0x81);
  Wire.endTransmission();

  Wire.beginTransmission(displayAddress);
  Wire.write(0xE0 | (bright & 0x0F));
  Wire.endTransmission();
}
/*
  void writeWord (uint8_t b) {

  Wire.write(b);
  Wire.write(0);
  }

  void blinkRate(uint8_t b) {

  if (b > 3) b = 0; // turn off if not sure
  Wire.beginTransmission(displayAddress);
  Wire.write(HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (b << 1));
  Wire.endTransmission();
  }
*/
// writes the raw display codes
void showDisplay (char *d) {

  Wire.beginTransmission(displayAddress);
  Wire.write(0);

  for (uint8_t i = 0; i < NUM_DIGITS; i++) {
    Wire.write(d[i]);
    Wire.write(0);
  }
  Wire.endTransmission();
}

// writes a string to display
void printDisplay (const char *s) {
  uint8_t i, pos, code;

  Wire.beginTransmission(displayAddress);
  Wire.write(0);

  i = 0;
  pos = 0;
  while (s[i] && pos < NUM_DIGITS) {

    // write ascii number with colon support
    code = getFont(s[i]);
    if (s[i + 1] == '.') {
      code |= COLON;
      i++;
    }
    Wire.write(code);
    Wire.write(0);
    i++;
    pos++;
  }
  Wire.endTransmission();
}

uint8_t getFont(uint8_t c) {
  if (c < 32) return 0;
  return sevensegfonttable[c - 32];
}

void showSync(uint8_t pos, bool sync) {
  char buffer[8];

  if (pos > 3) return;
  buffer[0] = getFont('S');
  buffer[1] = getFont('Y');
  buffer[2] = getFont('N');
  buffer[3] = getFont('C');

  buffer[4] = buffer[5] = buffer[6] = buffer[7] = 0;
  if (sync)
    buffer[4 + pos] = COLON;

  showDisplay(buffer);
}

void showTime(uint8_t mode, uint8_t hr, uint8_t min, uint8_t sec, uint8_t m, uint8_t d, bool colon, bool sync) {
  char buffer[8];
  uint8_t temp, tenth;

  // show time
  buffer[0] = getFont('0' + hr / 10);
  buffer[1] = getFont('0' + hr % 10) | (colon ? COLON : 0);
  buffer[2] = getFont('0' + min / 10);
  buffer[3] = getFont('0' + min % 10) | (sync ? 0 : COLON);

  switch (mode) {
    case SHOW_TIMEDATE:
      if (d >= 10 && m >= 10) {
        buffer[4] = getFont('0' + d / 10);
        buffer[5] = getFont('0' + d % 10) | COLON;
        buffer[6] = getFont('0' + m / 10);
        buffer[7] = getFont('0' + m % 10) | COLON;
      }
      if (d >= 10 && m < 10) {
        buffer[4] = 0;
        buffer[5] = getFont('0' + d / 10);
        buffer[6] = getFont('0' + d % 10) | COLON;
        buffer[7] = getFont('0' + m) | COLON;
      }
      if (d < 10 && m >= 10) {
        buffer[4] = 0;
        buffer[5] = getFont('0' + d) | COLON;
        buffer[6] = getFont('0' + m / 10);
        buffer[7] = getFont('0' + m % 10) | COLON;
      }
      if (d < 10 && m < 10) {
        buffer[4] = 0;
        buffer[5] = 0;
        buffer[6] = getFont('0' + d) | COLON;
        buffer[7] = getFont('0' + m) | COLON;
      }
      break;

    case SHOW_TIMEFULL:
      buffer[4] = getFont('0' + sec / 10);
      buffer[5] = getFont('0' + sec % 10);
      buffer[6] = 0;
      buffer[7] = 0;
      break;

    case SHOW_TIMETEMP:
      temp = tempRaw >> 4;
      tenth = ((tempRaw & 0x0F) * 10) >> 4;
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
      buffer[7] = getFont('C');
      break;

    case SHOW_LOWBATT:
      buffer[4] = 0;
      buffer[5] = 0;
      buffer[6] = getFont('0' + vcc[0]) | COLON;
      buffer[7] = getFont('0' + vcc[1]);
      break;
  }
  showDisplay(buffer);
}

void RTCinit(void) {

  uint8_t temp = CLKCTRL.XOSC32KCTRLA & ~CLKCTRL_ENABLE_bm;
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.XOSC32KCTRLA = temp;
  while (CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm);

  //temp = CLKCTRL.XOSC32KCTRLA & ~CLKCTRL_SEL_bm;
  temp = CLKCTRL.XOSC32KCTRLA | CLKCTRL_SEL_bm;
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.XOSC32KCTRLA = temp;

  temp = CLKCTRL.XOSC32KCTRLA | CLKCTRL_ENABLE_bm;
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.XOSC32KCTRLA = temp;

  while (RTC.STATUS > 0);
  //RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;
  RTC.CLKSEL = RTC_CLKSEL_TOSC32K_gc;
  RTC.PITCTRLA = RTC_PERIOD_CYC16384_gc | RTC_PITEN_bm;
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
