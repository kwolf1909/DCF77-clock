#include <Wire.h>
#include <OneWire.h>
#include <DS18B20_INT.h>
#include <OneButtonTiny.h>
#include <EEPROM.h>
#include "RotaryEncoder.h"
#include "DateTime.h"
#include "Display.h"

//#define DEBUGSERIAL

#if defined (__AVR_ATtiny84__)
#define ROTARY_A        0   // PB0
#define ROTARY_B        1   // PB1
#define ONEWIRE_PIN     3   // PA3
#define BUZZER_PIN      5   // PA5
#define BUTTON_PIN      7   // PA7
#else
#error "Wrong board selected!"
#endif

// ----------------------------- globals ----------------------------------------------

#define TEMP_DELAY          5000
#define TEMP_REQDELAY       1000
#define FLASH1_DELAY        250
#define FLASH2_DELAY        150
#define BUZZER_DELAY        200
#define BEEP_COUNT          80
#define TIMER_TICK          500
#define DISPLAY_BRIGHTNESS  4
#define DISPLAY_DIGITS      5
#define DISPLAY_ADDRESS     0x70

bool bShow, bColon, newVal, buttonPressed, buttonLongPressed, buzzer, buzzerCal;
int8_t rotaryDelta;
uint8_t timerMinutes, timerSeconds, timerState, buzzerState, min, sec, beepCount, oscVal, buzzerCount, digits;
int16_t temp;
uint16_t buzzerDuration;
uint32_t curTime, lastTempTime, lastTempReqTime, lastFlashTime, lastTimer;

enum { STATE_REQTEMP = 1, STATE_REQTEMPWAIT, STATE_SHOWTEMP, STATE_WAITTEMP, STATE_SELMIN, STATE_SELSEC,
       STATE_SELOSC, STATE_RUNTIMER, STATE_TIMERBEEP
     };
enum { BUZZER_IDLE = 1, BUZZER_START, BUZZER_ON, BUZZER_WAIT, BUZZER_CALSTART, BUZZER_CAL };

OneWire oneWire(ONEWIRE_PIN);
DS18B20_INT sensor(&oneWire);
OneButtonTiny button(BUTTON_PIN, true, true);
RotaryEncoder *rotary = nullptr;
Display display7;
DateTime dt(2026, 1, 1, 0, 0, 0);

#ifdef DEBUGSERIAL
char sDebug[64];
#endif

// ------------------------------- Setup ------------------------------------

void setup() {

  uint8_t cal = EEPROM.read(E2END);
  if (cal < 0xff) OSCCAL = cal;

  delay(100);
#ifdef DEBUGSERIAL
  Serial.begin(9600);
  Serial.println("Init...");
#endif

  Wire.begin();

  display7.init(DISPLAY_ADDRESS, DISPLAY_DIGITS, DISPLAY_BRIGHTNESS);
  display7.print("IN IT");
  
  rotary = new RotaryEncoder(RotaryEncoder::LatchMode::FOUR3);

  // setup input pins with pull-ups
  DDRB &= ~(1 << ROTARY_A | 1 << ROTARY_B);
  PORTB |= (1 << ROTARY_A | 1 << ROTARY_B);

  // init rotary decoder PCINTs
  PCMSK1 = (1 << ROTARY_A | 1 << ROTARY_B);
  GIMSK = 1 << PCIE1;
  GIFR = 1 << PCIF1;

  timerMinutes = 0;
  timerSeconds = 0;
  buzzer = false;
  buzzerCal = false;
  buttonPressed = false;
  buttonLongPressed = false;
  timerState = STATE_REQTEMP;
  buzzerState = BUZZER_IDLE;

  // Initialize Timer1 for 1000 Hz generation
  setupTimer1();
  triggerBuzzer(50, 2);

  sensor.begin();
  sensor.setResolution(11);

  // click event lambda
  button.attachClick([]() {
    buttonPressed = true;
  });
  button.attachLongPressStart([]() {
    buttonLongPressed = true;
  });

#ifdef DEBUGSERIAL
  // Read Vcc
  uint16_t volt = readVCC();
  //sprintf(sDebug, "Voltage: %u mV", volt);
  //Serial.println(sDebug);
#endif
}

// ------------------------------- Loop -------------------------------------

void loop () {

  curTime = millis();
  button.tick();

  min = dt.minute();
  sec = dt.second();

  //------------------- handle rotary encoder ------------------------
  rotaryDelta = 0;
  RotaryEncoder::Direction dir = rotary->getDirection();
  if (dir == RotaryEncoder::Direction::COUNTERCLOCKWISE) rotaryDelta = -1;
  if (dir == RotaryEncoder::Direction::CLOCKWISE) rotaryDelta = 1;

  //------------------ process state machines -------------------------
  timerState = handleButton(timerState);
  timerState = handleTimerState(timerState);
  buzzerState = handleBuzzerState(buzzerState);

  delay(10);
}

//----------------------- helper function -----------------------------

uint8_t handleTimerState(uint8_t state) {
  switch (state) {
    case STATE_REQTEMP:
#ifdef DEBUGSERIAL
      Serial.println("Requesting temperature...");
#endif
      sensor.requestTemperatures();
      lastTempReqTime = curTime;
      return STATE_REQTEMPWAIT;
      break;

    case STATE_REQTEMPWAIT:
      // check for conversion result
      if (curTime - lastTempReqTime > TEMP_REQDELAY) {
        if (!sensor.isConversionComplete()) {
          // conversion not complete
          lastTempReqTime = curTime;
          break;
        }
        temp = sensor.getTempCentiC();
        return STATE_SHOWTEMP;
      }
      break;

    case STATE_SHOWTEMP:
#ifdef DEBUGSERIAL
      //sprintf(sDebug, "Temperature: %d C", temp);
      //Serial.println(sDebug);
#endif
      showDisplay (temp / 1000, (temp / 100) % 10, (temp / 10) % 10, 0, 0b1111, false, true);
      lastTempTime = curTime;
      return STATE_WAITTEMP;

    case STATE_WAITTEMP:
      if (curTime - lastTempTime > TEMP_DELAY) return STATE_REQTEMP;
      break;

    case STATE_SELMIN:
      // grab new value
      newVal = false;
      if (rotaryDelta) {
        if (rotaryDelta == 1 && timerMinutes == 59 && timerSeconds == 0) {
          timerSeconds = 59;
          newVal = true;
        }
        if (rotaryDelta == 1 && timerMinutes < 59) {
          timerMinutes++;
          newVal = true;
        }
        if (rotaryDelta == -1 && timerMinutes > 0) {
          timerMinutes--;
          newVal = true;
        }
      }
      if (newVal && rotaryDelta) {
        triggerBuzzer(50, 1);
        bShow = true;
      }

      // flash minutes
      if (curTime - lastFlashTime > FLASH1_DELAY || newVal) {
        lastFlashTime = curTime;
        if (bShow) digits = 0b1111; else digits = 0b0011;
        showDisplay(timerMinutes / 10, timerMinutes % 10, timerSeconds / 10, timerSeconds % 10, digits, true, false);
        bShow = !bShow;
      }
      break;

    case STATE_SELSEC:
      // grab new value
      newVal = false;
      if (rotaryDelta) {
        if (rotaryDelta == 1 && timerSeconds < 59) {
          timerSeconds++;
          newVal = true;
        }
        if (rotaryDelta == -1 && timerSeconds > 0) {
          timerSeconds--;
          newVal = true;
        }
      }
      if (newVal && rotaryDelta) {
        triggerBuzzer(50, 1);
        bShow = true;
      }

      // flash minutes
      if (curTime - lastFlashTime > FLASH1_DELAY || newVal) {
        lastFlashTime = curTime;
        if (bShow) digits = 0b1111; else digits = 0b1100;
        showDisplay(timerMinutes / 10, timerMinutes % 10, timerSeconds / 10, timerSeconds % 10, digits, true, false);
        bShow = !bShow;
      }
      break;

    case STATE_SELOSC:
      // grab new value
      if (rotaryDelta) {
        if (rotaryDelta == 1 && oscVal < 255) oscVal++;
        if (rotaryDelta == -1 && oscVal) oscVal--;
        OSCCAL = oscVal;
        showDisplay (0, (oscVal / 100) % 10, (oscVal / 10) % 10, oscVal % 10, 0x0111, false, false);
      }
      break;

    case STATE_RUNTIMER:
      // update every half second
      if (curTime > lastTimer) {
        lastTimer += TIMER_TICK;
        showDisplay(dt.minute() / 10, dt.minute() % 10, dt.second() / 10, dt.second() % 10, 0b1111, bColon, false);
        if (!dt.minute() && !dt.second()) {
          beepCount = BEEP_COUNT;
          lastFlashTime = curTime;
          return STATE_TIMERBEEP;
        }
        if (bColon) {
          // trigger minute beeps
          if (dt.minute() <= 5 && dt.second() == 0) triggerBuzzer(100, dt.minute());
          // trigger seconds beeps
          if (dt.minute() == 0 && dt.second() && dt.second() <= 10) triggerBuzzer(100, 1);
        }
        else dt = dt - (TimeSpan)1;
        bColon = !bColon;
      }
      break;

    case STATE_TIMERBEEP:
      if (curTime - lastFlashTime > FLASH2_DELAY) {
        lastFlashTime = curTime;
        showDisplay(0, 0, 0, 0, bShow ? 0b1111 : 0, bShow, false);
        bShow = !bShow;
        triggerBuzzer(100, 1);

        // exit after number of beeps
        if (--beepCount == 0) return STATE_REQTEMP;
      }
      break;
  }
  return state;
}

uint8_t handleButton(uint8_t state) {

  switch (state) {
    case STATE_SHOWTEMP:
    case STATE_WAITTEMP:
      if (buttonPressed) {
        buttonPressed = false;
        bShow = true;
        timerMinutes = 0;
        timerSeconds = 0;
        triggerBuzzer(50, 1);
        return STATE_SELMIN;
      }
      if (buttonLongPressed) {
        buttonLongPressed = false;
        buzzerCal = true;
        oscVal = OSCCAL;
        showDisplay (0, (oscVal / 100) % 10, (oscVal / 10) % 10, oscVal % 10, 0x7, false, false);
        return STATE_SELOSC;
      }
      break;

    case STATE_SELMIN:
      if (buttonPressed) {
        buttonPressed = false;
        bShow = true;
        triggerBuzzer(50, 1);
        return STATE_SELSEC;
      }
      break;

    case STATE_SELSEC:
      if (buttonPressed) {
        buttonPressed = false;
        if (timerMinutes || timerSeconds) {
          // set timer start condition
          dt = DateTime(2026, 1, 1, 0, timerMinutes, timerSeconds);
          lastTimer = curTime + TIMER_TICK;
          triggerBuzzer(50, 1);
          bColon = true;
          return STATE_RUNTIMER;
        }
        else {
          triggerBuzzer(50, 1);
          delay(100);
          return STATE_SHOWTEMP;
        }
      }
      break;

    case STATE_SELOSC:
      if (buttonPressed) {
        buttonPressed = false;
        buzzerCal = false;
        EEPROM.write(E2END, oscVal);
        return STATE_SHOWTEMP;
      }
      break;

    case STATE_RUNTIMER:
      if (buttonPressed) {
        buttonPressed = false;
        timerMinutes = 0;
        timerSeconds = 0;
        triggerBuzzer(50, 1);
        delay(100);
        return STATE_SHOWTEMP;
      }
      break;

    case STATE_TIMERBEEP:
      if (buttonPressed) {
        buttonPressed = false;
        delay(100);
        return STATE_SHOWTEMP;
      }
  }
  return state;
}

uint8_t handleBuzzerState(uint8_t state) {

  static uint32_t buzzerTime;

  switch (state) {
    case BUZZER_IDLE:
      if (buzzer) return BUZZER_START;
      if (buzzerCal) return BUZZER_CALSTART;
      break;

    case BUZZER_START:
      // Setup buzzer
      buzzerTime = millis();
      DDRA |= 1 << BUZZER_PIN;
      return BUZZER_ON;

    case BUZZER_ON:
      if (millis() - buzzerTime > buzzerDuration) {
        DDRA &= ~(1 << BUZZER_PIN);
        buzzerTime = millis();
        buzzerCount--;
        return BUZZER_WAIT;
      }
      break;

    case BUZZER_WAIT:
      if (buzzerCount == 0) {
        buzzer = false;
        return BUZZER_IDLE;
      }
      if (millis() - buzzerTime > BUZZER_DELAY) return BUZZER_START;
      break;

    case BUZZER_CALSTART:
      DDRA |= 1 << BUZZER_PIN;
      return BUZZER_CAL;

    case BUZZER_CAL:
      if (buzzerCal == false) {
        DDRA &= ~(1 << BUZZER_PIN);
        return BUZZER_IDLE;
      }
      break;
  }
  return state;
}

void showDisplay(uint8_t digit1, uint8_t digit2, uint8_t digit3, uint8_t digit4, uint8_t showDigit, bool colon, bool temp) {
  char disp[DISPLAY_DIGITS + 1];

  for (uint8_t i = 0; i < DISPLAY_DIGITS; i++) disp[i] = 0;
  
  if (temp) {
    // show temperature
    disp[0] = (showDigit & 0b1000) ? '0' + digit1 : ' ';
    disp[1] = (showDigit & 0b0100) ? ('0' + digit2) | DOT : ' ';
    disp[2] = ' ';
    disp[3] = (showDigit & 0b0010) ? '0' + digit3 : ' ';
    disp[4] = (showDigit & 0b0001) ? 'C' : ' ';
  }
  else {
    // show timer value
    disp[0] = (showDigit & 0b1000) ? '0' + digit1 : ' ';
    disp[1] = (showDigit & 0b0100) ? '0' + digit2 : ' ';
    disp[2] = ' ' | (colon ? COLON : 0);
    disp[3] = (showDigit & 0b0010) ? '0' + digit3 : ' ';
    disp[4] = (showDigit & 0b0001) ? '0' + digit4 : ' ';
  }
  disp[5] = 0;
  display7.print(disp);
}

void setupTimer1() {
  // create 1000 Hz by Timer 1 (CTC-mode), output on OC1B
  TCCR1A = (1 << COM1B0);
  TCCR1B = (1 << WGM12) | (1 << CS10);
  OCR1A = 3999;
  OCR1B = 3999;
  // disable output OC1B
  DDRA &= ~(1 << BUZZER_PIN);
}

void triggerBuzzer(uint16_t duration, uint8_t count) {

  // setup buzzer cycle
  if (!buzzer && count) {
    buzzerDuration = duration;
    buzzerCount = count;
    buzzer = true;
  }
}

uint16_t readVCC(void) {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  ADMUX =  _BV(MUX5) | _BV(MUX0);
  ADCSRA = (1 << ADPS2) | (1 << ADPS1); // prescaler of 64 = 8MHz/64 = 125KHz.
  delay(2);                         // Wait for Vref to settle
  ADCSRA |= (1 << ADEN);            // Enable ADC
  delay(2);
  ADCSRA |= _BV(ADSC);              // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low = ADCL;               // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH;              // unlocks both
  uint32_t result = (high << 8) | low;

  result = 1126400L / result;       // Calculate Vcc (in mV); 1125300 = 1100*1024
  return result;                    // Vcc in millivolts
}

//-------------------------------- ISR Rotary ----------------------------------

ISR(PCINT1_vect) {
  uint8_t pin = PINB;
  uint8_t a = (pin >> ROTARY_A) & 1;
  uint8_t b = (pin >> ROTARY_B) & 1;

  // handle rotary encoder changes
  rotary->tick(a, b);
}

// ------------------------------- End -----------------------------------------
