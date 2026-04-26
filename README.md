This program receives the european DCF77 time signal and syncs it with the
external RTC-clock. It is displayed on a 8-digit 14- or 7-segment display
with display-controller HT16K33.
The MCU used is an ATtiny 814 or 1614. Without serial debugging and OneWire,
an ATtiny 412 can be used. The MCU used is selected by setting the appropriate
#define statement.
The circuit can be powered by a LiPo-cell. If the voltage drops below 3.0 V,
the voltage is displayed as a low voltage indicator.
A push button switches between different display modes:
Time with date, time with seconds, time with temperature, time with battery voltage

MCU-clock: 8 MHz,
Timers used: TCA0 (DCF signal processing), TCB0 (timing for OneWire),
             TCD0 (millis), RTC (2 Hz generation via interrupt)
             
External RTC: DS3231 with battery backup, supplies 32K clock for internal RTC,
8-digit display (I2C): DFRobot 7-segment, or custom built 14-segment.
External DCF77-receiver: ELV DCF-2 (MAS6180 AM-receiver)
Temperature-sensor: DS18B20 with external pullup

Pins used:<br>
PA3 - Input, DCF-signal, input-pullup enabled, low-active signal<br>
PA4 - Input/Output, used by tiny OneWire-implementation (temp sensor DS18B20),
      external pullup 4,7K<br>
PA6 - Output, LED which reflects the DCF-input signal<br>
PA7 - Input, mode select button, active low with input-pullup enabled<br>
PB0 - I2C CLK for display<br>
PB1 - I2C DATA for display<br>

Custom built alphanumeric display used: http://www.technoblogy.com/show?2ULE

<img width="640" height="214" alt="alpha" src="https://github.com/user-attachments/assets/6b047f4b-43a1-4883-a1a5-ab0c732ea372" />
<br>
Compiles with MegaTinyCore on Arduino IDE.
External libraries required: RTClib.h (Adafruit fork)
