This program receives the european DCF77 time signal and syncs it with the
external RTC-clock. It is displayed on a 8-digit 14- or 7-segment display
with display-controller HT16K33.
The MCU used is an ATtiny1614 or an ATmega4809 (Arduino Nano Every). Without external RTC, serial debugging and OneWire, an ATtiny412 can be
used (99 % of flash space used). With some small modifications, the newer AVR-series like the AVR32DA28 could be used as well.
The circuit can be powered by a Li-Ion cell. If the voltage drops below 3.0 V,
the voltage is displayed as a low voltage indicator.
A sync to DCF-time is performed in the background every 30 minutes, and immediately
after power-up.
The software uses a layer-based approach. Low-level: DCF-object with interrupt handler,
mid-level: receiving data state machine, upper level: display handling and user interface,
No blocking code, using several finite state machines (FSM).

Push-button based interface<br>
Short-press: select display modes: time with date | time with seconds | time with temperature | time with battery voltage.<br>
Long-press: manually invoke DCF77-resync.<br>

MCU-clock: 8 MHz<br>
Timers used: TCA0 (DCF pulse width measurement), TCB0 (OneWire timing), TCB3/TCD0 (millis), RTC (2 Hz periodic interrupt)
             
External RTC: DS3231 with battery backup, supplies 32K clock for internal RTC,
8-digit display (I2C): DFRobot 7-segment, or custom built 14-segment.
External DCF77-receiver: ELV DCF-2 (MAS6180 AM-receiver).
Temperature-sensor: DS18B20 with external pullup resistor 4,7k ohms.

Pins used (ATtiny1614):<br>
PA1 - Output, TX for serial debugging<br>
PA3 - Input, DCF-signal, input-pullup enabled, active-low signal<br>
PA4 - Input/Output, temp sensor DS18B20<br>
PA6 - Output, LED which reflects the DCF-input signal<br>
PA7 - Input, mode select button, active low with input-pullup enabled<br>
PB0 - I2C CLK for display<br>
PB1 - I2C DATA for display<br>

Compiles with MegaTinyCore on Arduino IDE.<br>
External libraries required: RTClib (Adafruit fork)<br>

Notes for ATmega4809 (Arduino Nano Every):<br>
Actually there is no way to feed the external 32K clock signal from the RTC because TOSC1-pin is not accessible. Hence, the internal 32K clock is used.
The voltage measurement for the own supply voltage is not working yet, therefore the functionality is disabled here.<br>

DFRobot 7-segment 8-digit display: https://www.dfrobot.com/product-1978.html<br>
Custom built alphanumeric display: http://www.technoblogy.com/show?2ULE<br>
Alphanumeric displays used on board: https://www.adafruit.com/product/2154

<img width="640" height="214" alt="alpha" src="https://github.com/user-attachments/assets/6b047f4b-43a1-4883-a1a5-ab0c732ea372" />
