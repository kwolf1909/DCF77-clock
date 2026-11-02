## Introduction
This application receives the european DCF77 time signal and syncs it with the external RTC-clock. It is displayed on a 8-digit 14- or 7-segment display with display-controller HT16K33.<br>
The MCU used is an ATtiny814/1614 or an ATmega4809 (Arduino Nano Every). Without external RTC, serial debugging and OneWire, an ATtiny412 can be
used (98 % of flash space used). With some small modifications, the newer AVR Dx-series could be used as well.<br>
The circuit can be powered by a Li-Ion cell. If the voltage drops below 3.0 V, the voltage is displayed as a low voltage indicator.<br>
The temperature can be read by internal sensor of RTC or external OneWire-based sensor.<br>
A sync to DCF-time is performed in the background every 30 minutes, and immediately after power-up. The DCF-module is powered only during sync-cycle.<br>
The software uses a layer-based approach. Low-level: DCF-object with interrupt handler, mid-level: receiving data state machine, upper level: display handling and user interface. No blocking code, using several finite state machines (FSM).<br>

Push-button based interface<br>
Select display modes: time with date | time with seconds | time with temperature | time with battery voltage.<br>

MCU-clock: 8 MHz<br>
Timers used: TCB0 (DCF pulse width measurement), RTC (2 Hz clock ticks)

Compiles with MegaTinyCore (ATtiny814/1614/412) or MegaAVR-core/MegaCoreX (Arduino Nano Every) on Arduino IDE.<br>

## Pins used (ATtiny814/1614)
Pins used (ATtiny814/1614):<br>
PA1 - Output, TX for serial debugging.<br>
PA2 - Input, RX (not used).<br>
PA3 - Input, DCF-signal, active-low signal, external pull-up 10 kilohm required, external RC low-pass filter recommended.<br>
PA4 - Input/Output, OneWire data, external pull-up 10 kilohm required.<br>
PA5 - Output, LED, mirrors the DCF-input signal in sync-state, active high.<br>
PA6 - Output, DCF-module power, connect to VCC of module.<br>
PA7 - Input, mode select button, active low with input-pullup enabled<br>
PB0 - I2C CLK for display & RTC.<br>
PB1 - I2C DATA for display & RTC.<br>
TOSC1 - 32K clock-signal from external RTC, external pull-up 10 kilohm required.<br>

### Notes for Arduino Nano Every (ATmega4809)
Actually there is no way to feed the external 32K clock signal from the RTC because TOSC1-pin is not accessible. Hence, the internal 32K clock is used.
The voltage measurement for the own supply voltage is done by using an external voltage divider using two resistors 56 kilohm (VCC) + 33 kilohm (GND). The divided voltage is fed into pin AIN2.<br>

## Serial Monitor
A serial monitor output (115200 baud) can be enabled by uncommenting SERIALDEBUG. Due to limited flash space on ATtiny814, this requires some other functions to be disabled temporarily, e.g. OneWire. This shows an output for a complete receiving-cycle as an example.
```
DCF: Minute marker detected
DCF: Startbit detected, receiving now...
0000000000000000100100000000100010000010000100001011001000
DCF: Receive complete
Receive: Receive complete
Receive: Decode successful
Receive: RTC synced to DCF-time
```
## Hardware
[DS3231 RTC Precision Breakout](https://www.adafruit.com/product/3013)<br>
[DFRobot 7-segment 8-digit display](https://www.dfrobot.com/product-1978.html) or [Custom built alphanumeric display](http://www.technoblogy.com/show?2ULE)<br>
DCF77-receiver: ELV DCF-2 (MAS6180-based AM-receiver), any other receiver possible with digital signal output.<br>
Temperature-sensor: DS18B20<br>
Resistors: 1x 1 kilohm (LED), 3x 10 kilohm (pullups)<br>
[Alphanumeric displays](https://www.adafruit.com/product/2154) used on board:<br>

<img width="640" height="214" alt="alpha" src="https://github.com/user-attachments/assets/6b047f4b-43a1-4883-a1a5-ab0c732ea372" /><br>
