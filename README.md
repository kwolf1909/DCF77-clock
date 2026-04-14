This program receives the european DCF77 time signal and syncs it with the
external RTC-clock. It is displayed on a 8-digit 7-segment display with HT16K33
controller.
The MCU used is an ATtiny 814 or 1614. Without serial debugging and OneWire,
an ATtiny 412 can be used. The MCU used is selected by setting the appropriate
#define statement.
The circuit can be powered by a LiPo-cell. If the voltage drops below 3.0 V,
the voltage is displayed as a low voltage indicator.
A push button switches between different display modes:
Time with seconds, time with date, time with battery voltage

MCU-clock: 8 MHz,
Timers used: TCA0 (DCF signal processing), RTC (2 Hz generation via interrupt),
             TCD0 (millis)
External RTC: DS3231 with battery backup, supplies 32K clock for internal RTC,
7-seg display: DFRobot 8-digit display with I2C interface,
External DCF77-receiver: ELV DCF-2 (MAS6180 AM-receiver)

Compiles with MegaTinyCore on Arduino IDE.
External libraries required: RTClib.h (Adafruit fork)
