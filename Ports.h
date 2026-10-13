#ifndef __PORTS_H
#define __PORTS_H
#if defined (__AVR_ATtiny412__)
PORT_t* const port[] = { &PORTA };
#endif
#if defined (__AVR_ATtiny1614__)
PORT_t* const port[] = { &PORTA, &PORTB };
#endif
#if defined (__AVR_ATmega4809__)
PORT_t* const port[] = { 0x0400, 0x0420, 0x0440, 0x0460, 0x0480, 0x04A0 };
#endif
#endif
