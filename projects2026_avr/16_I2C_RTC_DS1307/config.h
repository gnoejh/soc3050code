/*
 * config.h - I2C and a Real-Time Clock
 * SOC3050 ATmega128 Educational Framework, 2026 AVR edition
 *
 * Focus: TWI start, stop and acknowledge, and talking to a DS1307
 */

#ifndef CONFIG_H_
#define CONFIG_H_

/* The whole 2026 edition runs at 16 MHz, matching Frequency="16 MHz" on the
 * MCU in the shared SimulIDE board.  build.bat passes -DF_CPU=16000000UL, so
 * this guarded definition only takes effect when a file is compiled on its
 * own - the header and the compiler flag can never disagree. */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#ifndef BAUD
#define BAUD 9600
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>


/* --- carried over from the original lesson --- */
#include <stdio.h>
#include "../../shared_libs/_uart.h"


/* ---------------------------------------------------------------------------
 * Board map: status LEDs
 *
 * The shared SimulIDE board wires 8 LEDs to PORT B, anode to the 5 V rail
 * through a resistor and cathode to the pin, so a pin must be driven LOW to
 * light its LED.  This matches shared_libs/_port.c, which documents PORT B as
 * the LED array.  Write patterns through LED_WRITE so the inversion lives in
 * one place: bit set means lit.
 * ------------------------------------------------------------------------ */
#define LED_DDR DDRB
#define LED_PORT PORTB
#define LED_WRITE(v) (LED_PORT = (uint8_t)~(uint8_t)(v))
#define LED_TOGGLE(m) (LED_PORT ^= (uint8_t)(m))

#endif /* CONFIG_H_ */
