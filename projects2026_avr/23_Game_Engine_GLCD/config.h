/*
 * config.h - A game engine for the GLCD
 * SOC3050 ATmega128 Educational Framework, 2026 AVR edition
 *
 * Focus: a RAM framebuffer, dirty-page flushing, and a fixed-timestep loop
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

#include "_game.h"

/* --- board map ----------------------------------------------------------
 * The engine in shared_libs/_game.c drives these directly; they are listed
 * here because config.h is where this course keeps a lesson's board facts.
 *
 *   PORTA      GLCD 8-bit data bus
 *   PORTE  PE4 GLCD RS, PE5 E, PE6 CS2 (right half), PE7 CS1 (left half)
 *   PORTG  PG1 GLCD R/W - held LOW for write. Not tied to ground on this
 *              board, whatever _glcd.h's header comment says.
 *   PORTD  PD0 PD1 PD4 PD5 PD6 PD7 - six buttons, pulled up, active low.
 *              PD2 and PD3 are RXD1/TXD1 and are deliberately left alone.
 *   PORTB      the eight LEDs, active low
 */

/* The eight LEDs on PORT B are wired rail -> resistor -> anode, cathode -> pin,
 * so a pin driven LOW lights its LED.  One place handles the inversion. */
#define LED_DDR DDRB
#define LED_WRITE(v) (PORTB = (uint8_t) ~(v))

/* --- playfield geometry -------------------------------------------------
 * The screen is 128x64.  Page 0 (y 0-7) holds the status line and page 7
 * (y 56-63) the profile line, so the field owns pages 1-6. */
#define FIELD_X 0
#define FIELD_Y 9
#define FIELD_W 128
#define FIELD_H 45 /* y 9 .. 53 inclusive */

#define BALL_SIZE 3
#define PADDLE_W 20
#define PADDLE_H 3
#define PADDLE_Y 50
#define PADDLE_STEP 3

#define TICK_HZ 50 /* 20 ms per frame */

#endif /* CONFIG_H_ */
