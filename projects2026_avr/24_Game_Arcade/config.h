/*
 * config.h - Three games on the GLCD engine
 * SOC3050 ATmega128 Educational Framework, 2026 AVR edition
 *
 * Focus: game state machines, PROGMEM sprites, collision, and non-blocking sound
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
 *   PORTG  PG4 speaker, through the board's slide switch. If a tone plays
 *              and nothing is heard, check the switch before the code.
 *   PORTB      the eight LEDs, active low
 */

/* The eight LEDs on PORT B are wired rail -> resistor -> anode, cathode -> pin,
 * so a pin driven LOW lights its LED.  One place handles the inversion. */
#define LED_DDR DDRB
#define LED_WRITE(v) (PORTB = (uint8_t) ~(v))

#define TICK_HZ 50 /* 20 ms per frame */

/* Page 0 (y 0-7) is the score line; every game owns y 8-63 below it. */
#define HUD_H 8
#define FIELD_TOP HUD_H
#define FIELD_BOT 63

/* --- pong ---------------------------------------------------------------- */
#define PONG_PAD_W 24
#define PONG_PAD_H 2
#define PONG_BALL 3
#define PONG_PLAYER_Y 60
#define PONG_AI_Y 10
#define PONG_AI_SPEED 2 /* under the ball's speed, so the AI can be beaten */
#define PONG_WIN 9

/* --- snake --------------------------------------------------------------
 * A 4-pixel cell divides the 128-wide screen into 32 columns and the 56 rows
 * below the HUD into 14. The body is stored as two byte arrays rather than
 * packed coordinates, because 240 bytes is affordable and the code stays
 * readable - the framebuffer already spends 1024. */
#define CELL 4
#define GRID_W 32
#define GRID_H 14
#define SNAKE_MAX 120
#define SNAKE_TICKS 6 /* move every 6 frames: about 8 cells a second */

/* --- breakout ------------------------------------------------------------ */
#define BRK_COLS 8
#define BRK_ROWS 3
#define BRK_W 16
#define BRK_H 5
#define BRK_TOP 12
#define BRK_PAD_W 24
#define BRK_PAD_Y 58
#define BRK_LIVES 3

#endif /* CONFIG_H_ */
