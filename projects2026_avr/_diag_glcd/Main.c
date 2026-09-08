/*
 * Main.c - GLCD bisect diagnostic  (NOT a lesson; delete when done)
 *
 * The folder name starts with an underscore so verify-all.ps1, gen-readme.py
 * and build-slides.py all skip it - they match ^\d\d_ only.
 *
 * Purpose: separate three possibilities that all look like a blank screen.
 *
 *   LEDs alternate + screen alternates black/blank -> bus works, bug is higher up
 *   LEDs alternate + screen never changes          -> GLCD bus/model problem
 *   LEDs do not alternate                          -> firmware is not running
 *
 * No shared libraries, no framebuffer, no timers, no interrupts. Everything
 * this program depends on is in this file.
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>

/* Traced from Simulator.simu:
 *   PORTA0-7 -> dataPin0-7      PE4 -> PinDC (RS)     PE5 -> PinEn
 *   PE6 -> PinCs2               PE7 -> PinCs1         PG1 -> PinRW
 *   PinRst -> 100R -> 5V rail   (so reset is released, nothing to drive)
 * The board sets CS_Active_Low="false", so chip select is active HIGH.
 */
#define RS_BIT 4
#define E_BIT 5
#define CS2_BIT 6
#define CS1_BIT 7
#define RW_BIT 1

#define CS_LEFT 1
#define CS_RIGHT 2
#define CS_BOTH 3

static void wr(uint8_t value, uint8_t rs, uint8_t cs)
{
    PORTA = value;

    if (rs)
        PORTE |= (1 << RS_BIT);
    else
        PORTE &= (uint8_t) ~(1 << RS_BIT);

    if (cs & CS_LEFT)
        PORTE |= (1 << CS1_BIT);
    else
        PORTE &= (uint8_t) ~(1 << CS1_BIT);

    if (cs & CS_RIGHT)
        PORTE |= (1 << CS2_BIT);
    else
        PORTE &= (uint8_t) ~(1 << CS2_BIT);

    /* Deliberately generous timing - this is a diagnostic, not a fast path.
     * If the fast engine fails and this works, the pulse width is the culprit. */
    _delay_us(5);
    PORTE |= (1 << E_BIT);
    _delay_us(5);
    PORTE &= (uint8_t) ~(1 << E_BIT);
    _delay_us(5);
    PORTE &= (uint8_t) ~((1 << CS1_BIT) | (1 << CS2_BIT));
    _delay_us(10);
}

static void cmd(uint8_t c, uint8_t cs) { wr(c, 0, cs); }

static void fill(uint8_t pattern)
{
    for (uint8_t page = 0; page < 8; page++)
    {
        cmd(0xB8 | page, CS_BOTH); /* set page  */

        cmd(0x40, CS_LEFT); /* column 0 */
        for (uint8_t x = 0; x < 64; x++)
            wr(pattern, 1, CS_LEFT);

        cmd(0x40, CS_RIGHT);
        for (uint8_t x = 0; x < 64; x++)
            wr(pattern, 1, CS_RIGHT);
    }
}

int main(void)
{
    DDRA = 0xFF;    /* data bus out           */
    DDRE |= 0xF0;   /* RS, E, CS2, CS1 out    */
    DDRG |= (1 << RW_BIT);
    PORTG &= (uint8_t) ~(1 << RW_BIT); /* R/W low = write */
    DDRB = 0xFF;                       /* LEDs, active low */

    PORTE &= (uint8_t) ~((1 << E_BIT) | (1 << CS1_BIT) | (1 << CS2_BIT));
    PORTB = 0xFF; /* all LEDs off */

    _delay_ms(200);

    cmd(0x3F, CS_BOTH); /* display ON     */
    cmd(0xC0, CS_BOTH); /* start line 0   */
    cmd(0xB8, CS_BOTH); /* page 0         */
    cmd(0x40, CS_BOTH); /* column 0       */

    for (;;)
    {
        PORTB = (uint8_t)~0x0F; /* lower four LEDs on */
        fill(0xFF);             /* whole screen black */
        _delay_ms(1500);

        PORTB = (uint8_t)~0xF0; /* upper four LEDs on */
        fill(0x00);             /* whole screen clear */
        _delay_ms(1500);
    }
}
