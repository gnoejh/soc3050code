/*
 * engine.c - random numbers and the drawing every game shares.   PURE C.
 */
#include <stdio.h>
#include <string.h>
#include "engine.h"

/* ============================================================================
 *  xorshift32 (Marsaglia, 2003): three shifts and three XORs per number.
 *
 *  Deterministic: the same seed gives the same sequence, on the board and on a
 *  PC - which is what lets the host test replay a game exactly.  Not
 *  cryptographic, and it does not need to be: it decides where an apple goes.
 *  The state must never be 0 (0 maps to 0 forever), hence the guard.
 *
 *  The AVR edition's _game.c used the 16-bit version (shifts 7, 9, 8): a
 *  period of 65 535.  This one repeats after 2^32 - 1 numbers, and on a 32-bit
 *  core costs the same three instructions per step.
 * ========================================================================== */
static uint32_t rng_state = 2463534242u;

void rng_seed(uint32_t s) { rng_state = s ? s : 2463534242u; }

uint32_t rng_next(void)
{
    uint32_t x = rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return rng_state = x;
}

/* Scale into 0..n-1 (n up to 65535) with one multiply and a shift instead of
 * %: the M0+ has a MULS instruction but no divide instruction, so % would be a
 * call to libgcc's software division. */
uint32_t rng_below(uint32_t n)
{
    return ((rng_next() >> 16) * n) >> 16;
}

/* ============================================================================
 *  The HUD: page 0, plus a rule on row 8.
 *
 *  Redrawn only when something on it changed.  Redrawing identical pixels
 *  would be free on the bus anyway (oled_pixel() marks a page dirty only if a
 *  byte really changed) - but erasing the digits first and writing them back
 *  would not be, so the HUD remembers what it last showed.
 * ========================================================================== */
void hud(const char *name, int32_t score, int lives, int full)
{
    static const char *shown_name;
    static int32_t shown_score = -1;
    static int shown_lives = -2;
    if (!full && name == shown_name && score == shown_score && lives == shown_lives) {
        return;
    }
    shown_name = name;  shown_score = score;  shown_lives = lives;

    oled_fill_rect(0, 0, OLED_W, 8, OLED_OFF);
    oled_text(0, 0, name, OLED_ON);
    for (int i = 0; i < lives && i < 5; i++) {          /* lives: little blocks */
        oled_fill_rect(56 + i * 6, 2, 4, 4, OLED_ON);
    }
    char buf[12];
    int n = snprintf(buf, sizeof buf, "%ld", (long)score);
    oled_text(OLED_W - 6 * n, 0, buf, OLED_ON);         /* right-aligned        */
    oled_hline(0, 8, OLED_W, OLED_ON);
}

int text_center(int y, const char *s, int colour)
{
    int w = 6 * (int)strlen(s) - 1;
    return oled_text((OLED_W - w) / 2, y, s, colour);
}

/* A framed box over whatever the game drew: four text rows at y = 18, 28,
 * 38 and 48.  A NULL line is left blank (the arcade blinks NEW HIGH! there). */
void box_message(const char *l1, const char *l2, const char *l3, const char *l4)
{
    oled_fill_rect(8, 13, 112, 47, OLED_OFF);
    oled_rect(8, 13, 112, 47, OLED_ON);
    oled_rect(10, 15, 108, 43, OLED_ON);
    const char *line[4] = { l1, l2, l3, l4 };
    for (int i = 0; i < 4; i++) {
        if (line[i]) { text_center(18 + 10 * i, line[i], OLED_ON); }
    }
}
