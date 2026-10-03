/*
 * engine.h - the arcade's game engine: the contract every game keeps
 *
 * PURE C.  Nothing in this file, engine.c, arcade.c or the three games
 * includes stm32c031xx.h, so all of it compiles on a PC - host/test_games.c
 * runs the real games there, frame by frame, with scripted joysticks.
 *
 * The model, in four sentences.
 *   1. Time advances in fixed TICKS of 20 ms; step() moves the world one tick
 *      and never looks at a clock, so a game runs identically on the board,
 *      on a PC, fast or slow.
 *   2. draw() changes the framebuffer (oled.h) and touches no wire.
 *   3. Main.c decides how many ticks to run and when to draw and flush -
 *      "update at a fixed rate, render when time allows".
 *   4. Anything a game wants from the platform - a sound - goes through one
 *      hook, sfx(), which Main.c implements with the buzzer and the host test
 *      implements by counting.
 */
#ifndef ENGINE_H
#define ENGINE_H

#include <stdint.h>
#include "pad.h"          /* pad_t: joystick, knob, button edges - pure C */
#include "oled.h"

#define TICK_MS   20      /* one step() = 20 ms of game time: 50 updates/s  */
#define TICK_HZ   (1000 / TICK_MS)
#define HUD_H     9       /* rows 0..7 text, row 8 a line: the HUD           */
#define FIELD_Y   HUD_H   /* the first row a game may play in                */
#define FP        8       /* fixed point: positions in 1/256ths of a pixel   */

/* ---- what every game provides ------------------------------------------- */
typedef struct {
    const char *name;                 /* menu text and the $SCORE field       */
    void    (*start)(int level);      /* level 1..5, from the knob            */
    int     (*step)(const pad_t *in); /* one tick; returns 1 when it is over  */
    void    (*draw)(int full);        /* full = 1: repaint everything         */
    int32_t (*score)(void);
} game_t;

extern const game_t game_snake, game_breakout, game_flap;

/* ---- sound: the one hook to the platform -------------------------------- */
enum { SFX_MOVE, SFX_SELECT, SFX_START, SFX_EAT, SFX_BOUNCE, SFX_BRICK,
       SFX_FLAP, SFX_POINT, SFX_LOSE, SFX_LEVEL, SFX_OVER, SFX_HIGH, SFX_COUNT };
void sfx(int id);                     /* Main.c on the board; the test on a PC */

/* ---- a tiny deterministic random number generator ----------------------- */
void     rng_seed(uint32_t s);
uint32_t rng_next(void);              /* xorshift32: 3 shifts, 3 XORs          */
uint32_t rng_below(uint32_t n);       /* 0 .. n-1                              */

/* ---- collision: axis-aligned bounding boxes ----------------------------- */
/* Two rectangles overlap unless one is entirely left of, right of, above or
 * below the other.  Four comparisons; the whole of collision in this arcade. */
static inline int aabb(int ax, int ay, int aw, int ah,
                       int bx, int by, int bw, int bh)
{
    return ax < bx + bw && bx < ax + aw && ay < by + bh && by < ay + ah;
}

/* ---- shared drawing ------------------------------------------------------ */
void hud(const char *name, int32_t score, int lives, int full);  /* lives < 0: none */
int  text_center(int y, const char *s, int colour);
void box_message(const char *l1, const char *l2, const char *l3, const char *l4);

/* ---- read-only peeks, for the host test's autopilots -------------------- */
typedef struct {
    int x, y, w, h;       /* the thing the player steers, in pixels          */
    int tx, ty;           /* where it should go: food, ball, gap centre       */
    int vx, vy;           /* speed, 1/256 px per tick (ball, bird)            */
    int gap;              /* FLAP: the gap's height                           */
    int count;            /* snake length, bricks left, pipes passed          */
    int lives;
} peek_t;
void snake_peek(peek_t *p);
int  snake_cell_free(int cx, int cy);     /* inside the walls and not snake */
void breakout_peek(peek_t *p);
void flap_peek(peek_t *p);

#endif
