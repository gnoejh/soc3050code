/*
 * app.c - the demo application: STEER.  Replace this file with your project.
 *
 * A ball you steer with the joystick.  Touch the target dot to score; a new
 * one appears somewhere else.  The knob sets how hard the stick pushes,
 * A brakes, B restarts.  It is deliberately small - about 150 lines - so that
 * what remains visible is the SHAPE every project here should have:
 *
 *   state      static variables, all in one struct you can print and test
 *   step       pure integer arithmetic on that state, driven only by dt_ms
 *              and the controls - no clocks, no registers, no RTOS
 *   draw       reads the state, writes pixels, changes nothing
 *   telemetry  one line that lets the host see inside
 *
 * Positions and velocities are Q8 fixed point: 1/256 of a pixel, so the ball
 * can move a fraction of a pixel per step on a core with no FPU (lesson 09's
 * ball, lesson 10's argument).
 *
 * Pure C: host/test_app.c compiles this exact file with the PC's gcc.
 */
#include <stdio.h>
#include "oled.h"
#include "app.h"

#define HUD_H     9                    /* rows 0..8: the score line         */
#define R         3                    /* ball radius, px                   */
#define X_MIN     ((1 + R) << 8)       /* inside the 1-px border            */
#define X_MAX     ((OLED_W - 2 - R) << 8)
#define Y_MIN     ((HUD_H + 1 + R) << 8)
#define Y_MAX     ((OLED_H - 2 - R) << 8)
#define V_MAX     768                  /* 3 px per step = 150 px/s at 50 Hz */
#define HIT_R2    (6 * 6)              /* touching distance, squared, px^2  */

static app_state_t s;
static uint32_t    rng;                /* xorshift32 state - never 0        */

static uint32_t rand32(void)
{
    rng ^= rng << 13;
    rng ^= rng >> 17;
    rng ^= rng << 5;
    return rng;
}

static void new_target(void)
{
    /* Somewhere inside the arena, at least 20 px from the ball, so a point
     * always costs some steering. */
    for (int tries = 0; tries < 16; tries++) {
        s.tx = 8 + (int32_t)(rand32() % (uint32_t)(OLED_W - 16));
        s.ty = HUD_H + 6 + (int32_t)(rand32() % (uint32_t)(OLED_H - HUD_H - 12));
        int32_t dx = s.tx - (s.x >> 8), dy = s.ty - (s.y >> 8);
        if (dx * dx + dy * dy >= 20 * 20) { return; }
    }
}

const app_state_t *app_state(void) { return &s; }

void app_init(uint32_t seed)
{
    rng = seed ? seed : 0x1234567u;
    s.x  = (OLED_W / 2) << 8;
    s.y  = ((OLED_H + HUD_H) / 2) << 8;
    s.vx = s.vy = 0;
    s.score = s.wall_hits = s.time_ms = 0;
    new_target();
}

static int32_t clamp(int32_t v, int32_t lo, int32_t hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/* v/32, rounded AWAY from zero.  Plain v/32 truncates toward zero, so any
 * speed under 32 (1/8 px per step) would never decay and a released ball
 * would creep forever - a fixed-point trap worth one comment. */
static int32_t friction(int32_t v)
{
    return v > 0 ? (v + 31) / 32 : v < 0 ? (v - 31) / 32 : 0;
}

void app_step(uint32_t dt_ms, const pad_t *pad)
{
    if (pad->pressed & PAD_B) {                    /* restart, same RNG run */
        app_init(rand32());
        app_sound(440, 80);
        return;
    }
    s.time_ms += dt_ms;

    /* Stick -> acceleration.  The knob (0..1000) scales it from 1/8 to 5/8:
     * a gain the player tunes live, the way lesson 17 tunes a controller. */
    int32_t gain = 250 + pad->knob;                /* 250..1250             */
    s.vx += (int32_t)pad->x * gain / 2000;         /* Q8 per step           */
    s.vy -= (int32_t)pad->y * gain / 2000;         /* stick up = screen up  */

    if (pad->pressed & PAD_A) {                    /* brake                 */
        s.vx /= 4;  s.vy /= 4;
        app_sound(1200, 30);
    }
    s.vx -= friction(s.vx);                        /* rolling friction      */
    s.vy -= friction(s.vy);
    s.vx = clamp(s.vx, -V_MAX, V_MAX);
    s.vy = clamp(s.vy, -V_MAX, V_MAX);

    s.x += s.vx;
    s.y += s.vy;

    /* Walls: reflect, lose a quarter of the speed, click. */
    int hit = 0;
    if (s.x < X_MIN) { s.x = X_MIN; s.vx = -s.vx * 3 / 4; hit = 1; }
    if (s.x > X_MAX) { s.x = X_MAX; s.vx = -s.vx * 3 / 4; hit = 1; }
    if (s.y < Y_MIN) { s.y = Y_MIN; s.vy = -s.vy * 3 / 4; hit = 1; }
    if (s.y > Y_MAX) { s.y = Y_MAX; s.vy = -s.vy * 3 / 4; hit = 1; }
    if (hit) { s.wall_hits++; app_sound(220, 15); }

    /* Target. */
    int32_t dx = s.tx - (s.x >> 8), dy = s.ty - (s.y >> 8);
    if (dx * dx + dy * dy <= HIT_R2) {
        s.score++;
        app_sound(880, 60);
        new_target();
    }
}

void app_draw(void)
{
    oled_printf(0, 0, "SCORE %lu", (unsigned long)s.score);
    oled_printf(80, 0, "%3lu.%lus", (unsigned long)(s.time_ms / 1000u),
                (unsigned long)(s.time_ms / 100u % 10u));
    oled_rect(0, HUD_H, OLED_W, OLED_H - HUD_H, OLED_ON);
    oled_circle(s.tx, s.ty, 2, OLED_ON);
    oled_fill_circle((s.x + 128) >> 8, (s.y + 128) >> 8, R, OLED_ON);
}

int app_telemetry(char *body, size_t n)
{
    /* APP,<ms>,<x px>,<y px>,<vx Q8>,<vy Q8>,<score> */
    return snprintf(body, n, "APP,%lu,%ld,%ld,%ld,%ld,%lu",
                    (unsigned long)s.time_ms, (long)(s.x >> 8), (long)(s.y >> 8),
                    (long)s.vx, (long)s.vy, (unsigned long)s.score);
}
