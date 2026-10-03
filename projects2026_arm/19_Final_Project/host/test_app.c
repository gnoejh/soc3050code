/*
 * host/test_app.c - the host test harness.  COPY THIS FOR YOUR PROJECT.
 *
 * It compiles the project's pure-C files - app.c, health.c, and _lib/oled.c's
 * drawing code - with the PC's gcc, drives them with scripted controls and a
 * fake clock, and checks numbers.  No board, no simulator, no RTOS: a test
 * run takes milliseconds, so it can play an hour of game time, every time.
 *
 * The pattern, for your own tests:
 *   1. a fake platform:  stubs for anything the app calls OUT to
 *                        (app_sound, i2c_write_read) that record calls;
 *   2. scripted inputs:  a pad_t you fill in per step - or a "bot" that
 *                        computes one from the state, as a player would;
 *   3. CHECK(cond, ...): prints PASS/FAIL with the measured numbers;
 *   4. exit status:      non-zero if anything failed, so a script can tell.
 *
 * Build and run:  run.bat  (Windows)   or   ./run.sh  (Git Bash, Linux, macOS)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "app.h"
#include "health.h"
#include "oled.h"
#include "i2c.h"

/* ============================================================================
 *  1. The fake platform
 * ============================================================================ */
static uint32_t sounds, sound_hz_last;
void app_sound(uint32_t hz, uint32_t ms) { (void)ms; sounds++; sound_hz_last = hz; }

/* oled.c's only hardware call.  Count the bytes so a test can see what a
 * flush would have cost on the real bus. */
static uint32_t i2c_bytes;
int i2c_write_read(uint8_t addr, const uint8_t *w, size_t wn, uint8_t *r, size_t rn)
{
    (void)addr; (void)w; (void)r; (void)rn;
    i2c_bytes += (uint32_t)wn;
    return I2C_OK;
}

/* ============================================================================
 *  3. CHECK
 * ============================================================================ */
static int failures, checks;
/* `cond` is evaluated ONCE - it may call the code under test.  (The first
 * version evaluated it twice, and a check that called health_check() failed
 * on the second call: a test harness has bugs too.) */
#define CHECK(cond, ...) do {                                         \
        int ok_ = (cond);                                             \
        checks++;                                                     \
        printf("  %s  ", ok_ ? "PASS" : "FAIL");                      \
        printf(__VA_ARGS__);                                          \
        printf("\n");                                                 \
        if (!ok_) { failures++; }                                     \
    } while (0)

/* ============================================================================
 *  2. Scripted inputs
 * ============================================================================ */
static pad_t pad_idle(void)
{
    pad_t p;
    memset(&p, 0, sizeof p);
    p.knob = 500;                       /* the diagram's default: mid-travel */
    p.adc_ok = 1;
    return p;
}

static int16_t clamp100(int32_t v) { return (int16_t)(v > 100 ? 100 : v < -100 ? -100 : v); }

/* A simple autopilot: push towards the target, minus a velocity term so it
 * does not overshoot.  A PD controller (lesson 17) driving a game. */
static pad_t bot(const app_state_t *s)
{
    pad_t p = pad_idle();
    int32_t ex = s->tx * 256 - s->x, ey = s->ty * 256 - s->y;      /* Q8 error */
    p.x = clamp100(ex / 40 - s->vx / 3);
    p.y = clamp100(-(ey / 40 - s->vy / 3));                         /* up = +   */
    return p;
}

static uint32_t rng = 12345u;
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

static pad_t pad_random(void)
{
    pad_t p = pad_idle();
    p.x = (int16_t)((int32_t)(rnd() % 201u) - 100);
    p.y = (int16_t)((int32_t)(rnd() % 201u) - 100);
    p.knob = (int16_t)(rnd() % 1001u);
    if (rnd() % 50u == 0u) { p.pressed |= PAD_A; }
    if (rnd() % 2000u == 0u) { p.pressed |= PAD_B; }
    return p;
}

/* ============================================================================
 *  The tests
 * ============================================================================ */
static void test_steer_right(void)
{
    printf("\n[1] full right stick: how long to reach the right wall?\n");
    app_init(1);
    pad_t p = pad_idle();
    p.x = 100;
    uint32_t steps = 0, hits0 = app_state()->wall_hits;
    int32_t x0 = app_state()->x;
    while (app_state()->wall_hits == hits0 && steps < 1000u) { app_step(APP_PERIOD_MS, &p); steps++; }
    CHECK(app_state()->x > x0, "ball moved right: x %ld -> %ld px",
          (long)(x0 >> 8), (long)(app_state()->x >> 8));
    CHECK(steps < 100u, "reached the wall in %lu steps = %lu ms (limit 2000 ms)",
          (unsigned long)steps, (unsigned long)(steps * APP_PERIOD_MS));
}

static void test_coast_stops(void)
{
    printf("\n[2] release the stick: does the ball stop (fixed-point friction)?\n");
    app_init(1);
    pad_t p = pad_idle();
    p.x = 60;  p.y = 40;
    for (int i = 0; i < 20; i++) { app_step(APP_PERIOD_MS, &p); }
    p = pad_idle();
    uint32_t steps = 0;
    while ((app_state()->vx != 0 || app_state()->vy != 0) && steps < 5000u) {
        app_step(APP_PERIOD_MS, &p);
        steps++;
    }
    CHECK(app_state()->vx == 0 && app_state()->vy == 0,
          "stopped after %lu steps = %lu ms (limit 5000 steps)",
          (unsigned long)steps, (unsigned long)(steps * APP_PERIOD_MS));
}

static void test_fuzz(void)
{
    printf("\n[3] fuzz: one hour of random controls - does the ball ever escape?\n");
    app_init(7);
    const uint32_t n = 3600u * 1000u / APP_PERIOD_MS;    /* 180 000 steps */
    uint32_t escapes = 0;
    int32_t vmax = 0;
    for (uint32_t i = 0; i < n; i++) {
        pad_t p = pad_random();
        app_step(APP_PERIOD_MS, &p);
        const app_state_t *s = app_state();
        int px = (int)(s->x >> 8), py = (int)(s->y >> 8);
        if (px < 1 || px > OLED_W - 2 || py < 10 || py > OLED_H - 2) { escapes++; }
        int32_t v = s->vx < 0 ? -s->vx : s->vx;
        if (v > vmax) { vmax = v; }
    }
    CHECK(escapes == 0u, "%lu steps (%lu s of play), ball outside the arena on %lu",
          (unsigned long)n, (unsigned long)(n * APP_PERIOD_MS / 1000u), (unsigned long)escapes);
    CHECK(vmax <= 768, "fastest |vx| %ld Q8 = %ld.%02ld px/step (cap 3.00)",
          (long)vmax, (long)(vmax / 256), (long)(vmax % 256 * 100 / 256));
}

static void test_determinism(void)
{
    printf("\n[4] determinism: same seed + same inputs = same game?\n");
    char a[80], b[80];
    for (int run = 0; run < 2; run++) {
        app_init(99);
        rng = 2024u;
        for (int i = 0; i < 20000; i++) { pad_t p = pad_random(); app_step(APP_PERIOD_MS, &p); }
        app_telemetry(run ? b : a, sizeof a);
    }
    CHECK(strcmp(a, b) == 0, "run 1 \"%s\"  run 2 \"%s\"", a, b);
}

static void test_bot(void)
{
    printf("\n[5] the autopilot: points in 60 s of game time (beat it by hand!)\n");
    app_init(42);
    uint32_t s0 = sounds;
    for (uint32_t t = 0; t < 60000u; t += APP_PERIOD_MS) {
        pad_t p = bot(app_state());
        app_step(APP_PERIOD_MS, &p);
    }
    const app_state_t *s = app_state();
    CHECK(s->score >= 10u, "bot scored %lu in 60 s, %lu wall hits, %lu sounds requested",
          (unsigned long)s->score, (unsigned long)s->wall_hits, (unsigned long)(sounds - s0));
}

static void test_draw(void)
{
    printf("\n[6] drawing: is the ball where the state says, and what does a frame cost?\n");
    app_init(3);
    pad_t p = pad_idle();
    for (int i = 0; i < 30; i++) { app_step(APP_PERIOD_MS, &p); }
    oled_clear();
    app_draw();
    const app_state_t *s = app_state();
    int bx = (int)((s->x + 128) >> 8), by = (int)((s->y + 128) >> 8);
    int lit = 0;
    for (int y = 0; y < OLED_H; y++) { for (int x = 0; x < OLED_W; x++) { lit += oled_get(x, y); } }
    CHECK(oled_get(bx, by) == 1, "pixel at the ball centre (%d,%d) is lit; %d pixels lit", bx, by, lit);

    i2c_bytes = 0;
    oled_flush();                                     /* first: everything new */
    uint32_t first = i2c_bytes;
    i2c_bytes = 0;
    oled_clear(); app_draw(); oled_flush();           /* nothing moved        */
    uint32_t still = i2c_bytes;
    /* 400 kHz I2C, 9 clocks per byte: 22.5 us per byte on the wire. */
    CHECK(first <= 8u * 129u + 64u, "first frame %lu bytes on the bus = %lu us at 400 kHz",
          (unsigned long)first, (unsigned long)(first * 45u / 2u));
    printf("        a still frame re-sends %lu bytes = %lu us: oled_clear() dirties every page that had\n"
           "        something on it - here the border and HUD touch all 8.  Lab Part 6 is about this.\n",
           (unsigned long)still, (unsigned long)(still * 45u / 2u));
}

static void test_telemetry(void)
{
    printf("\n[7] telemetry: does the longest frame fit the 80-byte buffer?\n");
    char body[80];
    app_init(5);
    int longest = 0;
    rng = 99u;
    for (int i = 0; i < 50000; i++) {
        pad_t p = pad_random();
        app_step(APP_PERIOD_MS, &p);
        int n = app_telemetry(body, sizeof body);
        if (n > longest) { longest = n; }
    }
    CHECK(longest < (int)sizeof body, "longest $APP body %d bytes: \"%s\"", longest, body);
}

static void test_health(void)
{
    printf("\n[8] health: does the watchdog logic feed, starve and force at the right times?\n");
    health_t h;
    health_init(&h, 4, 1500, 0);
    uint32_t now = 0;
    int v = -1;
    /* 1 s, everyone beating every 10..100 ms; checked every 250 ms */
    for (now = 10; now <= 1000; now += 10) {
        health_beat(&h, 0);
        if (now % 20 == 0)  { health_beat(&h, 1); }
        if (now % 50 == 0)  { health_beat(&h, 2); }
        if (now % 100 == 0) { health_beat(&h, 3); }
        if (now % 250 == 0) { v = health_check(&h, now); }
    }
    CHECK(v == HEALTH_FEED && h.feeds == 4u, "all beating: verdict %d, %lu feeds in 1 s",
          v, (unsigned long)h.feeds);

    /* the display task (index 2) hangs at t = 1000 */
    uint32_t first_starve = 0, force_at = 0;
    for (; now <= 4000 && !force_at; now += 10) {
        health_beat(&h, 0);
        if (now % 20 == 0)  { health_beat(&h, 1); }
        if (now % 100 == 0) { health_beat(&h, 3); }
        if (now % 250 == 0) {
            v = health_check(&h, now);
            if (v == HEALTH_STARVE && !first_starve) { first_starve = now; }
            if (v == HEALTH_FORCE) { force_at = now; }
        }
    }
    CHECK(h.missing == (1u << 2), "missing bitmap 0x%02lX (expected 0x04: display)",
          (unsigned long)h.missing);
    CHECK(first_starve == 1250u, "first starved check at %lu ms (hang at 1000 ms)",
          (unsigned long)first_starve);
    CHECK(force_at == 2500u, "software reset forced at %lu ms = %lu ms after the last feed (limit 1500)",
          (unsigned long)force_at, (unsigned long)(force_at - 1000u));

    /* one write lost to a race would look like this: a task that beat ONCE
     * in the window is still alive.  Counters cannot lose a beat. */
    health_init(&h, 2, 1500, 0);
    health_beat(&h, 0); health_beat(&h, 1);
    CHECK(health_check(&h, 250) == HEALTH_FEED, "one beat each in a window is enough to feed");
}

int main(void)
{
    printf("host tests for app \"%s\" and the health monitor\n", APP_NAME);
    test_steer_right();
    test_coast_stops();
    test_fuzz();
    test_determinism();
    test_bot();
    test_draw();
    test_telemetry();
    test_health();
    printf("\n%d checks, %d failed\n", checks, failures);
    return failures ? 1 : 0;
}
