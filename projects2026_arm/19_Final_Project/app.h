/*
 * app.h - THE PLUG-IN INTERFACE.  Your project lives behind these functions.
 *
 * The template (Main.c) owns the hardware, the tasks, the locks and the
 * watchdog.  Your application owns its own state and nothing else.  It is
 * called from exactly three places, always under the template's app_lock, so
 * none of these functions ever runs at the same time as another:
 *
 *   app_init()        once, before the scheduler starts
 *   app_step()        from the APP task, every APP_PERIOD_MS, with the latest
 *                     controls.  Game logic, physics, state machines: here.
 *   app_draw()        from the DISPLAY task, ~20 times a second.  Draw into
 *                     the OLED's RAM framebuffer with oled_*() and return;
 *                     the template sends it over I2C afterwards.  Do NOT
 *                     change game state here - drawing is a read-only view.
 *   app_telemetry()   from the TELEMETRY task, 10 times a second.  Write one
 *                     frame body ("APP,...") for host.py; return its length.
 *
 * RULES that keep the app portable to the host test harness (host/):
 *   - include no "stm32c031xx.h", touch no register, call no os_*() function;
 *   - no floats unless you have measured their cost (lesson 10);
 *   - every bit of state in static variables in app.c - no malloc;
 *   - time comes ONLY from dt_ms.  Never read a clock: then the same inputs
 *     give the same game, on the board and on the PC, and a test can replay it.
 *
 * Sound goes OUT through app_sound(), which the platform provides: Main.c
 * turns it into a buzzer tone; the host harness just counts the calls.
 */
#ifndef APP_H
#define APP_H

#include <stddef.h>
#include <stdint.h>
#include "pad.h"

#define APP_NAME       "steer"     /* printed in the banner and $APP frames  */
#define APP_PERIOD_MS  20u         /* app_step() rate: 50 Hz                 */

void app_init(uint32_t seed);
void app_step(uint32_t dt_ms, const pad_t *pad);
void app_draw(void);
int  app_telemetry(char *body, size_t n);

/* ---- provided BY THE PLATFORM, called by the app ------------------------ */
void app_sound(uint32_t hz, uint32_t ms);     /* a tone; returns at once     */

/* ---- read-only views for tests and the shell ---------------------------- */
typedef struct {
    int32_t  x, y;          /* ball position, 1/256 px (Q8 fixed point)     */
    int32_t  vx, vy;        /* velocity, 1/256 px per step                  */
    int32_t  tx, ty;        /* target, whole pixels                         */
    uint32_t score, wall_hits, time_ms;
} app_state_t;

const app_state_t *app_state(void);

#endif
