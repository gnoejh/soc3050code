/*
 * ui.h - the ground-station screen on the 128x64 OLED
 *
 *   +----------------+---------------+-+
 *   |  top-down map  | MISSION       |a|   mode
 *   |  64 x 64 px    | WP 3/5        |l|   progress, or the failsafe
 *   |  1.6 m / px    | ALT 8.0m      |t|
 *   |  fence circle  | BAT 81%       | |
 *   |  waypoints x   | [#######   ]  |b|
 *   |  drone o-      | WND 5.0       |a|   m/s, from the knob
 *   |  trail . . .   | T 42s         |r|   since arming
 *   +----------------+---------------+-+
 *
 * Pure C on top of oled.h's RAM framebuffer: ui_draw() never touches the bus
 * (the caller flushes), so host/sitl.c draws the same screen on the PC and
 * prints it as text.
 */
#ifndef UI_H
#define UI_H

#include <stdint.h>
#include "flight.h"

typedef struct {
    uint8_t  mode, reason, cur, n_wp, mission_active;
    float    px, py, pz, yaw, battery, wind, fence_r, fence_alt;
    uint32_t t_ms;
    float    wpx[FC_MAX_WP], wpy[FC_MAX_WP];
} ui_view_t;

void ui_from_fc(ui_view_t *v, const fc_t *f, float wind);   /* copy: hold the lock */
void ui_draw(const ui_view_t *v);                             /* RAM only            */

#endif
