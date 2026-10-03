/*
 * view.c - drawing the race (see view.h for the screen layout)
 *
 * The bus is the expensive part (lesson 12): a whole 1 KB frame is ~25 ms of
 * I2C.  So the track is drawn ONCE, when it changes, and after that only
 * three things move:
 *
 *   - the robot, drawn with XOR: drawing the same triangle twice in the same
 *     place removes it and restores whatever track was under it, so moving
 *     it touches only the one or two pages it is in;
 *   - the top text row (page 0) and the bottom row (page 7).
 *
 * oled_flush() then sends 3-4 pages a frame instead of 8.
 */
#include <stdio.h>
#include "view.h"
#include "track.h"
#include "oled.h"
#include "fmath.h"

#define AREA_TOP   9          /* the track area: rows 9..54                  */
#define AREA_BOT   55
#define MASK       11         /* the robot fits in an 11 x 11 pixel box      */

static float sc, ox, oy;      /* pixels per mm, and the screen offset        */

static int px(float x) { float v = ox + sc * x; return (int)(v + (v >= 0.0f ? 0.5f : -0.5f)); }
static int py(float y) { float v = oy - sc * y; return (int)(v + (v >= 0.0f ? 0.5f : -0.5f)); }

/* the robot currently on screen: its box's corner, and one row per bit */
static int      rob_x, rob_y, rob_on;
static uint16_t rob_mask[MASK];

void view_menu(int selected)
{
    oled_clear();
    oled_text(0, 0, "LINE FOLLOWER  L15", OLED_ON);
    for (int i = 0; i < TRACK_COUNT; i++) {
        oled_printf(0, 12 + 10 * i, "%c%d %-8s %s", i == selected ? '>' : ' ',
                    i + 1, track_name(i), track_level(i));
    }
    oled_text(0, 56, "joy:pick  A:go", OLED_ON);
    rob_on = 0;
}

void view_track(void)
{
    oled_clear();
    float w = (float)(track.xmax - track.xmin), h = (float)(track.ymax - track.ymin);
    float sx = 124.0f / w, sy = (float)(AREA_BOT - AREA_TOP - 2) / h;
    sc = sx < sy ? sx : sy;
    ox = 64.0f - sc * 0.5f * (float)(track.xmin + track.xmax);
    oy = 0.5f * (float)(AREA_TOP + AREA_BOT - 1) + sc * 0.5f * (float)(track.ymin + track.ymax);

    for (int i = 0; i < track.n - 1; i++) {
        if (track_is_gap(i)) { continue; }                  /* no tape, no line */
        oled_line(px(track.pt[i].x), py(track.pt[i].y),
                  px(track.pt[i + 1].x), py(track.pt[i + 1].y), OLED_ON);
    }
    /* the start / finish line: 60 mm either side, square to the track */
    float nx = -f_sin(track.th0) * 60.0f, ny = f_cos(track.th0) * 60.0f;
    oled_line(px(track.x0 + nx), py(track.y0 + ny), px(track.x0 - nx), py(track.y0 - ny), OLED_ON);
    rob_on = 0;
}

static void xor_mask(void)
{
    for (int r = 0; r < MASK; r++) {
        int y = rob_y + r;
        if (!rob_mask[r] || y < AREA_TOP || y >= AREA_BOT) { continue; }   /* keep off the text */
        for (int c = 0; c < MASK; c++) {
            if (rob_mask[r] & (1u << c)) { oled_pixel(rob_x + c, y, OLED_XOR); }
        }
    }
}

void view_robot(float x, float y, float th)
{
    /* A triangle, 7.5 px nose to tail, the same size on every track so it
     * stays visible: nose (4.5, 0), tail corners (-3, +-3) in robot axes. */
    int cx = px(x), cy = py(y);
    float c = f_cos(th), s = f_sin(th);
    const float fx[3] = { 4.5f, -3.0f, -3.0f }, fy[3] = { 0.0f, 3.0f, -3.0f };
    float vx[3], vy[3];
    for (int k = 0; k < 3; k++) {                      /* screen y points DOWN */
        vx[k] =  fx[k] * c - fy[k] * s;
        vy[k] = -(fx[k] * s + fy[k] * c);
    }
    uint16_t m[MASK];
    for (int r = 0; r < MASK; r++) {
        m[r] = 0;
        for (int col = 0; col < MASK; col++) {
            float qx = (float)(col - MASK / 2), qy = (float)(r - MASK / 2);
            /* inside if on the same side of all three edges */
            int pos = 0, neg = 0;
            for (int k = 0; k < 3; k++) {
                int j = (k + 1) % 3;
                float e = (vx[j] - vx[k]) * (qy - vy[k]) - (vy[j] - vy[k]) * (qx - vx[k]);
                if (e > 0.0f) { pos++; } else if (e < 0.0f) { neg++; }
            }
            if (pos == 0 || neg == 0) { m[r] |= (uint16_t)(1u << col); }
        }
    }
    int same = rob_on && rob_x == cx - MASK / 2 && rob_y == cy - MASK / 2;
    for (int r = 0; same && r < MASK; r++) { if (m[r] != rob_mask[r]) { same = 0; } }
    if (same) { return; }                               /* nothing moved: no bus */

    if (rob_on) { xor_mask(); }                         /* erase the old one     */
    rob_x = cx - MASK / 2;
    rob_y = cy - MASK / 2;
    for (int r = 0; r < MASK; r++) { rob_mask[r] = m[r]; }
    xor_mask();                                         /* draw the new one      */
    rob_on = 1;
}

void view_hud(const world_t *w, const sensors_t *s, int manual, int knob)
{
    /* top row: track, laps, this lap, best lap */
    uint32_t now  = w->state == W_RUN ? w->t_ms - w->lap_start : 0u;
    uint32_t best = w->best_lap;
    oled_fill_rect(0, 0, OLED_W, 8, OLED_OFF);
    oled_printf(0, 0, "T%d L%u", track.id + 1, (unsigned)w->laps);
    oled_printf(42, 0, "%3lu.%02lu", (unsigned long)(now / 1000u), (unsigned long)(now % 1000u / 10u));
    if (best) {
        oled_printf(86, 0, "B%3lu.%02lu", (unsigned long)(best / 1000u), (unsigned long)(best % 1000u / 10u));
    }

    /* bottom row: the sensor bar as it would look from above (leftmost
     * sensor on the left), then mode, speed setting and state */
    oled_fill_rect(0, 56, OLED_W, 8, OLED_OFF);
    for (int j = 0; j < LINE_SENSORS; j++) {
        int i = LINE_SENSORS - 1 - j;
        int h = ((int)s->line[i] * 8 + 500) / 1000;
        if (h > 0) { oled_fill_rect(j * 5, 64 - h, 4, h, OLED_ON); }
    }
    const char *st = w->state == W_READY ? "READY" : w->state == W_DNF ? "DNF"
                   : w->slipping ? "SLIP" : "RUN";
    oled_printf(38, 57, "%c%4d %s", manual ? 'M' : 'A', knob * 2, st);
}
