/*
 * view.c - the OLED picture
 *
 *   +--------------------------------------------------------------+
 *   | LQR             tilt +1.2                                    |  text
 *   | x+0.12  v+0.05  u+1.5                                        |  text
 *   |                                     |  +---------------+     |
 *   |            []    <- payload         |  |  tilt history |     |
 *   |            /                        |  |~~~~~~~~~~~~~~~|     |
 *   |           /     <- body at the tilt |  |               |     |
 *   |          (o)    <- wheel, spoke turns  +---------------+     |
 *   |  ---|------^------|------ ground, a tick every 10 cm         |
 *   +--------------------------------------------------------------+
 *
 * The camera follows the robot in jumps, so the ground ticks show it moving
 * and a drifting controller is obvious at a glance.  Everything is drawn
 * into the RAM framebuffer; Main.c's display task calls oled_flush().
 */
#include <stdio.h>
#include "view.h"
#include "oled.h"
#include "fmath.h"
#include "params.h"

#define PX_PER_M   80.0f          /* 10 cm = 8 px                        */
#define SCENE_W    86             /* the scene is x 0..85                 */
#define GROUND_Y   61
#define WHEEL_R    5
#define BODY_LEN   26
#define CHART_X    90
#define CHART_Y    18
#define CHART_W    (VIEW_HIST + 2)
#define CHART_H    44

void view_history(view_t *v, float th)
{
    float d = th * 57.29578f;
    d = f_clamp(d, -30.0f, 30.0f);
    v->hist[v->head] = (int8_t)(d >= 0.0f ? d + 0.5f : d - 0.5f);
    v->head = (uint8_t)((v->head + 1u) % VIEW_HIST);
}

/* "+1.23": a float as text with no float printf (newlib-nano has none) */
static void fixed(char *out, int n, float v, int decimals)
{
    int scale = decimals == 2 ? 100 : 10;
    v = f_clamp(v, -999.0f, 999.0f);
    int32_t i = (int32_t)(v * (float)scale + (v >= 0.0f ? 0.5f : -0.5f));
    char sign = i < 0 ? '-' : '+';
    if (i < 0) { i = -i; }
    if (decimals == 2) { snprintf(out, (size_t)n, "%c%ld.%02ld", sign, (long)(i / 100), (long)(i % 100)); }
    else               { snprintf(out, (size_t)n, "%c%ld.%ld",   sign, (long)(i / 10),  (long)(i % 10)); }
}

static int to_px(float x_m, float cam) { return (int)((x_m - cam) * PX_PER_M) + SCENE_W / 2; }

void view_draw(view_t *v)
{
    char a[16], b[16], c[16];
    oled_clear();

    /* ---- two text lines ---- */
    oled_text(0, 0, v->mode, OLED_ON);
    fixed(a, sizeof a, v->th * 57.29578f, 1);
    oled_printf(68, 0, "tilt%s", a);
    fixed(a, sizeof a, v->x, 2);
    fixed(b, sizeof b, v->xd, 2);
    fixed(c, sizeof c, v->u, 1);
    oled_printf(0, 9, "x%s v%s u%s", a, b, c);

    /* ---- the camera: recentre when the robot nears an edge ---- */
    int rx = to_px(v->x, v->cam);
    if (rx < 14 || rx > SCENE_W - 14) { v->cam = v->x; rx = to_px(v->x, v->cam); }

    /* ---- ground, with a tick every 10 cm so motion shows ---- */
    oled_hline(0, GROUND_Y, SCENE_W, OLED_ON);
    float first = v->cam - (float)(SCENE_W / 2) / PX_PER_M;
    int32_t k = (int32_t)(first * 10.0f) - 1;
    for (; ; k++) {
        int px = to_px((float)k * 0.1f, v->cam);
        if (px >= SCENE_W) { break; }
        if (px >= 0) { oled_vline(px, GROUND_Y + 1, (k % 10 == 0) ? 3 : 2, OLED_ON); }
    }
    /* where the controller is trying to be: a caret under the ground */
    int tx = to_px(v->x_ref, v->cam);
    if (tx >= 2 && tx < SCENE_W - 2) {
        oled_line(tx - 2, 63, tx, GROUND_Y + 1, OLED_ON);
        oled_line(tx + 2, 63, tx, GROUND_Y + 1, OLED_ON);
    }

    /* ---- the robot ---- */
    int cy = GROUND_Y - WHEEL_R - 1;
    oled_circle(rx, cy, WHEEL_R, OLED_ON);
    float spin = v->x * (1.0f / BOT_R);                      /* the wheel's angle */
    oled_line(rx, cy, rx + (int)(4.0f * f_cos(spin)), cy - (int)(4.0f * f_sin(spin)), OLED_ON);
    float s = f_sin(v->th), co = f_cos(v->th);
    int tx2 = rx + (int)((float)BODY_LEN * s), ty2 = cy - (int)((float)BODY_LEN * co);
    oled_line(rx, cy, tx2, ty2, OLED_ON);
    oled_line(rx + 1, cy, tx2 + 1, ty2, OLED_ON);            /* two pixels thick */
    int ps = 2 + (int)(v->payload * 8.0f);                   /* payload box: 2..6 px */
    if (v->payload > 0.01f) { oled_fill_rect(tx2 - ps / 2, ty2 - ps, ps + 1, ps, OLED_ON); }

    /* ---- the tilt strip chart: +-30 degrees, newest on the right ---- */
    oled_rect(CHART_X, CHART_Y, CHART_W, CHART_H, OLED_ON);
    int mid = CHART_Y + CHART_H / 2;
    for (int x = CHART_X + 2; x < CHART_X + CHART_W - 1; x += 3) { oled_pixel(x, mid, OLED_ON); }
    for (int i = 0; i < VIEW_HIST; i++) {
        int d = v->hist[(v->head + i) % VIEW_HIST];
        oled_pixel(CHART_X + 1 + i, mid - d * (CHART_H / 2 - 2) / 30, OLED_ON);
    }

    if (v->fallen) {
        oled_fill_rect(4, 26, 78, 20, OLED_ON);
        oled_text(25, 28, "FALLEN", OLED_OFF);
        oled_text(10, 37, "A: stand up", OLED_OFF);
    }
}
