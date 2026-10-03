/*
 * ui.c - draw the map and the instrument panel.  See ui.h for the layout.
 *
 * Copy first, draw after: ui_from_fc() takes a snapshot while the caller
 * holds the flight computer's lock (microseconds), and ui_draw() works from
 * the snapshot with no lock at all (milliseconds).  Lesson 09's rule: a
 * mutex for the resource, a copy for the reader.
 */
#include "ui.h"
#include "oled.h"
#include "fmath.h"
#include "fm.h"

#define MAP_C       32              /* map centre = home, in pixels          */
#define PX_PER_M    0.625f          /* 1.6 m per pixel: +-51 m fits 64 px    */
#define TRAIL_N     96              
#define TRAIL_MS    500u            /* one breadcrumb every half second      */
#define PANEL_X     66
#define ALT_BAR_X   123

static int8_t   trail_x[TRAIL_N], trail_y[TRAIL_N];
static uint8_t  trail_len, trail_next, last_mode;
static uint32_t trail_t, t_armed;

__attribute__((noinline)) static int mx(float x) { return MAP_C + (int)(x * PX_PER_M + (x >= 0.0f ? 0.5f : -0.5f)); }
__attribute__((noinline)) static int my(float y) { return MAP_C - (int)(y * PX_PER_M + (y >= 0.0f ? 0.5f : -0.5f)); }  /* north up */

void ui_from_fc(ui_view_t *v, const fc_t *f, float wind)
{
    v->mode = f->mode;  v->reason = f->reason;  v->cur = f->cur;
    v->n_wp = f->n_wp;  v->mission_active = f->mission_active;
    v->px = f->px;  v->py = f->py;  v->pz = f->pz;  v->yaw = f->yaw;
    v->battery = f->battery;  v->wind = wind;  v->t_ms = f->t_ms;
    v->fence_r = f->p.fence_r;  v->fence_alt = f->p.fence_alt;
    for (unsigned i = 0; i < FC_MAX_WP; i++) { v->wpx[i] = f->wp[i].x;  v->wpy[i] = f->wp[i].y; }
}

/* A dotted line: every third pixel, so a planned leg looks planned. */
static void dotted(int x0, int y0, int x1, int y1)
{
    int dx = x1 - x0, dy = y1 - y0;
    int n = (dx < 0 ? -dx : dx) > (dy < 0 ? -dy : dy) ? (dx < 0 ? -dx : dx) : (dy < 0 ? -dy : dy);
    for (int i = 0; i <= n; i += 3) {
        oled_pixel(x0 + (n ? dx * i / n : 0), y0 + (n ? dy * i / n : 0), OLED_ON);
    }
}

static void draw_map(const ui_view_t *v)
{
    /* the fence: a dotted circle */
    float r = v->fence_r * PX_PER_M;
    for (int k = 0; k < 48; k++) {
        float a = (float)k * (F_2PI / 48.0f);
        oled_pixel(MAP_C + (int)(r * m_cos(a)), MAP_C + (int)(r * m_sin(a)), OLED_ON);
    }
    oled_rect(MAP_C - 1, MAP_C - 1, 3, 3, OLED_ON);              /* home: a box   */

    /* the mission: crosses, joined by dotted legs; the target boxed */
    int px = MAP_C, py = MAP_C;
    for (unsigned i = 0; i < v->n_wp; i++) {
        int x = mx(v->wpx[i]), y = my(v->wpy[i]);
        dotted(px, py, x, y);
        oled_line(x - 2, y - 2, x + 2, y + 2, OLED_ON);
        oled_line(x - 2, y + 2, x + 2, y - 2, OLED_ON);
        if (v->mission_active && i == v->cur) { oled_rect(x - 4, y - 4, 9, 9, OLED_ON); }
        px = x;  py = y;
    }

    /* breadcrumbs */
    for (unsigned i = 0; i < trail_len; i++) { oled_pixel(trail_x[i], trail_y[i], OLED_ON); }

    /* the drone: a dot with a heading tick, 5 px along the nose */
    int dx = mx(v->px), dy = my(v->py);
    oled_fill_circle(dx, dy, 2, OLED_ON);
    oled_line(dx, dy, dx + (int)(5.0f * m_cos(v->yaw)), dy - (int)(5.0f * m_sin(v->yaw)), OLED_ON);
}

static int dm(float m) { return (int)(m * 10.0f + (m >= 0.0f ? 0.5f : -0.5f)); }

static void draw_panel(const ui_view_t *v)
{
    oled_vline(PANEL_X - 2, 0, 64, OLED_ON);
    oled_text(PANEL_X, 0, fc_mode_name(v->mode), OLED_ON);

    if (v->reason == R_FENCE || v->reason == R_BATTERY || v->reason == R_BATTERY_CRIT) {
        oled_printf(PANEL_X, 9, "!%s", fc_reason_name(v->reason));     /* why: a failsafe */
    } else if (v->mission_active) {
        oled_printf(PANEL_X, 9, "WP %u/%u", (unsigned)v->cur + 1u, (unsigned)v->n_wp);
    } else if (v->mode == M_DISARMED) {
        oled_text(PANEL_X, 9, "A: fly", OLED_ON);
    }

    int a = dm(v->pz);
    if (a < 0) { a = 0; }
    oled_printf(PANEL_X, 18, "ALT %d.%dm", a / 10, a % 10);
    int b = (int)(v->battery + 0.5f);
    oled_printf(PANEL_X, 27, "BAT %d%%", b);
    oled_rect(PANEL_X, 36, 50, 5, OLED_ON);
    oled_fill_rect(PANEL_X, 36, b / 2, 5, OLED_ON);
    int w = dm(v->wind);
    oled_printf(PANEL_X, 44, "WND %d.%d", w / 10, w % 10);
    if (v->mode != M_DISARMED) {
        oled_printf(PANEL_X, 54, "T %lus", (unsigned long)((v->t_ms - t_armed) / 1000u));
    }

    /* altitude bar, full scale = the fence ceiling */
    oled_rect(ALT_BAR_X, 8, 5, 56, OLED_ON);
    int h = (int)(54.0f * f_clamp(v->pz / v->fence_alt, 0.0f, 1.0f));
    oled_fill_rect(ALT_BAR_X + 1, 63 - h, 3, h, OLED_ON);
}

void ui_draw(const ui_view_t *v)
{
    if (last_mode == M_DISARMED && v->mode != M_DISARMED) {      /* just armed */
        trail_len = 0;  trail_next = 0;  t_armed = v->t_ms;
    }
    last_mode = v->mode;
    if (v->mode != M_DISARMED && v->t_ms - trail_t >= TRAIL_MS) {
        trail_t = v->t_ms;
        trail_x[trail_next] = (int8_t)mx(v->px);
        trail_y[trail_next] = (int8_t)my(v->py);
        trail_next = (uint8_t)((trail_next + 1u) % TRAIL_N);
        if (trail_len < TRAIL_N) { trail_len++; }
    }
    oled_clear();
    draw_map(v);
    draw_panel(v);
}
