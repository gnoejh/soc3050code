/*
 * scene.c - draw the dohyo, the robots, their sensor cones and the score
 *
 * Screen: the ring fills the left 64x64 square (31 px radius, so 1 px is
 * 12.4 mm); the right 60 columns are ten characters of text.
 *
 *   robot 0 (the student, or you): an OPEN circle
 *   robot 1 (the opponent):        a FILLED circle
 *   heading: a line from the centre out through the nose
 *   cones (SEL toggles): a sensor that sees something draws a line to it,
 *   one that sees nothing a short stub - so you can watch what the robot
 *   "knows" while it fights.
 */
#include "scene.h"
#include "oled.h"
#include "world.h"
#include "fmath.h"

#define CX     32
#define CY     32
#define RPX    31
#define SCALE  ((float)RPX / RING_R)                 /* px per metre */
#define TX     68                                    /* text column  */

static int px(float x) { return CX + (int)(x * SCALE + (x >= 0.0f ? 0.5f : -0.5f)); }
static int py(float y) { return CY - (int)(y * SCALE + (y >= 0.0f ? 0.5f : -0.5f)); }  /* y up */

static void robot(const snap_t *s, int i)
{
    int x = px(s->x[i]), y = py(s->y[i]);
    int r = (int)(ROBOT_R * SCALE + 0.5f);
    float c = w_cos(s->th[i]), sn = w_sin(s->th[i]);
    if (i == 0) {
        oled_circle(x, y, r, OLED_ON);
        oled_line(x, y, x + (int)(c * (r + 2)), y - (int)(sn * (r + 2)), OLED_ON);
    } else {
        oled_fill_circle(x, y, r, OLED_ON);
        oled_line(x, y, x + (int)(c * r), y - (int)(sn * r), OLED_XOR);
        oled_pixel(x + (int)(c * (r + 2)), y - (int)(sn * (r + 2)), OLED_ON);
    }
    if (!s->cones) { return; }
    static const float axis[N_DIST] = { SENSOR_SPREAD, 0.0f, -SENSOR_SPREAD };
    float nx = s->x[i] + ROBOT_R * c, ny = s->y[i] + ROBOT_R * sn;   /* the nose */
    for (int k = 0; k < N_DIST; k++) {
        float a = s->th[i] + axis[k];
        float len = s->dist[i][k] == DIST_NONE ? 0.05f : s->dist[i][k] * 0.001f;
        oled_line(px(nx), py(ny), px(nx + len * w_cos(a)), py(ny + len * w_sin(a)), OLED_ON);
    }
}

static void menu(const snap_t *s)
{
    oled_text(0, 0, "SUMO  lesson 18", OLED_ON);
    oled_hline(0, 9, 128, OLED_ON);
    if (s->league) {
        oled_text(0, 14, "> TOURNAMENT", OLED_ON);
        oled_text(0, 24, "  student.c vs all 5", OLED_ON);
    } else {
        oled_printf(0, 14, "> vs %s", s->opp_name);
        oled_text(0, 24, s->opp_style, OLED_ON);
    }
    oled_printf(0, 36, "driver: %s", s->joystick ? "JOYSTICK" : s->name[0]);
    if (s->speed) { oled_printf(0, 46, "speed x%u  (knob)", s->speed); }
    else          { oled_text(0, 46, "speed MAX (knob)", OLED_ON); }
    oled_text(0, 56, "A go B next SEL drv", OLED_ON);
}

static void result(const snap_t *s)
{
    oled_text(0, 0, "TOURNAMENT", OLED_ON);
    oled_hline(0, 9, 128, OLED_ON);
    oled_text(0, 14, s->name[0], OLED_ON);
    oled_printf(0, 26, "W %u  D %u  L %u", s->W, s->D, s->L);
    oled_printf(0, 38, "points %u", 3u * s->W + s->D);
    oled_text(0, 50, "$RESULT on serial", OLED_ON);
}

void scene_draw(const snap_t *s)
{
    oled_clear();
    if (s->screen == SCR_MENU)   { menu(s);   return; }
    if (s->screen == SCR_RESULT) { result(s); return; }

    oled_circle(CX, CY, RPX, OLED_ON);                           /* the edge   */
    oled_circle(CX, CY, RPX - (int)(LINE_W * SCALE + 0.5f), OLED_ON); /* line */
    oled_pixel(CX, CY, OLED_ON);
    robot(s, 0);
    robot(s, 1);

    oled_printf(TX, 0,  "o%.9s", s->name[0]);
    oled_printf(TX, 8,  "*%.9s", s->name[1]);
    oled_printf(TX, 18, "R%u  %u-%u", s->round, s->won[0], s->won[1]);
    if (s->t_ms < 0) { oled_printf(TX, 28, "ready %ld", (long)((-s->t_ms + 999) / 1000)); }
    else             { oled_printf(TX, 28, "%2ld.%ld s", (long)(s->t_ms / 1000), (long)(s->t_ms / 100 % 10)); }
    if (s->speed) { oled_printf(TX, 38, "x%u", s->speed); }
    else          { oled_text(TX, 38, "MAX", OLED_ON); }
    oled_text(TX, 46, s->msg, OLED_ON);
    if (s->tour_on) { oled_printf(TX, 56, "%u/%u %u-%u-%u", s->tour_done + 1u, s->tour_n, s->W, s->D, s->L); }
}
