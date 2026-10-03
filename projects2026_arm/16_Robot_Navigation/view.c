/*
 * view.c - one frame of the navigation display
 *
 *   y 0..55   the map, 4 x 4 pixels per cell, as the ROBOT believes it:
 *               wall      solid block
 *               unknown   two dots - a grey checker you watch disappear
 *               floor     blank
 *             the planned path, a line through cell centres
 *             the goal, a flag
 *             the TRUE robot, a filled disc with a heading tick
 *             the ESTIMATED robot, a ring: where it thinks it is
 *             (SEL) true walls the robot has not mapped yet, as outlines
 *   y 57..63  status: level, mode, run time, collisions, replans
 *
 * Every page changes when the robot moves, so a frame is usually 2-4 dirty
 * pages (~260-520 bytes on I2C), not the full 1 KB - oled_flush() decides.
 */
#include "oled.h"
#include "fmath.h"
#include "mathx.h"
#include "sim.h"
#include "astar.h"
#include "nav.h"
#include "world.h"
#include "view.h"

#define MAP_PX_H  (GRID_H * VIEW_PX)

static int px_x(float x_mm) { return (int)(x_mm * VIEW_PX / CELL_MM); }
static int px_y(float y_mm) { return (int)(MAP_PX_H - y_mm * VIEW_PX / CELL_MM); }
static int cell_px(int x)   { return x * VIEW_PX; }
static int cell_py(int y)   { return (GRID_H - 1 - y) * VIEW_PX; }

void view_draw(int show_truth, uint32_t run_ms)
{
    oled_clear();

    /* the robot's map */
    for (int y = 0; y < GRID_H; y++) {
        for (int x = 0; x < GRID_W; x++) {
            int px = cell_px(x), py = cell_py(y), c = nav_cell(x, y);
            if (c == MAP_OCC) {
                oled_fill_rect(px, py, VIEW_PX, VIEW_PX, OLED_ON);
            } else {
                if (c == MAP_UNKNOWN) {
                    oled_pixel(px + 1, py + 1, OLED_ON);
                    oled_pixel(px + 3, py + 3, OLED_ON);
                }
                if (show_truth && world_occ(x, y)) {
                    oled_rect(px, py, VIEW_PX, VIEW_PX, OLED_ON);
                }
            }
        }
    }

    /* the plan, from where the robot is now to the goal */
    uint16_t len, from;
    const uint16_t *p = nav_path(&len, &from);
    for (uint16_t k = from; k + 1u < len; k++) {
        oled_line(cell_px(CELL_X(p[k])) + 2, cell_py(CELL_Y(p[k])) + 2,
                  cell_px(CELL_X(p[k + 1u])) + 2, cell_py(CELL_Y(p[k + 1u])) + 2, OLED_ON);
    }

    /* the goal: a flag on a pole */
    int gx, gy;
    nav_get_goal(&gx, &gy);
    if (gx >= 0) {
        int px = cell_px(gx) + 1, py = cell_py(gy) + 3;
        oled_vline(px, py - 6, 7, OLED_ON);
        oled_fill_rect(px + 1, py - 6, 3, 3, OLED_ON);
    }

    /* truth: a disc and a heading tick */
    const world_t *w = world();
    int tx = px_x(w->x), ty = px_y(w->y);
    oled_fill_circle(tx, ty, 2, OLED_ON);
    oled_line(tx, ty, tx + (int)(4.0f * m_cos(w->th)), ty - (int)(4.0f * m_sin(w->th)), OLED_ON);

    /* belief: a ring */
    float ex, ey, eth;
    nav_pose(&ex, &ey, &eth);
    oled_circle(px_x(ex), px_y(ey), 3, OLED_ON);

    /* status line */
    static const char *const m[] = { "IDLE", "CAL", "RUN", "GOAL", "NOWAY" };
    int mode = nav_mode();
    oled_hline(0, MAP_PX_H, OLED_W, OLED_ON);
    oled_printf(0, 57, "L%d %s %lu.%lus C%u R%u", w->level + 1, m[mode],
                (unsigned long)(run_ms / 1000u), (unsigned long)(run_ms / 100u % 10u),
                (unsigned)w->collisions, (unsigned)nav_stats()->replans);
}
