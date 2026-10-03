/*
 * track.c - four tracks, from easy to expert, written as turtle commands
 *
 *   1 OVAL      two straights, two 400 mm half-circles       easy
 *   2 S-BENDS   an oval with an S-bend in each straight      medium
 *   3 HAIRPINS  a serpentine: three 120 mm U-turns           hard
 *   4 FIGURE-8  two 270-degree loops, a crossing, a gap      expert
 *
 * Every track fits a 2:1 box (the OLED's shape) and is CLOSED: the turtle
 * ends where it started, pointing the same way.  host/sitl.c prints how far
 * each one misses before the last point is snapped shut (close_err) - the
 * arithmetic below is checked, not trusted.
 *
 * Why a turtle?  Because a track designer thinks in straights and corners,
 * not in coordinates - and because a closed track is then a fact you can
 * check by adding up angles: OVAL turns +360 degrees, FIGURE-8 turns
 * -270 + 270 = 0.  Lab Part 8 designs a fifth track this way.
 */
#include "track.h"
#include "fmath.h"

track_t track;

/* A command: deg == 0 is a straight of `mm`; otherwise an arc of radius `mm`
 * turning `deg` degrees, + = left (anticlockwise).  gap = no tape here. */
typedef struct { int16_t mm; int16_t deg; uint8_t gap; } turtle_t;

typedef struct {
    const char     *name, *level;
    float           x0, y0, th0_deg;   /* where the turtle starts            */
    uint8_t         crossing;
    uint8_t         ncmd;
    const turtle_t *cmd;
} track_def_t;

/* 1 OVAL - start mid-way along the bottom straight, heading east. */
static const turtle_t oval[] = {
    { 600, 0, 0 }, { 400, 180, 0 }, { 1200, 0, 0 }, { 400, 180, 0 }, { 600, 0, 0 },
};

/* 2 S-BENDS - each straight carries a wiggle: +60, -120, +60 on 220 mm.
 * A wiggle ends on the line it started on, pointing the same way, so the
 * oval still closes.  The top run is the bottom run turned through 180. */
static const turtle_t sbends[] = {
    { 150, 0, 0 }, { 220, 60, 0 }, { 220, -120, 0 }, { 220, 60, 0 }, { 150, 0, 0 },
    { 350, 180, 0 },
    { 150, 0, 0 }, { 220, 60, 0 }, { 220, -120, 0 }, { 220, 60, 0 }, { 150, 0, 0 },
    { 350, 180, 0 },
};

/* 3 HAIRPINS - a serpentine.  East 1500, U-turn left, west 1000, U-turn
 * RIGHT, east 1000, U-turn left, west 1500, then two 90s down the left side.
 * Worked through by hand in the comments of host/sitl.c's closure test. */
static const turtle_t hairpins[] = {
    { 750, 0, 0 },  { 120, 180, 0 }, { 1000, 0, 0 }, { 120, -180, 0 },
    { 1000, 0, 0 }, { 120, 180, 0 }, { 1500, 0, 0 }, { 200, 90, 0 },
    { 320, 0, 0 },  { 200, 90, 0 },  { 750, 0, 0 },
};

/* 4 FIGURE-8 - R = 420.  Out of the crossing at 45 degrees, a 270-degree
 * loop to the RIGHT, back through the crossing at 90 degrees to the way in,
 * a 270-degree loop to the LEFT, home.  The diagonal back carries a 126 mm
 * GAP in the tape, before the crossing.  Straight lengths: R/2, then
 * R/4 + 0.3R (gap) + 1.45R = 2R, then 1.5R. */
static const turtle_t fig8[] = {
    { 210, 0, 0 }, { 420, -270, 0 },
    { 105, 0, 0 }, { 126, 0, 1 }, { 609, 0, 0 },
    { 420, 270, 0 }, { 630, 0, 0 },
};

static const track_def_t defs[TRACK_COUNT] = {
    { "OVAL",     "easy",   0.0f,   0.0f,   0.0f,  0, sizeof oval / sizeof oval[0],         oval },
    { "S-BENDS",  "medium", 0.0f,   0.0f,   0.0f,  0, sizeof sbends / sizeof sbends[0],     sbends },
    { "HAIRPINS", "hard",   0.0f,   0.0f,   0.0f,  0, sizeof hairpins / sizeof hairpins[0], hairpins },
    { "FIGURE-8", "expert", 148.49f, 148.49f, 45.0f, 1, sizeof fig8 / sizeof fig8[0],       fig8 },
};

const char *track_name(int id)  { return defs[id % TRACK_COUNT].name; }
const char *track_level(int id) { return defs[id % TRACK_COUNT].level; }

/* ------------------------------------------------------------------------ */
static float s_acc;                       /* running arc length, mm         */

static void push(float x, float y, int gap)
{
    if (track.n >= TRACK_MAX_PTS) { return; }          /* host test catches it */
    int16_t ix = (int16_t)(x + (x >= 0.0f ? 0.5f : -0.5f));
    int16_t iy = (int16_t)(y + (y >= 0.0f ? 0.5f : -0.5f));
    if (track.n > 0) {
        const track_pt_t *p = &track.pt[track.n - 1];
        float dx = (float)(ix - p->x), dy = (float)(iy - p->y);
        s_acc += f_sqrt(dx * dx + dy * dy);
        if (gap) { track.gap[(track.n - 1) >> 3] |= (uint8_t)(1u << ((track.n - 1) & 7)); }
    }
    track.pt[track.n].x = ix;
    track.pt[track.n].y = iy;
    track.pt[track.n].s = (uint16_t)(s_acc + 0.5f);
    track.n++;
    if (ix < track.xmin) { track.xmin = ix; }
    if (ix > track.xmax) { track.xmax = ix; }
    if (iy < track.ymin) { track.ymin = iy; }
    if (iy > track.ymax) { track.ymax = iy; }
}

int track_build(int id)
{
    const track_def_t *d = &defs[id % TRACK_COUNT];
    track.id = id % TRACK_COUNT;
    track.n = 0;
    track.xmin = track.ymin = 32767;
    track.xmax = track.ymax = -32767;
    track.crossing = d->crossing;
    for (unsigned i = 0; i < sizeof track.gap; i++) { track.gap[i] = 0; }
    s_acc = 0.0f;

    float x = d->x0, y = d->y0, th = d->th0_deg * (F_PI / 180.0f);
    track.x0 = x;  track.y0 = y;  track.th0 = th;
    push(x, y, 0);

    for (unsigned c = 0; c < d->ncmd; c++) {
        const turtle_t *t = &d->cmd[c];
        if (t->deg == 0) {
            /* a straight, cut into pieces of at most 100 mm so that the
             * judge's "which segment am I on" search moves smoothly */
            int   k = (t->mm + 99) / 100;
            float step = (float)t->mm / (float)k;
            for (int j = 0; j < k; j++) {
                x += step * f_cos(th);
                y += step * f_sin(th);
                push(x, y, t->gap);
            }
        } else {
            /* an arc.  A chord c on radius R strays c*c/(8R) from the arc;
             * keep that under 1 mm: c = sqrt(8 R). */
            float R   = (float)t->mm;
            float ang = (float)t->deg * (F_PI / 180.0f);
            float Rs  = ang > 0.0f ? R : -R;            /* signed: + = centre on the left */
            float cx  = x - Rs * f_sin(th), cy = y + Rs * f_cos(th);
            float len = R * f_abs(ang);
            int   k   = (int)(len / f_sqrt(8.0f * R)) + 1;
            for (int j = 1; j <= k; j++) {
                float ph = th + ang * (float)j / (float)k;
                x = cx + Rs * f_sin(ph);
                y = cy - Rs * f_cos(ph);
                push(x, y, t->gap);
            }
            th += ang;
        }
    }
    /* Closed?  The last point should land on the first. */
    track_pt_t *last = &track.pt[track.n - 1];
    float ex = (float)(last->x - track.pt[0].x), ey = (float)(last->y - track.pt[0].y);
    track.close_err = f_sqrt(ex * ex + ey * ey);
    last->x = track.pt[0].x;
    last->y = track.pt[0].y;
    track.length = last->s;
    return track.n;
}

float track_seg_dist(int seg, float px, float py, float *t, int *side)
{
    const track_pt_t *a = &track.pt[seg], *b = &track.pt[seg + 1];
    float ax = (float)a->x, ay = (float)a->y;
    float dx = (float)b->x - ax, dy = (float)b->y - ay;
    float qx = px - ax, qy = py - ay;
    float L2 = dx * dx + dy * dy;
    float u  = L2 > 0.0f ? (qx * dx + qy * dy) / L2 : 0.0f;
    u = f_clamp(u, 0.0f, 1.0f);
    float rx = qx - u * dx, ry = qy - u * dy;
    *t = u;
    *side = (dx * qy - dy * qx) >= 0.0f ? 1 : -1;   /* cross product: left is + */
    return f_sqrt(rx * rx + ry * ry);
}
