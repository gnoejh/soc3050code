/*
 * bots.c - five built-in opponents, each a personality
 *
 *   Bull      spins until it sees you, then charges flat out.  Watches the
 *             edge only while it sees nothing - so it can be led off it.
 *   Matador   turns to face you and waits.  When you come in fast it steps
 *             aside, comes round, and pushes you from the side.
 *   Spinner   drives in wide circles, scanning; turns in and pushes.
 *   Turtle    never leaves its start spot.  Keeps its nose - its strongest
 *             side - pointed at you, and pushes only when touched.
 *   Coward    turns and runs from anything it sees.  Hard to lose to,
 *             annoyingly hard to beat before the clock runs out.
 *
 * They obey exactly the rules a student's strategy does: they see only
 * robot_view_t, keep state only in their 64-byte strategy_mem_t, and keep no
 * globals.  Read them for technique - and for their weaknesses.
 *
 * Time is counted in CALLS: the referee calls every 20 ms, so 15 calls is
 * 300 ms.
 */
#include "strategy.h"

#define MS(x) ((int16_t)((x) / 20))          /* milliseconds -> calls */

typedef struct {
    uint8_t  mode;          /* what the bot is doing                       */
    int8_t   turn;          /* +1 left, -1 right: which way to turn next   */
    int16_t  timer;         /* calls left in the current manoeuvre         */
    uint16_t last_d;        /* centre distance at the previous call        */
    int16_t  closing;       /* mm per call the gap closed by, filtered     */
    int16_t  patience;      /* calls spent waiting for a charge            */
} bot_state_t;
STRATEGY_STATE_FITS(bot_state_t);

enum { M_SEARCH, M_ATTACK, M_BACK, M_TURN, M_DODGE, M_FLANK, M_FLEE };

static void go(robot_cmd_t *out, int l, int r)
{
    out->left = (int8_t)l;  out->right = (int8_t)r;
}

static int seen(const robot_view_t *in, int k) { return in->dist_mm[k] != DIST_NONE; }
static int any_seen(const robot_view_t *in)
{
    return seen(in, DIST_LEFT) || seen(in, DIST_CENTRE) || seen(in, DIST_RIGHT);
}

/* The edge reflex nearly every bot shares.  Returns 1 if it took over.
 * Front corner on white: back off, then turn away from that corner.
 * Back corner on white: drive forward out of danger. */
static int edge_reflex(const robot_view_t *in, robot_cmd_t *out, bot_state_t *s)
{
    if (in->edge & (EDGE_FL | EDGE_FR)) {
        s->mode = M_BACK;  s->timer = MS(240);
        s->turn = (in->edge & EDGE_FL) ? -1 : 1;    /* left saw it: turn right */
    } else if (in->edge & (EDGE_BL | EDGE_BR)) {
        s->mode = M_SEARCH;
        go(out, 100, 100);
        return 1;
    }
    if (s->mode == M_BACK) {
        go(out, -100, -100);
        if (--s->timer <= 0) { s->mode = M_TURN;  s->timer = MS(200); }
        return 1;
    }
    if (s->mode == M_TURN) {
        go(out, -70 * s->turn, 70 * s->turn);
        if (--s->timer <= 0) { s->mode = M_SEARCH; }
        return 1;
    }
    return 0;
}

/* Steer at the opponent: the side sensor that sees it pulls that way. */
static void track(const robot_view_t *in, robot_cmd_t *out, bot_state_t *s, int fast, int slow)
{
    int l = seen(in, DIST_LEFT), c = seen(in, DIST_CENTRE), r = seen(in, DIST_RIGHT);
    if (l && !r)      { s->turn = 1;  go(out, slow, fast); }
    else if (r && !l) { s->turn = -1; go(out, fast, slow); }
    else if (c || (l && r)) { go(out, fast, fast); }
}

static void update_closing(const robot_view_t *in, bot_state_t *s)
{
    uint16_t d = in->dist_mm[DIST_CENTRE];
    if (d != DIST_NONE && s->last_d != DIST_NONE && s->last_d != 0) {
        int16_t now = (int16_t)((int)s->last_d - (int)d);
        s->closing = (int16_t)((3 * s->closing + now) / 4);
    } else {
        s->closing = 0;
    }
    s->last_d = d;
}

/* ============================================================== Bull ===== */
void bull_step(const robot_view_t *in, robot_cmd_t *out, strategy_mem_t *mem)
{
    bot_state_t *s = STRATEGY_STATE(bot_state_t, mem);
    go(out, 0, 0);
    if (in->t_ms < 0) { s->turn = 1; return; }

    if (any_seen(in) || (in->bump & BUMP_FRONT)) {     /* the red cape   */
        s->mode = M_ATTACK;
        if (in->bump & BUMP_FRONT) { go(out, 100, 100); return; }
        track(in, out, s, 100, 40);
        return;
    }
    if (edge_reflex(in, out, s)) { return; }           /* only when blind */
    if (in->bump & BUMP_LEFT)  { s->turn = 1; }
    if (in->bump & BUMP_RIGHT) { s->turn = -1; }
    go(out, -60 * s->turn, 60 * s->turn);              /* spin and look   */
}

/* =========================================================== Matador ===== */
void matador_step(const robot_view_t *in, robot_cmd_t *out, strategy_mem_t *mem)
{
    bot_state_t *s = STRATEGY_STATE(bot_state_t, mem);
    go(out, 0, 0);
    update_closing(in, s);
    if (in->t_ms < 0) { s->turn = 1; return; }
    if (edge_reflex(in, out, s)) { return; }

    switch (s->mode) {
    case M_DODGE:                       /* swing the hips out of the way */
        go(out, -100 * s->turn, 100 * s->turn);
        if (--s->timer <= 0) { s->mode = M_FLANK; s->timer = MS(260); }
        return;
    case M_FLANK:                       /* dart past ...                 */
        go(out, 100, 100);
        if (--s->timer <= 0) { s->mode = M_SEARCH; s->turn = (int8_t)-s->turn; }
        return;
    default: break;
    }

    uint16_t d = in->dist_mm[DIST_CENTRE];
    if (d != DIST_NONE && d < 140 && s->closing > 6) { /* > 0.3 m/s, close */
        s->mode = M_DODGE;  s->timer = MS(120);
        s->turn = (in->dist_mm[DIST_LEFT] != DIST_NONE) ? -1 : 1;
        return;
    }
    if (in->bump & (BUMP_LEFT | BUMP_BACK | BUMP_RIGHT)) { /* hit from aside */
        s->turn = (in->bump & BUMP_LEFT) ? 1 : -1;
        go(out, -100 * s->turn, 100 * s->turn);
        return;
    }
    if ((in->bump & BUMP_FRONT) || (d != DIST_NONE && d < 60)) {
        go(out, 100, 100);              /* ... and push                  */
        return;
    }
    if (any_seen(in)) {
        if (seen(in, DIST_CENTRE) && !seen(in, DIST_LEFT) && !seen(in, DIST_RIGHT)) {
            /* Facing it: wait for the charge - but not forever.  After 2 s
             * of nothing, walk in slowly and provoke one. */
            if (++s->patience > MS(2000)) { go(out, 45, 45); }
            else                         { go(out, 0, 0); }
        } else {
            track(in, out, s, 40, -40); /* turn on the spot to face it    */
        }
        return;
    }
    go(out, -45 * s->turn, 45 * s->turn);              /* look around    */
}

/* =========================================================== Spinner ===== */
void spinner_step(const robot_view_t *in, robot_cmd_t *out, strategy_mem_t *mem)
{
    bot_state_t *s = STRATEGY_STATE(bot_state_t, mem);
    go(out, 0, 0);
    if (in->t_ms < 0) { s->turn = 1; return; }
    if (edge_reflex(in, out, s)) { return; }

    if (in->bump & BUMP_FRONT) { go(out, 100, 100); return; }
    if (in->bump & BUMP_LEFT)  { go(out, -80, 80);  return; }
    if (in->bump & BUMP_RIGHT) { go(out, 80, -80);  return; }
    if (any_seen(in)) { track(in, out, s, 90, 20); return; }
    if (s->turn > 0) { go(out, 35, 85); } else { go(out, 85, 35); }   /* arc */
}

/* ============================================================ Turtle ===== */
void turtle_step(const robot_view_t *in, robot_cmd_t *out, strategy_mem_t *mem)
{
    bot_state_t *s = STRATEGY_STATE(bot_state_t, mem);
    go(out, 0, 0);
    if (in->t_ms < 0) { s->turn = 1; return; }
    if (edge_reflex(in, out, s)) { return; }

    uint16_t d = in->dist_mm[DIST_CENTRE];
    if ((in->bump & BUMP_FRONT) || (d != DIST_NONE && d < 40)) {
        go(out, 100, 100);              /* touched head-on: shove back    */
        return;
    }
    if (in->bump & (BUMP_LEFT | BUMP_BACK)) { go(out, -100, 100); return; }
    if (in->bump & BUMP_RIGHT)              { go(out, 100, -100); return; }
    if (any_seen(in)) { track(in, out, s, 0, -50); return; }   /* face it */
    go(out, -40 * s->turn, 40 * s->turn);
}

/* ============================================================ Coward ===== */
void coward_step(const robot_view_t *in, robot_cmd_t *out, strategy_mem_t *mem)
{
    bot_state_t *s = STRATEGY_STATE(bot_state_t, mem);
    go(out, 0, 0);
    if (in->t_ms < 0) { s->turn = 1; return; }
    if (edge_reflex(in, out, s)) { return; }

    if (in->bump & BUMP_BACK) { go(out, 100, 100); return; }  /* bolt  */
    if (s->mode == M_FLEE) {
        if (s->timer > MS(400)) { go(out, -90 * s->turn, 90 * s->turn); }
        else                    { go(out, 80, 80); }
        if (--s->timer <= 0) { s->mode = M_SEARCH; }
        return;
    }
    if (any_seen(in) || (in->bump & (BUMP_FRONT | BUMP_LEFT | BUMP_RIGHT))) {
        s->mode = M_FLEE;  s->timer = MS(700);           /* turn 300, run 400 */
        s->turn = seen(in, DIST_LEFT) ? -1 : 1;
        return;
    }
    go(out, 30, 50);                                     /* amble          */
}
