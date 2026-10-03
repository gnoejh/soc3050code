/*
 * alice.c - an example of a SUBMITTED strategy file, for the host league.
 *
 *   host\run.bat strategies\alice.c strategies\bob.c
 *
 * The league compiles this with -DSTRATEGY_PREFIX=alice, so strategy_step
 * below becomes alice_step and it plays under the name "alice".  It is the
 * starter strategy with each Lab part done once, plainly:
 *   - all four edge sensors, with a reflex that remembers which way to turn
 *   - the side sensors steer, and the last side seen picks the search turn
 *   - a hit from the side turns the nose - the strong side - to face it
 * It is a sparring partner, not the answer.
 */
#include "strategy.h"

const char strategy_name[] = "alice";

typedef struct {
    uint8_t mode;
    int8_t  turn;           /* +1 left, -1 right                            */
    int16_t timer;
} alice_t;
STRATEGY_STATE_FITS(alice_t);

enum { SEARCH, BACK, TURN };

static void go(robot_cmd_t *o, int l, int r) { o->left = (int8_t)l; o->right = (int8_t)r; }

void strategy_step(const robot_view_t *in, robot_cmd_t *out, strategy_mem_t *mem)
{
    alice_t *s = STRATEGY_STATE(alice_t, mem);
    int L = in->dist_mm[DIST_LEFT] != DIST_NONE;
    int C = in->dist_mm[DIST_CENTRE] != DIST_NONE;
    int R = in->dist_mm[DIST_RIGHT] != DIST_NONE;
    go(out, 0, 0);

    if (in->t_ms < 0) {                         /* plan the first turn      */
        if (s->turn == 0) { s->turn = 1; }
        if (L) { s->turn = 1; }
        if (R) { s->turn = -1; }
        return;
    }

    /* 1. edges */
    if (in->edge & (EDGE_FL | EDGE_FR)) {
        s->mode = BACK;  s->timer = 10;
        s->turn = (in->edge & EDGE_FL) ? -1 : 1;
    } else if (in->edge & (EDGE_BL | EDGE_BR)) {
        s->mode = SEARCH;
        go(out, 100, 100);                      /* being pushed out: go!    */
        return;
    }
    if (s->mode == BACK) {
        go(out, -100, -100);
        if (--s->timer <= 0) { s->mode = TURN; s->timer = 8; }
        return;
    }
    if (s->mode == TURN) {
        go(out, -80 * s->turn, 80 * s->turn);
        if (--s->timer <= 0) { s->mode = SEARCH; }
        return;
    }

    /* 2. hit from the side or behind: face the attacker */
    if (in->bump & BUMP_LEFT)  { s->turn = 1;  go(out, -100, 100); return; }
    if (in->bump & BUMP_RIGHT) { s->turn = -1; go(out, 100, -100); return; }
    if (in->bump & BUMP_BACK)  { go(out, -100 * s->turn, 100 * s->turn); return; }

    /* 3. steer at it */
    if (L && !R)      { s->turn = 1;  go(out, 10, 100); }
    else if (R && !L) { s->turn = -1; go(out, 100, 10); }
    else if (C || (L && R)) { go(out, 100, 100); }
    else { go(out, -70 * s->turn, 70 * s->turn); }      /* 4. search */
}
