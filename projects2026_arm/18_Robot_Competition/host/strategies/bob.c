/*
 * bob.c - a second example submission: the edge-walker.
 *
 * Bob's idea: the safest place is not the centre but just inside the line,
 * driving round it - and anything that comes at him gets met head-on.  He
 * uses the encoders to count how far he has driven without seeing anyone,
 * and cuts across the ring when that passes one lap.
 *
 * It shows the encoder fields and a `static const` table (allowed: it is
 * read-only, and lives in flash, not RAM).
 */
#include "strategy.h"

const char strategy_name[] = "bob";

typedef struct {
    uint8_t mode;
    int16_t timer;
    int32_t start_mm;       /* encoder reading when this lap began          */
} bob_t;
STRATEGY_STATE_FITS(bob_t);

enum { CRUISE, PEEL, CROSS, CHARGE };

/* Wheel commands for each mode: left, right. */
static const int8_t table[4][2] = {
    { 90, 90 },     /* CRUISE: straight until the line                     */
    { 20, 90 },     /* PEEL:   swing left, along the line                  */
    { 100, 100 },   /* CROSS:  cut across the ring                         */
    { 100, 100 },   /* CHARGE                                              */
};

void strategy_step(const robot_view_t *in, robot_cmd_t *out, strategy_mem_t *mem)
{
    bob_t *s = STRATEGY_STATE(bob_t, mem);
    out->left = out->right = 0;
    if (in->t_ms < 0) { return; }

    int32_t travel = (in->enc_l + in->enc_r) / 2;
    int see = in->dist_mm[DIST_CENTRE] != DIST_NONE;

    if (in->edge & (EDGE_FL | EDGE_FR)) { s->mode = PEEL; s->timer = 12; }
    else if (see || (in->bump & BUMP_FRONT)) { s->mode = CHARGE; }
    else if (in->dist_mm[DIST_LEFT] != DIST_NONE)  { out->left = -60; out->right = 60; return; }
    else if (in->dist_mm[DIST_RIGHT] != DIST_NONE) { out->left = 60; out->right = -60; return; }
    else if (s->mode == CHARGE) { s->mode = CRUISE; s->start_mm = travel; }

    if (s->mode == PEEL && --s->timer <= 0) { s->mode = CRUISE; }
    if (s->mode == CRUISE && travel - s->start_mm > 2400) {   /* ~one lap   */
        s->mode = CROSS;  s->start_mm = travel;
    }
    if (s->mode == CROSS && travel - s->start_mm > 300) { s->mode = CRUISE; }
    out->left = table[s->mode][0];
    out->right = table[s->mode][1];
}
