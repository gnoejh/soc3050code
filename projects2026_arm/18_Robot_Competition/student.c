/*
 * student.c - YOUR sumo robot.  This is the file you improve and submit.
 *
 * The rules of this file (the class tournament checks every one):
 *
 *   1. Write strategy_step() and strategy_name[].  Nothing else is called.
 *   2. Keep state ONLY in *mem (64 bytes, zeroed at the start of each round).
 *      No global variables and no `static` variables inside functions: the
 *      host league rejects an object file with any (host/run.bat checks).
 *      `static` helper FUNCTIONS and `static const` tables are fine.
 *   3. Be deterministic: no clocks, no rand().  Same sensors in, same
 *      command out.  Your result must be reproducible from its seed.
 *   4. Return within the time budget: 20 000 CPU cycles per call, measured
 *      on the chip (the `stats` command and the $RESULT line show your worst).
 *   5. Include only strategy.h and the C library's headers.
 *
 * What this starter does - deliberately badly, so there is room to win:
 *   - spins on the spot until the centre sensor sees something,
 *   - drives at it at 80 %,
 *   - backs off when a FRONT edge sensor sees the white line.
 * It ignores the side sensors, the rear edge sensors, the bump switch and
 * the encoders.  Each of those is a Lab part.
 */
#include "strategy.h"

const char strategy_name[] = "student";    /* <= 12 characters: your tag    */

typedef struct {
    uint8_t mode;
    int16_t timer;          /* calls (20 ms each) left in this manoeuvre     */
} my_state_t;
STRATEGY_STATE_FITS(my_state_t);           /* a compile error if > 64 bytes */

enum { SEARCH, BACK_OFF };

void strategy_step(const robot_view_t *in, robot_cmd_t *out, strategy_mem_t *mem)
{
    my_state_t *s = STRATEGY_STATE(my_state_t, mem);
    out->left = 0;
    out->right = 0;

    if (in->t_ms < 0) {                    /* countdown: wheels are locked  */
        return;                            /* anyway - plan, do not move    */
    }

    /* The edge first, always.  Losing by driving out is the commonest loss. */
    if (in->edge & (EDGE_FL | EDGE_FR)) {
        s->mode = BACK_OFF;
        s->timer = 15;                     /* 300 ms                        */
    }
    if (s->mode == BACK_OFF) {
        out->left = -80;
        out->right = -80;
        if (--s->timer <= 0) { s->mode = SEARCH; }
        return;
    }

    /* TODO (Lab Part 3): the side sensors say which way to turn.
     * TODO (Lab Part 4): the rear edge sensors - being pushed out backwards.
     * TODO (Lab Part 5): the bump switch - you are being hit from the side.  */
    if (in->dist_mm[DIST_CENTRE] != DIST_NONE) {
        out->left = 80;                    /* there it is: go               */
        out->right = 80;
    } else {
        out->left = 40;                    /* look around, clockwise        */
        out->right = -40;
    }
}
