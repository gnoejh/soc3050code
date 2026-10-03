/*
 * cheater.c - what the league REJECTS, and why.  Lab Part 7 runs it.
 *
 *   host\run.bat strategies\cheater.c
 *
 * It keeps a counter in a `static` variable instead of in *mem.  That looks
 * harmless, but there is ONE copy of it for every robot running this code:
 * cheater vs cheater would share it, and it is never reset between rounds or
 * matches - so match 7's result depends on matches 1-6, and the referee can
 * no longer re-run match 7 from its seed.  run.bat finds the variable in the
 * object file (nm shows it as 'b', a local in .bss) and refuses to link it.
 */
#include "strategy.h"

const char strategy_name[] = "cheater";

void strategy_step(const robot_view_t *in, robot_cmd_t *out, strategy_mem_t *mem)
{
    static int32_t calls;           /* <- the offence                       */
    (void)mem;
    calls++;
    out->left = out->right = (in->t_ms >= 0 && (calls & 64)) ? 100 : -100;
}
