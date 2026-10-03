/*
 * register.h - force-included into every extra strategy file the host
 * league compiles (gcc -include register.h -DSTRATEGY_PREFIX=alice alice.c).
 *
 * The student writes strategy_step(); strategy.h's macro has already renamed
 * it alice_step.  This adds one constructor - a function the C runtime calls
 * before main() - that hands alice_step to the league under the name "alice".
 * So the league finds every file it was linked with, and nobody edits a list.
 * (A GCC extension: fine on the PC, never used on the chip.)
 */
#include "strategy.h"

void league_register(const char *name, strategy_fn fn);

#define REG_STR2(x) #x
#define REG_STR(x)  REG_STR2(x)

__attribute__((constructor))
static void STRATEGY_CAT(STRATEGY_PREFIX, _register)(void)
{
    league_register(REG_STR(STRATEGY_PREFIX), strategy_step);
}
