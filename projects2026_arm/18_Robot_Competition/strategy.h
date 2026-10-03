/*
 * strategy.h - the contract between a sumo robot's BRAIN and its WORLD
 *
 *     void strategy_step(const robot_view_t *in, robot_cmd_t *out,
 *                        strategy_mem_t *mem);
 *
 * Every 20 ms of match time the referee calls each robot's strategy once.
 *
 *   in   what THIS robot's own sensors report - nothing else.  No positions,
 *        no opponent velocity, no map.  A real sumo robot does not know where
 *        it is either.
 *   out  two wheel commands, -100..100 % of full motor voltage.
 *   mem  64 bytes that belong to this robot alone, zeroed at the start of
 *        every round.  It is the ONLY state a strategy may keep: no globals,
 *        no `static` variables.  That is what makes two copies of one
 *        strategy independent, and a match repeatable from its seed.
 *
 * Hardware-free C: the firmware, the host league and every student's file
 * include this header and nothing chip-specific.
 *
 * NAMING.  You write `strategy_step` and `strategy_name`.  The macro below
 * renames them to <PREFIX>_step and <PREFIX>_name, and PREFIX defaults to
 * `student`.  The host league compiles alice.c with -DSTRATEGY_PREFIX=alice,
 * so thirty students' files link into one program without a name clash and
 * without anyone editing their file.
 */
#ifndef STRATEGY_H
#define STRATEGY_H

#include <stdint.h>

/* ---- what the robot senses ---------------------------------------------- */
#define N_DIST        3          /* forward distance sensors                  */
#define DIST_LEFT     0          /* aimed 30 degrees left of the nose         */
#define DIST_CENTRE   1          /* straight ahead                            */
#define DIST_RIGHT    2          /* aimed 30 degrees right of the nose        */
#define DIST_NONE     0xFFFFu    /* nothing in its cone within range          */
#define DIST_RANGE_MM 600        /* farther than this reads DIST_NONE         */

enum {                           /* edge (line) sensors at the four corners   */
    EDGE_FL = 1u << 0,           /* front-left is over the white border       */
    EDGE_FR = 1u << 1,
    EDGE_BL = 1u << 2,
    EDGE_BR = 1u << 3
};
enum {                           /* bump (crash) switch: where the hit was    */
    BUMP_FRONT = 1u << 0,
    BUMP_BACK  = 1u << 1,
    BUMP_LEFT  = 1u << 2,
    BUMP_RIGHT = 1u << 3
};

typedef struct {
    int32_t  t_ms;               /* ms since the start signal; NEGATIVE during
                                    the 3 s countdown, when wheels are locked  */
    uint16_t dist_mm[N_DIST];    /* to the opponent's surface, or DIST_NONE;
                                    noisy, and sometimes misses               */
    uint8_t  edge;               /* EDGE_* bits                               */
    uint8_t  bump;               /* BUMP_* bits: contact since the last call  */
    int32_t  enc_l, enc_r;       /* wheel travel, mm, since the round began.
                                    A slipping wheel counts too: encoders
                                    measure the WHEEL, not the robot          */
    uint8_t  round;              /* 1, 2 or 3                                 */
    uint8_t  my_rounds;          /* rounds won so far in this match           */
    uint8_t  their_rounds;
} robot_view_t;

/* ---- what the robot does ------------------------------------------------- */
typedef struct {
    int8_t left, right;          /* -100..100; anything outside is clamped    */
} robot_cmd_t;

/* ---- the robot's private memory ----------------------------------------- */
#define STRATEGY_MEM_WORDS 16    /* 64 bytes: the class tournament's limit    */
typedef struct { uint32_t w[STRATEGY_MEM_WORDS]; } strategy_mem_t;

/* Use your own struct for the memory, checked at COMPILE time:
 *
 *     typedef struct { uint8_t mode; int32_t timer; } my_state_t;
 *     STRATEGY_STATE_FITS(my_state_t);
 *     ...
 *     my_state_t *s = STRATEGY_STATE(my_state_t, mem);
 */
#define STRATEGY_STATE_FITS(type) \
    _Static_assert(sizeof(type) <= sizeof(strategy_mem_t), \
                   #type " is bigger than the 64-byte strategy memory")
#define STRATEGY_STATE(type, mem) ((type *)(void *)(mem)->w)

typedef void (*strategy_fn)(const robot_view_t *in, robot_cmd_t *out,
                            strategy_mem_t *mem);

/* ---- the renaming trick --------------------------------------------------- */
#ifndef STRATEGY_PREFIX
#define STRATEGY_PREFIX student
#endif
#define STRATEGY_CAT2(a, b) a##b
#define STRATEGY_CAT(a, b)  STRATEGY_CAT2(a, b)
#define strategy_step STRATEGY_CAT(STRATEGY_PREFIX, _step)
#define strategy_name STRATEGY_CAT(STRATEGY_PREFIX, _name)

void strategy_step(const robot_view_t *in, robot_cmd_t *out, strategy_mem_t *mem);
extern const char strategy_name[];   /* up to 12 characters, shown on the OLED */

/* ---- the built-in opponents (bots.c) --------------------------------------- */
void bull_step   (const robot_view_t *in, robot_cmd_t *out, strategy_mem_t *mem);
void matador_step(const robot_view_t *in, robot_cmd_t *out, strategy_mem_t *mem);
void spinner_step(const robot_view_t *in, robot_cmd_t *out, strategy_mem_t *mem);
void turtle_step (const robot_view_t *in, robot_cmd_t *out, strategy_mem_t *mem);
void coward_step (const robot_view_t *in, robot_cmd_t *out, strategy_mem_t *mem);

#endif
