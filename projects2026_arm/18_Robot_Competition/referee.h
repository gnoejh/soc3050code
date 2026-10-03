/*
 * referee.h - match rules, the roster, and the tournament
 *
 * The REFEREE sits between the world and the brains.  Each 5 ms step it
 * advances the world; every fourth step (20 ms) it builds each robot's view,
 * calls both strategies with the SAME instant's views, and latches their
 * commands.  It also keeps the rules:
 *
 *   - 3 s countdown after the start signal: strategies run, wheels locked
 *   - a robot whose CENTRE leaves the ring loses the round
 *   - 20 s of fighting with nobody out: the round is a draw
 *   - best of three rounds: first to 2 wins, at most 3 rounds, more round
 *     wins takes the match, equal is a drawn match
 *   - start positions come from the match SEED, so a match is repeatable
 *
 * Pure C, shared by the firmware and host/league.c.
 */
#ifndef REFEREE_H
#define REFEREE_H

#include <stdint.h>
#include "strategy.h"
#include "world.h"

#define STEPS_PER_CTRL   4u          /* 4 x 5 ms = strategies at 50 Hz        */
#define COUNTDOWN_STEPS  600u        /* 3 s                                   */
#define FIGHT_STEPS      4000u       /* 20 s                                  */
#define ROUNDS_MAX       3u
#define ROUNDS_TO_WIN    2u

typedef struct {
    const char *name;
    strategy_fn fn;
    const char *style;               /* one line for the menu and the table   */
} entry_t;

/* The built-in roster: the student first, then the five opponents. */
#define N_BOTS 5
extern const entry_t roster[1 + N_BOTS];
#define STUDENT (&roster[0])

enum { PHASE_COUNTDOWN, PHASE_FIGHT, PHASE_ROUND_OVER, PHASE_MATCH_OVER };

enum {                               /* match_step() events                   */
    EV_GO          = 1u << 0,        /* the countdown just ended              */
    EV_ROUND_OVER  = 1u << 1,
    EV_MATCH_OVER  = 1u << 2
};

typedef struct {
    world_t         w;
    const entry_t  *e[2];
    strategy_mem_t  mem[2];
    robot_view_t    view[2];         /* last views handed out (OLED cones)    */
    robot_cmd_t     cmd[2];
    uint32_t        seed;
    uint32_t        rng;             /* the referee's own: start positions    */
    uint32_t        t;               /* steps since this round began          */
    uint8_t         phase;
    uint8_t         round;           /* 1..3                                  */
    uint8_t         won[2];          /* round wins                            */
    int8_t          round_winner;    /* 0, 1, or -1 for a draw                */
    int8_t          winner;          /* the match: 0, 1, or -1 for a draw     */
    uint8_t         manual;          /* bit i: robot i driven by `joy`        */
    robot_cmd_t     joy;             /* the joystick's command (fun mode)     */
    uint32_t        hash;            /* FNV-1a of every round's outcome       */
    float           start[2][3];     /* this match's start x, y, heading      */
    uint32_t        last_touch;      /* step of the last contact              */
    uint8_t         self_out[2];     /* rounds lost with nobody touching: drove out */
    /* Cycle measurement - the firmware fills `clock`; the host leaves it 0. */
    uint32_t      (*clock)(void);
    uint32_t        cyc_max[2];      /* worst strategy_step, CPU cycles       */
} match_t;

void    match_begin(match_t *m, const entry_t *a, const entry_t *b, uint32_t seed);
uint8_t match_step(match_t *m);      /* one physics step; returns EV_* bits   */
void    match_next_round(match_t *m);  /* after EV_ROUND_OVER                 */
int32_t match_time_ms(const match_t *m);   /* negative in the countdown        */

/* Run a whole match with no display.  Returns the winner (0, 1, -1). */
int     match_run(match_t *m, const entry_t *a, const entry_t *b, uint32_t seed);

/* ---- the tournament the firmware runs and the host can repeat ------------- */
#define TOUR_MATCHES   4             /* per opponent                          */

/* The seed of match k against opponent `opp` (1..N_BOTS) - identical on the
 * chip and on the PC, so the host can re-run a submitted tournament. */
uint32_t tour_seed(uint32_t base, int opp, int k);

/* The student plays as robot 0 in even k and robot 1 in odd k: both corners. */
static inline int tour_student_side(int k) { return k & 1; }

/* FNV-1a, the hash every result line carries. */
static inline uint32_t fnv1a(uint32_t h, uint32_t v)
{
    for (int i = 0; i < 4; i++) { h = (h ^ (v & 0xFFu)) * 16777619u; v >>= 8; }
    return h;
}
#define FNV_START 2166136261u

#endif
