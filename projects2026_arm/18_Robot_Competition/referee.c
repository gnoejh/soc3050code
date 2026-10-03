/*
 * referee.c - match rules, start positions, the roster, the tournament seed
 *
 * See referee.h for the rules.  The one design choice worth reading twice:
 * both robots' views are built from the SAME instant before EITHER strategy
 * runs.  If robot 0 moved first and robot 1 then sensed the result, robot 1
 * would see 20 ms into the future - and whoever was "robot 1" would win more.
 */
#include <string.h>
#include "referee.h"
#include "fmath.h"

const entry_t roster[1 + N_BOTS] = {
    { student_name, student_step, "yours: student.c"},
    { "Bull",       bull_step,    "charges what it sees"},
    { "Matador",    matador_step, "waits, dodges, pushes"},
    { "Spinner",    spinner_step, "circles and scans"},
    { "Turtle",     turtle_step,  "faces you, stays home"},
    { "Coward",     coward_step,  "runs from everything"},
};

static float rnd(match_t *m, float lo, float hi)    /* uniform in [lo, hi) */
{
    float u = (float)(xorshift32(&m->rng) >> 8) * (1.0f / 16777216.0f);
    return lo + (hi - lo) * u;
}

/* Start positions.  Both robots sit on one random line through the centre,
 * 10-20 cm out, facing each other / sideways / away - then a little jitter.
 * Round 2 swaps round 1's places, so neither robot keeps a lucky corner. */
static void place_round(match_t *m)
{
    if (m->round != 2) {
        float phi = rnd(m, -F_PI, F_PI);
        float d0 = rnd(m, 0.10f, 0.20f), d1 = rnd(m, 0.10f, 0.20f);
        float h0 = 0.0f, h1 = 0.0f;               /* offsets from "facing" */
        switch (xorshift32(&m->rng) % 5u) {
        case 0:  break;                                           /* face to face */
        case 1:  h0 = (xorshift32(&m->rng) & 1u) ? F_PI_2 : -F_PI_2; break; /* 0 side-on */
        case 2:  h1 = (xorshift32(&m->rng) & 1u) ? F_PI_2 : -F_PI_2; break; /* 1 side-on */
        case 3:  h0 = F_PI; h1 = F_PI; break;                      /* back to back */
        default: h0 = rnd(m, -F_PI, F_PI); h1 = rnd(m, -F_PI, F_PI); break;
        }
        h0 += rnd(m, -0.26f, 0.26f);              /* +-15 degrees            */
        h1 += rnd(m, -0.26f, 0.26f);
        float c = w_cos(phi), s = w_sin(phi);
        m->start[0][0] = -d0 * c;  m->start[0][1] = -d0 * s;  m->start[0][2] = phi + h0;
        m->start[1][0] =  d1 * c;  m->start[1][1] =  d1 * s;  m->start[1][2] = phi + F_PI + h1;
    } else {
        for (int k = 0; k < 3; k++) {
            float t = m->start[0][k]; m->start[0][k] = m->start[1][k]; m->start[1][k] = t;
        }
    }
    for (int i = 0; i < 2; i++) {
        world_place(&m->w, i, m->start[i][0], m->start[i][1], m->start[i][2],
                    xorshift32(&m->rng) | 1u);
    }
}

static void begin_round(match_t *m)
{
    memset(m->mem, 0, sizeof m->mem);             /* fresh memory each round */
    memset(m->cmd, 0, sizeof m->cmd);
    m->t = 0;
    m->last_touch = 0;
    m->phase = PHASE_COUNTDOWN;
    m->round_winner = -1;
    place_round(m);
}

void match_begin(match_t *m, const entry_t *a, const entry_t *b, uint32_t seed)
{
    uint32_t (*clock)(void) = m->clock;           /* survive the memset      */
    uint8_t manual = m->manual;
    robot_cmd_t joy = m->joy;
    memset(m, 0, sizeof *m);
    m->clock = clock;  m->manual = manual;  m->joy = joy;
    m->e[0] = a;  m->e[1] = b;
    m->seed = seed;
    m->rng  = seed ? seed : 1u;
    m->round = 1;
    m->winner = -1;
    m->hash = fnv1a(FNV_START, seed);
    begin_round(m);
}

void match_next_round(match_t *m)
{
    if (m->phase != PHASE_ROUND_OVER) { return; }
    m->round++;
    begin_round(m);
}

int32_t match_time_ms(const match_t *m)
{
    return ((int32_t)m->t - (int32_t)COUNTDOWN_STEPS) * (int32_t)(DT * 1000.0f + 0.5f);
}

/* Every 20 ms: sense both, then think both, then latch both. */
static void control(match_t *m)
{
    for (int i = 0; i < 2; i++) {
        robot_view_t *v = &m->view[i];
        world_sense(&m->w, i, v);
        v->t_ms = match_time_ms(m);
        v->round = m->round;
        v->my_rounds = m->won[i];
        v->their_rounds = m->won[1 - i];
    }
    for (int i = 0; i < 2; i++) {
        if (m->manual & (1u << i)) {
            m->cmd[i] = m->joy;
        } else if (m->clock) {
            uint32_t t0 = m->clock();
            m->e[i]->fn(&m->view[i], &m->cmd[i], &m->mem[i]);
            uint32_t c = m->clock() - t0;
            if (c > m->cyc_max[i]) { m->cyc_max[i] = c; }
        } else {
            m->e[i]->fn(&m->view[i], &m->cmd[i], &m->mem[i]);
        }
    }
    static const robot_cmd_t locked = { 0, 0 };
    for (int i = 0; i < 2; i++) {
        world_drive(&m->w, i, m->phase == PHASE_FIGHT ? &m->cmd[i] : &locked);
    }
}

static uint32_t bits(float f) { uint32_t u; memcpy(&u, &f, 4); return u; }

uint8_t match_step(match_t *m)
{
    uint8_t ev = 0;
    if (m->phase == PHASE_ROUND_OVER || m->phase == PHASE_MATCH_OVER) { return 0; }

    if (m->t % STEPS_PER_CTRL == 0u) { control(m); }
    world_step(&m->w);
    m->t++;
    if (m->w.touching) { m->last_touch = m->t; }

    if (m->phase == PHASE_COUNTDOWN) {
        if (m->t >= COUNTDOWN_STEPS) { m->phase = PHASE_FIGHT; ev |= EV_GO; }
        return ev;
    }

    int out0 = world_radius(&m->w, 0) > RING_R;
    int out1 = world_radius(&m->w, 1) > RING_R;
    if (!out0 && !out1 && m->t < COUNTDOWN_STEPS + FIGHT_STEPS) { return ev; }

    /* The round is over. */
    m->round_winner = (out0 && !out1) ? 1 : (out1 && !out0) ? 0 : -1;
    if (m->round_winner >= 0) {
        m->won[m->round_winner]++;
        /* Out with no contact for half a second: it drove itself out. */
        if (m->t - m->last_touch > 100u) { m->self_out[1 - m->round_winner]++; }
    }
    m->hash = fnv1a(m->hash, (uint32_t)m->round_winner);
    m->hash = fnv1a(m->hash, m->t);
    for (int i = 0; i < 2; i++) {
        m->hash = fnv1a(m->hash, bits(m->w.b[i].x));
        m->hash = fnv1a(m->hash, bits(m->w.b[i].y));
    }
    ev |= EV_ROUND_OVER;

    if (m->won[0] >= ROUNDS_TO_WIN || m->won[1] >= ROUNDS_TO_WIN || m->round >= ROUNDS_MAX) {
        m->winner = m->won[0] > m->won[1] ? 0 : m->won[1] > m->won[0] ? 1 : -1;
        m->phase = PHASE_MATCH_OVER;
        ev |= EV_MATCH_OVER;
    } else {
        m->phase = PHASE_ROUND_OVER;
    }
    return ev;
}

int match_run(match_t *m, const entry_t *a, const entry_t *b, uint32_t seed)
{
    match_begin(m, a, b, seed);
    for (;;) {
        uint8_t ev = match_step(m);
        if (ev & EV_MATCH_OVER) { return m->winner; }
        if (ev & EV_ROUND_OVER) { match_next_round(m); }
    }
}

uint32_t tour_seed(uint32_t base, int opp, int k)
{
    uint32_t h = fnv1a(fnv1a(fnv1a(FNV_START, base), (uint32_t)opp), (uint32_t)k);
    return h ? h : 1u;
}
