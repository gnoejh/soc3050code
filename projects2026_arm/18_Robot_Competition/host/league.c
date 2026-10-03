/*
 * league.c - the sumo league on a PC: the same world, referee and brains as
 * the firmware, compiled with plain gcc and run thousands of times faster.
 *
 *   run.bat                                  built-ins + student.c
 *   run.bat strategies\alice.c strategies\bob.c   ... plus any number more
 *
 * What it does, in order, and it exits non-zero if any of it fails:
 *
 *   1. PHYSICS CHECKS   top speed and acceleration against the motor model's
 *                       own equation; a side-on robot is pushed out while a
 *                       nose-on one that pushes back holds; sensors read
 *                       what they should.
 *   2. THE LEAGUE       every strategy against every other, N matches a pair,
 *                       each pair's seeds alternating who starts as robot 0.
 *   3. DETERMINISM      the whole league again: every match must come out
 *                       bit-for-bit the same (compared by hash and by table).
 *   4. THE TOURNAMENT   exactly the tournament the firmware runs - the student
 *                       against the five bots, TOUR_MATCHES each - and the
 *                       hash the board's $RESULT line should carry for that
 *                       seed (not yet confirmed against a running board).
 *   5. THE PICTURE      scene.c draws a moment of a match into oled.c's
 *                       framebuffer - the firmware's own drawing code - and
 *                       the ring and the opponent must be where they belong.
 *   (6, in run.bat/run.sh: the same program built at -O0, -O2 and -Os must
 *    print the same hashes.)
 *
 * Options:  -n N      matches per pair (default 100)
 *           -s SEED   base seed (default 2026)
 *           --tour NAME   whose tournament to run (default: student)
 *           --hash    print only the league and tournament hashes
 *           --draw    also print the OLED picture as text
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "referee.h"
#include "world.h"
#include "fmath.h"
#include "scene.h"
#include "oled.h"

/* oled.c's one hardware call, stubbed: the PC has no I2C bus.  The drawing
 * code above it is the firmware's, unchanged. */
int i2c_write_read(uint8_t addr, const uint8_t *w, size_t wn, uint8_t *r, size_t rn)
{
    (void)addr; (void)w; (void)wn; (void)r; (void)rn;
    return 0;
}

/* ---- the entries: roster[] (referee.c) plus every registered file -------- */
#define MAX_ENTRIES 48
static entry_t entries[MAX_ENTRIES];
static int     n_entries;
static entry_t extra[MAX_ENTRIES];
static int     n_extra;

void league_register(const char *name, strategy_fn fn)
{
    if (n_extra < MAX_ENTRIES - (1 + N_BOTS)) {
        extra[n_extra].name = name;
        extra[n_extra].fn = fn;
        extra[n_extra].style = "registered file";
        n_extra++;
    }
}

static int by_name(const void *a, const void *b)
{
    return strcmp(((const entry_t *)a)->name, ((const entry_t *)b)->name);
}

static int fails;
static void check(int ok, const char *what)
{
    printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) { fails++; }
}

/* ============================================================================
 *  1. Physics checks - the world, with no strategies at all
 * ============================================================================ */
static void park(world_t *w, int i, float x, float y, float th)
{
    world_place(w, i, x, y, th, 12345u + (uint32_t)i);
}

static void drive(world_t *w, int i, int l, int r)
{
    robot_cmd_t c = { (int8_t)l, (int8_t)r };
    world_drive(w, i, &c);
}

static void physics_checks(void)
{
    world_t w;
    printf("1. Physics checks\n");

    /* --- straight-line run: compare with the motor model's own solution.
     * Phase 1: the motors could push 6 N but the tyres hold 4.9 N: the wheels
     * slip and the robot accelerates at mu*g until the motor's force falls to
     * the grip, at v1.  Phase 2: v approaches V_MAX with tau = m Vmax / 2 F. */
    park(&w, 0, -0.30f, 0.10f, 0.0f);
    park(&w, 1, 0.0f, -0.30f, F_PI_2);                    /* out of the way */
    drive(&w, 0, 100, 100);
    float grip = MU_LONG * ROBOT_M * GRAVITY * 0.5f;
    float v1 = V_MAX * (1.0f - grip / F_STALL);
    float t1 = v1 / (MU_LONG * GRAVITY);
    float tau = ROBOT_M * V_MAX / (2.0f * F_STALL);
    /* time to half speed, from v(t) = Vmax (1 - (1 - v1/Vmax) e^-(t-t1)/tau) */
    float t_half_model = t1 + tau * 0.4917f;              /* ln(0.8175/0.5) */
    float t_half = -1.0f, t_out = -1.0f, vmax_seen = 0.0f;
    int slipped = 0;
    for (int k = 1; k <= 400; k++) {
        world_step(&w);
        float v = f_sqrt(w.b[0].vx * w.b[0].vx + w.b[0].vy * w.b[0].vy);
        if (w.b[0].slip) { slipped = 1; }
        if (v > vmax_seen) { vmax_seen = v; }
        if (t_half < 0.0f && v >= 0.5f * V_MAX) { t_half = k * DT; }
        if (t_out < 0.0f && world_radius(&w, 0) > RING_R) { t_out = k * DT; }
    }
    printf("     full throttle from rest: top speed %.3f m/s (model %.3f), "
           "half speed at %.1f ms (model %.1f ms), wheels slipped at launch: %s\n",
           vmax_seen, V_MAX, t_half * 1000.0f, t_half_model * 1000.0f, slipped ? "yes" : "no");
    printf("     from x = -0.30 m it crossed the edge after %.0f ms\n", t_out * 1000.0f);
    check(vmax_seen > 0.99f * V_MAX && vmax_seen <= V_MAX * 1.001f, "top speed is V_MAX");
    check(f_abs(t_half - t_half_model) < 0.006f, "acceleration matches the motor model to one step");
    check(slipped, "launch is traction-limited: wheels slip");

    /* --- the push test: same pusher, three targets, from the ring centre.
     * The target stands still, wheels braked (command 0), and is turned
     * nose-on, side-on or tail-on to the pusher. */
    const char *how[3] = { "nose-on, braking", "side-on, braking", "nose-on, pushing back 100%" };
    float   ang[3]     = { F_PI, F_PI_2, F_PI };
    int     back[3]    = { 0, 0, 100 };
    float   t_push[3];
    for (int c = 0; c < 3; c++) {
        park(&w, 0, -2.0f * ROBOT_R - 0.001f, 0.0f, 0.0f);   /* touching, facing +x */
        park(&w, 1, 0.0f, 0.0f, ang[c]);
        drive(&w, 0, 100, 100);
        drive(&w, 1, back[c], back[c]);
        t_push[c] = -1.0f;
        for (int k = 1; k <= 1200; k++) {                    /* 6 s */
            world_step(&w);
            if (world_radius(&w, 1) > RING_R) { t_push[c] = k * DT; break; }
        }
        if (t_push[c] > 0.0f) {
            printf("     push a target %-28s out from the centre: %5.0f ms\n", how[c], t_push[c] * 1000.0f);
        } else {
            printf("     push a target %-28s out from the centre: not in 6 s (moved %.0f mm)\n",
                   how[c], world_radius(&w, 1) * 1000.0f);
        }
    }
    check(t_push[1] > 0.0f && t_push[2] < 0.0f,
          "side-on, a robot is pushed out; nose-on and pushing back, it holds");

    /* --- sensors: a target 300 mm ahead, 1000 reads */
    park(&w, 0, -0.20f, 0.0f, 0.0f);
    park(&w, 1, -0.20f + ROBOT_R + 0.300f + ROBOT_R, 0.0f, 0.0f);
    robot_view_t v;
    long sum = 0; int n = 0, miss = 0, side = 0;
    for (int k = 0; k < 1000; k++) {
        world_sense(&w, 0, &v);
        if (v.dist_mm[DIST_CENTRE] == DIST_NONE) { miss++; } else { sum += v.dist_mm[DIST_CENTRE]; n++; }
        if (v.dist_mm[DIST_LEFT] != DIST_NONE || v.dist_mm[DIST_RIGHT] != DIST_NONE) { side++; }
    }
    printf("     target 300 mm dead ahead: centre reads %ld mm mean, misses %d/1000, "
           "side sensors fired %d/1000\n", n ? sum / n : -1L, miss, side);
    check(n && labs(sum / n - 300) <= 5 && miss > 10 && miss < 60 && side == 0,
          "centre sensor: right distance, ~3% misses, side cones blind straight ahead");

    park(&w, 1, -0.20f + 0.25f * f_cos(0.5236f), 0.25f * f_sin(0.5236f), 0.0f);
    world_sense(&w, 0, &v);
    printf("     target 30 deg left: L=%u C=%u R=%u (65535 = nothing)\n",
           v.dist_mm[0], v.dist_mm[1], v.dist_mm[2]);
    check(v.dist_mm[DIST_LEFT] != DIST_NONE && v.dist_mm[DIST_RIGHT] == DIST_NONE,
          "left sensor sees a target 30 deg left; right does not");

    /* --- edge sensors: nose on the border, then dead centre */
    park(&w, 0, RING_R - LINE_W * 0.5f - EDGE_POS, 0.0f, 0.0f);
    park(&w, 1, -0.25f, 0.0f, 0.0f);
    world_sense(&w, 0, &v);
    uint8_t e1 = v.edge;
    park(&w, 0, 0.0f, 0.0f, 0.0f);
    world_sense(&w, 0, &v);
    printf("     edge bits with the nose on the line: 0x%X; at the centre: 0x%X\n", e1, v.edge);
    check(e1 == (EDGE_FL | EDGE_FR) && v.edge == 0, "front corner sensors see the line, nothing at the centre");
}

/* ============================================================================
 *  2. The league
 * ============================================================================ */
typedef struct { int p, w, d, l, rw, rl, so, pts; } row_t;
static row_t rows[MAX_ENTRIES];
static int   pair_w[MAX_ENTRIES][MAX_ENTRIES];      /* wins of i over j       */
static int   pair_d[MAX_ENTRIES][MAX_ENTRIES];

static uint32_t league(int n, uint32_t base, double *secs, long *sim_steps)
{
    static match_t m;
    uint32_t hash = FNV_START;
    memset(rows, 0, sizeof rows);
    memset(pair_w, 0, sizeof pair_w);
    memset(pair_d, 0, sizeof pair_d);
    *sim_steps = 0;
    clock_t c0 = clock();
    for (int i = 0; i < n_entries; i++) {
        for (int j = i + 1; j < n_entries; j++) {
            for (int k = 0; k < n; k++) {
                uint32_t seed = fnv1a(fnv1a(fnv1a(fnv1a(FNV_START, base), (uint32_t)i), (uint32_t)j), (uint32_t)k);
                int a = (k & 1) ? j : i, b = (k & 1) ? i : j;      /* swap corners */
                int win = match_run(&m, &entries[a], &entries[b], seed);
                hash = fnv1a(hash, m.hash);
                *sim_steps += (long)(m.round - 1) * (COUNTDOWN_STEPS + FIGHT_STEPS) + (long)m.t;
                int who[2] = { a, b };
                for (int s = 0; s < 2; s++) {
                    row_t *r = &rows[who[s]];
                    r->p++;
                    r->rw += m.won[s];  r->rl += m.won[1 - s];  r->so += m.self_out[s];
                    if (win == s)       { r->w++; r->pts += 3; pair_w[who[s]][who[1 - s]]++; }
                    else if (win < 0)   { r->d++; r->pts += 1; pair_d[who[s]][who[1 - s]]++; }
                    else                { r->l++; }
                }
            }
        }
    }
    *secs = (double)(clock() - c0) / CLOCKS_PER_SEC;
    return hash;
}

static void print_league(int n, uint32_t hash, double secs, long steps)
{
    int order[MAX_ENTRIES];
    for (int i = 0; i < n_entries; i++) { order[i] = i; }
    for (int i = 0; i < n_entries; i++) {              /* points, then wins */
        for (int j = i + 1; j < n_entries; j++) {
            row_t *a = &rows[order[i]], *b = &rows[order[j]];
            if (b->pts > a->pts || (b->pts == a->pts && b->w > a->w)) {
                int t = order[i]; order[i] = order[j]; order[j] = t;
            }
        }
    }
    printf("\n2. The league: %d strategies, %d matches per pair, %d matches\n",
           n_entries, n, n * n_entries * (n_entries - 1) / 2);
    printf("   #  %-12s %5s %5s %5s %5s %6s %6s %6s %5s\n",
           "strategy", "P", "W", "D", "L", "rnd W", "rnd L", "s-out", "Pts");
    for (int q = 0; q < n_entries; q++) {
        row_t *r = &rows[order[q]];
        printf("  %2d  %-12s %5d %5d %5d %5d %6d %6d %6d %5d\n", q + 1, entries[order[q]].name,
               r->p, r->w, r->d, r->l, r->rw, r->rl, r->so, r->pts);
    }
    printf("   (win 3, draw 1; s-out = rounds lost by driving out with no one touching)\n");

    printf("\n   head to head, wins-draws-losses of the ROW against the column:\n   %-12s", "");
    for (int j = 0; j < n_entries; j++) { printf(" %9.9s", entries[order[j]].name); }
    printf("\n");
    for (int i = 0; i < n_entries; i++) {
        int a = order[i];
        printf("   %-12s", entries[a].name);
        for (int j = 0; j < n_entries; j++) {
            int b = order[j];
            if (a == b) { printf(" %9s", "-"); continue; }
            char cell[16];
            snprintf(cell, sizeof cell, "%d-%d-%d", pair_w[a][b], pair_d[a][b], pair_w[b][a]);
            printf(" %9s", cell);
        }
        printf("\n");
    }
    double sim_s = (double)steps * DT;
    printf("\n   simulated %.1f hours of sumo in %.2f s (%.0fx real time); league hash %08lX\n",
           sim_s / 3600.0, secs, secs > 0 ? sim_s / secs : 0.0, (unsigned long)hash);
}

/* ============================================================================
 *  4. The firmware's tournament, repeated
 * ============================================================================ */
static uint32_t tournament(const entry_t *me, uint32_t base, int quiet)
{
    static match_t m;
    uint32_t hash = FNV_START;
    int W = 0, D = 0, L = 0;
    if (!quiet) { printf("\n4. The tournament the firmware runs: %s, seed %lu\n", me->name, (unsigned long)base); }
    for (int opp = 1; opp <= N_BOTS; opp++) {
        int w = 0, d = 0, l = 0;
        for (int k = 0; k < TOUR_MATCHES; k++) {
            int side = tour_student_side(k);
            const entry_t *a = side ? &roster[opp] : me, *b = side ? me : &roster[opp];
            int win = match_run(&m, a, b, tour_seed(base, opp, k));
            hash = fnv1a(hash, m.hash);
            if (win == side) { w++; } else if (win < 0) { d++; } else { l++; }
        }
        if (!quiet) { printf("   vs %-8s  W %d  D %d  L %d\n", roster[opp].name, w, d, l); }
        W += w; D += d; L += l;
    }
    char body[96];
    int n = snprintf(body, sizeof body, "RESULT,%s,%d,%d,%d,%08lX,host,host",
                     me->name, W, D, L, (unsigned long)hash);
    uint8_t cs = 0;
    for (int i = 0; i < n; i++) { cs ^= (uint8_t)body[i]; }
    if (!quiet) { printf("   $%s*%02X\n", body, cs); }
    return hash;
}

/* ============================================================================
 *  The picture: scene.c into oled.c's framebuffer, read back with oled_get()
 * ============================================================================ */
static void picture(int show)
{
    static match_t m;
    match_begin(&m, &roster[0], &roster[1], tour_seed(2026, 1, 0));
    while (m.phase == PHASE_COUNTDOWN || match_time_ms(&m) < 600) { match_step(&m); }
    snap_t s;
    memset(&s, 0, sizeof s);
    s.screen = SCR_MATCH;
    for (int i = 0; i < 2; i++) {
        s.x[i] = m.w.b[i].x;  s.y[i] = m.w.b[i].y;  s.th[i] = m.w.b[i].th;
        memcpy(s.dist[i], m.view[i].dist_mm, sizeof s.dist[i]);
        s.name[i] = m.e[i]->name;
    }
    s.round = 1;  s.t_ms = match_time_ms(&m);  s.speed = 1;  s.cones = 1;
    strcpy(s.msg, "GO!");
    scene_draw(&s);

    int x1 = 32 + (int)(s.x[1] * (31.0f / RING_R) + (s.x[1] >= 0 ? 0.5f : -0.5f));
    int y1 = 32 - (int)(s.y[1] * (31.0f / RING_R) + (s.y[1] >= 0 ? 0.5f : -0.5f));
    printf("\n5. The OLED picture, 600 ms into %s vs %s (scene.c + oled.c, as on the chip)\n",
           s.name[0], s.name[1]);
    if (show) {
        for (int y = 0; y < OLED_H; y += 2) {          /* two rows per line */
            printf("   ");
            for (int x = 0; x < OLED_W; x++) {
                int a = oled_get(x, y), b = oled_get(x, y + 1);
                putchar(a && b ? '#' : a ? '"' : b ? ',' : ' ');
            }
            printf("\n");
        }
    }
    int lit = 0;
    for (int y = 0; y < OLED_H; y++) { for (int x = 0; x < OLED_W; x++) { lit += oled_get(x, y); } }
    printf("     %d pixels lit; robot 1 drawn filled at (%d, %d)\n", lit, x1, y1);
    check(oled_get(32, 1) && oled_get(32, 63) && oled_get(1, 32) && oled_get(63, 32),
          "the ring's edge is drawn at radius 31 around (32, 32)");
    int disc = 0, area = 0;                  /* the heading line is XOR-ed out */
    for (int dy = -3; dy <= 3; dy++) {
        for (int dx = -3; dx <= 3; dx++) {
            if (dx * dx + dy * dy <= 9) { area++; disc += oled_get(x1 + dx, y1 + dy); }
        }
    }
    printf("     %d of the %d pixels within 3 px of it are lit\n", disc, area);
    check(disc >= area - 4, "the opponent is a filled circle where the physics put it");
}

int main(int argc, char **argv)
{
    int n = 100, hash_only = 0, draw = 0;
    uint32_t base = 2026;
    const char *tour_name = "student";
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-n") && i + 1 < argc)          { n = atoi(argv[++i]); }
        else if (!strcmp(argv[i], "-s") && i + 1 < argc)     { base = (uint32_t)strtoul(argv[++i], 0, 0); }
        else if (!strcmp(argv[i], "--tour") && i + 1 < argc) { tour_name = argv[++i]; }
        else if (!strcmp(argv[i], "--hash"))                 { hash_only = 1; }
        else if (!strcmp(argv[i], "--draw"))                 { draw = 1; }
        else { fprintf(stderr, "usage: league [-n N] [-s SEED] [--tour NAME] [--hash] [--draw]\n"); return 2; }
    }

    for (int i = 0; i < 1 + N_BOTS; i++) { entries[n_entries++] = roster[i]; }
    qsort(extra, (size_t)n_extra, sizeof extra[0], by_name);   /* link order must not matter */
    for (int i = 0; i < n_extra; i++) { entries[n_entries++] = extra[i]; }

    double secs; long steps;
    if (hash_only) {
        uint32_t h = league(n, base, &secs, &steps);
        printf("%08lX %08lX\n", (unsigned long)h, (unsigned long)tournament(&entries[0], base, 1));
        return 0;
    }

    physics_checks();

    uint32_t h1 = league(n, base, &secs, &steps);
    print_league(n, h1, secs, steps);

    printf("\n3. Determinism: the whole league again\n");
    static row_t first[MAX_ENTRIES];
    memcpy(first, rows, sizeof rows);
    double secs2; long steps2;
    uint32_t h2 = league(n, base, &secs2, &steps2);
    printf("     first run %08lX, second run %08lX\n", (unsigned long)h1, (unsigned long)h2);
    check(h1 == h2 && !memcmp(first, rows, sizeof rows) && steps == steps2,
          "same seed, same league, bit for bit");
    uint32_t h3 = league(n, base + 1, &secs2, &steps2);
    printf("     seed %lu instead: %08lX\n", (unsigned long)(base + 1), (unsigned long)h3);
    check(h3 != h1, "a different seed gives a different league");

    const entry_t *me = 0;
    for (int i = 0; i < n_entries; i++) { if (!strcmp(entries[i].name, tour_name)) { me = &entries[i]; } }
    if (!me) { printf("no strategy called %s\n", tour_name); return 2; }
    uint32_t t1 = tournament(me, base, 0);
    check(t1 == tournament(me, base, 1), "the tournament repeats: same $RESULT hash");

    picture(draw);

    printf("\n%s: %d check%s failed\n", fails ? "FAILED" : "ALL CHECKS PASSED", fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}
