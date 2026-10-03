/*
 * host/sitl.c - lesson 16's robot, run on a PC against lesson 16's world
 *
 * The SAME world.c, nav.c, astar.c, levels.c and view.c that the board
 * compiles, linked with a PC compiler and driven by a plain loop instead of
 * RTOS tasks:
 *
 *     every 20 ms:  world_step(command)
 *     every 40 ms:  world_sense(&s); control_step(&s, &command)
 *
 * which is exactly the schedule Main.c's tasks keep.  Nothing here is a
 * model of the firmware; it IS the firmware's navigation code.
 *
 * What it checks, and fails (exit 1) on:
 *   1. every level parses and has a path, with full knowledge of the map
 *   2. "no path" is reported correctly - and A* explored exactly the
 *      reachable cells before saying so
 *   3. every level, 5 random seeds: the robot reaches the goal; time, path
 *      length against the optimum, replans, collisions, A* nodes and time
 *   4. the labs' switches (reactive off, no inflation, heading from wheels,
 *      no gyro calibration) - measured, not judged
 *   5. a goal sealed in a box: the robot must give up, not wander for ever
 *   6. paths.txt for planner.py, and OLED frames as ASCII art for the slides
 *
 * Build and run: host\run.bat or host/run.sh
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "sim.h"
#include "levels.h"
#include "world.h"
#include "nav.h"
#include "astar.h"
#include "view.h"
#include "oled.h"

#ifdef _WIN32
#include <windows.h>
uint32_t nav_clock(void)                      /* the planner's stopwatch: ns */
{
    static LARGE_INTEGER f;
    LARGE_INTEGER c;
    if (!f.QuadPart) { QueryPerformanceFrequency(&f); }
    QueryPerformanceCounter(&c);
    return (uint32_t)(c.QuadPart * 1000000000.0 / (double)f.QuadPart);
}
#else
#include <time.h>
uint32_t nav_clock(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint32_t)(t.tv_sec * 1000000000ull + (uint64_t)t.tv_nsec);
}
#endif

/* oled.c's only hardware call: on a PC the "panel" is the framebuffer. */
int i2c_write_read(uint8_t addr, const uint8_t *w, size_t wn, uint8_t *r, size_t rn)
{
    (void)addr; (void)w; (void)wn; (void)r; (void)rn;
    return 0;
}

static int failures;
static unsigned total_overflows, worst_open;     /* over every run of the robot */
#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ---- the true map, as a costmap class function ---------------------------- */
static int truth_cls(int x, int y) { return world_occ(x, y) ? MAP_OCC : MAP_FREE; }

static uint16_t optimal_cost(int inflate, astar_result_t *r, uint16_t *path)
{
    static uint8_t cm[GRID_N];
    costmap_build(truth_cls, (uint8_t)inflate, cm);
    const world_t *w = world();
    astar_plan(cm, CELL(w->start_x, w->start_y), CELL(w->goal_x, w->goal_y), path, GRID_N, r);
    return r->cost;
}

/* ---- ASCII art of the OLED framebuffer ------------------------------------ */
static void dump_frame(FILE *f, int half)
{
    if (!half) {
        for (int y = 0; y < OLED_H; y++) {
            for (int x = 0; x < OLED_W; x++) { fputc(oled_get(x, y) ? '#' : '.', f); }
            fputc('\n', f);
        }
        return;
    }
    static const char shade[] = " .:+#";       /* 2 x 2 pixels per character */
    for (int y = 0; y < OLED_H; y += 2) {
        for (int x = 0; x < OLED_W; x += 2) {
            int n = oled_get(x, y) + oled_get(x + 1, y) + oled_get(x, y + 1) + oled_get(x + 1, y + 1);
            fputc(shade[n], f);
        }
        fputc('\n', f);
    }
}

/* ---- one run ---------------------------------------------------------------- */
typedef struct {
    int      arrived, gave_up;
    uint32_t t_ms;                 /* run time: start (after calibration) to arrival */
    float    driven_mm, optimal_mm;
    unsigned coll, plans, replans, brakes, backups;
    unsigned peak_exp, peak_open, overflows;
    uint32_t peak_ns;
    float    est_err_mm;           /* |true - estimated| position at the end   */
    float    est_err_max;
} run_t;

static run_t run(int level, const nav_params_t *prm, uint32_t seed, uint32_t limit_ms,
                 const char *frame_file, uint32_t frame_at_ms)
{
    run_t R;
    memset(&R, 0, sizeof R);
    world_set_seed(seed);
    world_load(level);
    const world_t *w = world();
    nav_reset(w->start_x, w->start_y);
    *nav_params() = *prm;
    nav_set_goal(w->goal_x, w->goal_y);
    nav_start();

    sensors_t s;
    actuators_t a = { 0, 0 };
    uint32_t start_ms = 0;
    for (uint32_t t = 0; t < limit_ms; t += WORLD_MS) {
        world_step(&a);
        if (w->t_ms % CONTROL_MS == 0u) {
            world_sense(&s);
            control_step(&s, &a);
        }
        if (nav_mode() == NAV_RUN && start_ms == 0u) { start_ms = w->t_ms; }
        float ex, ey, eth;
        nav_pose(&ex, &ey, &eth);
        float e2 = (ex - w->x) * (ex - w->x) + (ey - w->y) * (ey - w->y);
        if (e2 > R.est_err_max * R.est_err_max) { R.est_err_max = sqrtf(e2); }
        if (frame_file && w->t_ms == frame_at_ms) {
            FILE *f = fopen(frame_file, "w");
            if (f) {
                view_draw(0, w->t_ms - start_ms);
                dump_frame(f, 0);
                fclose(f);
            }
        }
        if (nav_mode() == NAV_ARRIVED || nav_mode() == NAV_NOPATH) { break; }
    }
    const nav_stats_t *st = nav_stats();
    R.arrived  = nav_mode() == NAV_ARRIVED;
    R.gave_up  = nav_mode() == NAV_NOPATH;
    R.t_ms     = R.arrived ? st->run_ms : w->t_ms - start_ms;
    R.driven_mm = w->odometer_mm;
    R.coll     = w->collisions;
    R.plans    = st->plans;
    R.replans  = st->replans;
    R.brakes   = st->brakes;
    R.backups  = st->backups;
    R.peak_exp = st->peak_expanded;
    R.peak_open = st->peak_open;
    R.peak_ns  = st->peak_plan_clk;
    R.overflows = st->overflows;
    total_overflows += st->overflows;
    if (st->peak_open > worst_open) { worst_open = st->peak_open; }
    float ex, ey, eth;
    nav_pose(&ex, &ey, &eth);
    R.est_err_mm = sqrtf((ex - w->x) * (ex - w->x) + (ey - w->y) * (ey - w->y));
    astar_result_t r;
    static uint16_t p[GRID_N];
    R.optimal_mm = (float)optimal_cost(0, &r, p) * (CELL_MM / (float)ASTAR_STRAIGHT);
    return R;
}

/* ======================================================================= tests == */
static const nav_params_t DEFAULTS = { 250, 1, 1, HEAD_GYRO, 1 };
#define SEEDS     5
#define LIMIT_MS  180000u

static void test_levels(FILE *paths)
{
    printf("\n[1] Levels, planned with FULL knowledge of the map (the optimum)\n");
    printf("    level        cost  cells  expanded  open-peak   optimal path\n");
    for (int lv = 0; lv < N_LEVELS; lv++) {
        world_load(lv);
        for (int inf = 0; inf <= 2; inf++) {
            astar_result_t r;
            static uint16_t p[GRID_N];
            optimal_cost(inf, &r, p);
            CHECK(r.cost != ASTAR_NONE, "level %d has no path", lv + 1);
            if (inf == 0) {
                printf("    %d %-9s  %5u  %5u  %8u  %9u   %.0f mm\n", lv + 1, level_table[lv].name,
                       r.cost, r.len, r.expanded, r.open_peak, r.cost * 10.0);
            }
            /* for planner.py: the same map, the same inflation -> the same cells */
            fprintf(paths, "PATH %d %d %u %u", lv, inf, r.cost, r.expanded);
            for (uint16_t k = 0; k < r.len; k++) { fprintf(paths, " %d,%d", CELL_X(p[k]), CELL_Y(p[k])); }
            fprintf(paths, "\n");
        }
    }
}

/* Count the cells reachable from `start` under A*'s own moves (8-way, no
 * corner cutting, lethal cells excluded): A* must expand exactly these
 * before it may say "no path". */
#define LETH(X, Y) ((X) < 0 || (Y) < 0 || (X) >= GRID_W || (Y) >= GRID_H || cm[CELL(X, Y)] == ASTAR_LETHAL)
static int reachable(const uint8_t *cm, int start)
{
    static uint8_t seen[GRID_N];
    static uint16_t q[GRID_N];
    static const int dx[8] = { 1, 0, -1, 0, 1, -1, -1, 1 }, dy[8] = { 0, 1, 0, -1, 1, 1, -1, -1 };
    memset(seen, 0, sizeof seen);
    int head = 0, tail = 0;
    q[tail++] = (uint16_t)start;
    seen[start] = 1;
    while (head < tail) {
        int c = q[head++], x = CELL_X(c), y = CELL_Y(c);
        for (int d = 0; d < 8; d++) {
            int nx = x + dx[d], ny = y + dy[d];
            if (LETH(nx, ny)) { continue; }
            if (d >= 4 && (LETH(nx, y) || LETH(x, ny))) { continue; }
            int n = CELL(nx, ny);
            if (!seen[n]) { seen[n] = 1;  q[tail++] = (uint16_t)n; }
        }
    }
    return tail;
}

static void sealed_goal_map(void)
{
    /* an empty room with the goal inside a closed 5 x 5 box of wall */
    for (int y = 0; y < GRID_H; y++) {
        uint32_t row = (y == 0 || y == GRID_H - 1) ? 0xFFFFFFFFu : 0x80000001u;
        if (y >= 4 && y <= 8) {
            for (int x = 22; x <= 26; x++) {
                if (y == 4 || y == 8 || x == 22 || x == 26) { row |= 1u << (31 - x); }
            }
        }
        world_custom_row(y, row);
    }
    world_custom_start(2, 6);
    world_custom_goal(24, 6);
}

static void test_no_path(void)
{
    printf("\n[2] \"No path\" must be said correctly\n");
    sealed_goal_map();
    world_load(LEVEL_CUSTOM);
    static uint8_t cm[GRID_N];
    static uint16_t p[GRID_N];
    astar_result_t r;
    costmap_build(truth_cls, 0, cm);
    int found = astar_plan(cm, CELL(2, 6), CELL(24, 6), p, GRID_N, &r);
    int reach = reachable(cm, CELL(2, 6));
    printf("    goal sealed in a box : %s, expanded %u nodes (open list peak %u); flood fill says %d reachable\n",
           found ? "PATH (wrong)" : "no path", r.expanded, r.open_peak, reach);
    CHECK(!r.overflow, "sealed goal: the open list overflowed");
    CHECK(!found && r.cost == ASTAR_NONE, "sealed goal: A* found a path");
    CHECK(r.expanded == (unsigned)reach, "sealed goal: expanded %u != reachable %d", r.expanded, reach);

    found = astar_plan(cm, CELL(2, 6), CELL(22, 6), p, GRID_N, &r);   /* the goal IS a wall */
    printf("    goal inside a wall   : %s, expanded %u nodes\n", found ? "PATH (wrong)" : "no path", r.expanded);
    CHECK(!found && r.expanded == 0u, "goal in a wall: found=%d expanded=%u", found, r.expanded);

    found = astar_plan(cm, CELL(2, 6), CELL(2, 6), p, GRID_N, &r);     /* already there */
    printf("    start == goal        : cost %u, %u cell(s)\n", r.cost, r.len);
    CHECK(found && r.cost == 0u && r.len == 1u, "start==goal: cost %u len %u", r.cost, r.len);

    /* a diagonal wall whose cells touch only at their corners: the robot
     * cannot squeeze through, so neither may the planner */
    memset(cm, 0, sizeof cm);
    for (int y = 0; y < GRID_H; y++) {
        for (int x = 0; x < GRID_W; x++) {
            if (x + y == 16) { cm[CELL(x, y)] = ASTAR_LETHAL; }
        }
    }
    found = astar_plan(cm, CELL(1, 1), CELL(30, 12), p, GRID_N, &r);
    printf("    diagonal wall, corners touching : %s (expanded %u)\n", found ? "PATH (wrong)" : "no path", r.expanded);
    CHECK(!found, "diagonal wall: A* cut a corner");

    /* and the robot itself, on the sealed map, must stop and say so */
    run_t R = run(LEVEL_CUSTOM, &DEFAULTS, 7, LIMIT_MS, 0, 0);
    printf("    robot, sealed goal   : %s after %.1f s, %u plans, %u collisions\n",
           R.gave_up ? "NOPATH" : (R.arrived ? "ARRIVED (impossible!)" : "still searching at the limit"),
           R.t_ms / 1000.0, R.plans, R.coll);
    CHECK(R.gave_up, "sealed goal: the robot did not report NOPATH");
}

typedef struct {
    int arrived, runs;
    double t, ratio, err, errmax;
    unsigned coll, replans, peak_exp, peak_open, brakes, backups;
    uint32_t peak_ns;
} agg_t;

static agg_t sweep(const nav_params_t *prm, int lv, int verbose)
{
    agg_t A;
    memset(&A, 0, sizeof A);
    for (uint32_t seed = 1; seed <= SEEDS; seed++) {
        run_t R = run(lv, prm, seed * 7919u, LIMIT_MS, 0, 0);
        A.runs++;
        A.arrived += R.arrived;
        A.t += R.t_ms / 1000.0;
        A.ratio += R.driven_mm / R.optimal_mm;
        A.err += R.est_err_mm;
        if (R.est_err_max > A.errmax) { A.errmax = R.est_err_max; }
        A.coll += R.coll;
        A.replans += R.replans;
        A.brakes += R.brakes;
        A.backups += R.backups;
        if (R.peak_exp > A.peak_exp) { A.peak_exp = R.peak_exp; }
        if (R.peak_open > A.peak_open) { A.peak_open = R.peak_open; }
        if (R.peak_ns > A.peak_ns) { A.peak_ns = R.peak_ns; }
        if (verbose) {
            printf("      seed %u: %s t=%.1f s, driven %.0f / optimal %.0f mm, coll %u, replans %u, err %.0f mm\n",
                   (unsigned)seed, R.arrived ? "ok" : (R.gave_up ? "NOPATH" : "TIMEOUT"), R.t_ms / 1000.0,
                   R.driven_mm, R.optimal_mm, R.coll, R.replans, R.est_err_mm);
        }
    }
    return A;
}

static void test_runs(int verbose)
{
    printf("\n[3] Every level, %d seeds each, defaults: 250 mm/s, inflate 1, reactive on, gyro heading\n", SEEDS);
    printf("    level       arrived  time s  path/opt  replans  coll/run  est.err mm end/max  A* nodes/open  A* us(host)\n");
    unsigned total_coll = 0;
    for (int lv = 0; lv < N_LEVELS; lv++) {
        agg_t A = sweep(&DEFAULTS, lv, verbose);
        printf("    %d %-9s   %d/%d   %6.1f   %5.2f    %5.1f    %5.1f     %4.0f / %4.0f     %4u / %3u     %6.1f\n",
               lv + 1, level_table[lv].name, A.arrived, A.runs, A.t / A.runs, A.ratio / A.runs,
               (double)A.replans / A.runs, (double)A.coll / A.runs, A.err / A.runs, A.errmax,
               A.peak_exp, A.peak_open, A.peak_ns / 1000.0);
        CHECK(A.arrived == A.runs, "level %d: %d of %d runs did not arrive", lv + 1, A.runs - A.arrived, A.runs);
        total_coll += A.coll;
    }
    printf("    collisions over all %d runs: %u\n", N_LEVELS * SEEDS, total_coll);
    CHECK(total_coll <= (unsigned)(N_LEVELS * SEEDS) / 5u, "default settings collide too often: %u", total_coll);
}

static void ablate(const char *name, nav_params_t p)
{
    agg_t T;
    memset(&T, 0, sizeof T);
    for (int lv = 0; lv < N_LEVELS; lv++) {
        agg_t A = sweep(&p, lv, 0);
        T.arrived += A.arrived;  T.runs += A.runs;  T.t += A.t;  T.ratio += A.ratio;
        T.coll += A.coll;  T.replans += A.replans;  T.err += A.err;
        if (A.errmax > T.errmax) { T.errmax = A.errmax; }
    }
    printf("    %-28s %2d/%d   %6.1f   %5.2f    %5.1f     %5.2f       %5.0f / %5.0f\n", name, T.arrived, T.runs,
           T.t / T.runs, T.ratio / T.runs, (double)T.replans / T.runs, (double)T.coll / T.runs,
           T.err / T.runs, T.errmax);
}

static void test_ablations(void)
{
    printf("\n[4] The lab's switches, all 5 levels x %d seeds (measured, not judged)\n", SEEDS);
    printf("    setting                      arrived  time s  path/opt  replans  coll/run   est.err mm end/max\n");
    nav_params_t p = DEFAULTS;
    ablate("defaults", p);
    p = DEFAULTS; p.reactive = 0;                  ablate("reactive layer OFF", p);
    p = DEFAULTS; p.inflate = 0;                   ablate("inflation 0", p);
    p = DEFAULTS; p.inflate = 2;                   ablate("inflation 2", p);
    p = DEFAULTS; p.inflate = 0; p.reactive = 0;   ablate("inflation 0 + reactive OFF", p);
    p = DEFAULTS; p.heading = HEAD_WHEELS;         ablate("heading from WHEELS", p);
    p = DEFAULTS; p.gyro_cal = 0;                  ablate("gyro NOT calibrated", p);
    p = DEFAULTS; p.speed_mm_s = 400;              ablate("speed 400 mm/s", p);
    p = DEFAULTS; p.speed_mm_s = 150;              ablate("speed 150 mm/s", p);
}

/* How big can the open list get?  Random arenas, 10-35% wall, random start
 * and goal: the largest open list A* needed must fit ASTAR_HEAP with room. */
static void test_open_list(void)
{
    static uint8_t cm[GRID_N];
    static uint16_t p[GRID_N];
    uint32_t r = 12345u;
    unsigned peak = 0, paths = 0, nopaths = 0, over = 0, expmax = 0;
    for (int trial = 0; trial < 2000; trial++) {
        int density = 10 + trial % 26;
        for (int i = 0; i < GRID_N; i++) {
            r ^= r << 13; r ^= r >> 17; r ^= r << 5;
            cm[i] = (r % 100u) < (uint32_t)density ? ASTAR_LETHAL : (uint8_t)(r % 3u == 0u ? COST_INFLATE1 : 0u);
        }
        r ^= r << 13; r ^= r >> 17; r ^= r << 5;
        uint16_t a = (uint16_t)(r % GRID_N), b = (uint16_t)((r >> 12) % GRID_N);
        cm[a] = cm[b] = 0;
        astar_result_t res;
        int ok = astar_plan(cm, a, b, p, GRID_N, &res);
        paths += ok;  nopaths += !ok && !res.overflow;  over += res.overflow;
        if (res.open_peak > peak) { peak = res.open_peak; }
        if (res.expanded > expmax) { expmax = res.expanded; }
    }
    /* and the emptiest arena there is, corner to corner */
    for (int i = 0; i < GRID_N; i++) { cm[i] = 0; }
    astar_result_t res;
    astar_plan(cm, CELL(0, 0), CELL(GRID_W - 1, GRID_H - 1), p, GRID_N, &res);
    printf("\n[7] Open-list size: 2000 random arenas (10-35%% wall) plus an empty one\n");
    printf("    %u found a path, %u had none, %u overflowed; peak open list %u, peak expanded %u\n",
           paths, nopaths, over, peak, expmax);
    printf("    empty arena corner to corner: expanded %u, open peak %u\n", res.expanded, res.open_peak);
    printf("    every robot run above: open peak %u, overflows %u; the heap has %u slots\n",
           worst_open, total_overflows, (unsigned)ASTAR_HEAP);
    CHECK(over == 0u && total_overflows == 0u, "the open list overflowed");
    CHECK(peak < ASTAR_HEAP && worst_open < ASTAR_HEAP, "open list peak too close to ASTAR_HEAP");
}

static void print_half(const char *file)
{
    FILE *f = fopen(file, "r");
    if (!f) { CHECK(0, "no frame written to %s", file); return; }
    static uint8_t px[OLED_H][OLED_W];
    char line[256];
    int y = 0;
    while (y < OLED_H && fgets(line, sizeof line, f)) {
        for (int x = 0; x < OLED_W; x++) { px[y][x] = line[x] == '#'; }
        y++;
    }
    fclose(f);
    static const char shade[] = " .:+#";
    for (int yy = 0; yy < OLED_H; yy += 2) {
        printf("    |");
        for (int x = 0; x < OLED_W; x += 2) {
            putchar(shade[px[yy][x] + px[yy][x + 1] + px[yy + 1][x] + px[yy + 1][x + 1]]);
        }
        printf("|\n");
    }
}

static void test_frames(void)
{
    printf("\n[6] OLED frames, rendered by view.c into oled.c's framebuffer (host/frame_*.txt, full size)\n");
    run(2, &DEFAULTS, 7919u, LIMIT_MS, "frame_trap.txt", 9000);
    run(4, &DEFAULTS, 7919u, LIMIT_MS, "frame_door.txt", 12000);
    run(3, &DEFAULTS, 7919u, LIMIT_MS, "frame_maze.txt", 20000);
    printf("    level 3 (Trap) at t = 9 s, 2 x 2 pixels per character:\n");
    print_half("frame_trap.txt");
}

int main(int argc, char **argv)
{
    int verbose = argc > 1 && strcmp(argv[1], "-v") == 0;
    printf("SOC3050 lesson 16 - Robot Navigation, software in the loop on the host\n");
    printf("grid %d x %d cells of %d mm; robot radius %d mm; %d rangefinders to %d mm\n",
           GRID_W, GRID_H, CELL_MM, ROBOT_R_MM, N_RANGE, RANGE_MAX_MM);

    FILE *paths = fopen("paths.txt", "w");
    if (!paths) { printf("cannot write paths.txt\n"); return 1; }
    test_levels(paths);
    fclose(paths);
    test_no_path();
    test_runs(verbose);
    test_ablations();

    printf("\n[5] RAM the robot's software holds (static arrays, sizeof)\n");
    printf("    occupancy map %u B + costmap %u B + path %u B + A* workspace %u B = %u B\n",
           (unsigned)GRID_N, (unsigned)GRID_N, (unsigned)(NAV_PATH_MAX * 2), (unsigned)astar_ram_bytes(),
           (unsigned)nav_ram_bytes());
    printf("    sensors_t %u B, actuators_t %u B\n", (unsigned)sizeof(sensors_t), (unsigned)sizeof(actuators_t));
    CHECK(nav_ram_bytes() < 5000u, "navigation RAM %u B is over its 5 KB share", (unsigned)nav_ram_bytes());

    test_frames();
    test_open_list();

    printf("\n%s: %d failure(s)\n", failures ? "FAILED" : "PASSED", failures);
    return failures ? 1 : 0;
}
