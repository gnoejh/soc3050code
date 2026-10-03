/*
 * astar.c - A*, written for a chip with 12 KB of RAM and no FPU
 *
 * The algorithm in four lines:
 *
 *   open = {start};  g(start) = 0
 *   repeat: take the open cell with the lowest f = g + h      (h: a guess of
 *           the cost still to go that is never too high)
 *           if it is the goal, follow the parents back: done
 *           otherwise close it, and offer each neighbour a cheaper g
 *
 * The engineering is all in the data structures, because they decide the RAM:
 *
 *   g[]      uint16_t per cell   the best cost found so far         896 B
 *   info[]   uint8_t  per cell   closed flag + parent direction     448 B
 *   hpos[]   uint8_t  per cell   where the cell sits in heap[]      448 B
 *   heap[]   uint16_t x 160      the open list, a binary heap       320 B
 *                                                                  ------
 *                                                                  2112 B
 *
 * Three choices keep it that small:
 *
 *  - A parent is stored as the DIRECTION it came from (3 bits), not as a
 *    16-bit cell index: half the memory.
 *  - DECREASE-KEY.  When a cell already on the open list is offered a
 *    cheaper g, its entry is moved up the heap in place (hpos[] says where
 *    it is) instead of being pushed a second time.  The first version of
 *    this file used "lazy deletion" - push again, skip the stale copy later
 *    - which needs no hpos[].  The host test caught it: in an open room the
 *    duplicates overflowed a 255-entry heap after 173 of 335 cells, and the
 *    planner said "no path" to a reachable goal.  Measure before you save.
 *  - The heap holds 160 entries, so a byte can index it, and the host test
 *    measures the real peak on every level.  If it ever fills, astar_plan()
 *    says so (result.overflow) - it never silently drops a cell.
 *
 * All integer.  There is not one float in this file.
 */
#include "astar.h"

#define F_CLOSED  0x80u
#define DIR_MASK  0x07u
#define NOT_OPEN  0xFFu

static uint16_t g[GRID_N];
static uint8_t  info[GRID_N];
static uint8_t  hpos[GRID_N];
static uint16_t heap[ASTAR_HEAP];
static uint16_t hn;
static int      gx, gy;           /* the goal, for h()                          */

/* E, N, W, S, then the diagonals NE, NW, SW, SE.  planner.py: same order. */
static const int8_t DX[8] = { 1, 0, -1, 0, 1, -1, -1, 1 };
static const int8_t DY[8] = { 0, 1, 0, -1, 1, 1, -1, -1 };

uint32_t astar_ram_bytes(void) { return sizeof g + sizeof info + sizeof hpos + sizeof heap; }

/* Octile distance: diagonal steps for the shorter axis, straight for the rest.
 * 14 * min + 10 * (max - min)  =  10 * max + 4 * min. */
static uint16_t h(uint16_t i)
{
    int dx = CELL_X(i) - gx, dy = CELL_Y(i) - gy;
    if (dx < 0) { dx = -dx; }
    if (dy < 0) { dy = -dy; }
    return (uint16_t)(dx > dy ? 10 * dx + 4 * dy : 10 * dy + 4 * dx);
}

/* A cell's sort key, packed into 32 bits:
 *
 *   bits 31..18  f = g + h     lowest first
 *   bits 17..9   h             then the cell nearer the goal
 *   bits  8..0   cell index    then the lower index
 *
 * so ordering two cells is one unsigned compare, and the tie-break that lets
 * planner.py reproduce the PATH (not just its cost) comes for free.  f must
 * stay under 2^14: checked where costs are made. */
static uint32_t key(uint16_t cell)
{
    uint32_t hc = h(cell);
    return (((uint32_t)g[cell] + hc) << 18) | (hc << 9) | cell;
}

static void place(uint16_t at, uint16_t cell) { heap[at] = cell; hpos[cell] = (uint8_t)at; }

static void sift_up(uint16_t at)
{
    uint16_t cell = heap[at];
    uint32_t k = key(cell);
    while (at > 0) {
        uint16_t parent = (uint16_t)((at - 1u) / 2u);
        if (key(heap[parent]) <= k) { break; }
        place(at, heap[parent]);
        at = parent;
    }
    place(at, cell);
}

static uint16_t pop(void)
{
    uint16_t top = heap[0], cell = heap[--hn], at = 0;
    uint32_t k = key(cell);
    for (;;) {
        uint16_t c = (uint16_t)(2u * at + 1u);
        if (c >= hn) { break; }
        if (c + 1u < hn && key(heap[c + 1u]) < key(heap[c])) { c++; }
        if (key(heap[c]) >= k) { break; }
        place(at, heap[c]);
        at = c;
    }
    if (hn > 0) { place(at, cell); }
    hpos[top] = NOT_OPEN;
    return top;
}

static int lethal(const uint8_t *cm, int x, int y)
{
    if (x < 0 || y < 0 || x >= GRID_W || y >= GRID_H) { return 1; }
    return cm[CELL(x, y)] == ASTAR_LETHAL;
}

int astar_plan(const uint8_t *cm, uint16_t start, uint16_t goal,
               uint16_t *path, uint16_t path_max, astar_result_t *res)
{
    res->cost = ASTAR_NONE;  res->len = 0;  res->expanded = 0;  res->open_peak = 0;
    res->overflow = 0;
    if (start >= GRID_N || goal >= GRID_N || cm[goal] == ASTAR_LETHAL) { return 0; }

    for (uint16_t i = 0; i < GRID_N; i++) { g[i] = 0xFFFFu; info[i] = 0; hpos[i] = NOT_OPEN; }
    gx = CELL_X(goal);  gy = CELL_Y(goal);

    g[start] = 0;
    hn = 1;
    place(0, start);
    res->open_peak = 1;

    while (hn > 0) {
        uint16_t cur = pop();                       /* the best open cell         */
        info[cur] |= F_CLOSED;
        res->expanded++;

        if (cur == goal) {                          /* follow the parents back    */
            uint16_t n = 1;
            for (uint16_t c = cur; c != start; n++) {
                uint8_t d = info[c] & DIR_MASK;
                c = CELL(CELL_X(c) - DX[d], CELL_Y(c) - DY[d]);
            }
            res->cost = g[goal];
            res->len  = n < path_max ? n : path_max;
            uint16_t c = cur;
            for (uint16_t k = n; k-- > 0;) {        /* write it start-first       */
                if (k < path_max) { path[k] = c; }
                uint8_t d = info[c] & DIR_MASK;
                if (k > 0) { c = CELL(CELL_X(c) - DX[d], CELL_Y(c) - DY[d]); }
            }
            return 1;
        }

        int cx = CELL_X(cur), cy = CELL_Y(cur);
        for (uint8_t d = 0; d < 8u; d++) {
            int nx = cx + DX[d], ny = cy + DY[d];
            if (lethal(cm, nx, ny)) { continue; }
            if (d >= 4u && (lethal(cm, nx, cy) || lethal(cm, cx, ny))) { continue; }  /* no corner cutting */
            uint16_t nb = CELL(nx, ny);
            if (info[nb] & F_CLOSED) { continue; }
            uint32_t ng = (uint32_t)g[cur] + (d < 4u ? ASTAR_STRAIGHT : ASTAR_DIAG) + cm[nb];
            if (ng >= g[nb]) { continue; }          /* not an improvement         */
            if (ng + h(nb) >= (1u << 14)) { res->overflow = 1; return 0; }
            g[nb] = (uint16_t)ng;
            info[nb] = d;                           /* parent = the way we came   */
            if (hpos[nb] != NOT_OPEN) {
                sift_up(hpos[nb]);                  /* already open: decrease-key */
            } else {
                if (hn >= ASTAR_HEAP) { res->overflow = 1; return 0; }   /* say so, never guess */
                place(hn, nb);
                sift_up(hn++);
                if (hn > res->open_peak) { res->open_peak = hn; }
            }
        }
    }
    return 0;                                       /* open list empty: no path    */
}
