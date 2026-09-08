/*
 * Main.c - 24_Game_Arcade
 * SOC3050 ATmega128 Educational Framework, 2026 AVR edition
 *
 * Three games on the engine built in lesson 23: Pong, Snake and Breakout,
 * behind a menu, with sound.
 *
 * Lesson 23 was about making the screen fast enough. This one is about what
 * you do with that: a state machine that owns the whole program, collision
 * tested as integer comparisons, sprites kept in flash, and a beep that does
 * not stop the game while it plays.
 *
 * Controls
 *   MENU    PD6/PD7 choose        PD0 start
 *   PLAY    PD4/PD5 move          PD6/PD7 also steer the snake
 *           PD1 quit to the menu
 *   OVER    PD0 back to the menu
 *
 * The LEDs on PORT B show which game is selected, and all eight light if a
 * frame overruns its 20 ms budget.
 */

#include "config.h"
#include <avr/pgmspace.h>

/* ------------------------------------------------------------ program state */

enum
{
    G_PONG = 0,
    G_SNAKE,
    G_BREAKOUT,
    G_COUNT
};

enum
{
    ST_MENU = 0,
    ST_PLAY,
    ST_OVER
};

static uint8_t state;
static uint8_t game;
static uint8_t menu_sel;
static uint16_t score;
static uint8_t won; /* ST_OVER shows a different message for a win */

/*
 * Menu labels live in flash. Three short strings is only about forty bytes,
 * but the habit matters: on this machine every string literal you leave in
 * RAM is taken from the same 4096 bytes the framebuffer already halves.
 */
static const char label_pong[] PROGMEM = "PONG";
static const char label_snake[] PROGMEM = "SNAKE";
static const char label_brk[] PROGMEM = "BREAKOUT";
static const char *const menu_labels[G_COUNT] PROGMEM = {
    label_pong, label_snake, label_brk};

/* Copy one flash string into a small stack buffer for drawing. */
static void draw_text_P(uint8_t x, uint8_t y, const char *src, uint8_t mode)
{
    char buf[12];
    uint8_t i = 0;

    while (i < sizeof(buf) - 1)
    {
        char c = (char)pgm_read_byte(&src[i]);
        if (!c)
            break;
        buf[i++] = c;
    }
    buf[i] = 0;
    gfx_text(x, y, buf, mode);
}

/* ------------------------------------------------------------------- sounds */

static void beep_move(void) { sound_tone(900, 15); }
static void beep_bounce(void) { sound_tone(600, 20); }
static void beep_point(void) { sound_tone(1300, 45); }
static void beep_lose(void) { sound_tone(200, 250); }

/* ================================================================== PONG == */

static int16_t pg_bx, pg_by;
static int8_t pg_vx, pg_vy;
static uint8_t pg_player, pg_ai;
static uint8_t pg_ps, pg_as;

static void pong_serve(int8_t dir)
{
    pg_bx = 64;
    pg_by = (FIELD_TOP + FIELD_BOT) / 2;
    pg_vx = (game_rand() & 1) ? 2 : -2;
    pg_vy = (int8_t)(2 * dir);
}

static void pong_start(void)
{
    pg_player = (128 - PONG_PAD_W) / 2;
    pg_ai = pg_player;
    pg_ps = 0;
    pg_as = 0;
    pong_serve(1);
}

static void pong_update(void)
{
    if (btn_held(BTN_LEFT) && pg_player >= 3)
        pg_player = (uint8_t)(pg_player - 3);
    if (btn_held(BTN_RIGHT) && pg_player + PONG_PAD_W + 3 <= 128)
        pg_player = (uint8_t)(pg_player + 3);

    /* The AI tracks the ball but is speed-limited below the ball's own 2 px
     * per tick, so a sharp angle beats it. An AI that simply matched the ball
     * would be unbeatable and therefore pointless. */
    uint8_t ai_centre = (uint8_t)(pg_ai + PONG_PAD_W / 2);
    if (ai_centre < pg_bx && pg_ai + PONG_PAD_W + PONG_AI_SPEED <= 128)
        pg_ai = (uint8_t)(pg_ai + PONG_AI_SPEED);
    else if (ai_centre > pg_bx && pg_ai >= PONG_AI_SPEED)
        pg_ai = (uint8_t)(pg_ai - PONG_AI_SPEED);

    pg_bx = (int16_t)(pg_bx + pg_vx);
    pg_by = (int16_t)(pg_by + pg_vy);

    if (pg_bx <= 0)
    {
        pg_bx = 0;
        pg_vx = (int8_t)-pg_vx;
        beep_bounce();
    }
    if (pg_bx >= 128 - PONG_BALL)
    {
        pg_bx = 128 - PONG_BALL;
        pg_vx = (int8_t)-pg_vx;
        beep_bounce();
    }

    /* Player paddle, tested only while descending. */
    if (pg_vy > 0 && pg_by + PONG_BALL >= PONG_PLAYER_Y &&
        pg_by < PONG_PLAYER_Y + PONG_PAD_H &&
        pg_bx + PONG_BALL >= pg_player && pg_bx <= pg_player + PONG_PAD_W)
    {
        pg_by = PONG_PLAYER_Y - PONG_BALL;
        pg_vy = (int8_t)-pg_vy;
        beep_bounce();
    }

    /* AI paddle, tested only while rising. */
    if (pg_vy < 0 && pg_by <= PONG_AI_Y + PONG_PAD_H &&
        pg_by + PONG_BALL > PONG_AI_Y &&
        pg_bx + PONG_BALL >= pg_ai && pg_bx <= pg_ai + PONG_PAD_W)
    {
        pg_by = PONG_AI_Y + PONG_PAD_H;
        pg_vy = (int8_t)-pg_vy;
        beep_bounce();
    }

    if (pg_by > FIELD_BOT)
    {
        pg_as++;
        beep_lose();
        pong_serve(-1);
    }
    else if (pg_by < FIELD_TOP)
    {
        pg_ps++;
        beep_point();
        pong_serve(1);
    }

    if (pg_ps >= PONG_WIN || pg_as >= PONG_WIN)
    {
        won = (pg_ps >= PONG_WIN);
        score = pg_ps;
        state = ST_OVER;
    }
}

static void pong_render(void)
{
    gfx_fill_rect(pg_player, PONG_PLAYER_Y, PONG_PAD_W, PONG_PAD_H, GFX_ON);
    gfx_fill_rect(pg_ai, PONG_AI_Y, PONG_PAD_W, PONG_PAD_H, GFX_ON);
    gfx_fill_rect((uint8_t)pg_bx, (uint8_t)pg_by, PONG_BALL, PONG_BALL, GFX_ON);

    /* A dashed centre line, drawn straight into the framebuffer. */
    for (uint8_t x = 0; x < 128; x += 8)
        gfx_hline(x, (FIELD_TOP + FIELD_BOT) / 2, 4, GFX_ON);
}

/* ================================================================= SNAKE == */

static uint8_t sn_x[SNAKE_MAX], sn_y[SNAKE_MAX];
static uint8_t sn_len, sn_dir, sn_fx, sn_fy, sn_wait;

static void snake_food(void)
{
    /* Try a few random cells, then fall back to a linear scan. A pure retry
     * loop would spin forever once the snake fills most of the board. */
    for (uint8_t attempt = 0; attempt < 40; attempt++)
    {
        uint16_t r = game_rand();
        uint8_t fx = (uint8_t)(r % GRID_W);
        uint8_t fy = (uint8_t)((r >> 5) % GRID_H);
        uint8_t clash = 0;

        for (uint8_t i = 0; i < sn_len; i++)
            if (sn_x[i] == fx && sn_y[i] == fy)
            {
                clash = 1;
                break;
            }
        if (!clash)
        {
            sn_fx = fx;
            sn_fy = fy;
            return;
        }
    }

    for (uint8_t fy = 0; fy < GRID_H; fy++)
        for (uint8_t fx = 0; fx < GRID_W; fx++)
        {
            uint8_t clash = 0;
            for (uint8_t i = 0; i < sn_len; i++)
                if (sn_x[i] == fx && sn_y[i] == fy)
                {
                    clash = 1;
                    break;
                }
            if (!clash)
            {
                sn_fx = fx;
                sn_fy = fy;
                return;
            }
        }
}

static void snake_start(void)
{
    sn_len = 3;
    for (uint8_t i = 0; i < sn_len; i++)
    {
        sn_x[i] = (uint8_t)(GRID_W / 2 - i);
        sn_y[i] = GRID_H / 2;
    }
    sn_dir = BTN_RIGHT;
    sn_wait = 0;
    score = 0;
    snake_food();
}

static void snake_update(void)
{
    /* Steering is read every frame even though the snake only moves every
     * SNAKE_TICKS, so a quick tap is never dropped. Reversing straight into
     * your own neck is rejected. */
    if (btn_pressed(BTN_LEFT) && sn_dir != BTN_RIGHT)
        sn_dir = BTN_LEFT;
    if (btn_pressed(BTN_RIGHT) && sn_dir != BTN_LEFT)
        sn_dir = BTN_RIGHT;
    if (btn_pressed(BTN_UP) && sn_dir != BTN_DOWN)
        sn_dir = BTN_UP;
    if (btn_pressed(BTN_DOWN) && sn_dir != BTN_UP)
        sn_dir = BTN_DOWN;

    if (++sn_wait < SNAKE_TICKS)
        return;
    sn_wait = 0;

    int8_t nx = (int8_t)sn_x[0];
    int8_t ny = (int8_t)sn_y[0];

    if (sn_dir == BTN_LEFT)
        nx--;
    else if (sn_dir == BTN_RIGHT)
        nx++;
    else if (sn_dir == BTN_UP)
        ny--;
    else
        ny++;

    if (nx < 0 || nx >= GRID_W || ny < 0 || ny >= GRID_H)
    {
        beep_lose();
        won = 0;
        state = ST_OVER;
        return;
    }

    /* The tail cell is about to move away, so it cannot be hit. */
    for (uint8_t i = 0; i + 1 < sn_len; i++)
        if (sn_x[i] == (uint8_t)nx && sn_y[i] == (uint8_t)ny)
        {
            beep_lose();
            won = 0;
            state = ST_OVER;
            return;
        }

    uint8_t grew = (nx == (int8_t)sn_fx && ny == (int8_t)sn_fy);
    if (grew && sn_len < SNAKE_MAX)
        sn_len++;

    /* Shuffle the body along by one. Cheap at this length, and it keeps the
     * head at index 0 where every test above expects it. */
    for (uint8_t i = (uint8_t)(sn_len - 1); i > 0; i--)
    {
        sn_x[i] = sn_x[i - 1];
        sn_y[i] = sn_y[i - 1];
    }
    sn_x[0] = (uint8_t)nx;
    sn_y[0] = (uint8_t)ny;

    if (grew)
    {
        score++;
        beep_point();
        snake_food();
        if (sn_len >= SNAKE_MAX)
        {
            won = 1;
            state = ST_OVER;
        }
    }
}

static void snake_render(void)
{
    for (uint8_t i = 0; i < sn_len; i++)
        gfx_fill_rect((uint8_t)(sn_x[i] * CELL),
                      (uint8_t)(FIELD_TOP + sn_y[i] * CELL),
                      CELL - 1, CELL - 1, GFX_ON);

    /* The food is hollow so it reads differently from the body. */
    gfx_rect((uint8_t)(sn_fx * CELL), (uint8_t)(FIELD_TOP + sn_fy * CELL),
             CELL, CELL, GFX_ON);
}

/* ============================================================== BREAKOUT == */

static uint8_t brk_rows[BRK_ROWS]; /* one bit per column; 1 = brick present */
static int16_t bk_x, bk_y;
static int8_t bk_vx, bk_vy;
static uint8_t bk_pad, bk_lives;

static void breakout_serve(void)
{
    bk_x = 64;
    bk_y = 40;
    bk_vx = (game_rand() & 1) ? 2 : -2;
    bk_vy = -2;
}

static void breakout_start(void)
{
    for (uint8_t r = 0; r < BRK_ROWS; r++)
        brk_rows[r] = 0xFF; /* eight columns, all present */
    bk_pad = (128 - BRK_PAD_W) / 2;
    bk_lives = BRK_LIVES;
    score = 0;
    breakout_serve();
}

static void breakout_update(void)
{
    if (btn_held(BTN_LEFT) && bk_pad >= 3)
        bk_pad = (uint8_t)(bk_pad - 3);
    if (btn_held(BTN_RIGHT) && bk_pad + BRK_PAD_W + 3 <= 128)
        bk_pad = (uint8_t)(bk_pad + 3);

    bk_x = (int16_t)(bk_x + bk_vx);
    bk_y = (int16_t)(bk_y + bk_vy);

    if (bk_x <= 0)
    {
        bk_x = 0;
        bk_vx = (int8_t)-bk_vx;
    }
    if (bk_x >= 128 - 3)
    {
        bk_x = 128 - 3;
        bk_vx = (int8_t)-bk_vx;
    }
    if (bk_y <= FIELD_TOP)
    {
        bk_y = FIELD_TOP;
        bk_vy = (int8_t)-bk_vy;
    }

    /* Bricks. The grid is regular, so the hit cell is arithmetic rather than
     * a search: no loop over 24 bricks per frame. */
    if (bk_y >= BRK_TOP && bk_y < BRK_TOP + BRK_ROWS * BRK_H)
    {
        uint8_t row = (uint8_t)((bk_y - BRK_TOP) / BRK_H);
        uint8_t col = (uint8_t)(bk_x / BRK_W);

        if (row < BRK_ROWS && col < BRK_COLS && (brk_rows[row] & (1 << col)))
        {
            brk_rows[row] &= (uint8_t) ~(1 << col);
            bk_vy = (int8_t)-bk_vy;
            score++;
            sound_tone((uint16_t)(900 + row * 300), 25);

            if (!brk_rows[0] && !brk_rows[1] && !brk_rows[2])
            {
                won = 1;
                state = ST_OVER;
                return;
            }
        }
    }

    if (bk_vy > 0 && bk_y + 3 >= BRK_PAD_Y && bk_y < BRK_PAD_Y + 2 &&
        bk_x + 3 >= bk_pad && bk_x <= bk_pad + BRK_PAD_W)
    {
        bk_y = BRK_PAD_Y - 3;
        bk_vy = (int8_t)-bk_vy;

        /* Where it lands on the paddle steers the bounce. */
        int16_t hit = (int16_t)(bk_x + 1 - (bk_pad + BRK_PAD_W / 2));
        bk_vx = (hit < -6) ? -2 : (hit > 6) ? 2 : bk_vx;
        beep_bounce();
    }

    if (bk_y > FIELD_BOT)
    {
        beep_lose();
        if (--bk_lives == 0)
        {
            won = 0;
            state = ST_OVER;
        }
        else
        {
            breakout_serve();
        }
    }
}

static void breakout_render(void)
{
    for (uint8_t r = 0; r < BRK_ROWS; r++)
        for (uint8_t c = 0; c < BRK_COLS; c++)
            if (brk_rows[r] & (1 << c))
                gfx_fill_rect((uint8_t)(c * BRK_W), (uint8_t)(BRK_TOP + r * BRK_H),
                              BRK_W - 1, BRK_H - 1, GFX_ON);

    gfx_fill_rect(bk_pad, BRK_PAD_Y, BRK_PAD_W, 2, GFX_ON);
    gfx_fill_rect((uint8_t)bk_x, (uint8_t)bk_y, 3, 3, GFX_ON);
}

/* ============================================================ the machine == */

static void start_game(void)
{
    game = menu_sel;
    won = 0;
    score = 0;

    if (game == G_PONG)
        pong_start();
    else if (game == G_SNAKE)
        snake_start();
    else
        breakout_start();

    state = ST_PLAY;
}

static void menu_update(void)
{
    if (btn_pressed(BTN_UP))
    {
        menu_sel = (uint8_t)((menu_sel + G_COUNT - 1) % G_COUNT);
        beep_move();
    }
    if (btn_pressed(BTN_DOWN))
    {
        menu_sel = (uint8_t)((menu_sel + 1) % G_COUNT);
        beep_move();
    }
    if (btn_pressed(BTN_A))
        start_game();
}

static void menu_render(void)
{
    gfx_text(28, 0, "ARCADE", GFX_ON);

    for (uint8_t i = 0; i < G_COUNT; i++)
    {
        const char *label = (const char *)pgm_read_word(&menu_labels[i]);
        uint8_t y = (uint8_t)(20 + i * 12);

        if (i == menu_sel)
            gfx_text(24, y, ">", GFX_ON);
        draw_text_P(36, y, label, GFX_ON);
    }
}

static void hud_render(void)
{
    if (game == G_PONG)
    {
        gfx_text(0, 0, "YOU", GFX_ON);
        gfx_number(24, 0, pg_ps, 1, GFX_ON);
        gfx_text(72, 0, "CPU", GFX_ON);
        gfx_number(96, 0, pg_as, 1, GFX_ON);
    }
    else
    {
        gfx_text(0, 0, "SCORE", GFX_ON);
        gfx_number(36, 0, score, 3, GFX_ON);
        if (game == G_BREAKOUT)
        {
            gfx_text(78, 0, "LIFE", GFX_ON);
            gfx_number(110, 0, bk_lives, 1, GFX_ON);
        }
    }
}

static void over_render(void)
{
    gfx_text(22, 20, won ? "YOU WIN" : "GAME OVER", GFX_ON);
    gfx_text(22, 34, "SCORE", GFX_ON);
    gfx_number(64, 34, score, 3, GFX_ON);
    gfx_text(10, 50, "PRESS A FOR MENU", GFX_ON);
}

int main(void)
{
    LED_DDR = 0xFF;
    LED_WRITE(0x00);

    gfx_init();
    sound_init();
    game_timer_init(TICK_HZ); /* sets up the buttons and enables interrupts */

    /* Seeding from the free-running timer means the first food and the first
     * serve differ between runs, because how long the board took to reach this
     * line is never quite the same. */
    game_srand((uint16_t)(TCNT1 | 1));

    state = ST_MENU;
    menu_sel = G_PONG;

    for (;;)
    {
        if (!game_tick_ready())
            continue;

        input_poll();

        if (state == ST_MENU)
        {
            menu_update();
        }
        else if (state == ST_PLAY)
        {
            if (btn_pressed(BTN_B))
                state = ST_MENU;
            else if (game == G_PONG)
                pong_update();
            else if (game == G_SNAKE)
                snake_update();
            else
                breakout_update();
        }
        else /* ST_OVER */
        {
            if (btn_pressed(BTN_A))
                state = ST_MENU;
        }

        /* Redraw. Every screen is rebuilt from scratch each frame; the engine
         * works out which pages actually changed. */
        gfx_clear();

        if (state == ST_MENU)
        {
            menu_render();
        }
        else if (state == ST_PLAY)
        {
            hud_render();
            if (game == G_PONG)
                pong_render();
            else if (game == G_SNAKE)
                snake_render();
            else
                breakout_render();
        }
        else
        {
            over_render();
        }

        gfx_flush();

        /* The LEDs mirror the selected game, and shout if a frame overran. */
        LED_WRITE((uint8_t)(1 << (state == ST_MENU ? menu_sel : game)));
        if (game_overrun())
            LED_WRITE(0xFF);
    }
}
