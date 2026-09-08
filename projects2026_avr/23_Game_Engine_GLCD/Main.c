/*
 * Main.c - 23_Game_Engine_GLCD
 * SOC3050 ATmega128 Educational Framework, 2026 AVR edition
 *
 * A bouncing ball and a paddle, which is the smallest program that needs
 * everything a game needs: a steady frame rate, input that feels immediate,
 * and a screen redrawn fast enough that motion looks like motion.
 *
 * The point of the lesson is not the ball.  It is the number in the bottom
 * left, which is how long the last screen update actually took.  Press
 * button A (PD0) to switch between pushing only the pages that changed and
 * pushing all eight, and watch that number move.  The ratio between the two
 * readings is what a framebuffer buys you.
 *
 * Controls
 *   PD4 / PD5   move the paddle left / right
 *   PD0         toggle dirty-page flushing on and off
 *   PD1         reset the miss counter
 *
 * The eight LEDs on PORT B show the paddle position as a coarse bar, so
 * something is visible even if the GLCD is not yet wired.
 */

#include "config.h"

/* ------------------------------------------------------------- profiling --
 * Timer3 free-runs at prescaler 8.  At 16 MHz that is 2 MHz, so one count is
 * half a microsecond and the 16-bit counter spans 32.7 ms - comfortably more
 * than one 20 ms frame, so a measurement can never silently wrap.
 */
static void profile_init(void)
{
    TCCR3A = 0;
    TCCR3B = (1 << CS31);
}

static inline void profile_start(void) { TCNT3 = 0; }

static inline uint16_t profile_us(void) { return (uint16_t)(TCNT3 >> 1); }

/* ------------------------------------------------------------ game state -- */

static int16_t ball_x, ball_y;
static int8_t ball_vx, ball_vy;
static uint8_t paddle_x;
static uint16_t misses;
static uint8_t flush_dirty_only = 1;

/* What the last frame cost, shown once every half second. */
static uint16_t last_bytes;
static uint16_t last_us;

static void ball_reset(void)
{
    ball_x = FIELD_X + FIELD_W / 2;
    ball_y = FIELD_Y + 4;

    /* Start in a random horizontal direction so consecutive rounds differ. */
    ball_vx = (game_rand() & 1) ? 2 : -2;
    ball_vy = 1;
}

static void game_init(void)
{
    paddle_x = (FIELD_W - PADDLE_W) / 2;
    misses = 0;
    ball_reset();
}

/* ---------------------------------------------------------------- update --
 * One fixed timestep.  Everything moves by a whole number of pixels per tick,
 * so the simulation is deterministic and needs no floating point.
 */
static void update(void)
{
    if (btn_held(BTN_LEFT) && paddle_x >= PADDLE_STEP)
        paddle_x = (uint8_t)(paddle_x - PADDLE_STEP);
    if (btn_held(BTN_RIGHT) && paddle_x + PADDLE_W + PADDLE_STEP <= FIELD_W)
        paddle_x = (uint8_t)(paddle_x + PADDLE_STEP);

    if (btn_pressed(BTN_A))
        flush_dirty_only = !flush_dirty_only;
    if (btn_pressed(BTN_B))
        misses = 0;

    ball_x = (int16_t)(ball_x + ball_vx);
    ball_y = (int16_t)(ball_y + ball_vy);

    /* Side walls. */
    if (ball_x <= FIELD_X + 1)
    {
        ball_x = FIELD_X + 1;
        ball_vx = (int8_t)-ball_vx;
    }
    if (ball_x >= FIELD_X + FIELD_W - 1 - BALL_SIZE)
    {
        ball_x = FIELD_X + FIELD_W - 1 - BALL_SIZE;
        ball_vx = (int8_t)-ball_vx;
    }

    /* Ceiling. */
    if (ball_y <= FIELD_Y + 1)
    {
        ball_y = FIELD_Y + 1;
        ball_vy = (int8_t)-ball_vy;
    }

    /* The paddle, tested only while the ball is descending so a ball that has
     * already passed the paddle cannot be caught from underneath. */
    if (ball_vy > 0 && ball_y + BALL_SIZE >= PADDLE_Y && ball_y < PADDLE_Y + PADDLE_H)
    {
        if (ball_x + BALL_SIZE >= paddle_x && ball_x <= paddle_x + PADDLE_W)
        {
            ball_y = PADDLE_Y - BALL_SIZE;
            ball_vy = (int8_t)-ball_vy;

            /* Where it struck steers the bounce, which is what makes the
             * paddle feel like a control rather than a wall. */
            int16_t hit = (int16_t)(ball_x + BALL_SIZE / 2 - (paddle_x + PADDLE_W / 2));
            if (hit < -4)
                ball_vx = -2;
            else if (hit > 4)
                ball_vx = 2;
        }
    }

    /* Missed: past the bottom of the field. */
    if (ball_y > FIELD_Y + FIELD_H)
    {
        misses++;
        ball_reset();
    }
}

/* ---------------------------------------------------------------- render -- */

static void render(uint8_t redraw_stats)
{
    /* Only the field is cleared each frame.  The status lines live in pages 0
     * and 7 and are rewritten twice a second, because redrawing them every
     * frame would dirty those pages every frame and throw away most of what
     * dirty-page flushing saves. */
    gfx_fill_rect(FIELD_X, FIELD_Y, FIELD_W, FIELD_H, GFX_OFF);
    gfx_rect(FIELD_X, FIELD_Y, FIELD_W, FIELD_H, GFX_ON);

    gfx_fill_rect((uint8_t)ball_x, (uint8_t)ball_y, BALL_SIZE, BALL_SIZE, GFX_ON);
    gfx_fill_rect(paddle_x, PADDLE_Y, PADDLE_W, PADDLE_H, GFX_ON);

    if (redraw_stats)
    {
        gfx_fill_rect(0, 0, 128, 8, GFX_OFF);
        gfx_text(0, 0, flush_dirty_only ? "DIRTY" : "FULL ", GFX_ON);
        gfx_text(40, 0, "MISS", GFX_ON);
        gfx_number(70, 0, misses, 3, GFX_ON);

        gfx_fill_rect(0, 56, 128, 8, GFX_OFF);
        gfx_text(0, 56, "US", GFX_ON);
        gfx_number(18, 56, last_us, 5, GFX_ON);
        gfx_text(60, 56, "BYTES", GFX_ON);
        gfx_number(96, 56, last_bytes, 4, GFX_ON);
    }
}

/* ------------------------------------------------------------------ main -- */

int main(void)
{
    LED_DDR = 0xFF;
    LED_WRITE(0x00);

    profile_init();
    gfx_init();
    game_timer_init(TICK_HZ); /* also sets up the buttons, and enables interrupts */
    game_srand(0xBEEF);
    game_init();

    for (;;)
    {
        /* Wait for the tick rather than delaying, so the frame rate is set by
         * the timer and does not drift with however long the frame took. */
        if (!game_tick_ready())
            continue;

        input_poll();
        update();

        uint8_t redraw_stats = ((game_ticks() % (TICK_HZ / 2)) == 0);
        render(redraw_stats);

        profile_start();
        last_bytes = flush_dirty_only ? gfx_flush() : gfx_flush_all();
        last_us = profile_us();

        /* Paddle position as a coarse bar on the LEDs: one lit LED per eighth
         * of the field, so the board shows something without the GLCD. */
        uint8_t lit = (uint8_t)(paddle_x / (FIELD_W / 8));
        LED_WRITE((uint8_t)(1 << lit));

        /* An overrun means the frame did not fit in its 20 ms slot.  Holding
         * all eight LEDs on is a loud, unmissable way to say so. */
        if (game_overrun())
            LED_WRITE(0xFF);
    }
}
