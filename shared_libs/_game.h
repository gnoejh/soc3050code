/*
 * _game.h - Minimal 2D game engine for the KS0108 GLCD
 * SOC3050 ATmega128 Educational Framework, 2026 AVR edition
 *
 * Used by 23_Game_Engine_GLCD and 24_Game_Arcade.
 *
 * WHY THIS EXISTS
 * ---------------
 * _glcd.c draws straight to the display controller.  That is correct, but it
 * costs 14 us per byte (setup 1 + enable 1 + hold 2 + an unconditional 10 us
 * command delay), so repainting all 1024 bytes of a 128x64 screen takes about
 * 14 ms.  At 50 frames per second the whole frame budget is 20 ms, so the
 * library path spends ~72% of it pushing pixels and a game cannot keep up.
 *
 * This engine trades RAM for time: it keeps the screen in a framebuffer, so
 * drawing is plain memory writes, and only the pages that actually changed are
 * pushed to the panel.  The flush writes the bus directly rather than calling
 * ks0108_data(), because the 10 us command delay is only required after a
 * command, not after every data byte.
 *
 * THE COST, STATED PLAINLY
 * ------------------------
 * The framebuffer is 8 pages x 128 columns = 1024 bytes.  The ATmega128 has
 * 4096 bytes of SRAM.  One quarter of the machine's memory is this array.
 * That is the central lesson of 23_Game_Engine_GLCD: on a small MCU you buy
 * speed with memory, and you must know what you spent.
 *
 * NOTE ON LINKING: this module defines the TIMER1_COMPA ISR (__vector_12), the
 * same vector _timer.c defines.  A lesson using _game must NOT also link
 * _timer, or the link fails with a duplicate symbol.
 */

#ifndef _GAME_H_
#define _GAME_H_

#include <avr/io.h>
#include <stdint.h>

/* ------------------------------------------------------------------ timing */

/**
 * Settling time in microseconds after every byte pushed to the panel.
 *
 * shared_libs/_glcd.c waits 10 us after EVERY byte - command and data alike -
 * and that library demonstrably drives this board. The KS0108 datasheet says
 * the wait is only required after a command, so an earlier version of this
 * engine dropped it for data and ran the bus at 4 us per byte instead of 14.
 * The result was a blank screen: correct by the datasheet, wrong on the board.
 *
 * So the default matches the implementation that is known to work. Lowering it
 * is the exercise, not the starting point - the demo reports its own flush time
 * in microseconds, so you can lower this, watch the number fall, and find where
 * the display starts to corrupt. That number is a property of this board and
 * this simulator, and the only way to learn it is to measure it.
 */
#ifndef PANEL_SETTLE_US
#define PANEL_SETTLE_US 10
#endif

/* ------------------------------------------------------------------ screen */

#define GFX_W 128     /* pixels across                                       */
#define GFX_H 64      /* pixels down                                         */
#define GFX_PAGES 8   /* a page is 8 vertically-stacked pixels in one byte    */

/* Pixel modes, matching the KS0108 library's names so the two agree. */
#define GFX_OFF 0
#define GFX_ON 1
#define GFX_XOR 2

/**
 * @brief Initialise the panel and the framebuffer.  Call once, before use.
 */
void gfx_init(void);

/**
 * @brief Clear the framebuffer to blank.  Does not touch the panel until the
 *        next gfx_flush().
 */
void gfx_clear(void);

/**
 * @brief Set, clear or invert one pixel in the framebuffer.
 *
 * Out-of-range coordinates are ignored rather than wrapping, so a sprite may
 * be drawn partly off-screen without corrupting the opposite edge.
 */
void gfx_pixel(uint8_t x, uint8_t y, uint8_t mode);

void gfx_hline(uint8_t x, uint8_t y, uint8_t len, uint8_t mode);
void gfx_vline(uint8_t x, uint8_t y, uint8_t len, uint8_t mode);
void gfx_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t mode);
void gfx_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t mode);
void gfx_circle(uint8_t cx, uint8_t cy, uint8_t r, uint8_t mode);

/**
 * @brief Draw an 8-row sprite held in flash.
 *
 * @param bitmap  PROGMEM array of w bytes; bit 0 of each byte is the top row.
 *                Sprites live in flash because 1 KB of SRAM is already spent
 *                on the framebuffer.
 */
void gfx_sprite_P(uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t w, uint8_t mode);

/**
 * @brief Draw one character in the built-in 5x7 font.  Supports ASCII 32-90
 *        (space through 'Z'); lowercase is folded to uppercase.
 */
void gfx_char(uint8_t x, uint8_t y, char c, uint8_t mode);
void gfx_text(uint8_t x, uint8_t y, const char *s, uint8_t mode);

/**
 * @brief Draw an unsigned number, right-aligned into @p width digits.
 *
 * Games want scores without dragging in printf, which costs over a kilobyte of
 * flash and a formatting buffer this engine has no room for.
 */
void gfx_number(uint8_t x, uint8_t y, uint16_t value, uint8_t width, uint8_t mode);

/**
 * @brief Push only the pages that changed since the last flush.
 * @return number of bytes actually written to the panel - the figure
 *         23_Game_Engine_GLCD asks students to watch.
 */
uint16_t gfx_flush(void);

/**
 * @brief Push all eight pages regardless of what changed.  Provided so the
 *        lesson can measure the difference against gfx_flush().
 */
uint16_t gfx_flush_all(void);

/* ------------------------------------------------------------------- input */

/* The shared board wires six buttons to PORTD, each pulled up, so a pressed
 * button reads LOW.  PD2 and PD3 are absent from this list on purpose: they
 * are RXD1/TXD1, the serial port. */
#define BTN_A 0x01     /* PD0 */
#define BTN_B 0x02     /* PD1 */
#define BTN_LEFT 0x10  /* PD4 */
#define BTN_RIGHT 0x20 /* PD5 */
#define BTN_UP 0x40    /* PD6 */
#define BTN_DOWN 0x80  /* PD7 */

/**
 * @brief Sample the buttons and update held/pressed state.  Call once per
 *        tick; the debounce is a per-tick agreement filter, so the tick rate
 *        sets the debounce window.
 */
void input_poll(void);

/** @brief Non-zero while any button in @p mask is down. */
uint8_t btn_held(uint8_t mask);

/**
 * @brief Non-zero on the tick a button in @p mask went down.
 *
 * Reading it does not clear it; the edge lasts exactly one tick, which is what
 * menus and single-shot actions want.
 */
uint8_t btn_pressed(uint8_t mask);

/* ------------------------------------------------------------------ timing */

/**
 * @brief Start a fixed-rate tick on Timer1 in CTC mode and enable interrupts.
 *
 * @param hz  ticks per second, 10-200.  50 is a good default: fast enough to
 *            feel responsive, slow enough that a flush fits in the budget.
 */
void game_timer_init(uint8_t hz);

/**
 * @brief Consume one pending tick.
 * @return 1 if a tick was waiting, 0 otherwise.
 *
 * The main loop spins on this.  It deliberately does NOT accumulate: if the
 * frame overran, the missed tick is dropped rather than queued, so a slow
 * frame slows the game instead of making it lurch forward.
 */
uint8_t game_tick_ready(void);

/** @brief Ticks since game_timer_init(), wrapping at 65535. */
uint16_t game_ticks(void);

/**
 * @brief Non-zero if a tick elapsed while the previous frame was still being
 *        drawn - i.e. the frame budget was blown.  Cleared by reading.
 */
uint8_t game_overrun(void);

/* ------------------------------------------------------------------- sound */

/**
 * @brief Prepare the speaker output on PG4.
 *
 * The shared board wires PG4 through a slide switch to an AudioOut component,
 * so the switch is a physical mute: if a tone plays and nothing is heard,
 * check the switch before debugging the code.
 */
void sound_init(void);

/**
 * @brief Start a square-wave tone and return immediately.
 *
 * @param hz  tone frequency, clamped to 123-4000 Hz
 * @param ms  duration in milliseconds
 *
 * Timer2 toggles the pin in its own interrupt, so this does NOT block the way
 * _buzzer.c's Sound() does.  A game cannot afford to stop for the length of a
 * beep - at 50 frames per second a 100 ms blocking tone would cost five whole
 * frames.  Calling it again replaces whatever is playing.
 */
void sound_tone(uint16_t hz, uint16_t ms);

/** @brief Non-zero while a tone is still sounding. */
uint8_t sound_busy(void);

/** @brief Stop any tone immediately. */
void sound_stop(void);

/* ------------------------------------------------------------------ random */

/**
 * @brief 16-bit xorshift pseudo-random number.
 *
 * A linear-feedback shift register, not rand(): four shifts and three XORs,
 * no division, no library call, and it never returns 0.
 */
uint16_t game_rand(void);

/** @brief Reseed the generator.  A seed of 0 is replaced by 1. */
void game_srand(uint16_t seed);

#endif /* _GAME_H_ */
