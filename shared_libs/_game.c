/*
 * _game.c - Minimal 2D game engine for the KS0108 GLCD
 * SOC3050 ATmega128 Educational Framework, 2026 AVR edition
 *
 * See _game.h for why this exists.  Drawing through _glcd.c costs 14 us per
 * byte, so a full 128x64 repaint is ~14 ms out of a 20 ms frame at 50 Hz.
 * This module keeps the screen in RAM and pushes only the pages that changed,
 * which is where the win comes from: a moving ball touches one or two pages
 * out of eight, so a typical frame sends 128-256 bytes instead of 1024.
 *
 * It does NOT win by shortening the bus transaction.  An earlier version did,
 * dropping the 10 us settling delay for data bytes on the datasheet's word
 * that it is only needed after commands.  That produced a blank screen on this
 * board.  The delay is back at PANEL_SETTLE_US, matching the library that
 * demonstrably works, and lowering it is now an exercise with a measurement
 * attached rather than an assumption baked into the engine.
 *
 * It deliberately does NOT link _glcd.c.  That library's font is declared
 * "static const uint8_t ks0108_font[95][5]" with no PROGMEM, so it lives in
 * SRAM: building 18_GLCD_Graphics reports .data = 476 bytes, essentially all
 * of it font.  On a 4096-byte machine that is 11.6% of memory spent before
 * the game owns a single object, so this engine carries its own font in
 * flash and initialises the panel itself.
 */

#include "_game.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>

/* ------------------------------------------------------- panel hardware --
 * Traced from projects2026_avr/Simulator.simu:
 *   PORTA      8-bit data bus
 *   PE4 RS     0 = command, 1 = data
 *   PE5 E      the falling edge latches
 *   PE6 CS2    right controller, columns 64-127
 *   PE7 CS1    left  controller, columns 0-63
 *   PG1 R/W    0 = write, 1 = read
 * Chip select is active HIGH here, matching _glcd.c and the board.
 *
 * R/W IS NOT TIED TO GROUND, whatever _glcd.h's header comment says.  Tracing
 * the board gives Ks0108-237-PinRW -> mega128-20-PORTG1, and _glcd.c never
 * drives it: 18_GLCD_Graphics only works because it also links _port.c, whose
 * Port_init() sets DDRG = 0xFF and PORTG = 0x00 and so happens to leave R/W
 * low.  A lesson that does not link _port leaves PG1 an input with no pull-up,
 * the panel reads R/W as a read request, nothing is ever written to display
 * RAM, and the screen stays blank.  _glcd_legacy.c had this right.
 */
#define PANEL_DATA PORTA
#define PANEL_DATA_DDR DDRA
#define PANEL_CTRL PORTE
#define PANEL_CTRL_DDR DDRE
#define PANEL_RW_PORT PORTG
#define PANEL_RW_DDR DDRG
#define RW_BIT 1

#define RS_BIT 4
#define E_BIT 5
#define CS2_BIT 6
#define CS1_BIT 7

#define CS_LEFT 1
#define CS_RIGHT 2
#define CS_BOTH 3

#define CMD_DISPLAY_ON 0x3F
#define CMD_SET_Y 0x40    /* OR with column 0-63 */
#define CMD_SET_PAGE 0xB8 /* OR with page 0-7    */
#define CMD_SET_START 0xC0

/*
 * One bus transaction.  The enable pulse is what the controller latches on,
 * so the sequence is: put the byte on the bus, select the target, pulse E.
 *
 * The 10 us settling delay _glcd.c applies after EVERY byte is only required
 * after a command; the KS0108 accepts back-to-back data writes at the bus
 * cycle time.  Dropping it for data is where most of the speed-up comes from.
 */
static void bus_write(uint8_t value, uint8_t rs, uint8_t cs)
{
    PANEL_DATA = value;

    if (rs)
        PANEL_CTRL |= (1 << RS_BIT);
    else
        PANEL_CTRL &= (uint8_t) ~(1 << RS_BIT);

    if (cs & CS_LEFT)
        PANEL_CTRL |= (1 << CS1_BIT);
    else
        PANEL_CTRL &= (uint8_t) ~(1 << CS1_BIT);

    if (cs & CS_RIGHT)
        PANEL_CTRL |= (1 << CS2_BIT);
    else
        PANEL_CTRL &= (uint8_t) ~(1 << CS2_BIT);

    _delay_us(1);                          /* setup        */
    PANEL_CTRL |= (1 << E_BIT);            /* E high       */
    _delay_us(1);                          /* pulse width  */
    PANEL_CTRL &= (uint8_t) ~(1 << E_BIT); /* falling edge */
    _delay_us(2);                          /* hold         */
    PANEL_CTRL &= (uint8_t) ~((1 << CS1_BIT) | (1 << CS2_BIT));
    _delay_us(PANEL_SETTLE_US);            /* see _game.h  */
}

static void panel_cmd(uint8_t cmd, uint8_t cs)
{
    bus_write(cmd, 0, cs);
}

/* ---------------------------------------------------------- framebuffer --
 * 8 pages x 128 columns = 1024 bytes = one quarter of this MCU's SRAM.
 * Bit n of fb[page][x] is the pixel at y = page * 8 + n.
 */
static uint8_t fb[GFX_PAGES][GFX_W];

/* One bit per page: set when that page differs from what the panel holds. */
static uint8_t fb_dirty;

void gfx_clear(void)
{
    /* Only pages that actually held something are blanked and marked dirty.
     * Scanning 1024 bytes of RAM costs a few dozen microseconds; flushing one
     * page it did not need to costs 128 bus writes, about half a millisecond.
     * The scan is the cheaper of the two by an order of magnitude.
     *
     * This matters for a game that clears and redraws the whole screen every
     * frame: the pages it never draws into stay blank, stay clean, and are
     * never sent. */
    for (uint8_t page = 0; page < GFX_PAGES; page++)
    {
        uint8_t *row = fb[page];
        uint8_t used = 0;

        for (uint8_t x = 0; x < GFX_W; x++)
        {
            if (row[x])
            {
                used = 1;
                break;
            }
        }

        if (used)
        {
            memset(row, 0, GFX_W);
            fb_dirty |= (uint8_t)(1 << page);
        }
    }
}

/* ------------------------------------------------------------- plotting -- */

void gfx_pixel(uint8_t x, uint8_t y, uint8_t mode)
{
    if (x >= GFX_W || y >= GFX_H)
        return; /* clip rather than wrap, so sprites may hang off an edge */

    uint8_t page = y >> 3;
    uint8_t bit = (uint8_t)(1 << (y & 7));
    uint8_t before = fb[page][x];

    if (mode == GFX_ON)
        fb[page][x] = (uint8_t)(before | bit);
    else if (mode == GFX_OFF)
        fb[page][x] = (uint8_t)(before & (uint8_t)~bit);
    else
        fb[page][x] = (uint8_t)(before ^ bit);

    if (fb[page][x] != before)
        fb_dirty |= (uint8_t)(1 << page);
}

void gfx_hline(uint8_t x, uint8_t y, uint8_t len, uint8_t mode)
{
    while (len--)
        gfx_pixel(x++, y, mode);
}

void gfx_vline(uint8_t x, uint8_t y, uint8_t len, uint8_t mode)
{
    while (len--)
        gfx_pixel(x, y++, mode);
}

void gfx_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t mode)
{
    if (!w || !h)
        return;
    gfx_hline(x, y, w, mode);
    gfx_hline(x, (uint8_t)(y + h - 1), w, mode);
    gfx_vline(x, y, h, mode);
    gfx_vline((uint8_t)(x + w - 1), y, h, mode);
}

void gfx_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t mode)
{
    while (h--)
        gfx_hline(x, y++, w, mode);
}

void gfx_circle(uint8_t cx, uint8_t cy, uint8_t r, uint8_t mode)
{
    /* Midpoint circle: integer only, no sqrt and no floating point. */
    int8_t x = (int8_t)r;
    int8_t y = 0;
    int16_t err = (int16_t)(1 - r);

    while (x >= y)
    {
        gfx_pixel((uint8_t)(cx + x), (uint8_t)(cy + y), mode);
        gfx_pixel((uint8_t)(cx - x), (uint8_t)(cy + y), mode);
        gfx_pixel((uint8_t)(cx + x), (uint8_t)(cy - y), mode);
        gfx_pixel((uint8_t)(cx - x), (uint8_t)(cy - y), mode);
        gfx_pixel((uint8_t)(cx + y), (uint8_t)(cy + x), mode);
        gfx_pixel((uint8_t)(cx - y), (uint8_t)(cy + x), mode);
        gfx_pixel((uint8_t)(cx + y), (uint8_t)(cy - x), mode);
        gfx_pixel((uint8_t)(cx - y), (uint8_t)(cy - x), mode);

        y++;
        if (err < 0)
        {
            err += (int16_t)(2 * y + 1);
        }
        else
        {
            x--;
            err += (int16_t)(2 * (y - x) + 1);
        }
    }
}

void gfx_sprite_P(uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t w, uint8_t mode)
{
    for (uint8_t col = 0; col < w; col++)
    {
        uint8_t bits = pgm_read_byte(&bitmap[col]);
        for (uint8_t row = 0; row < 8; row++)
        {
            if (bits & (1 << row))
                gfx_pixel((uint8_t)(x + col), (uint8_t)(y + row), mode);
        }
    }
}

/* ----------------------------------------------------------------- text --
 * ASCII 32-90 only: space through 'Z'.  59 glyphs x 5 bytes = 295 bytes, and
 * PROGMEM keeps every one of them in flash instead of SRAM.
 */
#define FONT_FIRST 32
#define FONT_LAST 90

static const uint8_t font5x7[FONT_LAST - FONT_FIRST + 1][5] PROGMEM = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, /* space */
    {0x00, 0x00, 0x5F, 0x00, 0x00}, /* !     */
    {0x00, 0x07, 0x00, 0x07, 0x00}, /* "     */
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, /* #     */
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, /* $     */
    {0x23, 0x13, 0x08, 0x64, 0x62}, /* %     */
    {0x36, 0x49, 0x55, 0x22, 0x50}, /* &     */
    {0x00, 0x05, 0x03, 0x00, 0x00}, /* '     */
    {0x00, 0x1C, 0x22, 0x41, 0x00}, /* (     */
    {0x00, 0x41, 0x22, 0x1C, 0x00}, /* )     */
    {0x14, 0x08, 0x3E, 0x08, 0x14}, /* *     */
    {0x08, 0x08, 0x3E, 0x08, 0x08}, /* +     */
    {0x00, 0x50, 0x30, 0x00, 0x00}, /* ,     */
    {0x08, 0x08, 0x08, 0x08, 0x08}, /* -     */
    {0x00, 0x60, 0x60, 0x00, 0x00}, /* .     */
    {0x20, 0x10, 0x08, 0x04, 0x02}, /* /     */
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, /* 0     */
    {0x00, 0x42, 0x7F, 0x40, 0x00}, /* 1     */
    {0x42, 0x61, 0x51, 0x49, 0x46}, /* 2     */
    {0x21, 0x41, 0x45, 0x4B, 0x31}, /* 3     */
    {0x18, 0x14, 0x12, 0x7F, 0x10}, /* 4     */
    {0x27, 0x45, 0x45, 0x45, 0x39}, /* 5     */
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, /* 6     */
    {0x01, 0x71, 0x09, 0x05, 0x03}, /* 7     */
    {0x36, 0x49, 0x49, 0x49, 0x36}, /* 8     */
    {0x06, 0x49, 0x49, 0x29, 0x1E}, /* 9     */
    {0x00, 0x36, 0x36, 0x00, 0x00}, /* :     */
    {0x00, 0x56, 0x36, 0x00, 0x00}, /* ;     */
    {0x08, 0x14, 0x22, 0x41, 0x00}, /* <     */
    {0x14, 0x14, 0x14, 0x14, 0x14}, /* =     */
    {0x00, 0x41, 0x22, 0x14, 0x08}, /* >     */
    {0x02, 0x01, 0x51, 0x09, 0x06}, /* ?     */
    {0x32, 0x49, 0x79, 0x41, 0x3E}, /* @     */
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, /* A     */
    {0x7F, 0x49, 0x49, 0x49, 0x36}, /* B     */
    {0x3E, 0x41, 0x41, 0x41, 0x22}, /* C     */
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, /* D     */
    {0x7F, 0x49, 0x49, 0x49, 0x41}, /* E     */
    {0x7F, 0x09, 0x09, 0x09, 0x01}, /* F     */
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, /* G     */
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, /* H     */
    {0x00, 0x41, 0x7F, 0x41, 0x00}, /* I     */
    {0x20, 0x40, 0x41, 0x3F, 0x01}, /* J     */
    {0x7F, 0x08, 0x14, 0x22, 0x41}, /* K     */
    {0x7F, 0x40, 0x40, 0x40, 0x40}, /* L     */
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, /* M     */
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, /* N     */
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, /* O     */
    {0x7F, 0x09, 0x09, 0x09, 0x06}, /* P     */
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, /* Q     */
    {0x7F, 0x09, 0x19, 0x29, 0x46}, /* R     */
    {0x46, 0x49, 0x49, 0x49, 0x31}, /* S     */
    {0x01, 0x01, 0x7F, 0x01, 0x01}, /* T     */
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, /* U     */
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, /* V     */
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, /* W     */
    {0x63, 0x14, 0x08, 0x14, 0x63}, /* X     */
    {0x07, 0x08, 0x70, 0x08, 0x07}, /* Y     */
    {0x61, 0x51, 0x49, 0x45, 0x43}, /* Z     */
};

void gfx_char(uint8_t x, uint8_t y, char c, uint8_t mode)
{
    if (c >= 'a' && c <= 'z')
        c = (char)(c - 32); /* fold lowercase; the table stops at 'Z' */
    if ((uint8_t)c < FONT_FIRST || (uint8_t)c > FONT_LAST)
        c = ' ';

    const uint8_t *glyph = font5x7[(uint8_t)c - FONT_FIRST];
    for (uint8_t col = 0; col < 5; col++)
    {
        uint8_t bits = pgm_read_byte(&glyph[col]);
        for (uint8_t row = 0; row < 7; row++)
        {
            if (bits & (1 << row))
                gfx_pixel((uint8_t)(x + col), (uint8_t)(y + row), mode);
        }
    }
}

void gfx_text(uint8_t x, uint8_t y, const char *s, uint8_t mode)
{
    while (*s)
    {
        gfx_char(x, y, *s++, mode);
        x = (uint8_t)(x + 6); /* 5 pixels plus one of spacing */
    }
}

void gfx_number(uint8_t x, uint8_t y, uint16_t value, uint8_t width, uint8_t mode)
{
    /* Right-aligned, built backwards.  Avoids printf, which would cost over a
     * kilobyte of flash and a buffer this engine has no room for. */
    char buf[6];
    uint8_t n = 0;

    do
    {
        buf[n++] = (char)('0' + (value % 10));
        value /= 10;
    } while (value && n < sizeof(buf));

    while (width > n)
    {
        gfx_char(x, y, ' ', mode);
        x = (uint8_t)(x + 6);
        width--;
    }
    while (n--)
    {
        gfx_char(x, y, buf[n], mode);
        x = (uint8_t)(x + 6);
    }
}

/* ---------------------------------------------------------------- flush -- */

static uint16_t flush_page(uint8_t page)
{
    /* The KS0108 auto-increments its column counter after each data write, so
     * the address is set once per controller and then 64 bytes are streamed. */
    panel_cmd(CMD_SET_PAGE | page, CS_BOTH);

    panel_cmd(CMD_SET_Y | 0, CS_LEFT);
    for (uint8_t x = 0; x < 64; x++)
        bus_write(fb[page][x], 1, CS_LEFT);

    panel_cmd(CMD_SET_Y | 0, CS_RIGHT);
    for (uint8_t x = 64; x < 128; x++)
        bus_write(fb[page][x], 1, CS_RIGHT);

    return 128;
}

uint16_t gfx_flush(void)
{
    uint16_t written = 0;

    for (uint8_t page = 0; page < GFX_PAGES; page++)
    {
        if (fb_dirty & (1 << page))
            written = (uint16_t)(written + flush_page(page));
    }
    fb_dirty = 0;
    return written;
}

uint16_t gfx_flush_all(void)
{
    uint16_t written = 0;

    for (uint8_t page = 0; page < GFX_PAGES; page++)
        written = (uint16_t)(written + flush_page(page));

    fb_dirty = 0;
    return written;
}

void gfx_init(void)
{
    PANEL_DATA_DDR = 0xFF;
    PANEL_CTRL_DDR |= (1 << RS_BIT) | (1 << E_BIT) | (1 << CS1_BIT) | (1 << CS2_BIT);
    PANEL_CTRL &= (uint8_t) ~((1 << E_BIT) | (1 << CS1_BIT) | (1 << CS2_BIT));

    /* R/W low, and held low for the life of the program: this engine only ever
     * writes.  Without this the pin floats and the panel never accepts a byte. */
    PANEL_RW_DDR |= (1 << RW_BIT);
    PANEL_RW_PORT &= (uint8_t) ~(1 << RW_BIT);

    _delay_ms(50); /* let the panel come up before talking to it */

    panel_cmd(CMD_DISPLAY_ON, CS_BOTH);
    panel_cmd(CMD_SET_START | 0, CS_BOTH);
    panel_cmd(CMD_SET_PAGE | 0, CS_BOTH);
    panel_cmd(CMD_SET_Y | 0, CS_BOTH);

    gfx_clear();
    gfx_flush_all();
}

/* ---------------------------------------------------------------- input --
 * Six buttons on PORTD, each with a pull-up, so pressed reads LOW.
 * PD2 and PD3 are skipped: they are RXD1/TXD1.
 */
#define BTN_MASK (BTN_A | BTN_B | BTN_LEFT | BTN_RIGHT | BTN_UP | BTN_DOWN)

static uint8_t btn_stable;
static uint8_t btn_prev;
static uint8_t btn_raw_last;
static uint8_t btn_edge;

void input_poll(void)
{
    uint8_t raw = (uint8_t)(~PIND) & BTN_MASK; /* 1 = pressed */

    /* Two consecutive agreeing samples count as settled.  The tick rate is
     * therefore the debounce window: at 50 Hz a bounce must survive 20 ms. */
    if (raw == btn_raw_last)
    {
        btn_prev = btn_stable;
        btn_stable = raw;
        btn_edge = (uint8_t)(btn_stable & (uint8_t)~btn_prev);
    }
    else
    {
        btn_edge = 0;
    }
    btn_raw_last = raw;
}

uint8_t btn_held(uint8_t mask) { return (uint8_t)(btn_stable & mask); }
uint8_t btn_pressed(uint8_t mask) { return (uint8_t)(btn_edge & mask); }

/* --------------------------------------------------------------- timing -- */

static volatile uint8_t tick_flag;
static volatile uint8_t tick_overrun;
static volatile uint16_t tick_count;

void game_timer_init(uint8_t hz)
{
    if (hz < 10)
        hz = 10;
    if (hz > 200)
        hz = 200;

    /* Buttons are inputs with pull-ups; leave PD2/PD3 alone for the UART. */
    DDRD &= (uint8_t)~BTN_MASK;
    PORTD |= BTN_MASK;

    /* CTC on Timer1, prescaler 256.  At 16 MHz that is 62500 counts/second,
     * so OCR1A = 62500/hz - 1 covers 10-200 Hz inside 16 bits. */
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS12);
    OCR1A = (uint16_t)((62500UL / hz) - 1);
    TIMSK |= (1 << OCIE1A);

    tick_flag = 0;
    tick_overrun = 0;
    tick_count = 0;

    sei();
}

ISR(TIMER1_COMPA_vect)
{
    /* If the previous tick has not been consumed the frame overran.  The tick
     * is dropped rather than queued, so a slow frame slows the game down
     * instead of making it lurch forward to catch up. */
    if (tick_flag)
        tick_overrun = 1;

    tick_flag = 1;
    tick_count++;
}

uint8_t game_tick_ready(void)
{
    if (!tick_flag)
        return 0;

    tick_flag = 0;
    return 1;
}

uint16_t game_ticks(void)
{
    uint16_t t;
    uint8_t s = SREG;

    cli(); /* tick_count is 16-bit; an interrupt mid-read would tear it */
    t = tick_count;
    SREG = s;
    return t;
}

uint8_t game_overrun(void)
{
    uint8_t o = tick_overrun;
    tick_overrun = 0;
    return o;
}

/* ---------------------------------------------------------------- sound --
 * PG4 drives an AudioOut through a slide switch on the shared board.
 *
 * There is no hardware way to toggle PG4 from a timer - OC2 is PB7 - so the
 * square wave is made by flipping the pin in the Timer2 compare interrupt.
 * That keeps sound_tone() non-blocking, unlike _buzzer.c's Sound(), which
 * busy-waits for the whole note and would cost a 50 Hz game five frames per
 * 100 ms beep.
 *
 * Timer2 runs at prescaler 256: 16 MHz / 256 = 62500 Hz.  A half-period of
 * OCR2+1 counts gives f = 62500 / (2 * (OCR2 + 1)), so the 8-bit OCR2 covers
 * about 123 Hz (OCR2 = 254) up to a few kHz - the whole useful beep range.
 */
#define SPEAKER_PORT PORTG
#define SPEAKER_DDR DDRG
#define SPEAKER_BIT 4

#define SOUND_TIMER_HZ 62500UL
#define SOUND_MIN_HZ 123
#define SOUND_MAX_HZ 4000

static volatile uint16_t tone_toggles;

void sound_init(void)
{
    SPEAKER_DDR |= (1 << SPEAKER_BIT);
    SPEAKER_PORT &= (uint8_t) ~(1 << SPEAKER_BIT);
    tone_toggles = 0;
}

void sound_stop(void)
{
    TIMSK &= (uint8_t) ~(1 << OCIE2);
    tone_toggles = 0;
    SPEAKER_PORT &= (uint8_t) ~(1 << SPEAKER_BIT);
}

void sound_tone(uint16_t hz, uint16_t ms)
{
    if (hz < SOUND_MIN_HZ)
        hz = SOUND_MIN_HZ;
    if (hz > SOUND_MAX_HZ)
        hz = SOUND_MAX_HZ;

    /* One toggle every half period, so the pin completes hz cycles a second
     * and the note lasts ms milliseconds: toggles = 2 * hz * ms / 1000. */
    uint32_t toggles = (2UL * hz * ms) / 1000UL;
    if (toggles == 0)
        toggles = 1;
    if (toggles > 0xFFFF)
        toggles = 0xFFFF;

    uint8_t ocr = (uint8_t)((SOUND_TIMER_HZ / (2UL * hz)) - 1);

    TIMSK &= (uint8_t) ~(1 << OCIE2); /* retune without a half-written state */
    TCCR2 = (1 << WGM21) | (1 << CS22) | (1 << CS21) | (1 << CS20); /* CTC, /256 */
    OCR2 = ocr;
    TCNT2 = 0;
    tone_toggles = (uint16_t)toggles;
    TIMSK |= (1 << OCIE2);
}

uint8_t sound_busy(void) { return tone_toggles != 0; }

ISR(TIMER2_COMP_vect)
{
    if (tone_toggles)
    {
        SPEAKER_PORT ^= (1 << SPEAKER_BIT);
        tone_toggles--;
    }
    else
    {
        /* Leave the pin low so the speaker is not held at a DC offset. */
        SPEAKER_PORT &= (uint8_t) ~(1 << SPEAKER_BIT);
        TIMSK &= (uint8_t) ~(1 << OCIE2);
    }
}

/* --------------------------------------------------------------- random -- */

static uint16_t rng_state = 0xACE1;

void game_srand(uint16_t seed) { rng_state = seed ? seed : 1; }

uint16_t game_rand(void)
{
    /* 16-bit xorshift.  Three shifts and three XORs, no division, and it
     * cycles through all 65535 non-zero values before repeating. */
    rng_state ^= (uint16_t)(rng_state << 7);
    rng_state ^= (uint16_t)(rng_state >> 9);
    rng_state ^= (uint16_t)(rng_state << 8);
    return rng_state;
}
