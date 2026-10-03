/*
 * oled.c - SSD1306 128x64 on I2C, framebuffer + dirty pages   LIBS=oled i2c
 *
 * The SSD1306 stores the screen as 8 PAGES of 128 bytes.  One byte is one
 * column of 8 pixels, least-significant bit at the top:
 *
 *        x = 0   1   2  ...  127
 *   page 0 [b0][b0][b0]       y 0      bit 0
 *          [b1][b1][b1]       y 1      bit 1
 *            ...                ...
 *          [b7][b7][b7]       y 7      bit 7
 *   page 1                    y 8..15
 *     ...
 *   page 7                    y 56..63
 *
 * So pixel (x, y) is bit (y % 8) of byte fb[y / 8][x].  Every function below
 * is that sentence plus clipping.
 *
 * Every I2C write to the panel starts with a CONTROL byte: 0x00 says "the
 * rest are commands", 0x40 says "the rest are pixels".  Each page's buffer
 * keeps a spare byte in front permanently set to 0x40, so a page goes out as
 * one i2c_write_read() with no copying: [0x40][128 pixel bytes].
 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "i2c.h"
#include "oled.h"

static uint8_t  fb[OLED_PAGES][1 + OLED_W];   /* [0] = 0x40, then 128 columns */
static uint8_t  dirty;                         /* bit p = page p changed       */
static uint32_t bytes_sent, flushes;

/* ------------------------------------------------------------- the wire -- */
static int command(const uint8_t *cmds, uint32_t n)
{
    uint8_t buf[8];
    buf[0] = 0x00;                             /* control byte: commands follow */
    memcpy(&buf[1], cmds, n);
    return i2c_write_read(OLED_ADDR, buf, n + 1u, 0, 0);
}

int oled_init(void)
{
    /* The SSD1306 datasheet's power-up sequence for a 128x64 panel with the
     * internal charge pump - the one every library (Adafruit, u8g2) sends. */
    static const uint8_t init[][3] = {
        {2, 0xAE, 0},          /* display off while we configure             */
        {3, 0xD5, 0x80},       /* clock divide / oscillator: the default      */
        {3, 0xA8, 0x3F},       /* multiplex ratio: 64 rows                    */
        {3, 0xD3, 0x00},       /* no vertical offset                          */
        {2, 0x40, 0},          /* start line 0                                */
        {3, 0x8D, 0x14},       /* charge pump ON - without it the panel is dark */
        {3, 0x20, 0x00},       /* horizontal addressing: column, then page    */
        {2, 0xA1, 0},          /* column 127 is SEG0: x runs left to right    */
        {2, 0xC8, 0},          /* scan COM63..0: y runs top to bottom         */
        {3, 0xDA, 0x12},       /* COM pins: alternative, for 64 rows          */
        {3, 0x81, 0xCF},       /* contrast                                    */
        {3, 0xD9, 0xF1},       /* pre-charge period                           */
        {3, 0xDB, 0x40},       /* VCOMH deselect level                        */
        {2, 0xA4, 0},          /* show RAM, not all-on                        */
        {2, 0xA6, 0},          /* normal, not inverted                        */
        {2, 0xAF, 0},          /* display ON                                  */
    };
    for (uint32_t p = 0; p < OLED_PAGES; p++) { fb[p][0] = 0x40; }
    for (uint32_t i = 0; i < sizeof init / sizeof init[0]; i++) {
        int rc = command(&init[i][1], (uint32_t)init[i][0] - 1u);
        if (rc) { return rc; }
    }
    oled_clear();
    oled_invalidate();
    return oled_flush();
}

int oled_flush(void)
{
    for (int p = 0; p < OLED_PAGES; p++) {
        if (!(dirty & (1u << p))) { continue; }
        /* Column window 0..127, page window p..p, then 128 data bytes: the
         * panel's address pointer walks the window by itself. */
        const uint8_t win[6] = {0x21, 0, OLED_W - 1, 0x22, (uint8_t)p, (uint8_t)p};
        int rc = command(win, sizeof win);
        if (rc == 0) { rc = i2c_write_read(OLED_ADDR, fb[p], 1u + OLED_W, 0, 0); }
        if (rc) { return rc; }               /* page stays dirty: next flush retries */
        dirty &= (uint8_t)~(1u << p);
        bytes_sent += 1u + OLED_W;
    }
    flushes++;
    return 0;
}

void oled_invalidate(void) { dirty = 0xFF; }

int oled_contrast(uint8_t level)
{
    const uint8_t c[2] = {0x81, level};
    return command(c, 2);
}

int oled_invert(int on)
{
    const uint8_t c[1] = {on ? 0xA7 : 0xA6};
    return command(c, 1);
}

uint32_t oled_bytes_sent(void) { return bytes_sent; }
uint32_t oled_flushes(void)    { return flushes; }
const uint8_t *oled_page(int page) { return &fb[page & 7][1]; }

/* ------------------------------------------------------------ pixels ---- */
void oled_clear(void)
{
    /* Only pages that hold something become dirty: a game that clears and
     * redraws every frame would otherwise resend all 1 KB, every frame. */
    for (int p = 0; p < OLED_PAGES; p++) {
        for (int x = 1; x <= OLED_W; x++) {
            if (fb[p][x]) {
                memset(&fb[p][1], 0, OLED_W);
                dirty |= (uint8_t)(1u << p);
                break;
            }
        }
    }
}

void oled_pixel(int x, int y, int colour)
{
    if ((unsigned)x >= OLED_W || (unsigned)y >= OLED_H) { return; }
    uint8_t *b = &fb[y >> 3][1 + x];
    uint8_t  m = (uint8_t)(1u << (y & 7));
    uint8_t  old = *b;
    if (colour == OLED_ON)       { *b |= m; }
    else if (colour == OLED_XOR) { *b ^= m; }
    else                         { *b &= (uint8_t)~m; }
    if (*b != old) { dirty |= (uint8_t)(1u << (y >> 3)); }
}

int oled_get(int x, int y)
{
    if ((unsigned)x >= OLED_W || (unsigned)y >= OLED_H) { return 0; }
    return (fb[y >> 3][1 + x] >> (y & 7)) & 1;
}

void oled_hline(int x, int y, int w, int colour)
{
    for (int i = 0; i < w; i++) { oled_pixel(x + i, y, colour); }
}

void oled_vline(int x, int y, int h, int colour)
{
    for (int i = 0; i < h; i++) { oled_pixel(x, y + i, colour); }
}

/* Bresenham: integer steps only, one pixel per step along the long axis. */
void oled_line(int x0, int y0, int x1, int y1, int colour)
{
    int dx = x1 > x0 ? x1 - x0 : x0 - x1, sx = x0 < x1 ? 1 : -1;
    int dy = y1 > y0 ? y0 - y1 : y1 - y0, sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        oled_pixel(x0, y0, colour);
        if (x0 == x1 && y0 == y1) { break; }
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void oled_rect(int x, int y, int w, int h, int colour)
{
    if (w <= 0 || h <= 0) { return; }
    oled_hline(x, y, w, colour);
    if (h > 1) { oled_hline(x, y + h - 1, w, colour); }
    if (h > 2) {
        oled_vline(x, y + 1, h - 2, colour);
        if (w > 1) { oled_vline(x + w - 1, y + 1, h - 2, colour); }
    }
}

void oled_fill_rect(int x, int y, int w, int h, int colour)
{
    for (int i = 0; i < h; i++) { oled_hline(x, y + i, w, colour); }
}

/* Midpoint circle: eight symmetric points per step. */
void oled_circle(int cx, int cy, int r, int colour)
{
    int x = r, y = 0, err = 1 - r;
    while (x >= y) {
        oled_pixel(cx + x, cy + y, colour); oled_pixel(cx - x, cy + y, colour);
        oled_pixel(cx + x, cy - y, colour); oled_pixel(cx - x, cy - y, colour);
        if (x != y) {
            oled_pixel(cx + y, cy + x, colour); oled_pixel(cx - y, cy + x, colour);
            oled_pixel(cx + y, cy - x, colour); oled_pixel(cx - y, cy - x, colour);
        }
        y++;
        if (err < 0) { err += 2 * y + 1; }
        else         { x--; err += 2 * (y - x) + 1; }
    }
}

void oled_fill_circle(int cx, int cy, int r, int colour)
{
    for (int dy = -r; dy <= r; dy++) {
        for (int dx = -r; dx <= r; dx++) {
            if (dx * dx + dy * dy <= r * r + r) { oled_pixel(cx + dx, cy + dy, colour); }
        }
    }
}

void oled_sprite(int x, int y, int w, int h, const uint8_t *bits, int colour)
{
    for (int col = 0; col < w; col++) {
        for (int row = 0; row < h; row++) {
            if (bits[(row >> 3) * w + col] & (1u << (row & 7))) {
                oled_pixel(x + col, y + row, colour);
            }
        }
    }
}

/* -------------------------------------------------------------- text ---- */
/* ASCII 32..126, 5 columns each, bit 0 at the top: the panel's own format.
 * `const` is enough to keep it in flash - on ARM, flash and RAM share one
 * address space and a plain load reads either.  The AVR edition needed
 * PROGMEM and pgm_read_byte() for the same table (shared_libs/_game.c). */
#define FONT_FIRST 32
#define FONT_LAST  126
static const uint8_t font5x7[FONT_LAST - FONT_FIRST + 1][5] = {
    {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, /* space !  */
    {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14}, /* "  #     */
    {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62}, /* $  %     */
    {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00}, /* &  '     */
    {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00}, /* (  )     */
    {0x14,0x08,0x3E,0x08,0x14}, {0x08,0x08,0x3E,0x08,0x08}, /* *  +     */
    {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, /* ,  -     */
    {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02}, /* .  /     */
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00}, /* 0  1     */
    {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31}, /* 2  3     */
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39}, /* 4  5     */
    {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03}, /* 6  7     */
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}, /* 8  9     */
    {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00}, /* :  ;     */
    {0x08,0x14,0x22,0x41,0x00}, {0x14,0x14,0x14,0x14,0x14}, /* <  =     */
    {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x51,0x09,0x06}, /* >  ?     */
    {0x32,0x49,0x79,0x41,0x3E}, {0x7E,0x11,0x11,0x11,0x7E}, /* @  A     */
    {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22}, /* B  C     */
    {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, /* D  E     */
    {0x7F,0x09,0x09,0x09,0x01}, {0x3E,0x41,0x49,0x49,0x7A}, /* F  G     */
    {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00}, /* H  I     */
    {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41}, /* J  K     */
    {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x0C,0x02,0x7F}, /* L  M     */
    {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E}, /* N  O     */
    {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, /* P  Q     */
    {0x7F,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31}, /* R  S     */
    {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F}, /* T  U     */
    {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x38,0x40,0x3F}, /* V  W     */
    {0x63,0x14,0x08,0x14,0x63}, {0x07,0x08,0x70,0x08,0x07}, /* X  Y     */
    {0x61,0x51,0x49,0x45,0x43}, {0x00,0x7F,0x41,0x41,0x00}, /* Z  [     */
    {0x02,0x04,0x08,0x10,0x20}, {0x00,0x41,0x41,0x7F,0x00}, /* \  ]     */
    {0x04,0x02,0x01,0x02,0x04}, {0x40,0x40,0x40,0x40,0x40}, /* ^  _     */
    {0x00,0x01,0x02,0x04,0x00}, {0x20,0x54,0x54,0x54,0x78}, /* `  a     */
    {0x7F,0x48,0x44,0x44,0x38}, {0x38,0x44,0x44,0x44,0x20}, /* b  c     */
    {0x38,0x44,0x44,0x48,0x7F}, {0x38,0x54,0x54,0x54,0x18}, /* d  e     */
    {0x08,0x7E,0x09,0x01,0x02}, {0x08,0x54,0x54,0x54,0x3C}, /* f  g     */
    {0x7F,0x08,0x04,0x04,0x78}, {0x00,0x44,0x7D,0x40,0x00}, /* h  i     */
    {0x20,0x40,0x44,0x3D,0x00}, {0x7F,0x10,0x28,0x44,0x00}, /* j  k     */
    {0x00,0x41,0x7F,0x40,0x00}, {0x7C,0x04,0x18,0x04,0x78}, /* l  m     */
    {0x7C,0x08,0x04,0x04,0x78}, {0x38,0x44,0x44,0x44,0x38}, /* n  o     */
    {0x7C,0x14,0x14,0x14,0x08}, {0x08,0x14,0x14,0x18,0x7C}, /* p  q     */
    {0x7C,0x08,0x04,0x04,0x08}, {0x48,0x54,0x54,0x54,0x20}, /* r  s     */
    {0x04,0x3F,0x44,0x40,0x20}, {0x3C,0x40,0x40,0x20,0x7C}, /* t  u     */
    {0x1C,0x20,0x40,0x20,0x1C}, {0x3C,0x40,0x30,0x40,0x3C}, /* v  w     */
    {0x44,0x28,0x10,0x28,0x44}, {0x0C,0x50,0x50,0x50,0x3C}, /* x  y     */
    {0x44,0x64,0x54,0x4C,0x44}, {0x00,0x08,0x36,0x41,0x00}, /* z  {     */
    {0x00,0x00,0x7F,0x00,0x00}, {0x00,0x41,0x36,0x08,0x00}, /* |  }     */
    {0x08,0x04,0x08,0x10,0x08},                             /* ~        */
};

int oled_char(int x, int y, char c, int colour)
{
    if ((unsigned char)c < FONT_FIRST || (unsigned char)c > FONT_LAST) { c = '?'; }
    oled_sprite(x, y, 5, 7, font5x7[(unsigned char)c - FONT_FIRST], colour);
    return x + 6;
}

int oled_text(int x, int y, const char *s, int colour)
{
    while (*s) { x = oled_char(x, y, *s++, colour); }
    return x;
}

int oled_printf(int x, int y, const char *fmt, ...)
{
    char buf[32];                       /* one line of text: 21 cells, and spare */
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    return oled_text(x, y, buf, OLED_ON);
}
