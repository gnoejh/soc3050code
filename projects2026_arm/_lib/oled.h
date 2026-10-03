/*
 * oled.h - SSD1306 128x64 OLED on I2C, with a RAM framebuffer   LIBS=oled i2c
 *
 * Written for lesson 12 (Game) and shared by every application lesson after
 * it.  The display is the app board's: a Wokwi `board-ssd1306` at address
 * 0x3C on I2C1 (PB8 SCL, PB9 SDA) - the same two wires as lesson 09's MPU6050,
 * which the drone and balancing lessons put on the bus beside it.
 *
 * The model, in three sentences.  Every drawing call changes a 1 KB copy of
 * the screen in RAM and touches no wire.  oled_flush() sends only the 8-row
 * PAGES that changed since the last flush.  So drawing is cheap, and the bus
 * cost of a frame is proportional to how much of the picture moved.
 *
 * Coordinates: x 0..127 left to right, y 0..63 top to bottom.  Anything off
 * the screen is clipped, never an error, so a sprite can slide off the edge.
 *
 * Pure C apart from i2c_write_read(): the drawing half compiles on a PC, which
 * is how the application lessons' host tests check what was drawn.
 */
#ifndef OLED_H_INCLUDED
#define OLED_H_INCLUDED

#include <stdint.h>

#define OLED_W      128
#define OLED_H      64
#define OLED_PAGES  (OLED_H / 8)
#define OLED_ADDR   0x3Cu           /* board-ssd1306 default; some modules 0x3D */

enum { OLED_OFF = 0, OLED_ON = 1, OLED_XOR = 2 };    /* a pixel's "colour" */

/* Bring the panel up.  i2c_init() first.  Returns 0, or the I2C error of the
 * first command that failed - I2C_NACK means nothing answered at 0x3C. */
int  oled_init(void);

/* ---- drawing: RAM only, no bus traffic -------------------------------- */
void oled_clear(void);
void oled_pixel(int x, int y, int colour);
int  oled_get(int x, int y);                       /* 1 if lit, 0 otherwise */
void oled_hline(int x, int y, int w, int colour);
void oled_vline(int x, int y, int h, int colour);
void oled_line(int x0, int y0, int x1, int y1, int colour);
void oled_rect(int x, int y, int w, int h, int colour);
void oled_fill_rect(int x, int y, int w, int h, int colour);
void oled_circle(int cx, int cy, int r, int colour);
void oled_fill_circle(int cx, int cy, int r, int colour);

/* A sprite in the panel's own format: `w` columns, each one byte per 8 rows,
 * least-significant bit at the top - the way the font below is stored.  Lit
 * bits are drawn in `colour`; clear bits leave the screen alone. */
void oled_sprite(int x, int y, int w, int h, const uint8_t *bits, int colour);

/* 5x7 text in a 6x8 cell.  ASCII 32..126.  Returns the x after the text. */
int  oled_char(int x, int y, char c, int colour);
int  oled_text(int x, int y, const char *s, int colour);
int  oled_printf(int x, int y, const char *fmt, ...)
     __attribute__((format(printf, 3, 4)));            /* integers only: nano */

/* ---- to the panel ------------------------------------------------------ */
int  oled_flush(void);              /* send dirty pages; 0 or an I2C error   */
void oled_invalidate(void);         /* mark every page dirty: full repaint   */
int  oled_contrast(uint8_t level);  /* 0..255                                */
int  oled_invert(int on);           /* hardware invert, no RAM change        */

/* ---- for the profile line every application lesson prints -------------- */
uint32_t oled_bytes_sent(void);     /* data bytes put on the bus so far      */
uint32_t oled_flushes(void);
const uint8_t *oled_page(int page); /* 128 bytes of RAM page `page` - tests  */

#endif
