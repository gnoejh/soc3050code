/*
 * Graphics Display Project (KS0108 GLCD, ATmega128)
 *
 * Teaching goals:
 * - Understand GLCD coordinate system and dual-controller layout
 * - Practice drawing pixels/lines/rectangles/circles and placing text
 * - Learn GLCD page/column addressing and 5x7 font rendering
 * - Compare hardware vs simulator timing behavior
 *
 * Hardware & API quick reference:
 * - Resolution: 128x64 pixels; API uses x=0..63 (rows/page pixels), y=0..127 (columns)
 * - Dual controllers: left (y 0..63), right (y 64..127)
 * - Key functions:
 *     lcd_init(), lcd_clear(), lcd_xy(rowPage, colChar), lcd_string(rowPage, colChar, str)
 *     GLCD_Dot(xRow, yCol), GLCD_Line(x1,y1,x2,y2), GLCD_Rectangle(x1,y1,x2,y2), GLCD_Circle(x,y,r)
 *     GLCD_4DigitDecimal(n)
 * - Text grid: lcd_xy(rowPage 0..7, charCol 0..19), 5x7 font stretched to 6x8 slots
 *
 * How to use in class:
 * 1) In main(), uncomment exactly ONE demo call to focus the lesson
 * 2) Build, then run in SimulIDE 0.4.15 (tools/simulide/Simulator0415.simu)
 * 3) Observe left/right controller split and pixel addressing
 * 4) Try modifying coordinates and see results
 */

#include <avr/io.h>
#include <util/delay.h>
#include "../../shared_libs/_glcd.h"
#include "../../shared_libs/_init.h"

// Screen dimensions for this GLCD API: x=0..63 (rows), y=0..127 (columns)
#define GLCD_ROWS 64
#define GLCD_COLS 128

// Display strings (from manual)
static char Dis_Scr_IO_ON1[] = {"O"};
static char Dis_Scr[] = {"#$%&'()*+,-./0123456"};

static void demo_header(void)
{
    lcd_clear();
    lcd_string(0, 0, "====================");
    lcd_string(1, 0, "   ATmega128 GLCD   ");
    lcd_string(2, 0, Dis_Scr);
    lcd_string(6, 0, "SOC3050 Graphics");
}

static void demo_primitives(void)
{
    lcd_clear();
    lcd_string(0, 0, "Primitives: line/rect/circ");

    // Line
    GLCD_Line(10, 5, 50, 30);
    GLCD_Line(5, 120, 40, 70);

    // Rectangle
    GLCD_Rectangle(20, 40, 40, 70);

    // Circle
    GLCD_Circle(20, 95, 8);

    // Number display
    lcd_xy(6, 0);
    GLCD_4DigitDecimal(1234);
}

static void demo_radiating_lines(void)
{
    const unsigned char cx = GLCD_ROWS / 2; // 32
    const unsigned char cy = GLCD_COLS / 2; // 64

    lcd_clear();
    lcd_string(0, 0, "Radiating Lines");

    // Radiate to top and bottom edges
    for (unsigned char y = 0; y < GLCD_COLS; y += 8)
    {
        GLCD_Line(cx, cy, 0, y);             // to top edge (x=0)
        GLCD_Line(cx, cy, GLCD_ROWS - 1, y); // to bottom edge (x=63)
    }
}

static void demo_nested_rectangles(void)
{
    lcd_clear();
    lcd_string(0, 0, "Nested Rectangles");

    unsigned char inset = 0;
    // Keep within bounds; leave a margin for the title row
    while (20 + inset < GLCD_COLS - 1 - inset && 5 + inset < GLCD_ROWS - 1 - inset)
    {
        unsigned char x1 = 5 + inset;
        unsigned char y1 = 20 + inset;
        unsigned char x2 = (GLCD_ROWS - 1) - inset;
        unsigned char y2 = (GLCD_COLS - 1) - inset;
        GLCD_Rectangle(x1, y1, x2, y2);
        inset += 4;
    }
}

static void demo_concentric_circles(void)
{
    lcd_clear();
    lcd_string(0, 0, "Concentric Circles");

    const unsigned char cx = GLCD_ROWS / 2; // 32
    const unsigned char cy = GLCD_COLS / 2; // 64

    for (unsigned char r = 6; r <= 26; r += 5)
    {
        GLCD_Circle(cx, cy, r);
    }
}

static void demo_grid(void)
{
    lcd_clear();
    lcd_string(0, 0, "Grid (8x8 spacing)");

    // Vertical lines every 8 columns (y axis)
    for (unsigned char y = 0; y < GLCD_COLS; y += 8)
    {
        GLCD_Line(1, y, GLCD_ROWS - 1, y);
    }
    // Horizontal lines every 8 rows (x axis)
    for (unsigned char x = 1; x < GLCD_ROWS; x += 8)
    {
        GLCD_Line(x, 0, x, GLCD_COLS - 1);
    }
}

static void demo_text_pages(void)
{
    lcd_clear();
    lcd_string(0, 0, "Text Pages");
    lcd_string(1, 0, "Left panel (0..9)");
    lcd_string(2, 10, "Right panel (10..19)");

    // Show page numbers on left (y 0..9)
    for (unsigned char row = 3; row <= 6; row++)
    {
        lcd_xy(row, 0);
        lcd_string(row, 0, "Y: 0 1 2 3 4 5 6 7 8 9");
    }

    // Right panel text examples (y 10..19)
    lcd_string(3, 10, "Hello, Right!");
    lcd_string(4, 10, "Symbols: ");
    lcd_string(5, 10, Dis_Scr);

    // A single character demo
    lcd_string(6, 5, Dis_Scr_IO_ON1); // prints "O"
}

static void demo_pixels_and_buffer(void)
{
    lcd_clear();
    lcd_string(0, 0, "Pixels & Buffer (Checker)");

    // Clear software buffer
    ScreenBuffer_clear();

    // Draw a checker pattern directly via GLCD_Dot (and buffer mirrors to screen)
    for (unsigned char x = 0; x < GLCD_ROWS; x++)
    {
        for (unsigned char y = 0; y < GLCD_COLS; y++)
        {
            if (((x >> 3) + (y >> 3)) & 1)
                GLCD_Dot(x, y);
        }
    }
}

static void demo_page_addressing(void)
{
    // Demonstrate page (x) vs column (y) moves and left/right controller split
    lcd_clear();
    lcd_string(0, 0, "Page Addressing");

    // Draw a vertical bar on the left controller area
    for (unsigned char x = 1; x < 8; x++)
    {
        // Column y=10 (left half), fill one bit in each page row
        GLCD_Axis_xy(x, 10);
        datal(0xFF); // solid byte downwards in that (page,column)
    }

    // Draw a vertical bar on the right controller area at y=90
    for (unsigned char x = 1; x < 8; x++)
    {
        GLCD_Axis_xy(x, 90);
        datar(0xFF);
    }

    // Annotate with text showing split
    lcd_string(6, 0, "CS1: y 0..63   CS2: y 64..127");
}

static void run_all_demos(void)
{
    demo_header();
    _delay_ms(1000);

    demo_pixels_and_buffer();
    _delay_ms(1200);

    demo_page_addressing();
    _delay_ms(1200);

    demo_primitives();
    _delay_ms(1200);

    demo_radiating_lines();
    _delay_ms(1200);

    demo_nested_rectangles();
    _delay_ms(1200);

    demo_concentric_circles();
    _delay_ms(1200);

    demo_grid();
    _delay_ms(1500);

    demo_text_pages();
    _delay_ms(1500);
}

int main(void)
{
    // Initialize system and GLCD
    init_devices();

    // IMPORTANT: Select ONE demo by uncommenting a single line below:
    // demo_header();              // Banner only (text basics)
    // demo_pixels_and_buffer();   // Pixels + buffer (checker pattern)
    // demo_page_addressing();     // Page/column addressing and CS1/CS2 split
    // demo_primitives();          // Lines, rectangle, circle, number
    // demo_radiating_lines();     // Radiating lines from center
    // demo_nested_rectangles();   // Inset rectangles
    // demo_concentric_circles();  // Multiple radii
    // demo_grid();                // 8x8 grid
    // demo_text_pages();          // Left/right text pages and symbols
    run_all_demos(); // Cycle through all demos (default)

    // demo_header(); // ← TEST: Text display only for debugging

    while (1)
    { /* keep display */
    }
    return 0;
}