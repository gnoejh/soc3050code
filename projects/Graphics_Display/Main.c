/*
 * =============================================================================
 * GRAPHICS LCD (GLCD) PROGRAMMING - EDUCATIONAL DEMONSTRATION
 * =============================================================================
 *
 * PROJECT: Graphics_Display
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Educational demonstration of KS0108-based Graphic LCD (128x64) programming.
 * Students learn coordinate systems, dual-controller architecture, drawing
 * primitives, text rendering, and page-based memory addressing.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Understand GLCD coordinate system (x=0..63 rows, y=0..127 columns)
 * 2. Learn dual-controller architecture (CS1: left half, CS2: right half)
 * 3. Master page-based memory organization (8 pages of 128 columns)
 * 4. Practice drawing primitives (pixels, lines, rectangles, circles)
 * 5. Implement text rendering with 5x7 font (20 chars × 8 rows grid)
 * 6. Compare hardware timing vs simulator behavior
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - KS0108 128x64 Graphic LCD (dual-controller)
 * - Contrast potentiometer for display adjustment
 * - 5V power supply
 *
 * DOCUMENTATION REFERENCE:
 * ATmega128 Datasheet: https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/2467S.pdf
 * - I/O Ports (pages 62-75)
 * - Timing specifications (pages 301-320)
 *
 * COORDINATE SYSTEM (CRITICAL FOR STUDENTS):
 * - Screen: 128 columns (y-axis) × 64 rows (x-axis)
 * - Origin: Top-left corner (x=0, y=0)
 * - X-axis (rows): 0..63 (vertical, top to bottom)
 * - Y-axis (cols): 0..127 (horizontal, left to right)
 * - Dual controllers: CS1 controls y=0..63 (left), CS2 controls y=64..127 (right)
 *
 * PAGE-BASED MEMORY:
 * - 8 pages (0..7), each page = 8 pixels tall
 * - Page 0: rows 0-7,   Page 1: rows 8-15,  ...,  Page 7: rows 56-63
 * - Each page has 128 columns (bytes)
 * - Writing 0xFF to page 0, column 10 → lights up 8 vertical pixels
 *
 * TEXT GRID:
 * - 20 characters wide (each char = 6 pixels including spacing)
 * - 8 rows tall (each row = 8 pixels, 5x7 font + 1 pixel spacing)
 * - lcd_xy(row, col): row 0..7, col 0..19
 *
 * LEARNING PROGRESSION (10 Demos):
 * - Demo 1: Text Header (basic text output)
 * - Demo 2: Single Pixel Drawing (GLCD_Dot)
 * - Demo 3: Page Addressing (understand memory layout)
 * - Demo 4: Line Drawing (Bresenham's algorithm)
 * - Demo 5: Rectangle Drawing (filled and outline)
 * - Demo 6: Circle Drawing (midpoint circle algorithm)
 * - Demo 7: Text Pages (left/right controller split)
 * - Demo 8: Radiating Lines (center-out patterns)
 * - Demo 9: Nested Shapes (concentric patterns)
 * - Demo 10: Grid Pattern (spacing and alignment)
 *
 * HOW TO USE IN CLASS:
 * 1. Uncomment ONE demo in main() to focus lesson
 * 2. Build with: cli-build-project.ps1 -ProjectDir Graphics_Display
 * 3. Run in SimulIDE 0.4.15: tools/simulide/Simulator0415.simu
 * 4. Observe dual-controller split and coordinate mapping
 * 5. Modify coordinates/parameters to see effects
 * 6. Use serial debug output (if enabled) to trace execution
 *
 * SIMULATOR NOTES:
 * - SimulIDE 0.4.15 required (newer versions have display issues)
 * - Timing delays may differ from hardware
 * - Use for visual verification and teaching
 * - Test on real hardware for production code
 *
 * =============================================================================
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "../../shared_libs/_glcd.h"
#include "../../shared_libs/_init.h"

/* =============================================================================
 * HARDWARE CONFIGURATION
 * =============================================================================
 */
#define GLCD_ROWS 64  // Screen height (x-axis: 0..63)
#define GLCD_COLS 128 // Screen width  (y-axis: 0..127)
#define TEXT_ROWS 8   // Text grid: 8 rows  (each row = 8 pixels high)
#define TEXT_COLS 20  // Text grid: 20 cols (each char = 6 pixels wide)

/* =============================================================================
 * DEMONSTRATION STRINGS (STORED IN FLASH MEMORY)
 * =============================================================================
 * Using PROGMEM to save SRAM (ATmega128 has only 4KB SRAM)
 * Access with lcd_string_P() or copy to buffer with strcpy_P()
 */

// Common symbols for display testing
const char STR_SYMBOLS[] PROGMEM = "#$%&'()*+,-./0123456";
const char STR_SINGLE_CHAR[] PROGMEM = "O";

// Demo headers and labels
const char STR_HEADER_BAR[] PROGMEM = "====================";
const char STR_TITLE[] PROGMEM = "   ATmega128 GLCD   ";
const char STR_COURSE[] PROGMEM = "SOC3050 Graphics";

// Demo-specific strings
const char STR_DEMO1[] PROGMEM = "Demo 1: Text Header";
const char STR_DEMO2[] PROGMEM = "Demo 2: Pixel Drawing";
const char STR_DEMO3[] PROGMEM = "Demo 3: Page Addressing";
const char STR_DEMO4[] PROGMEM = "Demo 4: Lines";
const char STR_DEMO5[] PROGMEM = "Demo 5: Rectangles";
const char STR_DEMO6[] PROGMEM = "Demo 6: Circles";
const char STR_DEMO7[] PROGMEM = "Demo 7: Text Pages";
const char STR_DEMO8[] PROGMEM = "Demo 8: Radiating Lines";
const char STR_DEMO9[] PROGMEM = "Demo 9: Nested Shapes";
const char STR_DEMO10[] PROGMEM = "Demo 10: Grid Pattern";

const char STR_LEFT_PANEL[] PROGMEM = "Left (CS1: y 0..63)";
const char STR_RIGHT_PANEL[] PROGMEM = "Right (CS2: y 64..127)";
const char STR_CS_SPLIT[] PROGMEM = "CS1: y 0..63  CS2: y 64..127";
const char STR_CHECKER[] PROGMEM = "Checker Pattern";

// Helper to print PROGMEM strings
// Note: lcd_string expects RAM string, so we copy to buffer first
static void lcd_string_P(byte row, byte col, const char *progmem_str)
{
    char buffer[21]; // Max 20 chars + null terminator
    strncpy_P(buffer, progmem_str, 20);
    buffer[20] = '\0';
    lcd_string(row, col, buffer);
}

/* =============================================================================
 * DEMO 1: TEXT HEADER - Basic Text Display
 * =============================================================================
 * PURPOSE: Introduce GLCD text output and lcd_string() function
 *
 * CONCEPTS:
 * - Text grid: 8 rows × 20 columns
 * - lcd_string(row, col, str): row 0..7, col 0..19
 * - Each character: 6 pixels wide (5x7 font + 1 spacing)
 * - Each row: 8 pixels tall
 *
 * TEACHING FOCUS:
 * - How to position text on GLCD
 * - Text coordinate system vs pixel coordinate system
 * - Character spacing and alignment
 */
static void demo_01_text_header(void)
{
    lcd_clear();

    // Display header banner
    lcd_string_P(0, 0, STR_HEADER_BAR); // "===================="
    lcd_string_P(1, 0, STR_TITLE);      // "   ATmega128 GLCD   "
    lcd_string_P(2, 0, STR_SYMBOLS);    // "#$%&'()*+,-./0123456"
    lcd_string_P(6, 0, STR_COURSE);     // "SOC3050 Graphics"
    lcd_string_P(7, 0, STR_DEMO1);      // "Demo 1: Text Header"

    _delay_ms(100);
}

/* =============================================================================
 * DEMO 2: SINGLE PIXEL DRAWING - Pixel-Level Control
 * =============================================================================
 * PURPOSE: Learn GLCD_Dot() for individual pixel manipulation
 *
 * CONCEPTS:
 * - GLCD_Dot(x, y): x=row (0..63), y=column (0..127)
 * - Checker pattern: ((x>>3) + (y>>3)) & 1
 * - Software buffer mirrors screen state
 *
 * TEACHING FOCUS:
 * - Coordinate system orientation (x=rows vertical, y=cols horizontal)
 * - Bit shifting for pattern generation
 * - Performance: drawing 8192 pixels (64×128)
 */
static void demo_02_pixel_drawing(void)
{
    lcd_clear();
    lcd_string_P(0, 0, STR_DEMO2);   // "Demo 2: Pixel Drawing"
    lcd_string_P(1, 0, STR_CHECKER); // "Checker Pattern"

    _delay_ms(500); // Let title display before drawing

    // Clear software buffer
    ScreenBuffer_clear();

    // Draw 8×8 pixel checker pattern
    for (unsigned char x = 0; x < GLCD_ROWS; x++)
    {
        for (unsigned char y = 0; y < GLCD_COLS; y++)
        {
            // Create checkerboard: divide screen into 8×8 blocks
            if (((x >> 3) + (y >> 3)) & 1)
                GLCD_Dot(x, y);
        }
    }

    _delay_ms(100);
}

/* =============================================================================
 * DEMO 3: PAGE ADDRESSING - Memory Organization
 * =============================================================================
 * PURPOSE: Understand page-based memory layout and dual controllers
 *
 * CONCEPTS:
 * - 8 pages (0..7), each page = 8 pixels tall, 128 columns wide
 * - Page 0 = rows 0-7,  Page 1 = rows 8-15, ..., Page 7 = rows 56-63
 * - CS1 (left controller):  y=0..63   (columns 0-63)
 * - CS2 (right controller): y=64..127 (columns 64-127)
 * - Writing 0xFF byte: lights up 8 vertical pixels in that column
 *
 * TEACHING FOCUS:
 * - Why GLCD uses pages (optimization for vertical byte writes)
 * - Controller split at y=64 midpoint
 * - Direct low-level drawing with GLCD_Axis_xy() and data()
 */
static void demo_03_page_addressing(void)
{
    lcd_clear();
    lcd_string_P(0, 0, STR_DEMO3);        // "Demo 3: Page Addressing"
    lcd_string_P(1, 0, STR_LEFT_PANEL);   // "Left (CS1: y 0..63)"
    lcd_string_P(2, 10, STR_RIGHT_PANEL); // "Right (CS2: y 64..127)"

    _delay_ms(500);

    // Draw vertical bar on LEFT controller (CS1) at y=10
    for (unsigned char page = 2; page < 7; page++)
    {
        GLCD_Axis_xy(page, 10); // Position at (page, column)
        datal(0xFF);            // Write full byte (8 pixels)
    }

    // Draw vertical bar on RIGHT controller (CS2) at y=90
    for (unsigned char page = 2; page < 7; page++)
    {
        GLCD_Axis_xy(page, 90);
        datar(0xFF); // Use right controller
    }

    // Annotate controller split
    lcd_string_P(7, 0, STR_CS_SPLIT); // "CS1: y 0..63  CS2: y 64..127"

    _delay_ms(100);
}

/* =============================================================================
 * DEMO 4: LINE DRAWING - Bresenham's Algorithm
 * =============================================================================
 * PURPOSE: Learn line drawing primitives and coordinate mapping
 *
 * CONCEPTS:
 * - GLCD_Line(x1, y1, x2, y2): connects two points
 * - Bresenham's line algorithm (integer-only, fast)
 * - Diagonal, horizontal, and vertical lines
 *
 * TEACHING FOCUS:
 * - Line drawing algorithm basics
 * - Coordinate order (x1,y1) to (x2,y2)
 * - Visual verification of endpoint accuracy
 */
static void demo_04_lines(void)
{
    lcd_clear();
    lcd_string_P(0, 0, STR_DEMO4); // "Demo 4: Lines"

    _delay_ms(300);

    // Diagonal line (top-left to middle)
    GLCD_Line(10, 5, 50, 30);

    // Diagonal line (left to right)
    GLCD_Line(5, 120, 40, 70);

    // Horizontal line
    GLCD_Line(20, 40, 20, 90);

    // Vertical line
    GLCD_Line(15, 50, 45, 50);

    _delay_ms(100);
}

/* =============================================================================
 * DEMO 5: RECTANGLE DRAWING - Filled and Outline
 * =============================================================================
 * PURPOSE: Practice rectangle primitives and boundary calculations
 *
 * CONCEPTS:
 * - GLCD_Rectangle(x1, y1, x2, y2): outline rectangle
 * - (x1,y1) = top-left corner, (x2,y2) = bottom-right corner
 * - Boundary clipping (ensure coordinates within 0..63, 0..127)
 *
 * TEACHING FOCUS:
 * - Rectangle coordinate system
 * - Nested rectangles with insets
 * - Calculating inset amounts
 */
static void demo_05_rectangles(void)
{
    lcd_clear();
    lcd_string_P(0, 0, STR_DEMO5); // "Demo 5: Rectangles"

    _delay_ms(300);

    // Single rectangle
    GLCD_Rectangle(10, 20, 30, 60);

    // Nested rectangles with increasing insets
    unsigned char inset = 0;
    while (15 + inset < GLCD_COLS - 1 - inset && 35 + inset < GLCD_ROWS - 1 - inset)
    {
        unsigned char x1 = 35 + inset;
        unsigned char y1 = 15 + inset;
        unsigned char x2 = (GLCD_ROWS - 1) - inset;
        unsigned char y2 = (GLCD_COLS - 1) - inset;
        GLCD_Rectangle(x1, y1, x2, y2);
        inset += 3;
    }

    _delay_ms(100);
}

/* =============================================================================
 * DEMO 6: CIRCLE DRAWING - Midpoint Circle Algorithm
 * =============================================================================
 * PURPOSE: Learn circle drawing and radius calculations
 *
 * CONCEPTS:
 * - GLCD_Circle(x, y, r): center at (x,y), radius r
 * - Midpoint circle algorithm (8-way symmetry)
 * - Concentric circles
 *
 * TEACHING FOCUS:
 * - Circle algorithm (symmetry optimization)
 * - Radius must fit within screen boundaries
 * - Center point calculation
 */
static void demo_06_circles(void)
{
    lcd_clear();
    lcd_string_P(0, 0, STR_DEMO6); // "Demo 6: Circles"

    _delay_ms(300);

    const unsigned char cx = GLCD_ROWS / 2; // Center x = 32
    const unsigned char cy = GLCD_COLS / 2; // Center y = 64

    // Concentric circles with increasing radii
    for (unsigned char r = 5; r <= 25; r += 5)
    {
        GLCD_Circle(cx, cy, r);
    }

    _delay_ms(100);
}

/* =============================================================================
 * DEMO 7: TEXT PAGES - Left/Right Controller Text
 * =============================================================================
 * PURPOSE: Demonstrate text positioning across dual controllers
 *
 * CONCEPTS:
 * - Text grid: 20 chars wide (10 per controller)
 * - lcd_string(row, col): col 0..9 → left controller, col 10..19 → right
 * - Character width: 6 pixels (5x7 font + 1 spacing)
 *
 * TEACHING FOCUS:
 * - Text wrapping behavior
 * - Symbol display from PROGMEM
 * - Left/right panel coordination
 */
static void demo_07_text_pages(void)
{
    lcd_clear();
    lcd_string_P(0, 0, STR_DEMO7);        // "Demo 7: Text Pages"
    lcd_string_P(1, 0, STR_LEFT_PANEL);   // "Left (CS1: y 0..63)"
    lcd_string_P(2, 10, STR_RIGHT_PANEL); // "Right (CS2: y 64..127)"

    _delay_ms(300);

    // Left panel text (columns 0..9)
    char buf[21];
    strcpy_P(buf, STR_SYMBOLS);
    lcd_string(3, 0, "0 1 2 3 4 5 6 7 8 9");
    lcd_string(4, 0, buf); // Symbols

    // Right panel text (columns 10..19)
    lcd_string(3, 10, "Right Side Text");
    lcd_string(4, 10, "Columns 10-19");

    // Single character demo
    strcpy_P(buf, STR_SINGLE_CHAR); // "O"
    lcd_string(6, 5, buf);

    // Number display
    lcd_xy(6, 12);
    GLCD_4DigitDecimal(1234);

    _delay_ms(100);
}

/* =============================================================================
 * DEMO 8: RADIATING LINES - Center-Out Patterns
 * =============================================================================
 * PURPOSE: Create complex patterns using multiple line primitives
 *
 * CONCEPTS:
 * - Drawing from center point to edges
 * - Loop-based pattern generation
 * - Coordinate calculations for edges
 *
 * TEACHING FOCUS:
 * - Pattern generation algorithms
 * - Combining primitives for complex graphics
 * - Performance of multiple draw operations
 */
static void demo_08_radiating_lines(void)
{
    const unsigned char cx = GLCD_ROWS / 2; // Center x = 32
    const unsigned char cy = GLCD_COLS / 2; // Center y = 64

    lcd_clear();
    lcd_string_P(0, 0, STR_DEMO8); // "Demo 8: Radiating Lines"

    _delay_ms(300);

    // Radiate to top and bottom edges
    for (unsigned char y = 0; y < GLCD_COLS; y += 8)
    {
        GLCD_Line(cx, cy, 0, y);             // Top edge (x=0)
        GLCD_Line(cx, cy, GLCD_ROWS - 1, y); // Bottom edge (x=63)
    }

    _delay_ms(100);
}

/* =============================================================================
 * DEMO 9: NESTED SHAPES - Concentric Patterns
 * =============================================================================
 * PURPOSE: Create nested shapes with calculated spacing
 *
 * CONCEPTS:
 * - Incremental insets for nested shapes
 * - Boundary checking to prevent overdraw
 * - Visual depth perception with nesting
 *
 * TEACHING FOCUS:
 * - Loop termination conditions
 * - Spacing calculations
 * - Combining shapes for visual effects
 */
static void demo_09_nested_shapes(void)
{
    lcd_clear();
    lcd_string_P(0, 0, STR_DEMO9); // "Demo 9: Nested Shapes"

    _delay_ms(300);

    // Nested rectangles
    unsigned char inset = 0;
    while (20 + inset < GLCD_COLS - 1 - inset && 10 + inset < GLCD_ROWS - 1 - inset)
    {
        unsigned char x1 = 10 + inset;
        unsigned char y1 = 20 + inset;
        unsigned char x2 = (GLCD_ROWS - 1) - inset;
        unsigned char y2 = (GLCD_COLS - 1) - inset;
        GLCD_Rectangle(x1, y1, x2, y2);
        inset += 4;
    }

    _delay_ms(100);
}

/* =============================================================================
 * DEMO 10: GRID PATTERN - Alignment and Spacing
 * =============================================================================
 * PURPOSE: Create regular grid patterns for measurement and alignment
 *
 * CONCEPTS:
 * - Regular spacing calculations
 * - Horizontal and vertical line grids
 * - Visual measurement tool
 *
 * TEACHING FOCUS:
 * - Loop-based grid generation
 * - Spacing consistency
 * - Practical use of grids (alignment, measurement)
 */
static void demo_10_grid(void)
{
    lcd_clear();
    lcd_string_P(0, 0, STR_DEMO10); // "Demo 10: Grid Pattern"

    _delay_ms(300);

    // Vertical lines every 8 columns
    for (unsigned char y = 0; y < GLCD_COLS; y += 8)
    {
        GLCD_Line(10, y, GLCD_ROWS - 1, y);
    }

    // Horizontal lines every 8 rows
    for (unsigned char x = 10; x < GLCD_ROWS; x += 8)
    {
        GLCD_Line(x, 0, x, GLCD_COLS - 1);
    }

    _delay_ms(100);
}

/* =============================================================================
 * RUN ALL DEMOS - Sequential Demonstration
 * =============================================================================
 * PURPOSE: Automatically cycle through all demos for overview presentation
 *
 * USAGE: Use for class introduction or end-of-unit review
 *        Each demo displays for 1-2 seconds before advancing
 */
static void demo_all_sequential(void)
{
    demo_01_text_header();
    _delay_ms(1500);

    demo_02_pixel_drawing();
    _delay_ms(2000);

    demo_03_page_addressing();
    _delay_ms(1500);

    demo_04_lines();
    _delay_ms(1500);

    demo_05_rectangles();
    _delay_ms(1500);

    demo_06_circles();
    _delay_ms(1500);

    demo_07_text_pages();
    _delay_ms(2000);

    demo_08_radiating_lines();
    _delay_ms(1500);

    demo_09_nested_shapes();
    _delay_ms(1500);

    demo_10_grid();
    _delay_ms(2000);
}

/* =============================================================================
 * MAIN FUNCTION - Demo Selection
 * =============================================================================
 *
 * INSTRUCTIONS FOR INSTRUCTORS:
 * 1. Uncomment EXACTLY ONE demo function to run that specific lesson
 * 2. Comment out all others (or use demo_all_sequential for overview)
 * 3. Build: cli-build-project.ps1 -ProjectDir Graphics_Display
 * 4. Run in SimulIDE 0.4.15 for visual demonstration
 *
 * TEACHING SEQUENCE (Recommended 4-week progression):
 *
 * Week 1: Text and Basics
 *   - demo_01_text_header()       : Text positioning and display
 *   - demo_02_pixel_drawing()     : Individual pixel control
 *   - demo_03_page_addressing()   : Memory organization
 *
 * Week 2: Drawing Primitives
 *   - demo_04_lines()             : Line drawing algorithm
 *   - demo_05_rectangles()        : Rectangle primitives
 *   - demo_06_circles()           : Circle algorithm
 *
 * Week 3: Text and Layout
 *   - demo_07_text_pages()        : Text across controllers
 *   - Review coordinate systems
 *
 * Week 4: Advanced Patterns
 *   - demo_08_radiating_lines()   : Pattern generation
 *   - demo_09_nested_shapes()     : Complex compositions
 *   - demo_10_grid()              : Alignment tools
 *
 * =============================================================================
 */
int main(void)
{
    // Initialize system and GLCD hardware
    init_devices();

    // =========================================================================
    // SELECT ONE DEMO TO RUN (uncomment exactly ONE line)
    // =========================================================================

    demo_01_text_header(); // Week 1: Text basics
    // demo_02_pixel_drawing();     // Week 1: Pixels and checker pattern
    // demo_03_page_addressing();   // Week 1: Memory layout
    // demo_04_lines();             // Week 2: Line drawing
    // demo_05_rectangles();        // Week 2: Rectangles
    // demo_06_circles();           // Week 2: Circles
    // demo_07_text_pages();        // Week 3: Text layout
    // demo_08_radiating_lines();   // Week 4: Patterns
    // demo_09_nested_shapes();     // Week 4: Nested graphics
    // demo_10_grid();              // Week 4: Grid alignment
    // demo_all_sequential();       // Run all demos (overview)

    // =========================================================================
    // Main loop - keep display static
    // =========================================================================
    while (1)
    {
        // Display persists; no updates needed
        // For animation, add code here with _delay_ms() between frames
    }

    return 0;
}

/* =============================================================================
 * END OF GRAPHICS_DISPLAY EDUCATIONAL MODULE
 * =============================================================================
 * For detailed documentation, see:
 * - GLCD_QUICK_REFERENCE.md      : Register and API reference
 * - GLCD_FLOW_DIAGRAMS.md         : Visual diagrams and flowcharts
 * - DOCUMENTATION_SUMMARY.md      : Instructor teaching guide
 * =============================================================================
 */