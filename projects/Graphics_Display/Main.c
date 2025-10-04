
#include "config.h"

/*=========================================================================*/
/*                             BASIC TESTS                                */
/*=========================================================================*/
#ifdef BASIC_TEST_MODE

void demo_01_basic_lcd_text(void)
{
    lcd_clear();
    lcd_string(0, 0, "=== BASIC MODE ===");
    lcd_string(1, 0, "LCD Text Display");
    _delay_ms(2000);

    // Test basic text positioning
    lcd_clear();
    lcd_string(0, 0, "Line 0: Hello");
    lcd_string(1, 0, "Line 1: World");
    lcd_string(2, 0, "Line 2: 12345");
    lcd_string(3, 0, "Line 3: !@#$%");
    lcd_string(4, 0, "Line 4: ABCDE");
    _delay_ms(3000);
}

void demo_02_basic_primitives(void)
{
    lcd_clear();
    lcd_string(0, 0, "Basic Graphics");
    _delay_ms(1500);

    // Test dots
    lcd_clear();
    lcd_string(0, 0, "Drawing Dots...");
    for (int i = 0; i < 40; i += 5)
    {
        GLCD_Dot(20 + i, 30);
        _delay_ms(200);
    }
    _delay_ms(1500);

    // Test lines
    lcd_clear();
    lcd_string(0, 0, "Drawing Lines...");
    GLCD_Line(10, 20, 50, 20); // Horizontal
    _delay_ms(500);
    GLCD_Line(10, 25, 50, 40); // Diagonal
    _delay_ms(500);
    GLCD_Line(30, 20, 30, 50); // Vertical
    _delay_ms(2000);

    // Test rectangles
    lcd_clear();
    lcd_string(0, 0, "Basic Rectangles");
    GLCD_Rectangle(10, 20, 30, 40);
    _delay_ms(1000);
    GLCD_Rectangle(35, 25, 55, 45);
    _delay_ms(2000);

    // Test circles
    lcd_clear();
    lcd_string(0, 0, "Basic Circles");
    GLCD_Circle(25, 35, 10);
    _delay_ms(1000);
    GLCD_Circle(45, 35, 15);
    _delay_ms(2000);
}

void demo_basic_test_suite(void)
{
    init_devices();
    lcd_clear();

    lcd_string(2, 0, "BASIC TEST MODE");
    lcd_string(3, 0, "Starting...");
    _delay_ms(2000);

    demo_01_basic_lcd_text();
    demo_02_basic_primitives();

    lcd_clear();
    lcd_string(2, 0, "BASIC TESTS");
    lcd_string(3, 0, "COMPLETED!");
    _delay_ms(3000);
}

#endif

/*=========================================================================*/
/*                         COMPREHENSIVE TESTS                            */
/*=========================================================================*/
#ifdef GRAPHICS_COMPREHENSIVE_TEST

void demo_03_comprehensive_text(void)
{
    lcd_clear();
    lcd_string(0, 0, "=COMPREHENSIVE=");
    lcd_string(1, 0, "Text Display Test");
    _delay_ms(2000);

    // Test character sets
    lcd_clear();
    lcd_string(0, 0, "UPPERCASE: ABCDEF");
    lcd_string(1, 0, "lowercase: abcdef");
    lcd_string(2, 0, "Numbers: 0123456");
    lcd_string(3, 0, "Symbols: !@#$%^&");
    lcd_string(4, 0, "More: ()[]{}|\\");
    _delay_ms(4000);
}

void demo_04_comprehensive_shapes(void)
{
    lcd_clear();
    lcd_string(0, 0, "Shape Variations");
    _delay_ms(1500);

    // Multiple circles
    lcd_clear();
    lcd_string(0, 0, "Circle Patterns");
    for (int r = 5; r <= 20; r += 5)
    {
        GLCD_Circle(30, 40, r);
        _delay_ms(800);
    }
    _delay_ms(1500);

    // Rectangle patterns
    lcd_clear();
    lcd_string(0, 0, "Rectangle Grid");
    for (int i = 0; i < 4; i++)
    {
        GLCD_Rectangle(10 + i * 12, 20, 20 + i * 12, 35);
        _delay_ms(500);
    }
    _delay_ms(2000);

    // Line patterns
    lcd_clear();
    lcd_string(0, 0, "Line Patterns");
    for (int i = 0; i < 6; i++)
    {
        GLCD_Line(10, 20 + i * 5, 50, 20 + i * 5);
        _delay_ms(300);
    }
    _delay_ms(2000);
}

void demo_05_comprehensive_numbers(void)
{
    lcd_clear();
    lcd_string(0, 0, "Number Functions");
    _delay_ms(1500);

    // Test number display functions
    lcd_clear();
    lcd_string(0, 0, "1-digit: ");
    GLCD_1DigitDecimal(7, 0);

    lcd_string(1, 0, "2-digit: ");
    lcd_xy(1, 9);
    GLCD_2DigitDecimal(42);

    lcd_string(2, 0, "3-digit: ");
    lcd_xy(2, 9);
    GLCD_3DigitDecimal(123);

    lcd_string(3, 0, "4-digit: ");
    lcd_xy(3, 9);
    GLCD_4DigitDecimal(9876);

    _delay_ms(4000);
}

void demo_comprehensive_test_suite(void)
{
    init_devices();
    lcd_clear();

    lcd_string(1, 0, "COMPREHENSIVE");
    lcd_string(2, 0, "TEST MODE");
    lcd_string(3, 0, "Starting...");
    _delay_ms(2000);

    demo_03_comprehensive_text();
    demo_04_comprehensive_shapes();
    demo_05_comprehensive_numbers();

    lcd_clear();
    lcd_string(1, 0, "COMPREHENSIVE");
    lcd_string(2, 0, "TESTS COMPLETED!");
    _delay_ms(3000);
}

#endif

#define GRAPHICS_FULL_TEST
/* Full Graphics Library Test */
#ifdef GRAPHICS_FULL_TEST

void demo_06_basic_functions(void)
{
    lcd_clear();
    lcd_string(0, 0, "Basic Functions Test");
    _delay_ms(1500);

    // Test dots
    for (int i = 0; i < 30; i += 3)
    {
        GLCD_Dot(20 + i, 30);
        GLCD_Dot(20 + i, 35);
        _delay_ms(100);
    }

    // Test lines
    GLCD_Line(10, 45, 50, 45);
    GLCD_Line(10, 50, 50, 60);
    _delay_ms(2000);
}

void demo_07_drawing_modes(void)
{
    lcd_clear();
    lcd_string(0, 0, "Drawing Modes Test");
    _delay_ms(1500);

    // Draw base pattern
    GLCD_SetDrawMode(GLCD_MODE_SET);
    GLCD_Rectangle_Filled(20, 30, 40, 50);
    _delay_ms(1000);

    // Test XOR mode
    GLCD_SetDrawMode(GLCD_MODE_XOR);
    GLCD_Circle(30, 40, 15);
    _delay_ms(1000);

    // Test OR mode
    GLCD_SetDrawMode(GLCD_MODE_OR);
    GLCD_Rectangle(10, 20, 50, 60);
    _delay_ms(2000);

    // Reset to normal mode
    GLCD_SetDrawMode(GLCD_MODE_SET);
}

void demo_08_filled_shapes(void)
{
    lcd_clear();
    lcd_string(0, 0, "Filled Shapes Test");
    _delay_ms(1500);

    // Filled rectangle
    GLCD_Rectangle_Filled(10, 25, 25, 40);
    _delay_ms(800);

    // Filled circle
    GLCD_Circle_Filled(40, 35, 10);
    _delay_ms(800);

    // Triangle
    GLCD_Triangle_Filled(15, 50, 30, 60, 45, 50);
    _delay_ms(2000);
}

void demo_09_advanced_shapes(void)
{
    lcd_clear();
    lcd_string(0, 0, "Advanced Shapes");
    _delay_ms(1500);

    // Ellipse
    GLCD_Ellipse(30, 40, 20, 12);
    _delay_ms(1000);

    // Polygon (hexagon)
    unsigned char hex_vertices[6][2] = {
        {20, 20}, {35, 15}, {50, 20}, {50, 35}, {35, 40}, {20, 35}};
    GLCD_Polygon(hex_vertices, 6);
    _delay_ms(2000);
}

void demo_10_patterns_bitmaps(void)
{
    lcd_clear();
    lcd_string(0, 0, "Patterns & Bitmaps");
    _delay_ms(1500);

    // Pattern fill
    GLCD_Pattern_Fill(10, 25, 30, 40, 0xAA); // Checkerboard pattern
    _delay_ms(1000);

    // Simple 8x8 bitmap (smiley face)
    unsigned char smiley[8] = {
        0x3C, 0x42, 0xA5, 0x81, 0xA5, 0x99, 0x42, 0x3C};
    GLCD_Bitmap_8x8(40, 30, smiley);
    _delay_ms(2000);
}

void demo_11_large_text(void)
{
    lcd_clear();
    lcd_string(0, 0, "Large Numbers Test");
    _delay_ms(1500);

    // Display large numbers
    for (int i = 0; i <= 9; i++)
    {
        lcd_clear();
        lcd_string(0, 0, "Large Number:");
        GLCD_Large_Number(25, 30, i);
        _delay_ms(500);
    }
    _delay_ms(1000);
}

void demo_12_ui_elements(void)
{
    lcd_clear();
    lcd_string(0, 0, "UI Elements Test");
    _delay_ms(1500);

    // Progress bar animation
    for (int progress = 0; progress <= 100; progress += 10)
    {
        lcd_clear();
        lcd_string(0, 0, "Progress Bar:");
        GLCD_Progress_Bar(10, 30, 40, 8, progress);

        // Show percentage
        char percent_str[10];
        sprintf(percent_str, "%d%%", progress);
        lcd_string(5, 0, percent_str);
        _delay_ms(300);
    }
    _delay_ms(1000);
}

void demo_13_charts(void)
{
    lcd_clear();
    lcd_string(0, 0, "Charts Test");
    _delay_ms(1500);

    // Bar chart data
    unsigned char bar_data[6] = {10, 25, 15, 35, 20, 30};
    GLCD_Bar_Chart(5, 20, bar_data, 6, 40);
    _delay_ms(2000);

    lcd_clear();
    lcd_string(0, 0, "Line Graph Test");
    _delay_ms(1000);

    // Line graph data
    unsigned char line_data[8] = {5, 15, 25, 20, 30, 10, 35, 25};
    GLCD_Line_Graph(5, 20, line_data, 8, 40);
    _delay_ms(2000);
}

void demo_14_screen_effects(void)
{
    lcd_clear();
    lcd_string(0, 0, "Screen Effects");
    _delay_ms(1500);

    // Draw something
    GLCD_Circle(30, 40, 15);
    GLCD_Rectangle(10, 20, 50, 60);
    _delay_ms(1000);

    // Invert screen
    lcd_string(6, 0, "Inverting...");
    _delay_ms(500);
    GLCD_Invert_Screen();
    _delay_ms(1500);

    // Invert back
    GLCD_Invert_Screen();
    _delay_ms(1000);
}

void demo_15_animation_demo(void)
{
    lcd_clear();
    lcd_string(0, 0, "Animation Demo");
    _delay_ms(1500);

    // Simple bouncing ball animation
    int x = 10, y = 30;
    int dx = 2, dy = 1;

    for (int frame = 0; frame < 100; frame++)
    {
        // Clear previous position
        GLCD_SetDrawMode(GLCD_MODE_XOR);
        GLCD_Circle_Filled(x, y, 3);

        // Update position
        x += dx;
        y += dy;

        // Bounce off walls
        if (x >= 60 || x <= 3)
            dx = -dx;
        if (y >= 60 || y <= 25)
            dy = -dy;

        // Draw new position
        GLCD_Circle_Filled(x, y, 3);

        _delay_ms(100);
    }

    GLCD_SetDrawMode(GLCD_MODE_SET); // Reset mode
    _delay_ms(1000);
}

void demo_advanced_test_suite(void)
{
    init_devices();
    lcd_clear();

    // Welcome message
    lcd_string(1, 0, "ADVANCED GRAPHICS");
    lcd_string(2, 0, "LIBRARY TEST");
    lcd_string(4, 0, "Starting in 3...");
    _delay_ms(1000);
    lcd_string(4, 0, "Starting in 2...");
    _delay_ms(1000);
    lcd_string(4, 0, "Starting in 1...");
    _delay_ms(1000);

    // Run all tests
    demo_06_basic_functions();
    demo_07_drawing_modes();
    demo_08_filled_shapes();
    demo_09_advanced_shapes();
    demo_10_patterns_bitmaps();
    demo_11_large_text();
    demo_12_ui_elements();
    demo_13_charts();
    demo_14_screen_effects();
    demo_15_animation_demo();

    // Final message
    lcd_clear();
    lcd_string(1, 0, "ALL GRAPHICS");
    lcd_string(2, 0, "TESTS COMPLETED!");
    lcd_string(4, 0, "Library Features:");
    lcd_string(5, 0, "- Drawing Modes");
    lcd_string(6, 0, "- Filled Shapes");
    lcd_string(7, 0, "- Advanced UI");

    while (1)
    {
        // Subtle animation - blinking cursor
        GLCD_Dot(120, 56);
        _delay_ms(500);
        GLCD_SetDrawMode(GLCD_MODE_XOR);
        GLCD_Dot(120, 56);
        GLCD_SetDrawMode(GLCD_MODE_SET);
        _delay_ms(500);
    }
}
#endif

// Main entry point
int main(void)
{
    init_devices();

    // Show main menu
    lcd_clear();
    lcd_string(0, 0, "GRAPHICS TEST SUITE");
    lcd_string(1, 0, "==================");
    lcd_string(3, 0, "Running ALL Modes:");
    lcd_string(4, 0, "1. Basic");
    lcd_string(5, 0, "2. Comprehensive");
    lcd_string(6, 0, "3. Advanced/Full");
    _delay_ms(4000);

// Run all enabled test modes
#ifdef BASIC_TEST_MODE
    demo_basic_test_suite();
#endif

#ifdef GRAPHICS_COMPREHENSIVE_TEST
    demo_comprehensive_test_suite();
#endif

#ifdef GRAPHICS_FULL_TEST
    demo_advanced_test_suite();
#endif // Final completion screen
    lcd_clear();
    lcd_string(0, 0, "ALL TESTS COMPLETE");
    lcd_string(1, 0, "==================");
    lcd_string(3, 0, "Library Features:");
    lcd_string(4, 0, "- Basic Graphics");
    lcd_string(5, 0, "- Drawing Modes");
    lcd_string(6, 0, "- Filled Shapes");
    lcd_string(7, 0, "- Advanced UI");

    // Blinking cursor animation
    while (1)
    {
        GLCD_Dot(120, 56);
        _delay_ms(500);
        GLCD_SetDrawMode(GLCD_MODE_XOR);
        GLCD_Dot(120, 56);
        GLCD_SetDrawMode(GLCD_MODE_SET);
        _delay_ms(500);
    }

    return 0;
}