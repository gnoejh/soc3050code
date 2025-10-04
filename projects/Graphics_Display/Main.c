
#include "config.h"

#define GRAPHICS_COMPREHENSIVE_TEST
/* Comprehensive Graphics Test */
#ifdef GRAPHICS_COMPREHENSIVE_TEST

void test_basic_text(void)
{
    lcd_clear();
    lcd_string(0, 0, "GLCD Test Suite");
    lcd_string(1, 0, "Basic Text Test");
    lcd_string(3, 0, "Line 1: ABCDEF");
    lcd_string(4, 0, "Line 2: 123456");
    lcd_string(5, 0, "Special: !@#$%");
    _delay_ms(3000);
}

void test_individual_dots(void)
{
    lcd_clear();
    lcd_string(0, 0, "Dot Pattern Test");
    _delay_ms(1000);

    // Draw a grid of dots
    for (int x = 5; x < 60; x += 10)
    {
        for (int y = 20; y < 120; y += 10)
        {
            GLCD_Dot(x, y);
            _delay_ms(50);
        }
    }
    _delay_ms(3000);
}

void test_lines(void)
{
    lcd_clear();
    lcd_string(0, 0, "Line Drawing Test");
    _delay_ms(1000);

    // Horizontal lines
    GLCD_Line(10, 20, 50, 20);
    _delay_ms(500);
    GLCD_Line(10, 30, 50, 30);
    _delay_ms(500);

    // Vertical lines
    GLCD_Line(60, 20, 60, 50);
    _delay_ms(500);
    GLCD_Line(70, 20, 70, 50);
    _delay_ms(500);

    // Diagonal lines
    GLCD_Line(10, 60, 50, 100);
    _delay_ms(500);
    GLCD_Line(50, 60, 10, 100);
    _delay_ms(500);

    _delay_ms(3000);
}

void test_rectangles(void)
{
    lcd_clear();
    lcd_string(0, 0, "Rectangle Test");
    _delay_ms(1000);

    // Small rectangle
    GLCD_Rectangle(10, 20, 30, 40);
    _delay_ms(1000);

    // Medium rectangle
    GLCD_Rectangle(40, 30, 60, 60);
    _delay_ms(1000);

    // Large rectangle
    GLCD_Rectangle(5, 70, 55, 120);
    _delay_ms(1000);

    _delay_ms(3000);
}

void test_circles(void)
{
    lcd_clear();
    lcd_string(0, 0, "Circle Test");
    _delay_ms(1000);

    // Small circles
    GLCD_Circle(20, 30, 5);
    _delay_ms(500);
    GLCD_Circle(40, 30, 8);
    _delay_ms(500);
    GLCD_Circle(20, 50, 12);
    _delay_ms(500);
    GLCD_Circle(40, 50, 15);
    _delay_ms(500);

    _delay_ms(3000);
}

void test_numbers(void)
{
    lcd_clear();
    lcd_string(0, 0, "Number Display Test");
    _delay_ms(1000);

    // Test different number functions
    lcd_xy(2, 0);
    lcd_string(2, 0, "1-digit: ");
    GLCD_1DigitDecimal(7, 0);

    lcd_string(3, 0, "2-digit: ");
    lcd_xy(3, 9);
    GLCD_2DigitDecimal(42);

    lcd_string(4, 0, "3-digit: ");
    lcd_xy(4, 9);
    GLCD_3DigitDecimal(123);

    lcd_string(5, 0, "4-digit: ");
    lcd_xy(5, 9);
    GLCD_4DigitDecimal(9876);

    _delay_ms(4000);
}

void test_animation_simple(void)
{
    lcd_clear();
    lcd_string(0, 0, "Simple Animation");
    _delay_ms(1000);

    // Moving dot across screen
    for (int y = 20; y < 100; y += 2)
    {
        GLCD_Dot(30, y);
        _delay_ms(100);
        // Clear by redrawing (if your implementation supports XOR)
        GLCD_Dot(30, y);
    }
    _delay_ms(2000);
}

void test_complex_pattern(void)
{
    lcd_clear();
    lcd_string(0, 0, "Complex Pattern");
    _delay_ms(1000);

    // Draw a pattern of shapes
    GLCD_Rectangle(10, 20, 25, 35);
    GLCD_Circle(35, 27, 8);
    GLCD_Line(45, 20, 55, 35);
    GLCD_Line(45, 35, 55, 20);

    // Add some dots around
    for (int i = 0; i < 10; i++)
    {
        GLCD_Dot(15 + i * 3, 45);
        _delay_ms(200);
    }

    _delay_ms(3000);
}

void main_graphics_comprehensive_test(void)
{
    init_devices(); // initialize LCD
    lcd_clear();

    // Welcome message
    lcd_string(2, 0, "GLCD Test Suite");
    lcd_string(3, 0, "Starting Tests...");
    _delay_ms(2000);

    // Run all tests
    test_basic_text();
    test_individual_dots();
    test_lines();
    test_rectangles();
    test_circles();
    test_numbers();
    test_animation_simple();
    test_complex_pattern();

    // Final message
    lcd_clear();
    lcd_string(2, 0, "All Tests");
    lcd_string(3, 0, "Completed!");
    lcd_string(5, 0, "Press Reset");
    lcd_string(6, 0, "to Run Again");

    while (1)
    {
        _delay_ms(1000);
    }
}
#endif

// Main entry point
int main(void)
{
#ifdef GRAPHICS_COMPREHENSIVE_TEST
    main_graphics_comprehensive_test();
#endif

    return 0;
}
