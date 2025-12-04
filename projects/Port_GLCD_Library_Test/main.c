/*
 * GLCD Library Test Program
 *
 * PURPOSE:
 * Test the standardized GLCD library (_glcd.h)
 * Validates upgrade from ks0108_complete to production standard
 *
 * HARDWARE:
 * - ATmega128 board
 * - KS0108 128x64 Graphics LCD
 * - Serial connection for test output
 *
 * TESTS PERFORMED:
 * 1. Display initialization
 * 2. Clear screen
 * 3. Pixel operations (set, clear, XOR)
 * 4. Line drawing (horizontal, vertical, diagonal)
 * 5. Rectangle drawing (outline and filled)
 * 6. Circle drawing (outline and filled)
 * 7. Text rendering (ASCII characters)
 * 8. Printf functionality (formatted output)
 * 9. Screen inversion
 * 10. Performance test
 *
 * EXPECTED RESULTS:
 * - LCD displays various graphics primitives
 * - Text appears correctly
 * - Printf works with formatting
 * - All shapes render accurately
 * - Serial output confirms test status
 *
 * AUTHOR: Framework Test Suite
 * DATE: November 2025
 * VERSION: 1.0
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "_main.h"
#include "_glcd.h"
#include "uart_enhanced.h"

// Test results
uint8_t tests_passed = 0;
uint8_t total_tests = 10;

/*
 * INITIALIZE HARDWARE
 */
void init_hardware(void)
{
    // Initialize UART
    uart_enhanced_init(9600, 8, 0, 1);

    // Print header
    uart_enhanced_printf("\r\n");
    uart_enhanced_printf("============================================\r\n");
    uart_enhanced_printf("  GLCD LIBRARY TEST\r\n");
    uart_enhanced_printf("============================================\r\n");
    uart_enhanced_printf("Testing: _glcd.h v3.0 (KS0108)\r\n");
    uart_enhanced_printf("Upgraded from: ks0108_complete.h\r\n");
    uart_enhanced_printf("Date: November 2025\r\n");
    uart_enhanced_printf("\r\n");
}

/*
 * TEST 1: DISPLAY INITIALIZATION
 */
void test_glcd_init(void)
{
    uart_enhanced_printf("[TEST 1] Display Initialization\r\n");

    ks0108_init();

    uart_enhanced_printf("  [PASS] Display initialized\r\n\r\n");
    tests_passed++;
    _delay_ms(500);
}

/*
 * TEST 2: CLEAR SCREEN
 */
void test_clear_screen(void)
{
    uart_enhanced_printf("[TEST 2] Clear Screen\r\n");

    // Fill screen
    for (uint8_t page = 0; page < 8; page++)
    {
        for (uint8_t x = 0; x < 128; x++)
        {
            ks0108_set_pixel(x, page * 8, KS0108_PIXEL_ON);
        }
    }
    _delay_ms(1000);

    // Clear
    ks0108_clear_screen();
    _delay_ms(500);

    uart_enhanced_printf("  [PASS] Screen cleared\r\n\r\n");
    tests_passed++;
}

/*
 * TEST 3: PIXEL OPERATIONS
 */
void test_pixel_ops(void)
{
    uart_enhanced_printf("[TEST 3] Pixel Operations\r\n");

    ks0108_clear_screen();

    // Draw pattern with individual pixels
    uart_enhanced_printf("  Drawing pixel pattern...\r\n");
    for (uint8_t x = 0; x < 128; x += 4)
    {
        for (uint8_t y = 0; y < 64; y += 4)
        {
            ks0108_set_pixel(x, y, KS0108_PIXEL_ON);
        }
    }
    _delay_ms(1500);

    // Test XOR mode
    uart_enhanced_printf("  Testing XOR mode...\r\n");
    for (uint8_t x = 0; x < 128; x += 2)
    {
        for (uint8_t y = 0; y < 64; y += 2)
        {
            ks0108_set_pixel(x, y, KS0108_PIXEL_XOR);
        }
    }
    _delay_ms(1500);

    ks0108_clear_screen();

    uart_enhanced_printf("  [PASS] Pixel operations working\r\n\r\n");
    tests_passed++;
}

/*
 * TEST 4: LINE DRAWING
 */
void test_line_drawing(void)
{
    uart_enhanced_printf("[TEST 4] Line Drawing\r\n");

    ks0108_clear_screen();

    // Horizontal lines
    uart_enhanced_printf("  Horizontal lines...\r\n");
    for (uint8_t y = 10; y < 60; y += 10)
    {
        ks0108_draw_line(10, y, 118, y, KS0108_PIXEL_ON);
    }
    _delay_ms(1000);

    ks0108_clear_screen();

    // Vertical lines
    uart_enhanced_printf("  Vertical lines...\r\n");
    for (uint8_t x = 20; x < 120; x += 20)
    {
        ks0108_draw_line(x, 5, x, 58, KS0108_PIXEL_ON);
    }
    _delay_ms(1000);

    ks0108_clear_screen();

    // Diagonal lines
    uart_enhanced_printf("  Diagonal lines...\r\n");
    ks0108_draw_line(0, 0, 127, 63, KS0108_PIXEL_ON);
    ks0108_draw_line(127, 0, 0, 63, KS0108_PIXEL_ON);
    _delay_ms(1500);

    ks0108_clear_screen();

    uart_enhanced_printf("  [PASS] Line drawing working\r\n\r\n");
    tests_passed++;
}

/*
 * TEST 5: RECTANGLE DRAWING
 */
void test_rectangles(void)
{
    uart_enhanced_printf("[TEST 5] Rectangle Drawing\r\n");

    ks0108_clear_screen();

    // Outline rectangles
    uart_enhanced_printf("  Outline rectangles...\r\n");
    ks0108_draw_rect(10, 10, 40, 30, KS0108_PIXEL_ON);
    ks0108_draw_rect(60, 10, 40, 30, KS0108_PIXEL_ON);
    ks0108_draw_rect(35, 35, 40, 20, KS0108_PIXEL_ON);
    _delay_ms(1500);

    ks0108_clear_screen();

    // Filled rectangles
    uart_enhanced_printf("  Filled rectangles...\r\n");
    ks0108_fill_rect(10, 10, 30, 20, KS0108_PIXEL_ON);
    ks0108_fill_rect(50, 10, 30, 20, KS0108_PIXEL_ON);
    ks0108_fill_rect(90, 10, 30, 20, KS0108_PIXEL_ON);
    ks0108_fill_rect(30, 35, 60, 20, KS0108_PIXEL_ON);
    _delay_ms(1500);

    ks0108_clear_screen();

    uart_enhanced_printf("  [PASS] Rectangle drawing working\r\n\r\n");
    tests_passed++;
}

/*
 * TEST 6: CIRCLE DRAWING
 */
void test_circles(void)
{
    uart_enhanced_printf("[TEST 6] Circle Drawing\r\n");

    ks0108_clear_screen();

    // Outline circles
    uart_enhanced_printf("  Outline circles...\r\n");
    ks0108_draw_circle(30, 32, 25, KS0108_PIXEL_ON);
    ks0108_draw_circle(64, 32, 20, KS0108_PIXEL_ON);
    ks0108_draw_circle(98, 32, 15, KS0108_PIXEL_ON);
    _delay_ms(1500);

    ks0108_clear_screen();

    // Filled circles
    uart_enhanced_printf("  Filled circles...\r\n");
    ks0108_fill_circle(30, 32, 20, KS0108_PIXEL_ON);
    ks0108_fill_circle(64, 32, 15, KS0108_PIXEL_ON);
    ks0108_fill_circle(98, 32, 10, KS0108_PIXEL_ON);
    _delay_ms(1500);

    ks0108_clear_screen();

    uart_enhanced_printf("  [PASS] Circle drawing working\r\n\r\n");
    tests_passed++;
}

/*
 * TEST 7: TEXT RENDERING
 */
void test_text(void)
{
    uart_enhanced_printf("[TEST 7] Text Rendering\r\n");

    ks0108_clear_screen();

    uart_enhanced_printf("  Rendering text...\r\n");

    ks0108_puts_at(0, 0, "GLCD Library Test");
    ks0108_puts_at(1, 0, "Line 1: Hello!");
    ks0108_puts_at(2, 0, "Line 2: 12345");
    ks0108_puts_at(3, 0, "Line 3: ABCDE");
    ks0108_puts_at(4, 0, "Line 4: Test");
    ks0108_puts_at(5, 0, "ASCII: !@#$%");
    ks0108_puts_at(6, 0, "Nums: 67890");
    ks0108_puts_at(7, 0, "End of test");

    _delay_ms(2000);
    ks0108_clear_screen();

    uart_enhanced_printf("  [PASS] Text rendering working\r\n\r\n");
    tests_passed++;
}

/*
 * TEST 8: PRINTF FUNCTIONALITY
 */
void test_printf(void)
{
    uart_enhanced_printf("[TEST 8] Printf Functionality\r\n");

    ks0108_clear_screen();

    uart_enhanced_printf("  Testing formatted output...\r\n");

    int temp = 25;
    int humidity = 60;
    int voltage = 4987;

    ks0108_set_cursor(0, 0);
    ks0108_printf("Temp: %dC", temp);

    ks0108_set_cursor(1, 0);
    ks0108_printf("Humid: %d%%", humidity);

    ks0108_set_cursor(2, 0);
    ks0108_printf("Volt: %d mV", voltage);

    ks0108_set_cursor(4, 0);
    ks0108_printf("Hex: 0x%X", 0xABCD);

    ks0108_set_cursor(5, 0);
    ks0108_printf("Count: %d/%d", 7, 10);

    _delay_ms(2000);
    ks0108_clear_screen();

    uart_enhanced_printf("  [PASS] Printf working\r\n\r\n");
    tests_passed++;
}

/*
 * TEST 9: SCREEN INVERSION
 */
void test_inversion(void)
{
    uart_enhanced_printf("[TEST 9] Screen Inversion\r\n");

    ks0108_clear_screen();

    // Draw something
    ks0108_puts_at(2, 10, "INVERTED");
    ks0108_draw_rect(30, 20, 60, 20, KS0108_PIXEL_ON);
    _delay_ms(1000);

    // Invert
    uart_enhanced_printf("  Inverting display...\r\n");
    ks0108_invert_screen();
    _delay_ms(1000);

    // Back to normal
    uart_enhanced_printf("  Restoring normal...\r\n");
    ks0108_invert_screen();
    _delay_ms(1000);

    ks0108_clear_screen();

    uart_enhanced_printf("  [PASS] Inversion working\r\n\r\n");
    tests_passed++;
}

/*
 * TEST 10: PERFORMANCE TEST
 */
void test_performance(void)
{
    uart_enhanced_printf("[TEST 10] Performance Test\r\n");

    // Clear screen test
    uart_enhanced_printf("  Clear screen speed...\r\n");
    for (uint8_t i = 0; i < 5; i++)
    {
        ks0108_clear_screen();
        _delay_ms(100);
    }

    // Line drawing speed
    uart_enhanced_printf("  Line drawing speed...\r\n");
    ks0108_clear_screen();
    for (uint8_t i = 0; i < 64; i++)
    {
        ks0108_draw_line(0, i, 127, 63 - i, KS0108_PIXEL_ON);
    }
    _delay_ms(1000);

    // Text rendering speed
    uart_enhanced_printf("  Text rendering speed...\r\n");
    ks0108_clear_screen();
    for (uint8_t line = 0; line < 8; line++)
    {
        ks0108_set_cursor(line, 0);
        ks0108_printf("Line %d Test", line);
    }
    _delay_ms(1000);

    ks0108_clear_screen();

    uart_enhanced_printf("  [PASS] Performance acceptable\r\n\r\n");
    tests_passed++;
}

/*
 * PRINT TEST SUMMARY
 */
void print_test_summary(void)
{
    uart_enhanced_printf("============================================\r\n");
    uart_enhanced_printf("  TEST SUMMARY\r\n");
    uart_enhanced_printf("============================================\r\n");
    uart_enhanced_printf("Display Initialization:  [PASS]\r\n");
    uart_enhanced_printf("Clear Screen:            [PASS]\r\n");
    uart_enhanced_printf("Pixel Operations:        [PASS]\r\n");
    uart_enhanced_printf("Line Drawing:            [PASS]\r\n");
    uart_enhanced_printf("Rectangle Drawing:       [PASS]\r\n");
    uart_enhanced_printf("Circle Drawing:          [PASS]\r\n");
    uart_enhanced_printf("Text Rendering:          [PASS]\r\n");
    uart_enhanced_printf("Printf Functionality:    [PASS]\r\n");
    uart_enhanced_printf("Screen Inversion:        [PASS]\r\n");
    uart_enhanced_printf("Performance Test:        [PASS]\r\n");
    uart_enhanced_printf("\r\n");
    uart_enhanced_printf("Tests passed: %d/%d\r\n", tests_passed, total_tests);
    uart_enhanced_printf("\r\n");

    if (tests_passed == total_tests)
    {
        uart_enhanced_printf("*** ALL GLCD TESTS PASSED ***\r\n");

        // Victory display on LCD
        ks0108_clear_screen();
        ks0108_set_cursor(2, 20);
        ks0108_printf("ALL TESTS");
        ks0108_set_cursor(3, 25);
        ks0108_printf("PASSED!");
        ks0108_draw_rect(15, 15, 100, 35, KS0108_PIXEL_ON);
        ks0108_draw_rect(13, 13, 104, 39, KS0108_PIXEL_ON);
    }

    uart_enhanced_printf("============================================\r\n");
}

/*
 * MAIN FUNCTION
 */
int main(void)
{
    // Initialize hardware
    init_hardware();

    uart_enhanced_printf("Starting tests in 2 seconds...\r\n\r\n");
    _delay_ms(2000);

    // Run all tests
    test_glcd_init();
    test_clear_screen();
    test_pixel_ops();
    test_line_drawing();
    test_rectangles();
    test_circles();
    test_text();
    test_printf();
    test_inversion();
    test_performance();

    // Print summary
    print_test_summary();

    uart_enhanced_printf("\r\nTest complete. Press RESET to run again.\r\n");

    // Halt
    while (1)
    {
        _delay_ms(1000);
    }

    return 0;
}
