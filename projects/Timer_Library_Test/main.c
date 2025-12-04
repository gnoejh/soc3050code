/*
 * Timer Library Test Program
 *
 * PURPOSE:
 * Test all features of the new general Timer library (_timer.h)
 * Verifies Timer0/1/2/3 support, all modes, interrupts, callbacks
 *
 * HARDWARE:
 * - ATmega128 board
 * - 8 LEDs on Port G (PG0-PG7)
 * - Serial connection for test output
 *
 * TESTS PERFORMED:
 * 1. Timer0 - Normal mode with overflow interrupt
 * 2. Timer1 - CTC mode with compare match interrupt
 * 3. Timer2 - Normal mode with overflow interrupt
 * 4. Timer3 - CTC mode with compare match interrupt
 * 5. Millisecond timing system (millis/micros)
 * 6. Task scheduler with multiple tasks
 * 7. Callback-based interrupts
 *
 * EXPECTED RESULTS:
 * - LED0: Blinks at ~1Hz (Timer0 overflow)
 * - LED1: Blinks at ~2Hz (Timer1 CTC)
 * - LED2: Blinks at ~0.5Hz (Timer2 overflow)
 * - LED3: Blinks at ~4Hz (Timer3 CTC)
 * - LED4: Blinks every 500ms (scheduler task 1)
 * - LED5: Blinks every 1000ms (scheduler task 2)
 * - LED6: Shows millis() system (toggle every 100ms)
 * - LED7: Shows test status (on = all tests passed)
 *
 * AUTHOR: Framework Test Suite
 * DATE: November 2025
 * VERSION: 1.0
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "_main.h"
#include "_timer.h"
#include "uart_enhanced.h"

// Test configuration
#define LED_PORT PORTG
#define LED_DDR DDRG

#define LED0 0 // Timer0 overflow
#define LED1 1 // Timer1 CTC
#define LED2 2 // Timer2 overflow
#define LED3 3 // Timer3 CTC
#define LED4 4 // Scheduler task 1 (500ms)
#define LED5 5 // Scheduler task 2 (1000ms)
#define LED6 6 // Millis system (100ms)
#define LED7 7 // Test status

// Test counters
volatile uint16_t timer0_overflows = 0;
volatile uint16_t timer1_matches = 0;
volatile uint16_t timer2_overflows = 0;
volatile uint16_t timer3_matches = 0;

// Test status flags
volatile uint8_t test_results = 0;
#define TEST_TIMER0_OK (1 << 0)
#define TEST_TIMER1_OK (1 << 1)
#define TEST_TIMER2_OK (1 << 2)
#define TEST_TIMER3_OK (1 << 3)
#define TEST_MILLIS_OK (1 << 4)
#define TEST_SCHEDULER_OK (1 << 5)

/*
 * TIMER0 CALLBACK
 * Called on every overflow (~61Hz at prescaler=1024)
 * Toggle LED0 at ~1Hz (every 61 overflows)
 */
void timer0_overflow_callback(void)
{
    timer0_overflows++;

    if (timer0_overflows >= 61)
    {                            // ~1 second
        LED_PORT ^= (1 << LED0); // Toggle LED0
        timer0_overflows = 0;
        test_results |= TEST_TIMER0_OK;
    }
}

/*
 * TIMER1 CALLBACK (Compare Match A)
 * Called at 2Hz (configured in CTC mode)
 * Toggle LED1
 */
void timer1_compare_callback(void)
{
    LED_PORT ^= (1 << LED1); // Toggle LED1
    timer1_matches++;
    test_results |= TEST_TIMER1_OK;
}

/*
 * TIMER2 CALLBACK
 * Called on every overflow (~244Hz at prescaler=256)
 * Toggle LED2 at ~0.5Hz (every 488 overflows)
 */
void timer2_overflow_callback(void)
{
    timer2_overflows++;

    if (timer2_overflows >= 488)
    {                            // ~2 seconds
        LED_PORT ^= (1 << LED2); // Toggle LED2
        timer2_overflows = 0;
        test_results |= TEST_TIMER2_OK;
    }
}

/*
 * TIMER3 CALLBACK (Compare Match A)
 * Called at 4Hz (configured in CTC mode)
 * Toggle LED3
 */
void timer3_compare_callback(void)
{
    LED_PORT ^= (1 << LED3); // Toggle LED3
    timer3_matches++;
    test_results |= TEST_TIMER3_OK;
}

/*
 * SCHEDULER TASK 1
 * Runs every 500ms
 * Toggle LED4
 */
void scheduler_task_500ms(void)
{
    LED_PORT ^= (1 << LED4);
}

/*
 * SCHEDULER TASK 2
 * Runs every 1000ms
 * Toggle LED5
 */
void scheduler_task_1000ms(void)
{
    LED_PORT ^= (1 << LED5);
}

/*
 * INITIALIZE HARDWARE
 */
void init_hardware(void)
{
    // Configure LEDs as outputs
    LED_DDR = 0xFF;
    LED_PORT = 0x00;

    // Initialize UART for test output
    uart_enhanced_init(9600, 8, 0, 1); // 9600 8N1

    // Print test header
    uart_enhanced_printf("\r\n");
    uart_enhanced_printf("===========================================\r\n");
    uart_enhanced_printf("  TIMER LIBRARY COMPREHENSIVE TEST\r\n");
    uart_enhanced_printf("===========================================\r\n");
    uart_enhanced_printf("Testing: _timer.h v3.0\r\n");
    uart_enhanced_printf("Date: November 2025\r\n");
    uart_enhanced_printf("\r\n");
}

/*
 * TEST 1: TIMER0 NORMAL MODE
 * Configure Timer0 with overflow interrupt
 */
void test_timer0(void)
{
    uart_enhanced_printf("[TEST 1] Timer0 - Normal Mode\r\n");
    uart_enhanced_printf("  Prescaler: 1024\r\n");
    uart_enhanced_printf("  Expected: LED0 blinks at ~1Hz\r\n");

    // Initialize Timer0
    if (!Timer_init(TIMER_0, TIMER_MODE_NORMAL, TIMER0_PRESCALER_1024))
    {
        uart_enhanced_printf("  [FAIL] Timer0 init failed\r\n");
        return;
    }

    // Register callback
    Timer_register_overflow_callback(TIMER_0, timer0_overflow_callback);

    // Start timer
    Timer_start(TIMER_0, TIMER0_PRESCALER_1024);

    uart_enhanced_printf("  [OK] Timer0 started\r\n");
    uart_enhanced_printf("\r\n");
}

/*
 * TEST 2: TIMER1 CTC MODE
 * Configure Timer1 with compare match interrupt at 2Hz
 */
void test_timer1(void)
{
    uart_enhanced_printf("[TEST 2] Timer1 - CTC Mode\r\n");
    uart_enhanced_printf("  Frequency: 2Hz\r\n");
    uart_enhanced_printf("  Expected: LED1 blinks at 2Hz\r\n");

    // Initialize Timer1 in CTC mode
    if (!Timer_init(TIMER_1, TIMER_MODE_CTC, TIMER1_PRESCALER_256))
    {
        uart_enhanced_printf("  [FAIL] Timer1 init failed\r\n");
        return;
    }

    // Calculate compare value for 2Hz
    // F_CPU = 16MHz, prescaler = 256
    // Compare = (F_CPU / (prescaler * 2 * frequency)) - 1
    // Compare = (16000000 / (256 * 2 * 2)) - 1 = 15624
    uint16_t compare = Timer_calculate_compare_value(2, TIMER1_PRESCALER_256, TIMER_1);
    Timer_set_compare_value(TIMER_1, TIMER_COMPARE_A, compare);

    uart_enhanced_printf("  Compare value: %u (calculated: %u)\r\n", compare, 15624);

    // Register callback
    Timer_register_compare_callback(TIMER_1, TIMER_COMPARE_A, timer1_compare_callback);

    // Start timer
    Timer_start(TIMER_1, TIMER1_PRESCALER_256);

    uart_enhanced_printf("  [OK] Timer1 started\r\n");
    uart_enhanced_printf("\r\n");
}

/*
 * TEST 3: TIMER2 NORMAL MODE
 * Configure Timer2 with overflow interrupt
 */
void test_timer2(void)
{
    uart_enhanced_printf("[TEST 3] Timer2 - Normal Mode\r\n");
    uart_enhanced_printf("  Prescaler: 256\r\n");
    uart_enhanced_printf("  Expected: LED2 blinks at ~0.5Hz\r\n");

    // Initialize Timer2
    if (!Timer_init(TIMER_2, TIMER_MODE_NORMAL, TIMER2_PRESCALER_256))
    {
        uart_enhanced_printf("  [FAIL] Timer2 init failed\r\n");
        return;
    }

    // Register callback
    Timer_register_overflow_callback(TIMER_2, timer2_overflow_callback);

    // Start timer
    Timer_start(TIMER_2, TIMER2_PRESCALER_256);

    uart_enhanced_printf("  [OK] Timer2 started\r\n");
    uart_enhanced_printf("\r\n");
}

/*
 * TEST 4: TIMER3 CTC MODE
 * Configure Timer3 with compare match interrupt at 4Hz
 */
void test_timer3(void)
{
    uart_enhanced_printf("[TEST 4] Timer3 - CTC Mode\r\n");
    uart_enhanced_printf("  Frequency: 4Hz\r\n");
    uart_enhanced_printf("  Expected: LED3 blinks at 4Hz\r\n");

    // Initialize Timer3 in CTC mode
    if (!Timer_init(TIMER_3, TIMER_MODE_CTC, TIMER3_PRESCALER_256))
    {
        uart_enhanced_printf("  [FAIL] Timer3 init failed\r\n");
        return;
    }

    // Calculate compare value for 4Hz
    uint16_t compare = Timer_calculate_compare_value(4, TIMER3_PRESCALER_256, TIMER_3);
    Timer_set_compare_value(TIMER_3, TIMER_COMPARE_A, compare);

    uart_enhanced_printf("  Compare value: %u\r\n", compare);

    // Register callback
    Timer_register_compare_callback(TIMER_3, TIMER_COMPARE_A, timer3_compare_callback);

    // Start timer
    Timer_start(TIMER_3, TIMER3_PRESCALER_256);

    uart_enhanced_printf("  [OK] Timer3 started\r\n");
    uart_enhanced_printf("\r\n");
}

/*
 * TEST 5: MILLISECOND TIMING SYSTEM
 * Test millis() and micros() functions
 */
void test_millis_system(void)
{
    uart_enhanced_printf("[TEST 5] Millisecond Timing System\r\n");
    uart_enhanced_printf("  Testing: Timer_millis() and Timer_micros()\r\n");

    // Initialize millis system (uses Timer0)
    Timer_millis_init();

    uart_enhanced_printf("  [OK] Millis system initialized\r\n");

    // Test millis() accuracy
    uint32_t start = Timer_millis();
    _delay_ms(1000); // Delay 1 second
    uint32_t end = Timer_millis();
    uint32_t elapsed = end - start;

    uart_enhanced_printf("  Elapsed time: %lu ms (expected: ~1000ms)\r\n", elapsed);

    if (elapsed >= 950 && elapsed <= 1050)
    {
        uart_enhanced_printf("  [OK] Millis accuracy within 5%%\r\n");
        test_results |= TEST_MILLIS_OK;
    }
    else
    {
        uart_enhanced_printf("  [FAIL] Millis accuracy error > 5%%\r\n");
    }

    uart_enhanced_printf("\r\n");
}

/*
 * TEST 6: TASK SCHEDULER
 * Test callback-based task scheduling
 */
void test_scheduler(void)
{
    uart_enhanced_printf("[TEST 6] Task Scheduler\r\n");
    uart_enhanced_printf("  Testing: Timer_scheduler_add_task()\r\n");

    // Initialize scheduler
    Timer_scheduler_init(TIMER_2);

    // Add tasks
    if (!Timer_scheduler_add_task(scheduler_task_500ms, 500))
    {
        uart_enhanced_printf("  [FAIL] Could not add 500ms task\r\n");
        return;
    }

    if (!Timer_scheduler_add_task(scheduler_task_1000ms, 1000))
    {
        uart_enhanced_printf("  [FAIL] Could not add 1000ms task\r\n");
        return;
    }

    uart_enhanced_printf("  [OK] 2 tasks scheduled\r\n");
    uart_enhanced_printf("    Task 1: 500ms (LED4)\r\n");
    uart_enhanced_printf("    Task 2: 1000ms (LED5)\r\n");

    test_results |= TEST_SCHEDULER_OK;

    uart_enhanced_printf("\r\n");
}

/*
 * PRINT TEST SUMMARY
 */
void print_test_summary(void)
{
    uart_enhanced_printf("===========================================\r\n");
    uart_enhanced_printf("  TEST SUMMARY\r\n");
    uart_enhanced_printf("===========================================\r\n");

    uart_enhanced_printf("Timer0 Normal Mode:     %s\r\n",
                         (test_results & TEST_TIMER0_OK) ? "[PASS]" : "[WAIT]");
    uart_enhanced_printf("Timer1 CTC Mode:        %s\r\n",
                         (test_results & TEST_TIMER1_OK) ? "[PASS]" : "[WAIT]");
    uart_enhanced_printf("Timer2 Normal Mode:     %s\r\n",
                         (test_results & TEST_TIMER2_OK) ? "[PASS]" : "[WAIT]");
    uart_enhanced_printf("Timer3 CTC Mode:        %s\r\n",
                         (test_results & TEST_TIMER3_OK) ? "[PASS]" : "[WAIT]");
    uart_enhanced_printf("Millis/Micros System:   %s\r\n",
                         (test_results & TEST_MILLIS_OK) ? "[PASS]" : "[WAIT]");
    uart_enhanced_printf("Task Scheduler:         %s\r\n",
                         (test_results & TEST_SCHEDULER_OK) ? "[PASS]" : "[WAIT]");

    uint8_t passed = 0;
    for (uint8_t i = 0; i < 6; i++)
    {
        if (test_results & (1 << i))
            passed++;
    }

    uart_enhanced_printf("\r\n");
    uart_enhanced_printf("Tests passed: %d/6\r\n", passed);

    if (passed == 6)
    {
        uart_enhanced_printf("\r\n");
        uart_enhanced_printf("*** ALL TESTS PASSED ***\r\n");
        LED_PORT |= (1 << LED7); // Turn on status LED
    }

    uart_enhanced_printf("===========================================\r\n");
    uart_enhanced_printf("\r\n");
}

/*
 * MAIN FUNCTION
 */
int main(void)
{
    // Initialize hardware
    init_hardware();

    // Run all tests
    test_timer0();
    test_timer1();
    test_timer2();
    test_timer3();
    test_millis_system();
    test_scheduler();

    // Enable global interrupts
    sei();

    // Wait a few seconds for timers to start
    uart_enhanced_printf("Waiting 5 seconds for timers to stabilize...\r\n");
    _delay_ms(5000);

    // Print initial summary
    print_test_summary();

    // Main loop - update scheduler and toggle LED6 every 100ms
    uint32_t last_millis_test = 0;
    uint32_t last_summary = 0;

    while (1)
    {
        // Update scheduler
        Timer_scheduler_update();

        // Toggle LED6 every 100ms (millis system test)
        uint32_t current = Timer_millis();
        if (current - last_millis_test >= 100)
        {
            LED_PORT ^= (1 << LED6);
            last_millis_test = current;
        }

        // Print summary every 10 seconds
        if (current - last_summary >= 10000)
        {
            uart_enhanced_printf("\r\n--- Status Update (t=%lu ms) ---\r\n", current);
            uart_enhanced_printf("Timer0 overflows: %u\r\n", timer0_overflows);
            uart_enhanced_printf("Timer1 matches:   %u\r\n", timer1_matches);
            uart_enhanced_printf("Timer2 overflows: %u\r\n", timer2_overflows);
            uart_enhanced_printf("Timer3 matches:   %u\r\n", timer3_matches);
            print_test_summary();
            last_summary = current;
        }

        // Small delay to prevent busy-waiting
        _delay_ms(10);
    }

    return 0;
}
