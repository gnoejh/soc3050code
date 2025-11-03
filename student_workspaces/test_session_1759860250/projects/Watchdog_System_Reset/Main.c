/*
 * =============================================================================
 * WATCHDOG TIMER SYSTEM RESET - EDUCATIONAL DEMONSTRATION
 * =============================================================================
 *
 * PROJECT: Watchdog_System_Reset
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Educational demonstration of watchdog timer for system reset and recovery.
 * Students learn fail-safe programming and system reliability techniques.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master watchdog timer (WDT) operation and configuration
 * 2. Learn system reset mechanisms and recovery strategies
 * 3. Practice fail-safe programming techniques
 * 4. Implement crash detection and automatic recovery
 * 5. Understand system reliability and fault tolerance
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - LEDs for system status indication
 * - Push buttons for manual reset testing
 * - Status indicators for watchdog operations
 * - Serial connection for debugging (9600 baud)
 *
 * WATCHDOG TIMER OVERVIEW:
 * - Independent RC oscillator (~1MHz)
 * - Timeout periods: 16ms to 2048ms
 * - Generates system reset if not cleared
 * - Useful for detecting software crashes and hangs
 *
 * TIMEOUT PERIODS:
 * WDP2 WDP1 WDP0 | Timeout
 * ---------------+---------
 *  0    0    0   | 16.3 ms
 *  0    0    1   | 32.5 ms
 *  0    1    0   | 65 ms
 *  0    1    1   | 0.13 s
 *  1    0    0   | 0.26 s
 *  1    0    1   | 0.52 s
 *  1    1    0   | 1.0 s
 *  1    1    1   | 2.1 s
 *
 * LEARNING PROGRESSION:
 * - Demo 1: Basic Watchdog Configuration
 * - Demo 2: System Reset Testing
 * - Demo 3: Automatic Recovery Implementation
 * - Demo 4: Fail-Safe System Design
 *
 * =============================================================================
 */
*0 0 1 | 32.5 ms * 0 1 0 | 65 ms * 0 1 1 | 0.13 s * 1 0 0 | 0.26 s * 1 0 1 | 0.52 s * 1 1 0 | 1.0 s * 1 1 1 | 2.1 s * /

#include "config.h"
#include <avr/wdt.h>

// Watchdog timeout periods
#define WDT_16MS 0
#define WDT_32MS 1
#define WDT_65MS 2
#define WDT_130MS 3
#define WDT_260MS 4
#define WDT_520MS 5
#define WDT_1S 6
#define WDT_2S 7

                                                                                                                  // Reset reason tracking (stored in EEPROM or RAM that survives reset)
                                                                                                                  volatile uint8_t reset_count __attribute__((section(".noinit")));
volatile uint8_t last_reset_reason __attribute__((section(".noinit")));

/*
 * Enable watchdog with specific timeout
 */
void watchdog_enable(uint8_t timeout)
{
    cli();       // Disable interrupts
    wdt_reset(); // Reset watchdog timer

    // Start timed sequence
    WDTCR = (1 << WDCE) | (1 << WDE);

    // Set new timeout value (within 4 cycles)
    WDTCR = (1 << WDE) | (timeout & 0x07);

    sei(); // Re-enable interrupts
}

/*
 * Disable watchdog
 */
void watchdog_disable(void)
{
    cli();
    wdt_reset();

    // Clear WDRF in MCUCSR
    MCUCSR &= ~(1 << WDRF);

    // Write logical one to WDCE and WDE
    WDTCR = (1 << WDCE) | (1 << WDE);

    // Turn off WDT (within 4 cycles)
    WDTCR = 0x00;

    sei();
}

/*
 * Check reset source
 */
const char *get_reset_source(void)
{
    uint8_t mcucsr = MCUCSR;

    if (mcucsr & (1 << WDRF))
    {
        return "Watchdog Reset";
    }
    else if (mcucsr & (1 << BORF))
    {
        return "Brown-out Reset";
    }
    else if (mcucsr & (1 << EXTRF))
    {
        return "External Reset";
    }
    else if (mcucsr & (1 << PORF))
    {
        return "Power-on Reset";
    }
    else
    {
        return "Unknown Reset";
    }
}

/* ========================================================================
 * DEMO 1: Basic Watchdog Reset
 * ======================================================================== */
void demo1_basic_reset(void)
{
    puts_USART1("\r\n=== DEMO 1: Basic Watchdog Reset ===\r\n");
    puts_USART1("Watchdog will reset system after timeout\r\n\r\n");

    puts_USART1("Select timeout period:\r\n");
    puts_USART1("  [1] 260ms\r\n");
    puts_USART1("  [2] 520ms\r\n");
    puts_USART1("  [3] 1 second\r\n");
    puts_USART1("  [4] 2 seconds\r\n");
    puts_USART1("Enter choice: ");

    char choice = getch_USART1();
    putch_USART1(choice);
    puts_USART1("\r\n\r\n");

    uint8_t timeout;
    uint16_t timeout_ms;

    switch (choice)
    {
    case '1':
        timeout = WDT_260MS;
        timeout_ms = 260;
        break;
    case '2':
        timeout = WDT_520MS;
        timeout_ms = 520;
        break;
    case '3':
        timeout = WDT_1S;
        timeout_ms = 1000;
        break;
    case '4':
        timeout = WDT_2S;
        timeout_ms = 2000;
        break;
    default:
        timeout = WDT_1S;
        timeout_ms = 1000;
        break;
    }

    char buf[80];
    sprintf(buf, "Enabling watchdog with %ums timeout...\r\n", timeout_ms);
    puts_USART1(buf);

    watchdog_enable(timeout);

    puts_USART1("Watchdog enabled!\r\n");
    puts_USART1("System will reset if watchdog not cleared.\r\n");
    puts_USART1("Waiting for reset...\r\n\r\n");

    // Countdown
    for (uint16_t i = timeout_ms / 100; i > 0; i--)
    {
        sprintf(buf, "\rReset in: %u.%u seconds... ", i / 10, i % 10);
        puts_USART1(buf);

        PORTC = (uint8_t)((timeout_ms - (i * 100)) * 255 / timeout_ms);

        _delay_ms(100);
    }

    puts_USART1("\r\n\r\n*** WATCHDOG RESET SHOULD OCCUR NOW ***\r\n");

    // System will reset here - code below won't execute
    while (1)
    {
        PORTC = 0xFF;
        _delay_ms(50);
        PORTC = 0x00;
        _delay_ms(50);
    }
}

/* ========================================================================
 * DEMO 2: Watchdog with Periodic Reset
 * ======================================================================== */
void demo2_periodic_reset(void)
{
    puts_USART1("\r\n=== DEMO 2: Watchdog with Periodic Reset ===\r\n");
    puts_USART1("Demonstrating proper watchdog usage\r\n");
    puts_USART1("Press any key to stop\r\n\r\n");

    // Enable watchdog with 1 second timeout
    watchdog_enable(WDT_1S);

    puts_USART1("Watchdog enabled (1 second timeout)\r\n");
    puts_USART1("Clearing watchdog every 500ms...\r\n\r\n");

    uint16_t iterations = 0;

    while (1)
    {
        // Do work
        iterations++;

        char buf[60];
        sprintf(buf, "\rIteration: %u (Watchdog OK)    ", iterations);
        puts_USART1(buf);

        // Blink LED to show activity
        PORTC ^= 0x01;

        // Simulate work
        _delay_ms(400);

        // Reset watchdog (CRITICAL - prevents system reset)
        wdt_reset();

        _delay_ms(100);

        // Check for user input
        if (UCSR1A & (1 << RXC1))
        {
            getch_USART1();

            watchdog_disable();

            sprintf(buf, "\r\n\r\nWatchdog disabled after %u iterations.\r\n", iterations);
            puts_USART1(buf);
            puts_USART1("System is now running without watchdog protection.\r\n");

            return;
        }
    }
}

/* ========================================================================
 * DEMO 3: Simulated System Hang
 * ======================================================================== */
void demo3_system_hang(void)
{
    puts_USART1("\r\n=== DEMO 3: Simulated System Hang ===\r\n");
    puts_USART1("Watchdog will recover from hang\r\n\r\n");

    // Enable watchdog
    watchdog_enable(WDT_2S);

    puts_USART1("Watchdog enabled (2 second timeout)\r\n");
    puts_USART1("Simulating normal operation for 5 seconds...\r\n");

    // Normal operation - clear watchdog regularly
    for (uint8_t i = 0; i < 50; i++)
    {
        char buf[40];
        sprintf(buf, "\rNormal operation: %u/50  ", i + 1);
        puts_USART1(buf);

        PORTC = i % 8;

        _delay_ms(100);
        wdt_reset(); // Clear watchdog
    }

    puts_USART1("\r\n\r\n*** SIMULATING INFINITE LOOP (HANG) ***\r\n");
    puts_USART1("Watchdog will NOT be cleared...\r\n");
    puts_USART1("System should reset in ~2 seconds\r\n\r\n");

    // Simulate hang - infinite loop without clearing watchdog
    uint16_t hang_count = 0;
    while (1)
    {
        hang_count++;
        char buf[50];
        sprintf(buf, "\rHanging... count: %u  ", hang_count);
        puts_USART1(buf);

        PORTC = 0xFF;
        _delay_ms(100);
        PORTC = 0x00;
        _delay_ms(100);

        // NOTE: wdt_reset() is NOT called - system will reset!
    }

    // This code is unreachable - system will reset
}

/* ========================================================================
 * DEMO 4: Reset Recovery System
 * ======================================================================== */
void demo4_reset_recovery(void)
{
    puts_USART1("\r\n=== DEMO 4: Reset Recovery System ===\r\n");

    // Check reset source
    const char *reset_source = get_reset_source();

    char buf[80];
    sprintf(buf, "Last reset source: %s\r\n", reset_source);
    puts_USART1(buf);
    sprintf(buf, "Reset count: %u\r\n\r\n", reset_count);
    puts_USART1(buf);

    if (MCUCSR & (1 << WDRF))
    {
        puts_USART1("*** RECOVERED FROM WATCHDOG RESET ***\r\n");
        puts_USART1("System was previously hung and has been reset.\r\n\r\n");

        // Flash LEDs to indicate recovery
        for (uint8_t i = 0; i < 5; i++)
        {
            PORTC = 0xFF;
            _delay_ms(100);
            PORTC = 0x00;
            _delay_ms(100);
        }
    }

    // Clear reset flags
    MCUCSR = 0;

    puts_USART1("Select action:\r\n");
    puts_USART1("  [1] Run normally (with watchdog protection)\r\n");
    puts_USART1("  [2] Trigger intentional hang (test recovery)\r\n");
    puts_USART1("  [3] Exit demo\r\n");
    puts_USART1("Enter choice: ");

    char choice = getch_USART1();
    putch_USART1(choice);
    puts_USART1("\r\n\r\n");

    if (choice == '1')
    {
        // Normal operation with watchdog
        watchdog_enable(WDT_1S);

        puts_USART1("Running with watchdog protection...\r\n");
        puts_USART1("Press any key to stop\r\n\r\n");

        uint16_t cycles = 0;

        while (1)
        {
            cycles++;

            sprintf(buf, "\rCycle: %u (Protected)    ", cycles);
            puts_USART1(buf);

            PORTC = (cycles % 8) | 0x80; // MSB indicates watchdog active

            _delay_ms(500);
            wdt_reset(); // Clear watchdog

            if (UCSR1A & (1 << RXC1))
            {
                getch_USART1();
                watchdog_disable();
                puts_USART1("\r\n\r\nStopped. Watchdog disabled.\r\n");
                return;
            }
        }
    }
    else if (choice == '2')
    {
        // Trigger hang
        reset_count++;

        watchdog_enable(WDT_2S);

        puts_USART1("Triggering system hang...\r\n");
        puts_USART1("Watchdog will reset system.\r\n");
        puts_USART1("After reset, run this demo again to see recovery.\r\n\r\n");

        _delay_ms(1000);

        puts_USART1("Entering infinite loop NOW...\r\n\r\n");

        // Infinite loop - system will reset
        while (1)
        {
            PORTC = 0xFF;
            _delay_ms(100);
            PORTC = 0x00;
            _delay_ms(100);
        }
    }
    else
    {
        puts_USART1("Exiting demo...\r\n");
        return;
    }
}

/* ========================================================================
 * Main Menu System
 * ======================================================================== */
void display_main_menu(void)
{
    puts_USART1("\r\n\r\n");
    puts_USART1("╔════════════════════════════════════════╗\r\n");
    puts_USART1("║  Watchdog Timer Demo - ATmega128      ║\r\n");
    puts_USART1("╚════════════════════════════════════════╝\r\n");
    puts_USART1("\r\n");
    puts_USART1("Select Demo:\r\n");
    puts_USART1("  [1] Basic Watchdog Reset\r\n");
    puts_USART1("  [2] Periodic Watchdog Reset\r\n");
    puts_USART1("  [3] Simulated System Hang\r\n");
    puts_USART1("  [4] Reset Recovery System\r\n");
    puts_USART1("\r\n");
    puts_USART1("Enter selection (1-4): ");
}

int main(void)
{
    // CRITICAL: Disable watchdog immediately at startup
    // (in case we're recovering from watchdog reset)
    MCUCSR &= ~(1 << WDRF);
    wdt_disable();

    // Initialize peripherals
    Uart1_init();

    // Configure status LEDs
    DDRC = 0xFF;
    PORTC = 0x00;

    // Send startup message
    _delay_ms(500);
    puts_USART1("\r\n\r\n*** Watchdog Timer System ***\r\n");
    puts_USART1("System Reset and Recovery\r\n\r\n");

    // Display reset information
    const char *reset_source = get_reset_source();
    char buf[60];
    sprintf(buf, "Boot reason: %s\r\n", reset_source);
    puts_USART1(buf);

    if (MCUCSR & (1 << WDRF))
    {
        puts_USART1("WARNING: System recovered from watchdog reset!\r\n");
        PORTC = 0xFF;
        _delay_ms(500);
        PORTC = 0x00;
    }

    // Clear reset flags
    MCUCSR = 0;

    PORTC = 0x01;
    _delay_ms(1000);

    while (1)
    {
        display_main_menu();

        char choice = getch_USART1();
        putch_USART1(choice);
        puts_USART1("\r\n");

        switch (choice)
        {
        case '1':
            demo1_basic_reset();
            break;
        case '2':
            demo2_periodic_reset();
            break;
        case '3':
            demo3_system_hang();
            break;
        case '4':
            demo4_reset_recovery();
            break;
        default:
            puts_USART1("Invalid selection!\r\n");
            _delay_ms(1000);
            break;
        }

        // Ensure watchdog is disabled between demos
        watchdog_disable();

        _delay_ms(500);
    }

    return 0;
}
