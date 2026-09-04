/*
 * ==============================================================================
 * WATCHDOG FAIL-SAFE SYSTEM - LED DEMO (Pure LED Version)
 * ==============================================================================
 * PROJECT: Watchdog_Fail_Safe
 * TARGET: ATmega128 @ 16 MHz
 *
 * LEARNING OBJECTIVES:
 * - Implement fail-safe watchdog strategies
 * - Demonstrate heartbeat monitoring
 * - Show graceful degradation patterns
 * - Practice recovery mechanisms
 *
 * HARDWARE:
 * - ATmega128 @ 16 MHz system clock
 * - Watchdog independent ~1 MHz RC oscillator
 * - PORTC LEDs (8 LEDs) for status indication
 *
 * LED PATTERNS:
 * - Pattern 1 (Binary count): Normal heartbeat monitoring
 * - Pattern 2 (Chase): Critical section execution
 * - Pattern 3 (Degraded): 1-3 LEDs for graceful degradation
 * - Pattern 4 (Fast blink): Recovery mode
 * - Pattern 5 (All ON): Watchdog reset detected
 *
 * AUTOMATIC DEMO SEQUENCE:
 * Demo 1: Heartbeat monitoring (5s) - Binary counting with watchdog
 * Demo 2: Critical section (3s) - LED chase pattern
 * Demo 3: Degradation (5s) - LED count reduces on "failures"
 * Demo 4: System hang - Triggers watchdog reset
 * After reset: Recovery indication with fast blinks
 * ==============================================================================
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>

// Watchdog timeouts
#define WDT_1S WDTO_1S
#define WDT_2S WDTO_2S

// EEPROM address for reset counter
#define EEPROM_RESET_COUNT 0

/*
 * LED Pattern Helpers
 */
void led_all_on(void)
{
    PORTC = 0xFF;
}

void led_all_off(void)
{
    PORTC = 0x00;
}

void led_set_pattern(uint8_t pattern)
{
    PORTC = pattern;
}

void led_blink_fast(uint8_t times)
{
    for (uint8_t i = 0; i < times; i++)
    {
        led_all_on();
        _delay_ms(100);
        led_all_off();
        _delay_ms(100);
    }
}

void led_chase(uint8_t cycles)
{
    for (uint8_t c = 0; c < cycles; c++)
    {
        for (uint8_t i = 0; i < 8; i++)
        {
            PORTC = (1 << i);
            _delay_ms(50);
        }
    }
}

void increment_reset_counter(void)
{
    uint8_t count = eeprom_read_byte((uint8_t *)EEPROM_RESET_COUNT);
    count++;
    eeprom_write_byte((uint8_t *)EEPROM_RESET_COUNT, count);
}

void show_reset_count(void)
{
    uint8_t count = eeprom_read_byte((uint8_t *)EEPROM_RESET_COUNT);
    // Display count on LEDs (binary)
    led_set_pattern(count);
    _delay_ms(2000);
}

/* ========================================================================
 * DEMO 1: Heartbeat Monitoring System
 * ======================================================================== */
void demo1_heartbeat_monitor(void)
{

    // Enable watchdog with 2 second timeout
    wdt_enable(WDT_2S);

    // Heartbeat monitoring: Binary counting pattern
    // Shows system is alive and watchdog is being cleared
    for (uint8_t i = 0; i < 50; i++)
    {
        led_set_pattern(i);
        _delay_ms(100);

        // Clear watchdog - critical for preventing reset
        wdt_reset();
    }

    wdt_disable();
}

/* ========================================================================
 * DEMO 2: Critical Section Protection
 * ======================================================================== */
void demo2_critical_section(void)
{
    // Enable watchdog
    wdt_enable(WDT_2S);

    // Critical section: LED chase pattern
    // Represents time-critical operation being monitored
    for (uint8_t cycle = 0; cycle < 5; cycle++)
    {
        for (uint8_t i = 0; i < 8; i++)
        {
            PORTC = (1 << i);
            _delay_ms(100);
        }
        wdt_reset(); // Clear watchdog after each cycle
    }

    wdt_disable();
}

/* ========================================================================
 * DEMO 3: Graceful Degradation
 * ======================================================================== */
void demo3_graceful_degradation(void)
{
    // Enable watchdog
    wdt_enable(WDT_2S);

    // Graceful degradation: LED count represents system capability
    // Starts with all LEDs (full function), reduces over time

    // Full functionality (8 LEDs)
    for (uint8_t i = 0; i < 10; i++)
    {
        led_set_pattern(0xFF);
        _delay_ms(150);
        wdt_reset();
    }

    // Degraded mode (4 LEDs)
    for (uint8_t i = 0; i < 10; i++)
    {
        led_set_pattern(0x0F);
        _delay_ms(150);
        wdt_reset();
    }

    // Minimal mode (1 LED)
    for (uint8_t i = 0; i < 10; i++)
    {
        led_set_pattern(0x01);
        _delay_ms(150);
        wdt_reset();
    }

    wdt_disable();
}

/* ========================================================================
 * DEMO 4: System Hang - Triggers Watchdog Reset
 * ======================================================================== */
void demo4_system_hang(void)
{
    // Enable watchdog with 2 second timeout
    wdt_enable(WDT_2S);

    // Increment reset counter for tracking
    increment_reset_counter();

    // Show countdown with fast alternating pattern
    for (uint8_t i = 0; i < 10; i++)
    {
        PORTC = 0xAA;
        _delay_ms(100);
        PORTC = 0x55;
        _delay_ms(100);
        // NOTE: wdt_reset() NOT called - system will hang and reset!
    }

    // Fast blink indicates imminent reset
    for (uint8_t i = 0; i < 10; i++)
    {
        led_all_on();
        _delay_ms(50);
        led_all_off();
        _delay_ms(50);
    }

    // Hold all LEDs ON - watchdog will reset system here
    led_all_on();
    while (1)
    {
        // Infinite loop - watchdog reset will occur
    }
}

int main(void)
{
    // Save and clear reset flags
    uint8_t mcucsr_save = MCUCSR;
    MCUCSR &= ~(1 << WDRF);
    wdt_disable();

    // Configure status LEDs
    DDRC = 0xFF;
    PORTC = 0x00;

    _delay_ms(100);

    // Check if recovering from watchdog reset
    if (mcucsr_save & (1 << WDRF))
    {
        // Watchdog reset detected - show recovery
        led_blink_fast(15);

        // Show reset count
        show_reset_count();

        led_all_off();
        _delay_ms(1000);
    }
    else
    {
        // Normal boot - single slow blink
        led_all_on();
        _delay_ms(1000);
        led_all_off();
        _delay_ms(500);
    }

    // Clear reset flags
    MCUCSR = 0;

    // Automatic demo sequence
    while (1)
    {
        // Demo 1: Heartbeat monitoring (5 seconds)
        demo1_heartbeat_monitor();
        led_all_off();
        _delay_ms(1000);

        // Demo 2: Critical section (3 seconds)
        demo2_critical_section();
        led_all_off();
        _delay_ms(1000);

        // Demo 3: Graceful degradation (4.5 seconds)
        demo3_graceful_degradation();
        led_all_off();
        _delay_ms(1000);

        // Demo 4: System hang - triggers watchdog reset
        // After reset, loop restarts and shows recovery
        demo4_system_hang();
    }

    return 0;
}
