/*
 * =============================================================================
 * WATCHDOG TIMER - SYSTEM RESET AND RECOVERY (Pure LED Version)
 * =============================================================================
 * PROJECT: Watchdog_System_Reset
 * TARGET: ATmega128 @ 16 MHz
 *
 * LEARNING OBJECTIVES:
 * - Understand watchdog timer operation
 * - Implement system reset protection
 * - Handle watchdog reset recovery
 * - Detect and respond to system hangs
 *
 * HARDWARE:
 * - ATmega128 @ 16 MHz system clock
 * - Watchdog runs on independent ~1 MHz RC oscillator
 * - PORTC LEDs for status indication (8 LEDs)
 *
 * THEORY - WATCHDOG TIMER:
 * Watchdog timer is an independent hardware timer that monitors program execution.
 * If the watchdog is not periodically reset ("fed"), it will force a system reset.
 * This provides automatic recovery from software crashes and infinite loops.
 *
 * Watchdog Timeout Periods (@ ~1 MHz oscillator):
 * - WDP[2:0] = 000: 16.3 ms
 * - WDP[2:0] = 001: 32.5 ms
 * - WDP[2:0] = 010: 65 ms
 * - WDP[2:0] = 011: 130 ms
 * - WDP[2:0] = 100: 260 ms
 * - WDP[2:0] = 101: 520 ms
 * - WDP[2:0] = 110: 1.0 s
 * - WDP[2:0] = 111: 2.1 s
 *
 * SIMULIDE COMPATIBILITY:
 * ⚠️  SimulIDE 1.1.0-SR1: Watchdog NOT fully functional
 * ⚠️  SimulIDE 0.4.15: Watchdog NOT fully functional
 * ✅ Hardware: Fully functional
 * Note: Watchdog timers often not simulated - test on real hardware!
 *
 * LED PATTERNS:
 * - Pattern 1 (Binary count): Normal operation with watchdog
 * - Pattern 2 (Alternating): Watchdog reset countdown
 * - Pattern 3 (Fast blink): System hang detected
 * - Pattern 4 (All ON): Watchdog reset occurred
 *
 * DEMO SEQUENCE:
 * Phase 1: Normal operation (5 seconds, watchdog cleared regularly)
 * Phase 2: Countdown to reset (3 seconds, watchdog NOT cleared)
 * Phase 3: System resets via watchdog
 * Phase 4: Boot detection shows watchdog reset occurred
 * =============================================================================
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/wdt.h>

// Watchdog timeout periods
#define WDT_16MS WDTO_15MS
#define WDT_32MS WDTO_30MS
#define WDT_65MS WDTO_60MS
#define WDT_130MS WDTO_120MS
#define WDT_260MS WDTO_250MS
#define WDT_520MS WDTO_500MS
#define WDT_1S WDTO_1S
#define WDT_2S WDTO_2S

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

void led_pattern_binary(uint8_t value)
{
    PORTC = value;
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

void led_blink_slow(uint8_t times)
{
    for (uint8_t i = 0; i < times; i++)
    {
        led_all_on();
        _delay_ms(500);
        led_all_off();
        _delay_ms(500);
    }
}

/* ========================================================================
 * PHASE 1: Normal Operation with Watchdog Protection
 * ======================================================================== */
void phase1_normal_operation(void)
{
    // Enable watchdog with 3 second timeout
    wdt_enable(WDT_2S);

    // Show normal operation: Binary counting pattern
    // LED pattern indicates normal operation with watchdog cleared regularly
    for (uint8_t i = 0; i < 30; i++)
    {
        led_pattern_binary(i);
        _delay_ms(100);

        // Clear watchdog every iteration (normal operation)
        wdt_reset();
    }
}

/* ========================================================================
 * PHASE 2: Countdown to Watchdog Reset
 * ======================================================================== */
void phase2_countdown_to_reset(void)
{
    // Alternating LED pattern to show countdown
    // Watchdog is NOT cleared - system will reset!
    for (uint8_t i = 0; i < 15; i++)
    {
        PORTC = 0xAA; // Alternating pattern
        _delay_ms(100);
        PORTC = 0x55;
        _delay_ms(100);
        // NOTE: wdt_reset() NOT called - watchdog will timeout!
    }

    // Fast blink to indicate imminent reset
    for (uint8_t i = 0; i < 10; i++)
    {
        led_all_on();
        _delay_ms(50);
        led_all_off();
        _delay_ms(50);
    }

    // Hold LEDs ON - system will reset here
    led_all_on();
    while (1)
    {
        // System will reset via watchdog
    }
}

int main(void)
{
    // =========================================================================
    // CRITICAL STARTUP SEQUENCE
    // =========================================================================
    // IMPORTANT: Disable watchdog immediately at startup!
    // If we're recovering from a watchdog reset, the watchdog may still be
    // enabled and could cause immediate reset if not disabled.
    uint8_t mcucsr_save = MCUCSR; // Save reset flags
    MCUCSR &= ~(1 << WDRF);       // Clear watchdog reset flag
    wdt_disable();                // Disable watchdog

    // Configure status LEDs (PORTC)
    DDRC = 0xFF;  // All pins as output
    PORTC = 0x00; // All LEDs OFF initially

    // Small delay for stability
    _delay_ms(100);

    // =========================================================================
    // PHASE 4: Check if recovering from watchdog reset
    // =========================================================================
    if (mcucsr_save & (1 << WDRF))
    {
        // Watchdog reset detected!
        // Flash all LEDs rapidly to indicate recovery
        led_blink_fast(10);

        // Hold all LEDs ON for 2 seconds
        led_all_on();
        _delay_ms(2000);
        led_all_off();
        _delay_ms(1000);
    }
    else
    {
        // Normal power-on or external reset
        // Single slow blink to indicate normal boot
        led_blink_slow(1);
        _delay_ms(500);
    }

    // Clear all reset flags
    MCUCSR = 0;

    // =========================================================================
    // Main Demo Loop
    // =========================================================================
    while (1)
    {
        // Phase 1: Normal operation with watchdog (3 seconds)
        // LEDs show binary counting pattern
        phase1_normal_operation();

        // Brief pause between phases
        led_all_off();
        _delay_ms(1000);

        // Phase 2: Countdown to reset (3 seconds)
        // LEDs show alternating pattern, then fast blink
        phase2_countdown_to_reset();

        // Phase 3: System will reset via watchdog here
        // After reset, Phase 4 (at startup) will detect watchdog reset
    }

    return 0;
}
