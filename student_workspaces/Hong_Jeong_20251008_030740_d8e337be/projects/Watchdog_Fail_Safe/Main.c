/*
 * =============================================================================
 * WATCHDOG FAIL-SAFE OPERATION - EDUCATIONAL DEMONSTRATION
 * =============================================================================
 *
 * PROJECT: Watchdog_Fail_Safe
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Educational demonstration of robust fail-safe systems using watchdog timer.
 * Students learn fault-tolerant programming and critical system monitoring.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master fail-safe system design principles
 * 2. Learn critical task monitoring and heartbeat systems
 * 3. Practice error recovery and graceful degradation
 * 4. Implement fault-tolerant embedded applications
 * 5. Understand safety-critical system requirements
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - Critical system components (sensors, actuators)
 * - Status indication LEDs (system health, error states)
 * - Manual intervention switches
 * - Backup power monitoring circuits
 * - Serial connection for system monitoring (9600 baud)
 *
 * FAIL-SAFE CONCEPTS:
 * - Watchdog as last line of defense
 * - Critical section protection and monitoring
 * - Graceful degradation strategies
 * - Recovery from known bad states
 * - Heartbeat monitoring systems
 *
 * APPLICATION SCENARIOS:
 * - Critical control systems (medical devices)
 * - Unattended operations (remote monitoring)
 * - Safety-critical applications (automotive, aerospace)
 * - Long-running embedded systems (IoT devices)
 *
 * LEARNING PROGRESSION:
 * - Demo 1: Critical Task Monitoring
 * - Demo 2: Error Detection and Recovery
 * - Demo 3: Graceful Degradation
 * - Demo 4: Fault-Tolerant System Design
 *
 * =============================================================================
 */

#include "config.h"
#include <avr/wdt.h>
#include <avr/eeprom.h>

// Watchdog timeouts (using WDP bits for prescaler)
#define WDT_TIMEOUT_NORMAL ((1 << WDP2) | (1 << WDP0))                 // ~1 second
#define WDT_TIMEOUT_CRITICAL ((1 << WDP2) | (1 << WDP1) | (1 << WDP0)) // ~2 seconds

// EEPROM addresses for persistent data
#define EEPROM_BOOT_COUNT 0
#define EEPROM_CRASH_COUNT 1
#define EEPROM_LAST_ERROR 2
#define EEPROM_RECOVERY_MODE 3

// System states
typedef enum
{
    STATE_INIT,
    STATE_NORMAL,
    STATE_WARNING,
    STATE_CRITICAL,
    STATE_RECOVERY
} system_state_t;

// Error codes
typedef enum
{
    ERROR_NONE = 0,
    ERROR_SENSOR_TIMEOUT,
    ERROR_COMM_FAILURE,
    ERROR_TASK_OVERRUN,
    ERROR_MEMORY_CORRUPT,
    ERROR_UNKNOWN_RESET
} error_code_t;

// System health tracking
typedef struct
{
    system_state_t state;
    error_code_t last_error;
    uint16_t heartbeat_counter;
    uint8_t task_watchdog[4]; // Per-task watchdog counters
    uint8_t recovery_attempts;
} system_health_t;

system_health_t health = {STATE_INIT, ERROR_NONE, 0, {0}, 0};

/*
 * Save system state to EEPROM
 */
void save_system_state(void)
{
    static uint8_t boot_count;
    boot_count = eeprom_read_byte((uint8_t *)EEPROM_BOOT_COUNT);
    boot_count++;
    eeprom_write_byte((uint8_t *)EEPROM_BOOT_COUNT, boot_count);

    if (health.last_error != ERROR_NONE)
    {
        uint8_t crash_count = eeprom_read_byte((uint8_t *)EEPROM_CRASH_COUNT);
        crash_count++;
        eeprom_write_byte((uint8_t *)EEPROM_CRASH_COUNT, crash_count);
        eeprom_write_byte((uint8_t *)EEPROM_LAST_ERROR, health.last_error);
    }
}

/*
 * Load system state from EEPROM
 */
void load_system_state(void)
{
    uint8_t boot_count = eeprom_read_byte((uint8_t *)EEPROM_BOOT_COUNT);
    uint8_t crash_count = eeprom_read_byte((uint8_t *)EEPROM_CRASH_COUNT);
    uint8_t last_error = eeprom_read_byte((uint8_t *)EEPROM_LAST_ERROR);

    char buf[80];
    sprintf(buf, "Boot count: %u\r\n", boot_count);
    puts_USART1(buf);
    sprintf(buf, "Crash count: %u\r\n", crash_count);
    puts_USART1(buf);

    if (last_error != ERROR_NONE)
    {
        sprintf(buf, "Last error code: %u\r\n", last_error);
        puts_USART1(buf);
    }
}

/*
 * Initialize watchdog with fail-safe configuration
 */
void watchdog_failsafe_init(void)
{
    cli();
    wdt_reset();

    WDTCR = (1 << WDCE) | (1 << WDE);
    WDTCR = (1 << WDE) | WDT_TIMEOUT_NORMAL;

    sei();
}

/*
 * Check system health and update state
 */
void check_system_health(void)
{
    // Check task watchdogs
    uint8_t tasks_ok = 1;
    for (uint8_t i = 0; i < 4; i++)
    {
        if (health.task_watchdog[i] > 10)
        {
            tasks_ok = 0;
            health.last_error = ERROR_TASK_OVERRUN;
        }
    }

    if (!tasks_ok)
    {
        health.state = STATE_WARNING;
        PORTC |= 0xF0; // Warning LEDs
    }
    else if (health.state == STATE_WARNING)
    {
        health.state = STATE_NORMAL;
        PORTC &= 0x0F;
    }
}

/* ========================================================================
 * DEMO 1: Heartbeat Monitoring System
 * ======================================================================== */
void demo1_heartbeat_monitor(void)
{
    puts_USART1("\r\n=== DEMO 1: Heartbeat Monitoring ===\r\n");
    puts_USART1("Monitoring critical task execution\r\n");
    puts_USART1("Press any key to stop\r\n\r\n");

    watchdog_failsafe_init();

    health.state = STATE_NORMAL;
    health.heartbeat_counter = 0;

    puts_USART1("System tasks running with watchdog protection...\r\n\r\n");

    while (1)
    {
        // Simulate Task 1 - Sensor Reading
        health.task_watchdog[0] = 0;
        _delay_ms(50);

        // Simulate Task 2 - Data Processing
        health.task_watchdog[1] = 0;
        _delay_ms(50);

        // Simulate Task 3 - Communication
        health.task_watchdog[2] = 0;
        _delay_ms(50);

        // Simulate Task 4 - Display Update
        health.task_watchdog[3] = 0;
        _delay_ms(50);

        // Increment all task watchdogs
        for (uint8_t i = 0; i < 4; i++)
        {
            health.task_watchdog[i]++;
        }

        // Check system health
        check_system_health();

        // Heartbeat
        health.heartbeat_counter++;

        char buf[60];
        sprintf(buf, "\rHeartbeat: %u  State: ", health.heartbeat_counter);
        puts_USART1(buf);

        switch (health.state)
        {
        case STATE_NORMAL:
            puts_USART1("NORMAL  ");
            break;
        case STATE_WARNING:
            puts_USART1("WARNING ");
            break;
        case STATE_CRITICAL:
            puts_USART1("CRITICAL");
            break;
        default:
            puts_USART1("UNKNOWN ");
            break;
        }

        // Reset watchdog (CRITICAL)
        wdt_reset();

        // Heartbeat LED
        PORTC = (PORTC & 0xF0) | (health.heartbeat_counter & 0x0F);

        // Check for user input
        if (UCSR1A & (1 << RXC1))
        {
            getch_USART1();
            wdt_disable();

            puts_USART1("\r\n\r\nMonitoring stopped.\r\n");
            sprintf(buf, "Total heartbeats: %u\r\n", health.heartbeat_counter);
            puts_USART1(buf);

            return;
        }

        _delay_ms(100);
    }
}

/* ========================================================================
 * DEMO 2: Critical Section Protection
 * ======================================================================== */
void demo2_critical_section(void)
{
    puts_USART1("\r\n=== DEMO 2: Critical Section Protection ===\r\n");
    puts_USART1("Protecting time-critical operations\r\n\r\n");

    watchdog_failsafe_init();

    puts_USART1("Select operation:\r\n");
    puts_USART1("  [1] Normal operation (completes in time)\r\n");
    puts_USART1("  [2] Slow operation (may timeout)\r\n");
    puts_USART1("  [3] Hung operation (will trigger watchdog)\r\n");
    puts_USART1("Enter choice: ");

    char choice = getch_USART1();
    putch_USART1(choice);
    puts_USART1("\r\n\r\n");

    if (choice == '1')
    {
        // Normal operation - completes quickly
        puts_USART1("Executing normal critical section...\r\n");

        for (uint8_t i = 0; i < 5; i++)
        {
            char buf[50];
            sprintf(buf, "  Critical task %u/5...\r\n", i + 1);
            puts_USART1(buf);

            PORTC = (i + 1) * 51;
            _delay_ms(100);

            // Reset watchdog during operation
            wdt_reset();
        }

        puts_USART1("Critical section completed successfully!\r\n");
        PORTC = 0xFF;
        _delay_ms(500);
    }
    else if (choice == '2')
    {
        // Slow operation - pushing the limits
        puts_USART1("Executing slow critical section...\r\n");
        puts_USART1("WARNING: Operation is near timeout limit!\r\n\r\n");

        for (uint8_t i = 0; i < 8; i++)
        {
            char buf[50];
            sprintf(buf, "  Slow task %u/8 (%.1fs)...\r\n", i + 1, (i + 1) * 0.3);
            puts_USART1(buf);

            PORTC = (i + 1) * 32;
            _delay_ms(300);

            // Reset watchdog frequently
            wdt_reset();
        }

        puts_USART1("Slow section completed (barely made it)!\r\n");
    }
    else if (choice == '3')
    {
        // Hung operation - will trigger watchdog
        puts_USART1("Executing hung critical section...\r\n");
        puts_USART1("ERROR: This operation will hang!\r\n");
        puts_USART1("Watchdog will reset system...\r\n\r\n");

        health.last_error = ERROR_TASK_OVERRUN;
        save_system_state();

        _delay_ms(500);

        puts_USART1("Entering infinite loop (simulating hang)...\r\n\r\n");

        // Simulate hung task - no watchdog reset
        while (1)
        {
            PORTC = 0xFF;
            _delay_ms(100);
            PORTC = 0x00;
            _delay_ms(100);

            puts_USART1("HUNG! ");
        }
    }

    wdt_disable();
    PORTC = 0x00;
}

/* ========================================================================
 * DEMO 3: Graceful Degradation
 * ======================================================================== */
void demo3_graceful_degradation(void)
{
    puts_USART1("\r\n=== DEMO 3: Graceful Degradation ===\r\n");
    puts_USART1("System continues with reduced functionality\r\n");
    puts_USART1("Press any key to stop\r\n\r\n");

    watchdog_failsafe_init();

    health.state = STATE_NORMAL;
    uint8_t sensor_failures = 0;

    puts_USART1("Starting multi-sensor system...\r\n\r\n");

    for (uint16_t cycle = 0; cycle < 100; cycle++)
    {
        // Simulate sensor readings with occasional failures
        uint8_t sensor1_ok = (cycle % 7) != 0;
        uint8_t sensor2_ok = (cycle % 11) != 0;
        uint8_t sensor3_ok = (cycle % 13) != 0;

        char buf[80];
        sprintf(buf, "\rCycle %u: S1:%s S2:%s S3:%s  ",
                cycle + 1,
                sensor1_ok ? "OK" : "FAIL",
                sensor2_ok ? "OK" : "FAIL",
                sensor3_ok ? "OK" : "FAIL");
        puts_USART1(buf);

        // Count failures
        if (!sensor1_ok || !sensor2_ok || !sensor3_ok)
        {
            sensor_failures++;
        }

        // Determine system state based on sensor health
        if (sensor1_ok && sensor2_ok && sensor3_ok)
        {
            health.state = STATE_NORMAL;
            PORTC = 0x01; // Green
        }
        else if ((sensor1_ok && sensor2_ok) ||
                 (sensor2_ok && sensor3_ok) ||
                 (sensor1_ok && sensor3_ok))
        {
            health.state = STATE_WARNING;
            PORTC = 0x03; // Yellow

            if (health.state != STATE_WARNING)
            {
                puts_USART1(" [DEGRADED MODE]");
            }
        }
        else
        {
            health.state = STATE_CRITICAL;
            PORTC = 0x07; // Red

            if (health.state != STATE_CRITICAL)
            {
                puts_USART1(" [CRITICAL: Minimal function]");
            }
        }

        // Reset watchdog
        wdt_reset();

        _delay_ms(200);

        if (UCSR1A & (1 << RXC1))
        {
            getch_USART1();
            break;
        }
    }

    wdt_disable();

    puts_USART1("\r\n\r\nSystem Statistics:\r\n");
    char buf[60];
    sprintf(buf, "  Sensor failures: %u\r\n", sensor_failures);
    puts_USART1(buf);
    sprintf(buf, "  Final state: ");
    puts_USART1(buf);

    switch (health.state)
    {
    case STATE_NORMAL:
        puts_USART1("NORMAL\r\n");
        break;
    case STATE_WARNING:
        puts_USART1("WARNING (Degraded)\r\n");
        break;
    case STATE_CRITICAL:
        puts_USART1("CRITICAL (Minimal)\r\n");
        break;
    default:
        break;
    }

    PORTC = 0x00;
}

/* ========================================================================
 * DEMO 4: Recovery Strategy
 * ======================================================================== */
void demo4_recovery_strategy(void)
{
    puts_USART1("\r\n=== DEMO 4: Recovery Strategy ===\r\n");
    puts_USART1("Demonstrating error recovery\r\n\r\n");

    // Load previous state
    load_system_state();

    puts_USART1("\r\nSelect scenario:\r\n");
    puts_USART1("  [1] Safe mode boot\r\n");
    puts_USART1("  [2] Full recovery test\r\n");
    puts_USART1("  [3] Reset error counters\r\n");
    puts_USART1("Enter choice: ");

    char choice = getch_USART1();
    putch_USART1(choice);
    puts_USART1("\r\n\r\n");

    if (choice == '1')
    {
        // Safe mode
        puts_USART1("Booting in SAFE MODE...\r\n");
        puts_USART1("- Watchdog enabled with long timeout\r\n");
        puts_USART1("- Non-essential features disabled\r\n");
        puts_USART1("- Diagnostic mode active\r\n\r\n");

        cli();
        wdt_reset();
        WDTCR = (1 << WDCE) | (1 << WDE);
        WDTCR = (1 << WDE) | (7); // 2 second timeout
        sei();

        for (uint8_t i = 0; i < 20; i++)
        {
            char buf[50];
            sprintf(buf, "\rSafe mode running: %u/20  ", i + 1);
            puts_USART1(buf);

            PORTC = 0x01 << (i % 8);

            _delay_ms(500);
            wdt_reset();
        }

        puts_USART1("\r\n\r\nSafe mode test complete.\r\n");
        wdt_disable();
    }
    else if (choice == '2')
    {
        // Full recovery
        puts_USART1("Initiating full system recovery...\r\n\r\n");

        const char *recovery_steps[] = {
            "Checking hardware integrity",
            "Verifying memory",
            "Restoring default settings",
            "Reinitializing peripherals",
            "Running self-test",
            "Recovery complete"};

        watchdog_failsafe_init();

        for (uint8_t step = 0; step < 6; step++)
        {
            char buf[80];
            sprintf(buf, "[%u/6] %s...\r\n", step + 1, recovery_steps[step]);
            puts_USART1(buf);

            PORTC = (step + 1) * 42;

            _delay_ms(800);
            wdt_reset();
        }

        puts_USART1("\r\n✓ System recovered successfully!\r\n");

        // Clear error state
        eeprom_write_byte((uint8_t *)EEPROM_LAST_ERROR, ERROR_NONE);

        wdt_disable();
        PORTC = 0xFF;
        _delay_ms(1000);
        PORTC = 0x00;
    }
    else if (choice == '3')
    {
        // Reset counters
        puts_USART1("Resetting error counters...\r\n");

        eeprom_write_byte((uint8_t *)EEPROM_CRASH_COUNT, 0);
        eeprom_write_byte((uint8_t *)EEPROM_LAST_ERROR, ERROR_NONE);

        puts_USART1("Error counters cleared.\r\n");

        PORTC = 0xFF;
        _delay_ms(500);
        PORTC = 0x00;
    }
}

/* ========================================================================
 * Main Menu System
 * ======================================================================== */
void display_main_menu(void)
{
    puts_USART1("\r\n\r\n");
    puts_USART1("╔════════════════════════════════════════╗\r\n");
    puts_USART1("║  Watchdog Fail-Safe - ATmega128       ║\r\n");
    puts_USART1("╚════════════════════════════════════════╝\r\n");
    puts_USART1("\r\n");
    puts_USART1("Select Demo:\r\n");
    puts_USART1("  [1] Heartbeat Monitoring\r\n");
    puts_USART1("  [2] Critical Section Protection\r\n");
    puts_USART1("  [3] Graceful Degradation\r\n");
    puts_USART1("  [4] Recovery Strategy\r\n");
    puts_USART1("\r\n");
    puts_USART1("Enter selection (1-4): ");
}

int main(void)
{
    // Disable watchdog at startup
    MCUCSR &= ~(1 << WDRF);
    wdt_disable();

    // Initialize peripherals
    Uart1_init();

    // Configure status LEDs
    DDRC = 0xFF;
    PORTC = 0x00;

    // Send startup message
    _delay_ms(500);
    puts_USART1("\r\n\r\n*** Watchdog Fail-Safe System ***\r\n");
    puts_USART1("Robust Error Recovery\r\n\r\n");

    // Check reset source
    if (MCUCSR & (1 << WDRF))
    {
        puts_USART1("⚠ RECOVERED FROM WATCHDOG RESET!\r\n");
        health.recovery_attempts++;

        PORTC = 0xFF;
        _delay_ms(1000);
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
            demo1_heartbeat_monitor();
            break;
        case '2':
            demo2_critical_section();
            break;
        case '3':
            demo3_graceful_degradation();
            break;
        case '4':
            demo4_recovery_strategy();
            break;
        default:
            puts_USART1("Invalid selection!\r\n");
            _delay_ms(1000);
            break;
        }

        // Ensure watchdog is disabled between demos
        wdt_disable();

        _delay_ms(500);
    }

    return 0;
}
