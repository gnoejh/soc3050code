/*
 * =============================================================================
 * POWER MANAGEMENT AND SLEEP MODES - EDUCATIONAL DEMONSTRATION
 * =============================================================================
 *
 * PROJECT: Power_Sleep_Modes
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Educational demonstration of power management techniques and sleep modes.
 * Students learn low-power programming and energy-efficient system design.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master different ATmega128 sleep modes and their applications
 * 2. Learn power consumption reduction techniques
 * 3. Practice wake-up source configuration and management
 * 4. Implement battery-efficient embedded applications
 * 5. Understand real-time clock and timer wake-up systems
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - Current measurement capability (multimeter or oscilloscope)
 * - Wake-up sources: buttons, timers, external interrupts
 * - LEDs for power state indication
 * - Battery power supply for testing
 * - Serial connection for monitoring (9600 baud)
 *
 * SLEEP MODES (ATmega128):
 * - Idle: CPU stopped, peripherals running (~5 mA)
 * - ADC Noise Reduction: CPU + I/O stopped, ADC running
 * - Power-down: Only asynchronous peripherals active (~1-2 µA)
 * - Power-save: Timer2 + watchdog active (~5-10 µA)
 * - Standby: Oscillator running (fast wake-up)
 * - Extended Standby: Timer2 + oscillator active
 *
 * LEARNING PROGRESSION:
 * - Demo 1: Sleep Mode Configuration
 * - Demo 2: Wake-up Source Implementation
 * - Demo 3: Power Consumption Measurement
 * - Demo 4: Battery-Powered Applications
 *
 * =============================================================================
 */
*-External interrupts(INT0 - 7) * -Pin change interrupts
    * -Timer interrupts
    * -UART,
    SPI, I2C interrupts * -Watchdog timer * /

#include "config.h"
#include <avr/sleep.h>

             // Power management modes
             typedef enum {
                 POWER_ACTIVE,
                 POWER_IDLE,
                 POWER_ADC_NOISE_REDUCTION,
                 POWER_POWER_SAVE,
                 POWER_POWER_DOWN,
                 POWER_STANDBY
             } power_mode_t;

// Sleep statistics
typedef struct
{
    uint32_t sleep_count;
    uint32_t wake_count;
    uint16_t total_sleep_cycles;
} sleep_stats_t;

sleep_stats_t stats = {0, 0, 0};

volatile uint8_t wake_flag = 0;
volatile uint16_t timer_ticks = 0;

/*
 * External Interrupt 0 ISR (wake-up from sleep)
 */
ISR(INT0_vect)
{
    wake_flag = 1;
    stats.wake_count++;
    PORTC ^= 0x01; // Toggle LED
}

/*
 * Timer2 Overflow ISR (periodic wake-up)
 */
ISR(TIMER2_OVF_vect)
{
    timer_ticks++;
    wake_flag = 1;
    PORTC ^= 0x02;
}

/*
 * Configure external interrupt for wake-up
 */
void config_wake_interrupt(void)
{
    // INT0 on PD0 - falling edge
    EICRA |= (1 << ISC01); // Falling edge on INT0
    EICRA &= ~(1 << ISC00);
    EIMSK |= (1 << INT0); // Enable INT0

    // Configure pin
    DDRD &= ~(1 << PD0); // Input
    PORTD |= (1 << PD0); // Pull-up
}

/*
 * Configure Timer2 for periodic wake-up
 */
void config_timer2_wakeup(uint8_t prescaler)
{
    // Normal mode, interrupt on overflow
    TCCR2 = prescaler;     // Set prescaler
    TIMSK |= (1 << TOIE2); // Enable overflow interrupt
    TCNT2 = 0;             // Reset counter
}

/*
 * Enter specified sleep mode
 */
void enter_sleep_mode(power_mode_t mode)
{
    // Set sleep mode
    switch (mode)
    {
    case POWER_IDLE:
        set_sleep_mode(SLEEP_MODE_IDLE);
        break;
    case POWER_ADC_NOISE_REDUCTION:
        set_sleep_mode(SLEEP_MODE_ADC);
        break;
    case POWER_POWER_SAVE:
        set_sleep_mode(SLEEP_MODE_PWR_SAVE);
        break;
    case POWER_POWER_DOWN:
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        break;
    case POWER_STANDBY:
        set_sleep_mode(SLEEP_MODE_STANDBY);
        break;
    default:
        return;
    }

    stats.sleep_count++;

    // Enable sleep and enter
    sleep_enable();
    sei();
    sleep_cpu();
    sleep_disable();
}

/* ========================================================================
 * DEMO 1: Sleep Mode Comparison
 * ======================================================================== */
void demo1_sleep_modes(void)
{
    puts_USART1("\r\n=== DEMO 1: Sleep Mode Comparison ===\r\n");
    puts_USART1("Comparing different sleep modes\r\n");
    puts_USART1("Connect button to PD0 (INT0) to wake up\r\n\r\n");

    config_wake_interrupt();

    const char *mode_names[] = {
        "IDLE",
        "ADC Noise Reduction",
        "Power-Save",
        "Power-Down",
        "Standby"};

    const power_mode_t modes[] = {
        POWER_IDLE,
        POWER_ADC_NOISE_REDUCTION,
        POWER_POWER_SAVE,
        POWER_POWER_DOWN,
        POWER_STANDBY};

    puts_USART1("Testing each sleep mode...\r\n\r\n");

    for (uint8_t i = 0; i < 5; i++)
    {
        char buf[80];
        sprintf(buf, "Mode %u: %s\r\n", i + 1, mode_names[i]);
        puts_USART1(buf);
        puts_USART1("  Press button to wake (or wait 5s)...\r\n");

        wake_flag = 0;
        PORTC = 0xFF;
        _delay_ms(500);
        PORTC = 0x00;

        // Enter sleep mode
        puts_USART1("  Entering sleep mode...\r\n");
        _delay_ms(100);

        uint16_t sleep_start = timer_ticks;
        enter_sleep_mode(modes[i]);
        uint16_t sleep_duration = timer_ticks - sleep_start;

        puts_USART1("  ✓ Wake-up!\r\n");
        PORTC = 0xFF;
        _delay_ms(200);
        PORTC = 0x00;

        sprintf(buf, "  Sleep duration: ~%u ticks\r\n\r\n", sleep_duration);
        puts_USART1(buf);

        _delay_ms(1000);
    }

    // Disable interrupt
    EIMSK &= ~(1 << INT0);

    puts_USART1("\r\nSleep Mode Test Complete!\r\n");

    char buf[60];
    sprintf(buf, "Total sleeps: %lu\r\n", stats.sleep_count);
    puts_USART1(buf);
    sprintf(buf, "Total wakes: %lu\r\n", stats.wake_count);
    puts_USART1(buf);
}

/* ========================================================================
 * DEMO 2: Periodic Sleep with Timer Wake-up
 * ======================================================================== */
void demo2_periodic_sleep(void)
{
    puts_USART1("\r\n=== DEMO 2: Periodic Sleep Wake-up ===\r\n");
    puts_USART1("Using Timer2 for periodic wake-up\r\n");
    puts_USART1("Press any key to stop\r\n\r\n");

    // Configure Timer2 for ~1 second wake-up
    // At 7.3728MHz, prescaler 1024, overflow ~28Hz
    config_timer2_wakeup((1 << CS22) | (1 << CS21) | (1 << CS20));

    timer_ticks = 0;
    uint8_t wake_count = 0;

    puts_USART1("Starting periodic sleep cycle...\r\n\r\n");

    while (1)
    {
        // Do some work
        wake_flag = 0;

        char buf[60];
        sprintf(buf, "\rWake #%u  Timer: %u  ", wake_count + 1, timer_ticks);
        puts_USART1(buf);

        // Visual indicator
        PORTC = wake_count & 0xFF;

        // Simulate brief active period
        _delay_ms(100);

        // Check for user input
        if (UCSR1A & (1 << RXC1))
        {
            getch_USART1();
            break;
        }

        // Enter power-save mode (Timer2 keeps running)
        enter_sleep_mode(POWER_POWER_SAVE);

        wake_count++;

        // Limit iterations for demonstration
        if (wake_count >= 30)
        {
            break;
        }
    }

    // Disable Timer2
    TCCR2 = 0;
    TIMSK &= ~(1 << TOIE2);

    puts_USART1("\r\n\r\nPeriodic sleep stopped.\r\n");

    char buf[60];
    sprintf(buf, "Wake-ups: %u\r\n", wake_count);
    puts_USART1(buf);
    sprintf(buf, "Timer ticks: %u\r\n", timer_ticks);
    puts_USART1(buf);

    PORTC = 0x00;
}

/* ========================================================================
 * DEMO 3: Battery-Powered Operation Simulation
 * ======================================================================== */
void demo3_battery_operation(void)
{
    puts_USART1("\r\n=== DEMO 3: Battery-Powered Operation ===\r\n");
    puts_USART1("Simulating ultra-low power device\r\n");
    puts_USART1("Connect button to PD0 for wake-up\r\n\r\n");

    config_wake_interrupt();
    config_timer2_wakeup((1 << CS22) | (1 << CS21) | (1 << CS20));

    uint16_t battery_level = 1000; // Simulated battery (10.00V)
    uint8_t measurement_count = 0;

    puts_USART1("Ultra-low power sensor node starting...\r\n");
    puts_USART1("Press button for immediate reading\r\n");
    puts_USART1("Or wait for periodic measurements\r\n\r\n");

    timer_ticks = 0;

    while (battery_level > 600 && measurement_count < 20)
    {
        wake_flag = 0;

        // Take measurement
        uint16_t sensor_reading = (rand() % 100) + 200; // Simulated sensor

        char buf[80];
        sprintf(buf, "[%u] Sensor: %u  Battery: %u.%02uV  ",
                measurement_count + 1,
                sensor_reading,
                battery_level / 100,
                battery_level % 100);
        puts_USART1(buf);

        // Simulate battery drain based on activity
        if (wake_flag)
        {
            battery_level -= 5; // Active drain
            puts_USART1("(active)\r\n");
        }
        else
        {
            battery_level -= 1; // Sleep drain
            puts_USART1("(sleep)\r\n");
        }

        measurement_count++;

        // Flash LED briefly
        PORTC = 0xFF;
        _delay_ms(50);
        PORTC = 0x00;

        // Enter deep sleep between measurements
        puts_USART1("Entering deep sleep...\r\n");
        _delay_ms(50);

        enter_sleep_mode(POWER_POWER_DOWN);

        puts_USART1("Wake-up! ");

        _delay_ms(500);
    }

    // Disable interrupts
    EIMSK &= ~(1 << INT0);
    TCCR2 = 0;
    TIMSK &= ~(1 << TOIE2);

    puts_USART1("\r\n\r\nBattery operation simulation complete.\r\n");

    char buf[80];
    sprintf(buf, "Measurements taken: %u\r\n", measurement_count);
    puts_USART1(buf);
    sprintf(buf, "Final battery: %u.%02uV\r\n",
            battery_level / 100, battery_level % 100);
    puts_USART1(buf);
    sprintf(buf, "Energy used: %u.%02uV\r\n",
            (1000 - battery_level) / 100,
            (1000 - battery_level) % 100);
    puts_USART1(buf);

    PORTC = 0x00;
}

/* ========================================================================
 * DEMO 4: Smart Sleep Management
 * ======================================================================== */
void demo4_smart_sleep(void)
{
    puts_USART1("\r\n=== DEMO 4: Smart Sleep Management ===\r\n");
    puts_USART1("Adaptive power management system\r\n");
    puts_USART1("Press any key to stop\r\n\r\n");

    config_timer2_wakeup((1 << CS22) | (1 << CS21) | (1 << CS20));

    uint16_t idle_time = 0;
    power_mode_t current_mode = POWER_ACTIVE;

    puts_USART1("Smart sleep manager started...\r\n\r\n");

    for (uint16_t cycle = 0; cycle < 100; cycle++)
    {
        // Simulate varying activity
        uint8_t activity = (cycle % 10) < 3 ? 1 : 0; // 30% activity

        if (activity)
        {
            idle_time = 0;
            current_mode = POWER_ACTIVE;
        }
        else
        {
            idle_time++;

            // Adaptive sleep mode selection
            if (idle_time > 20)
            {
                current_mode = POWER_POWER_DOWN;
            }
            else if (idle_time > 10)
            {
                current_mode = POWER_POWER_SAVE;
            }
            else if (idle_time > 5)
            {
                current_mode = POWER_IDLE;
            }
        }

        char buf[80];
        sprintf(buf, "\rCycle %u: ", cycle + 1);
        puts_USART1(buf);

        if (activity)
        {
            puts_USART1("ACTIVE      ");
            PORTC = 0xFF;
        }
        else
        {
            sprintf(buf, "SLEEP (idle:%u) ", idle_time);
            puts_USART1(buf);

            switch (current_mode)
            {
            case POWER_IDLE:
                puts_USART1("[IDLE]      ");
                PORTC = 0x01;
                break;
            case POWER_POWER_SAVE:
                puts_USART1("[SAVE]      ");
                PORTC = 0x03;
                break;
            case POWER_POWER_DOWN:
                puts_USART1("[DOWN]      ");
                PORTC = 0x07;
                break;
            default:
                PORTC = 0x00;
                break;
            }

            // Enter appropriate sleep mode
            if (current_mode != POWER_ACTIVE)
            {
                enter_sleep_mode(current_mode);
            }
        }

        _delay_ms(200);

        // Check for user input
        if (UCSR1A & (1 << RXC1))
        {
            getch_USART1();
            break;
        }
    }

    // Disable Timer2
    TCCR2 = 0;
    TIMSK &= ~(1 << TOIE2);

    puts_USART1("\r\n\r\nSmart sleep manager stopped.\r\n");

    char buf[60];
    sprintf(buf, "Total sleep cycles: %lu\r\n", stats.sleep_count);
    puts_USART1(buf);

    PORTC = 0x00;
}

/* ========================================================================
 * Main Menu System
 * ======================================================================== */
void display_main_menu(void)
{
    puts_USART1("\r\n\r\n");
    puts_USART1("╔════════════════════════════════════════╗\r\n");
    puts_USART1("║  Sleep Modes - ATmega128              ║\r\n");
    puts_USART1("╚════════════════════════════════════════╝\r\n");
    puts_USART1("\r\n");
    puts_USART1("Select Demo:\r\n");
    puts_USART1("  [1] Sleep Mode Comparison\r\n");
    puts_USART1("  [2] Periodic Sleep Wake-up\r\n");
    puts_USART1("  [3] Battery Operation Simulation\r\n");
    puts_USART1("  [4] Smart Sleep Management\r\n");
    puts_USART1("\r\n");
    puts_USART1("Enter selection (1-4): ");
}

int main(void)
{
    // Initialize peripherals
    Uart1_init();

    // Configure status LEDs
    DDRC = 0xFF;
    PORTC = 0x00;

    // Enable global interrupts
    sei();

    // Send startup message
    _delay_ms(500);
    puts_USART1("\r\n\r\n*** Sleep Modes & Power Management ***\r\n");
    puts_USART1("Ultra-Low Power Operation\r\n");

    PORTC = 0x01;
    _delay_ms(1000);
    PORTC = 0x00;

    while (1)
    {
        display_main_menu();

        char choice = getch_USART1();
        putch_USART1(choice);
        puts_USART1("\r\n");

        switch (choice)
        {
        case '1':
            demo1_sleep_modes();
            break;
        case '2':
            demo2_periodic_sleep();
            break;
        case '3':
            demo3_battery_operation();
            break;
        case '4':
            demo4_smart_sleep();
            break;
        default:
            puts_USART1("Invalid selection!\r\n");
            _delay_ms(1000);
            break;
        }

        _delay_ms(500);
    }

    return 0;
}
