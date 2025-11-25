/*
 * Wake-up Sources and Power Optimization
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Master all wake-up sources
 * - Optimize power consumption
 * - Implement event-driven architecture
 * - Practice advanced power techniques
 *
 * WAKE-UP SOURCES (ATmega128):
 * - External Interrupts (INT0-INT7)
 * - Pin Change Interrupts
 * - Timer Interrupts (Timer0/1/2/3)
 * - UART RX Complete
 * - SPI Transfer Complete
 * - TWI Address Match
 * - ADC Conversion Complete
 * - Watchdog Timer
 *
 * OPTIMIZATION STRATEGIES:
 * - Use appropriate sleep mode per scenario
 * - Minimize active time
 * - Disable unused peripherals
 * - Use internal pull-ups
 * - Optimize clock prescaler
 * - BOD (Brown-Out Detection) settings
 *
 * POWER BUDGET EXAMPLE:
 * - Active 1ms every 1s: ~15 mA × 0.001 = 15 µA avg
 * - Sleep 999ms: ~2 µA × 0.999 = 2 µA avg
 * - Total average: ~17 µA
 * - Battery life (CR2032 220mAh): ~1.4 years
 */

#include "config.h"
#include <avr/sleep.h>
#include <avr/wdt.h>

// Inline UART functions (not using _uart.c to avoid ISR conflict)
void Uart1_init(void)
{
    UBRR1H = 0;
    UBRR1L = 47; // 9600 baud at 7.3728 MHz
    UCSR1B = (1 << RXEN1) | (1 << TXEN1);
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
}

void putch_USART1(char data)
{
    while (!(UCSR1A & (1 << UDRE1)))
        ;
    UDR1 = data;
}

void puts_USART1(const char *str)
{
    while (*str)
    {
        putch_USART1(*str++);
    }
}

char getch_USART1(void)
{
    while (!(UCSR1A & (1 << RXC1)))
        ;
    return UDR1;
}

// Wake-up source tracking
typedef enum
{
    WAKE_NONE = 0,
    WAKE_INT0,
    WAKE_INT1,
    WAKE_TIMER0,
    WAKE_TIMER2,
    WAKE_UART,
    WAKE_ADC,
    WAKE_WATCHDOG
} wake_source_t;

volatile wake_source_t last_wake_source = WAKE_NONE;
volatile uint16_t wake_count[8] = {0};

// Power statistics
typedef struct
{
    uint32_t total_sleep_ms;
    uint32_t total_active_ms;
    uint16_t wake_events;
} power_stats_t;

power_stats_t power_stats = {0, 0, 0};

/*
 * External Interrupt 0 ISR
 */
ISR(INT0_vect)
{
    last_wake_source = WAKE_INT0;
    wake_count[WAKE_INT0]++;
}

/*
 * External Interrupt 1 ISR
 */
ISR(INT1_vect)
{
    last_wake_source = WAKE_INT1;
    wake_count[WAKE_INT1]++;
}

/*
 * Timer0 Overflow ISR
 */
ISR(TIMER0_OVF_vect)
{
    last_wake_source = WAKE_TIMER0;
    wake_count[WAKE_TIMER0]++;
}

/*
 * Timer2 Overflow ISR
 */
ISR(TIMER2_OVF_vect)
{
    last_wake_source = WAKE_TIMER2;
    wake_count[WAKE_TIMER2]++;
}

/*
 * UART RX Complete ISR - Custom for wake-up demo
 * Note: This conflicts with _uart.c default ISR
 * For this demo, we track wake-ups from UART
 */
ISR(USART1_RX_vect)
{
    last_wake_source = WAKE_UART;
    wake_count[WAKE_UART]++;

    // Read and discard to clear flag
    volatile uint8_t dummy = UDR1;
    (void)dummy;
}

/*
 * ADC Conversion Complete ISR
 */
ISR(ADC_vect)
{
    last_wake_source = WAKE_ADC;
    wake_count[WAKE_ADC]++;
}

/*
 * Get wake source name
 */
const char *get_wake_source_name(wake_source_t source)
{
    switch (source)
    {
    case WAKE_INT0:
        return "INT0";
    case WAKE_INT1:
        return "INT1";
    case WAKE_TIMER0:
        return "TIMER0";
    case WAKE_TIMER2:
        return "TIMER2";
    case WAKE_UART:
        return "UART";
    case WAKE_ADC:
        return "ADC";
    case WAKE_WATCHDOG:
        return "WATCHDOG";
    default:
        return "UNKNOWN";
    }
}

/*
 * Configure external interrupts
 */
void config_external_interrupts(void)
{
    // INT0 on PD0 - falling edge
    EICRA |= (1 << ISC01);
    EICRA &= ~(1 << ISC00);
    EIMSK |= (1 << INT0);

    // INT1 on PD1 - rising edge
    EICRA |= (1 << ISC11) | (1 << ISC10);
    EIMSK |= (1 << INT1);

    // Configure pins with pull-ups
    DDRD &= ~((1 << PD0) | (1 << PD1));
    PORTD |= (1 << PD0) | (1 << PD1);
}

/*
 * Configure Timer0 for periodic wake-up
 */
void config_timer0_wakeup(void)
{
    // Normal mode, prescaler 1024
    TCCR0 = (1 << CS02) | (1 << CS00);
    TIMSK |= (1 << TOIE0);
    TCNT0 = 0;
}

/*
 * Configure Timer2 for RTC-like operation
 */
void config_timer2_rtc(void)
{
    // Asynchronous mode with 32.768 kHz crystal (if available)
    // For demo, use normal mode with prescaler
    TCCR2 = (1 << CS22) | (1 << CS21) | (1 << CS20);
    TIMSK |= (1 << TOIE2);
    TCNT2 = 0;
}

/*
 * Optimize power consumption
 */
void optimize_power(void)
{
    // Disable analog comparator
    ACSR |= (1 << ACD);

    // Note: ATmega128 doesn't have PRR register
    // Power reduction is achieved through sleep modes

    puts_USART1("Power optimizations applied\r\n");
}

/* ========================================================================
 * DEMO 1: External Interrupt Wake-up
 * ======================================================================== */
void demo1_external_interrupts(void)
{
    puts_USART1("\r\n=== DEMO 1: External Interrupt Wake-up ===\r\n");
    puts_USART1("Wake from sleep using external interrupts\r\n");
    puts_USART1("Connect buttons to PD0 (INT0) and PD1 (INT1)\r\n");
    puts_USART1("Press any key (UART) to stop\r\n\r\n");

    config_external_interrupts();
    optimize_power();

    uint16_t sleep_cycle = 0;

    puts_USART1("System sleeping... Press buttons to wake\r\n\r\n");

    while (sleep_cycle < 20)
    {
        last_wake_source = WAKE_NONE;

        // Flash LED before sleep
        PORTC = 0x01;
        _delay_ms(50);
        PORTC = 0x00;

        // Enter power-down sleep
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sleep_enable();
        sei();
        sleep_cpu();
        sleep_disable();

        // Wake-up occurred
        power_stats.wake_events++;

        if (last_wake_source != WAKE_NONE)
        {
            char buf[80];
            sprintf(buf, "[Wake #%u] Source: %-10s  Count: %u\r\n",
                    sleep_cycle + 1,
                    get_wake_source_name(last_wake_source),
                    wake_count[last_wake_source]);
            puts_USART1(buf);

            // Visual feedback
            if (last_wake_source == WAKE_INT0)
            {
                PORTC = 0xFF;
            }
            else if (last_wake_source == WAKE_INT1)
            {
                PORTC = 0x0F;
            }

            _delay_ms(200);
            PORTC = 0x00;

            sleep_cycle++;
        }

        // Check for UART stop command
        if (UCSR1A & (1 << RXC1))
        {
            getch_USART1();
            break;
        }
    }

    // Disable interrupts
    EIMSK &= ~((1 << INT0) | (1 << INT1));

    puts_USART1("\r\nExternal interrupt demo complete.\r\n");

    char buf[60];
    sprintf(buf, "INT0 wake-ups: %u\r\n", wake_count[WAKE_INT0]);
    puts_USART1(buf);
    sprintf(buf, "INT1 wake-ups: %u\r\n", wake_count[WAKE_INT1]);
    puts_USART1(buf);

    PORTC = 0x00;
}

/* ========================================================================
 * DEMO 2: Timer-Based Periodic Wake-up
 * ======================================================================== */
void demo2_timer_wakeup(void)
{
    puts_USART1("\r\n=== DEMO 2: Timer Periodic Wake-up ===\r\n");
    puts_USART1("Using timers for scheduled wake-ups\r\n");
    puts_USART1("Press any key to stop\r\n\r\n");

    optimize_power();
    config_timer0_wakeup();
    config_timer2_rtc();

    uint16_t wake_cycle = 0;

    puts_USART1("Timer-based wake-up system active...\r\n\r\n");

    while (wake_cycle < 30)
    {
        last_wake_source = WAKE_NONE;

        // Enter power-save mode (timers keep running)
        set_sleep_mode(SLEEP_MODE_PWR_SAVE);
        sleep_enable();
        sei();
        sleep_cpu();
        sleep_disable();

        if (last_wake_source != WAKE_NONE)
        {
            char buf[80];
            sprintf(buf, "\r[%u] Wake: %-8s  T0:%u T2:%u  ",
                    wake_cycle + 1,
                    get_wake_source_name(last_wake_source),
                    wake_count[WAKE_TIMER0],
                    wake_count[WAKE_TIMER2]);
            puts_USART1(buf);

            // LED pattern based on timer
            if (last_wake_source == WAKE_TIMER0)
            {
                PORTC = 0x03;
            }
            else if (last_wake_source == WAKE_TIMER2)
            {
                PORTC = 0x0C;
            }

            _delay_ms(50);
            PORTC = 0x00;

            wake_cycle++;
        }

        // Check for stop
        if (UCSR1A & (1 << RXC1))
        {
            getch_USART1();
            break;
        }
    }

    // Disable timers
    TCCR0 = 0;
    TCCR2 = 0;
    TIMSK &= ~((1 << TOIE0) | (1 << TOIE2));

    puts_USART1("\r\n\r\nTimer wake-up demo complete.\r\n");

    char buf[60];
    sprintf(buf, "Timer0 wake-ups: %u\r\n", wake_count[WAKE_TIMER0]);
    puts_USART1(buf);
    sprintf(buf, "Timer2 wake-ups: %u\r\n", wake_count[WAKE_TIMER2]);
    puts_USART1(buf);

    PORTC = 0x00;
}

/* ========================================================================
 * DEMO 3: Multi-Source Wake-up System
 * ======================================================================== */
void demo3_multi_source(void)
{
    puts_USART1("\r\n=== DEMO 3: Multi-Source Wake-up ===\r\n");
    puts_USART1("Multiple wake-up sources active simultaneously\r\n");
    puts_USART1("Press any UART key to stop\r\n\r\n");

    optimize_power();
    config_external_interrupts();
    config_timer2_rtc();

    // Enable UART RX interrupt
    UCSR1B |= (1 << RXCIE1);

    // Enable ADC with interrupt
    ADMUX = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADIE) |
             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    uint16_t total_wakes = 0;

    puts_USART1("Multi-source system running...\r\n");
    puts_USART1("Wake sources: INT0, INT1, TIMER2, UART, ADC\r\n\r\n");

    while (total_wakes < 40)
    {
        last_wake_source = WAKE_NONE;

        // Enter power-save mode
        set_sleep_mode(SLEEP_MODE_PWR_SAVE);
        sleep_enable();
        sei();
        sleep_cpu();
        sleep_disable();

        if (last_wake_source != WAKE_NONE)
        {
            char buf[80];
            sprintf(buf, "\rWakes: %u | ", total_wakes + 1);
            puts_USART1(buf);

            sprintf(buf, "INT0:%u INT1:%u T2:%u UART:%u ADC:%u",
                    wake_count[WAKE_INT0],
                    wake_count[WAKE_INT1],
                    wake_count[WAKE_TIMER2],
                    wake_count[WAKE_UART],
                    wake_count[WAKE_ADC]);
            puts_USART1(buf);

            // Color-coded LED feedback
            uint8_t led_pattern = 0;
            led_pattern |= (wake_count[WAKE_INT0] > 0) ? 0x01 : 0;
            led_pattern |= (wake_count[WAKE_INT1] > 0) ? 0x02 : 0;
            led_pattern |= (wake_count[WAKE_TIMER2] > 0) ? 0x04 : 0;
            led_pattern |= (wake_count[WAKE_UART] > 0) ? 0x08 : 0;
            led_pattern |= (wake_count[WAKE_ADC] > 0) ? 0x10 : 0;

            PORTC = led_pattern;

            total_wakes++;

            // Check if UART was wake source (stop command)
            if (last_wake_source == WAKE_UART)
            {
                puts_USART1("\r\n\r\nStopped by UART wake-up.\r\n");
                break;
            }
        }

        _delay_ms(100);
    }

    // Disable all wake sources
    EIMSK &= ~((1 << INT0) | (1 << INT1));
    TCCR2 = 0;
    TIMSK &= ~(1 << TOIE2);
    UCSR1B &= ~(1 << RXCIE1);
    ADCSRA = 0;

    puts_USART1("\r\nMulti-source demo complete.\r\n\r\n");
    puts_USART1("Wake-up Summary:\r\n");

    char buf[60];
    sprintf(buf, "  INT0:   %u\r\n", wake_count[WAKE_INT0]);
    puts_USART1(buf);
    sprintf(buf, "  INT1:   %u\r\n", wake_count[WAKE_INT1]);
    puts_USART1(buf);
    sprintf(buf, "  TIMER2: %u\r\n", wake_count[WAKE_TIMER2]);
    puts_USART1(buf);
    sprintf(buf, "  UART:   %u\r\n", wake_count[WAKE_UART]);
    puts_USART1(buf);
    sprintf(buf, "  ADC:    %u\r\n", wake_count[WAKE_ADC]);
    puts_USART1(buf);
    sprintf(buf, "  TOTAL:  %u\r\n", total_wakes);
    puts_USART1(buf);

    PORTC = 0x00;
}

/* ========================================================================
 * DEMO 4: Power Budget Calculator
 * ======================================================================== */
void demo4_power_budget(void)
{
    puts_USART1("\r\n=== DEMO 4: Power Budget Analysis ===\r\n");
    puts_USART1("Calculate power consumption and battery life\r\n\r\n");

    puts_USART1("System Configuration:\r\n");
    puts_USART1("  MCU: ATmega128 @ 7.3728 MHz\r\n");
    puts_USART1("  Battery: CR2032 (220 mAh @ 3V)\r\n\r\n");

    // Get user input for duty cycle
    puts_USART1("Enter active time per cycle (ms): ");
    char input[10];
    uint8_t idx = 0;

    while (1)
    {
        char c = getch_USART1();
        if (c == '\r')
        {
            putch_USART1('\r');
            putch_USART1('\n');
            input[idx] = '\0';
            break;
        }
        if (c >= '0' && c <= '9' && idx < 9)
        {
            input[idx++] = c;
            putch_USART1(c);
        }
    }

    uint16_t active_ms = atoi(input);

    puts_USART1("Enter sleep time per cycle (ms): ");
    idx = 0;

    while (1)
    {
        char c = getch_USART1();
        if (c == '\r')
        {
            putch_USART1('\r');
            putch_USART1('\n');
            input[idx] = '\0';
            break;
        }
        if (c >= '0' && c <= '9' && idx < 9)
        {
            input[idx++] = c;
            putch_USART1(c);
        }
    }

    uint16_t sleep_ms = atoi(input);

    // Power calculations
    uint16_t cycle_ms = active_ms + sleep_ms;
    float duty_cycle = (float)active_ms / cycle_ms;

    // Current consumption estimates (mA)
    float active_current = 15.0; // ~15 mA active
    float sleep_current = 0.002; // ~2 µA in power-down

    float avg_current = (active_current * duty_cycle) +
                        (sleep_current * (1.0 - duty_cycle));

    float battery_mah = 220.0;
    float battery_life_hours = battery_mah / avg_current;
    float battery_life_days = battery_life_hours / 24.0;

    // Display results
    puts_USART1("\r\n╔════════════════════════════════════╗\r\n");
    puts_USART1("║     Power Budget Analysis         ║\r\n");
    puts_USART1("╚════════════════════════════════════╝\r\n\r\n");

    char buf[80];
    sprintf(buf, "Cycle Time:     %u ms\r\n", cycle_ms);
    puts_USART1(buf);
    sprintf(buf, "Active Time:    %u ms\r\n", active_ms);
    puts_USART1(buf);
    sprintf(buf, "Sleep Time:     %u ms\r\n", sleep_ms);
    puts_USART1(buf);
    sprintf(buf, "Duty Cycle:     %.2f%%\r\n\r\n", duty_cycle * 100);
    puts_USART1(buf);

    sprintf(buf, "Active Current: %.1f mA\r\n", active_current);
    puts_USART1(buf);
    sprintf(buf, "Sleep Current:  %.3f mA\r\n", sleep_current);
    puts_USART1(buf);
    sprintf(buf, "Average Current: %.3f mA\r\n\r\n", avg_current);
    puts_USART1(buf);

    sprintf(buf, "Battery Life:\r\n");
    puts_USART1(buf);
    sprintf(buf, "  %.0f hours\r\n", battery_life_hours);
    puts_USART1(buf);
    sprintf(buf, "  %.1f days\r\n", battery_life_days);
    puts_USART1(buf);

    if (battery_life_days > 365)
    {
        sprintf(buf, "  %.2f years\r\n", battery_life_days / 365.0);
        puts_USART1(buf);
    }

    // Visual representation
    puts_USART1("\r\nPower Distribution:\r\n");
    uint8_t active_bars = (uint8_t)(duty_cycle * 20);
    puts_USART1("Active: [");
    for (uint8_t i = 0; i < 20; i++)
    {
        putch_USART1(i < active_bars ? '#' : '-');
    }
    sprintf(buf, "] %.1f%%\r\n", duty_cycle * 100);
    puts_USART1(buf);

    puts_USART1("Sleep:  [");
    for (uint8_t i = 0; i < 20; i++)
    {
        putch_USART1(i >= active_bars ? '#' : '-');
    }
    sprintf(buf, "] %.1f%%\r\n", (1.0 - duty_cycle) * 100);
    puts_USART1(buf);

    // LED indicator of duty cycle
    PORTC = (uint8_t)(duty_cycle * 255);
    _delay_ms(2000);
    PORTC = 0x00;
}

/* ========================================================================
 * Main Menu System
 * ======================================================================== */
void display_main_menu(void)
{
    puts_USART1("\r\n\r\n");
    puts_USART1("╔════════════════════════════════════════╗\r\n");
    puts_USART1("║  Wake-up & Optimization - ATmega128   ║\r\n");
    puts_USART1("╚════════════════════════════════════════╝\r\n");
    puts_USART1("\r\n");
    puts_USART1("Select Demo:\r\n");
    puts_USART1("  [1] External Interrupt Wake-up\r\n");
    puts_USART1("  [2] Timer Periodic Wake-up\r\n");
    puts_USART1("  [3] Multi-Source Wake-up System\r\n");
    puts_USART1("  [4] Power Budget Calculator\r\n");
    puts_USART1("\r\n");
    puts_USART1("Enter selection (1-4): ");
}

int main(void)
{
    // Disable watchdog
    MCUCSR &= ~(1 << WDRF);
    wdt_disable();

    // Initialize peripherals
    Uart1_init();

    // Configure status LEDs
    DDRC = 0xFF;
    PORTC = 0x00;

    // Enable global interrupts
    sei();

    // Send startup message
    _delay_ms(500);
    puts_USART1("\r\n\r\n*** Wake-up Sources & Power Optimization ***\r\n");
    puts_USART1("Ultra-Low Power Techniques\r\n");

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
            demo1_external_interrupts();
            break;
        case '2':
            demo2_timer_wakeup();
            break;
        case '3':
            demo3_multi_source();
            break;
        case '4':
            demo4_power_budget();
            break;
        default:
            puts_USART1("Invalid selection!\r\n");
            _delay_ms(1000);
            break;
        }

        // Reset wake counters between demos
        for (uint8_t i = 0; i < 8; i++)
        {
            wake_count[i] = 0;
        }

        _delay_ms(500);
    }

    return 0;
}
