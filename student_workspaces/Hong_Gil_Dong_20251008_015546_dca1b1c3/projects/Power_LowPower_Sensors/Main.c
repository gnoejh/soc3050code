/*
 * Low-Power Sensor Monitoring
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Minimize power in sensor applications
 * - Use ADC Noise Reduction sleep mode
 * - Implement sensor wake-up strategies
 * - Practice efficient ADC conversion
 *
 * LOW-POWER TECHNIQUES:
 * - ADC Noise Reduction mode (best for ADC)
 * - Power-down between readings
 * - Disable unused peripherals
 * - Use internal pull-ups (no external resistors)
 * - Brown-out detection optimization
 *
 * ADC POWER CONSIDERATIONS:
 * - ADC consumes ~230 µA when active
 * - Conversion time: 13-260 µs
 * - Sleep during conversion reduces noise
 * - Auto-trigger from timer for regularity
 *
 * APPLICATION SCENARIOS:
 * - Weather station
 * - Environmental monitoring
 * - Battery-powered data logger
 * - Remote sensor node
 */

#include "config.h"
#include <avr/sleep.h>

// Sensor configuration
#define ADC_TEMP_CHANNEL 0     // Temperature sensor on ADC0
#define ADC_LIGHT_CHANNEL 1    // Light sensor on ADC1
#define ADC_MOISTURE_CHANNEL 2 // Moisture sensor on ADC2
#define ADC_VOLTAGE_CHANNEL 7  // Battery voltage on ADC7

// Thresholds
#define TEMP_THRESHOLD_LOW 100
#define TEMP_THRESHOLD_HIGH 900
#define LIGHT_THRESHOLD 500
#define MOISTURE_THRESHOLD 300
#define BATTERY_LOW_THRESHOLD 600

// Measurement intervals (in Timer2 ticks)
#define FAST_INTERVAL 10   // ~0.36 seconds
#define NORMAL_INTERVAL 28 // ~1 second
#define SLOW_INTERVAL 280  // ~10 seconds

volatile uint8_t adc_complete = 0;
volatile uint16_t adc_result = 0;
volatile uint16_t timer2_ticks = 0;

/*
 * ADC Conversion Complete ISR
 */
ISR(ADC_vect)
{
    adc_result = ADC;
    adc_complete = 1;
}

/*
 * Timer2 Overflow ISR (periodic wake-up)
 */
ISR(TIMER2_OVF_vect)
{
    timer2_ticks++;
}

/*
 * Initialize ADC with noise reduction capability
 */
void adc_init_low_power(void)
{
    // AVCC reference, right-adjusted
    ADMUX = (1 << REFS0);

    // Enable ADC, enable interrupt, prescaler 128
    // At 7.3728 MHz: ADC clock = 57.6 kHz (ideal 50-200 kHz)
    ADCSRA = (1 << ADEN) | (1 << ADIE) |
             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

/*
 * Start ADC conversion with sleep
 */
uint16_t adc_read_sleep(uint8_t channel)
{
    // Select channel
    ADMUX = (ADMUX & 0xE0) | (channel & 0x1F);

    // Set ADC Noise Reduction sleep mode
    set_sleep_mode(SLEEP_MODE_ADC);

    adc_complete = 0;

    // Start conversion
    ADCSRA |= (1 << ADSC);

    // Sleep during conversion
    sleep_enable();
    sei();
    sleep_cpu();
    sleep_disable();

    // Wait for completion (should be immediate after wake)
    while (!adc_complete)
        ;

    return adc_result;
}

/*
 * Configure Timer2 for periodic wake-up
 */
void timer2_init_wakeup(uint8_t prescaler)
{
    TCCR2 = prescaler;
    TIMSK |= (1 << TOIE2);
    TCNT2 = 0;
}

/*
 * Disable unused peripherals to save power
 */
void disable_unused_peripherals(void)
{
    // Disable analog comparator
    ACSR |= (1 << ACD);

    // Disable SPI
    SPCR &= ~(1 << SPE);

    // Disable TWI
    TWCR &= ~(1 << TWEN);

    puts_USART1("Unused peripherals disabled for power savings\r\n");
}

/* ========================================================================
 * DEMO 1: Low-Power Single Sensor Reading
 * ======================================================================== */
void demo1_single_sensor(void)
{
    puts_USART1("\r\n=== DEMO 1: Low-Power Single Sensor ===\r\n");
    puts_USART1("Reading temperature sensor with minimal power\r\n");
    puts_USART1("Press any key to stop\r\n\r\n");

    disable_unused_peripherals();

    uint16_t reading_count = 0;

    puts_USART1("Starting low-power monitoring...\r\n\r\n");

    while (reading_count < 30)
    {
        // Read sensor using ADC sleep mode
        uint16_t temp = adc_read_sleep(ADC_TEMP_CHANNEL);

        char buf[80];
        sprintf(buf, "[%u] Temperature: %4u  ", reading_count + 1, temp);
        puts_USART1(buf);

        // Check threshold
        if (temp < TEMP_THRESHOLD_LOW)
        {
            puts_USART1("[COLD]   ");
            PORTC = 0x01; // Blue LED
        }
        else if (temp > TEMP_THRESHOLD_HIGH)
        {
            puts_USART1("[HOT]    ");
            PORTC = 0x04; // Red LED
        }
        else
        {
            puts_USART1("[NORMAL] ");
            PORTC = 0x02; // Green LED
        }

        // Calculate power saving
        sprintf(buf, "ADC: %u µA\r\n", 230); // Active ADC power
        puts_USART1(buf);

        reading_count++;

        // Flash LED briefly
        _delay_ms(100);
        PORTC = 0x00;

        // Enter power-down sleep between readings
        puts_USART1("Sleeping...\r\n");
        _delay_ms(50);

        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sleep_enable();
        sei();

        // Sleep for ~1 second
        for (uint8_t i = 0; i < 10; i++)
        {
            _delay_ms(100);

            if (UCSR1A & (1 << RXC1))
            {
                getch_USART1();
                sleep_disable();
                goto exit_demo1;
            }
        }

        sleep_disable();
    }

exit_demo1:
    puts_USART1("\r\nMonitoring stopped.\r\n");

    char buf[60];
    sprintf(buf, "Total readings: %u\r\n", reading_count);
    puts_USART1(buf);

    PORTC = 0x00;
}

/* ========================================================================
 * DEMO 2: Multi-Sensor Monitoring
 * ======================================================================== */
void demo2_multi_sensor(void)
{
    puts_USART1("\r\n=== DEMO 2: Multi-Sensor Monitoring ===\r\n");
    puts_USART1("Monitoring temperature, light, and moisture\r\n");
    puts_USART1("Press any key to stop\r\n\r\n");

    disable_unused_peripherals();
    timer2_init_wakeup((1 << CS22) | (1 << CS21) | (1 << CS20));

    uint16_t cycle = 0;
    timer2_ticks = 0;

    puts_USART1("Multi-sensor monitoring active...\r\n\r\n");

    while (cycle < 20)
    {
        uint16_t last_tick = timer2_ticks;

        // Wait for next interval
        while ((timer2_ticks - last_tick) < NORMAL_INTERVAL)
        {
            set_sleep_mode(SLEEP_MODE_PWR_SAVE);
            sleep_enable();
            sei();
            sleep_cpu();
            sleep_disable();

            if (UCSR1A & (1 << RXC1))
            {
                getch_USART1();
                goto exit_demo2;
            }
        }

        // Take readings
        uint16_t temp = adc_read_sleep(ADC_TEMP_CHANNEL);
        uint16_t light = adc_read_sleep(ADC_LIGHT_CHANNEL);
        uint16_t moisture = adc_read_sleep(ADC_MOISTURE_CHANNEL);

        char buf[80];
        sprintf(buf, "[%u] T:%4u L:%4u M:%4u  ",
                cycle + 1, temp, light, moisture);
        puts_USART1(buf);

        // Alert conditions
        uint8_t alerts = 0;

        if (temp > TEMP_THRESHOLD_HIGH)
        {
            puts_USART1("[TEMP!] ");
            alerts |= 0x04;
        }

        if (light < LIGHT_THRESHOLD)
        {
            puts_USART1("[DARK!] ");
            alerts |= 0x02;
        }

        if (moisture < MOISTURE_THRESHOLD)
        {
            puts_USART1("[DRY!] ");
            alerts |= 0x01;
        }

        if (alerts == 0)
        {
            puts_USART1("[OK]");
        }

        puts_USART1("\r\n");

        PORTC = alerts;
        _delay_ms(200);
        PORTC = 0x00;

        cycle++;
    }

exit_demo2:
    TCCR2 = 0;
    TIMSK &= ~(1 << TOIE2);

    puts_USART1("\r\nMulti-sensor monitoring stopped.\r\n");

    char buf[60];
    sprintf(buf, "Monitoring cycles: %u\r\n", cycle);
    puts_USART1(buf);
    sprintf(buf, "Timer ticks: %u\r\n", timer2_ticks);
    puts_USART1(buf);

    PORTC = 0x00;
}

/* ========================================================================
 * DEMO 3: Battery-Powered Data Logger
 * ======================================================================== */
void demo3_data_logger(void)
{
    puts_USART1("\r\n=== DEMO 3: Battery Data Logger ===\r\n");
    puts_USART1("Logging sensor data with battery monitoring\r\n");
    puts_USART1("Press any key to stop\r\n\r\n");

    disable_unused_peripherals();
    timer2_init_wakeup((1 << CS22) | (1 << CS21) | (1 << CS20));

    uint16_t log_count = 0;
    uint16_t battery = 1000; // Simulated 10.00V
    timer2_ticks = 0;

    puts_USART1("Data logger started...\r\n");
    puts_USART1("Time, Temp, Light, Battery\r\n\r\n");

    while (battery > BATTERY_LOW_THRESHOLD && log_count < 30)
    {
        uint16_t last_tick = timer2_ticks;

        // Wait for logging interval (~10 seconds in demo)
        while ((timer2_ticks - last_tick) < (NORMAL_INTERVAL * 3))
        {
            set_sleep_mode(SLEEP_MODE_PWR_SAVE);
            sleep_enable();
            sei();
            sleep_cpu();
            sleep_disable();

            if (UCSR1A & (1 << RXC1))
            {
                getch_USART1();
                goto exit_demo3;
            }
        }

        // Take sensor readings
        uint16_t temp = adc_read_sleep(ADC_TEMP_CHANNEL);
        uint16_t light = adc_read_sleep(ADC_LIGHT_CHANNEL);

        // Simulate battery drain
        battery -= (rand() % 5) + 1;

        // Log data in CSV format
        char buf[80];
        sprintf(buf, "%u, %u, %u, %u.%02u\r\n",
                timer2_ticks,
                temp,
                light,
                battery / 100,
                battery % 100);
        puts_USART1(buf);

        log_count++;

        // Battery warning
        if (battery < (BATTERY_LOW_THRESHOLD + 100))
        {
            PORTC = 0xFF;
            _delay_ms(100);
            PORTC = 0x00;
        }
        else
        {
            PORTC = 0x01;
            _delay_ms(50);
            PORTC = 0x00;
        }
    }

exit_demo3:
    TCCR2 = 0;
    TIMSK &= ~(1 << TOIE2);

    if (battery <= BATTERY_LOW_THRESHOLD)
    {
        puts_USART1("\r\n⚠ BATTERY LOW - Logging stopped!\r\n");
    }
    else
    {
        puts_USART1("\r\nLogging stopped by user.\r\n");
    }

    char buf[80];
    sprintf(buf, "Log entries: %u\r\n", log_count);
    puts_USART1(buf);
    sprintf(buf, "Final battery: %u.%02uV\r\n", battery / 100, battery % 100);
    puts_USART1(buf);
    sprintf(buf, "Energy used: %u.%02uV\r\n",
            (1000 - battery) / 100,
            (1000 - battery) % 100);
    puts_USART1(buf);

    PORTC = 0x00;
}

/* ========================================================================
 * DEMO 4: Adaptive Sampling Rate
 * ======================================================================== */
void demo4_adaptive_sampling(void)
{
    puts_USART1("\r\n=== DEMO 4: Adaptive Sampling Rate ===\r\n");
    puts_USART1("Sample rate changes based on sensor activity\r\n");
    puts_USART1("Press any key to stop\r\n\r\n");

    disable_unused_peripherals();
    timer2_init_wakeup((1 << CS22) | (1 << CS21) | (1 << CS20));

    uint16_t sample_count = 0;
    uint16_t prev_temp = 512;
    uint16_t interval = NORMAL_INTERVAL;
    timer2_ticks = 0;

    puts_USART1("Adaptive monitoring started...\r\n\r\n");

    while (sample_count < 40)
    {
        uint16_t last_tick = timer2_ticks;

        // Wait for current interval
        while ((timer2_ticks - last_tick) < interval)
        {
            set_sleep_mode(SLEEP_MODE_PWR_SAVE);
            sleep_enable();
            sei();
            sleep_cpu();
            sleep_disable();

            if (UCSR1A & (1 << RXC1))
            {
                getch_USART1();
                goto exit_demo4;
            }
        }

        // Read sensor
        uint16_t temp = adc_read_sleep(ADC_TEMP_CHANNEL);

        // Calculate change rate
        int16_t delta = abs(temp - prev_temp);

        char buf[80];
        sprintf(buf, "[%u] Temp:%4u  Δ:%3d  ",
                sample_count + 1, temp, delta);
        puts_USART1(buf);

        // Adapt sampling rate based on change
        if (delta > 50)
        {
            // Fast change - sample quickly
            interval = FAST_INTERVAL;
            puts_USART1("Rate:FAST  ");
            PORTC = 0x07;
        }
        else if (delta > 20)
        {
            // Moderate change - normal rate
            interval = NORMAL_INTERVAL;
            puts_USART1("Rate:NORM  ");
            PORTC = 0x03;
        }
        else
        {
            // Slow change - save power
            interval = SLOW_INTERVAL;
            puts_USART1("Rate:SLOW  ");
            PORTC = 0x01;
        }

        sprintf(buf, "Int:%u\r\n", interval);
        puts_USART1(buf);

        prev_temp = temp;
        sample_count++;

        _delay_ms(100);
        PORTC = 0x00;
    }

exit_demo4:
    TCCR2 = 0;
    TIMSK &= ~(1 << TOIE2);

    puts_USART1("\r\nAdaptive sampling stopped.\r\n");

    char buf[60];
    sprintf(buf, "Total samples: %u\r\n", sample_count);
    puts_USART1(buf);
    sprintf(buf, "Timer ticks: %u\r\n", timer2_ticks);
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
    puts_USART1("║  Low-Power Sensors - ATmega128        ║\r\n");
    puts_USART1("╚════════════════════════════════════════╝\r\n");
    puts_USART1("\r\n");
    puts_USART1("Select Demo:\r\n");
    puts_USART1("  [1] Low-Power Single Sensor\r\n");
    puts_USART1("  [2] Multi-Sensor Monitoring\r\n");
    puts_USART1("  [3] Battery Data Logger\r\n");
    puts_USART1("  [4] Adaptive Sampling Rate\r\n");
    puts_USART1("\r\n");
    puts_USART1("Enter selection (1-4): ");
}

int main(void)
{
    // Initialize peripherals
    Uart1_init();
    adc_init_low_power();

    // Configure status LEDs
    DDRC = 0xFF;
    PORTC = 0x00;

    // Enable global interrupts
    sei();

    // Send startup message
    _delay_ms(500);
    puts_USART1("\r\n\r\n*** Low-Power Sensor Monitoring ***\r\n");
    puts_USART1("Battery-Optimized Operation\r\n");

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
            demo1_single_sensor();
            break;
        case '2':
            demo2_multi_sensor();
            break;
        case '3':
            demo3_data_logger();
            break;
        case '4':
            demo4_adaptive_sampling();
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
