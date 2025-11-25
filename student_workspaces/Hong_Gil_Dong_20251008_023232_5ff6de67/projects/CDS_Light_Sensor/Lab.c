/*
 * =============================================================================
 * SENSOR INTEGRATION - HANDS-ON LAB EXERCISES (CDS Light Sensor)
 * =============================================================================
 * ATmega128 Educational Framework - Lab Session
 *
 * OBJECTIVE: Master sensor integration and data processing
 * DURATION: 90 minutes
 * DIFFICULTY: Intermediate
 *
 * STUDENTS WILL:
 * - Characterize CDS light sensor
 * - Create light-responsive applications
 * - Implement auto-ranging ADC
 * - Build data visualization systems
 * - Design smart lighting control
 *
 * HARDWARE REQUIRED:
 * - ATmega128 board
 * - CDS photoresistor on ADC2
 * - LEDs on PORTB
 * - Optional: GLCD for visualization
 * - Flashlight/lamp for testing
 *
 * =============================================================================
 */

#include "config.h"

// Lab configuration
#define LIGHT_CHANNEL 2
#define LED_PORT PORTB
#define LED_DDR DDRB

// Global variables
uint16_t lab_score = 0;
ADC_Statistics light_stats;
ADC_Logger light_logger;
ADC_Threshold light_threshold;

/*
 * =============================================================================
 * LAB EXERCISE 1: SENSOR CHARACTERIZATION (20 minutes)
 * =============================================================================
 * OBJECTIVE: Understand sensor behavior and range
 * DIFFICULTY: ★★☆☆☆ (Easy-Medium)
 */

void lab_ex1_sensor_range_test(void)
{
    /*
     * CHALLENGE: Determine sensor operating range
     * TASK: Measure light levels from dark to bright
     * LEARNING: Sensor characteristics, dynamic range
     */

    puts_USART1("\r\n=== Lab 1.1: Sensor Range Test ===\r\n");
    puts_USART1("Measuring CDS sensor range...\r\n\r\n");

    puts_USART1("Step 1: Cover sensor completely (darkness)\r\n");
    puts_USART1("Press any key when ready...\r\n");
    getch_USART1();

    _delay_ms(500);
    uint16_t dark_value = Read_Adc_Median(LIGHT_CHANNEL, 20);

    char buffer[80];
    sprintf(buffer, "Dark reading: %u (%.2f V)\r\n\r\n",
            dark_value, dark_value * 5.0 / 1023);
    puts_USART1(buffer);

    puts_USART1("Step 2: Shine bright light directly on sensor\r\n");
    puts_USART1("Press any key when ready...\r\n");
    getch_USART1();

    _delay_ms(500);
    uint16_t bright_value = Read_Adc_Median(LIGHT_CHANNEL, 20);

    sprintf(buffer, "Bright reading: %u (%.2f V)\r\n\r\n",
            bright_value, bright_value * 5.0 / 1023);
    puts_USART1(buffer);

    uint16_t range = bright_value - dark_value;
    sprintf(buffer, "Dynamic range: %u ADC counts\r\n", range);
    puts_USART1(buffer);

    float ratio = (float)bright_value / dark_value;
    sprintf(buffer, "Bright/Dark ratio: %.2f\r\n", ratio);
    puts_USART1(buffer);

    // Test linearity
    puts_USART1("\r\nStep 3: Linearity test\r\n");
    puts_USART1("Slowly move hand closer/farther from sensor\r\n");
    puts_USART1("Observe readings (10 seconds)\r\n");
    puts_USART1("Press any key to start...\r\n");
    getch_USART1();

    for (uint8_t i = 0; i < 50; i++)
    {
        uint16_t reading = Read_Adc_Data(LIGHT_CHANNEL);

        sprintf(buffer, "Reading %2u: %4u [", i + 1, reading);
        puts_USART1(buffer);

        // Bar graph
        uint8_t bars = ((uint32_t)reading * 30) / 1023;
        for (uint8_t j = 0; j < 30; j++)
        {
            putch_USART1(j < bars ? '=' : ' ');
        }
        puts_USART1("]\r\n");

        _delay_ms(200);
    }

    puts_USART1("\r\nSensor characterization complete!\r\n");

    lab_score += 100;
}

void lab_ex1_response_time(void)
{
    /*
     * CHALLENGE: Measure sensor response time
     * TASK: Measure how fast sensor responds to light changes
     * LEARNING: Sensor dynamics, time constants
     */

    puts_USART1("\r\n=== Lab 1.2: Response Time Test ===\r\n");
    puts_USART1("Measuring sensor response to rapid changes\r\n\r\n");

    puts_USART1("When ready, quickly cover and uncover sensor\r\n");
    puts_USART1("Press any key to start logging...\r\n");
    getch_USART1();

    // Fast sampling for 2 seconds
    uint16_t samples[100];

    for (uint8_t i = 0; i < 100; i++)
    {
        samples[i] = Read_Adc_Data(LIGHT_CHANNEL);
        _delay_ms(20); // 50 Hz sampling
    }

    puts_USART1("Captured 100 samples at 50 Hz\r\n\r\n");

    // Analyze for transitions
    uint8_t transitions = 0;
    uint16_t threshold = 512; // Mid-range
    uint8_t last_state = samples[0] > threshold;

    puts_USART1("Sample | Value | State\r\n");
    puts_USART1("-------|-------|-------\r\n");

    for (uint8_t i = 0; i < 100; i++)
    {
        uint8_t state = samples[i] > threshold;

        if (state != last_state)
        {
            transitions++;
            char msg[40];
            sprintf(msg, "  %3u  | %4u  | %s\r\n",
                    i, samples[i], state ? "LIGHT" : "DARK ");
            puts_USART1(msg);
        }

        last_state = state;
    }

    char summary[60];
    sprintf(summary, "\r\nDetected %u transitions\r\n", transitions);
    puts_USART1(summary);

    lab_score += 75;
}

/*
 * =============================================================================
 * LAB EXERCISE 2: LIGHT-RESPONSIVE APPLICATIONS (20 minutes)
 * =============================================================================
 * OBJECTIVE: Create practical light-sensing applications
 * DIFFICULTY: ★★★☆☆ (Medium)
 */

void lab_ex2_night_light(void)
{
    /*
     * CHALLENGE: Automatic night light
     * TASK: Turn on LEDs when it gets dark
     * LEARNING: Threshold-based control, hysteresis
     */

    puts_USART1("\r\n=== Lab 2.1: Automatic Night Light ===\r\n");
    puts_USART1("LEDs will turn on automatically when dark\r\n");
    puts_USART1("Cover sensor to test. Press 'Q' to exit\r\n\r\n");

    // Configure LEDs
    LED_DDR = 0xFF;
    LED_PORT = 0xFF; // All off

    // Set threshold (adjust based on sensor range test)
    ADC_Set_Threshold(&light_threshold, 200, 400, 30);

    puts_USART1("Thresholds:\r\n");
    puts_USART1("  Turn ON:  < 200 (dark)\r\n");
    puts_USART1("  Turn OFF: > 400 (bright)\r\n");
    puts_USART1("  Hysteresis: 30\r\n\r\n");

    uint16_t on_time = 0;
    uint16_t off_time = 0;

    while (1)
    {
        uint16_t light = Read_Adc_Median(LIGHT_CHANNEL, 5);
        uint8_t status = ADC_Check_Threshold(&light_threshold, light);

        char buffer[100];

        if (status == 1) // Too dark - turn ON
        {
            LED_PORT = 0x00; // All LEDs on
            on_time++;
            sprintf(buffer, "\rLight: %4u | Status: DARK  | LEDs: ON  | On time: %us ",
                    light, on_time);
        }
        else // Bright enough - turn OFF
        {
            LED_PORT = 0xFF; // All LEDs off
            off_time++;
            sprintf(buffer, "\rLight: %4u | Status: LIGHT | LEDs: OFF | Off time: %us",
                    light, off_time);
        }

        puts_USART1(buffer);

        _delay_ms(1000);

        if (UCSR1A & (1 << RXC1))
        {
            char c = UDR1;
            if (c == 'Q' || c == 'q')
                break;
        }
    }

    LED_PORT = 0xFF; // All off

    puts_USART1("\r\n\r\nNight light test complete!\r\n");

    lab_score += 100;
}

void lab_ex2_light_meter(void)
{
    /*
     * CHALLENGE: LED bar graph light meter
     * TASK: Display light level on 8 LEDs
     * LEARNING: Data visualization with LEDs
     */

    puts_USART1("\r\n=== Lab 2.2: LED Bar Graph Light Meter ===\r\n");
    puts_USART1("8 LEDs show light intensity\r\n");
    puts_USART1("Press 'Q' to exit\r\n\r\n");

    LED_DDR = 0xFF;

    // Auto-calibrate range
    puts_USART1("Auto-calibrating... vary light for 5 seconds\r\n");

    uint16_t min_light = 1023, max_light = 0;

    for (uint8_t i = 0; i < 25; i++)
    {
        uint16_t reading = Read_Adc_Data(LIGHT_CHANNEL);
        if (reading < min_light)
            min_light = reading;
        if (reading > max_light)
            max_light = reading;
        _delay_ms(200);
    }

    char buffer[80];
    sprintf(buffer, "Calibrated: Min=%u, Max=%u\r\n\r\n", min_light, max_light);
    puts_USART1(buffer);

    uint16_t range = max_light - min_light;
    if (range < 100)
        range = 100; // Avoid division by zero

    while (1)
    {
        uint16_t light = Read_Adc_Data(LIGHT_CHANNEL);

        // Map to 0-8 LEDs
        uint8_t led_count;
        if (light <= min_light)
            led_count = 0;
        else if (light >= max_light)
            led_count = 8;
        else
            led_count = ((uint32_t)(light - min_light) * 8) / range;

        // Create LED pattern
        uint8_t pattern = 0;
        for (uint8_t i = 0; i < led_count; i++)
        {
            pattern |= (1 << i);
        }

        LED_PORT = ~pattern; // Active low

        sprintf(buffer, "\rLight: %4u | LEDs: %u/8 [", light, led_count);
        puts_USART1(buffer);

        for (uint8_t i = 0; i < 8; i++)
        {
            putch_USART1(i < led_count ? '█' : '░');
        }
        puts_USART1("]");

        _delay_ms(100);

        if (UCSR1A & (1 << RXC1))
        {
            char c = UDR1;
            if (c == 'Q' || c == 'q')
                break;
        }
    }

    LED_PORT = 0xFF;

    puts_USART1("\r\n\r\nLight meter complete!\r\n");

    lab_score += 125;
}

/*
 * =============================================================================
 * LAB EXERCISE 3: DATA LOGGING AND ANALYSIS (25 minutes)
 * =============================================================================
 * OBJECTIVE: Long-term data collection and analysis
 * DIFFICULTY: ★★★★☆ (Advanced)
 */

void lab_ex3_daylight_logger(void)
{
    /*
     * CHALLENGE: Log light levels over time
     * TASK: Record 64 samples with timestamps
     * LEARNING: Data logging, time-series analysis
     */

    puts_USART1("\r\n=== Lab 3.1: Daylight Logger ===\r\n");
    puts_USART1("Logging light levels every 2 seconds for ~2 minutes\r\n\r\n");

    ADC_Logger_Init(&light_logger);
    ADC_Init_Statistics(&light_stats);

    for (uint8_t i = 0; i < 64; i++)
    {
        uint16_t light = Read_Adc_Median(LIGHT_CHANNEL, 5);

        ADC_Logger_Add(&light_logger, light);
        ADC_Update_Statistics(&light_stats, light);

        char buffer[60];
        sprintf(buffer, "Sample %2u/64: %4u | Running avg: %4u\r\n",
                i + 1, light, light_stats.avg);
        puts_USART1(buffer);

        _delay_ms(2000);
    }

    puts_USART1("\r\n=== Logging Complete ===\r\n");

    // Display statistics
    ADC_Statistics stats = ADC_Get_Statistics(&light_stats);

    char buffer[80];
    sprintf(buffer, "Samples:    64\r\n");
    puts_USART1(buffer);
    sprintf(buffer, "Minimum:    %u\r\n", stats.min);
    puts_USART1(buffer);
    sprintf(buffer, "Maximum:    %u\r\n", stats.max);
    puts_USART1(buffer);
    sprintf(buffer, "Average:    %u\r\n", stats.avg);
    puts_USART1(buffer);
    sprintf(buffer, "Range:      %u\r\n", stats.max - stats.min);
    puts_USART1(buffer);

    // Calculate variance
    uint32_t variance = 0;
    for (uint8_t i = 0; i < 64; i++)
    {
        uint16_t sample = ADC_Logger_Get(&light_logger, i);
        int32_t diff = sample - stats.avg;
        variance += (diff * diff) / 64;
    }

    sprintf(buffer, "Variance:   %lu\r\n", variance);
    puts_USART1(buffer);

    // Find trends
    uint16_t first_avg = 0, last_avg = 0;

    for (uint8_t i = 0; i < 10; i++)
    {
        first_avg += ADC_Logger_Get(&light_logger, i) / 10;
        last_avg += ADC_Logger_Get(&light_logger, 54 + i) / 10;
    }

    puts_USART1("\r\n=== Trend Analysis ===\r\n");
    sprintf(buffer, "First 10 avg:  %u\r\n", first_avg);
    puts_USART1(buffer);
    sprintf(buffer, "Last 10 avg:   %u\r\n", last_avg);
    puts_USART1(buffer);

    if (last_avg > first_avg + 20)
    {
        puts_USART1("Trend: Getting BRIGHTER\r\n");
    }
    else if (last_avg < first_avg - 20)
    {
        puts_USART1("Trend: Getting DARKER\r\n");
    }
    else
    {
        puts_USART1("Trend: STABLE\r\n");
    }

    lab_score += 150;
}

void lab_ex3_histogram_analysis(void)
{
    /*
     * CHALLENGE: Create light distribution histogram
     * TASK: Analyze light level distribution
     * LEARNING: Statistical distribution analysis
     */

    puts_USART1("\r\n=== Lab 3.2: Histogram Analysis ===\r\n");
    puts_USART1("Collecting 100 samples for distribution analysis\r\n\r\n");

    uint16_t samples[100];
    uint16_t min_val = 1023, max_val = 0;

    // Collect samples
    for (uint8_t i = 0; i < 100; i++)
    {
        samples[i] = Read_Adc_Data(LIGHT_CHANNEL);

        if (samples[i] < min_val)
            min_val = samples[i];
        if (samples[i] > max_val)
            max_val = samples[i];

        if (i % 10 == 0)
        {
            char msg[30];
            sprintf(msg, "Collecting... %u%%\r", i);
            puts_USART1(msg);
        }

        _delay_ms(100);
    }

    puts_USART1("\r\n\r\n=== Distribution Histogram ===\r\n");

    // Create 10 bins
    uint8_t bins[10] = {0};
    uint16_t bin_size = (max_val - min_val + 9) / 10;
    if (bin_size == 0)
        bin_size = 1;

    for (uint8_t i = 0; i < 100; i++)
    {
        uint8_t bin = (samples[i] - min_val) / bin_size;
        if (bin > 9)
            bin = 9;
        bins[bin]++;
    }

    // Display histogram
    for (uint8_t i = 0; i < 10; i++)
    {
        uint16_t range_start = min_val + i * bin_size;
        uint16_t range_end = range_start + bin_size - 1;

        char buffer[80];
        sprintf(buffer, "%4u-%4u: [", range_start, range_end);
        puts_USART1(buffer);

        for (uint8_t j = 0; j < bins[i]; j++)
        {
            putch_USART1('*');
        }

        sprintf(buffer, "] %u\r\n", bins[i]);
        puts_USART1(buffer);
    }

    puts_USART1("\r\nHistogram analysis complete!\r\n");

    lab_score += 125;
}

/*
 * =============================================================================
 * LAB MENU SYSTEM
 * =============================================================================
 */

void print_lab_menu(void)
{
    puts_USART1("\r\n");
    puts_USART1("========================================\r\n");
    puts_USART1("  SENSOR INTEGRATION - LAB EXERCISES\r\n");
    puts_USART1("  (CDS Light Sensor)\r\n");
    puts_USART1("========================================\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 1: Sensor Characterization\r\n");
    puts_USART1("  1. Sensor Range Test\r\n");
    puts_USART1("  2. Response Time Test\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 2: Light-Responsive Apps\r\n");
    puts_USART1("  3. Automatic Night Light\r\n");
    puts_USART1("  4. LED Bar Graph Light Meter\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 3: Data Logging & Analysis\r\n");
    puts_USART1("  5. Daylight Logger\r\n");
    puts_USART1("  6. Histogram Analysis\r\n");
    puts_USART1("\r\n");
    puts_USART1("  0. Run All Exercises\r\n");
    puts_USART1("  X. Exit Lab\r\n");
    puts_USART1("\r\n");
    char score_str[40];
    sprintf(score_str, "Current Score: %u points\r\n\r\n", lab_score);
    puts_USART1(score_str);
    puts_USART1("Select exercise (1-6, 0, X): ");
}

int main(void)
{
    // Initialize system
    init_devices();
    Uart1_init();
    Adc_init();

    _delay_ms(100);

    puts_USART1("\r\n\r\n");
    puts_USART1("*************************************************\r\n");
    puts_USART1("*  ATmega128 SENSOR INTEGRATION LAB            *\r\n");
    puts_USART1("*  CDS Light Sensor Exercises                  *\r\n");
    puts_USART1("*************************************************\r\n");
    puts_USART1("\r\n");
    puts_USART1("Welcome to the Sensor Integration Lab!\r\n");
    puts_USART1("Master sensor interfacing and data analysis.\r\n");

    while (1)
    {
        print_lab_menu();

        char choice = getch_USART1();
        putch_USART1(choice);
        putch_USART1('\r');
        putch_USART1('\n');

        switch (choice)
        {
        case '1':
            lab_ex1_sensor_range_test();
            break;
        case '2':
            lab_ex1_response_time();
            break;
        case '3':
            lab_ex2_night_light();
            break;
        case '4':
            lab_ex2_light_meter();
            break;
        case '5':
            lab_ex3_daylight_logger();
            break;
        case '6':
            lab_ex3_histogram_analysis();
            break;

        case '0':
            puts_USART1("\r\n*** RUNNING ALL EXERCISES ***\r\n");
            lab_ex1_sensor_range_test();
            lab_ex1_response_time();
            lab_ex2_night_light();
            lab_ex2_light_meter();
            lab_ex3_daylight_logger();
            lab_ex3_histogram_analysis();

            char final_buffer[80];
            sprintf(final_buffer, "\r\n*** ALL EXERCISES COMPLETE! ***\r\nFinal Score: %u points\r\n", lab_score);
            puts_USART1(final_buffer);
            break;

        case 'X':
        case 'x':
            puts_USART1("\r\nExiting lab. Great work!\r\n");
            while (1)
                ;

        default:
            puts_USART1("Invalid choice. Please try again.\r\n");
        }

        puts_USART1("\r\nPress any key to continue...\r\n");
        getch_USART1();
    }

    return 0;
}
