/*
 * =============================================================================
 * ANALOG-TO-DIGITAL CONVERSION - HANDS-ON LAB EXERCISES
 * =============================================================================
 *
 * PROJECT: ADC_Basic
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Interactive laboratory exercises for hands-on experience with ATmega128 ADC systems.
 * Students practice analog signal processing through guided exercises and challenges.
 *
 * LAB OBJECTIVES:
 * 1. Calibrate ADC sensors with real-world measurements
 * 2. Implement threshold detection and alarm systems
 * 3. Create data logging and analysis applications
 * 4. Build sensor comparison and validation experiments
 * 5. Practice noise filtering and signal conditioning
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - Multiple analog sensors (temperature, light, pressure)
 * - Potentiometer for reference voltage testing
 * - LCD display for real-time data visualization
 * - Serial terminal for data logging (9600 baud)
 *
 * LAB STRUCTURE:
 * - Exercise 1: ADC Calibration and Linearity Testing
 * - Exercise 2: Multi-Channel Sensor Monitoring
 * - Exercise 3: Threshold-Based Control System
 * - Exercise 4: Data Acquisition and Logging
 * - Exercise 5: Advanced Signal Processing
 *
 * DURATION: 90 minutes
 * DIFFICULTY: Intermediate
 *
 * =============================================================================
 */
*-Temperature sensor(LM35)
on ADC1 * -Light sensor(CDS) on ADC2
 * - Optional: GLCD for visualization
 *
 * =============================================================================
 */

#include "config.h"

// Lab configuration
#define POT_CHANNEL 0
#define TEMP_CHANNEL 1
#define LIGHT_CHANNEL 2

// Global lab variables
ADC_Statistics pot_stats;
ADC_Statistics temp_stats;
ADC_Statistics light_stats;
ADC_Calibration temp_calibration;
ADC_Threshold temp_threshold;
ADC_Logger data_logger;

uint16_t lab_score = 0;
uint8_t calibration_points_added = 0;

/*
 * =============================================================================
 * LAB EXERCISE 1: SENSOR CALIBRATION (20 minutes)
 * =============================================================================
 * OBJECTIVE: Learn to calibrate sensors with known reference points
 * DIFFICULTY: ★★☆☆☆ (Easy-Medium)
 */

void lab_ex1_temperature_calibration(void)
{
    /*
     * CHALLENGE: Create a multi-point temperature calibration
     * TASK: Measure ADC values at known temperatures
     * LEARNING: Calibration tables and linear interpolation
     */

    puts_USART1("\r\n=== Lab 1.1: Temperature Calibration ===\r\n");
    puts_USART1("Calibrate temperature sensor with known references\r\n");
    puts_USART1("You will measure ADC values at 3 known temperatures\r\n\r\n");

    // Initialize calibration structure
    temp_calibration.num_points = 0;

    // Calibration point 1: Ice water (0°C)
    puts_USART1("Step 1: Place sensor in ice water (0°C)\r\n");
    puts_USART1("Press any key when ready...\r\n");
    getch_USART1();

    _delay_ms(2000); // Let sensor stabilize
    uint16_t adc_0c = Read_Adc_Median(TEMP_CHANNEL, 10);
    ADC_Add_Calibration_Point(&temp_calibration, adc_0c, 0);

    char buffer[80];
    sprintf(buffer, "Measured ADC at 0°C: %u\r\n\r\n", adc_0c);
    puts_USART1(buffer);

    // Calibration point 2: Room temperature (~25°C)
    puts_USART1("Step 2: Place sensor at room temperature (~25°C)\r\n");
    puts_USART1("Enter actual temperature in °C: ");

    char temp_str[10];
    uint8_t idx = 0;
    while (1)
    {
        char c = getch_USART1();
        putch_USART1(c);
        if (c == '\r')
            break;
        if (c >= '0' && c <= '9' && idx < 9)
        {
            temp_str[idx++] = c;
        }
    }
    temp_str[idx] = '\0';
    puts_USART1("\r\n");

    int16_t room_temp = atoi(temp_str);

    puts_USART1("Press any key to measure...\r\n");
    getch_USART1();

    _delay_ms(2000);
    uint16_t adc_room = Read_Adc_Median(TEMP_CHANNEL, 10);
    ADC_Add_Calibration_Point(&temp_calibration, adc_room, room_temp * 10);

    sprintf(buffer, "Measured ADC at %d°C: %u\r\n\r\n", room_temp, adc_room);
    puts_USART1(buffer);

    // Calibration point 3: Body temperature (~37°C)
    puts_USART1("Step 3: Hold sensor in hand (~37°C)\r\n");
    puts_USART1("Press any key when ready...\r\n");
    getch_USART1();

    _delay_ms(2000);
    uint16_t adc_body = Read_Adc_Median(TEMP_CHANNEL, 10);
    ADC_Add_Calibration_Point(&temp_calibration, adc_body, 370);

    sprintf(buffer, "Measured ADC at ~37°C: %u\r\n\r\n", adc_body);
    puts_USART1(buffer);

    // Display calibration table
    puts_USART1("=== Calibration Table Created ===\r\n");
    puts_USART1("Point | ADC Value | Temperature\r\n");
    puts_USART1("------|-----------|------------\r\n");

    for (uint8_t i = 0; i < temp_calibration.num_points; i++)
    {
        sprintf(buffer, "  %u   |   %4u    |   %d.%d°C\r\n",
                i + 1,
                temp_calibration.adc_values[i],
                temp_calibration.real_values[i] / 10,
                temp_calibration.real_values[i] % 10);
        puts_USART1(buffer);
    }

    puts_USART1("\r\nCalibration complete! Testing calibration...\r\n\r\n");

    // Test calibration with 10 readings
    for (uint8_t i = 0; i < 10; i++)
    {
        uint16_t raw_adc = Read_Adc_Data(TEMP_CHANNEL);
        int16_t calibrated = ADC_Apply_Calibration(&temp_calibration, raw_adc);

        sprintf(buffer, "Reading %u: ADC=%4u -> Temperature=%d.%d°C\r\n",
                i + 1, raw_adc, calibrated / 10, abs(calibrated % 10));
        puts_USART1(buffer);

        _delay_ms(500);
    }

    lab_score += 100;
    puts_USART1("\r\nScore: +100 points (Calibration complete)\r\n");
}

void lab_ex1_potentiometer_calibration(void)
{
    /*
     * CHALLENGE: Calibrate potentiometer to 0-100% scale
     * TASK: Map ADC range to percentage
     * LEARNING: Linear scaling and range mapping
     */

    puts_USART1("\r\n=== Lab 1.2: Potentiometer Calibration ===\r\n");
    puts_USART1("Calibrate potentiometer to 0-100% scale\r\n\r\n");

    puts_USART1("Step 1: Turn pot fully counter-clockwise (0%)\r\n");
    puts_USART1("Press any key when ready...\r\n");
    getch_USART1();

    _delay_ms(1000);
    uint16_t adc_min = Read_Adc_Median(POT_CHANNEL, 20);

    char buffer[80];
    sprintf(buffer, "Minimum ADC: %u\r\n\r\n", adc_min);
    puts_USART1(buffer);

    puts_USART1("Step 2: Turn pot fully clockwise (100%)\r\n");
    puts_USART1("Press any key when ready...\r\n");
    getch_USART1();

    _delay_ms(1000);
    uint16_t adc_max = Read_Adc_Median(POT_CHANNEL, 20);

    sprintf(buffer, "Maximum ADC: %u\r\n\r\n", adc_max);
    puts_USART1(buffer);

    uint16_t adc_range = adc_max - adc_min;

    sprintf(buffer, "Calibration: Range = %u ADC units\r\n", adc_range);
    puts_USART1(buffer);
    puts_USART1("Now adjust potentiometer - values will show as percentage\r\n");
    puts_USART1("Press 'Q' to quit\r\n\r\n");

    while (1)
    {
        uint16_t adc_raw = Read_Adc_Data(POT_CHANNEL);

        // Map to 0-100%
        int16_t percentage;
        if (adc_raw <= adc_min)
            percentage = 0;
        else if (adc_raw >= adc_max)
            percentage = 100;
        else
            percentage = ((uint32_t)(adc_raw - adc_min) * 100) / adc_range;

        sprintf(buffer, "ADC: %4u | Percentage: %3d%%\r", adc_raw, percentage);
        puts_USART1(buffer);

        _delay_ms(200);

        // Check for quit
        if (UCSR1A & (1 << RXC1))
        {
            char c = UDR1;
            if (c == 'Q' || c == 'q')
                break;
        }
    }

    puts_USART1("\r\n\r\nCalibration test complete!\r\n");
    lab_score += 50;
}

/*
 * =============================================================================
 * LAB EXERCISE 2: THRESHOLD DETECTION (15 minutes)
 * =============================================================================
 * OBJECTIVE: Implement threshold-based monitoring systems
 * DIFFICULTY: ★★★☆☆ (Medium)
 */

void lab_ex2_temperature_alarm(void)
{
    /*
     * CHALLENGE: Create temperature alarm system
     * TASK: Alert when temperature crosses thresholds
     * LEARNING: Threshold detection with hysteresis
     */

    puts_USART1("\r\n=== Lab 2.1: Temperature Alarm System ===\r\n");
    puts_USART1("Configure temperature thresholds for alarm\r\n\r\n");

    // Set thresholds (using calibrated values if available)
    ADC_Set_Threshold(&temp_threshold, 200, 600, 20); // Example: ~20°C to ~60°C, 2°C hysteresis

    puts_USART1("Temperature Alarm Configuration:\r\n");
    puts_USART1("  Low Threshold:  20°C\r\n");
    puts_USART1("  High Threshold: 30°C\r\n");
    puts_USART1("  Hysteresis:     2°C\r\n");
    puts_USART1("\r\n");
    puts_USART1("Monitor temperature for 30 seconds...\r\n");
    puts_USART1("Press 'Q' to quit early\r\n\r\n");

    uint8_t alarm_count = 0;

    for (uint16_t i = 0; i < 150; i++) // 30 seconds at 200ms intervals
    {
        uint16_t adc_value = Read_Adc_Data(TEMP_CHANNEL);
        uint8_t status = ADC_Check_Threshold(&temp_threshold, adc_value);

        char buffer[100];
        const char *state_str;

        switch (status)
        {
        case 0:
            state_str = "NORMAL ";
            break;
        case 1:
            state_str = "TOO LOW ";
            alarm_count++;
            break;
        case 2:
            state_str = "TOO HIGH";
            alarm_count++;
            break;
        default:
            state_str = "UNKNOWN";
            break;
        }

        // Apply calibration if available
        int16_t temp = ADC_Apply_Calibration(&temp_calibration, adc_value);

        sprintf(buffer, "ADC: %4u | Temp: %d.%d°C | Status: %s",
                adc_value, temp / 10, abs(temp % 10), state_str);

        if (status != 0)
        {
            sprintf(buffer + strlen(buffer), " <<<< ALARM!");
        }

        sprintf(buffer + strlen(buffer), "\r");
        puts_USART1(buffer);

        _delay_ms(200);

        // Check for quit
        if (UCSR1A & (1 << RXC1))
        {
            char c = UDR1;
            if (c == 'Q' || c == 'q')
                break;
        }
    }

    char summary[80];
    sprintf(summary, "\r\n\r\nMonitoring complete! Alarm triggered %u times.\r\n", alarm_count);
    puts_USART1(summary);

    if (alarm_count == 0)
    {
        puts_USART1("Perfect! No alarms triggered. +100 points\r\n");
        lab_score += 100;
    }
    else
    {
        sprintf(summary, "Alarms detected. +%u points\r\n", 50 + alarm_count * 5);
        puts_USART1(summary);
        lab_score += 50 + alarm_count * 5;
    }
}

void lab_ex2_light_level_detector(void)
{
    /*
     * CHALLENGE: Detect light levels (bright/normal/dark)
     * TASK: Classify lighting conditions
     * LEARNING: Multi-level threshold detection
     */

    puts_USART1("\r\n=== Lab 2.2: Light Level Detector ===\r\n");
    puts_USART1("Detecting light levels: DARK / NORMAL / BRIGHT\r\n\r\n");

    // Auto-calibrate by sampling for 5 seconds
    puts_USART1("Auto-calibrating... Move light sensor around.\r\n");

    uint16_t min_light = 1023, max_light = 0;

    for (uint8_t i = 0; i < 25; i++)
    {
        uint16_t light = Read_Adc_Data(LIGHT_CHANNEL);
        if (light < min_light)
            min_light = light;
        if (light > max_light)
            max_light = light;
        _delay_ms(200);
    }

    // Set thresholds at 33% and 66% of range
    uint16_t range = max_light - min_light;
    uint16_t low_threshold = min_light + range / 3;
    uint16_t high_threshold = min_light + (range * 2) / 3;

    char buffer[100];
    sprintf(buffer, "Calibrated: Min=%u, Max=%u\r\n", min_light, max_light);
    puts_USART1(buffer);
    sprintf(buffer, "Thresholds: Dark<%u, %u<Bright\r\n\r\n", low_threshold, high_threshold);
    puts_USART1(buffer);

    puts_USART1("Cover and uncover sensor. Press 'Q' to quit.\r\n\r\n");

    uint8_t dark_count = 0, normal_count = 0, bright_count = 0;

    for (uint16_t i = 0; i < 200; i++)
    {
        uint16_t light = Read_Adc_Data(LIGHT_CHANNEL);

        const char *level;
        if (light < low_threshold)
        {
            level = "DARK   ";
            dark_count++;
        }
        else if (light > high_threshold)
        {
            level = "BRIGHT ";
            bright_count++;
        }
        else
        {
            level = "NORMAL ";
            normal_count++;
        }

        sprintf(buffer, "Light ADC: %4u | Level: %s | [", light, level);
        puts_USART1(buffer);

        // Bar graph (20 chars)
        uint8_t bar_length = ((uint32_t)light * 20) / 1023;
        for (uint8_t j = 0; j < 20; j++)
        {
            putch_USART1(j < bar_length ? '=' : ' ');
        }
        puts_USART1("]\r");

        _delay_ms(100);

        if (UCSR1A & (1 << RXC1))
        {
            char c = UDR1;
            if (c == 'Q' || c == 'q')
                break;
        }
    }

    sprintf(buffer, "\r\n\r\nLight Level Summary:\r\n");
    puts_USART1(buffer);
    sprintf(buffer, "  Dark:   %u samples\r\n", dark_count);
    puts_USART1(buffer);
    sprintf(buffer, "  Normal: %u samples\r\n", normal_count);
    puts_USART1(buffer);
    sprintf(buffer, "  Bright: %u samples\r\n", bright_count);
    puts_USART1(buffer);

    lab_score += 75;
}

/*
 * =============================================================================
 * LAB EXERCISE 3: DATA LOGGING AND ANALYSIS (20 minutes)
 * =============================================================================
 * OBJECTIVE: Collect and analyze sensor data
 * DIFFICULTY: ★★★★☆ (Advanced)
 */

void lab_ex3_data_logger(void)
{
    /*
     * CHALLENGE: Log sensor data and calculate statistics
     * TASK: Record 64 samples and analyze
     * LEARNING: Circular buffers and statistical analysis
     */

    puts_USART1("\r\n=== Lab 3.1: Sensor Data Logger ===\r\n");
    puts_USART1("Logging 64 temperature samples...\r\n\r\n");

    // Initialize logger
    ADC_Logger_Init(&data_logger);

    // Collect 64 samples
    for (uint8_t i = 0; i < 64; i++)
    {
        uint16_t adc_value = Read_Adc_Data(TEMP_CHANNEL);
        ADC_Logger_Add(&data_logger, adc_value);

        if (i % 8 == 0)
        {
            char buffer[40];
            sprintf(buffer, "Logging... %u/64 samples\r", i);
            puts_USART1(buffer);
        }

        _delay_ms(100);
    }

    puts_USART1("\r\nLogging complete! Analyzing data...\r\n\r\n");

    // Calculate statistics
    uint16_t min_val = 1023, max_val = 0;
    uint32_t sum = 0;

    for (uint8_t i = 0; i < 64; i++)
    {
        uint16_t sample = ADC_Logger_Get(&data_logger, i);
        if (sample < min_val)
            min_val = sample;
        if (sample > max_val)
            max_val = sample;
        sum += sample;
    }

    uint16_t avg_val = sum / 64;
    uint16_t range = max_val - min_val;

    // Calculate standard deviation (simplified)
    uint32_t variance_sum = 0;
    for (uint8_t i = 0; i < 64; i++)
    {
        uint16_t sample = ADC_Logger_Get(&data_logger, i);
        int16_t diff = sample - avg_val;
        variance_sum += diff * diff;
    }
    uint16_t std_dev = sqrt(variance_sum / 64);

    // Display results
    char buffer[80];
    puts_USART1("=== Statistical Analysis ===\r\n");
    sprintf(buffer, "Samples:    64\r\n");
    puts_USART1(buffer);
    sprintf(buffer, "Minimum:    %u\r\n", min_val);
    puts_USART1(buffer);
    sprintf(buffer, "Maximum:    %u\r\n", max_val);
    puts_USART1(buffer);
    sprintf(buffer, "Average:    %u\r\n", avg_val);
    puts_USART1(buffer);
    sprintf(buffer, "Range:      %u\r\n", range);
    puts_USART1(buffer);
    sprintf(buffer, "Std Dev:    %u\r\n", std_dev);
    puts_USART1(buffer);

    // Display histogram
    puts_USART1("\r\n=== Data Histogram ===\r\n");

    // Create 8 bins
    uint8_t bins[8] = {0};
    uint16_t bin_size = (range + 7) / 8;

    for (uint8_t i = 0; i < 64; i++)
    {
        uint16_t sample = ADC_Logger_Get(&data_logger, i);
        uint8_t bin = (sample - min_val) / bin_size;
        if (bin > 7)
            bin = 7;
        bins[bin]++;
    }

    for (uint8_t i = 0; i < 8; i++)
    {
        sprintf(buffer, "Bin %u: [", i);
        puts_USART1(buffer);

        for (uint8_t j = 0; j < bins[i]; j++)
        {
            putch_USART1('*');
        }

        sprintf(buffer, "] %u\r\n", bins[i]);
        puts_USART1(buffer);
    }

    lab_score += 150;
    puts_USART1("\r\nScore: +150 points (Data logging complete)\r\n");
}

void lab_ex3_noise_filtering(void)
{
    /*
     * CHALLENGE: Compare filtering techniques
     * TASK: Test median filter vs moving average
     * LEARNING: Noise reduction methods
     */

    puts_USART1("\r\n=== Lab 3.2: Noise Filtering Comparison ===\r\n");
    puts_USART1("Comparing RAW vs MEDIAN vs MOVING AVERAGE\r\n\r\n");

    puts_USART1("Reading potentiometer for 20 seconds...\r\n");
    puts_USART1("Try to keep it steady at 50% position\r\n\r\n");

    Reset_Moving_Average(POT_CHANNEL);

    uint32_t raw_variance = 0, median_variance = 0, avg_variance = 0;
    uint16_t raw_mean = 0, median_mean = 0, avg_mean = 0;

    // First pass: calculate means
    for (uint8_t i = 0; i < 100; i++)
    {
        uint16_t raw = Read_Adc_Data(POT_CHANNEL);
        uint16_t median = Read_Adc_Median(POT_CHANNEL, 5);
        uint16_t moving_avg = Read_Adc_Moving_Average(POT_CHANNEL);

        raw_mean += raw / 100;
        median_mean += median / 100;
        avg_mean += moving_avg / 100;

        _delay_ms(200);
    }

    Reset_Moving_Average(POT_CHANNEL);

    // Second pass: calculate variance
    char buffer[120];
    puts_USART1("Method     | Current | Deviation\r\n");
    puts_USART1("-----------|---------|----------\r\n");

    for (uint8_t i = 0; i < 100; i++)
    {
        uint16_t raw = Read_Adc_Data(POT_CHANNEL);
        uint16_t median = Read_Adc_Median(POT_CHANNEL, 5);
        uint16_t moving_avg = Read_Adc_Moving_Average(POT_CHANNEL);

        int16_t raw_diff = raw - raw_mean;
        int16_t median_diff = median - median_mean;
        int16_t avg_diff = moving_avg - avg_mean;

        raw_variance += (raw_diff * raw_diff) / 100;
        median_variance += (median_diff * median_diff) / 100;
        avg_variance += (avg_diff * avg_diff) / 100;

        sprintf(buffer, "Raw:   %4u | %4u    | %+5d\r\n", raw, raw_mean, raw_diff);
        puts_USART1(buffer);
        sprintf(buffer, "Median:%4u | %4u    | %+5d\r\n", median, median_mean, median_diff);
        puts_USART1(buffer);
        sprintf(buffer, "MovAvg:%4u | %4u    | %+5d\r\n\r\n", moving_avg, avg_mean, avg_diff);
        puts_USART1(buffer);

        _delay_ms(200);
    }

    uint16_t raw_std = sqrt(raw_variance);
    uint16_t median_std = sqrt(median_variance);
    uint16_t avg_std = sqrt(avg_variance);

    puts_USART1("\r\n=== Noise Analysis Results ===\r\n");
    sprintf(buffer, "Raw Reading    - StdDev: %u\r\n", raw_std);
    puts_USART1(buffer);
    sprintf(buffer, "Median Filter  - StdDev: %u (%.1f%% reduction)\r\n",
            median_std, 100.0 * (raw_std - median_std) / raw_std);
    puts_USART1(buffer);
    sprintf(buffer, "Moving Average - StdDev: %u (%.1f%% reduction)\r\n",
            avg_std, 100.0 * (raw_std - avg_std) / raw_std);
    puts_USART1(buffer);

    lab_score += 125;
}

/*
 * =============================================================================
 * LAB EXERCISE 4: TEAM CHALLENGES (25 minutes)
 * =============================================================================
 * OBJECTIVE: Collaborative sensor projects
 * DIFFICULTY: ★★★★★ (Expert)
 */

void lab_ex4_multi_sensor_dashboard(void)
{
    /*
     * CHALLENGE: Create real-time sensor dashboard
     * TASK: Display multiple sensors simultaneously
     * LEARNING: Multi-channel ADC and display formatting
     */

    puts_USART1("\r\n=== Lab 4.1: Multi-Sensor Dashboard ===\r\n");
    puts_USART1("Real-time monitoring of all sensors\r\n\r\n");

    // Initialize statistics for all channels
    ADC_Init_Statistics(&pot_stats);
    ADC_Init_Statistics(&temp_stats);
    ADC_Init_Statistics(&light_stats);

    puts_USART1("Monitoring for 30 seconds. Press 'Q' to quit.\r\n\r\n");

    for (uint16_t i = 0; i < 300; i++)
    {
        // Read all sensors
        uint16_t pot = Read_Adc_Data(POT_CHANNEL);
        uint16_t temp = Read_Adc_Data(TEMP_CHANNEL);
        uint16_t light = Read_Adc_Data(LIGHT_CHANNEL);

        // Update statistics
        ADC_Update_Statistics(&pot_stats, pot);
        ADC_Update_Statistics(&temp_stats, temp);
        ADC_Update_Statistics(&light_stats, light);

        // Display dashboard
        char buffer[100];
        puts_USART1("\033[2J\033[H"); // Clear screen (VT100)

        puts_USART1("╔════════════════════════════════════════════════════════╗\r\n");
        puts_USART1("║         MULTI-SENSOR MONITORING DASHBOARD             ║\r\n");
        puts_USART1("╠════════════════════════════════════════════════════════╣\r\n");

        sprintf(buffer, "║ Potentiometer:  %4u  [", pot);
        puts_USART1(buffer);
        uint8_t bar_pot = pot / 51; // 0-20
        for (uint8_t j = 0; j < 20; j++)
            putch_USART1(j < bar_pot ? '█' : '░');
        puts_USART1("]  ║\r\n");

        sprintf(buffer, "║   Min: %4u  Max: %4u  Avg: %4u            ║\r\n",
                pot_stats.min, pot_stats.max, pot_stats.avg);
        puts_USART1(buffer);

        puts_USART1("║                                                        ║\r\n");

        sprintf(buffer, "║ Temperature:    %4u  [", temp);
        puts_USART1(buffer);
        uint8_t bar_temp = temp / 51;
        for (uint8_t j = 0; j < 20; j++)
            putch_USART1(j < bar_temp ? '█' : '░');
        puts_USART1("]  ║\r\n");

        sprintf(buffer, "║   Min: %4u  Max: %4u  Avg: %4u            ║\r\n",
                temp_stats.min, temp_stats.max, temp_stats.avg);
        puts_USART1(buffer);

        puts_USART1("║                                                        ║\r\n");

        sprintf(buffer, "║ Light Sensor:   %4u  [", light);
        puts_USART1(buffer);
        uint8_t bar_light = light / 51;
        for (uint8_t j = 0; j < 20; j++)
            putch_USART1(j < bar_light ? '█' : '░');
        puts_USART1("]  ║\r\n");

        sprintf(buffer, "║   Min: %4u  Max: %4u  Avg: %4u            ║\r\n",
                light_stats.min, light_stats.max, light_stats.avg);
        puts_USART1(buffer);

        puts_USART1("╠════════════════════════════════════════════════════════╣\r\n");
        sprintf(buffer, "║ Samples: %5u                    Press 'Q' to quit ║\r\n", i + 1);
        puts_USART1(buffer);
        puts_USART1("╚════════════════════════════════════════════════════════╝\r\n");

        _delay_ms(100);

        if (UCSR1A & (1 << RXC1))
        {
            char c = UDR1;
            if (c == 'Q' || c == 'q')
                break;
        }
    }

    puts_USART1("\r\nDashboard monitoring complete!\r\n");
    lab_score += 200;
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
    puts_USART1("  ADC PROGRAMMING - LAB EXERCISES\r\n");
    puts_USART1("========================================\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 1: Sensor Calibration\r\n");
    puts_USART1("  1. Temperature Calibration\r\n");
    puts_USART1("  2. Potentiometer Calibration\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 2: Threshold Detection\r\n");
    puts_USART1("  3. Temperature Alarm System\r\n");
    puts_USART1("  4. Light Level Detector\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 3: Data Logging & Analysis\r\n");
    puts_USART1("  5. Sensor Data Logger\r\n");
    puts_USART1("  6. Noise Filtering Comparison\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 4: Team Challenges\r\n");
    puts_USART1("  7. Multi-Sensor Dashboard\r\n");
    puts_USART1("\r\n");
    puts_USART1("  0. Run All Exercises\r\n");
    puts_USART1("  X. Exit Lab\r\n");
    puts_USART1("\r\n");
    sprintf(buffer, "Current Score: %u points\r\n\r\n", lab_score);
    puts_USART1(buffer);
    puts_USART1("Select exercise (1-7, 0, X): ");
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
    puts_USART1("*  ATmega128 ADC PROGRAMMING LAB               *\r\n");
    puts_USART1("*  Hands-On Sensor Exercises                   *\r\n");
    puts_USART1("*************************************************\r\n");
    puts_USART1("\r\n");
    puts_USART1("Welcome to the ADC Programming Lab!\r\n");
    puts_USART1("Master analog sensors through practical exercises.\r\n");

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
            lab_ex1_temperature_calibration();
            break;
        case '2':
            lab_ex1_potentiometer_calibration();
            break;
        case '3':
            lab_ex2_temperature_alarm();
            break;
        case '4':
            lab_ex2_light_level_detector();
            break;
        case '5':
            lab_ex3_data_logger();
            break;
        case '6':
            lab_ex3_noise_filtering();
            break;
        case '7':
            lab_ex4_multi_sensor_dashboard();
            break;

        case '0':
            puts_USART1("\r\n*** RUNNING ALL EXERCISES ***\r\n");
            lab_ex1_temperature_calibration();
            lab_ex1_potentiometer_calibration();
            lab_ex2_temperature_alarm();
            lab_ex2_light_level_detector();
            lab_ex3_data_logger();
            lab_ex3_noise_filtering();
            lab_ex4_multi_sensor_dashboard();

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
