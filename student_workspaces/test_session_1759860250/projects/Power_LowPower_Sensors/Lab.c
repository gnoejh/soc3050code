/*
 * =============================================================================
 * POWER LOW-POWER SENSORS - HANDS-ON LAB EXERCISES
 * =============================================================================
 * ATmega128 Educational Framework - Lab Session
 *
 * OBJECTIVE: Master low-power operation techniques for sensor applications
 * DURATION: 80 minutes
 * DIFFICULTY: Advanced
 *
 * STUDENTS WILL:
 * - Implement power management strategies for sensor applications
 * - Use sleep modes to reduce power consumption
 * - Design wake-up triggers and interrupt-driven sensing
 * - Optimize ADC operations for low power
 * - Measure and calculate power consumption
 *
 * HARDWARE REQUIRED:
 * - ATmega128 board
 * - Multiple sensors (temperature, light, accelerometer)
 * - Current measurement setup (multimeter or current sensor)
 * - External wake-up sources (buttons, timers)
 * - Power supply with voltage regulation
 * - Optional: Power measurement IC (INA219)
 *
 * POWER MODES COVERED:
 * - Idle Mode: CPU stopped, peripherals running
 * - ADC Noise Reduction: Optimized for ADC operation
 * - Power-down: All oscillators stopped, only external interrupts
 * - Power-save: Timer2 running for RTC applications
 * - Standby: External oscillator running
 * - Extended Standby: Timer2 + external oscillator
 *
 * LAB STRUCTURE:
 * - Exercise 1: Power measurement and sleep mode basics (25 min)
 * - Exercise 2: Interrupt-driven sensor reading (20 min)
 * - Exercise 3: Low-power ADC optimization (20 min)
 * - Exercise 4: Advanced power management system (15 min)
 *
 * =============================================================================
 */

#include "config.h"
#include <avr/sleep.h>
#include <avr/power.h>

// Power management configuration
#define POWER_LED_PIN 6     // PB6 - Power status LED
#define WAKE_BUTTON_PIN 0   // PE0 - External interrupt 0
#define SENSOR_ENABLE_PIN 7 // PB7 - Sensor power control

// Power measurement configuration (if using INA219 or similar)
#define CURRENT_SENSOR_ADDR 0x40 // I2C address for current sensor

// Sensor thresholds for wake-up
#define TEMP_THRESHOLD_HIGH 30 // °C
#define TEMP_THRESHOLD_LOW 10  // °C
#define LIGHT_THRESHOLD 200    // Light level
#define ACCEL_THRESHOLD 100    // Acceleration change

// Power management state
typedef enum
{
    POWER_ACTIVE,
    POWER_IDLE,
    POWER_SENSOR_ONLY,
    POWER_DEEP_SLEEP,
    POWER_EMERGENCY
} power_state_t;

// Lab session variables
uint16_t lab_score = 0;
uint32_t total_awake_time = 0;
uint32_t total_sleep_time = 0;
uint16_t wake_events = 0;
uint16_t sensor_readings = 0;
power_state_t current_power_state = POWER_ACTIVE;

// Power measurement variables
uint32_t active_current_ua = 0;    // Microamps in active mode
uint32_t sleep_current_ua = 0;     // Microamps in sleep mode
uint16_t supply_voltage_mv = 5000; // Supply voltage in millivolts

/*
 * =============================================================================
 * POWER MANAGEMENT FUNCTIONS
 * =============================================================================
 */

void power_init(void)
{
    // Configure power control pins
    DDRB |= (1 << POWER_LED_PIN) | (1 << SENSOR_ENABLE_PIN);

    // Configure wake-up button with pull-up
    DDRE &= ~(1 << WAKE_BUTTON_PIN);
    PORTE |= (1 << WAKE_BUTTON_PIN);

    // Enable external interrupt INT0 (PE0)
    EICRA |= (1 << ISC01); // Falling edge
    EIMSK |= (1 << INT0);  // Enable INT0

    // Initialize with sensors enabled
    PORTB |= (1 << SENSOR_ENABLE_PIN); // Sensors ON
    PORTB |= (1 << POWER_LED_PIN);     // Power LED ON

    // Disable unused peripherals for power saving
    power_spi_disable();
    power_twi_disable();
    power_timer1_disable();
    power_timer3_disable();
}

void enter_sleep_mode(uint8_t mode)
{
    set_sleep_mode(mode);

    // Update power LED
    if (mode == SLEEP_MODE_PWR_DOWN || mode == SLEEP_MODE_PWR_SAVE)
    {
        PORTB &= ~(1 << POWER_LED_PIN); // LED OFF in deep sleep
    }

    // Enable sleep and enter sleep mode
    sleep_enable();
    sei(); // Enable interrupts
    sleep_cpu();

    // Wake up here
    sleep_disable();

    // Restore power LED
    PORTB |= (1 << POWER_LED_PIN);

    wake_events++;
}

void sensors_power_control(uint8_t enable)
{
    if (enable)
    {
        PORTB |= (1 << SENSOR_ENABLE_PIN);
        _delay_ms(10); // Sensor startup time
    }
    else
    {
        PORTB &= ~(1 << SENSOR_ENABLE_PIN);
    }
}

uint16_t read_sensor_optimized(uint8_t channel)
{
    // Enable only ADC for reading
    power_adc_enable();

    // ADC setup for low-power operation
    ADMUX = (1 << REFS0) | channel;                     // AVCC reference
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Enable, prescaler 64

    // Wait for reference to settle
    _delay_us(100);

    // Start conversion
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC))
        ;

    uint16_t result = ADCW;

    // Disable ADC to save power
    ADCSRA &= ~(1 << ADEN);
    power_adc_disable();

    sensor_readings++;
    return result;
}

uint32_t simulate_current_measurement(void)
{
    /*
     * Simulate current measurement - in real application,
     * this would read from a current sensor like INA219
     */
    uint32_t estimated_current = 0;

    switch (current_power_state)
    {
    case POWER_ACTIVE:
        estimated_current = 20000; // 20mA active
        break;
    case POWER_IDLE:
        estimated_current = 8000; // 8mA idle
        break;
    case POWER_SENSOR_ONLY:
        estimated_current = 3000; // 3mA sensors only
        break;
    case POWER_DEEP_SLEEP:
        estimated_current = 50; // 50µA deep sleep
        break;
    case POWER_EMERGENCY:
        estimated_current = 10; // 10µA emergency
        break;
    }

    return estimated_current;
}

/*
 * =============================================================================
 * INTERRUPT SERVICE ROUTINES
 * =============================================================================
 */

ISR(INT0_vect)
{
    // External interrupt - wake from sleep
    _delay_ms(50); // Debounce
}

ISR(TIMER2_COMP_vect)
{
    // Timer2 compare match - for periodic wake-up
    // Used in power-save mode for regular sensor readings
}

/*
 * =============================================================================
 * LAB EXERCISE 1: POWER MEASUREMENT AND SLEEP BASICS (25 minutes)
 * =============================================================================
 * OBJECTIVE: Understand power consumption in different modes
 * DIFFICULTY: ★★★☆☆ (Medium)
 */

void lab_ex1_power_measurement(void)
{
    /*
     * CHALLENGE: Measure power consumption in different operating modes
     * TASK: Compare active vs. sleep power consumption
     * LEARNING: Power measurement techniques, sleep mode effects
     */

    puts_USART1("\\r\\n=== Lab 1: Power Measurement ===\\r\\n");
    puts_USART1("Measuring power consumption in different modes\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "POWER MEASUREMENT");
    lcd_string(1, 0, "Mode comparison");

    power_init();

    // Test 1: Active mode measurement
    puts_USART1("Test 1: Active mode power consumption\\r\\n");
    puts_USART1("All peripherals enabled, CPU running\\r\\n");
    lcd_string(3, 0, "Mode: ACTIVE");

    current_power_state = POWER_ACTIVE;

    // Enable all peripherals
    power_all_enable();

    // Simulate active work for measurement
    for (uint8_t i = 0; i < 10; i++)
    {
        // Read all sensors
        uint16_t temp = read_sensor_optimized(0);
        uint16_t light = read_sensor_optimized(1);
        uint16_t accel = read_sensor_optimized(2);

        char sensor_msg[60];
        sprintf(sensor_msg, "Sensors: T=%d, L=%d, A=%d\\r\\n", temp, light, accel);
        puts_USART1(sensor_msg);

        // Update LCD display
        char lcd_update[20];
        sprintf(lcd_update, "T:%d L:%d A:%d", temp, light, accel);
        lcd_string(4, 0, lcd_update);

        _delay_ms(1000);
        total_awake_time++;
    }

    active_current_ua = simulate_current_measurement();

    char active_msg[50];
    sprintf(active_msg, "Active current: %ld µA\\r\\n", active_current_ua);
    puts_USART1(active_msg);

    // Test 2: Idle mode measurement
    puts_USART1("\\r\\nTest 2: Idle mode power consumption\\r\\n");
    puts_USART1("CPU stopped, peripherals running\\r\\n");
    lcd_string(3, 0, "Mode: IDLE     ");

    current_power_state = POWER_IDLE;

    puts_USART1("Entering idle mode for 5 seconds...\\r\\n");
    lcd_string(4, 0, "Sleeping...");

    for (uint8_t i = 0; i < 5; i++)
    {
        char countdown[20];
        sprintf(countdown, "Idle: %d sec", 5 - i);
        lcd_string(5, 0, countdown);

        enter_sleep_mode(SLEEP_MODE_IDLE);
        _delay_ms(1000);
        total_sleep_time++;
    }

    uint32_t idle_current = simulate_current_measurement();

    char idle_msg[50];
    sprintf(idle_msg, "Idle current: %ld µA\\r\\n", idle_current);
    puts_USART1(idle_msg);

    // Test 3: Power-down mode measurement
    puts_USART1("\\r\\nTest 3: Power-down mode\\r\\n");
    puts_USART1("Press wake button to continue...\\r\\n");
    lcd_string(3, 0, "Mode: PWR-DOWN ");
    lcd_string(4, 0, "Press button");

    current_power_state = POWER_DEEP_SLEEP;

    // Disable non-essential peripherals
    sensors_power_control(0); // Turn off sensors

    enter_sleep_mode(SLEEP_MODE_PWR_DOWN);

    // Woken up by button press
    sensors_power_control(1); // Turn sensors back on

    sleep_current_ua = simulate_current_measurement();

    char sleep_msg[50];
    sprintf(sleep_msg, "Sleep current: %ld µA\\r\\n", sleep_current_ua);
    puts_USART1(sleep_msg);

    // Power efficiency analysis
    puts_USART1("\\r\\n=== POWER EFFICIENCY ANALYSIS ===\\r\\n");

    uint32_t power_active_uw = (active_current_ua * supply_voltage_mv) / 1000;
    uint32_t power_sleep_uw = (sleep_current_ua * supply_voltage_mv) / 1000;
    uint32_t efficiency_ratio = active_current_ua / (sleep_current_ua > 0 ? sleep_current_ua : 1);

    char analysis[80];
    sprintf(analysis, "Active power: %ld µW\\r\\n", power_active_uw);
    puts_USART1(analysis);

    sprintf(analysis, "Sleep power: %ld µW\\r\\n", power_sleep_uw);
    puts_USART1(analysis);

    sprintf(analysis, "Power reduction: %ldx\\r\\n", efficiency_ratio);
    puts_USART1(analysis);

    char lcd_efficiency[20];
    sprintf(lcd_efficiency, "Reduction: %ldx", efficiency_ratio);
    lcd_string(5, 0, lcd_efficiency);

    if (efficiency_ratio > 100)
    {
        lab_score += 150;
        puts_USART1("✓ Excellent power reduction achieved!\\r\\n");
    }
}

void lab_ex1_sleep_mode_comparison(void)
{
    /*
     * CHALLENGE: Compare different sleep modes and their wake-up sources
     * TASK: Test each sleep mode and measure wake-up behavior
     * LEARNING: Sleep mode selection, wake-up source configuration
     */

    puts_USART1("\\r\\n=== Lab 1.2: Sleep Mode Comparison ===\\r\\n");
    puts_USART1("Testing different sleep modes\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "SLEEP MODES");
    lcd_string(1, 0, "Comparison test");

    const char *mode_names[] = {
        "IDLE", "ADC_NR", "PWR_DOWN", "PWR_SAVE", "STANDBY", "EXT_STANDBY"};

    uint8_t modes[] = {
        SLEEP_MODE_IDLE,
        SLEEP_MODE_ADC,
        SLEEP_MODE_PWR_DOWN,
        SLEEP_MODE_PWR_SAVE,
        SLEEP_MODE_STANDBY,
        SLEEP_MODE_EXT_STANDBY};

    for (uint8_t i = 0; i < 6; i++)
    {
        char mode_msg[50];
        sprintf(mode_msg, "Testing %s mode...\\r\\n", mode_names[i]);
        puts_USART1(mode_msg);

        char lcd_mode[20];
        sprintf(lcd_mode, "Mode: %s", mode_names[i]);
        lcd_string(3, 0, lcd_mode);

        puts_USART1("Press button to wake up\\r\\n");
        lcd_string(4, 0, "Press button");

        uint32_t sleep_start = total_sleep_time;

        // Enter sleep mode
        enter_sleep_mode(modes[i]);

        uint32_t sleep_duration = total_sleep_time - sleep_start + 1;

        char wake_msg[50];
        sprintf(wake_msg, "Woke from %s mode\\r\\n", mode_names[i]);
        puts_USART1(wake_msg);

        char lcd_wake[20];
        sprintf(lcd_wake, "Woke: %ld sec", sleep_duration);
        lcd_string(5, 0, lcd_wake);

        _delay_ms(2000); // Show result
    }

    char wake_summary[50];
    sprintf(wake_summary, "Total wake events: %d\\r\\n", wake_events);
    puts_USART1(wake_summary);

    lab_score += 100;
}

/*
 * =============================================================================
 * LAB EXERCISE 2: INTERRUPT-DRIVEN SENSOR READING (20 minutes)
 * =============================================================================
 * OBJECTIVE: Implement interrupt-driven sensor monitoring
 * DIFFICULTY: ★★★★☆ (Medium-Hard)
 */

void lab_ex2_interrupt_sensing(void)
{
    /*
     * CHALLENGE: Create interrupt-driven sensor monitoring system
     * TASK: Use timer interrupts to wake up and read sensors periodically
     * LEARNING: Timer interrupts, periodic wake-up, power-efficient sensing
     */

    puts_USART1("\\r\\n=== Lab 2: Interrupt-Driven Sensing ===\\r\\n");
    puts_USART1("Implementing periodic sensor wake-up\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "INTERRUPT SENSE");
    lcd_string(1, 0, "Timer wake-up");

    // Configure Timer2 for periodic wake-up (1 second intervals)
    TCCR2 = (1 << WGM21) | (1 << CS22) | (1 << CS21) | (1 << CS20); // CTC mode, prescaler 1024
    OCR2 = 71;                                                      // For ~1 second at 7.3728MHz with 1024 prescaler
    TIMSK |= (1 << OCIE2);                                          // Enable Timer2 compare interrupt

    uint16_t sensor_samples = 0;
    uint16_t threshold_violations = 0;

    typedef struct
    {
        uint16_t temperature;
        uint16_t light_level;
        uint16_t acceleration;
        uint8_t timestamp;
    } sensor_data_t;

    sensor_data_t sensor_history[10]; // Store last 10 readings
    uint8_t history_index = 0;

    puts_USART1("Starting interrupt-driven sensing...\\r\\n");
    puts_USART1("System will wake every second to read sensors\\r\\n");
    puts_USART1("Press button to stop monitoring\\r\\n");

    current_power_state = POWER_SENSOR_ONLY;

    while (!button_pressed(0) && sensor_samples < 20)
    {
        lcd_string(3, 0, "Sleeping...");

        // Enter power-save mode (Timer2 continues running)
        enter_sleep_mode(SLEEP_MODE_PWR_SAVE);

        // Woken by Timer2 interrupt
        lcd_string(3, 0, "Reading sensors");

        // Read all sensors efficiently
        uint16_t temp = read_sensor_optimized(0);
        uint16_t light = read_sensor_optimized(1);
        uint16_t accel = read_sensor_optimized(2);

        // Store in history
        sensor_history[history_index].temperature = temp;
        sensor_history[history_index].light_level = light;
        sensor_history[history_index].acceleration = accel;
        sensor_history[history_index].timestamp = sensor_samples;

        history_index = (history_index + 1) % 10;

        // Check thresholds
        uint8_t alert = 0;
        if (temp > (TEMP_THRESHOLD_HIGH * 10) || temp < (TEMP_THRESHOLD_LOW * 10))
        {
            alert = 1;
            threshold_violations++;
        }
        if (light < LIGHT_THRESHOLD)
        {
            alert = 1;
            threshold_violations++;
        }
        if (accel > ACCEL_THRESHOLD)
        {
            alert = 1;
            threshold_violations++;
        }

        char sensor_msg[70];
        sprintf(sensor_msg, "Sample %d: T=%d, L=%d, A=%d %s\\r\\n",
                sensor_samples, temp, light, accel, alert ? "[ALERT]" : "");
        puts_USART1(sensor_msg);

        // Update LCD
        char lcd_data[20];
        sprintf(lcd_data, "S%d T%d L%d A%d", sensor_samples, temp, light, accel);
        lcd_string(4, 0, lcd_data);

        if (alert)
        {
            lcd_string(5, 0, "ALERT CONDITION");
        }
        else
        {
            char lcd_status[20];
            sprintf(lcd_status, "Normal - A:%d", threshold_violations);
            lcd_string(5, 0, lcd_status);
        }

        sensor_samples++;

        // Brief display time, then back to sleep
        _delay_ms(500);
    }

    // Analysis of interrupt-driven sensing
    puts_USART1("\\r\\n=== SENSING ANALYSIS ===\\r\\n");

    char analysis[60];
    sprintf(analysis, "Total samples: %d\\r\\n", sensor_samples);
    puts_USART1(analysis);

    sprintf(analysis, "Threshold violations: %d\\r\\n", threshold_violations);
    puts_USART1(analysis);

    sprintf(analysis, "Alert rate: %d%%\\r\\n", (threshold_violations * 100) / sensor_samples);
    puts_USART1(analysis);

    // Show sensor history
    puts_USART1("\\r\\nLast 5 sensor readings:\\r\\n");
    for (uint8_t i = 0; i < 5 && i < sensor_samples; i++)
    {
        uint8_t idx = (history_index - i - 1 + 10) % 10;
        char history_msg[60];
        sprintf(history_msg, "  [%d] T:%d, L:%d, A:%d\\r\\n",
                sensor_history[idx].timestamp,
                sensor_history[idx].temperature,
                sensor_history[idx].light_level,
                sensor_history[idx].acceleration);
        puts_USART1(history_msg);
    }

    if (sensor_samples >= 10)
    {
        lab_score += 200;
        puts_USART1("✓ Interrupt-driven sensing successful!\\r\\n");
    }

    // Disable Timer2 interrupt
    TIMSK &= ~(1 << OCIE2);
}

/*
 * =============================================================================
 * LAB EXERCISE 3: LOW-POWER ADC OPTIMIZATION (20 minutes)
 * =============================================================================
 * OBJECTIVE: Optimize ADC operations for minimum power consumption
 * DIFFICULTY: ★★★★☆ (Medium-Hard)
 */

void lab_ex3_adc_optimization(void)
{
    /*
     * CHALLENGE: Optimize ADC for lowest power consumption
     * TASK: Use ADC noise reduction mode and optimize conversion settings
     * LEARNING: ADC power optimization, noise reduction mode, timing optimization
     */

    puts_USART1("\\r\\n=== Lab 3: ADC Power Optimization ===\\r\\n");
    puts_USART1("Optimizing ADC for low-power operation\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "ADC OPTIMIZATION");
    lcd_string(1, 0, "Power efficiency");

    // Test different ADC configurations for power efficiency
    uint16_t config_results[4][3]; // [config][channel] results
    uint32_t config_times[4];      // Time for each configuration

    // Configuration 1: Standard ADC operation
    puts_USART1("Config 1: Standard ADC operation\\r\\n");
    lcd_string(3, 0, "Config: Standard");

    uint32_t start_time = 0;
    power_adc_enable();
    ADMUX = (1 << REFS0);                                              // AVCC reference
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable, prescaler 128

    for (uint8_t ch = 0; ch < 3; ch++)
    {
        ADMUX = (ADMUX & 0xF0) | ch;
        _delay_us(100); // Reference settling
        ADCSRA |= (1 << ADSC);
        while (ADCSRA & (1 << ADSC))
            start_time++;
        config_results[0][ch] = ADCW;
    }
    config_times[0] = start_time;
    power_adc_disable();

    char config1_msg[60];
    sprintf(config1_msg, "  Results: T=%d, L=%d, A=%d, Time=%ld\\r\\n",
            config_results[0][0], config_results[0][1], config_results[0][2], config_times[0]);
    puts_USART1(config1_msg);

    // Configuration 2: Lower prescaler (faster, higher power)
    puts_USART1("Config 2: Fast ADC (prescaler 64)\\r\\n");
    lcd_string(3, 0, "Config: Fast    ");

    start_time = 0;
    power_adc_enable();
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Prescaler 64

    for (uint8_t ch = 0; ch < 3; ch++)
    {
        ADMUX = (ADMUX & 0xF0) | ch;
        _delay_us(50);
        ADCSRA |= (1 << ADSC);
        while (ADCSRA & (1 << ADSC))
            start_time++;
        config_results[1][ch] = ADCW;
    }
    config_times[1] = start_time;
    power_adc_disable();

    char config2_msg[60];
    sprintf(config2_msg, "  Results: T=%d, L=%d, A=%d, Time=%ld\\r\\n",
            config_results[1][0], config_results[1][1], config_results[1][2], config_times[1]);
    puts_USART1(config2_msg);

    // Configuration 3: ADC Noise Reduction Mode
    puts_USART1("Config 3: ADC Noise Reduction Mode\\r\\n");
    lcd_string(3, 0, "Config: NoiseRed");

    start_time = 0;
    for (uint8_t ch = 0; ch < 3; ch++)
    {
        power_adc_enable();
        ADMUX = (1 << REFS0) | ch;
        ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1); // Enable interrupt

        // Use ADC Noise Reduction sleep mode
        set_sleep_mode(SLEEP_MODE_ADC);
        ADCSRA |= (1 << ADSC); // Start conversion
        sleep_enable();
        sei();
        sleep_cpu(); // CPU sleeps during conversion
        sleep_disable();

        config_results[2][ch] = ADCW;
        start_time++;
        power_adc_disable();
    }
    config_times[2] = start_time;

    char config3_msg[60];
    sprintf(config3_msg, "  Results: T=%d, L=%d, A=%d, Time=%ld\\r\\n",
            config_results[2][0], config_results[2][1], config_results[2][2], config_times[2]);
    puts_USART1(config3_msg);

    // Configuration 4: Ultra-low power (disable between readings)
    puts_USART1("Config 4: Ultra-low power mode\\r\\n");
    lcd_string(3, 0, "Config: Ultra   ");

    start_time = 0;
    for (uint8_t ch = 0; ch < 3; ch++)
    {
        // Power on ADC only when needed
        power_adc_enable();
        ADMUX = (1 << REFS0) | ch;
        ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

        _delay_us(200); // Extra settling time for accuracy

        ADCSRA |= (1 << ADSC);
        while (ADCSRA & (1 << ADSC))
            start_time++;
        config_results[3][ch] = ADCW;

        // Power off ADC immediately
        ADCSRA = 0;
        power_adc_disable();

        // Small delay between channels for power measurement
        _delay_ms(10);
    }
    config_times[3] = start_time;

    char config4_msg[60];
    sprintf(config4_msg, "  Results: T=%d, L=%d, A=%d, Time=%ld\\r\\n",
            config_results[3][0], config_results[3][1], config_results[3][2], config_times[3]);
    puts_USART1(config4_msg);

    // Analysis and optimization recommendations
    puts_USART1("\\r\\n=== ADC OPTIMIZATION ANALYSIS ===\\r\\n");

    // Find most efficient configuration
    uint8_t best_config = 0;
    uint32_t best_efficiency = config_times[0];

    for (uint8_t i = 1; i < 4; i++)
    {
        if (config_times[i] < best_efficiency)
        {
            best_efficiency = config_times[i];
            best_config = i;
        }
    }

    const char *config_names[] = {"Standard", "Fast", "Noise Reduction", "Ultra-Low Power"};

    char best_msg[50];
    sprintf(best_msg, "Most efficient: Config %d (%s)\\r\\n", best_config + 1, config_names[best_config]);
    puts_USART1(best_msg);

    char lcd_best[20];
    sprintf(lcd_best, "Best: %s", config_names[best_config]);
    lcd_string(4, 0, lcd_best);

    // Calculate power savings
    uint32_t power_savings = ((config_times[0] - best_efficiency) * 100) / config_times[0];

    char savings_msg[50];
    sprintf(savings_msg, "Power savings: %ld%% vs standard\\r\\n", power_savings);
    puts_USART1(savings_msg);

    char lcd_savings[20];
    sprintf(lcd_savings, "Savings: %ld%%", power_savings);
    lcd_string(5, 0, lcd_savings);

    if (power_savings > 30)
    {
        lab_score += 200;
        puts_USART1("✓ Excellent ADC optimization!\\r\\n");
    }
}

// ADC interrupt service routine for noise reduction mode
ISR(ADC_vect)
{
    // ADC conversion complete - wake up from sleep
}

/*
 * =============================================================================
 * LAB EXERCISE 4: ADVANCED POWER MANAGEMENT (15 minutes)
 * =============================================================================
 * OBJECTIVE: Build a complete power management system
 * DIFFICULTY: ★★★★★ (Hard)
 */

void lab_ex4_power_management_system(void)
{
    /*
     * CHALLENGE: Create an intelligent power management system
     * TASK: Implement adaptive power modes based on sensor activity
     * LEARNING: State machines, adaptive algorithms, system optimization
     */

    puts_USART1("\\r\\n=== Lab 4: Advanced Power Management ===\\r\\n");
    puts_USART1("Building intelligent power management system\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "ADVANCED POWER");
    lcd_string(1, 0, "Intelligent mgmt");

    // Power management state machine
    typedef enum
    {
        PM_STARTUP,
        PM_MONITORING,
        PM_LOW_ACTIVITY,
        PM_EMERGENCY_SAVE,
        PM_MAINTENANCE
    } pm_state_t;

    pm_state_t pm_state = PM_STARTUP;
    uint16_t activity_counter = 0;
    uint16_t low_activity_threshold = 5;
    uint16_t emergency_threshold = 2;
    uint32_t system_runtime = 0;

    puts_USART1("Initializing intelligent power management...\\r\\n");

    while (system_runtime < 30 && !button_pressed(0)) // Run for 30 cycles
    {
        char state_msg[50];
        switch (pm_state)
        {
        case PM_STARTUP:
            sprintf(state_msg, "State: STARTUP (cycle %ld)\\r\\n", system_runtime);
            puts_USART1(state_msg);
            lcd_string(3, 0, "State: STARTUP ");

            // Full system initialization
            current_power_state = POWER_ACTIVE;
            power_all_enable();
            sensors_power_control(1);

            // Read initial sensor baselines
            uint16_t baseline_temp = read_sensor_optimized(0);
            uint16_t baseline_light = read_sensor_optimized(1);
            uint16_t baseline_accel = read_sensor_optimized(2);

            char baseline_msg[60];
            sprintf(baseline_msg, "Baselines: T=%d, L=%d, A=%d\\r\\n",
                    baseline_temp, baseline_light, baseline_accel);
            puts_USART1(baseline_msg);

            pm_state = PM_MONITORING;
            break;

        case PM_MONITORING:
            sprintf(state_msg, "State: MONITORING (cycle %ld)\\r\\n", system_runtime);
            puts_USART1(state_msg);
            lcd_string(3, 0, "State: MONITOR ");

            current_power_state = POWER_SENSOR_ONLY;

            // Read sensors and check for activity
            uint16_t current_temp = read_sensor_optimized(0);
            uint16_t current_light = read_sensor_optimized(1);
            uint16_t current_accel = read_sensor_optimized(2);

            // Detect activity (significant changes)
            uint8_t activity_detected = 0;
            if (abs(current_temp - baseline_temp) > 50)
                activity_detected = 1;
            if (abs(current_light - baseline_light) > 100)
                activity_detected = 1;
            if (abs(current_accel - baseline_accel) > 80)
                activity_detected = 1;

            if (activity_detected)
            {
                activity_counter++;
                puts_USART1("  Activity detected!\\r\\n");
            }
            else
            {
                if (activity_counter > 0)
                    activity_counter--;
                puts_USART1("  No significant activity\\r\\n");
            }

            char activity_msg[50];
            sprintf(activity_msg, "  Activity level: %d\\r\\n", activity_counter);
            puts_USART1(activity_msg);

            char lcd_activity[20];
            sprintf(lcd_activity, "Activity: %d", activity_counter);
            lcd_string(4, 0, lcd_activity);

            // State transitions based on activity
            if (activity_counter <= emergency_threshold)
            {
                pm_state = PM_EMERGENCY_SAVE;
            }
            else if (activity_counter <= low_activity_threshold)
            {
                pm_state = PM_LOW_ACTIVITY;
            }

            // Update baselines slowly
            baseline_temp = (baseline_temp * 3 + current_temp) / 4;
            baseline_light = (baseline_light * 3 + current_light) / 4;
            baseline_accel = (baseline_accel * 3 + current_accel) / 4;

            _delay_ms(1000);
            break;

        case PM_LOW_ACTIVITY:
            sprintf(state_msg, "State: LOW_ACTIVITY (cycle %ld)\\r\\n", system_runtime);
            puts_USART1(state_msg);
            lcd_string(3, 0, "State: LOW_ACT ");

            current_power_state = POWER_IDLE;

            // Reduce sensor reading frequency
            puts_USART1("  Entering power-save mode...\\r\\n");
            lcd_string(4, 0, "Power saving");

            // Read sensors less frequently
            if (system_runtime % 3 == 0)
            { // Every 3rd cycle
                uint16_t temp = read_sensor_optimized(0);
                uint16_t light = read_sensor_optimized(1);

                // Check if activity has resumed
                if (abs(temp - baseline_temp) > 100 || abs(light - baseline_light) > 150)
                {
                    activity_counter += 2; // Quick recovery
                    puts_USART1("  Activity resumed!\\r\\n");
                }
            }

            // Enter idle mode between operations
            enter_sleep_mode(SLEEP_MODE_IDLE);

            // State transitions
            if (activity_counter > low_activity_threshold)
            {
                pm_state = PM_MONITORING;
            }
            else if (activity_counter <= emergency_threshold)
            {
                pm_state = PM_EMERGENCY_SAVE;
            }

            _delay_ms(2000); // Longer cycle time
            break;

        case PM_EMERGENCY_SAVE:
            sprintf(state_msg, "State: EMERGENCY_SAVE (cycle %ld)\\r\\n", system_runtime);
            puts_USART1(state_msg);
            lcd_string(3, 0, "State: EMERGENCY");

            current_power_state = POWER_EMERGENCY;

            puts_USART1("  Emergency power saving activated!\\r\\n");
            lcd_string(4, 0, "EMERGENCY MODE");

            // Disable non-essential sensors
            sensors_power_control(0);

            // Only check wake-up button
            puts_USART1("  Deep sleep - button wake only\\r\\n");
            lcd_string(5, 0, "Deep sleep");

            enter_sleep_mode(SLEEP_MODE_PWR_DOWN);

            // If woken up, check for recovery
            sensors_power_control(1);
            _delay_ms(100); // Sensor stabilization

            uint16_t recovery_temp = read_sensor_optimized(0);
            if (abs(recovery_temp - baseline_temp) > 200)
            {
                activity_counter = low_activity_threshold + 2; // Force recovery
                puts_USART1("  Emergency recovery - activity detected!\\r\\n");
                pm_state = PM_MONITORING;
            }
            else
            {
                puts_USART1("  Still in emergency mode\\r\\n");
            }

            _delay_ms(5000); // Long emergency cycle
            break;

        case PM_MAINTENANCE:
            sprintf(state_msg, "State: MAINTENANCE (cycle %ld)\\r\\n", system_runtime);
            puts_USART1(state_msg);
            lcd_string(3, 0, "State: MAINT   ");

            // Periodic system maintenance
            puts_USART1("  System maintenance cycle\\r\\n");

            // Reset activity counter periodically
            if (system_runtime % 10 == 0)
            {
                activity_counter = (activity_counter + 1) / 2; // Decay
            }

            pm_state = PM_MONITORING;
            break;
        }

        // Update power efficiency display
        uint32_t estimated_power = simulate_current_measurement();
        char power_msg[50];
        sprintf(power_msg, "  Estimated power: %ld µA\\r\\n", estimated_power);
        puts_USART1(power_msg);

        char lcd_power[20];
        sprintf(lcd_power, "Power: %ld µA", estimated_power);
        lcd_string(5, 0, lcd_power);

        system_runtime++;
        total_awake_time++;

        // Maintenance check
        if (system_runtime % 15 == 0)
        {
            pm_state = PM_MAINTENANCE;
        }
    }

    // Final power management analysis
    puts_USART1("\\r\\n=== POWER MANAGEMENT ANALYSIS ===\\r\\n");

    char final_analysis[80];
    sprintf(final_analysis, "System runtime: %ld cycles\\r\\n", system_runtime);
    puts_USART1(final_analysis);

    sprintf(final_analysis, "Final activity level: %d\\r\\n", activity_counter);
    puts_USART1(final_analysis);

    sprintf(final_analysis, "Total wake events: %d\\r\\n", wake_events);
    puts_USART1(final_analysis);

    sprintf(final_analysis, "Total sensor readings: %d\\r\\n", sensor_readings);
    puts_USART1(final_analysis);

    uint32_t avg_power = (active_current_ua + sleep_current_ua) / 2;
    sprintf(final_analysis, "Average power consumption: %ld µA\\r\\n", avg_power);
    puts_USART1(final_analysis);

    if (system_runtime >= 20 && wake_events >= 5)
    {
        lab_score += 250;
        puts_USART1("✓ Advanced power management system completed!\\r\\n");
    }
}

/*
 * =============================================================================
 * LAB MAIN PROGRAM - EXERCISE SELECTION
 * =============================================================================
 */

void show_lab_menu(void)
{
    puts_USART1("\\r\\n");
    puts_USART1("==============================================\\r\\n");
    puts_USART1("   POWER LOW-POWER SENSORS - LAB EXERCISES   \\r\\n");
    puts_USART1("==============================================\\r\\n");
    puts_USART1("1. Power Measurement & Sleep Mode Basics    \\r\\n");
    puts_USART1("2. Interrupt-Driven Sensor Reading          \\r\\n");
    puts_USART1("3. Low-Power ADC Optimization               \\r\\n");
    puts_USART1("4. Advanced Power Management System         \\r\\n");
    puts_USART1("                                              \\r\\n");
    puts_USART1("0. Run All Exercises                         \\r\\n");
    puts_USART1("X. Exit Lab                                   \\r\\n");
    puts_USART1("==============================================\\r\\n");
    char score_msg[50];
    sprintf(score_msg, "Current Score: %d points\\r\\n", lab_score);
    puts_USART1(score_msg);
    char power_msg[60];
    sprintf(power_msg, "Power Stats: %ld µA active, %ld µA sleep\\r\\n", active_current_ua, sleep_current_ua);
    puts_USART1(power_msg);
    char timing_msg[60];
    sprintf(timing_msg, "Timing: %ld awake, %ld sleep, %d wake events\\r\\n", total_awake_time, total_sleep_time, wake_events);
    puts_USART1(timing_msg);
    puts_USART1("Select exercise (1-4, 0, X): ");
}

int main(void)
{
    init_devices();
    sei(); // Enable global interrupts for power management

    puts_USART1("\\r\\n*** POWER LOW-POWER SENSORS LAB SESSION ***\\r\\n");
    puts_USART1("Welcome to power management and low-power sensing!\\r\\n");
    puts_USART1("This lab covers sleep modes, power optimization, and efficiency\\r\\n");
    puts_USART1("Ensure current measurement setup is ready\\r\\n");

    lcd_clear();
    lcd_string(1, 0, "POWER MGMT LAB");
    lcd_string(2, 0, "Low-power sensors");
    lcd_string(4, 0, "Use Serial Menu");

    while (1)
    {
        show_lab_menu();
        char choice = getch_USART1();
        putch_USART1(choice);
        putch_USART1('\\r');
        putch_USART1('\\n');

        switch (choice)
        {
        case '1':
            lab_ex1_power_measurement();
            lab_ex1_sleep_mode_comparison();
            break;

        case '2':
            lab_ex2_interrupt_sensing();
            break;

        case '3':
            lab_ex3_adc_optimization();
            break;

        case '4':
            lab_ex4_power_management_system();
            break;

        case '0':
            puts_USART1("\\r\\n*** RUNNING ALL EXERCISES ***\\r\\n");
            lab_ex1_power_measurement();
            lab_ex1_sleep_mode_comparison();
            lab_ex2_interrupt_sensing();
            lab_ex3_adc_optimization();
            lab_ex4_power_management_system();

            char final_buffer[80];
            sprintf(final_buffer, "\\r\\n*** ALL EXERCISES COMPLETE! ***\\r\\nFinal Score: %d points\\r\\n", lab_score);
            puts_USART1(final_buffer);
            break;

        case 'X':
        case 'x':
            puts_USART1("\\r\\nExiting lab. Great work on power management!\\r\\n");
            puts_USART1("Remember: Every µA counts in battery applications!\\r\\n");
            lcd_clear();
            lcd_string(2, 0, "LAB COMPLETE!");
            char exit_score[30];
            sprintf(exit_score, "Score: %d pts", lab_score);
            lcd_string(3, 0, exit_score);
            while (1)
                ;

        default:
            puts_USART1("Invalid choice. Please try again.\\r\\n");
        }

        puts_USART1("\\r\\nPress any key to continue...\\r\\n");
        getch_USART1();
    }

    return 0;
}