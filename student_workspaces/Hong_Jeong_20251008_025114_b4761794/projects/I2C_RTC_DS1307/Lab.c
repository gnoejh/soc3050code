/*
 * =============================================================================
 * I2C RTC DS1307 - HANDS-ON LAB EXERCISES
 * =============================================================================
 * ATmega128 Educational Framework - Lab Session
 *
 * OBJECTIVE: Master real-time clock integration using DS1307 over I2C
 * DURATION: 75 minutes
 * DIFFICULTY: Advanced
 *
 * STUDENTS WILL:
 * - Interface with DS1307 real-time clock via I2C
 * - Understand BCD (Binary-Coded Decimal) time format
 * - Implement time/date setting and reading functions
 * - Create alarm and scheduling systems
 * - Build time-based automation and logging
 *
 * HARDWARE REQUIRED:
 * - ATmega128 board with I2C/TWI interface
 * - DS1307 Real-Time Clock module
 * - 32.768 kHz crystal oscillator
 * - 3V coin cell battery for backup power
 * - Pull-up resistors on SDA/SCL lines (4.7kΩ typical)
 * - LCD display for time visualization
 * - LEDs for alarm/schedule indication
 * - Push buttons for time adjustment
 *
 * I2C RTC DS1307 CONCEPTS:
 * - Real-time clock operation and crystal timing
 * - Binary-Coded Decimal (BCD) format
 * - I2C communication with 7-bit addressing
 * - Time registers and memory organization
 * - Battery backup and power management
 * - Clock calibration and accuracy
 *
 * LAB STRUCTURE:
 * - Exercise 1: DS1307 initialization and basic time (25 min)
 * - Exercise 2: Time/date setting and BCD handling (20 min)
 * - Exercise 3: Alarm system and scheduling (15 min)
 * - Exercise 4: Advanced time-based applications (15 min)
 *
 * =============================================================================
 */

#include "config.h"

// DS1307 I2C Configuration
#define DS1307_ADDRESS 0x68 // 7-bit I2C address
#define DS1307_WRITE_ADDR (DS1307_ADDRESS << 1)
#define DS1307_READ_ADDR ((DS1307_ADDRESS << 1) | 0x01)

// DS1307 Register Addresses
#define DS1307_REG_SECONDS 0x00
#define DS1307_REG_MINUTES 0x01
#define DS1307_REG_HOURS 0x02
#define DS1307_REG_DAY 0x03
#define DS1307_REG_DATE 0x04
#define DS1307_REG_MONTH 0x05
#define DS1307_REG_YEAR 0x06
#define DS1307_REG_CONTROL 0x07
#define DS1307_REG_RAM_START 0x08
#define DS1307_REG_RAM_END 0x3F

// Time/Date Structure
typedef struct
{
    uint8_t seconds; // 0-59
    uint8_t minutes; // 0-59
    uint8_t hours;   // 0-23 (24-hour format)
    uint8_t day;     // 1-7 (1=Sunday)
    uint8_t date;    // 1-31
    uint8_t month;   // 1-12
    uint8_t year;    // 0-99 (2000-2099)
} rtc_time_t;

// Alarm Structure
typedef struct
{
    uint8_t alarm_hours;
    uint8_t alarm_minutes;
    uint8_t alarm_enabled;
    uint8_t alarm_triggered;
    char alarm_name[16];
} rtc_alarm_t;

// Schedule Entry Structure
typedef struct
{
    uint8_t hour;
    uint8_t minute;
    uint8_t day_mask; // Bit mask for days (bit 0=Sun, bit 1=Mon, etc.)
    uint8_t active;
    char description[20];
} schedule_entry_t;

// Activity indicators
#define RTC_STATUS_LED_PIN 2 // PD2 - RTC communication status
#define ALARM_LED_PIN 3      // PD3 - Alarm indicator
#define SCHEDULE_LED_PIN 4   // PD4 - Schedule indicator

// User input buttons
#define HOUR_BUTTON_PIN 0 // PC0 - Hour adjustment
#define MIN_BUTTON_PIN 1  // PC1 - Minute adjustment
#define SET_BUTTON_PIN 2  // PC2 - Set/confirm button

// Lab session variables
uint16_t lab_score = 0;
uint32_t i2c_transactions = 0;
uint16_t time_reads = 0;
uint16_t time_sets = 0;
uint8_t rtc_communication_errors = 0;

// Current time/date
rtc_time_t current_time = {0, 0, 12, 1, 1, 1, 24}; // Default: 12:00:00, Sunday, Jan 1, 2024

// Alarm system
#define MAX_ALARMS 3
rtc_alarm_t alarms[MAX_ALARMS] = {
    {8, 0, 0, 0, "Morning Alarm"},
    {13, 0, 0, 0, "Lunch Reminder"},
    {18, 30, 0, 0, "Evening Alert"}};

// Schedule system
#define MAX_SCHEDULE_ENTRIES 5
schedule_entry_t schedule[MAX_SCHEDULE_ENTRIES] = {
    {9, 0, 0b01111110, 0, "Workday Start"},    // Mon-Sat
    {12, 0, 0b01111110, 0, "Lunch Break"},     // Mon-Sat
    {17, 0, 0b01111110, 0, "Work End"},        // Mon-Sat
    {19, 0, 0b01111111, 0, "Dinner Time"},     // Every day
    {22, 0, 0b01111111, 0, "Bedtime Reminder"} // Every day
};

// Day names for display
char day_names[][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
char month_names[][4] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                         "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

/*
 * =============================================================================
 * BCD (BINARY-CODED DECIMAL) CONVERSION FUNCTIONS
 * =============================================================================
 */

uint8_t bcd_to_decimal(uint8_t bcd_value)
{
    return ((bcd_value >> 4) * 10) + (bcd_value & 0x0F);
}

uint8_t decimal_to_bcd(uint8_t decimal_value)
{
    return ((decimal_value / 10) << 4) | (decimal_value % 10);
}

/*
 * =============================================================================
 * DS1307 I2C COMMUNICATION FUNCTIONS
 * =============================================================================
 */

void rtc_i2c_init(void)
{
    // Initialize I2C/TWI for DS1307 communication
    // Set SCL frequency to 100kHz for DS1307 compatibility
    TWSR = 0x00; // Prescaler = 1
    TWBR = 72;   // 100kHz at 16MHz: TWBR = (F_CPU/SCL_freq - 16) / 2

    // Configure status LEDs
    DDRD |= (1 << RTC_STATUS_LED_PIN) | (1 << ALARM_LED_PIN) | (1 << SCHEDULE_LED_PIN);
    PORTD &= ~((1 << RTC_STATUS_LED_PIN) | (1 << ALARM_LED_PIN) | (1 << SCHEDULE_LED_PIN));

    // Configure input buttons with pull-ups
    DDRC &= ~((1 << HOUR_BUTTON_PIN) | (1 << MIN_BUTTON_PIN) | (1 << SET_BUTTON_PIN));
    PORTC |= (1 << HOUR_BUTTON_PIN) | (1 << MIN_BUTTON_PIN) | (1 << SET_BUTTON_PIN);

    puts_USART1("I2C initialized for DS1307 RTC\\r\\n");
}

uint8_t rtc_i2c_start(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)))
        ;

    if ((TWSR & 0xF8) != 0x08)
    { // START condition not transmitted
        rtc_communication_errors++;
        return 0;
    }

    return 1;
}

uint8_t rtc_i2c_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    while (TWCR & (1 << TWSTO))
        ; // Wait for STOP condition to complete
    return 1;
}

uint8_t rtc_i2c_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)))
        ;

    if ((TWSR & 0xF8) != 0x28)
    { // Data not transmitted successfully
        rtc_communication_errors++;
        return 0;
    }

    return 1;
}

uint8_t rtc_i2c_read_ack(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)))
        ;

    if ((TWSR & 0xF8) != 0x50)
    { // Data not received with ACK
        rtc_communication_errors++;
    }

    return TWDR;
}

uint8_t rtc_i2c_read_nack(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)))
        ;

    if ((TWSR & 0xF8) != 0x58)
    { // Data not received with NACK
        rtc_communication_errors++;
    }

    return TWDR;
}

uint8_t rtc_write_register(uint8_t reg_addr, uint8_t data)
{
    PORTD |= (1 << RTC_STATUS_LED_PIN); // Activity indicator

    if (!rtc_i2c_start())
    {
        PORTD &= ~(1 << RTC_STATUS_LED_PIN);
        return 0;
    }

    if (!rtc_i2c_write(DS1307_WRITE_ADDR))
    {
        rtc_i2c_stop();
        PORTD &= ~(1 << RTC_STATUS_LED_PIN);
        return 0;
    }

    if (!rtc_i2c_write(reg_addr))
    {
        rtc_i2c_stop();
        PORTD &= ~(1 << RTC_STATUS_LED_PIN);
        return 0;
    }

    if (!rtc_i2c_write(data))
    {
        rtc_i2c_stop();
        PORTD &= ~(1 << RTC_STATUS_LED_PIN);
        return 0;
    }

    rtc_i2c_stop();
    PORTD &= ~(1 << RTC_STATUS_LED_PIN);
    i2c_transactions++;

    return 1;
}

uint8_t rtc_read_register(uint8_t reg_addr)
{
    uint8_t data = 0xFF; // Error value

    PORTD |= (1 << RTC_STATUS_LED_PIN); // Activity indicator

    // Write register address
    if (!rtc_i2c_start())
    {
        PORTD &= ~(1 << RTC_STATUS_LED_PIN);
        return data;
    }

    if (!rtc_i2c_write(DS1307_WRITE_ADDR))
    {
        rtc_i2c_stop();
        PORTD &= ~(1 << RTC_STATUS_LED_PIN);
        return data;
    }

    if (!rtc_i2c_write(reg_addr))
    {
        rtc_i2c_stop();
        PORTD &= ~(1 << RTC_STATUS_LED_PIN);
        return data;
    }

    // Repeated start for read
    if (!rtc_i2c_start())
    {
        rtc_i2c_stop();
        PORTD &= ~(1 << RTC_STATUS_LED_PIN);
        return data;
    }

    if (!rtc_i2c_write(DS1307_READ_ADDR))
    {
        rtc_i2c_stop();
        PORTD &= ~(1 << RTC_STATUS_LED_PIN);
        return data;
    }

    data = rtc_i2c_read_nack();
    rtc_i2c_stop();

    PORTD &= ~(1 << RTC_STATUS_LED_PIN);
    i2c_transactions++;

    return data;
}

/*
 * =============================================================================
 * DS1307 TIME/DATE FUNCTIONS
 * =============================================================================
 */

uint8_t rtc_read_time(rtc_time_t *time)
{
    if (time == NULL)
        return 0;

    // Read all time registers in one burst
    PORTD |= (1 << RTC_STATUS_LED_PIN);

    if (!rtc_i2c_start())
    {
        PORTD &= ~(1 << RTC_STATUS_LED_PIN);
        return 0;
    }

    if (!rtc_i2c_write(DS1307_WRITE_ADDR))
    {
        rtc_i2c_stop();
        PORTD &= ~(1 << RTC_STATUS_LED_PIN);
        return 0;
    }

    if (!rtc_i2c_write(DS1307_REG_SECONDS))
    {
        rtc_i2c_stop();
        PORTD &= ~(1 << RTC_STATUS_LED_PIN);
        return 0;
    }

    // Repeated start for read
    if (!rtc_i2c_start())
    {
        rtc_i2c_stop();
        PORTD &= ~(1 << RTC_STATUS_LED_PIN);
        return 0;
    }

    if (!rtc_i2c_write(DS1307_READ_ADDR))
    {
        rtc_i2c_stop();
        PORTD &= ~(1 << RTC_STATUS_LED_PIN);
        return 0;
    }

    // Read all time registers
    uint8_t seconds_bcd = rtc_i2c_read_ack();
    uint8_t minutes_bcd = rtc_i2c_read_ack();
    uint8_t hours_bcd = rtc_i2c_read_ack();
    uint8_t day_bcd = rtc_i2c_read_ack();
    uint8_t date_bcd = rtc_i2c_read_ack();
    uint8_t month_bcd = rtc_i2c_read_ack();
    uint8_t year_bcd = rtc_i2c_read_nack();

    rtc_i2c_stop();
    PORTD &= ~(1 << RTC_STATUS_LED_PIN);

    // Convert BCD to decimal
    time->seconds = bcd_to_decimal(seconds_bcd & 0x7F); // Mask CH bit
    time->minutes = bcd_to_decimal(minutes_bcd);
    time->hours = bcd_to_decimal(hours_bcd & 0x3F); // Mask 12/24 hour bits
    time->day = bcd_to_decimal(day_bcd);
    time->date = bcd_to_decimal(date_bcd);
    time->month = bcd_to_decimal(month_bcd);
    time->year = bcd_to_decimal(year_bcd);

    time_reads++;
    i2c_transactions++;

    return 1;
}

uint8_t rtc_set_time(rtc_time_t *time)
{
    if (time == NULL)
        return 0;

    // Convert decimal to BCD and write all registers
    uint8_t success = 1;

    success &= rtc_write_register(DS1307_REG_SECONDS, decimal_to_bcd(time->seconds));
    success &= rtc_write_register(DS1307_REG_MINUTES, decimal_to_bcd(time->minutes));
    success &= rtc_write_register(DS1307_REG_HOURS, decimal_to_bcd(time->hours)); // 24-hour format
    success &= rtc_write_register(DS1307_REG_DAY, decimal_to_bcd(time->day));
    success &= rtc_write_register(DS1307_REG_DATE, decimal_to_bcd(time->date));
    success &= rtc_write_register(DS1307_REG_MONTH, decimal_to_bcd(time->month));
    success &= rtc_write_register(DS1307_REG_YEAR, decimal_to_bcd(time->year));

    if (success)
    {
        time_sets++;
    }

    return success;
}

uint8_t rtc_enable_oscillator(void)
{
    // Clear the CH (Clock Halt) bit in the seconds register
    uint8_t seconds_reg = rtc_read_register(DS1307_REG_SECONDS);
    seconds_reg &= 0x7F; // Clear CH bit (bit 7)

    return rtc_write_register(DS1307_REG_SECONDS, seconds_reg);
}

uint8_t rtc_is_running(void)
{
    uint8_t seconds_reg = rtc_read_register(DS1307_REG_SECONDS);
    return !(seconds_reg & 0x80); // CH bit indicates halt status
}

/*
 * =============================================================================
 * DS1307 RAM FUNCTIONS (56 bytes of battery-backed RAM)
 * =============================================================================
 */

uint8_t rtc_write_ram(uint8_t ram_addr, uint8_t data)
{
    if (ram_addr > 55)
        return 0; // Valid RAM addresses: 0x08-0x3F (0-55)

    return rtc_write_register(DS1307_REG_RAM_START + ram_addr, data);
}

uint8_t rtc_read_ram(uint8_t ram_addr)
{
    if (ram_addr > 55)
        return 0xFF; // Error

    return rtc_read_register(DS1307_REG_RAM_START + ram_addr);
}

/*
 * =============================================================================
 * LAB EXERCISE 1: DS1307 INITIALIZATION AND BASIC TIME (25 minutes)
 * =============================================================================
 * OBJECTIVE: Initialize DS1307 and read/display time
 * DIFFICULTY: ★★★☆☆ (Medium)
 */

void lab_ex1_rtc_initialization(void)
{
    /*
     * CHALLENGE: Initialize DS1307 and verify communication
     * TASK: Set up I2C, test DS1307 communication, enable oscillator
     * LEARNING: I2C communication, DS1307 registers, oscillator control
     */

    puts_USART1("\\r\\n=== Lab 1: DS1307 Initialization ===\\r\\n");
    puts_USART1("Initializing DS1307 Real-Time Clock\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "DS1307 RTC INIT");
    lcd_string(1, 0, "I2C Communication");

    rtc_i2c_init();

    // Test basic communication
    puts_USART1("Testing DS1307 communication...\\r\\n");
    lcd_string(3, 0, "Testing comm...");

    uint8_t test_data = rtc_read_register(DS1307_REG_SECONDS);

    if (test_data == 0xFF)
    {
        puts_USART1("❌ DS1307 communication failed!\\r\\n");
        puts_USART1("Check I2C connections and pull-up resistors\\r\\n");
        lcd_string(4, 0, "COMM FAILED!");
        _delay_ms(3000);
        return;
    }

    puts_USART1("✓ DS1307 communication successful\\r\\n");
    lcd_string(4, 0, "COMM OK!");

    char comm_msg[50];
    sprintf(comm_msg, "Initial seconds register: 0x%02X\\r\\n", test_data);
    puts_USART1(comm_msg);

    // Check if oscillator is running
    if (rtc_is_running())
    {
        puts_USART1("✓ RTC oscillator is running\\r\\n");
        lcd_string(5, 0, "OSC RUNNING");
    }
    else
    {
        puts_USART1("⚠ RTC oscillator is halted, enabling...\\r\\n");
        lcd_string(5, 0, "ENABLING OSC...");

        if (rtc_enable_oscillator())
        {
            puts_USART1("✓ RTC oscillator enabled\\r\\n");
            lcd_string(5, 0, "OSC ENABLED");
            lab_score += 50;
        }
        else
        {
            puts_USART1("❌ Failed to enable oscillator\\r\\n");
            lcd_string(5, 0, "OSC ENABLE FAIL");
        }
    }

    _delay_ms(2000);

    // Display current communication statistics
    char stats[60];
    sprintf(stats, "I2C transactions: %ld, Errors: %d\\r\\n", i2c_transactions, rtc_communication_errors);
    puts_USART1(stats);

    if (rtc_communication_errors == 0)
    {
        lab_score += 100;
    }
}

void lab_ex1_time_reading(void)
{
    /*
     * CHALLENGE: Read and display current time from DS1307
     * TASK: Implement time reading and formatted display
     * LEARNING: BCD conversion, time display formats, continuous monitoring
     */

    puts_USART1("\\r\\n=== Lab 1.2: Time Reading ===\\r\\n");
    puts_USART1("Reading time from DS1307 (30 seconds)\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "TIME READING");
    lcd_string(1, 0, "DS1307 Monitor");

    uint8_t reading_count = 0;
    uint8_t successful_reads = 0;

    for (uint8_t cycle = 0; cycle < 30; cycle++)
    {
        reading_count++;

        if (rtc_read_time(&current_time))
        {
            successful_reads++;

            // Display time in serial terminal
            char time_str[80];
            sprintf(time_str, "%s %02d/%02d/20%02d %02d:%02d:%02d\\r\\n",
                    day_names[current_time.day - 1],
                    current_time.month, current_time.date, current_time.year,
                    current_time.hours, current_time.minutes, current_time.seconds);
            puts_USART1(time_str);

            // Display time on LCD
            char lcd_time[20];
            sprintf(lcd_time, "%02d:%02d:%02d",
                    current_time.hours, current_time.minutes, current_time.seconds);
            lcd_string(3, 0, lcd_time);

            char lcd_date[20];
            sprintf(lcd_date, "%s %02d/%02d/20%02d",
                    day_names[current_time.day - 1],
                    current_time.month, current_time.date, current_time.year);
            lcd_string(4, 0, lcd_date);

            // Show reading progress
            char progress[20];
            sprintf(progress, "Reading: %d/30", cycle + 1);
            lcd_string(5, 0, progress);
        }
        else
        {
            puts_USART1("❌ Failed to read time\\r\\n");
            lcd_string(3, 0, "READ ERROR");
        }

        _delay_ms(1000);
    }

    // Display reading statistics
    char read_stats[80];
    sprintf(read_stats, "\\r\\nReading statistics: %d/%d successful\\r\\n",
            successful_reads, reading_count);
    puts_USART1(read_stats);

    uint8_t success_rate = (successful_reads * 100) / reading_count;
    sprintf(read_stats, "Success rate: %d%%\\r\\n", success_rate);
    puts_USART1(read_stats);

    char lcd_stats[20];
    sprintf(lcd_stats, "Success: %d%%", success_rate);
    lcd_string(5, 0, lcd_stats);

    if (success_rate >= 90)
    {
        lab_score += 150;
        puts_USART1("✓ Excellent reading performance!\\r\\n");
    }

    _delay_ms(2000);
}

/*
 * =============================================================================
 * LAB EXERCISE 2: TIME/DATE SETTING AND BCD HANDLING (20 minutes)
 * =============================================================================
 * OBJECTIVE: Master time setting and BCD data format
 * DIFFICULTY: ★★★★☆ (Medium-Hard)
 */

void lab_ex2_bcd_conversion(void)
{
    /*
     * CHALLENGE: Master BCD (Binary-Coded Decimal) conversion
     * TASK: Test BCD conversion functions with various values
     * LEARNING: BCD format, data representation, conversion algorithms
     */

    puts_USART1("\\r\\n=== Lab 2: BCD Conversion ===\\r\\n");
    puts_USART1("Testing Binary-Coded Decimal conversion\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "BCD CONVERSION");
    lcd_string(1, 0, "Format testing");

    // Test decimal to BCD conversion
    puts_USART1("Decimal to BCD conversion test:\\r\\n");
    uint8_t test_decimals[] = {0, 1, 9, 10, 19, 23, 59, 99};
    uint8_t test_count = 8;
    uint8_t bcd_pass_count = 0;

    for (uint8_t i = 0; i < test_count; i++)
    {
        uint8_t decimal_val = test_decimals[i];
        uint8_t bcd_val = decimal_to_bcd(decimal_val);
        uint8_t converted_back = bcd_to_decimal(bcd_val);

        char test_result[60];
        sprintf(test_result, "  %2d → 0x%02X → %2d ",
                decimal_val, bcd_val, converted_back);
        puts_USART1(test_result);

        if (converted_back == decimal_val)
        {
            puts_USART1("✓\\r\\n");
            bcd_pass_count++;
        }
        else
        {
            puts_USART1("❌\\r\\n");
        }

        // Display on LCD
        char lcd_test[20];
        sprintf(lcd_test, "%d->%02X->%d %s",
                decimal_val, bcd_val, converted_back,
                (converted_back == decimal_val) ? "OK" : "ER");
        lcd_string(3, 0, lcd_test);

        _delay_ms(800);
    }

    char bcd_result[50];
    sprintf(bcd_result, "BCD conversion: %d/%d tests passed\\r\\n", bcd_pass_count, test_count);
    puts_USART1(bcd_result);

    char lcd_bcd_result[20];
    sprintf(lcd_bcd_result, "BCD: %d/%d passed", bcd_pass_count, test_count);
    lcd_string(4, 0, lcd_bcd_result);

    if (bcd_pass_count == test_count)
    {
        lab_score += 100;
        puts_USART1("✓ Perfect BCD conversion!\\r\\n");
        lcd_string(5, 0, "BCD PERFECT!");
    }

    _delay_ms(2000);
}

void lab_ex2_time_setting(void)
{
    /*
     * CHALLENGE: Set time/date in DS1307 and verify accuracy
     * TASK: Set various times and verify they are stored correctly
     * LEARNING: Time setting, verification, data persistence
     */

    puts_USART1("\\r\\n=== Lab 2.2: Time Setting ===\\r\\n");
    puts_USART1("Setting and verifying time/date in DS1307\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "TIME SETTING");
    lcd_string(1, 0, "DS1307 update");

    // Test time settings
    rtc_time_t test_times[] = {
        {30, 45, 14, 2, 15, 3, 24}, // 14:45:30, Monday, March 15, 2024
        {0, 0, 0, 1, 1, 1, 25},     // 00:00:00, Sunday, January 1, 2025
        {59, 59, 23, 7, 31, 12, 23} // 23:59:59, Saturday, December 31, 2023
    };

    uint8_t time_test_count = 3;
    uint8_t successful_sets = 0;

    for (uint8_t test = 0; test < time_test_count; test++)
    {
        rtc_time_t *test_time = &test_times[test];

        char set_msg[80];
        sprintf(set_msg, "Setting test time %d: %s %02d/%02d/20%02d %02d:%02d:%02d\\r\\n",
                test + 1,
                day_names[test_time->day - 1],
                test_time->month, test_time->date, test_time->year,
                test_time->hours, test_time->minutes, test_time->seconds);
        puts_USART1(set_msg);

        char lcd_setting[20];
        sprintf(lcd_setting, "Set test %d/3", test + 1);
        lcd_string(3, 0, lcd_setting);

        // Set the time
        if (rtc_set_time(test_time))
        {
            puts_USART1("  Time set successfully\\r\\n");

            // Wait a moment for DS1307 to process
            _delay_ms(500);

            // Read back and verify
            rtc_time_t read_time;
            if (rtc_read_time(&read_time))
            {
                puts_USART1("  Verification read successful\\r\\n");

                // Compare times (allow 1 second difference due to delays)
                uint8_t time_match = 1;

                if (read_time.minutes != test_time->minutes ||
                    read_time.hours != test_time->hours ||
                    read_time.day != test_time->day ||
                    read_time.date != test_time->date ||
                    read_time.month != test_time->month ||
                    read_time.year != test_time->year)
                {
                    time_match = 0;
                }

                // Check seconds (allow for 1-2 second difference)
                uint8_t second_diff = 0;
                if (read_time.seconds >= test_time->seconds)
                {
                    second_diff = read_time.seconds - test_time->seconds;
                }
                else
                {
                    second_diff = (60 + read_time.seconds) - test_time->seconds;
                }

                if (second_diff > 2)
                {
                    time_match = 0;
                }

                char verify_msg[80];
                sprintf(verify_msg, "  Read back: %s %02d/%02d/20%02d %02d:%02d:%02d\\r\\n",
                        day_names[read_time.day - 1],
                        read_time.month, read_time.date, read_time.year,
                        read_time.hours, read_time.minutes, read_time.seconds);
                puts_USART1(verify_msg);

                char lcd_verify[20];
                sprintf(lcd_verify, "%02d:%02d:%02d %s",
                        read_time.hours, read_time.minutes, read_time.seconds,
                        time_match ? "OK" : "ER");
                lcd_string(4, 0, lcd_verify);

                if (time_match)
                {
                    puts_USART1("  ✓ Time verification successful\\r\\n");
                    successful_sets++;
                }
                else
                {
                    puts_USART1("  ❌ Time verification failed\\r\\n");
                }
            }
            else
            {
                puts_USART1("  ❌ Failed to read back time\\r\\n");
                lcd_string(4, 0, "READ FAIL");
            }
        }
        else
        {
            puts_USART1("  ❌ Failed to set time\\r\\n");
            lcd_string(4, 0, "SET FAIL");
        }

        puts_USART1("\\r\\n");
        _delay_ms(2000);
    }

    char final_result[60];
    sprintf(final_result, "Time setting results: %d/%d successful\\r\\n",
            successful_sets, time_test_count);
    puts_USART1(final_result);

    char lcd_final[20];
    sprintf(lcd_final, "Result: %d/%d", successful_sets, time_test_count);
    lcd_string(5, 0, lcd_final);

    if (successful_sets >= 2)
    {
        lab_score += 150;
    }

    _delay_ms(2000);
}

/*
 * =============================================================================
 * LAB EXERCISE 3: ALARM SYSTEM AND SCHEDULING (15 minutes)
 * =============================================================================
 * OBJECTIVE: Create alarm and scheduling functionality
 * DIFFICULTY: ★★★★☆ (Medium-Hard)
 */

void lab_ex3_alarm_system(void)
{
    /*
     * CHALLENGE: Implement alarm system using DS1307 time
     * TASK: Set alarms and monitor for alarm conditions
     * LEARNING: Time comparison, alarm logic, event handling
     */

    puts_USART1("\\r\\n=== Lab 3: Alarm System ===\\r\\n");
    puts_USART1("Implementing RTC-based alarm system\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "ALARM SYSTEM");
    lcd_string(1, 0, "RTC monitoring");

    // Set up test alarms
    rtc_time_t current_rtc_time;
    if (!rtc_read_time(&current_rtc_time))
    {
        puts_USART1("❌ Failed to read current time for alarm setup\\r\\n");
        return;
    }

    // Set alarms relative to current time
    alarms[0].alarm_hours = current_rtc_time.hours;
    alarms[0].alarm_minutes = (current_rtc_time.minutes + 2) % 60;
    alarms[0].alarm_enabled = 1;
    strcpy(alarms[0].alarm_name, "Test Alarm 1");

    alarms[1].alarm_hours = current_rtc_time.hours;
    alarms[1].alarm_minutes = (current_rtc_time.minutes + 4) % 60;
    alarms[1].alarm_enabled = 1;
    strcpy(alarms[1].alarm_name, "Test Alarm 2");

    alarms[2].alarm_hours = (current_rtc_time.hours + 1) % 24;
    alarms[2].alarm_minutes = current_rtc_time.minutes;
    alarms[2].alarm_enabled = 1;
    strcpy(alarms[2].alarm_name, "Hour Alarm");

    // Display alarm settings
    puts_USART1("Alarm Configuration:\\r\\n");
    for (uint8_t i = 0; i < MAX_ALARMS; i++)
    {
        if (alarms[i].alarm_enabled)
        {
            char alarm_info[60];
            sprintf(alarm_info, "  Alarm %d: %02d:%02d - %s\\r\\n",
                    i + 1, alarms[i].alarm_hours, alarms[i].alarm_minutes,
                    alarms[i].alarm_name);
            puts_USART1(alarm_info);
        }
    }
    puts_USART1("\\r\\n");

    // Monitor for alarms (5 minutes)
    uint16_t monitoring_seconds = 300; // 5 minutes
    uint8_t alarms_triggered = 0;

    for (uint16_t second = 0; second < monitoring_seconds; second++)
    {
        if (rtc_read_time(&current_rtc_time))
        {
            // Display current time
            char time_display[40];
            sprintf(time_display, "%02d:%02d:%02d",
                    current_rtc_time.hours, current_rtc_time.minutes, current_rtc_time.seconds);
            lcd_string(3, 0, time_display);

            // Check all alarms
            for (uint8_t alarm_idx = 0; alarm_idx < MAX_ALARMS; alarm_idx++)
            {
                rtc_alarm_t *alarm = &alarms[alarm_idx];

                if (alarm->alarm_enabled && !alarm->alarm_triggered)
                {
                    if (current_rtc_time.hours == alarm->alarm_hours &&
                        current_rtc_time.minutes == alarm->alarm_minutes)
                    {

                        // Alarm triggered!
                        alarm->alarm_triggered = 1;
                        alarms_triggered++;

                        char alarm_msg[80];
                        sprintf(alarm_msg, "🔔 ALARM TRIGGERED: %s at %02d:%02d\\r\\n",
                                alarm->alarm_name, alarm->alarm_hours, alarm->alarm_minutes);
                        puts_USART1(alarm_msg);

                        char lcd_alarm[20];
                        sprintf(lcd_alarm, "ALARM: %s", alarm->alarm_name);
                        lcd_string(4, 0, lcd_alarm);

                        // Visual/audio alarm indication
                        for (uint8_t flash = 0; flash < 10; flash++)
                        {
                            PORTD |= (1 << ALARM_LED_PIN);
                            _delay_ms(100);
                            PORTD &= ~(1 << ALARM_LED_PIN);
                            _delay_ms(100);
                        }

                        lab_score += 75;
                    }
                }
            }

            // Display monitoring progress
            if (second % 30 == 0)
            { // Every 30 seconds
                char progress[50];
                sprintf(progress, "Monitoring: %d/%d sec, Alarms: %d\\r\\n",
                        second, monitoring_seconds, alarms_triggered);
                puts_USART1(progress);

                char lcd_progress[20];
                sprintf(lcd_progress, "Mon: %d/%d A:%d",
                        second / 30, monitoring_seconds / 30, alarms_triggered);
                lcd_string(5, 0, lcd_progress);
            }
        }
        else
        {
            puts_USART1("⚠ Time read failed during alarm monitoring\\r\\n");
        }

        _delay_ms(1000);
    }

    char alarm_summary[60];
    sprintf(alarm_summary, "\\r\\nAlarm monitoring complete: %d alarms triggered\\r\\n",
            alarms_triggered);
    puts_USART1(alarm_summary);

    char lcd_summary[20];
    sprintf(lcd_summary, "Alarms: %d triggered", alarms_triggered);
    lcd_string(5, 0, lcd_summary);

    if (alarms_triggered >= 1)
    {
        lab_score += 100;
    }
}

/*
 * =============================================================================
 * LAB EXERCISE 4: ADVANCED TIME-BASED APPLICATIONS (15 minutes)
 * =============================================================================
 * OBJECTIVE: Build sophisticated time-based automation
 * DIFFICULTY: ★★★★★ (Hard)
 */

void lab_ex4_data_logging(void)
{
    /*
     * CHALLENGE: Implement timestamped data logging using DS1307 RAM
     * TASK: Log sensor data with accurate timestamps
     * LEARNING: Data logging, timestamp correlation, RAM utilization
     */

    puts_USART1("\\r\\n=== Lab 4: Advanced Applications ===\\r\\n");
    puts_USART1("Implementing timestamped data logging\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "DATA LOGGING");
    lcd_string(1, 0, "Timestamped logs");

    // Initialize logging parameters
    uint8_t log_interval_seconds = 10; // Log every 10 seconds
    uint8_t max_log_entries = 20;      // Maximum entries to log
    uint8_t log_count = 0;
    uint8_t last_logged_second = 255; // Invalid initial value

    puts_USART1("Data logging parameters:\\r\\n");
    char log_params[60];
    sprintf(log_params, "  Interval: %d seconds\\r\\n", log_interval_seconds);
    puts_USART1(log_params);
    sprintf(log_params, "  Max entries: %d\\r\\n", max_log_entries);
    puts_USART1(log_params);
    sprintf(log_params, "  RAM usage: %d bytes per entry\\r\\n", 8);
    puts_USART1(log_params);
    puts_USART1("\\r\\n");

    // Clear DS1307 RAM for logging
    for (uint8_t ram_addr = 0; ram_addr < 56; ram_addr++)
    {
        rtc_write_ram(ram_addr, 0x00);
    }
    puts_USART1("DS1307 RAM cleared for logging\\r\\n");

    // Data logging loop
    for (uint16_t cycle = 0; cycle < 180; cycle++)
    { // 3 minutes of monitoring
        rtc_time_t log_time;

        if (rtc_read_time(&log_time))
        {
            // Check if it's time to log (every log_interval_seconds)
            if ((log_time.seconds % log_interval_seconds == 0) &&
                (log_time.seconds != last_logged_second) &&
                (log_count < max_log_entries))
            {

                last_logged_second = log_time.seconds;

                // Simulate sensor readings
                uint16_t temp_reading = 250 + (cycle % 100);     // Simulated temperature
                uint16_t pressure_reading = 1000 + (cycle % 50); // Simulated pressure

                // Create log entry in DS1307 RAM
                uint8_t ram_base = log_count * 8; // 8 bytes per entry

                // Log format: Hours, Minutes, Seconds, Day, TempH, TempL, PressH, PressL
                rtc_write_ram(ram_base + 0, log_time.hours);
                rtc_write_ram(ram_base + 1, log_time.minutes);
                rtc_write_ram(ram_base + 2, log_time.seconds);
                rtc_write_ram(ram_base + 3, log_time.day);
                rtc_write_ram(ram_base + 4, (temp_reading >> 8) & 0xFF);
                rtc_write_ram(ram_base + 5, temp_reading & 0xFF);
                rtc_write_ram(ram_base + 6, (pressure_reading >> 8) & 0xFF);
                rtc_write_ram(ram_base + 7, pressure_reading & 0xFF);

                log_count++;

                char log_entry[100];
                sprintf(log_entry, "LOG #%d: %02d:%02d:%02d T=%d P=%d [RAM:%d]\\r\\n",
                        log_count, log_time.hours, log_time.minutes, log_time.seconds,
                        temp_reading, pressure_reading, ram_base);
                puts_USART1(log_entry);

                char lcd_log[20];
                sprintf(lcd_log, "Log #%d T%d P%d", log_count, temp_reading, pressure_reading);
                lcd_string(3, 0, lcd_log);

                // Visual indication
                PORTD |= (1 << SCHEDULE_LED_PIN);
                _delay_ms(200);
                PORTD &= ~(1 << SCHEDULE_LED_PIN);

                lab_score += 20;
            }

            // Display current time and progress
            char current_display[20];
            sprintf(current_display, "%02d:%02d:%02d",
                    log_time.hours, log_time.minutes, log_time.seconds);
            lcd_string(4, 0, current_display);

            char progress_display[20];
            sprintf(progress_display, "Logs: %d/%d", log_count, max_log_entries);
            lcd_string(5, 0, progress_display);
        }
        else
        {
            puts_USART1("⚠ Failed to read time during logging\\r\\n");
        }

        _delay_ms(1000);

        // Stop if we've reached max entries
        if (log_count >= max_log_entries)
        {
            puts_USART1("Maximum log entries reached\\r\\n");
            break;
        }
    }

    // Verify logged data by reading back from RAM
    puts_USART1("\\r\\n=== VERIFYING LOGGED DATA ===\\r\\n");

    uint8_t verified_entries = 0;

    for (uint8_t entry = 0; entry < log_count; entry++)
    {
        uint8_t ram_base = entry * 8;

        uint8_t hours = rtc_read_ram(ram_base + 0);
        uint8_t minutes = rtc_read_ram(ram_base + 1);
        uint8_t seconds = rtc_read_ram(ram_base + 2);
        uint8_t day = rtc_read_ram(ram_base + 3);
        uint16_t temp = (rtc_read_ram(ram_base + 4) << 8) | rtc_read_ram(ram_base + 5);
        uint16_t pressure = (rtc_read_ram(ram_base + 6) << 8) | rtc_read_ram(ram_base + 7);

        // Verify data integrity (basic sanity checks)
        if (hours <= 23 && minutes <= 59 && seconds <= 59 &&
            day >= 1 && day <= 7 && temp > 0 && pressure > 0)
        {
            verified_entries++;

            char verify_msg[80];
            sprintf(verify_msg, "Entry %d: %02d:%02d:%02d %s T=%d P=%d ✓\\r\\n",
                    entry + 1, hours, minutes, seconds, day_names[day - 1], temp, pressure);
            puts_USART1(verify_msg);
        }
        else
        {
            char error_msg[50];
            sprintf(error_msg, "Entry %d: Data corruption detected ❌\\r\\n", entry + 1);
            puts_USART1(error_msg);
        }
    }

    char verification_result[60];
    sprintf(verification_result, "\\r\\nData verification: %d/%d entries valid\\r\\n",
            verified_entries, log_count);
    puts_USART1(verification_result);

    if (verified_entries == log_count && log_count >= 10)
    {
        lab_score += 200;
        puts_USART1("✓ Excellent data logging performance!\\r\\n");
    }

    char lcd_verify[20];
    sprintf(lcd_verify, "Verified: %d/%d", verified_entries, log_count);
    lcd_string(5, 0, lcd_verify);

    _delay_ms(3000);
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
    puts_USART1("      I2C RTC DS1307 - LAB EXERCISES        \\r\\n");
    puts_USART1("==============================================\\r\\n");
    puts_USART1("1. DS1307 Initialization & Time Reading     \\r\\n");
    puts_USART1("2. Time Setting & BCD Format Handling       \\r\\n");
    puts_USART1("3. Alarm System & Scheduling                \\r\\n");
    puts_USART1("4. Advanced Time-Based Applications         \\r\\n");
    puts_USART1("                                              \\r\\n");
    puts_USART1("0. Run All Exercises                         \\r\\n");
    puts_USART1("X. Exit Lab                                   \\r\\n");
    puts_USART1("==============================================\\r\\n");
    char score_msg[50];
    sprintf(score_msg, "Current Score: %d points\\r\\n", lab_score);
    puts_USART1(score_msg);
    char rtc_stats[60];
    sprintf(rtc_stats, "I2C Transactions: %ld, Errors: %d\\r\\n", i2c_transactions, rtc_communication_errors);
    puts_USART1(rtc_stats);
    char time_stats[60];
    sprintf(time_stats, "Time Reads: %d, Time Sets: %d\\r\\n", time_reads, time_sets);
    puts_USART1(time_stats);
    puts_USART1("Select exercise (1-4, 0, X): ");
}

int main(void)
{
    init_devices();

    puts_USART1("\\r\\n*** I2C RTC DS1307 LAB SESSION ***\\r\\n");
    puts_USART1("Welcome to Real-Time Clock integration with DS1307!\\r\\n");
    puts_USART1("This lab covers I2C communication, time management, and RTC applications\\r\\n");
    puts_USART1("Ensure DS1307 is connected with proper I2C pull-ups and battery backup\\r\\n");

    lcd_clear();
    lcd_string(1, 0, "I2C RTC DS1307");
    lcd_string(2, 0, "Real-time clock");
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
            lab_ex1_rtc_initialization();
            lab_ex1_time_reading();
            break;

        case '2':
            lab_ex2_bcd_conversion();
            lab_ex2_time_setting();
            break;

        case '3':
            lab_ex3_alarm_system();
            break;

        case '4':
            lab_ex4_data_logging();
            break;

        case '0':
            puts_USART1("\\r\\n*** RUNNING ALL EXERCISES ***\\r\\n");
            lab_ex1_rtc_initialization();
            lab_ex1_time_reading();
            lab_ex2_bcd_conversion();
            lab_ex2_time_setting();
            lab_ex3_alarm_system();
            lab_ex4_data_logging();

            char final_buffer[80];
            sprintf(final_buffer, "\\r\\n*** ALL EXERCISES COMPLETE! ***\\r\\nFinal Score: %d points\\r\\n", lab_score);
            puts_USART1(final_buffer);
            break;

        case 'X':
        case 'x':
            puts_USART1("\\r\\nExiting lab. Great work on RTC integration!\\r\\n");
            puts_USART1("Remember: Accurate timekeeping is essential for many embedded applications!\\r\\n");
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