/*
 * =============================================================================
 * TIMER SOFTWARE RTC - REAL-TIME CLOCK IMPLEMENTATION
 * =============================================================================
 * PROJECT: Timer_Software_RTC
 *
 * LEARNING OBJECTIVES:
 * - Build complete RTC using Timer1
 * - Manage time/date structures
 * - Handle calendar calculations (leap years, month lengths)
 * - Format time display (24-hour and 12-hour)
 *
 * HARDWARE:
 * - LED on PB0 (seconds indicator - blinks every second)
 * - LEDs on PB1-PB7 (time display)
 * - ATmega128 @ 16 MHz
 *
 * THEORY:
 * Uses Timer1 CTC mode for 1 Hz (1 second) interrupts
 * OCR1A = (16MHz / 1024 / 1) - 1 = 15624
 * ISR increments seconds, handles rollover to minutes/hours/days
 *
 * DEMOS:
 * 1. Basic Clock - HH:MM:SS display
 * 2. Clock with Date - Full calendar
 * 3. 12-Hour Format - AM/PM display
 * 4. Alarm Clock - Set alarm time
 * =============================================================================
 */

#include "config.h"

// Time structure
typedef struct
{
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t day;
    uint8_t month;
    uint16_t year;
} RTC_Time;

// Global time
volatile RTC_Time current_time = {0, 0, 12, 1, 1, 2025}; // Start at noon, Jan 1, 2025

// Alarm settings
uint8_t alarm_hours = 12;
uint8_t alarm_minutes = 0;
uint8_t alarm_enabled = 0;
volatile uint8_t alarm_triggered = 0;

// Function prototypes
void demo1_basic_clock(void);
void demo2_clock_with_date(void);
void demo3_12hour_format(void);
void demo4_alarm_clock(void);
void setup_timer_1hz(void);
void update_time(void);
uint8_t days_in_month(uint8_t month, uint16_t year);
uint8_t is_leap_year(uint16_t year);

// ===== INTERRUPT SERVICE ROUTINE =====
// Fires every 1 second
ISR(TIMER1_COMPA_vect)
{
    update_time();
}

// ===== MAIN - SELECT YOUR DEMO =====
int main(void)
{
    // Initialize hardware
    DDRB = 0xFF;  // All Port B as OUTPUT
    PORTB = 0xFF; // All LEDs OFF

    // Uncomment ONE demo to run:
    demo1_basic_clock(); // Simple HH:MM:SS clock
    // demo2_clock_with_date();    // Full calendar clock
    // demo3_12hour_format();      // 12-hour AM/PM format
    // demo4_alarm_clock();        // Clock with alarm

    while (1)
    {
    }
    return 0;
}

// =============================================================================
// TIMER SETUP
// =============================================================================
void setup_timer_1hz(void)
{
    // Timer1 CTC Mode for 1 Hz (1 second period)
    TCCR1A = 0x00;
    TCCR1B = 0x00;
    TCNT1 = 0;

    // CTC Mode (WGM12=1), Prescaler 1024 (CS12=1, CS10=1)
    TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10);

    // 1 Hz: OCR1A = (16000000 / 1024 / 1) - 1 = 15624
    OCR1A = 15624;

    // Enable compare match interrupt
    TIMSK |= (1 << OCIE1A);
    sei();
}

// =============================================================================
// TIME MANAGEMENT FUNCTIONS
// =============================================================================

// Update time every second
void update_time(void)
{
    current_time.seconds++;

    if (current_time.seconds >= 60)
    {
        current_time.seconds = 0;
        current_time.minutes++;

        // Check alarm
        if (alarm_enabled &&
            current_time.hours == alarm_hours &&
            current_time.minutes == alarm_minutes)
        {
            alarm_triggered = 1;
        }

        if (current_time.minutes >= 60)
        {
            current_time.minutes = 0;
            current_time.hours++;

            if (current_time.hours >= 24)
            {
                current_time.hours = 0;
                current_time.day++;

                if (current_time.day > days_in_month(current_time.month, current_time.year))
                {
                    current_time.day = 1;
                    current_time.month++;

                    if (current_time.month > 12)
                    {
                        current_time.month = 1;
                        current_time.year++;
                    }
                }
            }
        }
    }
}

// Check if year is leap year
uint8_t is_leap_year(uint16_t year)
{
    if (year % 400 == 0)
        return 1;
    if (year % 100 == 0)
        return 0;
    if (year % 4 == 0)
        return 1;
    return 0;
}

// Get days in month
uint8_t days_in_month(uint8_t month, uint16_t year)
{
    const uint8_t days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    if (month == 2 && is_leap_year(year))
        return 29;

    return days[month - 1];
}

// =============================================================================
// DEMO 1: BASIC CLOCK (HH:MM:SS)
// =============================================================================
void demo1_basic_clock(void)
{
    setup_timer_1hz();

    // Set start time: 12:00:00
    current_time.hours = 12;
    current_time.minutes = 0;
    current_time.seconds = 0;

    while (1)
    {
        // Display hours on PB7-PB3 (0-23 = 5 bits)
        PORTB = ~((current_time.hours & 0x1F) << 3);

        // Blink PB0 every second (seconds indicator)
        if (current_time.seconds % 2 == 0)
            PORTB &= ~(1 << PB0);
        else
            PORTB |= (1 << PB0);

        // Show seconds on PB2-PB1 (divided by 15 to fit in 2 bits)
        uint8_t sec_display = (current_time.seconds / 15) & 0x03;
        if (sec_display & 0x01)
            PORTB &= ~(1 << PB1);
        if (sec_display & 0x02)
            PORTB &= ~(1 << PB2);

        _delay_ms(100);
    }
}

// =============================================================================
// DEMO 2: CLOCK WITH DATE (YYYY-MM-DD HH:MM:SS)
// =============================================================================
void demo2_clock_with_date(void)
{
    setup_timer_1hz();

    // Set start: 2025-01-01 00:00:00
    current_time.year = 2025;
    current_time.month = 1;
    current_time.day = 1;
    current_time.hours = 0;
    current_time.minutes = 0;
    current_time.seconds = 0;

    uint8_t display_mode = 0; // 0=day, 1=month, 2=hours, 3=minutes

    while (1)
    {
        // Cycle through display modes every 2 seconds
        display_mode = (current_time.seconds / 2) % 4;

        switch (display_mode)
        {
        case 0: // Display day (1-31)
            PORTB = ~(current_time.day & 0x1F);
            break;
        case 1: // Display month (1-12)
            PORTB = ~(current_time.month & 0x0F);
            break;
        case 2: // Display hours (0-23)
            PORTB = ~(current_time.hours & 0x1F);
            break;
        case 3: // Display minutes (0-59)
            PORTB = ~(current_time.minutes & 0x3F);
            break;
        }

        // Blink PB7 to show mode switching
        if ((current_time.seconds % 2) == 0)
            PORTB &= ~(1 << PB7);

        _delay_ms(100);
    }
}

// =============================================================================
// DEMO 3: 12-HOUR FORMAT (AM/PM)
// =============================================================================
void demo3_12hour_format(void)
{
    setup_timer_1hz();

    // Set start time: 11:59:50 AM (will roll to PM soon)
    current_time.hours = 11;
    current_time.minutes = 59;
    current_time.seconds = 50;

    while (1)
    {
        // Convert 24-hour to 12-hour format
        uint8_t display_hour = current_time.hours;
        uint8_t is_pm = 0;

        if (display_hour == 0)
            display_hour = 12; // Midnight
        else if (display_hour > 12)
        {
            display_hour -= 12;
            is_pm = 1;
        }
        else if (display_hour == 12)
        {
            is_pm = 1;
        }

        // Display hour on PB4-PB1 (1-12 = 4 bits)
        PORTB = ~((display_hour & 0x0F) << 1);

        // PB0: AM/PM indicator (OFF=AM, ON=PM)
        if (is_pm)
            PORTB &= ~(1 << PB0);
        else
            PORTB |= (1 << PB0);

        // PB7: Blink every second
        if (current_time.seconds % 2 == 0)
            PORTB &= ~(1 << PB7);

        _delay_ms(100);
    }
}

// =============================================================================
// DEMO 4: ALARM CLOCK
// =============================================================================
void demo4_alarm_clock(void)
{
    setup_timer_1hz();

    // Set current time: 11:59:55
    current_time.hours = 11;
    current_time.minutes = 59;
    current_time.seconds = 55;

    // Set alarm for 12:00
    alarm_hours = 12;
    alarm_minutes = 0;
    alarm_enabled = 1;
    alarm_triggered = 0;

    while (1)
    {
        // Check for alarm
        if (alarm_triggered)
        {
            // ALARM! Blink all LEDs rapidly for 10 seconds
            for (uint8_t i = 0; i < 50; i++)
            {
                PORTB = 0x00; // All ON
                _delay_ms(100);
                PORTB = 0xFF; // All OFF
                _delay_ms(100);
            }

            alarm_triggered = 0;
            alarm_enabled = 0; // Disable until reset
        }

        // Normal display: show hours
        PORTB = ~((current_time.hours & 0x1F) << 2);

        // PB0: Alarm enabled indicator
        if (alarm_enabled)
            PORTB &= ~(1 << PB0);

        // PB1: Seconds blink
        if (current_time.seconds % 2 == 0)
            PORTB &= ~(1 << PB1);

        _delay_ms(100);
    }
}

// =============================================================================
// LEARNING NOTES
// =============================================================================
/*
 * CALENDAR CALCULATIONS:
 *
 * Days in month:
 * Jan, Mar, May, Jul, Aug, Oct, Dec: 31 days
 * Apr, Jun, Sep, Nov: 30 days
 * Feb: 28 days (29 in leap year)
 *
 * Leap Year Rules:
 * - Divisible by 4: Leap year
 * - EXCEPT divisible by 100: NOT leap year
 * - EXCEPT divisible by 400: IS leap year
 * Examples: 2024=leap, 1900=not, 2000=leap
 *
 * TIME FORMATS:
 *
 * 24-Hour (Military):
 * 00:00 = Midnight
 * 12:00 = Noon
 * 23:59 = 11:59 PM
 *
 * 12-Hour (AM/PM):
 * 12:00 AM = Midnight
 * 12:00 PM = Noon
 * Conversion: If hour > 12, subtract 12 and set PM
 *
 * TIMER ACCURACY:
 * RTC accuracy depends on crystal oscillator
 * Typical: ±20 ppm (parts per million)
 * Drift: ~1.7 seconds per day
 * For better accuracy: Use external RTC chip (DS1307, DS3231)
 *
 * REAL-WORLD RTC APPLICATIONS:
 * - Digital clocks
 * - Data loggers (timestamp events)
 * - Scheduler systems
 * - Time-based access control
 * - Automatic lighting systems
 *
 * IMPROVEMENTS:
 * - Add battery backup
 * - Use external 32.768 kHz crystal
 * - Implement timezone support
 * - Add DST (Daylight Saving Time)
 * - Store time in EEPROM
 */
