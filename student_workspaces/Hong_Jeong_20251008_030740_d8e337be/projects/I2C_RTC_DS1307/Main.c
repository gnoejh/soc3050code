/*
 * =============================================================================
 * I2C REAL-TIME CLOCK INTERFACE - EDUCATIONAL DEMONSTRATION
 * =============================================================================
 *
 * PROJECT: I2C_RTC_DS1307
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Educational demonstration of I2C real-time clock (DS1307) interfacing and timekeeping.
 * Students learn I2C communication protocols and real-time system concepts.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master I2C communication protocol with DS1307 RTC
 * 2. Learn BCD (Binary-Coded Decimal) encoding and decoding
 * 3. Practice time/date read/write operations
 * 4. Implement real-time clock applications
 * 5. Understand battery-backed memory systems
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - DS1307 Real-Time Clock IC with 32.768kHz crystal
 * - I2C connections: PD0 (SCL), PD1 (SDA) with 4.7K pull-ups
 * - 3V battery backup for timekeeping
 * - LCD display for time/date visualization
 * - Serial connection for debugging (9600 baud)
 *
 * DS1307 SPECIFICATIONS:
 * - I2C Address: 0x68 (7-bit)
 * - BCD Format: Hours, minutes, seconds, date, month, year
 * - 56 bytes of battery-backed RAM (0x08-0x3F)
 * - Square wave output options (1Hz, 4kHz, 8kHz, 32kHz)
 *
 * LEARNING PROGRESSION:
 * - Demo 1: I2C Communication Setup
 * - Demo 2: Time Reading and Display
 * - Demo 3: Time Setting and Configuration
 * - Demo 4: Battery-Backed Memory Usage
 *
 * =============================================================================
 */

#include "config.h"

// DS1307 I2C address
#define DS1307_ADDR 0x68

// DS1307 Register addresses
#define DS1307_REG_SECONDS 0x00
#define DS1307_REG_MINUTES 0x01
#define DS1307_REG_HOURS 0x02
#define DS1307_REG_DAY 0x03 // Day of week (1-7)
#define DS1307_REG_DATE 0x04
#define DS1307_REG_MONTH 0x05
#define DS1307_REG_YEAR 0x06
#define DS1307_REG_CONTROL 0x07
#define DS1307_REG_RAM 0x08 // Start of user RAM (56 bytes)

// Time structure
typedef struct
{
    uint8_t seconds;     // 0-59
    uint8_t minutes;     // 0-59
    uint8_t hours;       // 0-23 (24-hour mode)
    uint8_t day_of_week; // 1-7 (Sunday=1)
    uint8_t date;        // 1-31
    uint8_t month;       // 1-12
    uint8_t year;        // 00-99 (2000-2099)
} rtc_time_t;

// I2C functions (from previous project)
void i2c_init(void)
{
    TWBR = 32; // 100kHz @ 7.3728MHz
    TWSR = 0x00;
    TWCR = (1 << TWEN);
}

uint8_t i2c_start(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)))
        ;
    uint8_t status = TWSR & 0xF8;
    if (status != 0x08 && status != 0x10)
        return 1;
    return 0;
}

void i2c_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    _delay_us(100);
}

uint8_t i2c_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)))
        ;
    uint8_t status = TWSR & 0xF8;
    if (status != 0x18 && status != 0x28)
        return 1;
    return 0;
}

uint8_t i2c_read(uint8_t *data, uint8_t send_ack)
{
    if (send_ack)
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    else
        TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)))
        ;
    *data = TWDR;
    return 0;
}

/*
 * BCD conversion helpers
 */
uint8_t bcd_to_dec(uint8_t bcd)
{
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

uint8_t dec_to_bcd(uint8_t dec)
{
    return ((dec / 10) << 4) | (dec % 10);
}

/*
 * Write byte to DS1307 register
 */
uint8_t ds1307_write_register(uint8_t reg_addr, uint8_t data)
{
    if (i2c_start())
    {
        i2c_stop();
        return 1;
    }
    if (i2c_write((DS1307_ADDR << 1) | 0x00))
    {
        i2c_stop();
        return 1;
    }
    if (i2c_write(reg_addr))
    {
        i2c_stop();
        return 1;
    }
    if (i2c_write(data))
    {
        i2c_stop();
        return 1;
    }
    i2c_stop();
    return 0;
}

/*
 * Read byte from DS1307 register
 */
uint8_t ds1307_read_register(uint8_t reg_addr, uint8_t *data)
{
    if (i2c_start())
    {
        i2c_stop();
        return 1;
    }
    if (i2c_write((DS1307_ADDR << 1) | 0x00))
    {
        i2c_stop();
        return 1;
    }
    if (i2c_write(reg_addr))
    {
        i2c_stop();
        return 1;
    }
    if (i2c_start())
    {
        i2c_stop();
        return 1;
    }
    if (i2c_write((DS1307_ADDR << 1) | 0x01))
    {
        i2c_stop();
        return 1;
    }
    if (i2c_read(data, 0))
    {
        i2c_stop();
        return 1;
    }
    i2c_stop();
    return 0;
}

/*
 * Set time on DS1307
 */
uint8_t ds1307_set_time(rtc_time_t *time)
{
    // Start I2C transaction
    if (i2c_start())
    {
        i2c_stop();
        return 1;
    }

    // Send device address + write
    if (i2c_write((DS1307_ADDR << 1) | 0x00))
    {
        i2c_stop();
        return 1;
    }

    // Set register pointer to 0x00 (seconds)
    if (i2c_write(0x00))
    {
        i2c_stop();
        return 1;
    }

    // Write all time registers (BCD format)
    if (i2c_write(dec_to_bcd(time->seconds) & 0x7F))
    {
        i2c_stop();
        return 1;
    } // Clear CH bit
    if (i2c_write(dec_to_bcd(time->minutes)))
    {
        i2c_stop();
        return 1;
    }
    if (i2c_write(dec_to_bcd(time->hours) & 0x3F))
    {
        i2c_stop();
        return 1;
    } // 24-hour mode
    if (i2c_write(time->day_of_week & 0x07))
    {
        i2c_stop();
        return 1;
    }
    if (i2c_write(dec_to_bcd(time->date)))
    {
        i2c_stop();
        return 1;
    }
    if (i2c_write(dec_to_bcd(time->month)))
    {
        i2c_stop();
        return 1;
    }
    if (i2c_write(dec_to_bcd(time->year)))
    {
        i2c_stop();
        return 1;
    }

    i2c_stop();
    return 0;
}

/*
 * Get time from DS1307
 */
uint8_t ds1307_get_time(rtc_time_t *time)
{
    uint8_t data[7];

    // Set register pointer to 0x00
    if (i2c_start())
    {
        i2c_stop();
        return 1;
    }
    if (i2c_write((DS1307_ADDR << 1) | 0x00))
    {
        i2c_stop();
        return 1;
    }
    if (i2c_write(0x00))
    {
        i2c_stop();
        return 1;
    }

    // Repeated start for reading
    if (i2c_start())
    {
        i2c_stop();
        return 1;
    }
    if (i2c_write((DS1307_ADDR << 1) | 0x01))
    {
        i2c_stop();
        return 1;
    }

    // Read 7 bytes
    for (uint8_t i = 0; i < 6; i++)
    {
        i2c_read(&data[i], 1); // ACK
    }
    i2c_read(&data[6], 0); // NACK on last byte

    i2c_stop();

    // Convert BCD to decimal
    time->seconds = bcd_to_dec(data[0] & 0x7F);
    time->minutes = bcd_to_dec(data[1]);
    time->hours = bcd_to_dec(data[2] & 0x3F);
    time->day_of_week = data[3] & 0x07;
    time->date = bcd_to_dec(data[4]);
    time->month = bcd_to_dec(data[5]);
    time->year = bcd_to_dec(data[6]);

    return 0;
}

/* ========================================================================
 * DEMO 1: Set Current Time
 * ======================================================================== */
void demo1_set_time(void)
{
    puts_USART1("\r\n=== DEMO 1: Set RTC Time ===\r\n");
    puts_USART1("Enter time (24-hour format)\r\n\r\n");

    rtc_time_t time;

    // Get hours (simple input for demo)
    puts_USART1("Hours (00-23): ");
    char h1 = getch_USART1();
    putch_USART1(h1);
    char h2 = getch_USART1();
    putch_USART1(h2);
    time.hours = (h1 - '0') * 10 + (h2 - '0');

    puts_USART1("\r\nMinutes (00-59): ");
    char m1 = getch_USART1();
    putch_USART1(m1);
    char m2 = getch_USART1();
    putch_USART1(m2);
    time.minutes = (m1 - '0') * 10 + (m2 - '0');

    puts_USART1("\r\nSeconds (00-59): ");
    char s1 = getch_USART1();
    putch_USART1(s1);
    char s2 = getch_USART1();
    putch_USART1(s2);
    time.seconds = (s1 - '0') * 10 + (s2 - '0');

    // Get date
    puts_USART1("\r\n\r\nDate (01-31): ");
    char d1 = getch_USART1();
    putch_USART1(d1);
    char d2 = getch_USART1();
    putch_USART1(d2);
    time.date = (d1 - '0') * 10 + (d2 - '0');

    puts_USART1("\r\nMonth (01-12): ");
    char mo1 = getch_USART1();
    putch_USART1(mo1);
    char mo2 = getch_USART1();
    putch_USART1(mo2);
    time.month = (mo1 - '0') * 10 + (mo2 - '0');

    puts_USART1("\r\nYear (00-99): ");
    char y1 = getch_USART1();
    putch_USART1(y1);
    char y2 = getch_USART1();
    putch_USART1(y2);
    time.year = (y1 - '0') * 10 + (y2 - '0');

    puts_USART1("\r\nDay of week (1=Sun, 7=Sat): ");
    time.day_of_week = getch_USART1() - '0';

    // Set time
    puts_USART1("\r\n\r\nSetting RTC...");
    if (ds1307_set_time(&time) == 0)
    {
        puts_USART1(" Success!\r\n");
        PORTC = 0x0F;
    }
    else
    {
        puts_USART1(" Failed!\r\n");
        PORTC = 0xF0;
    }

    puts_USART1("\r\nPress any key to continue...");
    getch_USART1();
}

/* ========================================================================
 * DEMO 2: Display Real-Time Clock
 * ======================================================================== */
void demo2_display_clock(void)
{
    puts_USART1("\r\n=== DEMO 2: Real-Time Clock Display ===\r\n");
    puts_USART1("Press any key to stop\r\n\r\n");

    char *days[] = {"", "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

    while (1)
    {
        rtc_time_t time;

        if (ds1307_get_time(&time) == 0)
        {
            // Display time
            char buf[100];
            sprintf(buf, "\r%s 20%02u-%02u-%02u  %02u:%02u:%02u  ",
                    days[time.day_of_week],
                    time.year, time.month, time.date,
                    time.hours, time.minutes, time.seconds);
            puts_USART1(buf);

            // Blink LED each second
            PORTC ^= 0x01;
        }
        else
        {
            puts_USART1("\rError reading RTC!     ");
            PORTC = 0xFF;
        }

        _delay_ms(500);

        if (UCSR1A & (1 << RXC1))
        {
            getch_USART1();
            puts_USART1("\r\n\r\nClock display stopped.\r\n");
            return;
        }
    }
}

/* ========================================================================
 * DEMO 3: Alarm and Timer Functions
 * ======================================================================== */
void demo3_alarm_demo(void)
{
    puts_USART1("\r\n=== DEMO 3: Simple Alarm Demo ===\r\n");
    puts_USART1("Set alarm time\r\n\r\n");

    uint8_t alarm_hour, alarm_min;

    puts_USART1("Alarm hour (00-23): ");
    char h1 = getch_USART1();
    putch_USART1(h1);
    char h2 = getch_USART1();
    putch_USART1(h2);
    alarm_hour = (h1 - '0') * 10 + (h2 - '0');

    puts_USART1("\r\nAlarm minute (00-59): ");
    char m1 = getch_USART1();
    putch_USART1(m1);
    char m2 = getch_USART1();
    putch_USART1(m2);
    alarm_min = (m1 - '0') * 10 + (m2 - '0');

    char buf[80];
    sprintf(buf, "\r\n\r\nAlarm set for %02u:%02u\r\n", alarm_hour, alarm_min);
    puts_USART1(buf);
    puts_USART1("Monitoring... Press any key to stop\r\n\r\n");

    uint8_t alarm_triggered = 0;

    while (1)
    {
        rtc_time_t time;

        if (ds1307_get_time(&time) == 0)
        {
            sprintf(buf, "\rCurrent time: %02u:%02u:%02u  ",
                    time.hours, time.minutes, time.seconds);
            puts_USART1(buf);

            // Check alarm
            if (time.hours == alarm_hour && time.minutes == alarm_min && !alarm_triggered)
            {
                puts_USART1("\r\n\r\n*** ALARM! ALARM! ALARM! ***\r\n\r\n");

                // Blink LEDs rapidly
                for (uint8_t i = 0; i < 10; i++)
                {
                    PORTC = 0xFF;
                    _delay_ms(100);
                    PORTC = 0x00;
                    _delay_ms(100);
                }

                alarm_triggered = 1;
            }

            // Reset trigger when minute changes
            if (time.minutes != alarm_min)
            {
                alarm_triggered = 0;
            }
        }

        _delay_ms(500);

        if (UCSR1A & (1 << RXC1))
        {
            getch_USART1();
            puts_USART1("\r\n\r\nAlarm monitoring stopped.\r\n");
            return;
        }
    }
}

/* ========================================================================
 * DEMO 4: RTC RAM Storage
 * ======================================================================== */
void demo4_ram_storage(void)
{
    puts_USART1("\r\n=== DEMO 4: Battery-Backed RAM ===\r\n");
    puts_USART1("DS1307 has 56 bytes of non-volatile RAM\r\n\r\n");

    // Write test data to RAM
    puts_USART1("Writing test data to RAM...\r\n");
    for (uint8_t i = 0; i < 16; i++)
    {
        ds1307_write_register(DS1307_REG_RAM + i, 0xA0 + i);
    }

    puts_USART1("Reading back data...\r\n\r\n");

    // Read and display
    puts_USART1("Addr: ");
    for (uint8_t i = 0; i < 16; i++)
    {
        char buf[6];
        sprintf(buf, "%02X ", DS1307_REG_RAM + i);
        puts_USART1(buf);
    }
    puts_USART1("\r\nData: ");

    for (uint8_t i = 0; i < 16; i++)
    {
        uint8_t data;
        ds1307_read_register(DS1307_REG_RAM + i, &data);
        char buf[6];
        sprintf(buf, "%02X ", data);
        puts_USART1(buf);
    }

    puts_USART1("\r\n\r\nRAM is battery-backed and survives power loss!\r\n");
    puts_USART1("\r\nPress any key to continue...");
    getch_USART1();
}

/* ========================================================================
 * Main Menu System
 * ======================================================================== */
void display_main_menu(void)
{
    puts_USART1("\r\n\r\n");
    puts_USART1("╔════════════════════════════════════════╗\r\n");
    puts_USART1("║   I2C RTC (DS1307) - ATmega128        ║\r\n");
    puts_USART1("╚════════════════════════════════════════╝\r\n");
    puts_USART1("\r\n");
    puts_USART1("Select Demo:\r\n");
    puts_USART1("  [1] Set RTC Time\r\n");
    puts_USART1("  [2] Display Real-Time Clock\r\n");
    puts_USART1("  [3] Simple Alarm Demo\r\n");
    puts_USART1("  [4] Battery-Backed RAM Test\r\n");
    puts_USART1("\r\n");
    puts_USART1("Enter selection (1-4): ");
}

int main(void)
{
    // Initialize peripherals
    Uart1_init();
    i2c_init();

    // Configure status LEDs
    DDRC = 0xFF;
    PORTC = 0x00;

    // Send startup message
    _delay_ms(500);
    puts_USART1("\r\n\r\n*** DS1307 Real-Time Clock System ***\r\n");
    puts_USART1("I2C RTC with Battery Backup\r\n");
    puts_USART1("Address: 0x68, 56 bytes RAM\r\n");

    // Check if DS1307 is present
    if (i2c_start() == 0 && i2c_write((DS1307_ADDR << 1) | 0x00) == 0)
    {
        puts_USART1("DS1307 detected!\r\n");
        PORTC = 0x01;
    }
    else
    {
        puts_USART1("WARNING: DS1307 not found! Check connections.\r\n");
        PORTC = 0xFF;
    }
    i2c_stop();

    while (1)
    {
        display_main_menu();

        // Wait for user selection
        char choice = getch_USART1();
        putch_USART1(choice);
        puts_USART1("\r\n");

        switch (choice)
        {
        case '1':
            demo1_set_time();
            break;
        case '2':
            demo2_display_clock();
            break;
        case '3':
            demo3_alarm_demo();
            break;
        case '4':
            demo4_ram_storage();
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
