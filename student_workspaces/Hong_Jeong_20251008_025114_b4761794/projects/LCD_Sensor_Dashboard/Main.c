/*
 * LCD Sensor Dashboard - Real-Time Display System
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Integrate multiple sensors with LCD display
 * - Design efficient data presentation layouts
 * - Update display without flickering
 * - Create professional dashboard interfaces
 *
 * HARDWARE SETUP:
 * LCD: Same connections as previous LCD projects (4-bit mode on PORTG)
 * Sensors:
 * - ADC0 (PF0): Temperature sensor (LM35 or similar)
 * - ADC1 (PF1): Light sensor (CDS photocell)
 * - ADC2 (PF2): Potentiometer (general purpose input)
 * - Optional I2C sensors on PD0/PD1
 *
 * DASHBOARD FEATURES:
 * - Multi-sensor real-time display
 * - Graphical indicators and bargraphs
 * - Alert thresholds and warnings
 * - Data logging mode
 */

#include "config.h"

// LCD Control Pins
#define LCD_DDR DDRG
#define LCD_PORT PORTG
#define LCD_RS 0
#define LCD_E 1
#define LCD_D4 2
#define LCD_D5 3
#define LCD_D6 4
#define LCD_D7 5

// LCD Commands
#define LCD_CLEAR 0x01
#define LCD_HOME 0x02
#define LCD_ENTRY_MODE 0x06
#define LCD_DISPLAY_ON 0x0C
#define LCD_FUNCTION_SET 0x28
#define LCD_CGRAM_ADDR 0x40
#define LCD_DDRAM_ADDR 0x80

// LCD dimensions
#define LCD_ROWS 2
#define LCD_COLS 16

// Sensor thresholds
#define TEMP_WARN_HIGH 35
#define TEMP_WARN_LOW 15
#define LIGHT_DARK 200
#define LIGHT_BRIGHT 800

// Sensor data structure
typedef struct
{
    uint16_t temperature;  // ADC value
    uint16_t light;        // ADC value
    uint16_t analog_input; // Generic input
    float temp_celsius;
    uint8_t light_percent;
} sensor_data_t;

sensor_data_t sensors;

/*
 * Basic LCD functions
 */
void lcd_enable_pulse(void)
{
    LCD_PORT |= (1 << LCD_E);
    _delay_us(1);
    LCD_PORT &= ~(1 << LCD_E);
    _delay_us(50);
}

void lcd_write_nibble(uint8_t nibble)
{
    LCD_PORT = (LCD_PORT & 0xC3) | ((nibble & 0x0F) << LCD_D4);
    lcd_enable_pulse();
}

void lcd_write_byte(uint8_t data, uint8_t rs)
{
    if (rs)
        LCD_PORT |= (1 << LCD_RS);
    else
        LCD_PORT &= ~(1 << LCD_RS);

    lcd_write_nibble(data >> 4);
    lcd_write_nibble(data & 0x0F);
    _delay_us(50);
}

void lcd_command(uint8_t cmd)
{
    lcd_write_byte(cmd, 0);
    if (cmd == LCD_CLEAR || cmd == LCD_HOME)
        _delay_ms(2);
}

void lcd_data(uint8_t data)
{
    lcd_write_byte(data, 1);
}

void lcd_init(void)
{
    LCD_DDR |= (1 << LCD_RS) | (1 << LCD_E) |
               (1 << LCD_D4) | (1 << LCD_D5) |
               (1 << LCD_D6) | (1 << LCD_D7);

    _delay_ms(50);
    LCD_PORT &= ~(1 << LCD_RS);

    lcd_write_nibble(0x03);
    _delay_ms(5);
    lcd_write_nibble(0x03);
    _delay_us(150);
    lcd_write_nibble(0x03);
    _delay_us(150);
    lcd_write_nibble(0x02);
    _delay_us(150);

    lcd_command(LCD_FUNCTION_SET);
    lcd_command(LCD_DISPLAY_ON);
    lcd_command(LCD_CLEAR);
    lcd_command(LCD_ENTRY_MODE);
}

void lcd_clear(void)
{
    lcd_command(LCD_CLEAR);
}

void lcd_goto(uint8_t row, uint8_t col)
{
    uint8_t address;
    switch (row)
    {
    case 0:
        address = 0x00 + col;
        break;
    case 1:
        address = 0x40 + col;
        break;
    default:
        address = 0x00;
        break;
    }
    lcd_command(LCD_DDRAM_ADDR | address);
}

void lcd_puts(const char *str)
{
    while (*str)
    {
        lcd_data(*str++);
    }
}

void lcd_puts_at(uint8_t row, uint8_t col, const char *str)
{
    lcd_goto(row, col);
    lcd_puts(str);
}

void lcd_create_char(uint8_t location, const uint8_t *pattern)
{
    lcd_command(LCD_CGRAM_ADDR | (location << 3));
    for (uint8_t i = 0; i < 8; i++)
    {
        lcd_data(pattern[i]);
    }
}

/*
 * Initialize ADC
 */
void adc_init(void)
{
    // AVCC reference, right adjusted, ADC0
    ADMUX = (1 << REFS0);

    // Enable ADC, prescaler 64 (7.3728MHz/64 = 115kHz)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
}

/*
 * Read ADC channel
 */
uint16_t adc_read(uint8_t channel)
{
    // Select channel
    ADMUX = (ADMUX & 0xE0) | (channel & 0x1F);

    // Start conversion
    ADCSRA |= (1 << ADSC);

    // Wait for completion
    while (ADCSRA & (1 << ADSC))
        ;

    return ADC;
}

/*
 * Read all sensors
 */
void read_sensors(void)
{
    sensors.temperature = adc_read(0);
    sensors.light = adc_read(1);
    sensors.analog_input = adc_read(2);

    // Convert temperature (LM35: 10mV/°C, 5V ref, 10-bit ADC)
    // ADC = (Vin * 1024) / 5V
    // LM35: Vout = Temp(°C) * 10mV
    sensors.temp_celsius = (sensors.temperature * 5.0 * 100.0) / 1024.0;

    // Convert light to percentage
    sensors.light_percent = (sensors.light * 100) / 1023;
}

/* ========================================================================
 * DEMO 1: Basic Sensor Dashboard
 * ======================================================================== */
void demo1_basic_dashboard(void)
{
    puts_USART1("\r\n=== DEMO 1: Basic Dashboard ===\r\n");
    puts_USART1("Press any key to stop\r\n");

    lcd_clear();

    while (1)
    {
        read_sensors();

        // Display temperature
        char buf[20];
        sprintf(buf, "T:%.1fC L:%u%%  ", sensors.temp_celsius, sensors.light_percent);
        lcd_puts_at(0, 0, buf);

        // Display analog input
        sprintf(buf, "A:%4u (%3u%%)  ",
                sensors.analog_input,
                (uint8_t)((sensors.analog_input * 100) / 1023));
        lcd_puts_at(1, 0, buf);

        // UART output
        sprintf(buf, "\rT:%.1fC L:%u%% A:%u    ",
                sensors.temp_celsius, sensors.light_percent, sensors.analog_input);
        puts_USART1(buf);

        // LED indicator based on light
        PORTC = sensors.light_percent / 13; // 0-7 LEDs

        _delay_ms(250);

        if (UCSR1A & (1 << RXC1))
        {
            getch_USART1();
            puts_USART1("\r\n\r\nDashboard stopped.\r\n");
            return;
        }
    }
}

/* ========================================================================
 * DEMO 2: Graphical Dashboard with Bargraphs
 * ======================================================================== */
void demo2_graphical_dashboard(void)
{
    puts_USART1("\r\n=== DEMO 2: Graphical Dashboard ===\r\n");
    puts_USART1("Press any key to stop\r\n");

    // Create bargraph characters
    const uint8_t bar0[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    const uint8_t bar1[8] = {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10};
    const uint8_t bar2[8] = {0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18};
    const uint8_t bar3[8] = {0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C};
    const uint8_t bar4[8] = {0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E};
    const uint8_t bar5[8] = {0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F};

    lcd_create_char(0, bar0);
    lcd_create_char(1, bar1);
    lcd_create_char(2, bar2);
    lcd_create_char(3, bar3);
    lcd_create_char(4, bar4);
    lcd_create_char(5, bar5);

    lcd_clear();

    while (1)
    {
        read_sensors();

        // Row 0: Temperature bargraph
        lcd_goto(0, 0);
        lcd_data('T');

        uint8_t temp_bars = (uint8_t)(sensors.temp_celsius * 14 / 50); // 0-50°C range
        for (uint8_t i = 0; i < 14; i++)
        {
            if (i < temp_bars)
            {
                lcd_data(5); // Full bar
            }
            else
            {
                lcd_data(0); // Empty
            }
        }

        // Row 1: Light bargraph
        lcd_goto(1, 0);
        lcd_data('L');

        uint8_t light_bars = sensors.light_percent / 8; // 0-12 bars
        for (uint8_t i = 0; i < 14; i++)
        {
            if (i < light_bars)
            {
                lcd_data(5); // Full bar
            }
            else
            {
                lcd_data(0); // Empty
            }
        }

        // UART output
        char buf[60];
        sprintf(buf, "\rTemp:%2u/14 Light:%2u/14    ", temp_bars, light_bars);
        puts_USART1(buf);

        // LED display
        PORTC = (temp_bars > 10) ? 0xF0 : light_bars;

        _delay_ms(200);

        if (UCSR1A & (1 << RXC1))
        {
            getch_USART1();
            puts_USART1("\r\n\r\nStopped.\r\n");
            return;
        }
    }
}

/* ========================================================================
 * DEMO 3: Alert and Warning System
 * ======================================================================== */
void demo3_alert_system(void)
{
    puts_USART1("\r\n=== DEMO 3: Alert System ===\r\n");
    puts_USART1("Monitoring for threshold violations\r\n");
    puts_USART1("Press any key to stop\r\n");

    // Warning icon
    const uint8_t warn_icon[8] = {
        0b00000, 0b00100, 0b01110, 0b01110,
        0b11111, 0b11111, 0b00000, 0b00000};
    lcd_create_char(0, warn_icon);

    lcd_clear();

    uint16_t alert_count = 0;

    while (1)
    {
        read_sensors();

        uint8_t alert = 0;
        char msg[17] = "Status: OK      ";

        // Check temperature
        if (sensors.temp_celsius > TEMP_WARN_HIGH)
        {
            alert = 1;
            sprintf(msg, "WARN: Temp High!");
            alert_count++;
        }
        else if (sensors.temp_celsius < TEMP_WARN_LOW)
        {
            alert = 1;
            sprintf(msg, "WARN: Temp Low!");
            alert_count++;
        }

        // Check light
        if (sensors.light < LIGHT_DARK)
        {
            if (!alert)
            {
                alert = 1;
                sprintf(msg, "WARN: Too Dark!");
                alert_count++;
            }
        }

        // Display status
        lcd_goto(0, 0);
        if (alert)
        {
            lcd_data(0); // Warning icon
            lcd_data(' ');
        }
        else
        {
            lcd_puts("  ");
        }
        lcd_puts(msg);

        // Display sensor values
        char buf[20];
        sprintf(buf, "T:%.1fC L:%u%%  ", sensors.temp_celsius, sensors.light_percent);
        lcd_puts_at(1, 0, buf);

        // UART logging
        if (alert)
        {
            sprintf(buf, "\r[ALERT #%u] %s T:%.1fC L:%u%%\r\n",
                    alert_count, msg, sensors.temp_celsius, sensors.light_percent);
            puts_USART1(buf);

            // Flash LEDs
            PORTC = 0xFF;
            _delay_ms(100);
            PORTC = 0x00;
            _delay_ms(100);
        }
        else
        {
            PORTC = 0x01;
        }

        _delay_ms(500);

        if (UCSR1A & (1 << RXC1))
        {
            getch_USART1();
            sprintf(buf, "\r\n\r\nTotal alerts: %u\r\n", alert_count);
            puts_USART1(buf);
            return;
        }
    }
}

/* ========================================================================
 * DEMO 4: Data Logger with Statistics
 * ======================================================================== */
void demo4_data_logger(void)
{
    puts_USART1("\r\n=== DEMO 4: Data Logger ===\r\n");
    puts_USART1("Logging 30 samples at 1-second intervals\r\n\r\n");

    float temp_min = 999, temp_max = -999, temp_avg = 0;
    uint8_t light_min = 255, light_max = 0;

    lcd_clear();
    lcd_puts_at(0, 0, "Logging...");

    // CSV header
    puts_USART1("Sample,Temp_C,Light_%,Analog\r\n");

    for (uint8_t sample = 0; sample < 30; sample++)
    {
        read_sensors();

        // Update statistics
        if (sensors.temp_celsius < temp_min)
            temp_min = sensors.temp_celsius;
        if (sensors.temp_celsius > temp_max)
            temp_max = sensors.temp_celsius;
        temp_avg += sensors.temp_celsius;

        if (sensors.light_percent < light_min)
            light_min = sensors.light_percent;
        if (sensors.light_percent > light_max)
            light_max = sensors.light_percent;

        // Display progress
        char buf[20];
        sprintf(buf, "Sample: %u/30   ", sample + 1);
        lcd_puts_at(1, 0, buf);

        // CSV output
        sprintf(buf, "%u,%.1f,%u,%u\r\n",
                sample + 1, sensors.temp_celsius,
                sensors.light_percent, sensors.analog_input);
        puts_USART1(buf);

        // Progress LEDs
        PORTC = (sample * 255) / 30;

        _delay_ms(1000);
    }

    temp_avg /= 30;

    // Display summary
    lcd_clear();
    lcd_puts_at(0, 0, "Log Complete!");

    char buf[80];
    puts_USART1("\r\n=== Statistics ===\r\n");
    sprintf(buf, "Temperature: Min=%.1fC Avg=%.1fC Max=%.1fC\r\n",
            temp_min, temp_avg, temp_max);
    puts_USART1(buf);
    sprintf(buf, "Light: Min=%u%% Max=%u%%\r\n", light_min, light_max);
    puts_USART1(buf);

    // Show stats on LCD
    sprintf(buf, "T:%.1f-%.1fC ", temp_min, temp_max);
    lcd_puts_at(1, 0, buf);

    _delay_ms(3000);

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
    puts_USART1("║  LCD Sensor Dashboard - ATmega128     ║\r\n");
    puts_USART1("╚════════════════════════════════════════╝\r\n");
    puts_USART1("\r\n");
    puts_USART1("Select Demo:\r\n");
    puts_USART1("  [1] Basic Dashboard\r\n");
    puts_USART1("  [2] Graphical Bargraphs\r\n");
    puts_USART1("  [3] Alert System\r\n");
    puts_USART1("  [4] Data Logger\r\n");
    puts_USART1("\r\n");
    puts_USART1("Enter selection (1-4): ");
}

int main(void)
{
    // Initialize peripherals
    Uart1_init();
    lcd_init();
    adc_init();

    // Configure status LEDs
    DDRC = 0xFF;
    PORTC = 0x00;

    // Send startup message
    _delay_ms(500);
    puts_USART1("\r\n\r\n*** LCD Sensor Dashboard ***\r\n");
    puts_USART1("Real-time sensor monitoring\r\n");

    // Welcome screen
    lcd_clear();
    lcd_puts_at(0, 0, " Sensor System");
    lcd_puts_at(1, 0, "  Initializing..");

    // Test sensors
    _delay_ms(1000);
    read_sensors();

    lcd_clear();
    lcd_puts_at(0, 0, "Sensors Ready!");
    char buf[20];
    sprintf(buf, "T:%.1fC L:%u%%", sensors.temp_celsius, sensors.light_percent);
    lcd_puts_at(1, 0, buf);

    PORTC = 0x01;
    _delay_ms(2000);

    while (1)
    {
        display_main_menu();

        char choice = getch_USART1();
        putch_USART1(choice);
        puts_USART1("\r\n");

        switch (choice)
        {
        case '1':
            demo1_basic_dashboard();
            break;
        case '2':
            demo2_graphical_dashboard();
            break;
        case '3':
            demo3_alert_system();
            break;
        case '4':
            demo4_data_logger();
            break;
        default:
            puts_USART1("Invalid selection!\r\n");
            lcd_clear();
            lcd_puts_at(0, 0, "Invalid!");
            _delay_ms(1000);
            break;
        }

        _delay_ms(500);
    }

    return 0;
}
