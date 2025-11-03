/*
 * =============================================================================
 * LCD CHARACTER DISPLAY CONTROL - EDUCATIONAL DEMONSTRATION
 * =============================================================================
 *
 * PROJECT: LCD_Character_Basic
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Educational demonstration of HD44780-compatible LCD character display control.
 * Students learn parallel interface protocols and text display systems.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master LCD initialization and configuration sequences
 * 2. Learn 4-bit mode communication protocol
 * 3. Practice timing requirements and data synchronization
 * 4. Display text, numbers, and custom characters
 * 5. Implement cursor control and display formatting
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - 16x2 or 20x4 HD44780-compatible LCD display
 * - LCD connected in 4-bit mode for efficiency
 * - Contrast control potentiometer
 * - Serial connection for debugging (9600 baud)
 *
 * DOCUMENTATION REFERENCE:
 * ATmega128 Datasheet: https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/2467S.pdf
 * - I/O Ports (pages 62-75)
 * - Timing specifications (pages 301-320)
 *
 * =============================================================================
 * PORT REGISTERS FOR LCD CONTROL - DETAILED REFERENCE FOR STUDENTS
 * =============================================================================
 *
 * REGISTER 1: DDRx (Data Direction Register) - PIN CONFIGURATION
 *
 *    Bit:   7      6      5      4      3      2      1      0
 *    Name:  DD7    DD6    DD5    DD4    DD3    DD2    DD1    DD0
 *
 * Configures each pin as input (0) or output (1)
 *
 * LCD on PORTG (4-bit mode):
 *   DDRG |= (1<<PG0)|(1<<PG1)|(1<<PG2)|(1<<PG3)|(1<<PG4)|(1<<PG5);
 *   // PG0=RS, PG1=E, PG2-PG5=D4-D7 as outputs
 *   or: DDRG |= 0x3F;  // Lower 6 bits as outputs
 *
 * REGISTER 2: PORTx (Port Data Register) - OUTPUT CONTROL
 *
 *    Bit:   7      6      5      4      3      2      1      0
 *    Name:  P7     P6     P5     P4     P3     P2     P1     P0
 *
 * Sets output pin states:
 *   0 = Low (0V)
 *   1 = High (VCC)
 *
 * LCD Control Signals:
 *   RS (PG0): 0=Command mode, 1=Data mode
 *   E (PG1):  Enable pulse (High→Low transition latches data)
 *   D4-D7:    4-bit data bus
 *
 * REGISTER 3: PINx (Port Input Pins Register) - READING INPUTS
 *
 * Reads current state of pins (not typically used with LCD in write-only mode)
 *
 * LCD 4-BIT MODE PROTOCOL:
 *
 * Why 4-bit mode?
 * - Saves 4 I/O pins (only uses D4-D7, leaves D0-D3 unconnected)
 * - Each byte sent in two 4-bit nibbles (high nibble first)
 * - Standard for embedded systems with limited pins
 *
 * TIMING CRITICAL SEQUENCE:
 *
 *   Send Command/Data:
 *     1. Set RS: PORTG &= ~(1<<PG0);  // Command
 *        or:     PORTG |= (1<<PG0);   // Data
 *     2. Send high nibble:
 *        PORTG = (PORTG & 0xC3) | ((data >> 2) & 0x3C);
 *     3. Enable pulse:
 *        PORTG |= (1<<PG1);   // E high
 *        _delay_us(1);        // Min 450ns
 *        PORTG &= ~(1<<PG1);  // E low
 *     4. Send low nibble:
 *        PORTG = (PORTG & 0xC3) | ((data << 2) & 0x3C);
 *     5. Enable pulse (repeat step 3)
 *     6. Delay: _delay_us(50);  // Command execution time
 *
 * LCD INITIALIZATION SEQUENCE (CRITICAL):
 *
 *   void lcd_init(void) {
 *       // Configure pins as outputs
 *       DDRG |= 0x3F;
 *       PORTG = 0x00;
 *
 *       // Power-on delay
 *       _delay_ms(50);
 *
 *       // Special init sequence for 4-bit mode
 *       lcd_nibble(0x03);  _delay_ms(5);   // Function set (8-bit)
 *       lcd_nibble(0x03);  _delay_us(150); // Repeat
 *       lcd_nibble(0x03);  _delay_us(150); // Repeat
 *       lcd_nibble(0x02);  _delay_us(150); // Switch to 4-bit mode
 *
 *       // Now in 4-bit mode, send full commands
 *       lcd_command(0x28);  // 4-bit, 2 lines, 5x8 font
 *       lcd_command(0x0C);  // Display on, cursor off
 *       lcd_command(0x06);  // Entry mode: increment, no shift
 *       lcd_command(0x01);  // Clear display
 *       _delay_ms(2);       // Clear needs 1.52ms
 *   }
 *
 * LCD COMMAND SET:
 *
 *   Clear Display: 0x01
 *     Clears all display, returns cursor to home
 *     Execution time: 1.52ms
 *
 *   Return Home: 0x02
 *     Returns cursor to position (0,0)
 *     Display content unchanged
 *     Execution time: 1.52ms
 *
 *   Entry Mode Set: 0x04-0x07
 *     0x06: Increment cursor, no display shift (standard)
 *     0x04: Decrement cursor
 *     0x05: Increment cursor with display shift
 *
 *   Display Control: 0x08-0x0F
 *     0x08: Display off
 *     0x0C: Display on, cursor off, blink off (standard)
 *     0x0E: Display on, cursor on, blink off
 *     0x0F: Display on, cursor on, blink on
 *
 *   Function Set: 0x20-0x3F
 *     0x28: 4-bit mode, 2 lines, 5x8 font (standard)
 *     0x38: 8-bit mode, 2 lines, 5x8 font
 *
 *   Set DDRAM Address: 0x80-0xFF
 *     Position cursor at specific location
 *     Line 1: 0x80-0x8F (addresses 0x00-0x0F)
 *     Line 2: 0xC0-0xCF (addresses 0x40-0x4F)
 *     Example: lcd_command(0x80 + col + (row * 0x40));
 *
 * DISPLAY FUNCTIONS:
 *
 *   Print String:
 *     void lcd_puts(const char *str) {
 *         while(*str) {
 *             lcd_data(*str++);
 *         }
 *     }
 *
 *   Position Cursor:
 *     void lcd_goto(uint8_t row, uint8_t col) {
 *         uint8_t addr = (row == 0) ? 0x80 + col : 0xC0 + col;
 *         lcd_command(addr);
 *     }
 *
 *   Print Number:
 *     void lcd_print_num(uint16_t num) {
 *         char buffer[6];
 *         itoa(num, buffer, 10);
 *         lcd_puts(buffer);
 *     }
 *
 * CUSTOM CHARACTER CREATION:
 *
 * LCD has 8 custom character slots (0x00-0x07)
 * Each character: 5×8 pixels
 *
 *   Define Custom Char:
 *     void lcd_create_char(uint8_t location, uint8_t charmap[]) {
 *         lcd_command(0x40 + (location * 8));  // Set CGRAM address
 *         for(uint8_t i = 0; i < 8; i++) {
 *             lcd_data(charmap[i]);
 *         }
 *     }
 *
 *   Example Heart Symbol:
 *     uint8_t heart[8] = {
 *         0b00000,
 *         0b01010,
 *         0b11111,
 *         0b11111,
 *         0b01110,
 *         0b00100,
 *         0b00000,
 *         0b00000
 *     };
 *     lcd_create_char(0, heart);
 *     lcd_data(0);  // Display custom char 0
 *
 * TIMING REQUIREMENTS (HD44780):
 * - Enable pulse width: Min 450ns (use 1µs to be safe)
 * - Enable cycle time: Min 1µs (1µs high + delay)
 * - Command execution: 37µs (use 50µs)
 * - Clear/Home: 1.52ms (use 2ms)
 * - Power-on delay: 50ms minimum
 *
 * CRITICAL NOTES:
 * 1. ALWAYS send high nibble first, then low nibble
 * 2. Initialize sequence MUST be exact (3× 0x03, then 0x02)
 * 3. RW pin tied to GND = write-only mode (simpler, standard)
 * 4. Contrast (V0) needs ~0.5V for good visibility
 * 5. Backlight current: 20-100mA (use series resistor)
 *
 * =============================================================================
 *
 * LEARNING PROGRESSION:
 * - Demo 1: LCD Initialization Sequence
 * - Demo 2: Basic Text Display
 * - Demo 3: Cursor and Display Control
 * - Demo 4: Custom Character Creation
 * - Demo 5: Dynamic Content Updates
 *
 * =============================================================================
 */
*V0(3)  → Contrast adjustment(potentiometer) * RS(4)  → PG0 *RW(5)  → GND(always write mode) * E(6)  → PG1 *D4(11) → PG2 *D5(12) → PG3 *D6(13) → PG4 *D7(14) → PG5 *A(15) → + 5V(backlight anode, with resistor) * K(16) → GND(backlight cathode) * *STANDARD LCD COMMANDS : *-Clear Display : 0x01 * -Return Home : 0x02 * -Entry Mode Set : 0x04 - 0x07 * -Display Control : 0x08 - 0x0F * -Cursor / Display Shift : 0x10 - 0x1F * -Function Set : 0x20 - 0x3F * -Set CGRAM Address : 0x40 - 0x7F * -Set DDRAM Address : 0x80 - 0xFF * /

#include "config.h"

// LCD Control Pin Definitions
#define LCD_DDR DDRG
#define LCD_PORT PORTG
#define LCD_RS 0 // Register Select (0=Command, 1=Data)
#define LCD_E 1  // Enable
#define LCD_D4 2 // Data bit 4
#define LCD_D5 3 // Data bit 5
#define LCD_D6 4 // Data bit 6
#define LCD_D7 5 // Data bit 7

// LCD Commands
#define LCD_CLEAR 0x01
#define LCD_HOME 0x02
#define LCD_ENTRY_MODE 0x06     // Increment cursor, no shift
#define LCD_DISPLAY_ON 0x0C     // Display on, cursor off, blink off
#define LCD_DISPLAY_CURSOR 0x0E // Display on, cursor on
#define LCD_DISPLAY_BLINK 0x0F  // Display on, cursor on, blink on
#define LCD_FUNCTION_SET 0x28   // 4-bit mode, 2 lines, 5x8 font
#define LCD_CGRAM_ADDR 0x40
#define LCD_DDRAM_ADDR 0x80

// LCD dimensions
#define LCD_ROWS 2
#define LCD_COLS 16

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      /*
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       * Send enable pulse
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       */
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      void lcd_enable_pulse(void)
{
    LCD_PORT |= (1 << LCD_E);
    _delay_us(1);
    LCD_PORT &= ~(1 << LCD_E);
    _delay_us(50);
}

/*
 * Write 4 bits to LCD
 */
void lcd_write_nibble(uint8_t nibble)
{
    // Set data pins
    LCD_PORT = (LCD_PORT & 0xC3) | ((nibble & 0x0F) << LCD_D4);
    lcd_enable_pulse();
}

/*
 * Write byte to LCD (8 bits as two 4-bit transfers)
 */
void lcd_write_byte(uint8_t data, uint8_t rs)
{
    // Set RS pin
    if (rs)
        LCD_PORT |= (1 << LCD_RS);
    else
        LCD_PORT &= ~(1 << LCD_RS);

    // Send high nibble
    lcd_write_nibble(data >> 4);

    // Send low nibble
    lcd_write_nibble(data & 0x0F);

    _delay_us(50);
}

/*
 * Send command to LCD
 */
void lcd_command(uint8_t cmd)
{
    lcd_write_byte(cmd, 0);
    if (cmd == LCD_CLEAR || cmd == LCD_HOME)
        _delay_ms(2);
}

/*
 * Send data (character) to LCD
 */
void lcd_data(uint8_t data)
{
    lcd_write_byte(data, 1);
}

/*
 * Initialize LCD in 4-bit mode
 */
void lcd_init(void)
{
    // Configure pins as outputs
    LCD_DDR |= (1 << LCD_RS) | (1 << LCD_E) |
               (1 << LCD_D4) | (1 << LCD_D5) |
               (1 << LCD_D6) | (1 << LCD_D7);

    // Wait for LCD to power up
    _delay_ms(50);

    // Initialization sequence for 4-bit mode
    LCD_PORT &= ~(1 << LCD_RS); // Command mode

    // Send 0x03 three times (8-bit mode reset)
    lcd_write_nibble(0x03);
    _delay_ms(5);
    lcd_write_nibble(0x03);
    _delay_us(150);
    lcd_write_nibble(0x03);
    _delay_us(150);

    // Switch to 4-bit mode
    lcd_write_nibble(0x02);
    _delay_us(150);

    // Function set: 4-bit, 2 lines, 5x8 font
    lcd_command(LCD_FUNCTION_SET);

    // Display control: display on, cursor off, blink off
    lcd_command(LCD_DISPLAY_ON);

    // Clear display
    lcd_command(LCD_CLEAR);

    // Entry mode: increment cursor, no display shift
    lcd_command(LCD_ENTRY_MODE);
}

/*
 * Clear LCD screen
 */
void lcd_clear(void)
{
    lcd_command(LCD_CLEAR);
}

/*
 * Set cursor position (row: 0-1, col: 0-15 for 16x2)
 */
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
    case 2:
        address = 0x14 + col;
        break; // For 20x4 LCDs
    case 3:
        address = 0x54 + col;
        break;
    default:
        address = 0x00;
        break;
    }

    lcd_command(LCD_DDRAM_ADDR | address);
}

/*
 * Print string to LCD
 */
void lcd_puts(const char *str)
{
    while (*str)
    {
        lcd_data(*str++);
    }
}

/*
 * Print string at specific position
 */
void lcd_puts_at(uint8_t row, uint8_t col, const char *str)
{
    lcd_goto(row, col);
    lcd_puts(str);
}

/*
 * Create custom character (8 bytes for 5x8 character)
 */
void lcd_create_char(uint8_t location, const uint8_t *pattern)
{
    lcd_command(LCD_CGRAM_ADDR | (location << 3));
    for (uint8_t i = 0; i < 8; i++)
    {
        lcd_data(pattern[i]);
    }
}

/* ========================================================================
 * DEMO 1: Basic Text Display
 * ======================================================================== */
void demo1_basic_text(void)
{
    puts_USART1("\r\n=== DEMO 1: Basic Text Display ===\r\n");

    lcd_clear();
    lcd_puts_at(0, 0, "ATmega128");
    lcd_puts_at(1, 0, "LCD Demo 4-bit");

    puts_USART1("Displaying basic text on LCD\r\n");
    puts_USART1("Line 1: ATmega128\r\n");
    puts_USART1("Line 2: LCD Demo 4-bit\r\n");

    _delay_ms(3000);

    // Demonstrate cursor positioning
    lcd_clear();
    for (uint8_t row = 0; row < LCD_ROWS; row++)
    {
        for (uint8_t col = 0; col < LCD_COLS; col++)
        {
            lcd_goto(row, col);
            lcd_data('A' + (row * LCD_COLS + col) % 26);
            _delay_ms(50);
        }
    }

    puts_USART1("\r\nDisplaying alphabet pattern...\r\n");
    _delay_ms(2000);

    puts_USART1("Press any key to continue...");
    getch_USART1();
}

/* ========================================================================
 * DEMO 2: Numbers and Formatting
 * ======================================================================== */
void demo2_numbers(void)
{
    puts_USART1("\r\n=== DEMO 2: Numbers and Formatting ===\r\n");

    lcd_clear();
    lcd_puts_at(0, 0, "Counter Demo:");

    puts_USART1("Displaying counter on LCD\r\n");
    puts_USART1("Press any key to stop\r\n");

    uint16_t count = 0;

    while (1)
    {
        char buf[17];
        sprintf(buf, "Count: %5u", count);
        lcd_puts_at(1, 0, buf);

        // Also send to UART
        sprintf(buf, "\rCount: %u    ", count);
        puts_USART1(buf);

        count++;

        // Blink LED
        PORTC ^= 0x01;

        _delay_ms(200);

        if (UCSR1A & (1 << RXC1))
        {
            getch_USART1();
            break;
        }
    }

    // Demonstrate hex and binary
    lcd_clear();
    lcd_puts_at(0, 0, "Dec:123 Hex:7B");
    lcd_puts_at(1, 0, "Bin:01111011");

    puts_USART1("\r\n\r\nShowing different number formats\r\n");
    _delay_ms(3000);

    puts_USART1("Press any key to continue...");
    getch_USART1();
}

/* ========================================================================
 * DEMO 3: Custom Characters
 * ======================================================================== */
void demo3_custom_chars(void)
{
    puts_USART1("\r\n=== DEMO 3: Custom Characters ===\r\n");

    // Define custom characters
    const uint8_t heart[8] = {
        0b00000,
        0b01010,
        0b11111,
        0b11111,
        0b01110,
        0b00100,
        0b00000,
        0b00000};

    const uint8_t bell[8] = {
        0b00100,
        0b01110,
        0b01110,
        0b01110,
        0b11111,
        0b00000,
        0b00100,
        0b00000};

    const uint8_t arrow_right[8] = {
        0b00000,
        0b01000,
        0b01100,
        0b01110,
        0b01100,
        0b01000,
        0b00000,
        0b00000};

    const uint8_t arrow_left[8] = {
        0b00000,
        0b00010,
        0b00110,
        0b01110,
        0b00110,
        0b00010,
        0b00000,
        0b00000};

    const uint8_t battery_full[8] = {
        0b01110,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111};

    // Create custom characters
    lcd_create_char(0, heart);
    lcd_create_char(1, bell);
    lcd_create_char(2, arrow_right);
    lcd_create_char(3, arrow_left);
    lcd_create_char(4, battery_full);

    // Display custom characters
    lcd_clear();
    lcd_puts_at(0, 0, "Custom Chars:");
    lcd_goto(1, 0);
    lcd_data(0); // Heart
    lcd_data(' ');
    lcd_data(1); // Bell
    lcd_data(' ');
    lcd_data(2); // Arrow right
    lcd_data(' ');
    lcd_data(3); // Arrow left
    lcd_data(' ');
    lcd_data(4); // Battery

    puts_USART1("Displaying custom characters:\r\n");
    puts_USART1("Heart, Bell, Arrows, Battery\r\n");

    _delay_ms(3000);

    // Animated arrows
    lcd_clear();
    lcd_puts_at(0, 0, "Animation:");

    puts_USART1("\r\nAnimating arrows...\r\n");

    for (uint8_t i = 0; i < 3; i++)
    {
        for (uint8_t pos = 0; pos < 12; pos++)
        {
            lcd_goto(1, pos);
            lcd_data(2); // Arrow right
            _delay_ms(100);
            lcd_goto(1, pos);
            lcd_data(' ');
        }
    }

    puts_USART1("Press any key to continue...");
    getch_USART1();
}

/* ========================================================================
 * DEMO 4: Real-Time Display
 * ======================================================================== */
void demo4_realtime(void)
{
    puts_USART1("\r\n=== DEMO 4: Real-Time Display ===\r\n");
    puts_USART1("Press any key to stop\r\n");

    lcd_clear();
    lcd_puts_at(0, 0, "Real-Time Data:");

    uint16_t counter = 0;

    while (1)
    {
        // Simulate sensor readings
        uint16_t temp = 20 + (counter % 15);
        uint16_t humid = 45 + (counter % 30);
        uint8_t light = counter % 100;

        // Display on LCD
        char buf[17];
        sprintf(buf, "T:%uC H:%u%%    ", temp, humid);
        lcd_puts_at(1, 0, buf);

        sprintf(buf, "L:%u%%  ", light);
        lcd_puts_at(1, 11, buf);

        // Send to UART
        sprintf(buf, "\rTemp:%uC Humid:%u%% Light:%u%%    ", temp, humid, light);
        puts_USART1(buf);

        // Progress bar on LEDs
        PORTC = (light < 25) ? 0x01 : (light < 50) ? 0x03
                                  : (light < 75)   ? 0x07
                                                   : 0x0F;

        counter++;
        _delay_ms(500);

        if (UCSR1A & (1 << RXC1))
        {
            getch_USART1();
            puts_USART1("\r\n\r\nStopped.\r\n");
            return;
        }
    }
}

/* ========================================================================
 * Main Menu System
 * ======================================================================== */
void display_main_menu(void)
{
    puts_USART1("\r\n\r\n");
    puts_USART1("╔════════════════════════════════════════╗\r\n");
    puts_USART1("║   LCD Display (4-bit) - ATmega128     ║\r\n");
    puts_USART1("╚════════════════════════════════════════╝\r\n");
    puts_USART1("\r\n");
    puts_USART1("Select Demo:\r\n");
    puts_USART1("  [1] Basic Text Display\r\n");
    puts_USART1("  [2] Numbers and Formatting\r\n");
    puts_USART1("  [3] Custom Characters\r\n");
    puts_USART1("  [4] Real-Time Display\r\n");
    puts_USART1("\r\n");
    puts_USART1("Enter selection (1-4): ");
}

int main(void)
{
    // Initialize peripherals
    Uart1_init();
    lcd_init();

    // Configure status LEDs
    DDRC = 0xFF;
    PORTC = 0x00;

    // Send startup message
    _delay_ms(500);
    puts_USART1("\r\n\r\n*** HD44780 LCD Character Display ***\r\n");
    puts_USART1("4-bit mode, 16x2 display\r\n");

    // Welcome message on LCD
    lcd_clear();
    lcd_puts_at(0, 0, "  ATmega128  ");
    lcd_puts_at(1, 0, " LCD Ready! ");

    PORTC = 0x01;
    _delay_ms(2000);

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
            demo1_basic_text();
            break;
        case '2':
            demo2_numbers();
            break;
        case '3':
            demo3_custom_chars();
            break;
        case '4':
            demo4_realtime();
            break;
        default:
            puts_USART1("Invalid selection!\r\n");
            lcd_clear();
            lcd_puts_at(0, 0, "Invalid choice!");
            _delay_ms(1000);
            break;
        }

        _delay_ms(500);
    }

    return 0;
}
