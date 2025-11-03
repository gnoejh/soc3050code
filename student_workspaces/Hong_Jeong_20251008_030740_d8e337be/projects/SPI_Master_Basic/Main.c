/*
 * =============================================================================
 * SPI MASTER COMMUNICATION - EDUCATIONAL DEMONSTRATION
 * =============================================================================
 *
 * PROJECT: SPI_Master_Basic
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Educational demonstration of SPI master communication protocols.
 * Students learn synchronous serial communication and device interfacing.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master SPI protocol configuration and timing
 * 2. Learn master-slave communication concepts
 * 3. Practice device selection and control
 * 4. Understand clock polarity and phase settings
 * 5. Implement multi-device SPI networks
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - SPI slave device (EEPROM, DAC, or shift register)
 * - SPI pins: MOSI, MISO, SCK, SS
 * - LED indicators for status feedback
 * - Serial connection for debugging (9600 baud)
 *
 * LEARNING PROGRESSION:
 * - Demo 1: SPI Master Configuration
 * - Demo 2: Single Device Communication
 * - Demo 3: Data Transfer Operations
 * - Demo 4: Multiple Device Control
 * - Demo 5: Protocol Error Handling
 *
 * =============================================================================
 */
*-Full - Duplex : Simultaneous bidirectional communication * -Clock Polarity(CPOL) : Clock idle state(0 = low, 1 = high) * -Clock Phase(CPHA) : Data sampling edge(0 = leading, 1 = trailing) * -MSB First : Most significant bit transmitted first
                  *
                  -Speeds : F_CPU / 2,
    / 4, / 8, / 16, / 32, / 64, / 128 * /

#include "config.h"

// SPI Pin Definitions
#define SPI_SS PB0
#define SPI_SCK PB1
#define SPI_MOSI PB2
#define SPI_MISO PB3

#define SPI_DDR DDRB
#define SPI_PORT PORTB

// Chip Select Macros
#define CS_LOW() (SPI_PORT &= ~(1 << SPI_SS))
#define CS_HIGH() (SPI_PORT |= (1 << SPI_SS))

                                    /*
                                     * Initialize SPI in Master Mode
                                     * Clock: F_CPU / 16 (460.8 kHz @ 7.3728 MHz)
                                     * Mode: 0 (CPOL=0, CPHA=0)
                                     * MSB first
                                     */
                                    void spi_master_init(void)
{
    // Set MOSI, SCK, SS as outputs, MISO as input
    SPI_DDR |= (1 << SPI_MOSI) | (1 << SPI_SCK) | (1 << SPI_SS);
    SPI_DDR &= ~(1 << SPI_MISO);

    // Set SS high initially (chip deselected)
    CS_HIGH();

    // Enable SPI, Master mode, Clock = F_CPU/16
    // SPCR = SPI Control Register
    // SPE  = SPI Enable
    // MSTR = Master Mode
    // SPR0 = Clock Rate Select (with SPR1=0: F_CPU/16)
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

/*
 * Transmit and receive one byte via SPI
 * SPI is full-duplex: sending also receives
 */
uint8_t spi_transfer(uint8_t data)
{
    // Load data into SPI data register
    SPDR = data;

    // Wait for transmission complete
    // SPIF = SPI Interrupt Flag (set when transfer done)
    while (!(SPSR & (1 << SPIF)))
        ;

    // Return received data
    return SPDR;
}

/*
 * Send multiple bytes
 */
void spi_write_bytes(uint8_t *data, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        spi_transfer(data[i]);
    }
}

/*
 * Read multiple bytes (send dummy 0xFF)
 */
void spi_read_bytes(uint8_t *buffer, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        buffer[i] = spi_transfer(0xFF);
    }
}

/*
 * Set SPI clock speed
 */
void spi_set_speed(uint8_t speed)
{
    // Speed: 0=F_CPU/4, 1=F_CPU/16, 2=F_CPU/64, 3=F_CPU/128
    SPCR &= ~((1 << SPR1) | (1 << SPR0));

    switch (speed)
    {
    case 0: // F_CPU/4 (fastest)
        SPSR |= (1 << SPI2X);
        break;
    case 1: // F_CPU/16
        SPCR |= (1 << SPR0);
        break;
    case 2: // F_CPU/64
        SPCR |= (1 << SPR1);
        break;
    case 3: // F_CPU/128 (slowest)
        SPCR |= (1 << SPR1) | (1 << SPR0);
        break;
    }
}

/* ========================================================================
 * DEMO 1: Basic SPI Transmission Test
 * ======================================================================== */
void demo1_basic_transmission(void)
{
    puts_USART1("\r\n=== DEMO 1: Basic SPI Transmission ===\r\n");
    puts_USART1("Sending test patterns via SPI\r\n");
    puts_USART1("Monitor with logic analyzer or SPI slave\r\n");
    puts_USART1("Press 'q' to return to menu\r\n\r\n");

    uint8_t test_patterns[] = {0x00, 0xFF, 0xAA, 0x55, 0x0F, 0xF0};
    uint8_t num_patterns = sizeof(test_patterns) / sizeof(test_patterns[0]);

    while (1)
    {
        for (uint8_t i = 0; i < num_patterns; i++)
        {
            // Select device
            CS_LOW();
            _delay_us(1);

            // Send byte
            uint8_t received = spi_transfer(test_patterns[i]);

            // Deselect device
            _delay_us(1);
            CS_HIGH();

            // Display result
            char buf[80];
            sprintf(buf, "Sent: 0x%02X  Received: 0x%02X\r\n",
                    test_patterns[i], received);
            puts_USART1(buf);

            _delay_ms(500);

            // Check for quit
            if (UCSR1A & (1 << RXC1))
            {
                char cmd = getch_USART1();
                if (cmd == 'q' || cmd == 'Q')
                {
                    CS_HIGH();
                    return;
                }
            }
        }

        puts_USART1("\r\n");
    }
}

/* ========================================================================
 * DEMO 2: SPI Loopback Test (MOSI → MISO)
 * ======================================================================== */
void demo2_loopback_test(void)
{
    puts_USART1("\r\n=== DEMO 2: SPI Loopback Test ===\r\n");
    puts_USART1("Connect MOSI (PB2) to MISO (PB3) for testing\r\n");
    puts_USART1("This verifies SPI hardware operation\r\n");
    puts_USART1("Press 'q' to return to menu\r\n\r\n");

    uint8_t test_counter = 0;
    uint8_t errors = 0;
    uint8_t success = 0;

    while (1)
    {
        CS_LOW();
        _delay_us(1);

        uint8_t sent = test_counter;
        uint8_t received = spi_transfer(sent);

        _delay_us(1);
        CS_HIGH();

        // Verify loopback
        if (received == sent)
        {
            success++;
            PORTC = 0x01; // Success LED
        }
        else
        {
            errors++;
            PORTC = 0x80; // Error LED
        }

        // Display every 10 transfers
        if (test_counter % 10 == 0)
        {
            char buf[80];
            sprintf(buf, "Count: %3u  Success: %3u  Errors: %3u  Last: 0x%02X\r\n",
                    test_counter, success, errors, received);
            puts_USART1(buf);
        }

        test_counter++;
        _delay_ms(100);

        // Check for quit
        if (UCSR1A & (1 << RXC1))
        {
            char cmd = getch_USART1();
            if (cmd == 'q' || cmd == 'Q')
            {
                puts_USART1("\r\nLoopback test complete!\r\n");
                char summary[80];
                sprintf(summary, "Total: %u  Success: %u  Errors: %u  Rate: %u%%\r\n",
                        test_counter, success, errors, (success * 100) / test_counter);
                puts_USART1(summary);
                CS_HIGH();
                return;
            }
        }
    }
}

/* ========================================================================
 * DEMO 3: SPI Speed Comparison
 * ======================================================================== */
void demo3_speed_comparison(void)
{
    puts_USART1("\r\n=== DEMO 3: SPI Speed Comparison ===\r\n");
    puts_USART1("Testing different SPI clock speeds\r\n");
    puts_USART1("Measuring transfer time for 1000 bytes\r\n\r\n");

    char *speed_names[] = {"F_CPU/4", "F_CPU/16", "F_CPU/64", "F_CPU/128"};
    uint32_t frequencies[] = {1843200, 460800, 115200, 57600}; // @ 7.3728 MHz

    for (uint8_t speed = 0; speed < 4; speed++)
    {
        spi_set_speed(speed);

        // Prepare test data
        uint8_t test_data[100];
        for (uint8_t i = 0; i < 100; i++)
        {
            test_data[i] = i;
        }

        // Measure time for 1000 bytes (10 x 100)
        CS_LOW();

        uint16_t start_time = TCNT1; // Assuming Timer1 is running

        for (uint8_t loop = 0; loop < 10; loop++)
        {
            spi_write_bytes(test_data, 100);
        }

        uint16_t end_time = TCNT1;
        uint16_t elapsed = end_time - start_time;

        CS_HIGH();

        // Display results
        char buf[100];
        sprintf(buf, "Speed: %-10s  Freq: %6lu Hz  Time: %u ticks\r\n",
                speed_names[speed], frequencies[speed], elapsed);
        puts_USART1(buf);

        _delay_ms(500);
    }

    puts_USART1("\r\nSpeed comparison complete!\r\n");
    puts_USART1("Press any key to continue...");
    getch_USART1();
}

/* ========================================================================
 * DEMO 4: Interactive SPI Terminal
 * ======================================================================== */
void demo4_interactive_terminal(void)
{
    puts_USART1("\r\n=== DEMO 4: Interactive SPI Terminal ===\r\n");
    puts_USART1("Commands:\r\n");
    puts_USART1("  s[XX]: Send hex byte (e.g., sAA)\r\n");
    puts_USART1("  r: Read one byte (send 0xFF)\r\n");
    puts_USART1("  c: Toggle chip select\r\n");
    puts_USART1("  q: Return to menu\r\n\r\n");

    uint8_t cs_state = 1; // Initially high
    char input_buffer[10];
    uint8_t buf_index = 0;

    while (1)
    {
        if (UCSR1A & (1 << RXC1))
        {
            char c = getch_USART1();
            putch_USART1(c); // Echo

            if (c == '\r' || c == '\n')
            {
                input_buffer[buf_index] = '\0';
                puts_USART1("\r\n");

                if (buf_index > 0)
                {
                    if (input_buffer[0] == 's' || input_buffer[0] == 'S')
                    {
                        // Send hex byte
                        if (buf_index >= 3)
                        {
                            uint8_t value = 0;
                            // Simple hex parsing
                            for (uint8_t i = 1; i <= 2; i++)
                            {
                                char hex_char = input_buffer[i];
                                value <<= 4;
                                if (hex_char >= '0' && hex_char <= '9')
                                    value |= (hex_char - '0');
                                else if (hex_char >= 'A' && hex_char <= 'F')
                                    value |= (hex_char - 'A' + 10);
                                else if (hex_char >= 'a' && hex_char <= 'f')
                                    value |= (hex_char - 'a' + 10);
                            }

                            uint8_t received = spi_transfer(value);
                            char msg[60];
                            sprintf(msg, "→ Sent: 0x%02X  ← Received: 0x%02X\r\n",
                                    value, received);
                            puts_USART1(msg);
                        }
                    }
                    else if (input_buffer[0] == 'r' || input_buffer[0] == 'R')
                    {
                        uint8_t received = spi_transfer(0xFF);
                        char msg[40];
                        sprintf(msg, "← Received: 0x%02X\r\n", received);
                        puts_USART1(msg);
                    }
                    else if (input_buffer[0] == 'c' || input_buffer[0] == 'C')
                    {
                        cs_state = !cs_state;
                        if (cs_state)
                        {
                            CS_HIGH();
                            puts_USART1("CS: HIGH (deselected)\r\n");
                        }
                        else
                        {
                            CS_LOW();
                            puts_USART1("CS: LOW (selected)\r\n");
                        }
                    }
                    else if (input_buffer[0] == 'q' || input_buffer[0] == 'Q')
                    {
                        CS_HIGH();
                        return;
                    }
                }
                buf_index = 0;
            }
            else if (c == 8 || c == 127)
            { // Backspace
                if (buf_index > 0)
                {
                    buf_index--;
                    puts_USART1(" \b");
                }
            }
            else if (buf_index < sizeof(input_buffer) - 1)
            {
                input_buffer[buf_index++] = c;
            }
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
    puts_USART1("║   SPI MASTER BASIC - ATmega128        ║\r\n");
    puts_USART1("╚════════════════════════════════════════╝\r\n");
    puts_USART1("\r\n");
    puts_USART1("Select Demo:\r\n");
    puts_USART1("  [1] Basic SPI Transmission\r\n");
    puts_USART1("  [2] Loopback Test (MOSI→MISO)\r\n");
    puts_USART1("  [3] Speed Comparison\r\n");
    puts_USART1("  [4] Interactive SPI Terminal\r\n");
    puts_USART1("\r\n");
    puts_USART1("Enter selection (1-4): ");
}

int main(void)
{
    // Initialize peripherals
    Uart1_init();
    spi_master_init();

    // Configure Timer1 for timing measurements
    TCCR1B = (1 << CS10); // No prescaler

    // Configure status LEDs
    DDRC = 0xFF;
    PORTC = 0x00;

    // Send startup message
    _delay_ms(500);
    puts_USART1("\r\n\r\n*** SPI Master Basic Communication ***\r\n");
    puts_USART1("ATmega128 SPI Learning System\r\n");
    char buf[60];
    sprintf(buf, "Clock: %lu Hz, Default: F_CPU/16\r\n", F_CPU / 16);
    puts_USART1(buf);
    puts_USART1("Pins: PB0=SS, PB1=SCK, PB2=MOSI, PB3=MISO\r\n");

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
            demo1_basic_transmission();
            break;
        case '2':
            demo2_loopback_test();
            break;
        case '3':
            demo3_speed_comparison();
            break;
        case '4':
            demo4_interactive_terminal();
            break;
        default:
            puts_USART1("Invalid selection!\r\n");
            _delay_ms(1000);
            break;
        }

        // Ensure CS is high between demos
        CS_HIGH();
        _delay_ms(500);
    }

    return 0;
}
