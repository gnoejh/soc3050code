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
 * DOCUMENTATION REFERENCE:
 * ATmega128 Datasheet: https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/2467S.pdf
 * - SPI section (pages 150-161)
 * - SPI registers (pages 158-161)
 * - SPI timing diagrams (page 153)
 *
 * =============================================================================
 * SPI CONTROL REGISTERS - DETAILED REFERENCE FOR STUDENTS
 * =============================================================================
 *
 * REGISTER 1: SPCR (SPI Control Register) - PRIMARY CONFIGURATION
 *
 *    Bit:   7      6      5      4      3      2      1      0
 *    Name:  SPIE   SPE   DORD   MSTR  CPOL   CPHA   SPR1   SPR0
 *
 * SPIE (bit 7): SPI Interrupt Enable
 *               1 = Enable interrupt when SPIF flag is set
 *               0 = Polling mode (check SPIF manually in SPSR)
 *               Usage with interrupt: SPCR |= (1<<SPIE); sei();
 *                                     ISR(SPI_STC_vect) { data = SPDR; }
 *
 * SPE (bit 6): SPI Enable - CRITICAL: MUST BE SET
 *              1 = Enable SPI interface (activates MOSI, MISO, SCK pins)
 *              0 = Disable SPI (pins return to PORT control)
 *              Always set for SPI operations
 *
 * DORD (bit 5): Data Order
 *               0 = MSB transmitted first (standard, most common)
 *               1 = LSB transmitted first (rare, device-specific)
 *               Most SPI devices use MSB-first (DORD=0)
 *
 * MSTR (bit 4): Master/Slave Select - DEFINES ROLE
 *               1 = Master mode (ATmega128 generates SCK clock)
 *               0 = Slave mode (external device provides SCK)
 *               Master mode is typical for microcontroller applications
 *
 * CPOL (bit 3): Clock Polarity - SCK IDLE STATE
 *               0 = SCK low when idle (rising edge is leading)
 *               1 = SCK high when idle (falling edge is leading)
 *               Must match slave device specification
 *               Common: CPOL=0 for most devices
 *
 * CPHA (bit 2): Clock Phase - DATA SAMPLING EDGE
 *               0 = Sample on leading edge, setup on trailing edge
 *               1 = Setup on leading edge, sample on trailing edge
 *               Must match slave device specification
 *               Common: CPHA=0 for most devices
 *
 * SPR1:0 (bits 1-0): SPI Clock Rate Select (with SPI2X in SPSR)
 *                    Combined with SPI2X for 7 speed options:
 *
 *                    SPI2X SPR1 SPR0 | Divisor | Freq @ 16MHz | Freq @ 8MHz
 *                    ---------------------------------------------------------
 *                      0     0    0  |    4    |   4 MHz      | 2 MHz
 *                      0     0    1  |   16    |   1 MHz      | 500 kHz
 *                      0     1    0  |   64    | 250 kHz      | 125 kHz
 *                      0     1    1  |  128    | 125 kHz      | 62.5 kHz
 *                      1     0    0  |    2    |   8 MHz      | 4 MHz (FASTEST)
 *                      1     0    1  |    8    |   2 MHz      | 1 MHz
 *                      1     1    0  |   32    | 500 kHz      | 250 kHz
 *                      1     1    1  |   64    | 250 kHz      | 125 kHz
 *
 * REGISTER 2: SPSR (SPI Status Register) - STATUS AND SPEED
 *
 *    Bit:   7      6      5      4      3      2      1      0
 *    Name:  SPIF  WCOL   -      -      -      -      -     SPI2X
 *
 * SPIF (bit 7): SPI Interrupt Flag - TRANSFER COMPLETE INDICATOR
 *               Set by hardware when transfer completes
 *               Cleared automatically by: reading SPSR then accessing SPDR
 *               Polling: while(!(SPSR & (1<<SPIF)));
 *               CRITICAL: Always check SPIF before reading received data
 *
 * WCOL (bit 6): Write Collision Flag
 *               Set if SPDR written during transfer (illegal operation)
 *               Clear by reading SPSR then accessing SPDR
 *               Indicates programming error if set
 *
 * SPI2X (bit 0): Double SPI Speed
 *                1 = Double the SPI clock rate (use with SPR1:0)
 *                0 = Normal SPI speed
 *                See SPR1:0 table above for combined speeds
 *
 * REGISTER 3: SPDR (SPI Data Register) - BIDIRECTIONAL DATA BUFFER
 *
 * SPDR serves dual purpose:
 * - Write to SPDR: Initiates transmission and shifts out byte
 * - Read from SPDR: Returns last received byte
 *
 * SPI is FULL-DUPLEX: Sending and receiving happen simultaneously
 * - Writing to SPDR starts SCK clock and shifts out 8 bits
 * - While shifting out, 8 bits are simultaneously shifted in from MISO
 * - After 8 clock cycles, SPIF=1 and received byte is in SPDR
 *
 * TYPICAL TRANSFER SEQUENCE:
 *
 *   uint8_t spi_transfer(uint8_t data) {
 *       SPDR = data;                    // Start transmission
 *       while(!(SPSR & (1<<SPIF)));     // Wait for completion
 *       return SPDR;                    // Return received byte
 *   }
 *
 * TIMING:
 * @ F_CPU=16MHz, SPR1:0=00, SPI2X=1 (fastest: F_CPU/2 = 8MHz):
 *   - 8 bits × 125ns/bit = 1µs per byte
 * @ F_CPU=16MHz, SPR1:0=01, SPI2X=0 (F_CPU/16 = 1MHz):
 *   - 8 bits × 1µs/bit = 8µs per byte
 *
 * MASTER MODE INITIALIZATION (Most Common):
 *
 *   void spi_master_init(void) {
 *       // Configure pins
 *       DDRB |= (1<<PB0) | (1<<PB1) | (1<<PB2);  // SS, SCK, MOSI as outputs
 *       DDRB &= ~(1<<PB3);                       // MISO as input
 *       PORTB |= (1<<PB0);                       // SS high (deselect)
 *
 *       // Configure SPI: Master, MSB first, CPOL=0, CPHA=0, F_CPU/16
 *       SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0);
 *
 *       // Optional: Double speed for F_CPU/8
 *       SPSR |= (1<<SPI2X);
 *   }
 *
 * SPI MODE CONFIGURATIONS (CPOL:CPHA combinations):
 *
 *   Mode 0: CPOL=0, CPHA=0 - Most common (idle low, sample leading edge)
 *           SPCR = (1<<SPE) | (1<<MSTR);
 *
 *   Mode 1: CPOL=0, CPHA=1 - Idle low, sample trailing edge
 *           SPCR = (1<<SPE) | (1<<MSTR) | (1<<CPHA);
 *
 *   Mode 2: CPOL=1, CPHA=0 - Idle high, sample leading edge
 *           SPCR = (1<<SPE) | (1<<MSTR) | (1<<CPOL);
 *
 *   Mode 3: CPOL=1, CPHA=1 - Idle high, sample trailing edge
 *           SPCR = (1<<SPE) | (1<<MSTR) | (1<<CPOL) | (1<<CPHA);
 *
 * MULTI-DEVICE OPERATION:
 * - Use separate SS (Slave Select) pins for each device
 * - Only one SS should be LOW at a time
 * - Master controls which device is active
 *
 *   Example with 2 devices:
 *     #define SS1 PB0
 *     #define SS2 PB4
 *
 *     // Select device 1
 *     PORTB &= ~(1<<SS1);  // SS1 low
 *     PORTB |= (1<<SS2);   // SS2 high
 *     data = spi_transfer(0x42);
 *     PORTB |= (1<<SS1);   // Deselect
 *
 * PIN CONNECTIONS (ATmega128):
 *   PB0 = SS   (Slave Select - output in master mode, must be high or output)
 *   PB1 = SCK  (Serial Clock - output in master mode)
 *   PB2 = MOSI (Master Out Slave In - output in master mode)
 *   PB3 = MISO (Master In Slave Out - input in master mode)
 *
 * CRITICAL NOTES:
 * 1. SS pin MUST be output or held high in master mode to prevent mode switch
 * 2. SPI is synchronous - no start/stop bits, pure clock-driven
 * 3. Full-duplex - always sends and receives simultaneously
 * 4. Check device datasheet for correct CPOL/CPHA mode
 * 5. Maximum speed limited by slave device and wiring capacitance
 *
 * =============================================================================
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
