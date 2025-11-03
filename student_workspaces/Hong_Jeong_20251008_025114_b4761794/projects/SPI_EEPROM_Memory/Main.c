/*
 * =============================================================================
 * SPI EEPROM MEMORY INTERFACE - EDUCATIONAL DEMONSTRATION
 * =============================================================================
 *
 * PROJECT: SPI_EEPROM_Memory
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Educational demonstration of external SPI EEPROM memory interfacing.
 * Students learn SPI communication protocols and non-volatile memory systems.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master SPI communication with external EEPROM (25LC256)
 * 2. Learn memory addressing and page-based operations
 * 3. Practice command sequences and status register handling
 * 4. Implement data persistence and storage systems
 * 5. Understand non-volatile memory characteristics
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - 25LC256 SPI EEPROM (32KB, 64-byte pages)
 * - SPI connections: PB0 (SS/CS), PB1 (SCK), PB2 (MOSI), PB3 (MISO)
 * - Pull-up resistors on WP and HOLD pins
 * - Status LEDs for operation indication
 * - Serial connection for debugging (9600 baud)
 *
 * 25LC256 SPECIFICATIONS:
 * - Memory size: 32KB (256K bits)
 * - Page size: 64 bytes
 * - Address range: 0x0000 to 0x7FFF
 * - Endurance: 1,000,000 write/erase cycles
 * - Data retention: 200+ years
 *
 * SPI COMMAND SET:
 * - 0x06: WREN (Write Enable)
 * - 0x04: WRDI (Write Disable)
 * - 0x05: RDSR (Read Status Register)
 * - 0x02: WRITE (Page Write)
 * - 0x03: READ (Sequential Read)
 *
 * LEARNING PROGRESSION:
 * - Demo 1: Basic SPI Communication
 * - Demo 2: Memory Read/Write Operations
 * - Demo 3: Page-Based Data Management
 * - Demo 4: Data Logging Applications
 *
 * =============================================================================
 */
*-0x01 : WRSR(Write Status Register) * -0x03 : READ(Read Data) * -0x02 : WRITE(Write Data) * /

#include "config.h"

// SPI Pin Definitions
#define SPI_SS PB0
#define SPI_SCK PB1
#define SPI_MOSI PB2
#define SPI_MISO PB3
#define SPI_DDR DDRB
#define SPI_PORT PORTB

// 25LC256 Commands
#define EEPROM_CMD_WREN 0x06  // Set Write Enable Latch
#define EEPROM_CMD_WRDI 0x04  // Reset Write Enable Latch
#define EEPROM_CMD_RDSR 0x05  // Read Status Register
#define EEPROM_CMD_WRSR 0x01  // Write Status Register
#define EEPROM_CMD_READ 0x03  // Read from Memory
#define EEPROM_CMD_WRITE 0x02 // Write to Memory

// Status Register Bits
#define EEPROM_SR_WIP 0x01 // Write In Progress
#define EEPROM_SR_WEL 0x02 // Write Enable Latch

// Memory specifications
#define EEPROM_SIZE 32768   // 32KB
#define EEPROM_PAGE_SIZE 64 // 64-byte pages

// Chip Select Macros
#define EEPROM_SELECT() (SPI_PORT &= ~(1 << SPI_SS))
#define EEPROM_DESELECT() (SPI_PORT |= (1 << SPI_SS))

    /*
     * Initialize SPI for EEPROM communication
     */
    void spi_init(void)
{
    // Set MOSI, SCK, SS as outputs
    SPI_DDR |= (1 << SPI_MOSI) | (1 << SPI_SCK) | (1 << SPI_SS);
    SPI_DDR &= ~(1 << SPI_MISO);

    // Deselect EEPROM
    EEPROM_DESELECT();

    // Enable SPI, Master mode, F_CPU/16, Mode 0
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

/*
 * Transfer one byte via SPI
 */
uint8_t spi_transfer(uint8_t data)
{
    SPDR = data;
    while (!(SPSR & (1 << SPIF)))
        ;
    return SPDR;
}

/*
 * Read EEPROM status register
 */
uint8_t eeprom_read_status(void)
{
    EEPROM_SELECT();
    spi_transfer(EEPROM_CMD_RDSR);
    uint8_t status = spi_transfer(0xFF);
    EEPROM_DESELECT();
    return status;
}

/*
 * Wait for write operation to complete
 */
void eeprom_wait_ready(void)
{
    while (eeprom_read_status() & EEPROM_SR_WIP)
    {
        _delay_us(10);
    }
}

/*
 * Enable write operations
 */
void eeprom_write_enable(void)
{
    EEPROM_SELECT();
    spi_transfer(EEPROM_CMD_WREN);
    EEPROM_DESELECT();
}

/*
 * Disable write operations
 */
void eeprom_write_disable(void)
{
    EEPROM_SELECT();
    spi_transfer(EEPROM_CMD_WRDI);
    EEPROM_DESELECT();
}

/*
 * Read single byte from EEPROM
 */
uint8_t eeprom_read_byte(uint16_t address)
{
    eeprom_wait_ready();

    EEPROM_SELECT();
    spi_transfer(EEPROM_CMD_READ);
    spi_transfer((address >> 8) & 0xFF); // Address high byte
    spi_transfer(address & 0xFF);        // Address low byte
    uint8_t data = spi_transfer(0xFF);
    EEPROM_DESELECT();

    return data;
}

/*
 * Write single byte to EEPROM
 */
void eeprom_write_byte(uint16_t address, uint8_t data)
{
    eeprom_wait_ready();
    eeprom_write_enable();

    EEPROM_SELECT();
    spi_transfer(EEPROM_CMD_WRITE);
    spi_transfer((address >> 8) & 0xFF);
    spi_transfer(address & 0xFF);
    spi_transfer(data);
    EEPROM_DESELECT();

    eeprom_wait_ready();
}

/*
 * Read multiple bytes sequentially
 */
void eeprom_read_bytes(uint16_t address, uint8_t *buffer, uint16_t length)
{
    eeprom_wait_ready();

    EEPROM_SELECT();
    spi_transfer(EEPROM_CMD_READ);
    spi_transfer((address >> 8) & 0xFF);
    spi_transfer(address & 0xFF);

    for (uint16_t i = 0; i < length; i++)
    {
        buffer[i] = spi_transfer(0xFF);
    }

    EEPROM_DESELECT();
}

/*
 * Write page (up to 64 bytes within same page)
 */
void eeprom_write_page(uint16_t address, uint8_t *data, uint8_t length)
{
    if (length > EEPROM_PAGE_SIZE)
        length = EEPROM_PAGE_SIZE;

    eeprom_wait_ready();
    eeprom_write_enable();

    EEPROM_SELECT();
    spi_transfer(EEPROM_CMD_WRITE);
    spi_transfer((address >> 8) & 0xFF);
    spi_transfer(address & 0xFF);

    for (uint8_t i = 0; i < length; i++)
    {
        spi_transfer(data[i]);
    }

    EEPROM_DESELECT();
    eeprom_wait_ready();
}

/* ========================================================================
 * DEMO 1: Basic Read/Write Test
 * ======================================================================== */
void demo1_basic_readwrite(void)
{
    puts_USART1("\r\n=== DEMO 1: Basic Read/Write Test ===\r\n");
    puts_USART1("Writing and reading single bytes\r\n\r\n");

    uint16_t test_address = 0x0100;

    // Write test data
    puts_USART1("Writing test bytes...\r\n");
    for (uint8_t i = 0; i < 10; i++)
    {
        uint8_t value = 0xA0 + i;
        eeprom_write_byte(test_address + i, value);

        char buf[50];
        sprintf(buf, "  Address 0x%04X = 0x%02X\r\n", test_address + i, value);
        puts_USART1(buf);
    }

    puts_USART1("\r\nReading back data...\r\n");
    uint8_t errors = 0;
    for (uint8_t i = 0; i < 10; i++)
    {
        uint8_t expected = 0xA0 + i;
        uint8_t actual = eeprom_read_byte(test_address + i);

        char buf[70];
        if (actual == expected)
        {
            sprintf(buf, "  Address 0x%04X = 0x%02X ✓\r\n",
                    test_address + i, actual);
        }
        else
        {
            sprintf(buf, "  Address 0x%04X = 0x%02X (expected 0x%02X) ✗\r\n",
                    test_address + i, actual, expected);
            errors++;
        }
        puts_USART1(buf);
    }

    if (errors == 0)
    {
        puts_USART1("\r\n✓ All tests passed!\r\n");
        PORTC = 0x0F; // Success indicator
    }
    else
    {
        char msg[40];
        sprintf(msg, "\r\n✗ %u errors detected!\r\n", errors);
        puts_USART1(msg);
        PORTC = 0xF0; // Error indicator
    }

    puts_USART1("\r\nPress any key to continue...");
    getch_USART1();
}

/* ========================================================================
 * DEMO 2: Page Write Test
 * ======================================================================== */
void demo2_page_write(void)
{
    puts_USART1("\r\n=== DEMO 2: Page Write Test ===\r\n");
    puts_USART1("Writing full 64-byte page\r\n\r\n");

    uint16_t page_address = 0x0200; // Start of page
    uint8_t page_data[EEPROM_PAGE_SIZE];

    // Prepare test pattern
    puts_USART1("Preparing test pattern...\r\n");
    for (uint8_t i = 0; i < EEPROM_PAGE_SIZE; i++)
    {
        page_data[i] = i;
    }

    // Write entire page
    puts_USART1("Writing page to address 0x0200...\r\n");
    uint16_t start_time = TCNT1;
    eeprom_write_page(page_address, page_data, EEPROM_PAGE_SIZE);
    uint16_t write_time = TCNT1 - start_time;

    char buf[60];
    sprintf(buf, "Page write completed in %u timer ticks\r\n", write_time);
    puts_USART1(buf);

    // Read back and verify
    puts_USART1("\r\nVerifying data...\r\n");
    uint8_t read_buffer[EEPROM_PAGE_SIZE];
    eeprom_read_bytes(page_address, read_buffer, EEPROM_PAGE_SIZE);

    uint8_t errors = 0;
    for (uint8_t i = 0; i < EEPROM_PAGE_SIZE; i++)
    {
        if (read_buffer[i] != page_data[i])
        {
            errors++;
            sprintf(buf, "  Byte %u: Got 0x%02X, Expected 0x%02X\r\n",
                    i, read_buffer[i], page_data[i]);
            puts_USART1(buf);
        }
    }

    if (errors == 0)
    {
        puts_USART1("✓ Page write successful! All 64 bytes verified.\r\n");
    }
    else
    {
        sprintf(buf, "✗ Page write failed! %u byte errors.\r\n", errors);
        puts_USART1(buf);
    }

    puts_USART1("\r\nPress any key to continue...");
    getch_USART1();
}

/* ========================================================================
 * DEMO 3: Sequential Read Performance
 * ======================================================================== */
void demo3_sequential_read(void)
{
    puts_USART1("\r\n=== DEMO 3: Sequential Read Performance ===\r\n");
    puts_USART1("Reading 1KB of data sequentially\r\n\r\n");

    uint16_t start_address = 0x0000;
    uint16_t bytes_to_read = 1024;
    uint8_t buffer[128];

    puts_USART1("Reading in 128-byte chunks...\r\n");

    uint16_t start_time = TCNT1;

    for (uint16_t offset = 0; offset < bytes_to_read; offset += 128)
    {
        eeprom_read_bytes(start_address + offset, buffer, 128);

        char msg[50];
        sprintf(msg, "  Read bytes %4u - %4u\r\n", offset, offset + 127);
        puts_USART1(msg);
    }

    uint16_t total_time = TCNT1 - start_time;

    char buf[80];
    sprintf(buf, "\r\nTotal read time: %u timer ticks\r\n", total_time);
    puts_USART1(buf);

    uint32_t bytes_per_second = ((uint32_t)bytes_to_read * F_CPU) / total_time;
    sprintf(buf, "Transfer rate: ~%lu bytes/second\r\n", bytes_per_second);
    puts_USART1(buf);

    puts_USART1("\r\nPress any key to continue...");
    getch_USART1();
}

/* ========================================================================
 * DEMO 4: Memory Dump Utility
 * ======================================================================== */
void demo4_memory_dump(void)
{
    puts_USART1("\r\n=== DEMO 4: Memory Dump Utility ===\r\n");
    puts_USART1("Hex dump of EEPROM contents\r\n");
    puts_USART1("Enter start address in hex (e.g., 0100): ");

    // Simple hex input (4 digits)
    char addr_str[5];
    for (uint8_t i = 0; i < 4; i++)
    {
        addr_str[i] = getch_USART1();
        putch_USART1(addr_str[i]);
    }
    addr_str[4] = '\0';

    // Convert hex string to number
    uint16_t address = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        address <<= 4;
        char c = addr_str[i];
        if (c >= '0' && c <= '9')
            address |= (c - '0');
        else if (c >= 'A' && c <= 'F')
            address |= (c - 'A' + 10);
        else if (c >= 'a' && c <= 'f')
            address |= (c - 'a' + 10);
    }

    puts_USART1("\r\n\r\nDumping 256 bytes starting from 0x");
    char msg[20];
    sprintf(msg, "%04X", address);
    puts_USART1(msg);
    puts_USART1(":\r\n\r\n");

    // Dump 16 lines of 16 bytes each
    for (uint8_t line = 0; line < 16; line++)
    {
        uint16_t line_addr = address + (line * 16);

        // Print address
        sprintf(msg, "%04X: ", line_addr);
        puts_USART1(msg);

        // Read and print 16 bytes in hex
        uint8_t line_data[16];
        eeprom_read_bytes(line_addr, line_data, 16);

        for (uint8_t i = 0; i < 16; i++)
        {
            sprintf(msg, "%02X ", line_data[i]);
            puts_USART1(msg);
        }

        // Print ASCII representation
        puts_USART1(" |");
        for (uint8_t i = 0; i < 16; i++)
        {
            char c = line_data[i];
            if (c >= 32 && c <= 126)
            {
                putch_USART1(c);
            }
            else
            {
                putch_USART1('.');
            }
        }
        puts_USART1("|\r\n");
    }

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
    puts_USART1("║   SPI EEPROM (25LC256) - ATmega128    ║\r\n");
    puts_USART1("╚════════════════════════════════════════╝\r\n");
    puts_USART1("\r\n");
    puts_USART1("Select Demo:\r\n");
    puts_USART1("  [1] Basic Read/Write Test\r\n");
    puts_USART1("  [2] Page Write Test (64 bytes)\r\n");
    puts_USART1("  [3] Sequential Read Performance\r\n");
    puts_USART1("  [4] Memory Dump Utility\r\n");
    puts_USART1("\r\n");
    puts_USART1("Enter selection (1-4): ");
}

int main(void)
{
    // Initialize peripherals
    Uart1_init();
    spi_init();

    // Configure Timer1 for performance measurement
    TCCR1B = (1 << CS10); // No prescaler

    // Configure status LEDs
    DDRC = 0xFF;
    PORTC = 0x00;

    // Send startup message
    _delay_ms(500);
    puts_USART1("\r\n\r\n*** SPI EEPROM Memory System ***\r\n");
    puts_USART1("25LC256 External EEPROM (32KB)\r\n");
    puts_USART1("Page size: 64 bytes\r\n");

    // Read and display status register
    uint8_t status = eeprom_read_status();
    char buf[60];
    sprintf(buf, "Status Register: 0x%02X\r\n", status);
    puts_USART1(buf);

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
            demo1_basic_readwrite();
            break;
        case '2':
            demo2_page_write();
            break;
        case '3':
            demo3_sequential_read();
            break;
        case '4':
            demo4_memory_dump();
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
