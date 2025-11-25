/*
 * SPI Multi-Device Bus
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Manage multiple SPI devices on shared bus
 * - Implement proper chip select sequencing
 * - Handle devices with different SPI modes/speeds
 * - Practice bus arbitration and device addressing
 *
 * HARDWARE SETUP:
 * - Shared SPI bus: SCK (PB1), MOSI (PB2), MISO (PB3)
 * - Device 1 CS: PB0 (e.g., EEPROM 25LC256)
 * - Device 2 CS: PB4 (e.g., SD Card or second EEPROM)
 * - Device 3 CS: PB5 (e.g., SPI DAC or ADC)
 * - UART for monitoring and control
 *
 * SPI BUS CONCEPTS:
 * - Shared Bus: Multiple devices share MOSI, MISO, SCK
 * - Individual CS: Each device has unique chip select
 * - Bus Contention: Only one device active at a time
 * - Mode Switching: Reconfigure SPI for device requirements
 */

#include "config.h"

// SPI Pin Definitions
#define SPI_SCK PB1
#define SPI_MOSI PB2
#define SPI_MISO PB3
#define SPI_DDR DDRB
#define SPI_PORT PORTB

// Chip Select pins for 3 devices
#define CS1_PIN PB0 // Device 1
#define CS2_PIN PB4 // Device 2
#define CS3_PIN PB5 // Device 3

// Device selection macros
#define SELECT_DEVICE1() (SPI_PORT &= ~(1 << CS1_PIN))
#define DESELECT_DEVICE1() (SPI_PORT |= (1 << CS1_PIN))
#define SELECT_DEVICE2() (SPI_PORT &= ~(1 << CS2_PIN))
#define DESELECT_DEVICE2() (SPI_PORT |= (1 << CS2_PIN))
#define SELECT_DEVICE3() (SPI_PORT &= ~(1 << CS3_PIN))
#define DESELECT_DEVICE3() (SPI_PORT |= (1 << CS3_PIN))
#define DESELECT_ALL() (SPI_PORT |= (1 << CS1_PIN) | (1 << CS2_PIN) | (1 << CS3_PIN))

// Device types (for demonstration)
typedef enum
{
    DEVICE_TYPE_EEPROM = 1,
    DEVICE_TYPE_DAC = 2,
    DEVICE_TYPE_ADC = 3
} device_type_t;

// Device configuration structure
typedef struct
{
    uint8_t cs_pin;
    device_type_t type;
    uint8_t spi_mode;  // 0-3 (CPOL, CPHA combinations)
    uint8_t spi_speed; // 0-3 (prescaler setting)
    char name[20];
} spi_device_t;

// Device database
spi_device_t devices[3] = {
    {CS1_PIN, DEVICE_TYPE_EEPROM, 0, 1, "EEPROM 25LC256"},
    {CS2_PIN, DEVICE_TYPE_DAC, 0, 0, "SPI DAC MCP4921"},
    {CS3_PIN, DEVICE_TYPE_ADC, 0, 1, "SPI ADC MCP3008"}};

uint8_t current_device = 0xFF; // No device selected

/*
 * Initialize SPI bus and all chip selects
 */
void spi_bus_init(void)
{
    // Set MOSI, SCK, and all CS pins as outputs
    SPI_DDR |= (1 << SPI_MOSI) | (1 << SPI_SCK) |
               (1 << CS1_PIN) | (1 << CS2_PIN) | (1 << CS3_PIN);
    SPI_DDR &= ~(1 << SPI_MISO);

    // Deselect all devices
    DESELECT_ALL();

    // Enable SPI, Master mode, default F_CPU/16
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
 * Configure SPI for specific mode and speed
 */
void spi_configure(uint8_t mode, uint8_t speed)
{
    // Mode: CPOL and CPHA bits
    // Mode 0: CPOL=0, CPHA=0
    // Mode 1: CPOL=0, CPHA=1
    // Mode 2: CPOL=1, CPHA=0
    // Mode 3: CPOL=1, CPHA=1
    uint8_t spcr = (1 << SPE) | (1 << MSTR);

    if (mode & 0x02)
        spcr |= (1 << CPOL);
    if (mode & 0x01)
        spcr |= (1 << CPHA);

    // Speed setting
    switch (speed)
    {
    case 0: // F_CPU/4 (with SPI2X)
        SPSR |= (1 << SPI2X);
        break;
    case 1: // F_CPU/16
        spcr |= (1 << SPR0);
        break;
    case 2: // F_CPU/64
        spcr |= (1 << SPR1);
        break;
    case 3: // F_CPU/128
        spcr |= (1 << SPR1) | (1 << SPR0);
        break;
    }

    SPCR = spcr;
}

/*
 * Select a device by index (0-2)
 */
void select_device(uint8_t device_index)
{
    if (device_index >= 3)
        return;

    // Deselect current device
    if (current_device < 3)
    {
        SPI_PORT |= (1 << devices[current_device].cs_pin);
    }

    // Configure SPI for new device
    spi_configure(devices[device_index].spi_mode,
                  devices[device_index].spi_speed);

    // Small delay for SPI reconfiguration
    _delay_us(10);

    // Select new device
    SPI_PORT &= ~(1 << devices[device_index].cs_pin);
    current_device = device_index;

    // Chip select setup time
    _delay_us(1);
}

/*
 * Deselect current device
 */
void deselect_device(void)
{
    if (current_device < 3)
    {
        SPI_PORT |= (1 << devices[current_device].cs_pin);
        current_device = 0xFF;
    }
    _delay_us(1);
}

/* ========================================================================
 * DEMO 1: Device Enumeration and Info
 * ======================================================================== */
void demo1_device_info(void)
{
    puts_USART1("\r\n=== DEMO 1: Device Information ===\r\n");
    puts_USART1("Scanning SPI bus for devices...\r\n\r\n");

    for (uint8_t i = 0; i < 3; i++)
    {
        char buf[100];
        sprintf(buf, "Device %u: %s\r\n", i + 1, devices[i].name);
        puts_USART1(buf);

        sprintf(buf, "  CS Pin: PB%u\r\n", devices[i].cs_pin);
        puts_USART1(buf);

        sprintf(buf, "  SPI Mode: %u  Speed: ", devices[i].spi_mode);
        puts_USART1(buf);

        switch (devices[i].spi_speed)
        {
        case 0:
            puts_USART1("F_CPU/4\r\n");
            break;
        case 1:
            puts_USART1("F_CPU/16\r\n");
            break;
        case 2:
            puts_USART1("F_CPU/64\r\n");
            break;
        case 3:
            puts_USART1("F_CPU/128\r\n");
            break;
        }

        puts_USART1("\r\n");
    }

    puts_USART1("Press any key to continue...");
    getch_USART1();
}

/* ========================================================================
 * DEMO 2: Sequential Device Access
 * ======================================================================== */
void demo2_sequential_access(void)
{
    puts_USART1("\r\n=== DEMO 2: Sequential Device Access ===\r\n");
    puts_USART1("Sending data to each device in sequence\r\n");
    puts_USART1("Press any key to stop\r\n\r\n");

    uint8_t counter = 0;

    while (1)
    {
        for (uint8_t dev = 0; dev < 3; dev++)
        {
            // Select device
            select_device(dev);

            // Send test pattern
            uint8_t data_sent = 0x80 + counter;
            uint8_t data_received = spi_transfer(data_sent);

            // Deselect device
            deselect_device();

            // Display result
            char buf[80];
            sprintf(buf, "Device %u: Sent 0x%02X, Received 0x%02X\r\n",
                    dev + 1, data_sent, data_received);
            puts_USART1(buf);

            // Visual indicator
            PORTC = (1 << dev);

            _delay_ms(300);

            // Check for quit
            if (UCSR1A & (1 << RXC1))
            {
                getch_USART1();
                DESELECT_ALL();
                puts_USART1("\r\nSequential access stopped.\r\n");
                return;
            }
        }

        counter++;
        puts_USART1("\r\n");
    }
}

/* ========================================================================
 * DEMO 3: Rapid Device Switching Test
 * ======================================================================== */
void demo3_rapid_switching(void)
{
    puts_USART1("\r\n=== DEMO 3: Rapid Device Switching ===\r\n");
    puts_USART1("Testing fast switching between devices\r\n");
    puts_USART1("Measuring switching overhead\r\n\r\n");

    uint16_t num_switches = 1000;

    // Measure time for switching and data transfer
    uint16_t start_time = TCNT1;

    for (uint16_t i = 0; i < num_switches; i++)
    {
        uint8_t dev = i % 3; // Cycle through devices

        select_device(dev);
        spi_transfer(0xAA); // Send dummy byte
        deselect_device();
    }

    uint16_t elapsed_time = TCNT1 - start_time;

    char buf[100];
    sprintf(buf, "Completed %u device switches\r\n", num_switches);
    puts_USART1(buf);

    sprintf(buf, "Total time: %u timer ticks\r\n", elapsed_time);
    puts_USART1(buf);

    uint32_t avg_time = ((uint32_t)elapsed_time * 1000) / num_switches;
    sprintf(buf, "Average per switch: %lu.%03lu ms\r\n",
            avg_time / 1000, avg_time % 1000);
    puts_USART1(buf);

    puts_USART1("\r\nPress any key to continue...");
    getch_USART1();
}

/* ========================================================================
 * DEMO 4: Interactive Device Control
 * ======================================================================== */
void demo4_interactive_control(void)
{
    puts_USART1("\r\n=== DEMO 4: Interactive Device Control ===\r\n");
    puts_USART1("Commands:\r\n");
    puts_USART1("  1-3: Select device\r\n");
    puts_USART1("  s[XX]: Send hex byte to selected device\r\n");
    puts_USART1("  d: Deselect all\r\n");
    puts_USART1("  i: Show device info\r\n");
    puts_USART1("  q: Return to menu\r\n\r\n");

    char input_buffer[10];
    uint8_t buf_index = 0;

    while (1)
    {
        if (UCSR1A & (1 << RXC1))
        {
            char c = getch_USART1();
            putch_USART1(c);

            if (c == '\r' || c == '\n')
            {
                input_buffer[buf_index] = '\0';
                puts_USART1("\r\n");

                if (buf_index > 0)
                {
                    if (input_buffer[0] >= '1' && input_buffer[0] <= '3')
                    {
                        uint8_t dev = input_buffer[0] - '1';
                        select_device(dev);
                        char msg[50];
                        sprintf(msg, "Selected: %s\r\n", devices[dev].name);
                        puts_USART1(msg);
                        PORTC = (1 << dev);
                    }
                    else if (input_buffer[0] == 's' || input_buffer[0] == 'S')
                    {
                        if (current_device < 3 && buf_index >= 3)
                        {
                            // Parse hex byte
                            uint8_t value = 0;
                            for (uint8_t i = 1; i <= 2; i++)
                            {
                                char hex = input_buffer[i];
                                value <<= 4;
                                if (hex >= '0' && hex <= '9')
                                    value |= (hex - '0');
                                else if (hex >= 'A' && hex <= 'F')
                                    value |= (hex - 'A' + 10);
                                else if (hex >= 'a' && hex <= 'f')
                                    value |= (hex - 'a' + 10);
                            }

                            uint8_t received = spi_transfer(value);
                            char msg[80];
                            sprintf(msg, "→ Sent: 0x%02X  ← Received: 0x%02X\r\n",
                                    value, received);
                            puts_USART1(msg);
                        }
                        else
                        {
                            puts_USART1("No device selected!\r\n");
                        }
                    }
                    else if (input_buffer[0] == 'd' || input_buffer[0] == 'D')
                    {
                        deselect_device();
                        puts_USART1("All devices deselected\r\n");
                        PORTC = 0x00;
                    }
                    else if (input_buffer[0] == 'i' || input_buffer[0] == 'I')
                    {
                        if (current_device < 3)
                        {
                            char msg[60];
                            sprintf(msg, "Current: %s (Device %u)\r\n",
                                    devices[current_device].name, current_device + 1);
                            puts_USART1(msg);
                        }
                        else
                        {
                            puts_USART1("No device selected\r\n");
                        }
                    }
                    else if (input_buffer[0] == 'q' || input_buffer[0] == 'Q')
                    {
                        DESELECT_ALL();
                        return;
                    }
                }
                buf_index = 0;
            }
            else if (c == 8 || c == 127)
            {
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
    puts_USART1("║   SPI MULTI-DEVICE BUS - ATmega128    ║\r\n");
    puts_USART1("╚════════════════════════════════════════╝\r\n");
    puts_USART1("\r\n");
    puts_USART1("Select Demo:\r\n");
    puts_USART1("  [1] Device Information\r\n");
    puts_USART1("  [2] Sequential Device Access\r\n");
    puts_USART1("  [3] Rapid Switching Test\r\n");
    puts_USART1("  [4] Interactive Device Control\r\n");
    puts_USART1("\r\n");
    puts_USART1("Enter selection (1-4): ");
}

int main(void)
{
    // Initialize peripherals
    Uart1_init();
    spi_bus_init();

    // Configure Timer1 for timing
    TCCR1B = (1 << CS10);

    // Configure status LEDs
    DDRC = 0xFF;
    PORTC = 0x00;

    // Send startup message
    _delay_ms(500);
    puts_USART1("\r\n\r\n*** SPI Multi-Device Bus System ***\r\n");
    puts_USART1("ATmega128 SPI Bus Manager\r\n");
    puts_USART1("Supporting 3 SPI devices on shared bus\r\n");

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
            demo1_device_info();
            break;
        case '2':
            demo2_sequential_access();
            break;
        case '3':
            demo3_rapid_switching();
            break;
        case '4':
            demo4_interactive_control();
            break;
        default:
            puts_USART1("Invalid selection!\r\n");
            _delay_ms(1000);
            break;
        }

        // Ensure all devices deselected between demos
        DESELECT_ALL();
        PORTC = 0x00;
        _delay_ms(500);
    }

    return 0;
}
