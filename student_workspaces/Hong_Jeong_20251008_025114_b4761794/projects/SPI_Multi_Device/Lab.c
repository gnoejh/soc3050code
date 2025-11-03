/*
 * =============================================================================
 * SPI MULTI-DEVICE - HANDS-ON LAB EXERCISES
 * =============================================================================
 * ATmega128 Educational Framework - Lab Session
 *
 * OBJECTIVE: Master SPI communication with multiple slave devices
 * DURATION: 80 minutes
 * DIFFICULTY: Advanced
 *
 * STUDENTS WILL:
 * - Manage multiple SPI devices on a shared bus
 * - Implement device selection and isolation
 * - Handle different SPI modes and timing requirements
 * - Create device-specific communication protocols
 * - Optimize bus utilization and performance
 *
 * HARDWARE REQUIRED:
 * - ATmega128 board
 * - Multiple SPI devices (EEPROM, ADC, DAC, Display, etc.)
 * - Example devices: 25LC256 EEPROM, MCP3008 ADC, MCP4921 DAC
 * - Individual chip select (CS) lines for each device
 * - Shared MOSI, MISO, SCK lines with proper buffering
 * - Logic analyzer for protocol verification (optional)
 * - Status LEDs for device activity indication
 *
 * SPI MULTI-DEVICE CONCEPTS:
 * - Chip select management and timing
 * - SPI mode compatibility (CPOL/CPHA)
 * - Clock frequency optimization per device
 * - Bus arbitration and device isolation
 * - Daisy chaining vs. independent selection
 *
 * LAB STRUCTURE:
 * - Exercise 1: Multi-device setup and selection (25 min)
 * - Exercise 2: Different SPI modes and timing (20 min)
 * - Exercise 3: Device coordination and data flow (20 min)
 * - Exercise 4: Advanced multi-device applications (15 min)
 *
 * =============================================================================
 */

#include "config.h"

// SPI Multi-Device Configuration
#define MAX_SPI_DEVICES 8

// Device-specific chip select pins
#define CS_EEPROM_PIN 4  // PB4 - EEPROM 25LC256
#define CS_ADC_PIN 5     // PB5 - ADC MCP3008
#define CS_DAC_PIN 6     // PB6 - DAC MCP4921
#define CS_DISPLAY_PIN 7 // PB7 - Display controller
#define CS_SPARE1_PIN 0  // PC0 - Spare device 1
#define CS_SPARE2_PIN 1  // PC1 - Spare device 2
#define CS_SPARE3_PIN 2  // PC2 - Spare device 3
#define CS_SPARE4_PIN 3  // PC3 - Spare device 4

// Device identification
typedef enum
{
    DEVICE_EEPROM = 0,
    DEVICE_ADC,
    DEVICE_DAC,
    DEVICE_DISPLAY,
    DEVICE_SPARE1,
    DEVICE_SPARE2,
    DEVICE_SPARE3,
    DEVICE_SPARE4
} spi_device_t;

// SPI device configuration structure
typedef struct
{
    spi_device_t device_id;
    volatile uint8_t *cs_port;
    uint8_t cs_pin;
    uint8_t spi_mode;           // 0-3 (CPOL/CPHA combinations)
    uint8_t clock_prescaler;    // SPI clock prescaler
    uint16_t max_frequency_khz; // Maximum supported frequency
    char device_name[16];
    uint8_t is_active;
} spi_device_config_t;

// Communication activity indicators
#define ACTIVITY_LED_PIN 2 // PD2 - General activity
#define ERROR_LED_PIN 3    // PD3 - Error indicator

// Lab session variables
uint16_t lab_score = 0;
uint32_t total_transactions = 0;
uint16_t device_switches = 0;
uint16_t successful_operations = 0;
uint16_t spi_errors = 0;

// Device configuration table
spi_device_config_t spi_devices[MAX_SPI_DEVICES] = {
    {DEVICE_EEPROM, &PORTB, CS_EEPROM_PIN, 0, 6, 10000, "EEPROM_25LC256", 1},
    {DEVICE_ADC, &PORTB, CS_ADC_PIN, 0, 4, 20000, "ADC_MCP3008", 1},
    {DEVICE_DAC, &PORTB, CS_DAC_PIN, 0, 4, 20000, "DAC_MCP4921", 1},
    {DEVICE_DISPLAY, &PORTB, CS_DISPLAY_PIN, 3, 6, 8000, "Display_Ctrl", 1},
    {DEVICE_SPARE1, &PORTC, CS_SPARE1_PIN, 0, 6, 5000, "Spare_Device1", 0},
    {DEVICE_SPARE2, &PORTC, CS_SPARE2_PIN, 1, 6, 5000, "Spare_Device2", 0},
    {DEVICE_SPARE3, &PORTC, CS_SPARE3_PIN, 2, 6, 5000, "Spare_Device3", 0},
    {DEVICE_SPARE4, &PORTC, CS_SPARE4_PIN, 3, 6, 5000, "Spare_Device4", 0}};

uint8_t active_device_count = 0;
spi_device_t current_device = DEVICE_EEPROM;

/*
 * =============================================================================
 * SPI MULTI-DEVICE MANAGEMENT FUNCTIONS
 * =============================================================================
 */

void spi_multi_init(void)
{
    // Configure SPI pins: MOSI, SCK, SS as outputs, MISO as input
    DDRB |= (1 << PB2) | (1 << PB1) | (1 << PB0); // MOSI, SCK, SS
    DDRB &= ~(1 << PB3);                          // MISO as input
    PORTB |= (1 << PB3);                          // MISO pull-up

    // Configure all chip select pins as outputs (initially high)
    DDRB |= (1 << CS_EEPROM_PIN) | (1 << CS_ADC_PIN) | (1 << CS_DAC_PIN) | (1 << CS_DISPLAY_PIN);
    DDRC |= (1 << CS_SPARE1_PIN) | (1 << CS_SPARE2_PIN) | (1 << CS_SPARE3_PIN) | (1 << CS_SPARE4_PIN);

    // Set all CS pins high (inactive)
    PORTB |= (1 << CS_EEPROM_PIN) | (1 << CS_ADC_PIN) | (1 << CS_DAC_PIN) | (1 << CS_DISPLAY_PIN);
    PORTC |= (1 << CS_SPARE1_PIN) | (1 << CS_SPARE2_PIN) | (1 << CS_SPARE3_PIN) | (1 << CS_SPARE4_PIN);

    // Configure activity indicators
    DDRD |= (1 << ACTIVITY_LED_PIN) | (1 << ERROR_LED_PIN);
    PORTD &= ~((1 << ACTIVITY_LED_PIN) | (1 << ERROR_LED_PIN));

    // Count active devices
    active_device_count = 0;
    for (uint8_t i = 0; i < MAX_SPI_DEVICES; i++)
    {
        if (spi_devices[i].is_active)
        {
            active_device_count++;
        }
    }

    // Initialize SPI with default settings
    spi_configure_for_device(DEVICE_EEPROM);

    char init_msg[60];
    sprintf(init_msg, "SPI multi-device initialized: %d active devices\\r\\n", active_device_count);
    puts_USART1(init_msg);
}

void spi_configure_for_device(spi_device_t device)
{
    if (device >= MAX_SPI_DEVICES || !spi_devices[device].is_active)
    {
        spi_errors++;
        return;
    }

    spi_device_config_t *config = &spi_devices[device];

    // Configure SPI control register based on device requirements
    uint8_t spcr_value = (1 << SPE) | (1 << MSTR); // Enable SPI, Master mode

    // Set clock prescaler
    switch (config->clock_prescaler)
    {
    case 2:
        spcr_value |= 0;
        SPSR |= (1 << SPI2X);
        break;
    case 4:
        spcr_value |= 0;
        SPSR &= ~(1 << SPI2X);
        break;
    case 8:
        spcr_value |= (1 << SPR0);
        SPSR |= (1 << SPI2X);
        break;
    case 16:
        spcr_value |= (1 << SPR0);
        SPSR &= ~(1 << SPI2X);
        break;
    case 32:
        spcr_value |= (1 << SPR1);
        SPSR |= (1 << SPI2X);
        break;
    case 64:
        spcr_value |= (1 << SPR1) | (1 << SPR0);
        SPSR &= ~(1 << SPI2X);
        break;
    case 128:
        spcr_value |= (1 << SPR1);
        SPSR &= ~(1 << SPI2X);
        break;
    default:
        spcr_value |= (1 << SPR1) | (1 << SPR0);
        SPSR &= ~(1 << SPI2X);
        break; // /128
    }

    // Set SPI mode (CPOL/CPHA)
    switch (config->spi_mode)
    {
    case 0: /* CPOL=0, CPHA=0 */
        break;
    case 1:
        spcr_value |= (1 << CPHA);
        break; // CPOL=0, CPHA=1
    case 2:
        spcr_value |= (1 << CPOL);
        break; // CPOL=1, CPHA=0
    case 3:
        spcr_value |= (1 << CPOL) | (1 << CPHA);
        break; // CPOL=1, CPHA=1
    }

    SPCR = spcr_value;
    current_device = device;

    char config_msg[80];
    sprintf(config_msg, "SPI configured for %s: Mode %d, Prescaler /%d\\r\\n",
            config->device_name, config->spi_mode, config->clock_prescaler);
    puts_USART1(config_msg);
}

void spi_select_device(spi_device_t device)
{
    if (device >= MAX_SPI_DEVICES || !spi_devices[device].is_active)
    {
        PORTD |= (1 << ERROR_LED_PIN);
        _delay_ms(100);
        PORTD &= ~(1 << ERROR_LED_PIN);
        spi_errors++;
        return;
    }

    // Deselect all devices first
    for (uint8_t i = 0; i < MAX_SPI_DEVICES; i++)
    {
        if (spi_devices[i].is_active)
        {
            *spi_devices[i].cs_port |= (1 << spi_devices[i].cs_pin); // CS high (inactive)
        }
    }

    // Small delay for device deselection
    _delay_us(10);

    // Select target device
    *spi_devices[device].cs_port &= ~(1 << spi_devices[device].cs_pin); // CS low (active)

    // Activity indication
    PORTD |= (1 << ACTIVITY_LED_PIN);
    _delay_us(50);
    PORTD &= ~(1 << ACTIVITY_LED_PIN);

    device_switches++;

    // Setup delay for device selection
    _delay_us(50);
}

void spi_deselect_device(spi_device_t device)
{
    if (device >= MAX_SPI_DEVICES)
    {
        return;
    }

    // Deselect device
    *spi_devices[device].cs_port |= (1 << spi_devices[device].cs_pin); // CS high (inactive)

    // Hold time after deselection
    _delay_us(10);
}

uint8_t spi_transfer_multi(uint8_t data)
{
    SPDR = data;
    while (!(SPSR & (1 << SPIF)))
        ;
    total_transactions++;
    return SPDR;
}

uint8_t spi_device_transaction(spi_device_t device, uint8_t *tx_data, uint8_t *rx_data, uint8_t length)
{
    if (device >= MAX_SPI_DEVICES || !spi_devices[device].is_active || length == 0)
    {
        spi_errors++;
        return 0;
    }

    // Configure SPI for this device
    if (current_device != device)
    {
        spi_configure_for_device(device);
    }

    // Select device
    spi_select_device(device);

    // Perform transaction
    for (uint8_t i = 0; i < length; i++)
    {
        uint8_t tx_byte = (tx_data != NULL) ? tx_data[i] : 0x00;
        uint8_t rx_byte = spi_transfer_multi(tx_byte);

        if (rx_data != NULL)
        {
            rx_data[i] = rx_byte;
        }
    }

    // Deselect device
    spi_deselect_device(device);

    successful_operations++;
    return 1;
}

/*
 * =============================================================================
 * DEVICE-SPECIFIC COMMUNICATION FUNCTIONS
 * =============================================================================
 */

uint8_t eeprom_write_byte(uint16_t address, uint8_t data)
{
    uint8_t tx_buffer[4] = {0x06, 0x02, (address >> 8) & 0xFF, address & 0xFF};

    // Write enable
    if (!spi_device_transaction(DEVICE_EEPROM, tx_buffer, NULL, 1))
    {
        return 0;
    }

    // Write command with address and data
    tx_buffer[0] = 0x02; // Write command
    tx_buffer[3] = data;

    return spi_device_transaction(DEVICE_EEPROM, tx_buffer, NULL, 4);
}

uint8_t eeprom_read_byte(uint16_t address)
{
    uint8_t tx_buffer[4] = {0x03, (address >> 8) & 0xFF, address & 0xFF, 0x00};
    uint8_t rx_buffer[4];

    if (spi_device_transaction(DEVICE_EEPROM, tx_buffer, rx_buffer, 4))
    {
        return rx_buffer[3];
    }

    return 0xFF; // Error
}

uint16_t adc_read_channel(uint8_t channel)
{
    if (channel > 7)
        return 0xFFFF; // Invalid channel

    // MCP3008 command: Start bit + SGL + Channel + Don't care
    uint8_t tx_buffer[3] = {0x01, (0x80 | (channel << 4)), 0x00};
    uint8_t rx_buffer[3];

    if (spi_device_transaction(DEVICE_ADC, tx_buffer, rx_buffer, 3))
    {
        // Extract 10-bit result from response
        uint16_t result = ((rx_buffer[1] & 0x03) << 8) | rx_buffer[2];
        return result;
    }

    return 0xFFFF; // Error
}

uint8_t dac_write_value(uint16_t value)
{
    if (value > 4095)
        value = 4095; // 12-bit DAC

    // MCP4921 command: 0011 + 12-bit value
    uint8_t tx_buffer[2] = {
        0x30 | ((value >> 8) & 0x0F), // Command + upper 4 bits
        value & 0xFF                  // Lower 8 bits
    };

    return spi_device_transaction(DEVICE_DAC, tx_buffer, NULL, 2);
}

uint8_t display_send_command(uint8_t command)
{
    uint8_t tx_buffer[1] = {command};
    return spi_device_transaction(DEVICE_DISPLAY, tx_buffer, NULL, 1);
}

/*
 * =============================================================================
 * LAB EXERCISE 1: MULTI-DEVICE SETUP AND SELECTION (25 minutes)
 * =============================================================================
 * OBJECTIVE: Master device selection and bus management
 * DIFFICULTY: ★★★☆☆ (Medium)
 */

void lab_ex1_device_setup(void)
{
    /*
     * CHALLENGE: Set up and verify multiple SPI devices
     * TASK: Initialize device configurations and test selection
     * LEARNING: Chip select management, device isolation, configuration
     */

    puts_USART1("\\r\\n=== Lab 1: Multi-Device Setup ===\\r\\n");
    puts_USART1("Initializing SPI multi-device system\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "SPI MULTI-DEVICE");
    lcd_string(1, 0, "Setup & selection");

    spi_multi_init();

    // Display device configuration table
    puts_USART1("Device Configuration Table:\\r\\n");
    puts_USART1("ID  Name           Mode  Freq(kHz)  Active\\r\\n");
    puts_USART1("--  -----------    ----  ---------  ------\\r\\n");

    for (uint8_t i = 0; i < MAX_SPI_DEVICES; i++)
    {
        spi_device_config_t *config = &spi_devices[i];
        char config_line[60];
        sprintf(config_line, "%2d  %-12s    %d     %5d     %s\\r\\n",
                i, config->device_name, config->spi_mode,
                config->max_frequency_khz, config->is_active ? "YES" : "NO");
        puts_USART1(config_line);
    }

    char active_msg[50];
    sprintf(active_msg, "\\r\\nActive devices: %d/%d\\r\\n", active_device_count, MAX_SPI_DEVICES);
    puts_USART1(active_msg);

    char lcd_active[20];
    sprintf(lcd_active, "Active: %d/%d", active_device_count, MAX_SPI_DEVICES);
    lcd_string(3, 0, lcd_active);

    _delay_ms(2000);
}

void lab_ex1_device_selection_test(void)
{
    /*
     * CHALLENGE: Test device selection and isolation
     * TASK: Verify that device selection works correctly and devices don't interfere
     * LEARNING: Electrical isolation, timing requirements, bus sharing
     */

    puts_USART1("\\r\\n=== Lab 1.2: Device Selection Test ===\\r\\n");
    puts_USART1("Testing device selection and isolation\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "DEVICE SELECTION");
    lcd_string(1, 0, "Testing isolation");

    // Test each active device
    for (uint8_t device_idx = 0; device_idx < MAX_SPI_DEVICES; device_idx++)
    {
        if (!spi_devices[device_idx].is_active)
            continue;

        spi_device_t device = (spi_device_t)device_idx;
        spi_device_config_t *config = &spi_devices[device_idx];

        char test_msg[60];
        sprintf(test_msg, "Testing device %d (%s)\\r\\n", device_idx, config->device_name);
        puts_USART1(test_msg);

        char lcd_device[20];
        sprintf(lcd_device, "Test: %s", config->device_name);
        lcd_string(3, 0, lcd_device);

        // Configure SPI for this device
        spi_configure_for_device(device);

        // Test device selection timing
        puts_USART1("  Testing selection timing...\\r\\n");

        for (uint8_t timing_test = 0; timing_test < 5; timing_test++)
        {
            spi_select_device(device);
            _delay_us(100); // Hold selection

            // Send test pattern
            uint8_t test_data = 0xAA + timing_test;
            uint8_t response = spi_transfer_multi(test_data);

            spi_deselect_device(device);
            _delay_us(50); // Deselection time

            char timing_msg[50];
            sprintf(timing_msg, "    Test %d: Sent 0x%02X, Got 0x%02X\\r\\n",
                    timing_test + 1, test_data, response);
            puts_USART1(timing_msg);
        }

        // Test device isolation (ensure other devices don't respond)
        puts_USART1("  Testing device isolation...\\r\\n");

        spi_select_device(device);
        uint8_t isolated_response = spi_transfer_multi(0x55);
        spi_deselect_device(device);

        char isolation_msg[50];
        sprintf(isolation_msg, "    Isolation test: Response 0x%02X\\r\\n", isolated_response);
        puts_USART1(isolation_msg);

        char lcd_test[20];
        sprintf(lcd_test, "Device %d: OK", device_idx);
        lcd_string(4, 0, lcd_test);

        _delay_ms(1500);
    }

    // Device switching speed test
    puts_USART1("\\r\\nDevice switching speed test...\\r\\n");
    lcd_string(3, 0, "Switching test");

    uint32_t switch_start_time = device_switches;

    for (uint8_t cycle = 0; cycle < 20; cycle++)
    {
        for (uint8_t device_idx = 0; device_idx < MAX_SPI_DEVICES; device_idx++)
        {
            if (!spi_devices[device_idx].is_active)
                continue;

            spi_device_t device = (spi_device_t)device_idx;
            spi_configure_for_device(device);

            uint8_t test_data[2] = {0x01, cycle};
            spi_device_transaction(device, test_data, NULL, 2);
        }

        if (cycle % 5 == 0)
        {
            char switch_progress[20];
            sprintf(switch_progress, "Cycle: %d/20", cycle + 1);
            lcd_string(4, 0, switch_progress);
        }

        _delay_ms(100);
    }

    uint32_t total_switches = device_switches - switch_start_time;
    char switch_result[60];
    sprintf(switch_result, "Switching test complete: %ld switches\\r\\n", total_switches);
    puts_USART1(switch_result);

    char lcd_switches[20];
    sprintf(lcd_switches, "Switches: %ld", total_switches);
    lcd_string(5, 0, lcd_switches);

    if (total_switches >= 60)
    { // 20 cycles * 3 active devices minimum
        lab_score += 150;
    }

    _delay_ms(2000);
}

/*
 * =============================================================================
 * LAB EXERCISE 2: DIFFERENT SPI MODES AND TIMING (20 minutes)
 * =============================================================================
 * OBJECTIVE: Handle devices with different SPI mode requirements
 * DIFFICULTY: ★★★★☆ (Medium-Hard)
 */

void lab_ex2_spi_modes(void)
{
    /*
     * CHALLENGE: Communicate with devices using different SPI modes
     * TASK: Test all four SPI modes and verify compatibility
     * LEARNING: CPOL/CPHA settings, clock polarity, data sampling
     */

    puts_USART1("\\r\\n=== Lab 2: SPI Modes and Timing ===\\r\\n");
    puts_USART1("Testing different SPI modes (CPOL/CPHA combinations)\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "SPI MODES");
    lcd_string(1, 0, "CPOL/CPHA testing");

    // SPI mode explanations
    char mode_explanations[][30] = {
        "Mode 0: CPOL=0, CPHA=0",
        "Mode 1: CPOL=0, CPHA=1",
        "Mode 2: CPOL=1, CPHA=0",
        "Mode 3: CPOL=1, CPHA=1"};

    puts_USART1("SPI Mode Reference:\\r\\n");
    for (uint8_t mode = 0; mode < 4; mode++)
    {
        char ref_line[50];
        sprintf(ref_line, "  %s\\r\\n", mode_explanations[mode]);
        puts_USART1(ref_line);
    }
    puts_USART1("\\r\\n");

    // Test each SPI mode with active devices
    for (uint8_t mode = 0; mode < 4; mode++)
    {
        char mode_msg[50];
        sprintf(mode_msg, "Testing SPI Mode %d...\\r\\n", mode);
        puts_USART1(mode_msg);

        char lcd_mode[20];
        sprintf(lcd_mode, "Mode %d test", mode);
        lcd_string(3, 0, lcd_mode);

        // Temporarily set all active devices to this mode
        uint8_t original_modes[MAX_SPI_DEVICES];
        for (uint8_t i = 0; i < MAX_SPI_DEVICES; i++)
        {
            original_modes[i] = spi_devices[i].spi_mode;
            if (spi_devices[i].is_active)
            {
                spi_devices[i].spi_mode = mode;
            }
        }

        uint8_t mode_success_count = 0;

        // Test each device with the current mode
        for (uint8_t device_idx = 0; device_idx < MAX_SPI_DEVICES; device_idx++)
        {
            if (!spi_devices[device_idx].is_active)
                continue;

            spi_device_t device = (spi_device_t)device_idx;
            spi_configure_for_device(device);

            // Send test pattern and check for reasonable response
            uint8_t test_patterns[] = {0x00, 0xFF, 0xAA, 0x55, 0x01};
            uint8_t valid_responses = 0;

            for (uint8_t pattern = 0; pattern < 5; pattern++)
            {
                uint8_t tx_data = test_patterns[pattern];
                uint8_t rx_data;

                if (spi_device_transaction(device, &tx_data, &rx_data, 1))
                {
                    // Simple response validation (device should respond somehow)
                    if (rx_data != 0x00 || tx_data == 0x00)
                    {
                        valid_responses++;
                    }
                }
            }

            char device_test[70];
            sprintf(device_test, "  Device %d (%s): %d/5 valid responses\\r\\n",
                    device_idx, spi_devices[device_idx].device_name, valid_responses);
            puts_USART1(device_test);

            if (valid_responses >= 3)
            {
                mode_success_count++;
            }
        }

        // Restore original modes
        for (uint8_t i = 0; i < MAX_SPI_DEVICES; i++)
        {
            spi_devices[i].spi_mode = original_modes[i];
        }

        char mode_result[60];
        sprintf(mode_result, "Mode %d results: %d/%d devices compatible\\r\\n",
                mode, mode_success_count, active_device_count);
        puts_USART1(mode_result);

        char lcd_result[20];
        sprintf(lcd_result, "Mode %d: %d/%d OK", mode, mode_success_count, active_device_count);
        lcd_string(4, 0, lcd_result);

        if (mode_success_count > 0)
        {
            lab_score += 50;
        }

        _delay_ms(2000);
    }

    puts_USART1("SPI mode testing complete\\r\\n");
    lcd_string(5, 0, "Mode test complete");
}

void lab_ex2_clock_optimization(void)
{
    /*
     * CHALLENGE: Optimize SPI clock frequencies for different devices
     * TASK: Test maximum reliable frequencies for each device
     * LEARNING: Clock frequency limits, signal integrity, timing margins
     */

    puts_USART1("\\r\\n=== Lab 2.2: Clock Optimization ===\\r\\n");
    puts_USART1("Testing SPI clock frequency optimization\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "CLOCK OPTIMIZATION");
    lcd_string(1, 0, "Frequency testing");

    // Clock prescaler test values
    uint8_t prescalers[] = {2, 4, 8, 16, 32, 64, 128};
    uint8_t prescaler_count = 7;

    // Test each active device at different frequencies
    for (uint8_t device_idx = 0; device_idx < MAX_SPI_DEVICES; device_idx++)
    {
        if (!spi_devices[device_idx].is_active)
            continue;

        spi_device_t device = (spi_device_t)device_idx;
        spi_device_config_t *config = &spi_devices[device_idx];

        char freq_test_msg[60];
        sprintf(freq_test_msg, "\\r\\nFrequency test for %s:\\r\\n", config->device_name);
        puts_USART1(freq_test_msg);

        char lcd_device[20];
        sprintf(lcd_device, "Test: %s", config->device_name);
        lcd_string(3, 0, lcd_device);

        uint8_t max_working_prescaler = 128;

        // Test prescalers from fastest to slowest
        for (uint8_t p = 0; p < prescaler_count; p++)
        {
            uint8_t prescaler = prescalers[p];
            uint32_t frequency_khz = F_CPU / (prescaler * 1000);

            char freq_msg[60];
            sprintf(freq_msg, "  Testing prescaler /%d (%ld kHz)...\\r\\n",
                    prescaler, frequency_khz);
            puts_USART1(freq_msg);

            // Temporarily set prescaler
            uint8_t original_prescaler = config->clock_prescaler;
            config->clock_prescaler = prescaler;

            spi_configure_for_device(device);

            // Test communication reliability
            uint8_t reliable_transfers = 0;
            uint8_t test_data[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

            for (uint8_t test = 0; test < 8; test++)
            {
                uint8_t tx_data = test_data[test];
                uint8_t rx_data;

                if (spi_device_transaction(device, &tx_data, &rx_data, 1))
                {
                    // Simple reliability check
                    reliable_transfers++;
                }

                _delay_ms(10);
            }

            char reliability_msg[50];
            sprintf(reliability_msg, "    Reliability: %d/8 transfers\\r\\n", reliable_transfers);
            puts_USART1(reliability_msg);

            char lcd_freq[20];
            sprintf(lcd_freq, "/%d: %d/8 OK", prescaler, reliable_transfers);
            lcd_string(4, 0, lcd_freq);

            if (reliable_transfers >= 6)
            { // 75% success rate
                max_working_prescaler = prescaler;
                char success_msg[50];
                sprintf(success_msg, "    ✓ Prescaler /%d works reliably\\r\\n", prescaler);
                puts_USART1(success_msg);
                lab_score += 25;
            }
            else
            {
                char fail_msg[50];
                sprintf(fail_msg, "    ❌ Prescaler /%d unreliable\\r\\n", prescaler);
                puts_USART1(fail_msg);
                break; // Stop testing faster frequencies
            }

            // Restore original prescaler
            config->clock_prescaler = original_prescaler;

            _delay_ms(1000);
        }

        uint32_t max_freq_khz = F_CPU / (max_working_prescaler * 1000);
        char max_freq_msg[80];
        sprintf(max_freq_msg, "Maximum reliable frequency for %s: %ld kHz (/%d)\\r\\n",
                config->device_name, max_freq_khz, max_working_prescaler);
        puts_USART1(max_freq_msg);

        char lcd_max[20];
        sprintf(lcd_max, "Max: %ld kHz", max_freq_khz);
        lcd_string(5, 0, lcd_max);

        _delay_ms(2000);
    }

    puts_USART1("Clock optimization complete\\r\\n");
}

/*
 * =============================================================================
 * LAB EXERCISE 3: DEVICE COORDINATION (20 minutes)
 * =============================================================================
 * OBJECTIVE: Coordinate data flow between multiple devices
 * DIFFICULTY: ★★★★☆ (Medium-Hard)
 */

void lab_ex3_data_pipeline(void)
{
    /*
     * CHALLENGE: Create a data processing pipeline using multiple devices
     * TASK: Read from ADC, process data, store in EEPROM, output to DAC
     * LEARNING: Device coordination, data flow, system integration
     */

    puts_USART1("\\r\\n=== Lab 3: Device Coordination ===\\r\\n");
    puts_USART1("Creating multi-device data pipeline\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "DATA PIPELINE");
    lcd_string(1, 0, "Multi-device flow");

    // Pipeline: ADC → Processing → EEPROM → DAC
    puts_USART1("Data Pipeline: ADC → Process → EEPROM → DAC\\r\\n\\r\\n");

    uint16_t pipeline_cycles = 15;
    uint16_t successful_cycles = 0;
    uint16_t eeprom_address = 0x1000; // Starting address for data storage

    for (uint16_t cycle = 0; cycle < pipeline_cycles; cycle++)
    {
        char cycle_msg[50];
        sprintf(cycle_msg, "Pipeline cycle %d/%d:\\r\\n", cycle + 1, pipeline_cycles);
        puts_USART1(cycle_msg);

        char lcd_cycle[20];
        sprintf(lcd_cycle, "Cycle: %d/%d", cycle + 1, pipeline_cycles);
        lcd_string(3, 0, lcd_cycle);

        // Step 1: Read from ADC (simulate multiple channels)
        uint16_t adc_values[4];
        uint8_t adc_success = 1;

        puts_USART1("  Step 1: Reading ADC channels...\\r\\n");
        for (uint8_t channel = 0; channel < 4; channel++)
        {
            adc_values[channel] = adc_read_channel(channel);

            if (adc_values[channel] == 0xFFFF)
            {
                adc_success = 0;
                break;
            }

            char adc_msg[40];
            sprintf(adc_msg, "    Ch%d: %d\\r\\n", channel, adc_values[channel]);
            puts_USART1(adc_msg);
        }

        if (!adc_success)
        {
            puts_USART1("  ADC reading failed, skipping cycle\\r\\n");
            continue;
        }

        // Step 2: Process data (simple averaging and scaling)
        puts_USART1("  Step 2: Processing data...\\r\\n");

        uint32_t sum = 0;
        for (uint8_t i = 0; i < 4; i++)
        {
            sum += adc_values[i];
        }
        uint16_t average = sum / 4;
        uint16_t scaled_output = (average * 4095) / 1023; // Scale to 12-bit DAC range

        char process_msg[60];
        sprintf(process_msg, "    Average: %d, Scaled: %d\\r\\n", average, scaled_output);
        puts_USART1(process_msg);

        char lcd_process[20];
        sprintf(lcd_process, "Avg:%d Sc:%d", average, scaled_output);
        lcd_string(4, 0, lcd_process);

        // Step 3: Store in EEPROM
        puts_USART1("  Step 3: Storing to EEPROM...\\r\\n");

        uint8_t eeprom_success = 1;

        // Store timestamp (cycle number) and processed data
        if (!eeprom_write_byte(eeprom_address, cycle & 0xFF))
        {
            eeprom_success = 0;
        }
        _delay_ms(10); // EEPROM write delay

        if (!eeprom_write_byte(eeprom_address + 1, (average >> 8) & 0xFF))
        {
            eeprom_success = 0;
        }
        _delay_ms(10);

        if (!eeprom_write_byte(eeprom_address + 2, average & 0xFF))
        {
            eeprom_success = 0;
        }
        _delay_ms(10);

        if (eeprom_success)
        {
            char eeprom_msg[50];
            sprintf(eeprom_msg, "    Stored at 0x%04X\\r\\n", eeprom_address);
            puts_USART1(eeprom_msg);
            eeprom_address += 4; // Move to next storage location
        }
        else
        {
            puts_USART1("    EEPROM storage failed\\r\\n");
        }

        // Step 4: Output to DAC
        puts_USART1("  Step 4: Updating DAC output...\\r\\n");

        if (dac_write_value(scaled_output))
        {
            char dac_msg[40];
            sprintf(dac_msg, "    DAC output: %d\\r\\n", scaled_output);
            puts_USART1(dac_msg);

            char lcd_dac[20];
            sprintf(lcd_dac, "DAC: %d", scaled_output);
            lcd_string(5, 0, lcd_dac);

            if (adc_success && eeprom_success)
            {
                successful_cycles++;
            }
        }
        else
        {
            puts_USART1("    DAC output failed\\r\\n");
        }

        puts_USART1("\\r\\n");
        _delay_ms(1000);
    }

    // Pipeline statistics
    char pipeline_stats[80];
    sprintf(pipeline_stats, "Pipeline complete: %d/%d successful cycles\\r\\n",
            successful_cycles, pipeline_cycles);
    puts_USART1(pipeline_stats);

    char success_rate[50];
    sprintf(success_rate, "Success rate: %d%%\\r\\n",
            (successful_cycles * 100) / pipeline_cycles);
    puts_USART1(success_rate);

    if (successful_cycles >= 10)
    {
        lab_score += 200;
    }

    _delay_ms(2000);
}

/*
 * =============================================================================
 * LAB EXERCISE 4: ADVANCED MULTI-DEVICE APPLICATIONS (15 minutes)
 * =============================================================================
 * OBJECTIVE: Build complex applications using device coordination
 * DIFFICULTY: ★★★★★ (Hard)
 */

void lab_ex4_advanced_coordination(void)
{
    /*
     * CHALLENGE: Implement advanced multi-device coordination
     * TASK: Create a complete sensor monitoring and control system
     * LEARNING: System integration, real-time coordination, advanced protocols
     */

    puts_USART1("\\r\\n=== Lab 4: Advanced Device Coordination ===\\r\\n");
    puts_USART1("Building integrated sensor and control system\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "ADVANCED COORD");
    lcd_string(1, 0, "System integration");

    // System states
    typedef enum
    {
        SYS_INIT,
        SYS_MONITORING,
        SYS_ALERT,
        SYS_LOGGING,
        SYS_CONTROL
    } system_state_t;

    system_state_t current_state = SYS_INIT;
    uint16_t monitoring_cycles = 0;
    uint16_t alert_conditions = 0;
    uint16_t control_actions = 0;

    // System thresholds
    uint16_t temp_threshold_high = 800; // ADC value
    uint16_t temp_threshold_low = 200;
    uint16_t pressure_threshold = 700;

    puts_USART1("System Parameters:\\r\\n");
    char params[80];
    sprintf(params, "  Temperature: %d - %d ADC units\\r\\n", temp_threshold_low, temp_threshold_high);
    puts_USART1(params);
    sprintf(params, "  Pressure threshold: %d ADC units\\r\\n", pressure_threshold);
    puts_USART1(params);
    puts_USART1("\\r\\n");

    // Main system loop
    for (uint8_t system_cycle = 0; system_cycle < 20; system_cycle++)
    {
        char sys_msg[50];
        sprintf(sys_msg, "System cycle %d/20:\\r\\n", system_cycle + 1);
        puts_USART1(sys_msg);

        char lcd_sys[20];
        sprintf(lcd_sys, "Sys: %d/20", system_cycle + 1);
        lcd_string(3, 0, lcd_sys);

        switch (current_state)
        {
        case SYS_INIT:
            puts_USART1("  State: INITIALIZATION\\r\\n");
            lcd_string(4, 0, "State: INIT");

            // Initialize all devices
            for (uint8_t dev = 0; dev < MAX_SPI_DEVICES; dev++)
            {
                if (spi_devices[dev].is_active)
                {
                    spi_configure_for_device((spi_device_t)dev);

                    // Send initialization command
                    uint8_t init_cmd = 0x01;
                    spi_device_transaction((spi_device_t)dev, &init_cmd, NULL, 1);
                }
            }

            current_state = SYS_MONITORING;
            puts_USART1("  → Transitioning to MONITORING\\r\\n");
            break;

        case SYS_MONITORING:
            puts_USART1("  State: MONITORING\\r\\n");
            lcd_string(4, 0, "State: MONITOR");

            // Read sensor data
            uint16_t temperature = adc_read_channel(0);
            uint16_t pressure = adc_read_channel(1);
            uint16_t humidity = adc_read_channel(2);

            char sensor_data[80];
            sprintf(sensor_data, "    Sensors: T=%d, P=%d, H=%d\\r\\n",
                    temperature, pressure, humidity);
            puts_USART1(sensor_data);

            char lcd_sensors[20];
            sprintf(lcd_sensors, "T%d P%d H%d", temperature, pressure, humidity);
            lcd_string(5, 0, lcd_sensors);

            monitoring_cycles++;

            // Check for alert conditions
            if (temperature > temp_threshold_high || temperature < temp_threshold_low ||
                pressure > pressure_threshold)
            {
                current_state = SYS_ALERT;
                puts_USART1("  → Alert condition detected!\\r\\n");
                alert_conditions++;
            }
            else if (monitoring_cycles % 5 == 0)
            {
                current_state = SYS_LOGGING;
                puts_USART1("  → Periodic logging\\r\\n");
            }
            else if (monitoring_cycles % 3 == 0)
            {
                current_state = SYS_CONTROL;
                puts_USART1("  → Control update\\r\\n");
            }
            break;

        case SYS_ALERT:
            puts_USART1("  State: ALERT\\r\\n");
            lcd_string(4, 0, "State: ALERT");

            // Alert actions: Log to EEPROM and update DAC
            uint16_t alert_addr = 0x2000 + (alert_conditions * 4);

            // Log alert data
            eeprom_write_byte(alert_addr, 0xFF); // Alert marker
            eeprom_write_byte(alert_addr + 1, system_cycle);
            eeprom_write_byte(alert_addr + 2, monitoring_cycles & 0xFF);

            // Set DAC to alert level
            dac_write_value(4095); // Maximum output

            puts_USART1("    Alert logged and DAC set to maximum\\r\\n");
            current_state = SYS_MONITORING;
            break;

        case SYS_LOGGING:
            puts_USART1("  State: LOGGING\\r\\n");
            lcd_string(4, 0, "State: LOG");

            // Log current sensor readings
            uint16_t log_addr = 0x3000 + (monitoring_cycles * 8);

            uint16_t current_temp = adc_read_channel(0);
            uint16_t current_press = adc_read_channel(1);

            eeprom_write_byte(log_addr + 0, system_cycle);
            eeprom_write_byte(log_addr + 1, (current_temp >> 8) & 0xFF);
            eeprom_write_byte(log_addr + 2, current_temp & 0xFF);
            eeprom_write_byte(log_addr + 3, (current_press >> 8) & 0xFF);
            eeprom_write_byte(log_addr + 4, current_press & 0xFF);

            char log_msg[60];
            sprintf(log_msg, "    Logged to 0x%04X: T=%d, P=%d\\r\\n",
                    log_addr, current_temp, current_press);
            puts_USART1(log_msg);

            current_state = SYS_MONITORING;
            break;

        case SYS_CONTROL:
            puts_USART1("  State: CONTROL\\r\\n");
            lcd_string(4, 0, "State: CONTROL");

            // Control logic: Adjust DAC based on sensor readings
            uint16_t control_temp = adc_read_channel(0);
            uint16_t dac_output;

            if (control_temp < 300)
            {
                dac_output = 1000; // Low heating
            }
            else if (control_temp < 600)
            {
                dac_output = 2000; // Medium heating
            }
            else if (control_temp < 900)
            {
                dac_output = 3000; // High heating
            }
            else
            {
                dac_output = 0; // No heating
            }

            dac_write_value(dac_output);
            control_actions++;

            char control_msg[60];
            sprintf(control_msg, "    Control: T=%d → DAC=%d\\r\\n",
                    control_temp, dac_output);
            puts_USART1(control_msg);

            current_state = SYS_MONITORING;
            break;
        }

        _delay_ms(800);
    }

    // System performance summary
    puts_USART1("\\r\\n=== SYSTEM PERFORMANCE SUMMARY ===\\r\\n");

    char perf_stats[80];
    sprintf(perf_stats, "Monitoring cycles: %d\\r\\n", monitoring_cycles);
    puts_USART1(perf_stats);

    sprintf(perf_stats, "Alert conditions: %d\\r\\n", alert_conditions);
    puts_USART1(perf_stats);

    sprintf(perf_stats, "Control actions: %d\\r\\n", control_actions);
    puts_USART1(perf_stats);

    sprintf(perf_stats, "Total SPI transactions: %ld\\r\\n", total_transactions);
    puts_USART1(perf_stats);

    sprintf(perf_stats, "Device switches: %d\\r\\n", device_switches);
    puts_USART1(perf_stats);

    sprintf(perf_stats, "SPI errors: %d\\r\\n", spi_errors);
    puts_USART1(perf_stats);

    uint8_t system_efficiency = 0;
    if (total_transactions > 0)
    {
        system_efficiency = ((total_transactions - spi_errors) * 100) / total_transactions;
    }

    sprintf(perf_stats, "System efficiency: %d%%\\r\\n", system_efficiency);
    puts_USART1(perf_stats);

    if (system_efficiency >= 90 && monitoring_cycles >= 15)
    {
        lab_score += 250;
        puts_USART1("✓ Excellent system performance!\\r\\n");
    }

    char lcd_final[20];
    sprintf(lcd_final, "Eff: %d%% Cyc: %d", system_efficiency, monitoring_cycles);
    lcd_string(5, 0, lcd_final);
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
    puts_USART1("     SPI MULTI-DEVICE - LAB EXERCISES       \\r\\n");
    puts_USART1("==============================================\\r\\n");
    puts_USART1("1. Multi-Device Setup & Selection           \\r\\n");
    puts_USART1("2. Different SPI Modes & Timing             \\r\\n");
    puts_USART1("3. Device Coordination & Data Flow          \\r\\n");
    puts_USART1("4. Advanced Multi-Device Applications       \\r\\n");
    puts_USART1("                                              \\r\\n");
    puts_USART1("0. Run All Exercises                         \\r\\n");
    puts_USART1("X. Exit Lab                                   \\r\\n");
    puts_USART1("==============================================\\r\\n");
    char score_msg[50];
    sprintf(score_msg, "Current Score: %d points\\r\\n", lab_score);
    puts_USART1(score_msg);
    char device_stats[60];
    sprintf(device_stats, "Active Devices: %d, Switches: %d\\r\\n", active_device_count, device_switches);
    puts_USART1(device_stats);
    char transaction_stats[60];
    sprintf(transaction_stats, "Transactions: %ld, Successful: %d, Errors: %d\\r\\n",
            total_transactions, successful_operations, spi_errors);
    puts_USART1(transaction_stats);
    puts_USART1("Select exercise (1-4, 0, X): ");
}

int main(void)
{
    init_devices();

    puts_USART1("\\r\\n*** SPI MULTI-DEVICE LAB SESSION ***\\r\\n");
    puts_USART1("Welcome to advanced SPI multi-device communication!\\r\\n");
    puts_USART1("This lab covers device management, coordination, and system integration\\r\\n");
    puts_USART1("Ensure all SPI devices are properly connected with individual CS lines\\r\\n");

    lcd_clear();
    lcd_string(1, 0, "SPI MULTI-DEVICE");
    lcd_string(2, 0, "Advanced systems");
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
            lab_ex1_device_setup();
            lab_ex1_device_selection_test();
            break;

        case '2':
            lab_ex2_spi_modes();
            lab_ex2_clock_optimization();
            break;

        case '3':
            lab_ex3_data_pipeline();
            break;

        case '4':
            lab_ex4_advanced_coordination();
            break;

        case '0':
            puts_USART1("\\r\\n*** RUNNING ALL EXERCISES ***\\r\\n");
            lab_ex1_device_setup();
            lab_ex1_device_selection_test();
            lab_ex2_spi_modes();
            lab_ex2_clock_optimization();
            lab_ex3_data_pipeline();
            lab_ex4_advanced_coordination();

            char final_buffer[80];
            sprintf(final_buffer, "\\r\\n*** ALL EXERCISES COMPLETE! ***\\r\\nFinal Score: %d points\\r\\n", lab_score);
            puts_USART1(final_buffer);
            break;

        case 'X':
        case 'x':
            puts_USART1("\\r\\nExiting lab. Great work on multi-device SPI!\\r\\n");
            puts_USART1("Remember: SPI multi-device systems are key to complex embedded applications!\\r\\n");
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