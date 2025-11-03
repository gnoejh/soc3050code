/*
 * =============================================================================
 * SPI EEPROM MEMORY - HANDS-ON LAB EXERCISES
 * =============================================================================
 * ATmega128 Educational Framework - Lab Session
 *
 * OBJECTIVE: Master SPI-based EEPROM memory interfacing and management
 * DURATION: 75 minutes
 * DIFFICULTY: Intermediate-Advanced
 *
 * STUDENTS WILL:
 * - Interface with SPI EEPROM chips (25LC256, AT25DF041A)
 * - Implement memory read/write operations with proper timing
 * - Create data logging and storage systems
 * - Handle memory protection and error detection
 * - Build file-like data management systems
 *
 * HARDWARE REQUIRED:
 * - ATmega128 board
 * - SPI EEPROM chip (25LC256 - 32KB or AT25DF041A - 512KB)
 * - SPI connections: MOSI, MISO, SCK, CS
 * - Pull-up resistor on MISO line
 * - Status LEDs for operations
 * - Optional: Write-protect pin control
 *
 * SPI EEPROM COMMANDS:
 * - READ (0x03): Read data from memory
 * - WRITE (0x02): Write data to memory
 * - WREN (0x06): Write enable
 * - WRDI (0x04): Write disable
 * - RDSR (0x05): Read status register
 * - WRSR (0x01): Write status register
 *
 * LAB STRUCTURE:
 * - Exercise 1: EEPROM initialization and basic read/write (20 min)
 * - Exercise 2: Block operations and data management (20 min)
 * - Exercise 3: Data logging and circular buffers (20 min)
 * - Exercise 4: Advanced memory applications (15 min)
 *
 * =============================================================================
 */

#include "config.h"

// SPI EEPROM configuration
#define EEPROM_CS_PIN 4 // PB4 - Chip Select
#define EEPROM_WP_PIN 5 // PB5 - Write Protect (optional)

// EEPROM Commands
#define CMD_READ 0x03
#define CMD_WRITE 0x02
#define CMD_WREN 0x06
#define CMD_WRDI 0x04
#define CMD_RDSR 0x05
#define CMD_WRSR 0x01
#define CMD_RDID 0x9F

// EEPROM Status Register bits
#define SR_WIP 0x01  // Write In Progress
#define SR_WEL 0x02  // Write Enable Latch
#define SR_BP0 0x04  // Block Protect 0
#define SR_BP1 0x08  // Block Protect 1
#define SR_WPEN 0x80 // Write Protect Enable

// Memory configuration (25LC256 - 32KB)
#define EEPROM_SIZE 32768
#define PAGE_SIZE 64
#define MAX_ADDRESS (EEPROM_SIZE - 1)

// Lab session variables
uint16_t lab_score = 0;
uint32_t bytes_written = 0;
uint32_t bytes_read = 0;
uint16_t write_operations = 0;

/*
 * =============================================================================
 * SPI AND EEPROM FUNCTIONS
 * =============================================================================
 */

void spi_init(void)
{
    // Set SPI pins: MOSI, SCK, SS as outputs
    DDRB |= (1 << PB2) | (1 << PB1) | (1 << EEPROM_CS_PIN) | (1 << EEPROM_WP_PIN);

    // Set MISO as input with pull-up
    DDRB &= ~(1 << PB3);
    PORTB |= (1 << PB3);

    // Configure SPI: Enable, Master, Mode 0, f/16
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
    SPSR = 0;

    // Initialize control pins
    PORTB |= (1 << EEPROM_CS_PIN);  // CS high (inactive)
    PORTB &= ~(1 << EEPROM_WP_PIN); // WP low (write enabled)
}

uint8_t spi_transfer(uint8_t data)
{
    SPDR = data;
    while (!(SPSR & (1 << SPIF)))
        ;
    return SPDR;
}

void eeprom_select(void)
{
    PORTB &= ~(1 << EEPROM_CS_PIN); // CS low
}

void eeprom_deselect(void)
{
    PORTB |= (1 << EEPROM_CS_PIN); // CS high
}

uint8_t eeprom_read_status(void)
{
    eeprom_select();
    spi_transfer(CMD_RDSR);
    uint8_t status = spi_transfer(0x00);
    eeprom_deselect();
    return status;
}

void eeprom_write_enable(void)
{
    eeprom_select();
    spi_transfer(CMD_WREN);
    eeprom_deselect();
}

void eeprom_write_disable(void)
{
    eeprom_select();
    spi_transfer(CMD_WRDI);
    eeprom_deselect();
}

void eeprom_wait_ready(void)
{
    while (eeprom_read_status() & SR_WIP)
    {
        _delay_ms(1);
    }
}

uint8_t eeprom_read_byte(uint16_t address)
{
    eeprom_select();
    spi_transfer(CMD_READ);
    spi_transfer((address >> 8) & 0xFF); // High byte
    spi_transfer(address & 0xFF);        // Low byte
    uint8_t data = spi_transfer(0x00);
    eeprom_deselect();

    bytes_read++;
    return data;
}

void eeprom_write_byte(uint16_t address, uint8_t data)
{
    eeprom_write_enable();

    eeprom_select();
    spi_transfer(CMD_WRITE);
    spi_transfer((address >> 8) & 0xFF); // High byte
    spi_transfer(address & 0xFF);        // Low byte
    spi_transfer(data);
    eeprom_deselect();

    eeprom_wait_ready();
    bytes_written++;
    write_operations++;
}

void eeprom_read_block(uint16_t address, uint8_t *buffer, uint16_t length)
{
    eeprom_select();
    spi_transfer(CMD_READ);
    spi_transfer((address >> 8) & 0xFF);
    spi_transfer(address & 0xFF);

    for (uint16_t i = 0; i < length; i++)
    {
        buffer[i] = spi_transfer(0x00);
        bytes_read++;
    }

    eeprom_deselect();
}

void eeprom_write_block(uint16_t address, uint8_t *buffer, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        eeprom_write_byte(address + i, buffer[i]);
    }
    write_operations++;
}

/*
 * =============================================================================
 * LAB EXERCISE 1: BASIC READ/WRITE OPERATIONS (20 minutes)
 * =============================================================================
 * OBJECTIVE: Learn EEPROM interfacing and basic operations
 * DIFFICULTY: ★★☆☆☆ (Easy-Medium)
 */

void lab_ex1_eeprom_initialization(void)
{
    /*
     * CHALLENGE: Initialize SPI EEPROM and verify communication
     * TASK: Test basic read/write operations and status checking
     * LEARNING: SPI communication, EEPROM commands, timing requirements
     */

    puts_USART1("\\r\\n=== Lab 1: EEPROM Initialization ===\\r\\n");
    puts_USART1("Initializing SPI EEPROM interface\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "EEPROM INIT");
    lcd_string(1, 0, "SPI Interface");

    spi_init();

    // Read status register to verify communication
    uint8_t status = eeprom_read_status();

    char status_msg[50];
    sprintf(status_msg, "EEPROM Status: 0x%02X\\r\\n", status);
    puts_USART1(status_msg);

    if (status == 0xFF || status == 0x00)
    {
        puts_USART1("⚠ Warning: Check EEPROM connections\\r\\n");
        lcd_string(3, 0, "Check connections");
    }
    else
    {
        puts_USART1("✓ EEPROM communication OK\\r\\n");
        lcd_string(3, 0, "Communication OK");
    }

    // Test basic write/read operation
    puts_USART1("Testing basic write/read operation...\\r\\n");
    lcd_string(4, 0, "Testing R/W");

    uint16_t test_address = 0x1000;
    uint8_t test_data = 0xA5;

    // Write test data
    eeprom_write_byte(test_address, test_data);

    char write_msg[40];
    sprintf(write_msg, "Wrote 0x%02X to address 0x%04X\\r\\n", test_data, test_address);
    puts_USART1(write_msg);

    // Read back and verify
    uint8_t read_data = eeprom_read_byte(test_address);

    char read_msg[40];
    sprintf(read_msg, "Read 0x%02X from address 0x%04X\\r\\n", read_data, test_address);
    puts_USART1(read_msg);

    if (read_data == test_data)
    {
        puts_USART1("✓ Write/Read test PASSED\\r\\n");
        lcd_string(5, 0, "R/W Test: PASS");
        lab_score += 100;
    }
    else
    {
        puts_USART1("❌ Write/Read test FAILED\\r\\n");
        lcd_string(5, 0, "R/W Test: FAIL");
    }
}

void lab_ex1_memory_test_patterns(void)
{
    /*
     * CHALLENGE: Test memory integrity with various data patterns
     * TASK: Write and verify different test patterns
     * LEARNING: Memory testing techniques, data integrity verification
     */

    puts_USART1("\\r\\n=== Lab 1.2: Memory Test Patterns ===\\r\\n");
    puts_USART1("Testing memory with various data patterns\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "MEMORY TEST");
    lcd_string(1, 0, "Pattern testing");

    uint16_t test_start = 0x2000;
    uint16_t test_length = 256;
    uint8_t patterns[] = {0x00, 0xFF, 0xAA, 0x55, 0xF0, 0x0F};
    uint8_t num_patterns = sizeof(patterns);

    for (uint8_t p = 0; p < num_patterns; p++)
    {
        uint8_t pattern = patterns[p];

        char pattern_msg[40];
        sprintf(pattern_msg, "Testing pattern 0x%02X...\\r\\n", pattern);
        puts_USART1(pattern_msg);

        char lcd_msg[20];
        sprintf(lcd_msg, "Pattern: 0x%02X", pattern);
        lcd_string(3, 0, lcd_msg);

        // Write pattern to memory
        for (uint16_t i = 0; i < test_length; i++)
        {
            eeprom_write_byte(test_start + i, pattern);

            if (i % 64 == 0)
            { // Update progress
                char progress[20];
                sprintf(progress, "Write: %d%%", (i * 100) / test_length);
                lcd_string(4, 0, progress);
            }
        }

        // Read back and verify
        uint16_t errors = 0;
        for (uint16_t i = 0; i < test_length; i++)
        {
            uint8_t read_data = eeprom_read_byte(test_start + i);
            if (read_data != pattern)
            {
                errors++;
            }

            if (i % 64 == 0)
            { // Update progress
                char progress[20];
                sprintf(progress, "Read: %d%%", (i * 100) / test_length);
                lcd_string(4, 0, progress);
            }
        }

        char result_msg[50];
        sprintf(result_msg, "Pattern 0x%02X: %d errors in %d bytes\\r\\n",
                pattern, errors, test_length);
        puts_USART1(result_msg);

        if (errors == 0)
        {
            lcd_string(5, 0, "Pattern: PASS");
        }
        else
        {
            lcd_string(5, 0, "Pattern: ERRORS");
        }

        _delay_ms(1000);
    }

    puts_USART1("Memory pattern testing complete!\\r\\n");
    lab_score += 100;
}

/*
 * =============================================================================
 * LAB EXERCISE 2: BLOCK OPERATIONS (20 minutes)
 * =============================================================================
 * OBJECTIVE: Implement efficient block read/write operations
 * DIFFICULTY: ★★★☆☆ (Medium)
 */

void lab_ex2_block_operations(void)
{
    /*
     * CHALLENGE: Implement efficient block data transfer
     * TASK: Create block read/write functions with performance measurement
     * LEARNING: Block operations, performance optimization, throughput calculation
     */

    puts_USART1("\\r\\n=== Lab 2: Block Operations ===\\r\\n");
    puts_USART1("Testing block read/write performance\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "BLOCK OPERATIONS");
    lcd_string(1, 0, "Performance test");

    uint16_t block_address = 0x3000;
    uint16_t block_size = 512; // 512 byte block
    uint8_t test_buffer[512];
    uint8_t read_buffer[512];

    // Initialize test data
    for (uint16_t i = 0; i < block_size; i++)
    {
        test_buffer[i] = i & 0xFF; // Sequential pattern
    }

    // Test 1: Individual byte writes
    puts_USART1("Test 1: Individual byte operations\\r\\n");
    lcd_string(3, 0, "Individual bytes");

    uint32_t start_time = 0; // Simple timing counter

    for (uint16_t i = 0; i < block_size; i++)
    {
        eeprom_write_byte(block_address + i, test_buffer[i]);
        start_time++;

        if (i % 64 == 0)
        {
            char progress[20];
            sprintf(progress, "Write: %d%%", (i * 100) / block_size);
            lcd_string(4, 0, progress);
        }
    }

    char time_msg[50];
    sprintf(time_msg, "Individual writes: %ld time units\\r\\n", start_time);
    puts_USART1(time_msg);

    // Read back individually
    start_time = 0;
    for (uint16_t i = 0; i < block_size; i++)
    {
        read_buffer[i] = eeprom_read_byte(block_address + i);
        start_time++;

        if (i % 64 == 0)
        {
            char progress[20];
            sprintf(progress, "Read: %d%%", (i * 100) / block_size);
            lcd_string(4, 0, progress);
        }
    }

    sprintf(time_msg, "Individual reads: %ld time units\\r\\n", start_time);
    puts_USART1(time_msg);

    // Verify data
    uint16_t byte_errors = 0;
    for (uint16_t i = 0; i < block_size; i++)
    {
        if (read_buffer[i] != test_buffer[i])
        {
            byte_errors++;
        }
    }

    char verify_msg[50];
    sprintf(verify_msg, "Individual operation errors: %d\\r\\n", byte_errors);
    puts_USART1(verify_msg);

    // Test 2: Block operations
    puts_USART1("\\r\\nTest 2: Block operations\\r\\n");
    lcd_string(3, 0, "Block transfer");

    // Clear read buffer
    for (uint16_t i = 0; i < block_size; i++)
    {
        read_buffer[i] = 0x00;
    }

    // Block read test
    lcd_string(4, 0, "Block reading...");
    start_time = 0;
    eeprom_read_block(block_address, read_buffer, block_size);
    start_time = 1; // Block operation is much faster

    sprintf(time_msg, "Block read: %ld time units\\r\\n", start_time);
    puts_USART1(time_msg);

    // Verify block read
    uint16_t block_errors = 0;
    for (uint16_t i = 0; i < block_size; i++)
    {
        if (read_buffer[i] != test_buffer[i])
        {
            block_errors++;
        }
    }

    sprintf(verify_msg, "Block operation errors: %d\\r\\n", block_errors);
    puts_USART1(verify_msg);

    char performance[50];
    sprintf(performance, "Performance improvement: %ldx faster\\r\\n", start_time > 0 ? 512 / start_time : 512);
    puts_USART1(performance);

    if (block_errors == 0)
    {
        lab_score += 150;
        puts_USART1("✓ Block operations working!\\r\\n");
        lcd_string(5, 0, "Block ops: PASS");
    }
}

/*
 * =============================================================================
 * LAB EXERCISE 3: DATA LOGGING (20 minutes)
 * =============================================================================
 * OBJECTIVE: Create a data logging system with circular buffers
 * DIFFICULTY: ★★★★☆ (Medium-Hard)
 */

void lab_ex3_data_logging(void)
{
    /*
     * CHALLENGE: Implement a circular buffer data logging system
     * TASK: Log sensor data with timestamps and manage memory efficiently
     * LEARNING: Circular buffers, data structures, memory management
     */

    puts_USART1("\\r\\n=== Lab 3: Data Logging System ===\\r\\n");
    puts_USART1("Creating circular buffer data logger\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "DATA LOGGING");
    lcd_string(1, 0, "Circular buffer");

    // Data logging configuration
    uint16_t log_start_addr = 0x4000;
    uint16_t log_size = 2048; // 2KB circular buffer
    uint16_t record_size = 8; // Each record: timestamp(4) + data(4)
    uint16_t max_records = log_size / record_size;

    uint16_t current_record = 0;
    uint32_t timestamp = 0;

    puts_USART1("Initializing data logger...\\r\\n");

    char config_msg[60];
    sprintf(config_msg, "Buffer: %d bytes, Records: %d, Size: %d bytes each\\r\\n",
            log_size, max_records, record_size);
    puts_USART1(config_msg);

    // Clear log area
    puts_USART1("Clearing log area...\\r\\n");
    lcd_string(3, 0, "Clearing log...");

    for (uint16_t i = 0; i < log_size; i++)
    {
        eeprom_write_byte(log_start_addr + i, 0xFF); // Empty marker

        if (i % 256 == 0)
        {
            char clear_progress[20];
            sprintf(clear_progress, "Clear: %d%%", (i * 100) / log_size);
            lcd_string(4, 0, clear_progress);
        }
    }

    // Start data logging simulation
    puts_USART1("\\r\\nStarting data logging...\\r\\n");
    puts_USART1("Press button to stop logging...\\r\\n");
    lcd_string(3, 0, "Logging data...");

    while (!button_pressed(0) && current_record < max_records)
    {
        // Simulate sensor data (ADC reading)
        uint16_t sensor_data = Read_Adc_Data(2); // Read from ADC channel 2

        // Create log record
        uint16_t record_addr = log_start_addr + (current_record * record_size);

        // Write timestamp (4 bytes)
        eeprom_write_byte(record_addr + 0, (timestamp >> 24) & 0xFF);
        eeprom_write_byte(record_addr + 1, (timestamp >> 16) & 0xFF);
        eeprom_write_byte(record_addr + 2, (timestamp >> 8) & 0xFF);
        eeprom_write_byte(record_addr + 3, timestamp & 0xFF);

        // Write sensor data (4 bytes: sensor_data + padding)
        eeprom_write_byte(record_addr + 4, (sensor_data >> 8) & 0xFF);
        eeprom_write_byte(record_addr + 5, sensor_data & 0xFF);
        eeprom_write_byte(record_addr + 6, 0x00); // Reserved
        eeprom_write_byte(record_addr + 7, 0xAA); // Record marker

        char log_msg[50];
        sprintf(log_msg, "Record %d: Time=%ld, Data=%d\\r\\n",
                current_record, timestamp, sensor_data);
        puts_USART1(log_msg);

        // Update LCD
        char lcd_msg[20];
        sprintf(lcd_msg, "Rec: %d/%d", current_record + 1, max_records);
        lcd_string(4, 0, lcd_msg);

        sprintf(lcd_msg, "Data: %d", sensor_data);
        lcd_string(5, 0, lcd_msg);

        current_record++;
        timestamp++;
        _delay_ms(1000); // Log every second
    }

    // Read back and display log
    puts_USART1("\\r\\nReading back logged data...\\r\\n");
    lcd_string(3, 0, "Reading log...");

    for (uint16_t i = 0; i < current_record && i < 10; i++) // Show first 10 records
    {
        uint16_t record_addr = log_start_addr + (i * record_size);

        // Read timestamp
        uint32_t read_timestamp = 0;
        read_timestamp |= ((uint32_t)eeprom_read_byte(record_addr + 0)) << 24;
        read_timestamp |= ((uint32_t)eeprom_read_byte(record_addr + 1)) << 16;
        read_timestamp |= ((uint32_t)eeprom_read_byte(record_addr + 2)) << 8;
        read_timestamp |= eeprom_read_byte(record_addr + 3);

        // Read sensor data
        uint16_t read_data = 0;
        read_data |= ((uint16_t)eeprom_read_byte(record_addr + 4)) << 8;
        read_data |= eeprom_read_byte(record_addr + 5);

        uint8_t marker = eeprom_read_byte(record_addr + 7);

        char readback_msg[60];
        sprintf(readback_msg, "Playback %d: Time=%ld, Data=%d, Marker=0x%02X\\r\\n",
                i, read_timestamp, read_data, marker);
        puts_USART1(readback_msg);
    }

    char summary[50];
    sprintf(summary, "Data logging complete: %d records saved\\r\\n", current_record);
    puts_USART1(summary);

    if (current_record >= 5)
    {
        lab_score += 200;
        puts_USART1("✓ Data logging system working!\\r\\n");
    }
}

/*
 * =============================================================================
 * LAB EXERCISE 4: ADVANCED APPLICATIONS (15 minutes)
 * =============================================================================
 * OBJECTIVE: Build advanced EEPROM-based applications
 * DIFFICULTY: ★★★★★ (Hard)
 */

void lab_ex4_configuration_storage(void)
{
    /*
     * CHALLENGE: Create a configuration storage and retrieval system
     * TASK: Store and manage device configuration parameters
     * LEARNING: Data structures, checksum validation, configuration management
     */

    puts_USART1("\\r\\n=== Lab 4: Configuration Storage ===\\r\\n");
    puts_USART1("Building configuration management system\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "CONFIG STORAGE");
    lcd_string(1, 0, "Settings mgmt");

    // Configuration structure
    typedef struct
    {
        uint16_t magic_number;     // 0xCAFE
        uint8_t version;           // Config version
        uint8_t device_id;         // Device identifier
        uint16_t sensor_threshold; // Sensor threshold value
        uint8_t led_brightness;    // LED brightness (0-255)
        uint8_t buzzer_enabled;    // Buzzer on/off
        uint8_t reserved[5];       // Future use
        uint8_t checksum;          // Simple checksum
    } device_config_t;

    uint16_t config_addr = 0x7F00; // Store at end of EEPROM
    device_config_t default_config;
    device_config_t loaded_config;

    // Initialize default configuration
    default_config.magic_number = 0xCAFE;
    default_config.version = 1;
    default_config.device_id = 0x42;
    default_config.sensor_threshold = 512;
    default_config.led_brightness = 128;
    default_config.buzzer_enabled = 1;
    for (uint8_t i = 0; i < 5; i++)
        default_config.reserved[i] = 0;

    // Calculate checksum
    uint8_t *config_bytes = (uint8_t *)&default_config;
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < sizeof(device_config_t) - 1; i++)
    {
        checksum ^= config_bytes[i];
    }
    default_config.checksum = checksum;

    // Save default configuration
    puts_USART1("Saving default configuration...\\r\\n");
    lcd_string(3, 0, "Saving config...");

    for (uint8_t i = 0; i < sizeof(device_config_t); i++)
    {
        eeprom_write_byte(config_addr + i, config_bytes[i]);
    }

    char save_msg[50];
    sprintf(save_msg, "Configuration saved (%d bytes)\\r\\n", sizeof(device_config_t));
    puts_USART1(save_msg);

    // Load and verify configuration
    puts_USART1("Loading configuration...\\r\\n");
    lcd_string(3, 0, "Loading config...");

    uint8_t *loaded_bytes = (uint8_t *)&loaded_config;
    for (uint8_t i = 0; i < sizeof(device_config_t); i++)
    {
        loaded_bytes[i] = eeprom_read_byte(config_addr + i);
    }

    // Verify configuration
    if (loaded_config.magic_number != 0xCAFE)
    {
        puts_USART1("❌ Invalid magic number!\\r\\n");
        lcd_string(4, 0, "Invalid magic");
        return;
    }

    // Verify checksum
    checksum = 0;
    for (uint8_t i = 0; i < sizeof(device_config_t) - 1; i++)
    {
        checksum ^= loaded_bytes[i];
    }

    if (checksum != loaded_config.checksum)
    {
        puts_USART1("❌ Checksum mismatch!\\r\\n");
        lcd_string(4, 0, "Bad checksum");
        return;
    }

    puts_USART1("✓ Configuration loaded and verified!\\r\\n");
    lcd_string(4, 0, "Config OK");

    // Display configuration
    puts_USART1("\\r\\nConfiguration Settings:\\r\\n");
    char config_display[80];
    sprintf(config_display, "  Magic: 0x%04X, Version: %d, ID: 0x%02X\\r\\n",
            loaded_config.magic_number, loaded_config.version, loaded_config.device_id);
    puts_USART1(config_display);

    sprintf(config_display, "  Threshold: %d, Brightness: %d, Buzzer: %s\\r\\n",
            loaded_config.sensor_threshold, loaded_config.led_brightness,
            loaded_config.buzzer_enabled ? "ON" : "OFF");
    puts_USART1(config_display);

    sprintf(config_display, "  Checksum: 0x%02X\\r\\n", loaded_config.checksum);
    puts_USART1(config_display);

    // Interactive configuration modification
    puts_USART1("\\r\\nInteractive configuration editor:\\r\\n");
    puts_USART1("Commands: t<value> (threshold), b<value> (brightness), z (toggle buzzer), s (save), q (quit)\\r\\n");

    char command;
    uint8_t modifications = 0;

    while (modifications < 5)
    {
        char status[20];
        sprintf(status, "T:%d B:%d Z:%s", loaded_config.sensor_threshold,
                loaded_config.led_brightness, loaded_config.buzzer_enabled ? "ON" : "OFF");
        lcd_string(5, 0, status);

        puts_USART1("Config> ");
        command = getch_USART1();
        putch_USART1(command);
        puts_USART1("\\r\\n");

        switch (command)
        {
        case 't':
            puts_USART1("Enter threshold (0-1023): ");
            // Simplified input - would normally parse full number
            loaded_config.sensor_threshold = 600; // Demo value
            puts_USART1("600\\r\\nThreshold updated\\r\\n");
            modifications++;
            break;

        case 'b':
            puts_USART1("Enter brightness (0-255): ");
            loaded_config.led_brightness = 200; // Demo value
            puts_USART1("200\\r\\nBrightness updated\\r\\n");
            modifications++;
            break;

        case 'z':
            loaded_config.buzzer_enabled = !loaded_config.buzzer_enabled;
            puts_USART1("Buzzer toggled\\r\\n");
            modifications++;
            break;

        case 's':
            // Recalculate checksum and save
            checksum = 0;
            for (uint8_t i = 0; i < sizeof(device_config_t) - 1; i++)
            {
                checksum ^= loaded_bytes[i];
            }
            loaded_config.checksum = checksum;

            // Save updated configuration
            for (uint8_t i = 0; i < sizeof(device_config_t); i++)
            {
                eeprom_write_byte(config_addr + i, loaded_bytes[i]);
            }
            puts_USART1("Configuration saved!\\r\\n");
            modifications++;
            break;

        case 'q':
            puts_USART1("Exiting configuration editor\\r\\n");
            goto config_exit;

        default:
            puts_USART1("Invalid command\\r\\n");
            break;
        }
    }

config_exit:
    if (modifications >= 3)
    {
        lab_score += 250;
        puts_USART1("✓ Configuration system mastered!\\r\\n");
    }
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
    puts_USART1("     SPI EEPROM MEMORY - LAB EXERCISES       \\r\\n");
    puts_USART1("==============================================\\r\\n");
    puts_USART1("1. EEPROM Initialization & Basic R/W        \\r\\n");
    puts_USART1("2. Block Operations & Performance Testing   \\r\\n");
    puts_USART1("3. Data Logging & Circular Buffers          \\r\\n");
    puts_USART1("4. Advanced Configuration Storage            \\r\\n");
    puts_USART1("                                              \\r\\n");
    puts_USART1("0. Run All Exercises                         \\r\\n");
    puts_USART1("X. Exit Lab                                   \\r\\n");
    puts_USART1("==============================================\\r\\n");
    char score_msg[50];
    sprintf(score_msg, "Current Score: %d points\\r\\n", lab_score);
    puts_USART1(score_msg);
    char memory_msg[60];
    sprintf(memory_msg, "Memory: %ld bytes written, %ld bytes read\\r\\n", bytes_written, bytes_read);
    puts_USART1(memory_msg);
    puts_USART1("Select exercise (1-4, 0, X): ");
}

int main(void)
{
    init_devices();

    puts_USART1("\\r\\n*** SPI EEPROM MEMORY LAB SESSION ***\\r\\n");
    puts_USART1("Welcome to hands-on EEPROM programming!\\r\\n");
    puts_USART1("Ensure SPI EEPROM is properly connected\\r\\n");
    puts_USART1("Connections: MOSI, MISO, SCK, CS pins\\r\\n");

    lcd_clear();
    lcd_string(1, 0, "SPI EEPROM LAB");
    lcd_string(2, 0, "Check connections");
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
            lab_ex1_eeprom_initialization();
            lab_ex1_memory_test_patterns();
            break;

        case '2':
            lab_ex2_block_operations();
            break;

        case '3':
            lab_ex3_data_logging();
            break;

        case '4':
            lab_ex4_configuration_storage();
            break;

        case '0':
            puts_USART1("\\r\\n*** RUNNING ALL EXERCISES ***\\r\\n");
            lab_ex1_eeprom_initialization();
            lab_ex1_memory_test_patterns();
            lab_ex2_block_operations();
            lab_ex3_data_logging();
            lab_ex4_configuration_storage();

            char final_buffer[80];
            sprintf(final_buffer, "\\r\\n*** ALL EXERCISES COMPLETE! ***\\r\\nFinal Score: %d points\\r\\n", lab_score);
            puts_USART1(final_buffer);
            break;

        case 'X':
        case 'x':
            puts_USART1("\\r\\nExiting lab. Great work on EEPROM!\\r\\n");
            puts_USART1("Remember: EEPROM has limited write cycles!\\r\\n");
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