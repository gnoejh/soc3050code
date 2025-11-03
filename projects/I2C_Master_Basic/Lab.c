/*
 * =============================================================================
 * I2C MASTER BASIC - HANDS-ON LAB EXERCISES
 * =============================================================================
 * ATmega128 Educational Framework - Lab Session
 *
 * OBJECTIVE: Master I2C (TWI) communication as bus master
 * DURATION: 75 minutes
 * DIFFICULTY: Intermediate-Advanced
 *
 * STUDENTS WILL:
 * - Initialize and configure I2C/TWI interface
 * - Implement master read and write operations
 * - Handle I2C addressing and acknowledgments
 * - Communicate with multiple slave devices
 * - Implement error detection and recovery
 *
 * HARDWARE REQUIRED:
 * - ATmega128 board
 * - Pull-up resistors (4.7kΩ) on SDA and SCL lines
 * - I2C slave devices (EEPROM, RTC, sensors)
 * - Example devices: 24LC256 EEPROM, DS1307 RTC, LM75 temperature sensor
 * - Optional: I2C bus analyzer or logic analyzer
 * - Status LEDs for communication indication
 *
 * I2C/TWI PROTOCOL FEATURES:
 * - 7-bit and 10-bit addressing modes
 * - Standard (100kHz) and Fast (400kHz) modes
 * - Master transmitter and receiver modes
 * - Bus arbitration and error handling
 * - Multi-master capability (advanced)
 *
 * LAB STRUCTURE:
 * - Exercise 1: I2C initialization and device scanning (20 min)
 * - Exercise 2: Basic read/write operations (20 min)
 * - Exercise 3: Multi-device communication (20 min)
 * - Exercise 4: Advanced protocols and error handling (15 min)
 *
 * =============================================================================
 */

#include "config.h"

// I2C/TWI Configuration
#define I2C_FREQ_100K 100000UL // Standard mode
#define I2C_FREQ_400K 400000UL // Fast mode
#define I2C_TIMEOUT 1000       // Operation timeout in ms

// Common I2C device addresses (7-bit)
#define EEPROM_ADDR 0x50      // 24LC256 EEPROM
#define RTC_ADDR 0x68         // DS1307 RTC
#define TEMP_SENSOR_ADDR 0x48 // LM75 temperature sensor
#define IO_EXPANDER_ADDR 0x20 // PCF8574 I/O expander

// I2C status codes (from ATmega128 datasheet)
#define TWI_START 0x08        // START transmitted
#define TWI_REP_START 0x10    // Repeated START transmitted
#define TWI_MT_SLA_ACK 0x18   // SLA+W transmitted, ACK received
#define TWI_MT_SLA_NACK 0x20  // SLA+W transmitted, NACK received
#define TWI_MT_DATA_ACK 0x28  // Data transmitted, ACK received
#define TWI_MT_DATA_NACK 0x30 // Data transmitted, NACK received
#define TWI_MR_SLA_ACK 0x40   // SLA+R transmitted, ACK received
#define TWI_MR_SLA_NACK 0x48  // SLA+R transmitted, NACK received
#define TWI_MR_DATA_ACK 0x50  // Data received, ACK transmitted
#define TWI_MR_DATA_NACK 0x58 // Data received, NACK transmitted

// Communication indicators
#define I2C_SCL_LED_PIN 6   // PB6 - SCL activity indicator
#define I2C_SDA_LED_PIN 7   // PB7 - SDA activity indicator
#define I2C_ERROR_LED_PIN 5 // PB5 - Error indicator

// Lab session variables
uint16_t lab_score = 0;
uint32_t i2c_transactions = 0;
uint16_t successful_reads = 0;
uint16_t successful_writes = 0;
uint16_t i2c_errors = 0;
uint8_t devices_found = 0;

// Device detection results
uint8_t detected_devices[16]; // Store up to 16 detected device addresses
uint8_t device_count = 0;

/*
 * =============================================================================
 * I2C/TWI COMMUNICATION FUNCTIONS
 * =============================================================================
 */

void i2c_init(uint32_t frequency)
{
    // Configure I2C indicator LEDs
    DDRB |= (1 << I2C_SCL_LED_PIN) | (1 << I2C_SDA_LED_PIN) | (1 << I2C_ERROR_LED_PIN);
    PORTB &= ~((1 << I2C_SCL_LED_PIN) | (1 << I2C_SDA_LED_PIN) | (1 << I2C_ERROR_LED_PIN));

    // Calculate TWI bit rate register value
    // TWBR = (F_CPU / frequency - 16) / 2
    uint8_t twbr_value = ((F_CPU / frequency) - 16) / 2;

    // Set bit rate
    TWBR = twbr_value;

    // Set prescaler to 1 (TWPS1:0 = 00)
    TWSR = 0x00;

    // Enable TWI
    TWCR = (1 << TWEN);

    char init_msg[60];
    sprintf(init_msg, "I2C initialized at %ld Hz (TWBR=%d)\\r\\n", frequency, twbr_value);
    puts_USART1(init_msg);
}

uint8_t i2c_start(void)
{
    // Send START condition
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

    // Wait for operation to complete
    uint16_t timeout = I2C_TIMEOUT;
    while (!(TWCR & (1 << TWINT)) && timeout--)
    {
        _delay_ms(1);
    }

    if (timeout == 0)
    {
        PORTB |= (1 << I2C_ERROR_LED_PIN);
        return 0; // Timeout
    }

    // Check status
    uint8_t status = TWSR & 0xF8;
    PORTB |= (1 << I2C_SCL_LED_PIN);
    _delay_ms(10);
    PORTB &= ~(1 << I2C_SCL_LED_PIN);

    return (status == TWI_START || status == TWI_REP_START);
}

uint8_t i2c_write(uint8_t data)
{
    // Load data into TWDR
    TWDR = data;

    // Start transmission
    TWCR = (1 << TWINT) | (1 << TWEN);

    // Wait for operation to complete
    uint16_t timeout = I2C_TIMEOUT;
    while (!(TWCR & (1 << TWINT)) && timeout--)
    {
        _delay_ms(1);
    }

    if (timeout == 0)
    {
        PORTB |= (1 << I2C_ERROR_LED_PIN);
        return 0; // Timeout
    }

    // Check status
    uint8_t status = TWSR & 0xF8;
    PORTB |= (1 << I2C_SDA_LED_PIN);
    _delay_ms(10);
    PORTB &= ~(1 << I2C_SDA_LED_PIN);

    return (status == TWI_MT_SLA_ACK || status == TWI_MT_DATA_ACK);
}

uint8_t i2c_read_ack(void)
{
    // Start reception with ACK
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);

    // Wait for operation to complete
    uint16_t timeout = I2C_TIMEOUT;
    while (!(TWCR & (1 << TWINT)) && timeout--)
    {
        _delay_ms(1);
    }

    if (timeout == 0)
    {
        PORTB |= (1 << I2C_ERROR_LED_PIN);
        return 0xFF; // Timeout
    }

    PORTB |= (1 << I2C_SDA_LED_PIN);
    _delay_ms(10);
    PORTB &= ~(1 << I2C_SDA_LED_PIN);

    return TWDR;
}

uint8_t i2c_read_nack(void)
{
    // Start reception with NACK (last byte)
    TWCR = (1 << TWINT) | (1 << TWEN);

    // Wait for operation to complete
    uint16_t timeout = I2C_TIMEOUT;
    while (!(TWCR & (1 << TWINT)) && timeout--)
    {
        _delay_ms(1);
    }

    if (timeout == 0)
    {
        PORTB |= (1 << I2C_ERROR_LED_PIN);
        return 0xFF; // Timeout
    }

    PORTB |= (1 << I2C_SDA_LED_PIN);
    _delay_ms(10);
    PORTB &= ~(1 << I2C_SDA_LED_PIN);

    return TWDR;
}

void i2c_stop(void)
{
    // Send STOP condition
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);

    // Wait for STOP to complete
    while (TWCR & (1 << TWSTO))
        ;

    PORTB |= (1 << I2C_SCL_LED_PIN);
    _delay_ms(10);
    PORTB &= ~(1 << I2C_SCL_LED_PIN);
}

uint8_t i2c_detect_device(uint8_t address)
{
    i2c_transactions++;

    if (!i2c_start())
    {
        i2c_stop();
        i2c_errors++;
        return 0;
    }

    if (!i2c_write((address << 1) | 0))
    { // Write address
        i2c_stop();
        return 0;
    }

    i2c_stop();
    return 1; // Device responded
}

uint8_t i2c_write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t data)
{
    i2c_transactions++;

    if (!i2c_start())
    {
        i2c_stop();
        i2c_errors++;
        return 0;
    }

    if (!i2c_write((device_addr << 1) | 0))
    { // Device address + write
        i2c_stop();
        i2c_errors++;
        return 0;
    }

    if (!i2c_write(reg_addr))
    { // Register address
        i2c_stop();
        i2c_errors++;
        return 0;
    }

    if (!i2c_write(data))
    { // Data
        i2c_stop();
        i2c_errors++;
        return 0;
    }

    i2c_stop();
    successful_writes++;
    return 1;
}

uint8_t i2c_read_byte(uint8_t device_addr, uint8_t reg_addr)
{
    i2c_transactions++;

    // Write register address
    if (!i2c_start())
    {
        i2c_stop();
        i2c_errors++;
        return 0xFF;
    }

    if (!i2c_write((device_addr << 1) | 0))
    { // Device address + write
        i2c_stop();
        i2c_errors++;
        return 0xFF;
    }

    if (!i2c_write(reg_addr))
    { // Register address
        i2c_stop();
        i2c_errors++;
        return 0xFF;
    }

    // Repeated start for read
    if (!i2c_start())
    {
        i2c_stop();
        i2c_errors++;
        return 0xFF;
    }

    if (!i2c_write((device_addr << 1) | 1))
    { // Device address + read
        i2c_stop();
        i2c_errors++;
        return 0xFF;
    }

    uint8_t data = i2c_read_nack();
    i2c_stop();

    successful_reads++;
    return data;
}

/*
 * =============================================================================
 * LAB EXERCISE 1: I2C INITIALIZATION AND DEVICE SCANNING (20 minutes)
 * =============================================================================
 * OBJECTIVE: Set up I2C bus and discover connected devices
 * DIFFICULTY: ★★★☆☆ (Medium)
 */

void lab_ex1_i2c_initialization(void)
{
    /*
     * CHALLENGE: Initialize I2C interface and verify bus operation
     * TASK: Configure I2C parameters and test bus functionality
     * LEARNING: I2C initialization, bus timing, pull-up requirements
     */

    puts_USART1("\\r\\n=== Lab 1: I2C Initialization ===\\r\\n");
    puts_USART1("Setting up I2C/TWI interface\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "I2C MASTER BASIC");
    lcd_string(1, 0, "Initialization");

    // Test different I2C frequencies
    puts_USART1("Testing I2C frequencies...\\r\\n");

    // Standard mode (100kHz)
    puts_USART1("Initializing at 100kHz (Standard mode)\\r\\n");
    lcd_string(3, 0, "Mode: Standard");
    i2c_init(I2C_FREQ_100K);
    _delay_ms(1000);

    // Test bus with simple operation
    puts_USART1("Testing bus operation...\\r\\n");
    if (i2c_start())
    {
        puts_USART1("✓ START condition successful\\r\\n");
        i2c_stop();
        puts_USART1("✓ STOP condition successful\\r\\n");
        lcd_string(4, 0, "Bus: OK");
        lab_score += 50;
    }
    else
    {
        puts_USART1("❌ Bus operation failed\\r\\n");
        lcd_string(4, 0, "Bus: ERROR");
    }

    _delay_ms(2000);

    // Fast mode (400kHz)
    puts_USART1("\\r\\nInitializing at 400kHz (Fast mode)\\r\\n");
    lcd_string(3, 0, "Mode: Fast    ");
    i2c_init(I2C_FREQ_400K);

    // Test fast mode operation
    if (i2c_start())
    {
        puts_USART1("✓ Fast mode START successful\\r\\n");
        i2c_stop();
        puts_USART1("✓ Fast mode STOP successful\\r\\n");
        lcd_string(4, 0, "Fast: OK");
        lab_score += 50;
    }
    else
    {
        puts_USART1("❌ Fast mode operation failed\\r\\n");
        lcd_string(4, 0, "Fast: ERROR");
    }

    // Return to standard mode for compatibility
    i2c_init(I2C_FREQ_100K);
    puts_USART1("Returned to 100kHz for device compatibility\\r\\n");

    _delay_ms(1000);
}

void lab_ex1_device_scanning(void)
{
    /*
     * CHALLENGE: Scan I2C bus to discover connected devices
     * TASK: Test all possible 7-bit addresses and identify responding devices
     * LEARNING: I2C addressing, device detection, bus probing
     */

    puts_USART1("\\r\\n=== Lab 1.2: Device Scanning ===\\r\\n");
    puts_USART1("Scanning I2C bus for devices...\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "DEVICE SCANNING");
    lcd_string(1, 0, "I2C bus probe");

    device_count = 0;
    memset(detected_devices, 0, sizeof(detected_devices));

    puts_USART1("Address scan (7-bit addresses):\\r\\n");
    puts_USART1("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\\r\\n");

    for (uint8_t row = 0; row < 8; row++)
    {
        char row_output[60];
        sprintf(row_output, "%02X: ", row * 16);

        for (uint8_t col = 0; col < 16; col++)
        {
            uint8_t address = row * 16 + col;

            // Skip invalid addresses (0x00-0x07 and 0x78-0x7F are reserved)
            if (address < 0x08 || address > 0x77)
            {
                strcat(row_output, "   ");
                continue;
            }

            char lcd_scan[20];
            sprintf(lcd_scan, "Scan: 0x%02X", address);
            lcd_string(3, 0, lcd_scan);

            if (i2c_detect_device(address))
            {
                char addr_str[4];
                sprintf(addr_str, "%02X ", address);
                strcat(row_output, addr_str);

                // Store detected device
                if (device_count < 16)
                {
                    detected_devices[device_count++] = address;
                }

                devices_found++;
            }
            else
            {
                strcat(row_output, "-- ");
            }

            _delay_ms(50); // Small delay between probes
        }

        strcat(row_output, "\\r\\n");
        puts_USART1(row_output);
    }

    char scan_summary[50];
    sprintf(scan_summary, "\\r\\nScan complete: %d devices found\\r\\n", device_count);
    puts_USART1(scan_summary);

    char lcd_found[20];
    sprintf(lcd_found, "Found: %d devices", device_count);
    lcd_string(4, 0, lcd_found);

    // Display found devices with likely identification
    if (device_count > 0)
    {
        puts_USART1("Detected devices:\\r\\n");

        for (uint8_t i = 0; i < device_count; i++)
        {
            uint8_t addr = detected_devices[i];
            char device_info[60];

            // Identify common devices by address
            sprintf(device_info, "  0x%02X - ", addr);

            switch (addr)
            {
            case 0x50:
            case 0x51:
            case 0x52:
            case 0x53:
            case 0x54:
            case 0x55:
            case 0x56:
            case 0x57:
                strcat(device_info, "EEPROM (24LCxx)");
                break;
            case 0x68:
                strcat(device_info, "RTC (DS1307/DS3231)");
                break;
            case 0x48:
            case 0x49:
            case 0x4A:
            case 0x4B:
                strcat(device_info, "Temperature sensor (LM75/DS18B20)");
                break;
            case 0x20:
            case 0x21:
            case 0x22:
            case 0x23:
            case 0x24:
            case 0x25:
            case 0x26:
            case 0x27:
                strcat(device_info, "I/O Expander (PCF8574)");
                break;
            case 0x3C:
            case 0x3D:
                strcat(device_info, "OLED Display (SSD1306)");
                break;
            case 0x1E:
                strcat(device_info, "Magnetometer (HMC5883L)");
                break;
            case 0x77:
                strcat(device_info, "Pressure sensor (BMP180/BMP280)");
                break;
            default:
                strcat(device_info, "Unknown device");
                break;
            }

            strcat(device_info, "\\r\\n");
            puts_USART1(device_info);
        }

        char lcd_devices[20];
        sprintf(lcd_devices, "0x%02X 0x%02X 0x%02X...",
                detected_devices[0],
                device_count > 1 ? detected_devices[1] : 0,
                device_count > 2 ? detected_devices[2] : 0);
        lcd_string(5, 0, lcd_devices);

        lab_score += (device_count * 25);
    }
    else
    {
        puts_USART1("No I2C devices detected.\\r\\n");
        puts_USART1("Check connections and pull-up resistors.\\r\\n");
        lcd_string(5, 0, "No devices found");
    }

    _delay_ms(3000);
}

/*
 * =============================================================================
 * LAB EXERCISE 2: BASIC READ/WRITE OPERATIONS (20 minutes)
 * =============================================================================
 * OBJECTIVE: Implement fundamental I2C read and write transactions
 * DIFFICULTY: ★★★★☆ (Medium-Hard)
 */

void lab_ex2_basic_operations(void)
{
    /*
     * CHALLENGE: Perform basic I2C read and write operations
     * TASK: Communicate with detected devices using standard protocols
     * LEARNING: I2C transactions, register access, data validation
     */

    puts_USART1("\\r\\n=== Lab 2: Basic Read/Write Operations ===\\r\\n");
    puts_USART1("Testing I2C read and write transactions\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "I2C READ/WRITE");
    lcd_string(1, 0, "Basic operations");

    if (device_count == 0)
    {
        puts_USART1("No devices available for testing\\r\\n");
        puts_USART1("Using simulated device operations\\r\\n");
        lcd_string(3, 0, "Simulated mode");

        // Simulate operations for demonstration
        for (uint8_t sim = 0; sim < 5; sim++)
        {
            char sim_msg[50];
            sprintf(sim_msg, "Simulated write to reg 0x%02X: 0x%02X\\r\\n", sim, sim * 16);
            puts_USART1(sim_msg);

            sprintf(sim_msg, "Simulated read from reg 0x%02X: 0x%02X\\r\\n", sim, sim * 16 + 5);
            puts_USART1(sim_msg);

            successful_writes++;
            successful_reads++;

            char lcd_sim[20];
            sprintf(lcd_sim, "Sim: %d/5", sim + 1);
            lcd_string(4, 0, lcd_sim);

            _delay_ms(800);
        }

        lab_score += 100;
        return;
    }

    // Test with first detected device
    uint8_t test_device = detected_devices[0];

    char test_msg[60];
    sprintf(test_msg, "Testing with device at address 0x%02X\\r\\n", test_device);
    puts_USART1(test_msg);

    char lcd_device[20];
    sprintf(lcd_device, "Device: 0x%02X", test_device);
    lcd_string(3, 0, lcd_device);

    // Test different register addresses and values
    uint8_t test_registers[] = {0x00, 0x01, 0x02, 0x10, 0x20};
    uint8_t test_values[] = {0xAA, 0x55, 0xFF, 0x00, 0x42};

    for (uint8_t test = 0; test < 5; test++)
    {
        uint8_t reg = test_registers[test];
        uint8_t value = test_values[test];

        char operation_msg[60];
        sprintf(operation_msg, "Test %d: Write 0x%02X to register 0x%02X\\r\\n",
                test + 1, value, reg);
        puts_USART1(operation_msg);

        // Attempt write operation
        if (i2c_write_byte(test_device, reg, value))
        {
            puts_USART1("  Write: SUCCESS\\r\\n");

            // Small delay for device processing
            _delay_ms(10);

            // Attempt read back
            uint8_t read_value = i2c_read_byte(test_device, reg);

            char read_msg[50];
            sprintf(read_msg, "  Read back: 0x%02X\\r\\n", read_value);
            puts_USART1(read_msg);

            if (read_value == value)
            {
                puts_USART1("  Verification: PASS\\r\\n");
                lab_score += 50;
            }
            else
            {
                puts_USART1("  Verification: Data mismatch\\r\\n");
            }
        }
        else
        {
            puts_USART1("  Write: FAILED\\r\\n");
        }

        char lcd_test[20];
        sprintf(lcd_test, "Test: %d/5", test + 1);
        lcd_string(4, 0, lcd_test);

        char lcd_stats[20];
        sprintf(lcd_stats, "W:%d R:%d E:%d", successful_writes, successful_reads, i2c_errors);
        lcd_string(5, 0, lcd_stats);

        _delay_ms(1000);
    }

    // Summary of operations
    char summary[80];
    sprintf(summary, "Operations complete: %d writes, %d reads, %d errors\\r\\n",
            successful_writes, successful_reads, i2c_errors);
    puts_USART1(summary);
}

void lab_ex2_eeprom_testing(void)
{
    /*
     * CHALLENGE: Implement EEPROM-specific I2C operations
     * TASK: Handle EEPROM addressing and page operations
     * LEARNING: Device-specific protocols, memory addressing, timing requirements
     */

    puts_USART1("\\r\\n=== Lab 2.2: EEPROM Testing ===\\r\\n");
    puts_USART1("Testing EEPROM-specific operations\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "EEPROM TESTING");
    lcd_string(1, 0, "Memory operations");

    // Look for EEPROM device (address 0x50-0x57)
    uint8_t eeprom_addr = 0;
    for (uint8_t i = 0; i < device_count; i++)
    {
        if (detected_devices[i] >= 0x50 && detected_devices[i] <= 0x57)
        {
            eeprom_addr = detected_devices[i];
            break;
        }
    }

    if (eeprom_addr == 0)
    {
        puts_USART1("No EEPROM device found, using simulation\\r\\n");
        lcd_string(3, 0, "EEPROM: Simulated");

        // Simulate EEPROM operations
        for (uint8_t addr = 0; addr < 8; addr++)
        {
            char sim_msg[50];
            sprintf(sim_msg, "EEPROM[0x%04X] = 0x%02X\\r\\n", addr * 256, addr * 16);
            puts_USART1(sim_msg);
            successful_writes++;
            successful_reads++;
        }

        lab_score += 100;
        return;
    }

    char eeprom_msg[50];
    sprintf(eeprom_msg, "Testing EEPROM at address 0x%02X\\r\\n", eeprom_addr);
    puts_USART1(eeprom_msg);

    char lcd_eeprom[20];
    sprintf(lcd_eeprom, "EEPROM: 0x%02X", eeprom_addr);
    lcd_string(3, 0, lcd_eeprom);

    // EEPROM write operation (with 16-bit addressing)
    uint16_t eeprom_addresses[] = {0x0000, 0x0010, 0x0100, 0x1000, 0x7FF0};
    uint8_t eeprom_data[] = {0x11, 0x22, 0x33, 0x44, 0x55};

    for (uint8_t test = 0; test < 5; test++)
    {
        uint16_t addr = eeprom_addresses[test];
        uint8_t data = eeprom_data[test];

        char write_msg[60];
        sprintf(write_msg, "Writing 0x%02X to EEPROM address 0x%04X\\r\\n", data, addr);
        puts_USART1(write_msg);

        // EEPROM write with 16-bit addressing
        i2c_transactions++;
        if (i2c_start() &&
            i2c_write((eeprom_addr << 1) | 0) && // Device address + write
            i2c_write((addr >> 8) & 0xFF) &&     // High address byte
            i2c_write(addr & 0xFF) &&            // Low address byte
            i2c_write(data))
        { // Data

            i2c_stop();
            puts_USART1("  EEPROM write: SUCCESS\\r\\n");
            successful_writes++;

            // EEPROM write cycle delay
            _delay_ms(10);

            // Read back for verification
            if (i2c_start() &&
                i2c_write((eeprom_addr << 1) | 0) && // Device address + write
                i2c_write((addr >> 8) & 0xFF) &&     // High address byte
                i2c_write(addr & 0xFF) &&            // Low address byte
                i2c_start() &&                       // Repeated start
                i2c_write((eeprom_addr << 1) | 1))
            { // Device address + read

                uint8_t read_data = i2c_read_nack();
                i2c_stop();

                char read_msg[50];
                sprintf(read_msg, "  EEPROM read: 0x%02X\\r\\n", read_data);
                puts_USART1(read_msg);

                successful_reads++;

                if (read_data == data)
                {
                    puts_USART1("  EEPROM verification: PASS\\r\\n");
                    lab_score += 40;
                }
                else
                {
                    puts_USART1("  EEPROM verification: FAIL\\r\\n");
                }
            }
            else
            {
                i2c_stop();
                puts_USART1("  EEPROM read: FAILED\\r\\n");
                i2c_errors++;
            }
        }
        else
        {
            i2c_stop();
            puts_USART1("  EEPROM write: FAILED\\r\\n");
            i2c_errors++;
        }

        char lcd_test[20];
        sprintf(lcd_test, "EEPROM: %d/5", test + 1);
        lcd_string(4, 0, lcd_test);

        _delay_ms(1000);
    }

    char lcd_final[20];
    sprintf(lcd_final, "EEPROM complete");
    lcd_string(5, 0, lcd_final);
}

/*
 * =============================================================================
 * LAB EXERCISE 3: MULTI-DEVICE COMMUNICATION (20 minutes)
 * =============================================================================
 * OBJECTIVE: Communicate with multiple I2C devices simultaneously
 * DIFFICULTY: ★★★★☆ (Medium-Hard)
 */

void lab_ex3_multi_device(void)
{
    /*
     * CHALLENGE: Manage communication with multiple I2C devices
     * TASK: Coordinate data exchange between different device types
     * LEARNING: Device coordination, data correlation, bus arbitration
     */

    puts_USART1("\\r\\n=== Lab 3: Multi-Device Communication ===\\r\\n");
    puts_USART1("Coordinating multiple I2C devices\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "MULTI-DEVICE");
    lcd_string(1, 0, "Communication");

    if (device_count < 2)
    {
        puts_USART1("Need at least 2 devices for multi-device test\\r\\n");
        puts_USART1("Simulating multi-device scenario\\r\\n");

        // Simulate multi-device operations
        char sim_devices[][20] = {"EEPROM", "RTC", "TempSensor", "IOExpander"};

        for (uint8_t cycle = 0; cycle < 6; cycle++)
        {
            for (uint8_t dev = 0; dev < 4; dev++)
            {
                char sim_msg[60];
                sprintf(sim_msg, "Cycle %d - %s: Read=0x%02X, Write=0x%02X\\r\\n",
                        cycle + 1, sim_devices[dev], (cycle * 4 + dev) & 0xFF,
                        ((cycle + 1) * 4 + dev) & 0xFF);
                puts_USART1(sim_msg);

                successful_reads++;
                successful_writes++;
            }

            char lcd_cycle[20];
            sprintf(lcd_cycle, "Sim cycle: %d/6", cycle + 1);
            lcd_string(3, 0, lcd_cycle);

            _delay_ms(1000);
        }

        lab_score += 150;
        return;
    }

    char multi_msg[50];
    sprintf(multi_msg, "Testing with %d detected devices\\r\\n", device_count);
    puts_USART1(multi_msg);

    char lcd_count[20];
    sprintf(lcd_count, "Devices: %d", device_count);
    lcd_string(3, 0, lcd_count);

    // Multi-device polling cycle
    for (uint8_t cycle = 0; cycle < 8; cycle++)
    {
        char cycle_msg[40];
        sprintf(cycle_msg, "\\r\\nCommunication cycle %d:\\r\\n", cycle + 1);
        puts_USART1(cycle_msg);

        char lcd_cycle[20];
        sprintf(lcd_cycle, "Cycle: %d/8", cycle + 1);
        lcd_string(4, 0, lcd_cycle);

        uint8_t successful_devices = 0;

        // Communicate with each detected device
        for (uint8_t dev = 0; dev < device_count && dev < 8; dev++)
        {
            uint8_t addr = detected_devices[dev];
            uint8_t reg = cycle % 4; // Cycle through registers 0-3
            uint8_t write_value = (cycle * 8 + dev) & 0xFF;

            char dev_msg[60];
            sprintf(dev_msg, "  Device 0x%02X: ", addr);

            // Attempt communication
            if (i2c_write_byte(addr, reg, write_value))
            {
                strcat(dev_msg, "Write OK, ");

                uint8_t read_value = i2c_read_byte(addr, reg);
                char read_part[20];
                sprintf(read_part, "Read=0x%02X", read_value);
                strcat(dev_msg, read_part);

                successful_devices++;
            }
            else
            {
                strcat(dev_msg, "Communication FAILED");
            }

            strcat(dev_msg, "\\r\\n");
            puts_USART1(dev_msg);

            _delay_ms(200); // Small delay between devices
        }

        char success_msg[50];
        sprintf(success_msg, "  Cycle %d: %d/%d devices responded\\r\\n",
                cycle + 1, successful_devices, device_count);
        puts_USART1(success_msg);

        char lcd_success[20];
        sprintf(lcd_success, "OK: %d/%d", successful_devices, device_count);
        lcd_string(5, 0, lcd_success);

        if (successful_devices == device_count)
        {
            lab_score += 25;
        }

        _delay_ms(1000);
    }

    char final_multi[60];
    sprintf(final_multi, "Multi-device test complete: %ld transactions\\r\\n",
            i2c_transactions);
    puts_USART1(final_multi);
}

/*
 * =============================================================================
 * LAB EXERCISE 4: ADVANCED PROTOCOLS AND ERROR HANDLING (15 minutes)
 * =============================================================================
 * OBJECTIVE: Implement advanced I2C features and robust error handling
 * DIFFICULTY: ★★★★★ (Hard)
 */

void lab_ex4_advanced_protocols(void)
{
    /*
     * CHALLENGE: Implement advanced I2C protocols and error recovery
     * TASK: Handle complex transactions, timeouts, and bus recovery
     * LEARNING: Error handling, bus recovery, advanced protocols
     */

    puts_USART1("\\r\\n=== Lab 4: Advanced Protocols ===\\r\\n");
    puts_USART1("Testing advanced I2C features and error handling\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "ADVANCED I2C");
    lcd_string(1, 0, "Error handling");

    // Test 1: Bus recovery procedure
    puts_USART1("Test 1: Bus recovery procedure\\r\\n");
    lcd_string(3, 0, "Test: Bus recovery");

    // Simulate bus stuck condition and recovery
    puts_USART1("Simulating bus stuck condition...\\r\\n");

    // Attempt to clear any stuck condition
    for (uint8_t recovery = 0; recovery < 9; recovery++)
    {
        // Toggle SCL to clear any stuck slave
        DDRD |= (1 << PD0);   // SCL as output (normally not recommended)
        PORTD &= ~(1 << PD0); // SCL low
        _delay_us(5);
        PORTD |= (1 << PD0); // SCL high
        _delay_us(5);
        DDRD &= ~(1 << PD0); // SCL back to input (with pull-up)
    }

    // Reinitialize I2C
    i2c_init(I2C_FREQ_100K);

    // Test bus after recovery
    if (i2c_start())
    {
        puts_USART1("✓ Bus recovery successful\\r\\n");
        i2c_stop();
        lab_score += 50;
    }
    else
    {
        puts_USART1("❌ Bus recovery failed\\r\\n");
    }

    _delay_ms(1000);

    // Test 2: Timeout handling
    puts_USART1("\\r\\nTest 2: Timeout handling\\r\\n");
    lcd_string(3, 0, "Test: Timeouts  ");

    // Test communication with non-existent device
    uint8_t fake_addr = 0x7E; // Likely non-existent address
    puts_USART1("Testing with non-existent device (0x7E)...\\r\\n");

    uint32_t start_errors = i2c_errors;

    for (uint8_t timeout_test = 0; timeout_test < 3; timeout_test++)
    {
        if (!i2c_detect_device(fake_addr))
        {
            char timeout_msg[50];
            sprintf(timeout_msg, "  Timeout test %d: Correctly detected failure\\r\\n",
                    timeout_test + 1);
            puts_USART1(timeout_msg);
        }
        _delay_ms(500);
    }

    uint32_t timeout_errors = i2c_errors - start_errors;
    char error_msg[50];
    sprintf(error_msg, "Generated %ld timeout errors (expected)\\r\\n", timeout_errors);
    puts_USART1(error_msg);

    if (timeout_errors >= 3)
    {
        lab_score += 50;
    }

    // Test 3: Block read/write operations
    puts_USART1("\\r\\nTest 3: Block operations\\r\\n");
    lcd_string(3, 0, "Test: Block ops ");

    if (device_count > 0)
    {
        uint8_t block_device = detected_devices[0];
        uint8_t block_data[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
        uint8_t read_block[8];

        char block_msg[50];
        sprintf(block_msg, "Block operations with device 0x%02X\\r\\n", block_device);
        puts_USART1(block_msg);

        // Block write simulation
        uint8_t block_success = 1;
        for (uint8_t i = 0; i < 8; i++)
        {
            if (!i2c_write_byte(block_device, i, block_data[i]))
            {
                block_success = 0;
                break;
            }
            _delay_ms(10);
        }

        if (block_success)
        {
            puts_USART1("  Block write: SUCCESS\\r\\n");

            // Block read
            for (uint8_t i = 0; i < 8; i++)
            {
                read_block[i] = i2c_read_byte(block_device, i);
                _delay_ms(5);
            }

            puts_USART1("  Block read: ");
            for (uint8_t i = 0; i < 8; i++)
            {
                char byte_str[5];
                sprintf(byte_str, "%02X ", read_block[i]);
                puts_USART1(byte_str);
            }
            puts_USART1("\\r\\n");

            lab_score += 100;
        }
        else
        {
            puts_USART1("  Block write: FAILED\\r\\n");
        }
    }
    else
    {
        puts_USART1("No devices available for block testing\\r\\n");
        lab_score += 50; // Partial credit
    }

    // Final statistics
    puts_USART1("\\r\\n=== FINAL I2C STATISTICS ===\\r\\n");
    char final_stats[80];
    sprintf(final_stats, "Total transactions: %ld\\r\\n", i2c_transactions);
    puts_USART1(final_stats);

    sprintf(final_stats, "Successful reads: %d\\r\\n", successful_reads);
    puts_USART1(final_stats);

    sprintf(final_stats, "Successful writes: %d\\r\\n", successful_writes);
    puts_USART1(final_stats);

    sprintf(final_stats, "Errors encountered: %d\\r\\n", i2c_errors);
    puts_USART1(final_stats);

    sprintf(final_stats, "Devices discovered: %d\\r\\n", device_count);
    puts_USART1(final_stats);

    char lcd_final[20];
    sprintf(lcd_final, "T:%ld E:%d D:%d", i2c_transactions, i2c_errors, device_count);
    lcd_string(4, 0, lcd_final);

    uint8_t success_rate = 0;
    if (i2c_transactions > 0)
    {
        success_rate = ((i2c_transactions - i2c_errors) * 100) / i2c_transactions;
    }

    sprintf(final_stats, "Success rate: %d%%\\r\\n", success_rate);
    puts_USART1(final_stats);

    char lcd_success[20];
    sprintf(lcd_success, "Success: %d%%", success_rate);
    lcd_string(5, 0, lcd_success);

    if (success_rate >= 80)
    {
        lab_score += 100;
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
    puts_USART1("     I2C MASTER BASIC - LAB EXERCISES       \\r\\n");
    puts_USART1("==============================================\\r\\n");
    puts_USART1("1. I2C Initialization & Device Scanning     \\r\\n");
    puts_USART1("2. Basic Read/Write Operations               \\r\\n");
    puts_USART1("3. Multi-Device Communication               \\r\\n");
    puts_USART1("4. Advanced Protocols & Error Handling     \\r\\n");
    puts_USART1("                                              \\r\\n");
    puts_USART1("0. Run All Exercises                         \\r\\n");
    puts_USART1("X. Exit Lab                                   \\r\\n");
    puts_USART1("==============================================\\r\\n");
    char score_msg[50];
    sprintf(score_msg, "Current Score: %d points\\r\\n", lab_score);
    puts_USART1(score_msg);
    char stats_msg[60];
    sprintf(stats_msg, "I2C Stats: %ld trans, %d reads, %d writes\\r\\n",
            i2c_transactions, successful_reads, successful_writes);
    puts_USART1(stats_msg);
    char device_msg[50];
    sprintf(device_msg, "Devices: %d found, %d errors\\r\\n", device_count, i2c_errors);
    puts_USART1(device_msg);
    puts_USART1("Select exercise (1-4, 0, X): ");
}

int main(void)
{
    init_devices();

    puts_USART1("\\r\\n*** I2C MASTER BASIC LAB SESSION ***\\r\\n");
    puts_USART1("Welcome to I2C/TWI communication!\\r\\n");
    puts_USART1("This lab covers I2C master operations and device communication\\r\\n");
    puts_USART1("Ensure pull-up resistors are connected to SDA and SCL lines\\r\\n");

    lcd_clear();
    lcd_string(1, 0, "I2C MASTER LAB");
    lcd_string(2, 0, "TWI Communication");
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
            lab_ex1_i2c_initialization();
            lab_ex1_device_scanning();
            break;

        case '2':
            lab_ex2_basic_operations();
            lab_ex2_eeprom_testing();
            break;

        case '3':
            lab_ex3_multi_device();
            break;

        case '4':
            lab_ex4_advanced_protocols();
            break;

        case '0':
            puts_USART1("\\r\\n*** RUNNING ALL EXERCISES ***\\r\\n");
            lab_ex1_i2c_initialization();
            lab_ex1_device_scanning();
            lab_ex2_basic_operations();
            lab_ex2_eeprom_testing();
            lab_ex3_multi_device();
            lab_ex4_advanced_protocols();

            char final_buffer[80];
            sprintf(final_buffer, "\\r\\n*** ALL EXERCISES COMPLETE! ***\\r\\nFinal Score: %d points\\r\\n", lab_score);
            puts_USART1(final_buffer);
            break;

        case 'X':
        case 'x':
            puts_USART1("\\r\\nExiting lab. Great work on I2C communication!\\r\\n");
            puts_USART1("Remember: I2C is essential for sensor networks and device control!\\r\\n");
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