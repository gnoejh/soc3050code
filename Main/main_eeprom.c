#include "config.h"

/*
 * EEPROM_BASIC - Basic EEPROM Operations and Data Persistence
 * Educational demonstration of:
 * - EEPROM read/write operations
 * - Data persistence across power cycles
 * - EEPROM address management
 * - Data integrity verification
 * - Educational EEPROM testing patterns
 */

#ifdef EEPROM_BASIC

// EEPROM addresses for different data types
#define EEPROM_CONFIG_BASE 0x00   // Configuration area
#define EEPROM_COUNTER_ADDR 0x10  // Boot counter
#define EEPROM_SETTINGS_ADDR 0x20 // User settings
#define EEPROM_LOG_BASE 0x50      // Data logging area
#define EEPROM_TEST_PATTERN 0x100 // Test pattern area

// Configuration structure stored in EEPROM
typedef struct
{
    uint8_t magic_number;     // 0xAB to verify valid data
    uint8_t version;          // Configuration version
    uint16_t boot_count;      // Number of system boots
    uint8_t display_mode;     // Current display mode
    uint8_t sensor_threshold; // Sensor trigger threshold
    uint8_t checksum;         // Simple checksum for integrity
} eeprom_config_t;

// Current configuration in SRAM
static eeprom_config_t current_config;
static uint8_t config_loaded = 0;

// EEPROM utility functions
uint8_t eeprom_read_byte_safe(uint16_t address)
{
    // Wait for completion of previous write
    while (EECR & (1 << EEWE))
        ;

    // Set up address register
    EEAR = address;

    // Start EEPROM read
    EECR |= (1 << EERE);

    // Return data from data register
    return EEDR;
}

void eeprom_write_byte_safe(uint16_t address, uint8_t data)
{
    // Wait for completion of previous write
    while (EECR & (1 << EEWE))
        ;

    // Set up address and data registers
    EEAR = address;
    EEDR = data;

    // Write logical one to EEMWE
    EECR |= (1 << EEMWE);

    // Start EEPROM write by setting EEWE
    EECR |= (1 << EEWE);
}

uint8_t calculate_checksum(eeprom_config_t *config)
{
    uint8_t checksum = 0;
    uint8_t *ptr = (uint8_t *)config;

    // Calculate checksum of all bytes except the checksum field itself
    for (uint8_t i = 0; i < sizeof(eeprom_config_t) - 1; i++)
    {
        checksum ^= ptr[i];
    }

    return checksum;
}

void load_config_from_eeprom()
{
    // Read configuration structure from EEPROM
    uint8_t *ptr = (uint8_t *)&current_config;

    for (uint8_t i = 0; i < sizeof(eeprom_config_t); i++)
    {
        ptr[i] = eeprom_read_byte_safe(EEPROM_CONFIG_BASE + i);
    }

    // Verify magic number and checksum
    if (current_config.magic_number == 0xAB &&
        current_config.checksum == calculate_checksum(&current_config))
    {
        config_loaded = 1;
        current_config.boot_count++; // Increment boot counter
    }
    else
    {
        // Initialize with default values
        current_config.magic_number = 0xAB;
        current_config.version = 1;
        current_config.boot_count = 1;
        current_config.display_mode = 0;
        current_config.sensor_threshold = 128;
        current_config.checksum = calculate_checksum(&current_config);
        config_loaded = 1;
    }
}

void save_config_to_eeprom()
{
    // Update checksum before saving
    current_config.checksum = calculate_checksum(&current_config);

    // Write configuration structure to EEPROM
    uint8_t *ptr = (uint8_t *)&current_config;

    for (uint8_t i = 0; i < sizeof(eeprom_config_t); i++)
    {
        eeprom_write_byte_safe(EEPROM_CONFIG_BASE + i, ptr[i]);
    }
}

void display_config_info()
{
    glcd_clear();
    glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);

    char buffer[25];

    glcd_tiny_draw_string(0, 0, "EEPROM CONFIG:");

    sprintf(buffer, "Magic: 0x%02X", current_config.magic_number);
    glcd_tiny_draw_string(0, 10, buffer);

    sprintf(buffer, "Version: %d", current_config.version);
    glcd_tiny_draw_string(0, 20, buffer);

    sprintf(buffer, "Boot count: %d", current_config.boot_count);
    glcd_tiny_draw_string(0, 30, buffer);

    sprintf(buffer, "Display mode: %d", current_config.display_mode);
    glcd_tiny_draw_string(0, 40, buffer);

    sprintf(buffer, "Threshold: %d", current_config.sensor_threshold);
    glcd_tiny_draw_string(0, 50, buffer);

    sprintf(buffer, "Checksum: 0x%02X", current_config.checksum);
    glcd_tiny_draw_string(0, 60, buffer);

    glcd_tiny_draw_string(0, 80, "Status:");
    glcd_tiny_draw_string(0, 90, config_loaded ? "LOADED OK" : "DEFAULT");

    // Visual feedback on LEDs
    PORTA = current_config.boot_count & 0xFF;
}

void test_eeprom_patterns()
{
    glcd_clear();
    glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);
    glcd_tiny_draw_string(0, 0, "EEPROM PATTERN TEST:");

    uint8_t test_patterns[] = {0x00, 0xFF, 0xAA, 0x55, 0xF0, 0x0F};
    uint8_t pattern_count = sizeof(test_patterns);
    uint8_t errors = 0;

    // Test each pattern
    for (uint8_t p = 0; p < pattern_count; p++)
    {
        uint8_t pattern = test_patterns[p];

        // Write pattern to test area
        for (uint16_t addr = EEPROM_TEST_PATTERN; addr < EEPROM_TEST_PATTERN + 32; addr++)
        {
            eeprom_write_byte_safe(addr, pattern);
        }

        // Read back and verify
        for (uint16_t addr = EEPROM_TEST_PATTERN; addr < EEPROM_TEST_PATTERN + 32; addr++)
        {
            uint8_t read_value = eeprom_read_byte_safe(addr);
            if (read_value != pattern)
            {
                errors++;
            }
        }

        char buffer[25];
        sprintf(buffer, "Pattern 0x%02X: %s", pattern,
                (errors == 0) ? "PASS" : "FAIL");
        glcd_tiny_draw_string(0, 10 + p * 10, buffer);

        // Visual progress
        PORTB = pattern;
        _delay_ms(300);
    }

    char buffer[25];
    sprintf(buffer, "Total errors: %d", errors);
    glcd_tiny_draw_string(0, 80, buffer);

    glcd_tiny_draw_string(0, 100, "Press key to continue");
    while (!(PINF & 0x0F))
        ; // Wait for button
    while (PINF & 0x0F)
        ; // Wait for release
}

void interactive_settings()
{
    uint8_t setting_index = 0;
    uint8_t settings_count = 3; // display_mode, sensor_threshold, version

    while (1)
    {
        glcd_clear();
        glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);

        glcd_tiny_draw_string(0, 0, "INTERACTIVE SETTINGS:");

        char buffer[25];

        // Display current setting being edited
        switch (setting_index)
        {
        case 0:
            sprintf(buffer, "> Display Mode: %d", current_config.display_mode);
            glcd_tiny_draw_string(0, 20, buffer);
            sprintf(buffer, "  Threshold: %d", current_config.sensor_threshold);
            glcd_tiny_draw_string(0, 30, buffer);
            sprintf(buffer, "  Version: %d", current_config.version);
            glcd_tiny_draw_string(0, 40, buffer);
            break;
        case 1:
            sprintf(buffer, "  Display Mode: %d", current_config.display_mode);
            glcd_tiny_draw_string(0, 20, buffer);
            sprintf(buffer, "> Threshold: %d", current_config.sensor_threshold);
            glcd_tiny_draw_string(0, 30, buffer);
            sprintf(buffer, "  Version: %d", current_config.version);
            glcd_tiny_draw_string(0, 40, buffer);
            break;
        case 2:
            sprintf(buffer, "  Display Mode: %d", current_config.display_mode);
            glcd_tiny_draw_string(0, 20, buffer);
            sprintf(buffer, "  Threshold: %d", current_config.sensor_threshold);
            glcd_tiny_draw_string(0, 30, buffer);
            sprintf(buffer, "> Version: %d", current_config.version);
            glcd_tiny_draw_string(0, 40, buffer);
            break;
        }

        glcd_tiny_draw_string(0, 60, "SW0: Next  SW1: +");
        glcd_tiny_draw_string(0, 70, "SW2: -     SW3: Save");

        // Wait for button press
        while (!(PINF & 0x0F))
            ;
        uint8_t button = PINF & 0x0F;
        while (PINF & 0x0F)
            ; // Wait for release

        if (button & 0x01)
        { // SW0 - Next setting
            setting_index = (setting_index + 1) % settings_count;
        }
        else if (button & 0x02)
        { // SW1 - Increment
            switch (setting_index)
            {
            case 0:
                current_config.display_mode = (current_config.display_mode + 1) % 4;
                break;
            case 1:
                if (current_config.sensor_threshold < 255)
                    current_config.sensor_threshold++;
                break;
            case 2:
                if (current_config.version < 255)
                    current_config.version++;
                break;
            }
        }
        else if (button & 0x04)
        { // SW2 - Decrement
            switch (setting_index)
            {
            case 0:
                current_config.display_mode = (current_config.display_mode == 0) ? 3 : current_config.display_mode - 1;
                break;
            case 1:
                if (current_config.sensor_threshold > 0)
                    current_config.sensor_threshold--;
                break;
            case 2:
                if (current_config.version > 1)
                    current_config.version--;
                break;
            }
        }
        else if (button & 0x08)
        { // SW3 - Save and exit
            save_config_to_eeprom();

            glcd_clear();
            glcd_tiny_draw_string(30, 30, "SETTINGS");
            glcd_tiny_draw_string(40, 45, "SAVED!");
            _delay_ms(1000);
            break;
        }

        // Visual feedback
        PORTA = (setting_index << 4) | (current_config.display_mode);
        _delay_ms(200);
    }
}

void main_eeprom_basic()
{
    DDRA = 0xFF;  // Debug LEDs
    DDRB = 0xFF;  // Status LEDs
    DDRF = 0x00;  // Buttons input
    PORTF = 0xFF; // Enable pull-ups

    // Initialize systems
    init_GLCD();

    glcd_clear();
    glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);

    while (1)
    {
        // Phase 1: Introduction
        glcd_clear();
        glcd_tiny_draw_string(15, 20, "EEPROM BASIC");
        glcd_tiny_draw_string(20, 35, "OPERATIONS");
        glcd_tiny_draw_string(25, 50, "Press any key...");
        while (!(PINF & 0x0F))
            ; // Wait for button
        while (PINF & 0x0F)
            ; // Wait for release

        // Phase 2: Load configuration
        glcd_clear();
        glcd_tiny_draw_string(0, 0, "Loading config...");
        _delay_ms(1000);

        load_config_from_eeprom();
        display_config_info();
        _delay_ms(3000);

        // Phase 3: EEPROM pattern testing
        test_eeprom_patterns();

        // Phase 4: Interactive settings
        interactive_settings();

        // Phase 5: Display final configuration
        display_config_info();
        _delay_ms(2000);

        // Phase 6: Save updated configuration
        save_config_to_eeprom();

        glcd_clear();
        glcd_tiny_draw_string(20, 20, "EEPROM DEMO");
        glcd_tiny_draw_string(25, 35, "COMPLETE");
        glcd_tiny_draw_string(10, 50, "Press key to restart");
        while (!(PINF & 0x0F))
            ; // Wait for button
        while (PINF & 0x0F)
            ; // Wait for release

        // Visual feedback
        PORTA = ~PORTA; // Toggle LEDs
    }
}

#endif

#ifdef EEPROM_LOGGER
// Data Logging to EEPROM - Comprehensive Data Storage System
// Educational demonstration of:
// - Circular buffer implementation in EEPROM
// - Data logging with timestamps
// - Log management and retrieval
// - Data integrity and wear leveling concepts
// - Real-time sensor data logging

#define LOG_START_ADDR 0x200 // Start of log area
#define LOG_END_ADDR 0x3FF   // End of log area (512 bytes total)
#define LOG_ENTRY_SIZE 8     // Size of each log entry
#define MAX_LOG_ENTRIES ((LOG_END_ADDR - LOG_START_ADDR + 1) / LOG_ENTRY_SIZE)

// Log entry structure
typedef struct
{
    uint16_t timestamp;    // Simple timestamp counter
    uint8_t sensor_type;   // Type of sensor (ADC channel)
    uint8_t sensor_value;  // Sensor reading (0-255)
    uint8_t status_flags;  // Status information
    uint8_t battery_level; // Battery level simulation
    uint16_t checksum;     // Entry integrity check
} log_entry_t;

// Log management structure
typedef struct
{
    uint16_t write_index;  // Next write position
    uint16_t entry_count;  // Total entries written
    uint16_t oldest_entry; // Index of oldest valid entry
    uint8_t log_full;      // Flag indicating circular buffer is full
} log_header_t;

static log_header_t log_header;
static uint16_t current_timestamp = 0;
static uint8_t logging_active = 0;

// EEPROM logger utility functions
void eeprom_write_word(uint16_t address, uint16_t data)
{
    eeprom_write_byte_safe(address, data & 0xFF);
    eeprom_write_byte_safe(address + 1, (data >> 8) & 0xFF);
}

uint16_t eeprom_read_word(uint16_t address)
{
    uint16_t data = eeprom_read_byte_safe(address);
    data |= ((uint16_t)eeprom_read_byte_safe(address + 1)) << 8;
    return data;
}

uint16_t calculate_entry_checksum(log_entry_t *entry)
{
    uint16_t checksum = 0;
    uint8_t *ptr = (uint8_t *)entry;

    // Calculate checksum of all bytes except checksum field itself
    for (uint8_t i = 0; i < sizeof(log_entry_t) - 2; i++)
    {
        checksum += ptr[i];
    }

    return checksum;
}

void init_eeprom_logger()
{
    // Initialize log header from EEPROM or create new
    uint8_t *ptr = (uint8_t *)&log_header;
    for (uint8_t i = 0; i < sizeof(log_header_t); i++)
    {
        ptr[i] = eeprom_read_byte_safe(LOG_START_ADDR - sizeof(log_header_t) + i);
    }

    // Verify header integrity and initialize if needed
    if (log_header.write_index >= MAX_LOG_ENTRIES ||
        log_header.entry_count > MAX_LOG_ENTRIES)
    {

        // Initialize new log system
        log_header.write_index = 0;
        log_header.entry_count = 0;
        log_header.oldest_entry = 0;
        log_header.log_full = 0;

        // Save header to EEPROM
        ptr = (uint8_t *)&log_header;
        for (uint8_t i = 0; i < sizeof(log_header_t); i++)
        {
            eeprom_write_byte_safe(LOG_START_ADDR - sizeof(log_header_t) + i, ptr[i]);
        }
    }

    current_timestamp = log_header.entry_count; // Continue timestamp from last session
}

void save_log_header()
{
    uint8_t *ptr = (uint8_t *)&log_header;
    for (uint8_t i = 0; i < sizeof(log_header_t); i++)
    {
        eeprom_write_byte_safe(LOG_START_ADDR - sizeof(log_header_t) + i, ptr[i]);
    }
}

void write_log_entry(uint8_t sensor_type, uint8_t sensor_value, uint8_t status_flags)
{
    log_entry_t entry;

    // Populate log entry
    entry.timestamp = current_timestamp++;
    entry.sensor_type = sensor_type;
    entry.sensor_value = sensor_value;
    entry.status_flags = status_flags;
    entry.battery_level = 85 + (current_timestamp % 20); // Simulate battery drain
    entry.checksum = calculate_entry_checksum(&entry);

    // Calculate EEPROM address for this entry
    uint16_t eeprom_addr = LOG_START_ADDR + (log_header.write_index * LOG_ENTRY_SIZE);

    // Write entry to EEPROM
    uint8_t *ptr = (uint8_t *)&entry;
    for (uint8_t i = 0; i < sizeof(log_entry_t); i++)
    {
        eeprom_write_byte_safe(eeprom_addr + i, ptr[i]);
    }

    // Update log management
    log_header.write_index = (log_header.write_index + 1) % MAX_LOG_ENTRIES;

    if (log_header.entry_count < MAX_LOG_ENTRIES)
    {
        log_header.entry_count++;
    }
    else
    {
        log_header.log_full = 1;
        log_header.oldest_entry = log_header.write_index; // Circular buffer wrapping
    }

    // Save updated header
    save_log_header();
}

uint8_t read_log_entry(uint16_t entry_index, log_entry_t *entry)
{
    if (entry_index >= log_header.entry_count)
    {
        return 0; // Invalid index
    }

    // Calculate actual EEPROM address
    uint16_t actual_index;
    if (log_header.log_full)
    {
        actual_index = (log_header.oldest_entry + entry_index) % MAX_LOG_ENTRIES;
    }
    else
    {
        actual_index = entry_index;
    }

    uint16_t eeprom_addr = LOG_START_ADDR + (actual_index * LOG_ENTRY_SIZE);

    // Read entry from EEPROM
    uint8_t *ptr = (uint8_t *)entry;
    for (uint8_t i = 0; i < sizeof(log_entry_t); i++)
    {
        ptr[i] = eeprom_read_byte_safe(eeprom_addr + i);
    }

    // Verify checksum
    if (entry->checksum != calculate_entry_checksum(entry))
    {
        return 0; // Corrupted entry
    }

    return 1; // Success
}

void display_log_status()
{
    glcd_clear();
    glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);

    char buffer[25];

    glcd_tiny_draw_string(0, 0, "EEPROM LOGGER STATUS:");

    sprintf(buffer, "Total entries: %d", log_header.entry_count);
    glcd_tiny_draw_string(0, 15, buffer);

    sprintf(buffer, "Write index: %d", log_header.write_index);
    glcd_tiny_draw_string(0, 25, buffer);

    sprintf(buffer, "Max entries: %d", MAX_LOG_ENTRIES);
    glcd_tiny_draw_string(0, 35, buffer);

    sprintf(buffer, "Log full: %s", log_header.log_full ? "YES" : "NO");
    glcd_tiny_draw_string(0, 45, buffer);

    sprintf(buffer, "Oldest: %d", log_header.oldest_entry);
    glcd_tiny_draw_string(0, 55, buffer);

    sprintf(buffer, "Timestamp: %d", current_timestamp);
    glcd_tiny_draw_string(0, 65, buffer);

    sprintf(buffer, "Active: %s", logging_active ? "YES" : "NO");
    glcd_tiny_draw_string(0, 75, buffer);

    // Memory usage bar
    uint8_t usage_percent = (log_header.entry_count * 100) / MAX_LOG_ENTRIES;
    sprintf(buffer, "Usage: %d%%", usage_percent);
    glcd_tiny_draw_string(0, 90, buffer);

    // Visual usage bar
    for (uint8_t x = 0; x < 100; x++)
    {
        if (x < usage_percent)
        {
            for (uint8_t y = 0; y < 4; y++)
            {
                glcd_set_pixel(x + 10, 105 + y, 1);
            }
        }
        else
        {
            for (uint8_t y = 0; y < 4; y++)
            {
                glcd_set_pixel(x + 10, 105 + y, (x % 4 == 0) ? 1 : 0);
            }
        }
    }
}

void simulate_sensor_logging()
{
    glcd_clear();
    glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);
    glcd_tiny_draw_string(0, 0, "SIMULATING SENSORS:");

    logging_active = 1;

    for (uint8_t cycle = 0; cycle < 30; cycle++)
    {
        // Simulate multiple sensor readings
        uint8_t temp_sensor = 20 + (cycle % 15) + (TCNT0 % 5); // Temperature simulation
        uint8_t light_sensor = 128 + (cycle * 3) % 127;        // Light level simulation
        uint8_t motion_sensor = (cycle % 7 == 0) ? 255 : 0;    // Motion detection

        // Log each sensor with different types
        write_log_entry(0, temp_sensor, 0x01);   // Temperature sensor
        write_log_entry(1, light_sensor, 0x02);  // Light sensor
        write_log_entry(2, motion_sensor, 0x04); // Motion sensor

        // Display current readings
        char buffer[25];
        sprintf(buffer, "Cycle: %d", cycle);
        glcd_tiny_draw_string(0, 20, buffer);

        sprintf(buffer, "Temp: %d", temp_sensor);
        glcd_tiny_draw_string(0, 30, buffer);

        sprintf(buffer, "Light: %d", light_sensor);
        glcd_tiny_draw_string(0, 40, buffer);

        sprintf(buffer, "Motion: %s", motion_sensor ? "YES" : "NO");
        glcd_tiny_draw_string(0, 50, buffer);

        sprintf(buffer, "Entries: %d", log_header.entry_count);
        glcd_tiny_draw_string(0, 70, buffer);

        // Visual feedback
        PORTA = temp_sensor;
        PORTB = light_sensor;

        _delay_ms(500);

        // Clear some lines for next update
        for (uint8_t y = 20; y < 80; y++)
        {
            for (uint8_t x = 0; x < 128; x++)
            {
                glcd_set_pixel(x, y, 0);
            }
        }
    }

    logging_active = 0;

    glcd_clear();
    glcd_tiny_draw_string(30, 40, "LOGGING");
    glcd_tiny_draw_string(30, 55, "COMPLETE");
    _delay_ms(1500);
}

void browse_log_entries()
{
    if (log_header.entry_count == 0)
    {
        glcd_clear();
        glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);
        glcd_tiny_draw_string(30, 40, "NO LOG");
        glcd_tiny_draw_string(30, 55, "ENTRIES");
        _delay_ms(2000);
        return;
    }

    uint16_t current_entry = 0;

    while (1)
    {
        log_entry_t entry;

        if (read_log_entry(current_entry, &entry))
        {
            glcd_clear();
            glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);

            char buffer[25];

            sprintf(buffer, "Entry %d/%d", current_entry + 1, log_header.entry_count);
            glcd_tiny_draw_string(0, 0, buffer);

            sprintf(buffer, "Timestamp: %d", entry.timestamp);
            glcd_tiny_draw_string(0, 15, buffer);

            const char *sensor_names[] = {"Temp", "Light", "Motion", "Unknown"};
            uint8_t sensor_idx = (entry.sensor_type < 3) ? entry.sensor_type : 3;
            sprintf(buffer, "Sensor: %s (%d)", sensor_names[sensor_idx], entry.sensor_type);
            glcd_tiny_draw_string(0, 25, buffer);

            sprintf(buffer, "Value: %d", entry.sensor_value);
            glcd_tiny_draw_string(0, 35, buffer);

            sprintf(buffer, "Status: 0x%02X", entry.status_flags);
            glcd_tiny_draw_string(0, 45, buffer);

            sprintf(buffer, "Battery: %d%%", entry.battery_level);
            glcd_tiny_draw_string(0, 55, buffer);

            sprintf(buffer, "Checksum: 0x%04X", entry.checksum);
            glcd_tiny_draw_string(0, 65, buffer);

            // Visual sensor value bar
            glcd_tiny_draw_string(0, 80, "Value:");
            for (uint8_t x = 0; x < 100; x++)
            {
                uint8_t bar_height = (entry.sensor_value * x) / 2550; // Scale to display
                if (bar_height > 0)
                {
                    for (uint8_t y = 0; y < bar_height && y < 20; y++)
                    {
                        glcd_set_pixel(x + 10, 110 - y, 1);
                    }
                }
            }

            glcd_tiny_draw_string(0, 115, "SW0: Prev  SW1: Next");
        }
        else
        {
            glcd_clear();
            glcd_tiny_draw_string(20, 40, "CORRUPTED");
            glcd_tiny_draw_string(30, 55, "ENTRY!");
        }

        // Wait for button press
        while (!(PINF & 0x0F))
            ;
        uint8_t button = PINF & 0x0F;
        while (PINF & 0x0F)
            ; // Wait for release

        if (button & 0x01)
        { // SW0 - Previous
            if (current_entry > 0)
            {
                current_entry--;
            }
        }
        else if (button & 0x02)
        { // SW1 - Next
            if (current_entry < log_header.entry_count - 1)
            {
                current_entry++;
            }
        }
        else if (button & 0x08)
        { // SW3 - Exit
            break;
        }

        PORTA = current_entry; // Visual feedback
    }
}

void clear_log_data()
{
    glcd_clear();
    glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);
    glcd_tiny_draw_string(20, 30, "CLEAR LOG?");
    glcd_tiny_draw_string(10, 50, "SW1: Yes  SW3: No");

    while (!(PINF & 0x0F))
        ;
    uint8_t button = PINF & 0x0F;
    while (PINF & 0x0F)
        ; // Wait for release

    if (button & 0x02)
    { // SW1 - Yes, clear
        // Reset log header
        log_header.write_index = 0;
        log_header.entry_count = 0;
        log_header.oldest_entry = 0;
        log_header.log_full = 0;
        save_log_header();

        current_timestamp = 0;

        glcd_clear();
        glcd_tiny_draw_string(30, 40, "LOG");
        glcd_tiny_draw_string(25, 55, "CLEARED");
        _delay_ms(1500);
    }
}

void main_eeprom_logger()
{
    DDRA = 0xFF;  // Debug LEDs
    DDRB = 0xFF;  // Status LEDs
    DDRF = 0x00;  // Buttons input
    PORTF = 0xFF; // Enable pull-ups

    // Initialize systems
    init_GLCD();
    init_eeprom_logger();

    glcd_clear();
    glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);

    while (1)
    {
        // Main menu
        glcd_clear();
        glcd_tiny_draw_string(15, 10, "EEPROM LOGGER");
        glcd_tiny_draw_string(0, 30, "SW0: Status");
        glcd_tiny_draw_string(0, 40, "SW1: Simulate logging");
        glcd_tiny_draw_string(0, 50, "SW2: Browse entries");
        glcd_tiny_draw_string(0, 60, "SW3: Clear log");

        char buffer[25];
        sprintf(buffer, "Entries: %d", log_header.entry_count);
        glcd_tiny_draw_string(0, 80, buffer);

        // Wait for button press
        while (!(PINF & 0x0F))
            ;
        uint8_t button = PINF & 0x0F;
        while (PINF & 0x0F)
            ; // Wait for release

        if (button & 0x01)
        { // SW0 - Status
            display_log_status();
            _delay_ms(3000);
        }
        else if (button & 0x02)
        { // SW1 - Simulate logging
            simulate_sensor_logging();
        }
        else if (button & 0x04)
        { // SW2 - Browse entries
            browse_log_entries();
        }
        else if (button & 0x08)
        { // SW3 - Clear log
            clear_log_data();
        }

        // Visual feedback
        PORTA = log_header.entry_count & 0xFF;
        PORTB = (log_header.write_index << 4) | (log_header.log_full ? 0x0F : 0x00);
    }
}

#endif

#ifdef EEPROM_SETTINGS
// Advanced Settings Management System in EEPROM
// Educational demonstration of:
// - Hierarchical settings organization
// - Settings validation and defaults
// - Settings backup and restore
// - User preference management
// - Settings export/import functionality

#define SETTINGS_BASE_ADDR 0x400   // Start of settings area
#define SETTINGS_BACKUP_ADDR 0x500 // Backup settings area
#define SETTINGS_SIZE 64           // Total settings structure size

// Settings categories and structures
typedef struct
{
    uint8_t brightness;  // Display brightness (0-100)
    uint8_t contrast;    // Display contrast (0-100)
    uint8_t timeout;     // Screen timeout in seconds
    uint8_t orientation; // Screen orientation (0-3)
} display_settings_t;

typedef struct
{
    uint8_t volume;         // Audio volume (0-100)
    uint8_t tone_frequency; // Beep frequency (0-255)
    uint8_t enable_sounds;  // Enable/disable sounds
    uint8_t alarm_enabled;  // Enable alarm functionality
} audio_settings_t;

typedef struct
{
    uint8_t temp_threshold_low;  // Low temperature alert
    uint8_t temp_threshold_high; // High temperature alert
    uint8_t light_sensitivity;   // Light sensor sensitivity
    uint8_t motion_timeout;      // Motion sensor timeout
} sensor_settings_t;

typedef struct
{
    uint8_t auto_log;     // Enable automatic logging
    uint8_t log_interval; // Logging interval in minutes
    uint8_t max_entries;  // Maximum log entries to keep
    uint8_t log_level;    // Logging verbosity level
} system_settings_t;

// Main settings structure
typedef struct
{
    uint16_t magic_number; // 0x5E77 for valid settings
    uint8_t version;       // Settings version
    display_settings_t display;
    audio_settings_t audio;
    sensor_settings_t sensors;
    system_settings_t system;
    uint8_t user_profile;        // Current user profile (0-3)
    uint8_t factory_reset_count; // Number of factory resets
    uint32_t last_modified;      // Timestamp of last modification
    uint16_t checksum;           // Settings integrity checksum
} device_settings_t;

static device_settings_t current_settings;
static device_settings_t backup_settings;
static uint8_t settings_loaded = 0;
static uint8_t settings_modified = 0;

// Utility function for value constraining
uint8_t constrain_uint8(int16_t value, uint8_t min_val, uint8_t max_val)
{
    if (value < min_val)
        return min_val;
    if (value > max_val)
        return max_val;
    return (uint8_t)value;
}

// Default settings profiles
const device_settings_t default_profiles[4] PROGMEM = {
    // Profile 0: Standard
    {
        .magic_number = 0x5E77,
        .version = 1,
        .display = {80, 75, 30, 0},
        .audio = {50, 128, 1, 1},
        .sensors = {15, 35, 128, 60},
        .system = {1, 5, 100, 2},
        .user_profile = 0,
        .factory_reset_count = 0,
        .last_modified = 0,
        .checksum = 0},
    // Profile 1: Power Saver
    {
        .magic_number = 0x5E77,
        .version = 1,
        .display = {40, 60, 10, 0},
        .audio = {25, 100, 0, 0},
        .sensors = {20, 30, 100, 120},
        .system = {0, 15, 50, 1},
        .user_profile = 1,
        .factory_reset_count = 0,
        .last_modified = 0,
        .checksum = 0},
    // Profile 2: Performance
    {
        .magic_number = 0x5E77,
        .version = 1,
        .display = {100, 90, 60, 0},
        .audio = {75, 150, 1, 1},
        .sensors = {10, 40, 200, 30},
        .system = {1, 1, 200, 3},
        .user_profile = 2,
        .factory_reset_count = 0,
        .last_modified = 0,
        .checksum = 0},
    // Profile 3: Silent
    {
        .magic_number = 0x5E77,
        .version = 1,
        .display = {60, 70, 45, 0},
        .audio = {0, 80, 0, 0},
        .sensors = {25, 35, 150, 90},
        .system = {1, 10, 75, 1},
        .user_profile = 3,
        .factory_reset_count = 0,
        .last_modified = 0,
        .checksum = 0}};

uint16_t calculate_settings_checksum(device_settings_t *settings)
{
    uint16_t checksum = 0;
    uint8_t *ptr = (uint8_t *)settings;

    // Calculate checksum of all bytes except checksum field itself
    for (uint16_t i = 0; i < sizeof(device_settings_t) - 2; i++)
    {
        checksum += ptr[i];
        checksum = (checksum << 1) | (checksum >> 15); // Rotate for better distribution
    }

    return checksum;
}

void load_default_profile(uint8_t profile_index)
{
    if (profile_index < 4)
    {
        // Copy from program memory
        memcpy_P(&current_settings, &default_profiles[profile_index], sizeof(device_settings_t));
        current_settings.checksum = calculate_settings_checksum(&current_settings);
        settings_modified = 1;
    }
}

void load_settings_from_eeprom()
{
    // Load primary settings
    uint8_t *ptr = (uint8_t *)&current_settings;
    for (uint16_t i = 0; i < sizeof(device_settings_t); i++)
    {
        ptr[i] = eeprom_read_byte_safe(SETTINGS_BASE_ADDR + i);
    }

    // Verify integrity
    if (current_settings.magic_number == 0x5E77 &&
        current_settings.checksum == calculate_settings_checksum(&current_settings))
    {
        settings_loaded = 1;
    }
    else
    {
        // Try loading backup
        ptr = (uint8_t *)&backup_settings;
        for (uint16_t i = 0; i < sizeof(device_settings_t); i++)
        {
            ptr[i] = eeprom_read_byte_safe(SETTINGS_BACKUP_ADDR + i);
        }

        if (backup_settings.magic_number == 0x5E77 &&
            backup_settings.checksum == calculate_settings_checksum(&backup_settings))
        {
            // Restore from backup
            memcpy(&current_settings, &backup_settings, sizeof(device_settings_t));
            settings_loaded = 1;
            settings_modified = 1; // Will trigger save of restored settings
        }
        else
        {
            // Load default profile
            load_default_profile(0);
            settings_loaded = 1;
        }
    }
}

void save_settings_to_eeprom()
{
    // Update timestamp and checksum
    current_settings.last_modified++;
    current_settings.checksum = calculate_settings_checksum(&current_settings);

    // Save primary settings
    uint8_t *ptr = (uint8_t *)&current_settings;
    for (uint16_t i = 0; i < sizeof(device_settings_t); i++)
    {
        eeprom_write_byte_safe(SETTINGS_BASE_ADDR + i, ptr[i]);
    }

    // Create backup copy
    for (uint16_t i = 0; i < sizeof(device_settings_t); i++)
    {
        eeprom_write_byte_safe(SETTINGS_BACKUP_ADDR + i, ptr[i]);
    }

    settings_modified = 0;
}

void display_settings_menu(uint8_t category, uint8_t item)
{
    glcd_clear();
    glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);

    const char *categories[] = {"Display", "Audio", "Sensors", "System"};
    glcd_tiny_draw_string(0, 0, "SETTINGS:");
    glcd_tiny_draw_string(60, 0, categories[category]);

    char buffer[25];

    switch (category)
    {
    case 0: // Display settings
    {
        const char *items[] = {"Brightness", "Contrast", "Timeout", "Orientation"};
        uint8_t values[] = {current_settings.display.brightness,
                            current_settings.display.contrast,
                            current_settings.display.timeout,
                            current_settings.display.orientation};

        for (uint8_t i = 0; i < 4; i++)
        {
            if (i == item)
            {
                sprintf(buffer, "> %s: %d", items[i], values[i]);
            }
            else
            {
                sprintf(buffer, "  %s: %d", items[i], values[i]);
            }
            glcd_tiny_draw_string(0, 15 + i * 10, buffer);
        }
    }
    break;

    case 1: // Audio settings
    {
        const char *items[] = {"Volume", "Tone Freq", "Sounds", "Alarm"};
        uint8_t values[] = {current_settings.audio.volume,
                            current_settings.audio.tone_frequency,
                            current_settings.audio.enable_sounds,
                            current_settings.audio.alarm_enabled};

        for (uint8_t i = 0; i < 4; i++)
        {
            if (i == item)
            {
                if (i >= 2)
                { // Boolean settings
                    sprintf(buffer, "> %s: %s", items[i], values[i] ? "ON" : "OFF");
                }
                else
                {
                    sprintf(buffer, "> %s: %d", items[i], values[i]);
                }
            }
            else
            {
                if (i >= 2)
                { // Boolean settings
                    sprintf(buffer, "  %s: %s", items[i], values[i] ? "ON" : "OFF");
                }
                else
                {
                    sprintf(buffer, "  %s: %d", items[i], values[i]);
                }
            }
            glcd_tiny_draw_string(0, 15 + i * 10, buffer);
        }
    }
    break;

    case 2: // Sensor settings
    {
        const char *items[] = {"Temp Low", "Temp High", "Light Sens", "Motion TO"};
        uint8_t values[] = {current_settings.sensors.temp_threshold_low,
                            current_settings.sensors.temp_threshold_high,
                            current_settings.sensors.light_sensitivity,
                            current_settings.sensors.motion_timeout};

        for (uint8_t i = 0; i < 4; i++)
        {
            if (i == item)
            {
                sprintf(buffer, "> %s: %d", items[i], values[i]);
            }
            else
            {
                sprintf(buffer, "  %s: %d", items[i], values[i]);
            }
            glcd_tiny_draw_string(0, 15 + i * 10, buffer);
        }
    }
    break;

    case 3: // System settings
    {
        const char *items[] = {"Auto Log", "Log Interval", "Max Entries", "Log Level"};
        uint8_t values[] = {current_settings.system.auto_log,
                            current_settings.system.log_interval,
                            current_settings.system.max_entries,
                            current_settings.system.log_level};

        for (uint8_t i = 0; i < 4; i++)
        {
            if (i == item)
            {
                if (i == 0)
                { // Boolean setting
                    sprintf(buffer, "> %s: %s", items[i], values[i] ? "ON" : "OFF");
                }
                else
                {
                    sprintf(buffer, "> %s: %d", items[i], values[i]);
                }
            }
            else
            {
                if (i == 0)
                { // Boolean setting
                    sprintf(buffer, "  %s: %s", items[i], values[i] ? "ON" : "OFF");
                }
                else
                {
                    sprintf(buffer, "  %s: %d", items[i], values[i]);
                }
            }
            glcd_tiny_draw_string(0, 15 + i * 10, buffer);
        }
    }
    break;
    }

    // Control instructions
    glcd_tiny_draw_string(0, 70, "SW0: Category  SW1: +");
    glcd_tiny_draw_string(0, 80, "SW2: -         SW3: Item");

    // Status information
    sprintf(buffer, "Profile: %d  Modified: %s",
            current_settings.user_profile,
            settings_modified ? "YES" : "NO");
    glcd_tiny_draw_string(0, 100, buffer);
}

void modify_setting_value(uint8_t category, uint8_t item, int8_t delta)
{
    switch (category)
    {
    case 0: // Display
        switch (item)
        {
        case 0: // Brightness
            current_settings.display.brightness = constrain_uint8(
                current_settings.display.brightness + delta, 0, 100);
            break;
        case 1: // Contrast
            current_settings.display.contrast = constrain_uint8(
                current_settings.display.contrast + delta, 0, 100);
            break;
        case 2: // Timeout
            current_settings.display.timeout = constrain_uint8(
                current_settings.display.timeout + delta, 5, 120);
            break;
        case 3: // Orientation
            current_settings.display.orientation = constrain_uint8(
                current_settings.display.orientation + delta, 0, 3);
            break;
        }
        break;

    case 1: // Audio
        switch (item)
        {
        case 0: // Volume
            current_settings.audio.volume = constrain_uint8(
                current_settings.audio.volume + delta, 0, 100);
            break;
        case 1: // Tone frequency
            current_settings.audio.tone_frequency = constrain_uint8(
                current_settings.audio.tone_frequency + delta, 50, 255);
            break;
        case 2: // Enable sounds
            current_settings.audio.enable_sounds =
                current_settings.audio.enable_sounds ? 0 : 1;
            break;
        case 3: // Alarm enabled
            current_settings.audio.alarm_enabled =
                current_settings.audio.alarm_enabled ? 0 : 1;
            break;
        }
        break;

    case 2: // Sensors
        switch (item)
        {
        case 0: // Temp threshold low
            current_settings.sensors.temp_threshold_low = constrain_uint8(
                current_settings.sensors.temp_threshold_low + delta, 0, 50);
            break;
        case 1: // Temp threshold high
            current_settings.sensors.temp_threshold_high = constrain_uint8(
                current_settings.sensors.temp_threshold_high + delta, 25, 60);
            break;
        case 2: // Light sensitivity
            current_settings.sensors.light_sensitivity = constrain_uint8(
                current_settings.sensors.light_sensitivity + delta, 50, 255);
            break;
        case 3: // Motion timeout
            current_settings.sensors.motion_timeout = constrain_uint8(
                current_settings.sensors.motion_timeout + delta, 10, 300);
            break;
        }
        break;

    case 3: // System
        switch (item)
        {
        case 0: // Auto log
            current_settings.system.auto_log =
                current_settings.system.auto_log ? 0 : 1;
            break;
        case 1: // Log interval
            current_settings.system.log_interval = constrain_uint8(
                current_settings.system.log_interval + delta, 1, 60);
            break;
        case 2: // Max entries
            current_settings.system.max_entries = constrain_uint8(
                current_settings.system.max_entries + delta, 10, 255);
            break;
        case 3: // Log level
            current_settings.system.log_level = constrain_uint8(
                current_settings.system.log_level + delta, 0, 3);
            break;
        }
        break;
    }

    settings_modified = 1;
}

void settings_management_menu()
{
    uint8_t menu_item = 0;

    while (1)
    {
        glcd_clear();
        glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);

        glcd_tiny_draw_string(10, 0, "SETTINGS MANAGEMENT");

        const char *menu_items[] = {
            "Edit Settings",
            "Load Profile",
            "Save Settings",
            "Factory Reset",
            "Settings Info",
            "Exit"};

        for (uint8_t i = 0; i < 6; i++)
        {
            if (i == menu_item)
            {
                glcd_tiny_draw_string(0, 20 + i * 12, ">");
            }
            else
            {
                glcd_tiny_draw_string(0, 20 + i * 12, " ");
            }
            glcd_tiny_draw_string(10, 20 + i * 12, menu_items[i]);
        }

        // Status line
        char buffer[25];
        sprintf(buffer, "Modified: %s", settings_modified ? "YES" : "NO");
        glcd_tiny_draw_string(0, 105, buffer);

        // Wait for button press
        while (!(PINF & 0x0F))
            ;
        uint8_t button = PINF & 0x0F;
        while (PINF & 0x0F)
            ; // Wait for release

        if (button & 0x01)
        { // SW0 - Previous item
            menu_item = (menu_item == 0) ? 5 : menu_item - 1;
        }
        else if (button & 0x02)
        { // SW1 - Next item
            menu_item = (menu_item + 1) % 6;
        }
        else if (button & 0x04 || button & 0x08)
        { // SW2 or SW3 - Select
            switch (menu_item)
            {
            case 0: // Edit Settings
            {
                uint8_t category = 0, item = 0;
                while (1)
                {
                    display_settings_menu(category, item);

                    while (!(PINF & 0x0F))
                        ;
                    uint8_t btn = PINF & 0x0F;
                    while (PINF & 0x0F)
                        ;

                    if (btn & 0x01)
                    { // Change category
                        category = (category + 1) % 4;
                        item = 0;
                    }
                    else if (btn & 0x02)
                    { // Increase value
                        modify_setting_value(category, item, 1);
                    }
                    else if (btn & 0x04)
                    { // Decrease value
                        modify_setting_value(category, item, -1);
                    }
                    else if (btn & 0x08)
                    { // Change item
                        item = (item + 1) % 4;
                    }

                    // Quick exit with multiple button press
                    if ((btn & 0x09) == 0x09)
                        break; // SW0 + SW3
                }
            }
            break;

            case 1: // Load Profile
            {
                glcd_clear();
                glcd_tiny_draw_string(0, 20, "Select Profile:");
                glcd_tiny_draw_string(0, 40, "SW0: Standard");
                glcd_tiny_draw_string(0, 50, "SW1: Power Saver");
                glcd_tiny_draw_string(0, 60, "SW2: Performance");
                glcd_tiny_draw_string(0, 70, "SW3: Silent");

                while (!(PINF & 0x0F))
                    ;
                uint8_t profile_btn = PINF & 0x0F;
                while (PINF & 0x0F)
                    ;

                if (profile_btn & 0x01)
                    load_default_profile(0);
                else if (profile_btn & 0x02)
                    load_default_profile(1);
                else if (profile_btn & 0x04)
                    load_default_profile(2);
                else if (profile_btn & 0x08)
                    load_default_profile(3);

                glcd_clear();
                glcd_tiny_draw_string(30, 40, "PROFILE");
                glcd_tiny_draw_string(35, 55, "LOADED");
                _delay_ms(1000);
            }
            break;

            case 2: // Save Settings
                save_settings_to_eeprom();
                glcd_clear();
                glcd_tiny_draw_string(30, 40, "SETTINGS");
                glcd_tiny_draw_string(35, 55, "SAVED");
                _delay_ms(1000);
                break;

            case 3: // Factory Reset
                glcd_clear();
                glcd_tiny_draw_string(10, 30, "FACTORY RESET?");
                glcd_tiny_draw_string(10, 50, "SW1: Yes  SW3: No");

                while (!(PINF & 0x0F))
                    ;
                uint8_t reset_btn = PINF & 0x0F;
                while (PINF & 0x0F)
                    ;

                if (reset_btn & 0x02)
                { // Yes
                    current_settings.factory_reset_count++;
                    load_default_profile(0);
                    save_settings_to_eeprom();

                    glcd_clear();
                    glcd_tiny_draw_string(25, 40, "FACTORY");
                    glcd_tiny_draw_string(30, 55, "RESET");
                    _delay_ms(1500);
                }
                break;

            case 4: // Settings Info
            {
                glcd_clear();
                glcd_tiny_draw_string(0, 0, "SETTINGS INFO:");

                char buffer[25];
                sprintf(buffer, "Version: %d", current_settings.version);
                glcd_tiny_draw_string(0, 15, buffer);

                sprintf(buffer, "Profile: %d", current_settings.user_profile);
                glcd_tiny_draw_string(0, 25, buffer);

                sprintf(buffer, "Resets: %d", current_settings.factory_reset_count);
                glcd_tiny_draw_string(0, 35, buffer);

                sprintf(buffer, "Modified: %ld", current_settings.last_modified);
                glcd_tiny_draw_string(0, 45, buffer);

                sprintf(buffer, "Checksum: 0x%04X", current_settings.checksum);
                glcd_tiny_draw_string(0, 55, buffer);

                sprintf(buffer, "Size: %d bytes", sizeof(device_settings_t));
                glcd_tiny_draw_string(0, 65, buffer);

                glcd_tiny_draw_string(0, 85, "Press any key...");
                while (!(PINF & 0x0F))
                    ;
                while (PINF & 0x0F)
                    ;
            }
            break;

            case 5: // Exit
                return;
            }
        }

        PORTA = menu_item; // Visual feedback
    }
}

void main_eeprom_settings()
{
    DDRA = 0xFF;  // Debug LEDs
    DDRB = 0xFF;  // Status LEDs
    DDRF = 0x00;  // Buttons input
    PORTF = 0xFF; // Enable pull-ups

    // Initialize systems
    init_GLCD();
    load_settings_from_eeprom();

    glcd_clear();
    glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);

    while (1)
    {
        // Welcome screen
        glcd_clear();
        glcd_tiny_draw_string(10, 15, "EEPROM SETTINGS");
        glcd_tiny_draw_string(15, 30, "MANAGEMENT");

        char buffer[25];
        sprintf(buffer, "Profile: %d (%s)",
                current_settings.user_profile,
                settings_loaded ? "OK" : "DEFAULT");
        glcd_tiny_draw_string(0, 50, buffer);

        sprintf(buffer, "Modified: %s", settings_modified ? "YES" : "NO");
        glcd_tiny_draw_string(0, 65, buffer);

        glcd_tiny_draw_string(20, 85, "Press any key...");

        while (!(PINF & 0x0F))
            ; // Wait for button
        while (PINF & 0x0F)
            ; // Wait for release

        // Enter settings management
        settings_management_menu();

        // Visual feedback with current profile
        PORTA = current_settings.user_profile;
        PORTB = (current_settings.display.brightness >> 2);
    }
}

#endif