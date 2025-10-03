/*
 * ===========================================================================
 * EEPROM CONFIG - Configuration and Settings Management
 * ATmega128 @ 16MHz, EEPROM for persistent settings
 * ===========================================================================
 *
 * FOCUS: Using EEPROM to save and load system configuration
 *
 * CONFIGURATION USE CASES:
 * - User preferences (brightness, speed, modes)
 * - Calibration data (sensor offsets, scaling factors)
 * - System state (last mode, operation hours)
 * - Network settings (IDs, addresses, keys)
 *
 * DESIGN CONSIDERATIONS:
 * - Structure versioning: Handle format changes
 * - Default values: Initialize on first use
 * - Validation: Detect corruption
 * - Grouping: Organize related settings
 *
 * LEARNING OBJECTIVES:
 * 1. Design configuration structures
 * 2. Implement load/save functions
 * 3. Handle default initialization
 * 4. Validate stored data
 * 5. Version configuration formats
 *
 * HARDWARE:
 * - ATmega128 @ 16MHz
 * - LEDs on PORTB
 * - Buttons for configuration changes
 * - UART for menu interface
 */

#include "config.h"

// ============================================================================
// EEPROM FUNCTIONS (from EEPROM_Basic)
// ============================================================================

uint8_t eeprom_read_byte(uint16_t address)
{
    while (EECR & (1 << EEWE))
        ;
    EEAR = address;
    EECR |= (1 << EERE);
    return EEDR;
}

void eeprom_write_byte(uint16_t address, uint8_t data)
{
    while (EECR & (1 << EEWE))
        ;
    EEAR = address;
    EEDR = data;
    cli();
    EECR |= (1 << EEMWE);
    EECR |= (1 << EEWE);
    sei();
}

void eeprom_update_byte(uint16_t address, uint8_t data)
{
    if (eeprom_read_byte(address) != data)
        eeprom_write_byte(address, data);
}

// ============================================================================
// CONFIGURATION STRUCTURE
// ============================================================================

#define CONFIG_BASE_ADDR 0x0000
#define CONFIG_MAGIC 0x5A3C // Signature to detect valid config
#define CONFIG_VERSION 1

typedef struct
{
    uint16_t magic;  // Magic number for validation
    uint8_t version; // Config format version

    // User settings
    uint8_t led_brightness; // 0-255
    uint8_t blink_speed;    // delay in 10ms units
    uint8_t auto_mode;      // 0=manual, 1=auto

    // System settings
    uint16_t run_hours; // Total operation hours
    uint8_t last_mode;  // Last operating mode

    // Calibration
    int8_t sensor_offset; // Sensor calibration offset
    uint8_t reserved[5];  // For future use

    uint8_t checksum; // Simple checksum
} config_t;

config_t g_config; // Global configuration

// ============================================================================
// CONFIGURATION FUNCTIONS
// ============================================================================

// Calculate checksum
uint8_t calc_checksum(config_t *cfg)
{
    uint8_t *ptr = (uint8_t *)cfg;
    uint8_t sum = 0;

    // Sum all bytes except checksum itself
    for (uint16_t i = 0; i < sizeof(config_t) - 1; i++)
        sum += ptr[i];

    return ~sum; // One's complement
}

// Load config from EEPROM
uint8_t config_load(config_t *cfg)
{
    uint8_t *ptr = (uint8_t *)cfg;
    uint16_t addr = CONFIG_BASE_ADDR;

    // Read all bytes
    for (uint16_t i = 0; i < sizeof(config_t); i++)
        ptr[i] = eeprom_read_byte(addr++);

    // Validate magic number
    if (cfg->magic != CONFIG_MAGIC)
        return 0; // Invalid

    // Validate checksum
    uint8_t calc_sum = calc_checksum(cfg);
    if (calc_sum != cfg->checksum)
        return 0; // Corrupted

    // Check version compatibility
    if (cfg->version != CONFIG_VERSION)
        return 0; // Incompatible version

    return 1; // Valid
}

// Save config to EEPROM
void config_save(config_t *cfg)
{
    cfg->magic = CONFIG_MAGIC;
    cfg->version = CONFIG_VERSION;
    cfg->checksum = calc_checksum(cfg);

    uint8_t *ptr = (uint8_t *)cfg;
    uint16_t addr = CONFIG_BASE_ADDR;

    // Write all bytes
    for (uint16_t i = 0; i < sizeof(config_t); i++)
        eeprom_update_byte(addr++, ptr[i]); // Only write if changed
}

// Set defaults
void config_defaults(config_t *cfg)
{
    cfg->led_brightness = 128; // 50%
    cfg->blink_speed = 50;     // 500ms
    cfg->auto_mode = 1;        // Auto ON
    cfg->run_hours = 0;
    cfg->last_mode = 0;
    cfg->sensor_offset = 0;

    for (uint8_t i = 0; i < 5; i++)
        cfg->reserved[i] = 0;
}

// Initialize config (load or set defaults)
void config_init(config_t *cfg)
{
    if (!config_load(cfg))
    {
        // Invalid or missing config - use defaults
        config_defaults(cfg);
        config_save(cfg);
    }
}

// ============================================================================
// UART FUNCTIONS
// ============================================================================

void uart_init(void)
{
    UBRR0H = 0;
    UBRR0L = 103;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_transmit(char c)
{
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    UDR0 = c;
}

void uart_print(const char *str)
{
    while (*str)
        uart_transmit(*str++);
}

char uart_receive(void)
{
    while (!(UCSR0A & (1 << RXC0)))
        ;
    return UDR0;
}

void uart_print_dec(uint16_t value)
{
    char buffer[6];
    uint8_t i = 0;

    if (value == 0)
    {
        uart_transmit('0');
        return;
    }

    while (value > 0)
    {
        buffer[i++] = '0' + (value % 10);
        value /= 10;
    }

    while (i > 0)
        uart_transmit(buffer[--i]);
}

// ============================================================================
// Demo 1: Load/Save Configuration
// ============================================================================

void demo_01_load_save(void)
{
    DDRB = 0xFF;
    PORTB = 0x00;

    uart_init();

    uart_print("=== Configuration Demo ===\r\n\r\n");

    // Initialize (load or defaults)
    config_init(&g_config);

    uart_print("Current config:\r\n");
    uart_print("  Brightness: ");
    uart_print_dec(g_config.led_brightness);
    uart_print("\r\n  Blink speed: ");
    uart_print_dec(g_config.blink_speed);
    uart_print(" x10ms\r\n  Auto mode: ");
    uart_print(g_config.auto_mode ? "ON\r\n" : "OFF\r\n");
    uart_print("  Run hours: ");
    uart_print_dec(g_config.run_hours);
    uart_print("\r\n\r\n");

    // Demonstrate operation with config
    PORTB = g_config.led_brightness;
    uart_print("LEDs set to configured brightness\r\n");
    uart_print("Power cycle to see persistence\r\n");

    while (1)
        ;
}

// ============================================================================
// Demo 2: Interactive Configuration Menu
// ============================================================================

void print_menu(void)
{
    uart_print("\r\n=== CONFIG MENU ===\r\n");
    uart_print("1. Set brightness\r\n");
    uart_print("2. Set blink speed\r\n");
    uart_print("3. Toggle auto mode\r\n");
    uart_print("4. View config\r\n");
    uart_print("5. Reset to defaults\r\n");
    uart_print("6. Save and exit\r\n");
    uart_print("Choice: ");
}

void demo_02_config_menu(void)
{
    DDRB = 0xFF;
    PORTB = 0x00;

    uart_init();

    uart_print("=== Interactive Config ===\r\n");

    config_init(&g_config);

    while (1)
    {
        print_menu();
        char choice = uart_receive();
        uart_transmit(choice);
        uart_print("\r\n\r\n");

        switch (choice)
        {
        case '1':
            uart_print("Enter brightness (0-255): ");
            // Simple input (just increment for demo)
            g_config.led_brightness += 32;
            uart_print_dec(g_config.led_brightness);
            uart_print("\r\n");
            PORTB = g_config.led_brightness;
            break;

        case '2':
            uart_print("Enter speed (1-100): ");
            g_config.blink_speed += 10;
            if (g_config.blink_speed > 100)
                g_config.blink_speed = 10;
            uart_print_dec(g_config.blink_speed);
            uart_print(" x10ms\r\n");
            break;

        case '3':
            g_config.auto_mode = !g_config.auto_mode;
            uart_print("Auto mode: ");
            uart_print(g_config.auto_mode ? "ON\r\n" : "OFF\r\n");
            break;

        case '4':
            uart_print("Brightness: ");
            uart_print_dec(g_config.led_brightness);
            uart_print("\r\nBlink: ");
            uart_print_dec(g_config.blink_speed);
            uart_print(" x10ms\r\nAuto: ");
            uart_print(g_config.auto_mode ? "ON\r\n" : "OFF\r\n");
            break;

        case '5':
            config_defaults(&g_config);
            uart_print("Reset to defaults\r\n");
            break;

        case '6':
            config_save(&g_config);
            uart_print("Configuration saved!\r\n");
            uart_print("Reset to apply\r\n");
            while (1)
                ;
            break;
        }
    }
}

// ============================================================================
// Demo 3: Run Hours Counter
// ============================================================================

void demo_03_run_hours(void)
{
    DDRB = 0xFF;
    PORTB = 0x00;

    uart_init();

    uart_print("=== Run Hours Tracker ===\r\n\r\n");

    config_init(&g_config);

    uart_print("Total run hours: ");
    uart_print_dec(g_config.run_hours);
    uart_print("\r\n");

    uart_print("Running for 1 hour...\r\n");

    // Simulate 1 hour of operation (actually 10 seconds for demo)
    for (uint8_t i = 0; i < 10; i++)
    {
        PORTB = ~PORTB; // Blink
        _delay_ms(1000);

        uart_transmit('.');
    }

    // Increment hours
    g_config.run_hours++;

    uart_print("\r\n\r\nUpdating to ");
    uart_print_dec(g_config.run_hours);
    uart_print(" hours\r\n");

    // Save updated hours
    config_save(&g_config);

    uart_print("Saved! Power cycle to verify\r\n");

    while (1)
        ;
}

// ============================================================================
// Demo 4: Factory Reset
// ============================================================================

void demo_04_factory_reset(void)
{
    DDRB = 0xFF;
    PORTB = 0x00;

    uart_init();

    uart_print("=== Factory Reset ===\r\n\r\n");

    // Load current config
    config_init(&g_config);

    uart_print("Current run hours: ");
    uart_print_dec(g_config.run_hours);
    uart_print("\r\n\r\n");

    uart_print("Press 'R' for factory reset...\r\n");

    char c = uart_receive();
    if (c == 'R' || c == 'r')
    {
        uart_print("\r\nResetting to factory defaults...\r\n");

        config_defaults(&g_config);
        config_save(&g_config);

        uart_print("Reset complete!\r\n");
        uart_print("All settings restored to defaults\r\n");

        // Blink confirmation
        for (uint8_t i = 0; i < 5; i++)
        {
            PORTB = 0xFF;
            _delay_ms(200);
            PORTB = 0x00;
            _delay_ms(200);
        }
    }
    else
    {
        uart_print("Reset cancelled\r\n");
    }

    while (1)
        ;
}

// ============================================================================
// MAIN - Select Demo
// ============================================================================

int main(void)
{
    // CHOOSE ONE DEMO TO RUN:

    // demo_01_load_save();       // Basic load/save
    // demo_02_config_menu();     // Interactive menu
    // demo_03_run_hours();       // Track usage
    demo_04_factory_reset(); // Reset function

    return 0;
}
