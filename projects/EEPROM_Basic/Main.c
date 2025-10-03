/*
 * ===========================================================================
 * EEPROM BASIC - Internal EEPROM Read/Write Operations
 * ATmega128 @ 16MHz, EEPROM: 4KB (0x0000-0x0FFF)
 * ===========================================================================
 *
 * FOCUS: Using internal EEPROM for non-volatile data storage
 *
 * WHY EEPROM?
 * - Non-volatile: Data persists after power-off
 * - Independent: Separate from program flash and RAM
 * - Flexible: Read/write at runtime
 * - Persistent settings: Save configuration, calibration
 *
 * EEPROM CHARACTERISTICS:
 * - Size: 4096 bytes (ATmega128)
 * - Endurance: ~100,000 write cycles per byte
 * - Read time: 4 clock cycles
 * - Write time: ~3.4ms (must wait!)
 * - Byte-addressable: 0x0000 to 0x0FFF
 *
 * LEARNING OBJECTIVES:
 * 1. Read and write EEPROM bytes
 * 2. Understand write timing and polling
 * 3. Implement wear leveling basics
 * 4. Store multi-byte data correctly
 * 5. Handle EEPROM endurance limits
 *
 * HARDWARE:
 * - ATmega128 @ 16MHz
 * - LEDs on PORTB for status display
 * - UART for debugging output
 */

#include "config.h"

// ============================================================================
// EEPROM BASIC FUNCTIONS
// ============================================================================

// Read single byte from EEPROM
uint8_t eeprom_read_byte(uint16_t address)
{
    // Wait for previous write to complete
    while (EECR & (1 << EEWE))
        ;

    // Set address
    EEAR = address;

    // Start read
    EECR |= (1 << EERE);

    // Return data (takes 4 cycles)
    return EEDR;
}

// Write single byte to EEPROM
void eeprom_write_byte(uint16_t address, uint8_t data)
{
    // Wait for previous write to complete
    while (EECR & (1 << EEWE))
        ;

    // Set address and data
    EEAR = address;
    EEDR = data;

    // Write sequence (must execute within 4 cycles)
    cli();                // Disable interrupts
    EECR |= (1 << EEMWE); // Master Write Enable
    EECR |= (1 << EEWE);  // Write Enable
    sei();                // Re-enable interrupts

    // Note: Write takes ~3.4ms to complete!
}

// Update byte only if different (saves write cycles)
void eeprom_update_byte(uint16_t address, uint8_t data)
{
    if (eeprom_read_byte(address) != data)
        eeprom_write_byte(address, data);
}

// ============================================================================
// MULTI-BYTE READ/WRITE
// ============================================================================

// Write array to EEPROM
void eeprom_write_block(uint16_t address, uint8_t *data, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        eeprom_write_byte(address + i, data[i]);
    }
}

// Read array from EEPROM
void eeprom_read_block(uint16_t address, uint8_t *data, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        data[i] = eeprom_read_byte(address + i);
    }
}

// Write 16-bit value
void eeprom_write_word(uint16_t address, uint16_t value)
{
    eeprom_write_byte(address, (uint8_t)(value & 0xFF));
    eeprom_write_byte(address + 1, (uint8_t)(value >> 8));
}

// Read 16-bit value
uint16_t eeprom_read_word(uint16_t address)
{
    uint16_t low = eeprom_read_byte(address);
    uint16_t high = eeprom_read_byte(address + 1);
    return (high << 8) | low;
}

// ============================================================================
// UART FOR DEBUGGING
// ============================================================================

void uart_init(void)
{
    UBRR0H = 0;
    UBRR0L = 103; // 9600 @ 16MHz
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

void uart_print_hex(uint8_t value)
{
    const char hex[] = "0123456789ABCDEF";
    uart_transmit(hex[value >> 4]);
    uart_transmit(hex[value & 0x0F]);
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
// Demo 1: Basic Read/Write
// ============================================================================

void demo_01_basic_read_write(void)
{
    DDRB = 0xFF;
    PORTB = 0x00;

    uart_init();

    uart_print("=== EEPROM Basic Read/Write ===\r\n\r\n");

    // Write sequence
    uart_print("Writing test values...\r\n");
    eeprom_write_byte(0x0000, 0xAA);
    eeprom_write_byte(0x0001, 0x55);
    eeprom_write_byte(0x0002, 0xFF);
    eeprom_write_byte(0x0003, 0x00);

    uart_print("Write complete!\r\n\r\n");

    // Read back
    uart_print("Reading values:\r\n");
    for (uint8_t i = 0; i < 4; i++)
    {
        uint8_t value = eeprom_read_byte(i);
        uart_print("  [0x");
        uart_print_hex(i);
        uart_print("] = 0x");
        uart_print_hex(value);
        uart_print("\r\n");

        PORTB = value;
        _delay_ms(500);
    }

    uart_print("\r\nPress RESET to run again\r\n");
    while (1)
        ;
}

// ============================================================================
// Demo 2: Counter with EEPROM Persistence
// ============================================================================

#define COUNTER_ADDR 0x0010

void demo_02_persistent_counter(void)
{
    DDRB = 0xFF;
    PORTB = 0x00;

    uart_init();

    uart_print("=== Persistent Counter ===\r\n\r\n");

    // Read previous counter value
    uint16_t counter = eeprom_read_word(COUNTER_ADDR);

    uart_print("Previous count: ");
    uart_print_dec(counter);
    uart_print("\r\n");

    // Increment
    counter++;

    uart_print("New count: ");
    uart_print_dec(counter);
    uart_print("\r\n\r\n");

    // Save new value
    eeprom_write_word(COUNTER_ADDR, counter);

    uart_print("Saved to EEPROM!\r\n");
    uart_print("Power cycle to see persistence\r\n");

    // Display on LEDs
    PORTB = (uint8_t)counter;

    while (1)
        ;
}

// ============================================================================
// Demo 3: Data Array Storage
// ============================================================================

#define ARRAY_ADDR 0x0020
#define ARRAY_SIZE 10

void demo_03_array_storage(void)
{
    DDRB = 0xFF;
    PORTB = 0x00;

    uart_init();

    uart_print("=== Array Storage ===\r\n\r\n");

    // Create test array
    uint8_t test_data[ARRAY_SIZE];
    for (uint8_t i = 0; i < ARRAY_SIZE; i++)
        test_data[i] = i * 10;

    // Write array
    uart_print("Writing array: ");
    for (uint8_t i = 0; i < ARRAY_SIZE; i++)
    {
        uart_print_dec(test_data[i]);
        uart_print(" ");
    }
    uart_print("\r\n");

    eeprom_write_block(ARRAY_ADDR, test_data, ARRAY_SIZE);
    uart_print("Write complete!\r\n\r\n");

    // Clear array
    for (uint8_t i = 0; i < ARRAY_SIZE; i++)
        test_data[i] = 0;

    // Read back
    eeprom_read_block(ARRAY_ADDR, test_data, ARRAY_SIZE);

    uart_print("Read back: ");
    for (uint8_t i = 0; i < ARRAY_SIZE; i++)
    {
        uart_print_dec(test_data[i]);
        uart_print(" ");

        PORTB = test_data[i];
        _delay_ms(300);
    }
    uart_print("\r\n");

    while (1)
        ;
}

// ============================================================================
// Demo 4: Wear Leveling Basics
// ============================================================================

/*
 * EEPROM has limited write cycles (~100,000)
 * Wear leveling spreads writes across multiple locations
 * Simple approach: Rotate through addresses
 */

#define WEAR_START 0x0100
#define WEAR_SLOTS 10

void demo_04_wear_leveling(void)
{
    DDRB = 0xFF;
    PORTB = 0x00;

    uart_init();

    uart_print("=== Wear Leveling Demo ===\r\n\r\n");

    // Find current slot (last non-zero value)
    uint8_t current_slot = 0;
    for (uint8_t i = 0; i < WEAR_SLOTS; i++)
    {
        if (eeprom_read_byte(WEAR_START + i) != 0)
            current_slot = i;
    }

    uart_print("Current slot: ");
    uart_print_dec(current_slot);
    uart_print("\r\n");

    // Read current value
    uint8_t value = eeprom_read_byte(WEAR_START + current_slot);
    uart_print("Current value: ");
    uart_print_dec(value);
    uart_print("\r\n");

    // Increment value
    value++;

    // Move to next slot if overflow
    if (value == 0)
    {
        current_slot = (current_slot + 1) % WEAR_SLOTS;
        uart_print("Rotating to slot ");
        uart_print_dec(current_slot);
        uart_print("\r\n");
    }

    // Write new value
    eeprom_write_byte(WEAR_START + current_slot, value);

    uart_print("New value: ");
    uart_print_dec(value);
    uart_print(" saved\r\n\r\n");

    // Show all slots
    uart_print("All slots: ");
    for (uint8_t i = 0; i < WEAR_SLOTS; i++)
    {
        uint8_t slot_val = eeprom_read_byte(WEAR_START + i);
        uart_print_dec(slot_val);
        uart_print(" ");
    }
    uart_print("\r\n");

    PORTB = value;

    while (1)
        ;
}

// ============================================================================
// MAIN - Select Demo
// ============================================================================

int main(void)
{
    // CHOOSE ONE DEMO TO RUN:

    // demo_01_basic_read_write();     // Simple read/write
    // demo_02_persistent_counter();   // Counter survives reset
    // demo_03_array_storage();        // Store arrays
    demo_04_wear_leveling(); // Extend EEPROM life

    return 0;
}
