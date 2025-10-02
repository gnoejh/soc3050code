/*
 * IoT Backup - Educational Example
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Understand data backup and recovery in IoT systems
 * - Learn persistent storage and data integrity
 * - Practice fault tolerance and system recovery
 * - Master backup scheduling and data synchronization
 *
 * HARDWARE SETUP:
 * - EEPROM for backup data storage
 * - Multiple sensors for continuous data collection
 * - UART for backup data transmission
 * - LEDs for backup status indication
 */

#include "config.h"

// Backup system configuration
#define BACKUP_MAGIC_BYTE 0xBA
#define BACKUP_START_ADDR 0x100
#define BACKUP_ENTRY_SIZE 8
#define MAX_BACKUP_ENTRIES 32
#define DEVICE_ID "ATM128_BACKUP_001"

// Backup data entry structure
typedef struct
{
    uint16_t timestamp;
    uint8_t sensor_id;
    uint16_t sensor_value;
    uint8_t data_quality;
    uint8_t checksum;
    uint8_t reserved;
} backup_entry_t;

// Backup system status
typedef struct
{
    uint8_t magic_byte;
    uint16_t entry_count;
    uint16_t current_index;
    uint8_t backup_enabled;
    uint16_t last_sync_time;
    uint8_t integrity_check;
} backup_header_t;

// Global variables
backup_header_t backup_header;
uint16_t system_time = 0;
uint8_t backup_cycle = 0;

// Function to calculate simple checksum
uint8_t calculate_checksum(backup_entry_t *entry)
{
    uint8_t sum = 0;
    sum += (entry->timestamp >> 8) & 0xFF;
    sum += entry->timestamp & 0xFF;
    sum += entry->sensor_id;
    sum += (entry->sensor_value >> 8) & 0xFF;
    sum += entry->sensor_value & 0xFF;
    sum += entry->data_quality;
    return sum;
}

// Function to save backup header to EEPROM
void save_backup_header()
{
    EEPROM_write(0x01, (backup_header.entry_count >> 8) & 0xFF);
    _delay_ms(5);
    EEPROM_write(0x02, backup_header.entry_count & 0xFF);
    _delay_ms(5);
    EEPROM_write(0x03, (backup_header.current_index >> 8) & 0xFF);
    _delay_ms(5);
    EEPROM_write(0x04, backup_header.current_index & 0xFF);
    _delay_ms(5);
    EEPROM_write(0x05, backup_header.backup_enabled);
    _delay_ms(5);
}

// Function to initialize backup system
void init_backup_system()
{
    puts_USART1("Initializing backup system...\r\n");

    // Read backup header from EEPROM
    backup_header.magic_byte = EEPROM_read(0x00);

    if (backup_header.magic_byte == BACKUP_MAGIC_BYTE)
    {
        // Valid backup system found
        backup_header.entry_count = (EEPROM_read(0x01) << 8) | EEPROM_read(0x02);
        backup_header.current_index = (EEPROM_read(0x03) << 8) | EEPROM_read(0x04);
        backup_header.backup_enabled = EEPROM_read(0x05);

        char buffer[60];
        sprintf(buffer, "Backup system restored: %u entries, index %u\r\n",
                backup_header.entry_count, backup_header.current_index);
        puts_USART1(buffer);
    }
    else
    {
        // Initialize new backup system
        puts_USART1("Creating new backup system...\r\n");
        backup_header.magic_byte = BACKUP_MAGIC_BYTE;
        backup_header.entry_count = 0;
        backup_header.current_index = 0;
        backup_header.backup_enabled = 1;
        backup_header.last_sync_time = 0;

        // Write header to EEPROM
        EEPROM_write(0x00, backup_header.magic_byte);
        _delay_ms(5);
        save_backup_header();
    }
}

// Function to backup sensor data
void backup_sensor_data(uint8_t sensor_id, uint16_t sensor_value, uint8_t quality)
{
    if (!backup_header.backup_enabled)
        return;

    backup_entry_t entry;
    entry.timestamp = system_time;
    entry.sensor_id = sensor_id;
    entry.sensor_value = sensor_value;
    entry.data_quality = quality;
    entry.checksum = calculate_checksum(&entry);
    entry.reserved = 0;

    // Calculate EEPROM address
    uint16_t addr = BACKUP_START_ADDR + (backup_header.current_index * BACKUP_ENTRY_SIZE);

    // Write entry to EEPROM
    EEPROM_write(addr + 0, (entry.timestamp >> 8) & 0xFF);
    _delay_ms(5);
    EEPROM_write(addr + 1, entry.timestamp & 0xFF);
    _delay_ms(5);
    EEPROM_write(addr + 2, entry.sensor_id);
    _delay_ms(5);
    EEPROM_write(addr + 3, (entry.sensor_value >> 8) & 0xFF);
    _delay_ms(5);
    EEPROM_write(addr + 4, entry.sensor_value & 0xFF);
    _delay_ms(5);
    EEPROM_write(addr + 5, entry.data_quality);
    _delay_ms(5);
    EEPROM_write(addr + 6, entry.checksum);
    _delay_ms(5);

    // Update backup header
    backup_header.current_index = (backup_header.current_index + 1) % MAX_BACKUP_ENTRIES;
    if (backup_header.entry_count < MAX_BACKUP_ENTRIES)
        backup_header.entry_count++;

    save_backup_header();

    // Visual feedback
    PORTB = ~(1 << sensor_id);
    _delay_ms(50);
    PORTB = 0xFF;
}

// Function to transmit backup data
void transmit_backup_data()
{
    puts_USART1("BACKUP-TRANSMISSION: Starting data recovery\r\n");

    char buffer[80];
    sprintf(buffer, "BACKUP-INFO: %u entries available\r\n", backup_header.entry_count);
    puts_USART1(buffer);

    for (uint16_t i = 0; i < backup_header.entry_count && i < 10; i++) // Limit to first 10 entries
    {
        uint16_t addr = BACKUP_START_ADDR + (i * BACKUP_ENTRY_SIZE);

        // Read entry from EEPROM
        backup_entry_t entry;
        entry.timestamp = (EEPROM_read(addr + 0) << 8) | EEPROM_read(addr + 1);
        entry.sensor_id = EEPROM_read(addr + 2);
        entry.sensor_value = (EEPROM_read(addr + 3) << 8) | EEPROM_read(addr + 4);
        entry.data_quality = EEPROM_read(addr + 5);
        entry.checksum = EEPROM_read(addr + 6);

        // Verify integrity
        uint8_t calculated_checksum = calculate_checksum(&entry);
        if (calculated_checksum == entry.checksum)
        {
            sprintf(buffer, "BACKUP-DATA: T=%u,S=%u,V=%u,Q=%u,OK\r\n",
                    entry.timestamp, entry.sensor_id, entry.sensor_value, entry.data_quality);
            puts_USART1(buffer);
        }
        else
        {
            sprintf(buffer, "BACKUP-ERROR: Entry %u corrupted\r\n", i);
            puts_USART1(buffer);
        }

        // Visual progress
        PORTB = ~(1 << (i % 8));
        _delay_ms(100);
    }

    PORTB = 0xFF;
    puts_USART1("BACKUP-TRANSMISSION: Complete\r\n");
}

// Function to handle backup commands
void process_backup_commands()
{
    if (is_USART1_received())
    {
        char cmd = get_USART1();
        char response[60];

        switch (cmd)
        {
        case 'B': // Backup status
            sprintf(response, "BACKUP-STATUS: enabled=%u, entries=%u, index=%u\r\n",
                    backup_header.backup_enabled, backup_header.entry_count, backup_header.current_index);
            puts_USART1(response);
            break;

        case 'T': // Transmit backup data
            transmit_backup_data();
            break;

        case 'C': // Clear backup
            puts_USART1("BACKUP-CLEAR: Clearing all backup data...\r\n");
            backup_header.entry_count = 0;
            backup_header.current_index = 0;
            save_backup_header();
            puts_USART1("BACKUP-CLEAR: Complete\r\n");
            break;

        case 'E': // Enable/disable backup
            backup_header.backup_enabled = !backup_header.backup_enabled;
            sprintf(response, "BACKUP-TOGGLE: Backup %s\r\n",
                    backup_header.backup_enabled ? "ENABLED" : "DISABLED");
            puts_USART1(response);
            save_backup_header();
            break;

        default:
            puts_USART1("BACKUP-ERROR: Unknown command. Available: B,T,C,E\r\n");
            break;
        }
    }
}

int main(void)
{
    // Initialize system components
    init_devices();
    Adc_init();
    Uart1_init();
    EEPROM_init();

    puts_USART1("IoT Backup System Starting...\r\n");
    puts_USART1("Device: ");
    puts_USART1(DEVICE_ID);
    puts_USART1("\r\n");
    puts_USART1("Features: Data backup, Recovery, Integrity checking\r\n");
    puts_USART1("Commands: B=Status, T=Transmit, C=Clear, E=Enable/Disable\r\n");

    DDRB = 0xFF;
    PORTB = 0xFF;

    // Initialize backup system
    init_backup_system();

    puts_USART1("BACKUP-SYSTEM: Ready for operation\r\n");

    while (1)
    {
        system_time++;
        backup_cycle++;

        // Collect and backup sensor data every 5 cycles
        if (backup_cycle >= 5)
        {
            for (uint8_t sensor = 0; sensor < 3; sensor++)
            {
                uint16_t value = Adc_read_ch(sensor);
                uint8_t quality = (value > 50 && value < 950) ? 100 : 50; // Simple quality check

                backup_sensor_data(sensor, value, quality);

                char status[50];
                sprintf(status, "SENSOR-%u: %u (Q:%u) backed up\r\n", sensor, value, quality);
                puts_USART1(status);
            }
            backup_cycle = 0;
        }

        // Process backup commands
        process_backup_commands();

        // System heartbeat
        PORTB ^= (1 << 7);

        _delay_ms(1000); // 1 second cycle
    }

    return 0;
}