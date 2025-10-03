/*
 * ===========================================================================
 * EEPROM DATA LOGGER - Sensor Data Logging with Timestamps
 * ATmega128 @ 16MHz, EEPROM for data storage
 * ===========================================================================
 *
 * FOCUS: Using EEPROM as a simple data logger for sensor readings
 *
 * DATA LOGGING APPLICATIONS:
 * - Temperature logging
 * - Event recording
 * - System diagnostics
 * - Black box recording
 * - Historical analysis
 *
 * LOGGING CHALLENGES:
 * - Limited space: 4KB EEPROM fills quickly
 * - Write endurance: ~100K cycles per location
 * - Circular buffering: Overwrite oldest data
 * - Timestamp management: Track when data recorded
 * - Data retrieval: Read back logged data
 *
 * LEARNING OBJECTIVES:
 * 1. Implement circular log buffer
 * 2. Timestamp data entries
 * 3. Handle buffer wrap-around
 * 4. Retrieve and display logs
 * 5. Calculate storage capacity
 *
 * HARDWARE:
 * - ATmega128 @ 16MHz
 * - ADC for sensor simulation
 * - Timer for timestamps
 * - UART for log retrieval
 * - LEDs for status
 */

#include "config.h"

// ============================================================================
// EEPROM FUNCTIONS
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

uint16_t eeprom_read_word(uint16_t address)
{
    uint16_t low = eeprom_read_byte(address);
    uint16_t high = eeprom_read_byte(address + 1);
    return (high << 8) | low;
}

void eeprom_write_word(uint16_t address, uint16_t value)
{
    eeprom_write_byte(address, (uint8_t)(value & 0xFF));
    eeprom_write_byte(address + 1, (uint8_t)(value >> 8));
}

// ============================================================================
// DATA LOG STRUCTURE
// ============================================================================

#define LOG_START_ADDR 0x0100
#define LOG_ENTRY_SIZE 4    // timestamp(2) + value(2)
#define MAX_LOG_ENTRIES 100 // 400 bytes total
#define LOG_END_ADDR (LOG_START_ADDR + (MAX_LOG_ENTRIES * LOG_ENTRY_SIZE))

#define LOG_INDEX_ADDR 0x0000 // Store current log position

typedef struct
{
    uint16_t timestamp; // Seconds since start
    uint16_t value;     // Sensor reading
} log_entry_t;

volatile uint16_t g_seconds = 0; // Global time counter

// ============================================================================
// TIMER FOR TIMESTAMP
// ============================================================================

// Timer1 CTC interrupt - 1 second tick
ISR(TIMER1_COMPA_vect)
{
    g_seconds++;
}

void timer_init(void)
{
    // CTC mode, prescaler 256
    TCCR1B = (1 << WGM12) | (1 << CS12);

    // Compare match for 1 second @ 16MHz/256
    OCR1A = 62500 - 1;

    // Enable compare interrupt
    TIMSK |= (1 << OCIE1A);

    sei();
}

// ============================================================================
// LOG FUNCTIONS
// ============================================================================

// Get current log write position
uint16_t log_get_index(void)
{
    return eeprom_read_word(LOG_INDEX_ADDR);
}

// Set log write position
void log_set_index(uint16_t index)
{
    eeprom_write_word(LOG_INDEX_ADDR, index);
}

// Write log entry
void log_write(uint16_t timestamp, uint16_t value)
{
    // Get current position
    uint16_t index = log_get_index();

    // Calculate EEPROM address
    uint16_t addr = LOG_START_ADDR + (index * LOG_ENTRY_SIZE);

    // Write entry
    eeprom_write_word(addr, timestamp);
    eeprom_write_word(addr + 2, value);

    // Update index (circular)
    index = (index + 1) % MAX_LOG_ENTRIES;
    log_set_index(index);
}

// Read log entry
void log_read(uint16_t index, log_entry_t *entry)
{
    uint16_t addr = LOG_START_ADDR + (index * LOG_ENTRY_SIZE);

    entry->timestamp = eeprom_read_word(addr);
    entry->value = eeprom_read_word(addr + 2);
}

// Clear all logs
void log_clear(void)
{
    log_set_index(0);

    // Optional: Erase all entries (takes time!)
    for (uint16_t i = 0; i < MAX_LOG_ENTRIES; i++)
    {
        uint16_t addr = LOG_START_ADDR + (i * LOG_ENTRY_SIZE);
        eeprom_write_word(addr, 0);
        eeprom_write_word(addr + 2, 0);
    }
}

// Get number of valid entries
uint16_t log_count(void)
{
    uint16_t index = log_get_index();

    // Check if buffer has wrapped
    log_entry_t entry;
    log_read(index, &entry);

    if (entry.timestamp == 0 && entry.value == 0)
        return index; // Not wrapped yet
    else
        return MAX_LOG_ENTRIES; // Buffer full/wrapped
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
// ADC FOR SENSOR SIMULATION
// ============================================================================

void adc_init(void)
{
    // AVcc reference, ADC0
    ADMUX = (1 << REFS0);

    // Enable ADC, prescaler 128 (125kHz @ 16MHz)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_read(void)
{
    // Start conversion
    ADCSRA |= (1 << ADSC);

    // Wait for completion
    while (ADCSRA & (1 << ADSC))
        ;

    return ADC;
}

// ============================================================================
// Demo 1: Basic Data Logging
// ============================================================================

void demo_01_basic_logging(void)
{
    DDRB = 0xFF;
    PORTB = 0x00;

    uart_init();
    timer_init();
    adc_init();

    uart_print("=== Data Logger ===\r\n\r\n");
    uart_print("Logging sensor data every 5 seconds\r\n");
    uart_print("Press RESET to view logs\r\n\r\n");

    uint16_t last_log_time = 0;

    while (1)
    {
        // Log every 5 seconds
        if (g_seconds - last_log_time >= 5)
        {
            uint16_t sensor_value = adc_read();

            log_write(g_seconds, sensor_value);

            uart_print("[");
            uart_print_dec(g_seconds);
            uart_print("s] Logged: ");
            uart_print_dec(sensor_value);
            uart_print("\r\n");

            PORTB = (uint8_t)(sensor_value >> 2); // Display on LEDs

            last_log_time = g_seconds;
        }
    }
}

// ============================================================================
// Demo 2: Log Retrieval and Display
// ============================================================================

void demo_02_view_logs(void)
{
    DDRB = 0xFF;
    PORTB = 0x00;

    uart_init();

    uart_print("=== Log Viewer ===\r\n\r\n");

    uint16_t count = log_count();

    uart_print("Total entries: ");
    uart_print_dec(count);
    uart_print("/");
    uart_print_dec(MAX_LOG_ENTRIES);
    uart_print("\r\n\r\n");

    if (count == 0)
    {
        uart_print("No logs found!\r\n");
        while (1)
            ;
    }

    uart_print("TIME   VALUE\r\n");
    uart_print("----   -----\r\n");

    // Display all entries
    for (uint16_t i = 0; i < count; i++)
    {
        log_entry_t entry;
        log_read(i, &entry);

        // Skip empty entries
        if (entry.timestamp == 0 && entry.value == 0)
            continue;

        uart_print_dec(entry.timestamp);
        uart_print("s   ");
        uart_print_dec(entry.value);
        uart_print("\r\n");

        PORTB = (uint8_t)i;
        _delay_ms(100);
    }

    uart_print("\r\nEnd of logs\r\n");

    while (1)
        ;
}

// ============================================================================
// Demo 3: Event Logger
// ============================================================================

#define EVENT_BUTTON_PRESS 0x8000
#define EVENT_THRESHOLD_HIGH 0x4000
#define EVENT_THRESHOLD_LOW 0x2000

void log_event(uint16_t event_code)
{
    log_write(g_seconds, event_code | g_seconds);
}

void demo_03_event_logger(void)
{
    DDRB = 0xFF;
    PORTB = 0x00;

    // Button on PD0
    DDRD &= ~(1 << PD0);
    PORTD |= (1 << PD0); // Pull-up

    uart_init();
    timer_init();
    adc_init();

    uart_print("=== Event Logger ===\r\n\r\n");
    uart_print("Logging system events\r\n");
    uart_print("Press button or change sensor\r\n\r\n");

    uint8_t last_button = 1;
    uint16_t last_sensor = 512;

    while (1)
    {
        // Check button
        uint8_t button = (PIND & (1 << PD0)) ? 1 : 0;
        if (button != last_button && button == 0)
        {
            log_event(EVENT_BUTTON_PRESS);
            uart_print("[");
            uart_print_dec(g_seconds);
            uart_print("s] BUTTON PRESS\r\n");
            PORTB ^= 0xFF;
        }
        last_button = button;

        // Check sensor thresholds
        uint16_t sensor = adc_read();
        if (sensor > 800 && last_sensor <= 800)
        {
            log_event(EVENT_THRESHOLD_HIGH);
            uart_print("[");
            uart_print_dec(g_seconds);
            uart_print("s] HIGH THRESHOLD\r\n");
        }
        else if (sensor < 200 && last_sensor >= 200)
        {
            log_event(EVENT_THRESHOLD_LOW);
            uart_print("[");
            uart_print_dec(g_seconds);
            uart_print("s] LOW THRESHOLD\r\n");
        }
        last_sensor = sensor;

        _delay_ms(100);
    }
}

// ============================================================================
// Demo 4: Statistical Summary
// ============================================================================

void demo_04_statistics(void)
{
    DDRB = 0xFF;
    PORTB = 0x00;

    uart_init();

    uart_print("=== Log Statistics ===\r\n\r\n");

    uint16_t count = log_count();

    if (count == 0)
    {
        uart_print("No data!\r\n");
        while (1)
            ;
    }

    uint16_t min_val = 0xFFFF;
    uint16_t max_val = 0;
    uint32_t sum = 0;
    uint16_t first_time = 0;
    uint16_t last_time = 0;

    // Calculate statistics
    for (uint16_t i = 0; i < count; i++)
    {
        log_entry_t entry;
        log_read(i, &entry);

        if (entry.timestamp == 0 && entry.value == 0)
            continue;

        if (i == 0)
            first_time = entry.timestamp;
        last_time = entry.timestamp;

        if (entry.value < min_val)
            min_val = entry.value;
        if (entry.value > max_val)
            max_val = entry.value;
        sum += entry.value;
    }

    uint16_t avg = sum / count;
    uint16_t duration = last_time - first_time;

    // Display results
    uart_print("Entries: ");
    uart_print_dec(count);
    uart_print("\r\n");

    uart_print("Duration: ");
    uart_print_dec(duration);
    uart_print("s\r\n");

    uart_print("Min: ");
    uart_print_dec(min_val);
    uart_print("\r\n");

    uart_print("Max: ");
    uart_print_dec(max_val);
    uart_print("\r\n");

    uart_print("Average: ");
    uart_print_dec(avg);
    uart_print("\r\n");

    // Visualize on LEDs
    PORTB = (uint8_t)(avg >> 2);

    while (1)
        ;
}

// ============================================================================
// MAIN - Select Demo
// ============================================================================

int main(void)
{
    // CHOOSE ONE DEMO TO RUN:

    // demo_01_basic_logging();   // Log sensor data
    demo_02_view_logs(); // View saved logs
    // demo_03_event_logger();    // Log events
    // demo_04_statistics();      // Analyze logs

    return 0;
}
