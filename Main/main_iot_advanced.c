#include "config.h"

// Simple math functions for embedded systems
float simple_sin(float x)
{
    // Simple sine approximation using Taylor series (first few terms)
    while (x > 3.14159)
        x -= 6.28318; // Normalize to -π to π range
    while (x < -3.14159)
        x += 6.28318;

    float x2 = x * x;
    return x - (x * x2) / 6.0 + (x * x2 * x2) / 120.0;
}

float simple_sqrt(float x)
{
    if (x <= 0)
        return 0;

    // Newton's method for square root
    float guess = x / 2.0;
    for (uint8_t i = 0; i < 10; i++)
    {
        guess = (guess + x / guess) / 2.0;
    }
    return guess;
}

/*
 * IOT_SENSOR_MONITORING - Comprehensive IoT Sensor Data Transmission
 * Educational demonstration of:
 * - Serial communication protocols for IoT
 * - JSON data formatting for web compatibility
 * - Real-time sensor data streaming
 * - Command/response protocols
 * - Data buffering and transmission reliability
 */

#ifdef IOT_SENSOR_MONITORING

// IoT Communication Protocol Configuration
#define IOT_BAUD_RATE 9600
#define IOT_BUFFER_SIZE 256
#define IOT_MAX_SENSORS 8
#define IOT_TRANSMISSION_INTERVAL 1000 // milliseconds

// Protocol message types
#define MSG_TYPE_SENSOR_DATA 0x01
#define MSG_TYPE_STATUS_UPDATE 0x02
#define MSG_TYPE_COMMAND 0x03
#define MSG_TYPE_RESPONSE 0x04
#define MSG_TYPE_HEARTBEAT 0x05

// IoT sensor data structure
typedef struct
{
    uint8_t sensor_id;
    uint8_t sensor_type;   // 0=temp, 1=light, 2=motion, 3=distance, etc.
    uint16_t raw_value;    // Raw ADC or sensor value
    float processed_value; // Processed/calibrated value
    uint8_t quality;       // Signal quality (0-100)
    uint32_t timestamp;    // System timestamp
} iot_sensor_data_t;

// IoT device status structure
typedef struct
{
    uint8_t device_id;
    uint8_t battery_level;   // Battery percentage
    uint8_t signal_strength; // Communication signal strength
    uint8_t temperature;     // Device temperature
    uint16_t uptime_hours;   // Device uptime in hours
    uint8_t error_flags;     // Error status flags
} iot_device_status_t;

// IoT communication buffer
static char iot_tx_buffer[IOT_BUFFER_SIZE];
static char iot_rx_buffer[IOT_BUFFER_SIZE];
static uint8_t iot_rx_index = 0;
static uint8_t iot_command_ready = 0;

// Sensor data arrays
static iot_sensor_data_t sensor_readings[IOT_MAX_SENSORS];
static iot_device_status_t device_status;
static uint32_t system_timestamp = 0;
static uint8_t active_sensors = 0;

// IoT protocol functions
void iot_init_communication()
{
    // Initialize UART for IoT communication
    UBRR0H = (uint8_t)(((F_CPU / (IOT_BAUD_RATE * 16UL)) - 1) >> 8);
    UBRR0L = (uint8_t)((F_CPU / (IOT_BAUD_RATE * 16UL)) - 1);

    // Enable transmitter and receiver
    UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);

    // Set frame format: 8 data bits, 1 stop bit, no parity
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    // Initialize device status
    device_status.device_id = 0x42; // Unique device ID
    device_status.battery_level = 95;
    device_status.signal_strength = 85;
    device_status.temperature = 25;
    device_status.uptime_hours = 0;
    device_status.error_flags = 0;

    // Clear buffers
    memset(iot_tx_buffer, 0, IOT_BUFFER_SIZE);
    memset(iot_rx_buffer, 0, IOT_BUFFER_SIZE);
    iot_rx_index = 0;
    iot_command_ready = 0;
}

void iot_send_char(char c)
{
    // Wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    UDR0 = c;
}

void iot_send_string(const char *str)
{
    while (*str)
    {
        iot_send_char(*str++);
    }
}

void iot_send_json_sensor_data(iot_sensor_data_t *sensor)
{
    // Format sensor data as JSON
    snprintf(iot_tx_buffer, IOT_BUFFER_SIZE,
             "{\"type\":\"sensor\",\"id\":%d,\"sensor_type\":%d,\"raw\":%d,\"value\":%.2f,\"quality\":%d,\"timestamp\":%lu}\\n",
             sensor->sensor_id,
             sensor->sensor_type,
             sensor->raw_value,
             sensor->processed_value,
             sensor->quality,
             sensor->timestamp);

    iot_send_string(iot_tx_buffer);
}

void iot_send_device_status()
{
    // Format device status as JSON
    snprintf(iot_tx_buffer, IOT_BUFFER_SIZE,
             "{\"type\":\"status\",\"device_id\":%d,\"battery\":%d,\"signal\":%d,\"temp\":%d,\"uptime\":%d,\"errors\":%d}\\n",
             device_status.device_id,
             device_status.battery_level,
             device_status.signal_strength,
             device_status.temperature,
             device_status.uptime_hours,
             device_status.error_flags);

    iot_send_string(iot_tx_buffer);
}

void iot_send_heartbeat()
{
    // Simple heartbeat message
    snprintf(iot_tx_buffer, IOT_BUFFER_SIZE,
             "{\"type\":\"heartbeat\",\"timestamp\":%lu,\"active_sensors\":%d}\\n",
             system_timestamp,
             active_sensors);

    iot_send_string(iot_tx_buffer);
}

// UART receive interrupt handler
ISR(USART0_RX_vect)
{
    char received_char = UDR0;

    if (received_char == '\\n' || received_char == '\\r')
    {
        iot_rx_buffer[iot_rx_index] = '\\0';
        iot_command_ready = 1;
        iot_rx_index = 0;
    }
    else if (iot_rx_index < IOT_BUFFER_SIZE - 1)
    {
        iot_rx_buffer[iot_rx_index++] = received_char;
    }
}

void iot_process_command()
{
    if (!iot_command_ready)
        return;

    // Simple command parsing
    if (strncmp(iot_rx_buffer, "GET_STATUS", 10) == 0)
    {
        iot_send_device_status();
    }
    else if (strncmp(iot_rx_buffer, "GET_SENSORS", 11) == 0)
    {
        for (uint8_t i = 0; i < active_sensors; i++)
        {
            iot_send_json_sensor_data(&sensor_readings[i]);
        }
    }
    else if (strncmp(iot_rx_buffer, "PING", 4) == 0)
    {
        iot_send_string("{\"type\":\"pong\"}\\n");
    }
    else if (strncmp(iot_rx_buffer, "RESET", 5) == 0)
    {
        // Simulate device reset
        system_timestamp = 0;
        device_status.uptime_hours = 0;
        iot_send_string("{\"type\":\"reset_ok\"}\\n");
    }

    iot_command_ready = 0;
}

void iot_read_sensors()
{
    // Temperature sensor simulation (ADC channel 0)
    ADMUX = (1 << REFS0) | 0; // AVcc reference, channel 0
    ADCSRA |= (1 << ADSC);    // Start conversion
    while (ADCSRA & (1 << ADSC))
        ; // Wait for completion

    uint16_t temp_raw = ADC;
    float temperature = (temp_raw * 5.0 / 1024.0 - 0.5) * 100; // LM35 formula

    sensor_readings[0].sensor_id = 0;
    sensor_readings[0].sensor_type = 0; // Temperature
    sensor_readings[0].raw_value = temp_raw;
    sensor_readings[0].processed_value = temperature;
    sensor_readings[0].quality = (temp_raw > 50) ? 95 : 70; // Quality based on signal
    sensor_readings[0].timestamp = system_timestamp;

    // Light sensor simulation (ADC channel 1)
    ADMUX = (1 << REFS0) | 1; // AVcc reference, channel 1
    ADCSRA |= (1 << ADSC);    // Start conversion
    while (ADCSRA & (1 << ADSC))
        ; // Wait for completion

    uint16_t light_raw = ADC;
    float light_percent = (light_raw * 100.0) / 1024.0;

    sensor_readings[1].sensor_id = 1;
    sensor_readings[1].sensor_type = 1; // Light
    sensor_readings[1].raw_value = light_raw;
    sensor_readings[1].processed_value = light_percent;
    sensor_readings[1].quality = 90;
    sensor_readings[1].timestamp = system_timestamp;

    // Motion sensor simulation (digital input on PINF)
    uint8_t motion_detected = (PINF & 0x01) ? 0 : 1; // Active low

    sensor_readings[2].sensor_id = 2;
    sensor_readings[2].sensor_type = 2; // Motion
    sensor_readings[2].raw_value = motion_detected;
    sensor_readings[2].processed_value = motion_detected;
    sensor_readings[2].quality = 85;
    sensor_readings[2].timestamp = system_timestamp;

    // Distance sensor simulation (ADC channel 2)
    ADMUX = (1 << REFS0) | 2; // AVcc reference, channel 2
    ADCSRA |= (1 << ADSC);    // Start conversion
    while (ADCSRA & (1 << ADSC))
        ; // Wait for completion

    uint16_t distance_raw = ADC;
    float distance_cm = (distance_raw * 200.0) / 1024.0; // 0-200cm range

    sensor_readings[3].sensor_id = 3;
    sensor_readings[3].sensor_type = 3; // Distance
    sensor_readings[3].raw_value = distance_raw;
    sensor_readings[3].processed_value = distance_cm;
    sensor_readings[3].quality = (distance_raw > 20) ? 88 : 60;
    sensor_readings[3].timestamp = system_timestamp;

    active_sensors = 4; // Number of active sensors
}

void iot_update_device_status()
{
    // Simulate battery drain
    if (system_timestamp % 3600 == 0 && device_status.battery_level > 0)
    {
        device_status.battery_level--;
    }

    // Update uptime
    device_status.uptime_hours = system_timestamp / 3600;

    // Simulate temperature variations
    device_status.temperature = 25 + (system_timestamp % 10) - 5;

    // Update signal strength based on sensor quality
    uint16_t avg_quality = 0;
    for (uint8_t i = 0; i < active_sensors; i++)
    {
        avg_quality += sensor_readings[i].quality;
    }
    device_status.signal_strength = avg_quality / active_sensors;
}

void iot_display_status()
{
    glcd_clear();
    glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);

    char buffer[25];

    glcd_tiny_draw_string(0, 0, "IoT SENSOR MONITOR:");

    sprintf(buffer, "Timestamp: %lu", system_timestamp);
    glcd_tiny_draw_string(0, 15, buffer);

    sprintf(buffer, "Active sensors: %d", active_sensors);
    glcd_tiny_draw_string(0, 25, buffer);

    sprintf(buffer, "Battery: %d%%", device_status.battery_level);
    glcd_tiny_draw_string(0, 35, buffer);

    sprintf(buffer, "Signal: %d%%", device_status.signal_strength);
    glcd_tiny_draw_string(0, 45, buffer);

    // Display latest sensor readings
    if (active_sensors > 0)
    {
        sprintf(buffer, "Temp: %.1fC", sensor_readings[0].processed_value);
        glcd_tiny_draw_string(0, 60, buffer);
    }

    if (active_sensors > 1)
    {
        sprintf(buffer, "Light: %.0f%%", sensor_readings[1].processed_value);
        glcd_tiny_draw_string(0, 70, buffer);
    }

    if (active_sensors > 2)
    {
        sprintf(buffer, "Motion: %s", sensor_readings[2].processed_value ? "YES" : "NO");
        glcd_tiny_draw_string(0, 80, buffer);
    }

    if (active_sensors > 3)
    {
        sprintf(buffer, "Dist: %.0fcm", sensor_readings[3].processed_value);
        glcd_tiny_draw_string(0, 90, buffer);
    }

    // Connection status indicator
    glcd_tiny_draw_string(0, 110, "Status: TRANSMITTING");

    // Visual indicators
    for (uint8_t i = 0; i < active_sensors; i++)
    {
        uint8_t x = 100 + i * 6;
        uint8_t quality_height = (sensor_readings[i].quality * 20) / 100;

        for (uint8_t y = 0; y < quality_height && y < 20; y++)
        {
            glcd_set_pixel(x, 127 - y, 1);
        }
    }
}

void main_iot_sensor_monitoring()
{
    DDRA = 0xFF;  // LED outputs for status indication
    DDRB = 0xFF;  // Status outputs
    DDRF = 0x00;  // Digital inputs (motion sensor, buttons)
    PORTF = 0xFF; // Enable pull-ups

    // Initialize systems
    init_GLCD();
    init_ADC();
    iot_init_communication();
    sei(); // Enable global interrupts

    glcd_clear();
    glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);

    uint32_t last_transmission = 0;
    uint32_t last_heartbeat = 0;
    uint32_t last_status_update = 0;

    while (1)
    {
        // Update system timestamp (approximate milliseconds)
        system_timestamp++;

        // Read sensors every cycle
        iot_read_sensors();

        // Update device status periodically
        if (system_timestamp - last_status_update > 10000)
        {
            iot_update_device_status();
            last_status_update = system_timestamp;
        }

        // Send sensor data at regular intervals
        if (system_timestamp - last_transmission > IOT_TRANSMISSION_INTERVAL)
        {
            for (uint8_t i = 0; i < active_sensors; i++)
            {
                iot_send_json_sensor_data(&sensor_readings[i]);
                _delay_ms(50); // Small delay between transmissions
            }
            last_transmission = system_timestamp;
        }

        // Send heartbeat every 5 seconds
        if (system_timestamp - last_heartbeat > 5000)
        {
            iot_send_heartbeat();
            last_heartbeat = system_timestamp;
        }

        // Process incoming commands
        iot_process_command();

        // Update display every 100ms
        if (system_timestamp % 100 == 0)
        {
            iot_display_status();
        }

        // Visual feedback on LEDs
        PORTA = (uint8_t)(system_timestamp & 0xFF);
        PORTB = device_status.battery_level;

        // Small delay for system timing
        _delay_ms(10);

        // Check for exit condition (all buttons pressed)
        if ((PINF & 0x0F) == 0x00)
        {
            break;
        }
    }

    // Send shutdown message
    iot_send_string("{\"type\":\"shutdown\",\"timestamp\":");
    sprintf(iot_tx_buffer, "%lu", system_timestamp);
    iot_send_string(iot_tx_buffer);
    iot_send_string("}\\n");

    glcd_clear();
    glcd_tiny_draw_string(30, 40, "IoT DEMO");
    glcd_tiny_draw_string(25, 55, "COMPLETE");
    _delay_ms(2000);
}

#endif

#ifdef IOT_REMOTE_CONTROL
// Remote Control via IoT - Comprehensive Hardware Control System
// Educational demonstration of:
// - Bidirectional IoT communication
// - Remote hardware control protocols
// - Real-time command processing
// - Safety and security features
// - Device state synchronization

#define REMOTE_BAUD_RATE 9600
#define REMOTE_BUFFER_SIZE 128
#define MAX_DEVICES 8
#define COMMAND_TIMEOUT 5000 // 5 seconds

// Device types that can be controlled
typedef enum
{
    DEVICE_LED = 0,
    DEVICE_MOTOR = 1,
    DEVICE_SERVO = 2,
    DEVICE_BUZZER = 3,
    DEVICE_RELAY = 4,
    DEVICE_PWM = 5,
    DEVICE_DIGITAL_OUT = 6,
    DEVICE_ANALOG_OUT = 7
} device_type_t;

// Device control structure
typedef struct
{
    uint8_t device_id;
    device_type_t type;
    uint8_t pin;            // Hardware pin assignment
    uint16_t current_value; // Current device value/state
    uint16_t target_value;  // Target value to reach
    uint8_t enabled;        // Device enabled flag
    uint32_t last_update;   // Last update timestamp
    char name[16];          // Human-readable device name
} remote_device_t;

// Command structure for remote control
typedef struct
{
    uint8_t device_id;
    uint16_t value;
    uint8_t duration;     // For timed operations
    uint8_t command_type; // SET, GET, TOGGLE, etc.
    uint32_t timestamp;
} remote_command_t;

// System status structure
typedef struct
{
    uint8_t security_level; // 0=open, 1=basic, 2=secure
    uint8_t remote_enabled; // Remote control enabled flag
    uint8_t devices_active; // Number of active devices
    uint32_t commands_processed;
    uint32_t last_command_time;
    uint8_t connection_status; // 0=disconnected, 1=connected
} remote_system_status_t;

// Global variables
static remote_device_t controlled_devices[MAX_DEVICES];
static remote_system_status_t system_status;
static char remote_rx_buffer[REMOTE_BUFFER_SIZE];
static char remote_tx_buffer[REMOTE_BUFFER_SIZE];
static uint8_t remote_rx_index = 0;
static uint8_t remote_command_pending = 0;
static uint32_t remote_timestamp = 0;

// Device control functions
void remote_init_devices()
{
    // Initialize LED device (PORTA)
    controlled_devices[0] = (remote_device_t){
        .device_id = 0, .type = DEVICE_LED, .pin = 0, .current_value = 0, .target_value = 0, .enabled = 1, .last_update = 0, .name = "LED_PORTA"};

    // Initialize Motor device (PWM on PORTB)
    controlled_devices[1] = (remote_device_t){
        .device_id = 1, .type = DEVICE_MOTOR, .pin = 4, .current_value = 0, .target_value = 0, .enabled = 1, .last_update = 0, .name = "DC_MOTOR"};

    // Initialize Servo device (PWM on PORTB)
    controlled_devices[2] = (remote_device_t){
        .device_id = 2, .type = DEVICE_SERVO, .pin = 5, .current_value = 90, .target_value = 90, .enabled = 1, .last_update = 0, .name = "SERVO"};

    // Initialize Buzzer device (PORTC)
    controlled_devices[3] = (remote_device_t){
        .device_id = 3, .type = DEVICE_BUZZER, .pin = 0, .current_value = 0, .target_value = 0, .enabled = 1, .last_update = 0, .name = "BUZZER"};

    // Initialize Relay device (PORTD)
    controlled_devices[4] = (remote_device_t){
        .device_id = 4, .type = DEVICE_RELAY, .pin = 2, .current_value = 0, .target_value = 0, .enabled = 1, .last_update = 0, .name = "RELAY"};

    // Initialize PWM device (Timer2)
    controlled_devices[5] = (remote_device_t){
        .device_id = 5, .type = DEVICE_PWM, .pin = 7, .current_value = 0, .target_value = 0, .enabled = 1, .last_update = 0, .name = "PWM_OUT"};

    system_status.devices_active = 6;
    system_status.security_level = 1;
    system_status.remote_enabled = 1;
    system_status.commands_processed = 0;
    system_status.last_command_time = 0;
    system_status.connection_status = 0;
}

void remote_init_communication()
{
    // Initialize UART for remote control
    UBRR0H = (uint8_t)(((F_CPU / (REMOTE_BAUD_RATE * 16UL)) - 1) >> 8);
    UBRR0L = (uint8_t)((F_CPU / (REMOTE_BAUD_RATE * 16UL)) - 1);

    UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    // Initialize hardware ports
    DDRA = 0xFF; // LED outputs
    DDRB = 0xFF; // Motor/Servo PWM outputs
    DDRC = 0xFF; // Buzzer and control outputs
    DDRD = 0xFF; // Relay outputs

    // Initialize PWM for motor control (Timer0)
    TCCR0 = (1 << WGM00) | (1 << COM01) | (1 << CS01); // Fast PWM, prescaler 8

    // Initialize PWM for servo control (Timer1)
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    ICR1 = 2499; // 50Hz frequency for servo
}

void remote_send_char(char c)
{
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    UDR0 = c;
}

void remote_send_string(const char *str)
{
    while (*str)
    {
        remote_send_char(*str++);
    }
}

void remote_update_device(uint8_t device_id, uint16_t value)
{
    if (device_id >= system_status.devices_active)
        return;

    remote_device_t *device = &controlled_devices[device_id];
    if (!device->enabled)
        return;

    device->target_value = value;
    device->last_update = remote_timestamp;

    // Apply the control value to hardware
    switch (device->type)
    {
    case DEVICE_LED:
        PORTA = (uint8_t)value;
        device->current_value = value;
        break;

    case DEVICE_MOTOR:
        OCR0 = (uint8_t)((value * 255) / 100); // Convert percentage to PWM
        device->current_value = value;
        break;

    case DEVICE_SERVO:
        // Convert angle (0-180) to PWM duty cycle for servo
        uint16_t servo_pwm = 125 + (value * 250) / 180; // 1ms to 2ms pulse width
        OCR1A = servo_pwm;
        device->current_value = value;
        break;

    case DEVICE_BUZZER:
        if (value > 0)
        {
            PORTC |= (1 << device->pin);
        }
        else
        {
            PORTC &= ~(1 << device->pin);
        }
        device->current_value = value;
        break;

    case DEVICE_RELAY:
        if (value > 0)
        {
            PORTD |= (1 << device->pin);
        }
        else
        {
            PORTD &= ~(1 << device->pin);
        }
        device->current_value = value;
        break;

    case DEVICE_PWM:
        // Use Timer2 for additional PWM output
        TCCR2 = (1 << WGM20) | (1 << COM21) | (1 << CS21);
        OCR2 = (uint8_t)((value * 255) / 100);
        device->current_value = value;
        break;
    }
}

void remote_send_device_status(uint8_t device_id)
{
    if (device_id >= system_status.devices_active)
        return;

    remote_device_t *device = &controlled_devices[device_id];

    snprintf(remote_tx_buffer, REMOTE_BUFFER_SIZE,
             "{\"type\":\"device_status\",\"id\":%d,\"name\":\"%s\",\"type\":%d,\"value\":%d,\"enabled\":%d,\"pin\":%d}\\n",
             device->device_id,
             device->name,
             device->type,
             device->current_value,
             device->enabled,
             device->pin);

    remote_send_string(remote_tx_buffer);
}

void remote_send_system_status()
{
    snprintf(remote_tx_buffer, REMOTE_BUFFER_SIZE,
             "{\"type\":\"system_status\",\"security\":%d,\"enabled\":%d,\"devices\":%d,\"commands\":%lu,\"connected\":%d}\\n",
             system_status.security_level,
             system_status.remote_enabled,
             system_status.devices_active,
             system_status.commands_processed,
             system_status.connection_status);

    remote_send_string(remote_tx_buffer);
}

// UART receive interrupt for remote commands
ISR(USART0_RX_vect)
{
    char received = UDR0;

    if (received == '\\n' || received == '\\r')
    {
        remote_rx_buffer[remote_rx_index] = '\\0';
        remote_command_pending = 1;
        remote_rx_index = 0;
        system_status.connection_status = 1;
        system_status.last_command_time = remote_timestamp;
    }
    else if (remote_rx_index < REMOTE_BUFFER_SIZE - 1)
    {
        remote_rx_buffer[remote_rx_index++] = received;
    }
}

void remote_process_command()
{
    if (!remote_command_pending || !system_status.remote_enabled)
        return;

    // Parse commands in format: CMD:DEVICE:VALUE
    char *cmd = strtok(remote_rx_buffer, ":");
    char *device_str = strtok(NULL, ":");
    char *value_str = strtok(NULL, ":");

    if (cmd && device_str)
    {
        uint8_t device_id = atoi(device_str);
        uint16_t value = value_str ? atoi(value_str) : 0;

        if (strcmp(cmd, "SET") == 0)
        {
            remote_update_device(device_id, value);
            remote_send_string("{\"response\":\"SET_OK\"}\\n");
            system_status.commands_processed++;
        }
        else if (strcmp(cmd, "GET") == 0)
        {
            remote_send_device_status(device_id);
        }
        else if (strcmp(cmd, "TOGGLE") == 0)
        {
            if (device_id < system_status.devices_active)
            {
                uint16_t new_value = controlled_devices[device_id].current_value ? 0 : 100;
                remote_update_device(device_id, new_value);
                remote_send_string("{\"response\":\"TOGGLE_OK\"}\\n");
                system_status.commands_processed++;
            }
        }
        else if (strcmp(cmd, "STATUS") == 0)
        {
            remote_send_system_status();
        }
        else if (strcmp(cmd, "LIST") == 0)
        {
            for (uint8_t i = 0; i < system_status.devices_active; i++)
            {
                remote_send_device_status(i);
                _delay_ms(10);
            }
        }
        else if (strcmp(cmd, "ENABLE") == 0)
        {
            if (device_id < system_status.devices_active)
            {
                controlled_devices[device_id].enabled = value ? 1 : 0;
                remote_send_string("{\"response\":\"ENABLE_OK\"}\\n");
            }
        }
        else if (strcmp(cmd, "SECURITY") == 0)
        {
            system_status.security_level = value % 3;
            remote_send_string("{\"response\":\"SECURITY_OK\"}\\n");
        }
        else if (strcmp(cmd, "SHUTDOWN") == 0)
        {
            // Emergency shutdown - turn off all devices
            for (uint8_t i = 0; i < system_status.devices_active; i++)
            {
                remote_update_device(i, 0);
            }
            system_status.remote_enabled = 0;
            remote_send_string("{\"response\":\"SHUTDOWN_OK\"}\\n");
        }
        else
        {
            remote_send_string("{\"error\":\"UNKNOWN_COMMAND\"}\\n");
        }
    }

    remote_command_pending = 0;
}

void remote_display_status()
{
    glcd_clear();
    glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);

    char buffer[25];

    glcd_tiny_draw_string(0, 0, "IoT REMOTE CONTROL:");

    sprintf(buffer, "Connected: %s", system_status.connection_status ? "YES" : "NO");
    glcd_tiny_draw_string(0, 15, buffer);

    sprintf(buffer, "Security: %d", system_status.security_level);
    glcd_tiny_draw_string(0, 25, buffer);

    sprintf(buffer, "Commands: %lu", system_status.commands_processed);
    glcd_tiny_draw_string(0, 35, buffer);

    sprintf(buffer, "Devices: %d", system_status.devices_active);
    glcd_tiny_draw_string(0, 45, buffer);

    // Display device states
    for (uint8_t i = 0; i < 4 && i < system_status.devices_active; i++)
    {
        sprintf(buffer, "%s: %d", controlled_devices[i].name, controlled_devices[i].current_value);
        glcd_tiny_draw_string(0, 60 + i * 10, buffer);
    }

    // Visual device status indicators
    for (uint8_t i = 0; i < system_status.devices_active && i < 8; i++)
    {
        uint8_t x = 100 + (i % 4) * 6;
        uint8_t y = 100 + (i / 4) * 12;

        // Draw device activity indicator
        if (controlled_devices[i].enabled && controlled_devices[i].current_value > 0)
        {
            for (uint8_t px = 0; px < 4; px++)
            {
                for (uint8_t py = 0; py < 8; py++)
                {
                    glcd_set_pixel(x + px, y + py, 1);
                }
            }
        }
        else
        {
            // Draw outline only
            for (uint8_t px = 0; px < 4; px++)
            {
                glcd_set_pixel(x + px, y, 1);
                glcd_set_pixel(x + px, y + 7, 1);
            }
            for (uint8_t py = 0; py < 8; py++)
            {
                glcd_set_pixel(x, y + py, 1);
                glcd_set_pixel(x + 3, y + py, 1);
            }
        }
    }

    // Connection timeout check
    if (remote_timestamp - system_status.last_command_time > COMMAND_TIMEOUT)
    {
        system_status.connection_status = 0;
    }
}

void remote_demo_sequence()
{
    glcd_clear();
    glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);
    glcd_tiny_draw_string(0, 0, "DEMO SEQUENCE:");

    // Demo LED control
    glcd_tiny_draw_string(0, 20, "Demo: LED Pattern");
    for (uint8_t i = 0; i < 8; i++)
    {
        remote_update_device(0, 1 << i); // LED device
        _delay_ms(200);
    }

    // Demo Motor control
    glcd_tiny_draw_string(0, 40, "Demo: Motor Speed");
    for (uint8_t speed = 0; speed <= 100; speed += 10)
    {
        remote_update_device(1, speed); // Motor device
        _delay_ms(300);
    }
    remote_update_device(1, 0); // Stop motor

    // Demo Servo control
    glcd_tiny_draw_string(0, 60, "Demo: Servo Sweep");
    for (uint8_t angle = 0; angle <= 180; angle += 30)
    {
        remote_update_device(2, angle); // Servo device
        _delay_ms(500);
    }
    remote_update_device(2, 90); // Center position

    // Demo Buzzer
    glcd_tiny_draw_string(0, 80, "Demo: Buzzer Test");
    for (uint8_t i = 0; i < 3; i++)
    {
        remote_update_device(3, 1); // Buzzer on
        _delay_ms(200);
        remote_update_device(3, 0); // Buzzer off
        _delay_ms(200);
    }

    glcd_clear();
    glcd_tiny_draw_string(20, 40, "DEMO COMPLETE");
    _delay_ms(1000);
}

void main_iot_remote_control()
{
    // Initialize hardware
    remote_init_devices();
    remote_init_communication();
    init_GLCD();
    sei(); // Enable global interrupts

    glcd_clear();
    glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);

    uint32_t last_display_update = 0;
    uint32_t demo_timer = 0;
    uint8_t demo_mode = 0;

    // Send startup message
    remote_send_string("{\"type\":\"startup\",\"version\":\"IoT_Remote_v1.0\"}\\n");

    while (1)
    {
        remote_timestamp++;

        // Process incoming commands
        remote_process_command();

        // Update display every 100ms
        if (remote_timestamp - last_display_update > 100)
        {
            remote_display_status();
            last_display_update = remote_timestamp;
        }

        // Check for demo mode activation (button press)
        if ((PINF & 0x01) == 0 && demo_mode == 0)
        {
            demo_mode = 1;
            demo_timer = remote_timestamp;
            remote_demo_sequence();
        }
        else if ((PINF & 0x01) != 0)
        {
            demo_mode = 0;
        }

        // Visual feedback
        PORTA = (uint8_t)(remote_timestamp & 0xFF);
        PORTB = system_status.connection_status ? 0xFF : 0x00;

        _delay_ms(10);

        // Exit condition (multiple buttons pressed)
        if ((PINF & 0x0F) == 0x00)
        {
            break;
        }
    }

    // Shutdown sequence
    for (uint8_t i = 0; i < system_status.devices_active; i++)
    {
        remote_update_device(i, 0);
    }

    remote_send_string("{\"type\":\"shutdown\",\"message\":\"Remote control stopped\"}\\n");

    glcd_clear();
    glcd_tiny_draw_string(15, 40, "REMOTE CONTROL");
    glcd_tiny_draw_string(30, 55, "STOPPED");
    _delay_ms(2000);
}

#endif

#ifdef IOT_DATA_VISUALIZATION
// Real-time Data Visualization and Web Dashboard
// Educational demonstration of:
// - Real-time data plotting and visualization
// - Multi-channel data streaming
// - Statistical analysis and trend detection
// - Interactive dashboard protocols
// - Data aggregation and filtering

#define VIZ_BAUD_RATE 115200 // Higher baud rate for data streaming
#define VIZ_BUFFER_SIZE 256
#define MAX_CHANNELS 6
#define SAMPLE_RATE_HZ 10
#define HISTORY_SIZE 100

// Data channel types
typedef enum
{
    CHANNEL_ANALOG = 0,
    CHANNEL_DIGITAL = 1,
    CHANNEL_CALCULATED = 2,
    CHANNEL_FREQUENCY = 3,
    CHANNEL_WAVEFORM = 4
} channel_type_t;

// Data channel structure
typedef struct
{
    uint8_t channel_id;
    channel_type_t type;
    char name[16];
    char unit[8];
    float min_value;
    float max_value;
    float current_value;
    float average;
    float peak_value;
    uint8_t enabled;
    uint32_t sample_count;
    uint16_t update_rate_ms;
    uint32_t last_update;
} viz_channel_t;

// Data sample structure for history
typedef struct
{
    uint32_t timestamp;
    float values[MAX_CHANNELS];
    uint8_t valid_mask; // Bit mask for valid channels
} data_sample_t;

// Statistics structure
typedef struct
{
    float mean;
    float std_dev;
    float min_val;
    float max_val;
    uint32_t sample_count;
} channel_stats_t;

// Visualization system status
typedef struct
{
    uint8_t streaming_active;
    uint8_t dashboard_connected;
    uint32_t samples_transmitted;
    uint32_t data_rate_bps;
    uint8_t visualization_mode; // 0=realtime, 1=buffered, 2=analysis
    uint16_t display_timespan;  // Timespan in seconds
} viz_system_t;

// Global variables
static viz_channel_t data_channels[MAX_CHANNELS];
static data_sample_t sample_history[HISTORY_SIZE];
static channel_stats_t channel_statistics[MAX_CHANNELS];
static viz_system_t viz_system;
static char viz_tx_buffer[VIZ_BUFFER_SIZE];
static char viz_rx_buffer[VIZ_BUFFER_SIZE];
static uint8_t viz_rx_index = 0;
static uint8_t viz_command_ready = 0;
static uint32_t viz_timestamp = 0;
static uint16_t history_write_index = 0;

void viz_init_channels()
{
    // Temperature channel (ADC0)
    data_channels[0] = (viz_channel_t){
        .channel_id = 0, .type = CHANNEL_ANALOG, .name = "Temperature", .unit = "°C", .min_value = -40, .max_value = 125, .current_value = 0, .average = 0, .peak_value = 0, .enabled = 1, .sample_count = 0, .update_rate_ms = 100, .last_update = 0};

    // Light sensor channel (ADC1)
    data_channels[1] = (viz_channel_t){
        .channel_id = 1, .type = CHANNEL_ANALOG, .name = "Light", .unit = "%", .min_value = 0, .max_value = 100, .current_value = 0, .average = 0, .peak_value = 0, .enabled = 1, .sample_count = 0, .update_rate_ms = 200, .last_update = 0};

    // Pressure/Distance channel (ADC2)
    data_channels[2] = (viz_channel_t){
        .channel_id = 2, .type = CHANNEL_ANALOG, .name = "Pressure", .unit = "kPa", .min_value = 0, .max_value = 200, .current_value = 0, .average = 0, .peak_value = 0, .enabled = 1, .sample_count = 0, .update_rate_ms = 150, .last_update = 0};

    // Motion detection (Digital)
    data_channels[3] = (viz_channel_t){
        .channel_id = 3, .type = CHANNEL_DIGITAL, .name = "Motion", .unit = "bool", .min_value = 0, .max_value = 1, .current_value = 0, .average = 0, .peak_value = 0, .enabled = 1, .sample_count = 0, .update_rate_ms = 50, .last_update = 0};

    // Calculated channel (Power consumption)
    data_channels[4] = (viz_channel_t){
        .channel_id = 4, .type = CHANNEL_CALCULATED, .name = "Power", .unit = "W", .min_value = 0, .max_value = 50, .current_value = 0, .average = 0, .peak_value = 0, .enabled = 1, .sample_count = 0, .update_rate_ms = 300, .last_update = 0};

    // Waveform generator (Sine wave)
    data_channels[5] = (viz_channel_t){
        .channel_id = 5, .type = CHANNEL_WAVEFORM, .name = "Waveform", .unit = "V", .min_value = -5, .max_value = 5, .current_value = 0, .average = 0, .peak_value = 0, .enabled = 1, .sample_count = 0, .update_rate_ms = 80, .last_update = 0};

    // Initialize system
    viz_system.streaming_active = 0;
    viz_system.dashboard_connected = 0;
    viz_system.samples_transmitted = 0;
    viz_system.data_rate_bps = 0;
    viz_system.visualization_mode = 0;
    viz_system.display_timespan = 60;

    // Clear sample history
    memset(sample_history, 0, sizeof(sample_history));
    history_write_index = 0;
}

void viz_init_communication()
{
    // High-speed UART for data visualization
    uint16_t ubrr = ((F_CPU / (VIZ_BAUD_RATE * 16UL)) - 1);
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)(ubrr);

    UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    // Initialize ADC for sensor readings
    ADMUX = (1 << REFS0);                                              // AVcc reference
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable, prescaler 128
}

void viz_send_char(char c)
{
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    UDR0 = c;
}

void viz_send_string(const char *str)
{
    while (*str)
    {
        viz_send_char(*str++);
    }
}

void viz_read_sensors()
{
    for (uint8_t i = 0; i < MAX_CHANNELS; i++)
    {
        if (!data_channels[i].enabled)
            continue;
        if (viz_timestamp - data_channels[i].last_update < data_channels[i].update_rate_ms)
            continue;

        float new_value = 0;

        switch (data_channels[i].type)
        {
        case CHANNEL_ANALOG:
        {
            // Read ADC channel
            ADMUX = (ADMUX & 0xF0) | i; // Select channel
            ADCSRA |= (1 << ADSC);      // Start conversion
            while (ADCSRA & (1 << ADSC))
                ; // Wait for completion

            uint16_t adc_value = ADC;

            if (i == 0)
            {                                                       // Temperature
                new_value = (adc_value * 5.0 / 1024.0 - 0.5) * 100; // LM35
            }
            else if (i == 1)
            { // Light
                new_value = (adc_value * 100.0) / 1024.0;
            }
            else if (i == 2)
            { // Pressure
                new_value = (adc_value * 200.0) / 1024.0;
            }
        }
        break;

        case CHANNEL_DIGITAL:
            // Motion sensor on PINF
            new_value = (PINF & (1 << i)) ? 0 : 1;
            break;

        case CHANNEL_CALCULATED:
            // Power calculation based on other sensors
            new_value = (data_channels[0].current_value * data_channels[1].current_value) / 100.0;
            break;

        case CHANNEL_WAVEFORM:
            // Generate sine wave
            new_value = 2.5 * simple_sin((viz_timestamp * 0.01) + (i * 1.5));
            break;
        }

        // Update channel data
        data_channels[i].current_value = new_value;
        data_channels[i].sample_count++;
        data_channels[i].last_update = viz_timestamp;

        // Update statistics
        if (data_channels[i].sample_count == 1)
        {
            data_channels[i].average = new_value;
            data_channels[i].peak_value = new_value;
            channel_statistics[i].min_val = new_value;
            channel_statistics[i].max_val = new_value;
        }
        else
        {
            // Update running average
            data_channels[i].average = (data_channels[i].average * (data_channels[i].sample_count - 1) + new_value) / data_channels[i].sample_count;

            // Update peak and statistics
            if (new_value > data_channels[i].peak_value)
            {
                data_channels[i].peak_value = new_value;
            }
            if (new_value > channel_statistics[i].max_val)
            {
                channel_statistics[i].max_val = new_value;
            }
            if (new_value < channel_statistics[i].min_val)
            {
                channel_statistics[i].min_val = new_value;
            }
        }
    }
}

void viz_add_sample_to_history()
{
    data_sample_t *sample = &sample_history[history_write_index];

    sample->timestamp = viz_timestamp;
    sample->valid_mask = 0;

    for (uint8_t i = 0; i < MAX_CHANNELS; i++)
    {
        if (data_channels[i].enabled)
        {
            sample->values[i] = data_channels[i].current_value;
            sample->valid_mask |= (1 << i);
        }
    }

    history_write_index = (history_write_index + 1) % HISTORY_SIZE;
}

void viz_send_realtime_data()
{
    snprintf(viz_tx_buffer, VIZ_BUFFER_SIZE,
             "{\"type\":\"realtime\",\"timestamp\":%lu,\"data\":[",
             viz_timestamp);
    viz_send_string(viz_tx_buffer);

    for (uint8_t i = 0; i < MAX_CHANNELS; i++)
    {
        if (data_channels[i].enabled)
        {
            snprintf(viz_tx_buffer, VIZ_BUFFER_SIZE,
                     "{\"ch\":%d,\"val\":%.3f}%s",
                     i, data_channels[i].current_value,
                     (i < MAX_CHANNELS - 1) ? "," : "");
            viz_send_string(viz_tx_buffer);
        }
    }

    viz_send_string("]}\\n");
}

void viz_send_channel_info()
{
    for (uint8_t i = 0; i < MAX_CHANNELS; i++)
    {
        if (data_channels[i].enabled)
        {
            snprintf(viz_tx_buffer, VIZ_BUFFER_SIZE,
                     "{\"type\":\"channel_info\",\"id\":%d,\"name\":\"%s\",\"unit\":\"%s\",\"min\":%.2f,\"max\":%.2f,\"avg\":%.3f,\"peak\":%.3f,\"samples\":%lu}\\n",
                     i, data_channels[i].name, data_channels[i].unit,
                     data_channels[i].min_value, data_channels[i].max_value,
                     data_channels[i].average, data_channels[i].peak_value,
                     data_channels[i].sample_count);
            viz_send_string(viz_tx_buffer);
        }
    }
}

void viz_send_statistics()
{
    for (uint8_t i = 0; i < MAX_CHANNELS; i++)
    {
        if (data_channels[i].enabled && data_channels[i].sample_count > 0)
        {
            // Calculate standard deviation
            float variance = 0;
            uint16_t count = 0;

            for (uint16_t j = 0; j < HISTORY_SIZE; j++)
            {
                if (sample_history[j].valid_mask & (1 << i))
                {
                    float diff = sample_history[j].values[i] - data_channels[i].average;
                    variance += diff * diff;
                    count++;
                }
            }

            float std_dev = (count > 1) ? simple_sqrt(variance / (count - 1)) : 0;

            snprintf(viz_tx_buffer, VIZ_BUFFER_SIZE,
                     "{\"type\":\"statistics\",\"ch\":%d,\"mean\":%.3f,\"std\":%.3f,\"min\":%.3f,\"max\":%.3f,\"count\":%d}\\n",
                     i, data_channels[i].average, std_dev,
                     channel_statistics[i].min_val, channel_statistics[i].max_val, count);
            viz_send_string(viz_tx_buffer);
        }
    }
}

void viz_send_historical_data(uint16_t samples)
{
    if (samples > HISTORY_SIZE)
        samples = HISTORY_SIZE;

    viz_send_string("{\"type\":\"history\",\"samples\":[");

    uint16_t start_idx = (history_write_index + HISTORY_SIZE - samples) % HISTORY_SIZE;

    for (uint16_t i = 0; i < samples; i++)
    {
        uint16_t idx = (start_idx + i) % HISTORY_SIZE;
        data_sample_t *sample = &sample_history[idx];

        snprintf(viz_tx_buffer, VIZ_BUFFER_SIZE,
                 "{\"t\":%lu,\"d\":[",
                 sample->timestamp);
        viz_send_string(viz_tx_buffer);

        for (uint8_t ch = 0; ch < MAX_CHANNELS; ch++)
        {
            if (sample->valid_mask & (1 << ch))
            {
                snprintf(viz_tx_buffer, VIZ_BUFFER_SIZE,
                         "%.3f%s",
                         sample->values[ch],
                         (ch < MAX_CHANNELS - 1) ? "," : "");
                viz_send_string(viz_tx_buffer);
            }
        }

        snprintf(viz_tx_buffer, VIZ_BUFFER_SIZE,
                 "]}%s",
                 (i < samples - 1) ? "," : "");
        viz_send_string(viz_tx_buffer);
    }

    viz_send_string("]}\\n");
}

// UART receive interrupt for dashboard commands
ISR(USART0_RX_vect)
{
    char received = UDR0;

    if (received == '\\n' || received == '\\r')
    {
        viz_rx_buffer[viz_rx_index] = '\\0';
        viz_command_ready = 1;
        viz_rx_index = 0;
        viz_system.dashboard_connected = 1;
    }
    else if (viz_rx_index < VIZ_BUFFER_SIZE - 1)
    {
        viz_rx_buffer[viz_rx_index++] = received;
    }
}

void viz_process_dashboard_command()
{
    if (!viz_command_ready)
        return;

    if (strcmp(viz_rx_buffer, "START_STREAM") == 0)
    {
        viz_system.streaming_active = 1;
        viz_send_string("{\"response\":\"STREAM_STARTED\"}\\n");
    }
    else if (strcmp(viz_rx_buffer, "STOP_STREAM") == 0)
    {
        viz_system.streaming_active = 0;
        viz_send_string("{\"response\":\"STREAM_STOPPED\"}\\n");
    }
    else if (strcmp(viz_rx_buffer, "GET_CHANNELS") == 0)
    {
        viz_send_channel_info();
    }
    else if (strcmp(viz_rx_buffer, "GET_STATS") == 0)
    {
        viz_send_statistics();
    }
    else if (strncmp(viz_rx_buffer, "GET_HISTORY:", 12) == 0)
    {
        uint16_t samples = atoi(viz_rx_buffer + 12);
        viz_send_historical_data(samples);
    }
    else if (strncmp(viz_rx_buffer, "SET_MODE:", 9) == 0)
    {
        viz_system.visualization_mode = atoi(viz_rx_buffer + 9) % 3;
        viz_send_string("{\"response\":\"MODE_SET\"}\\n");
    }
    else if (strncmp(viz_rx_buffer, "ENABLE_CH:", 10) == 0)
    {
        uint8_t ch = atoi(viz_rx_buffer + 10);
        if (ch < MAX_CHANNELS)
        {
            data_channels[ch].enabled = 1;
            viz_send_string("{\"response\":\"CHANNEL_ENABLED\"}\\n");
        }
    }
    else if (strncmp(viz_rx_buffer, "DISABLE_CH:", 11) == 0)
    {
        uint8_t ch = atoi(viz_rx_buffer + 11);
        if (ch < MAX_CHANNELS)
        {
            data_channels[ch].enabled = 0;
            viz_send_string("{\"response\":\"CHANNEL_DISABLED\"}\\n");
        }
    }
    else if (strcmp(viz_rx_buffer, "RESET_STATS") == 0)
    {
        for (uint8_t i = 0; i < MAX_CHANNELS; i++)
        {
            data_channels[i].sample_count = 0;
            data_channels[i].average = 0;
            data_channels[i].peak_value = 0;
        }
        viz_send_string("{\"response\":\"STATS_RESET\"}\\n");
    }

    viz_command_ready = 0;
}

void viz_display_dashboard()
{
    glcd_clear();
    glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);

    char buffer[25];

    glcd_tiny_draw_string(0, 0, "IoT DATA VISUALIZATION");

    sprintf(buffer, "Stream: %s", viz_system.streaming_active ? "ACTIVE" : "PAUSED");
    glcd_tiny_draw_string(0, 15, buffer);

    sprintf(buffer, "Dashboard: %s", viz_system.dashboard_connected ? "CONNECTED" : "OFFLINE");
    glcd_tiny_draw_string(0, 25, buffer);

    sprintf(buffer, "Samples: %lu", viz_system.samples_transmitted);
    glcd_tiny_draw_string(0, 35, buffer);

    sprintf(buffer, "Mode: %d", viz_system.visualization_mode);
    glcd_tiny_draw_string(0, 45, buffer);

    // Display channel values (first 3 channels)
    for (uint8_t i = 0; i < 3 && i < MAX_CHANNELS; i++)
    {
        if (data_channels[i].enabled)
        {
            sprintf(buffer, "%s: %.1f%s", data_channels[i].name,
                    data_channels[i].current_value, data_channels[i].unit);
            glcd_tiny_draw_string(0, 60 + i * 10, buffer);
        }
    }

    // Real-time data visualization (simple plot)
    for (uint8_t x = 0; x < 128; x++)
    {
        uint16_t hist_idx = (history_write_index + HISTORY_SIZE - 128 + x) % HISTORY_SIZE;

        if (sample_history[hist_idx].valid_mask & 0x01)
        { // Temperature channel
            float normalized = (sample_history[hist_idx].values[0] - data_channels[0].min_value) /
                               (data_channels[0].max_value - data_channels[0].min_value);
            uint8_t y = 127 - (uint8_t)(normalized * 20); // Scale to 20 pixels height

            if (y >= 100 && y <= 120)
            {
                glcd_set_pixel(x, y, 1);
            }
        }
    }

    // Connection timeout check
    static uint32_t last_activity = 0;
    if (viz_timestamp - last_activity > 10000)
    { // 10 second timeout
        viz_system.dashboard_connected = 0;
    }
}

void main_iot_data_visualization()
{
    // Initialize systems
    viz_init_channels();
    viz_init_communication();
    init_GLCD();
    sei(); // Enable global interrupts

    glcd_clear();
    glcd_tiny_set_font(Font5x7, 5, 7, 32, 127);

    uint32_t last_sample_time = 0;
    uint32_t last_stream_time = 0;
    uint32_t last_display_update = 0;

    // Send startup message
    viz_send_string("{\"type\":\"startup\",\"system\":\"IoT_DataViz_v1.0\",\"channels\":");
    snprintf(viz_tx_buffer, VIZ_BUFFER_SIZE, "%d", MAX_CHANNELS);
    viz_send_string(viz_tx_buffer);
    viz_send_string("}\\n");

    while (1)
    {
        viz_timestamp++;

        // Read sensors at sample rate
        if (viz_timestamp - last_sample_time >= (1000 / SAMPLE_RATE_HZ))
        {
            viz_read_sensors();
            viz_add_sample_to_history();
            last_sample_time = viz_timestamp;
        }

        // Process dashboard commands
        viz_process_dashboard_command();

        // Stream data if active
        if (viz_system.streaming_active && viz_timestamp - last_stream_time >= 100)
        {
            viz_send_realtime_data();
            viz_system.samples_transmitted++;
            last_stream_time = viz_timestamp;
        }

        // Update display
        if (viz_timestamp - last_display_update >= 200)
        {
            viz_display_dashboard();
            last_display_update = viz_timestamp;
        }

        // Visual feedback
        PORTA = (uint8_t)(viz_timestamp & 0xFF);
        PORTB = viz_system.streaming_active ? 0xFF : 0x00;

        _delay_ms(1);

        // Exit condition
        if ((PINF & 0x0F) == 0x00)
        {
            break;
        }
    }

    // Send shutdown message
    viz_send_string("{\"type\":\"shutdown\",\"total_samples\":");
    snprintf(viz_tx_buffer, VIZ_BUFFER_SIZE, "%lu", viz_system.samples_transmitted);
    viz_send_string(viz_tx_buffer);
    viz_send_string("}\\n");

    glcd_clear();
    glcd_tiny_draw_string(10, 40, "DATA VISUALIZATION");
    glcd_tiny_draw_string(30, 55, "STOPPED");
    _delay_ms(2000);
}

#endif