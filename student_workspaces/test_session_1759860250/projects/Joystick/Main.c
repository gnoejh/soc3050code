/*
 * =============================================================================
 * JOYSTICK CONTROL INTERFACE - EDUCATIONAL DEMONSTRATION
 * =============================================================================
 *
 * PROJECT: Joystick
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Educational demonstration of analog joystick interfacing and coordinate mapping.
 * Students learn multi-channel ADC usage and analog-to-digital control systems.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master analog joystick interface and multi-channel ADC
 * 2. Learn coordinate system mapping and calibration
 * 3. Practice threshold-based digital control from analog input
 * 4. Implement directional control and position monitoring
 * 5. Understand analog signal processing fundamentals
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - Analog joystick with X-axis (ADC0) and Y-axis (ADC1)
 * - Optional joystick button connected to digital input
 * - LEDs on PORTB for direction indication (Up, Down, Left, Right)
 * - LCD display for position visualization
 * - Serial connection for calibration (9600 baud)
 *
 * LEARNING PROGRESSION:
 * - Demo 1: Basic Joystick Reading
 * - Demo 2: Coordinate Mapping and Calibration
 * - Demo 3: Digital Direction Control
 * - Demo 4: Advanced Control Applications
 *
 * =============================================================================
 */

#include "config.h"

// Joystick configuration
#define JOYSTICK_X_CHANNEL 0
#define JOYSTICK_Y_CHANNEL 1
#define JOYSTICK_BUTTON_PIN PINC0
#define JOYSTICK_BUTTON_PORT PINC

// Direction LEDs on PORTB
#define LED_UP (1 << 0)
#define LED_DOWN (1 << 1)
#define LED_LEFT (1 << 2)
#define LED_RIGHT (1 << 3)
#define LED_CENTER (1 << 4)

// Joystick calibration values
typedef struct
{
    uint16_t x_center;
    uint16_t y_center;
    uint16_t x_min;
    uint16_t x_max;
    uint16_t y_min;
    uint16_t y_max;
    uint16_t deadzone;
} joystick_calibration_t;

// Joystick position structure
typedef struct
{
    uint16_t x_raw;
    uint16_t y_raw;
    int16_t x_scaled;  // -100 to +100
    int16_t y_scaled;  // -100 to +100
    uint8_t direction; // Bit mask for directions
    uint8_t button_pressed;
} joystick_position_t;

// Global variables
joystick_calibration_t joystick_cal;
joystick_position_t joystick_pos;
uint8_t calibration_mode = 0;

// Function to initialize joystick system
void init_joystick_control()
{
    puts_USART1("Initializing Joystick Control System...\r\n");

    // Initialize ADC for joystick reading
    ADC_init();

    // Configure direction LEDs
    DDRB = 0xFF;  // PORTB as output
    PORTB = 0x00; // All LEDs off initially

    // Configure button input (with pull-up)
    DDRC &= ~(1 << JOYSTICK_BUTTON_PIN);
    PORTC |= (1 << JOYSTICK_BUTTON_PIN);

    // Set default calibration values
    joystick_cal.x_center = 512;
    joystick_cal.y_center = 512;
    joystick_cal.x_min = 0;
    joystick_cal.x_max = 1023;
    joystick_cal.y_min = 0;
    joystick_cal.y_max = 1023;
    joystick_cal.deadzone = 50;

    puts_USART1("Joystick Control Ready!\r\n");
    puts_USART1("Commands: 'c'=calibrate, 'r'=raw values, 's'=scaled values\r\n");
    puts_USART1("         'd'=demo mode, 'h'=help\r\n");
}

// Function to read joystick position
void read_joystick_position()
{
    // Read X and Y channels
    joystick_pos.x_raw = ADC_read(JOYSTICK_X_CHANNEL);
    joystick_pos.y_raw = ADC_read(JOYSTICK_Y_CHANNEL);

    // Read button state (active low)
    joystick_pos.button_pressed = !(JOYSTICK_BUTTON_PORT & (1 << JOYSTICK_BUTTON_PIN));

    // Scale to -100 to +100 range
    int32_t x_temp = (int32_t)joystick_pos.x_raw - joystick_cal.x_center;
    int32_t y_temp = (int32_t)joystick_pos.y_raw - joystick_cal.y_center;

    // Apply scaling
    if (x_temp > 0)
    {
        joystick_pos.x_scaled = (x_temp * 100) / (joystick_cal.x_max - joystick_cal.x_center);
    }
    else
    {
        joystick_pos.x_scaled = (x_temp * 100) / (joystick_cal.x_center - joystick_cal.x_min);
    }

    if (y_temp > 0)
    {
        joystick_pos.y_scaled = (y_temp * 100) / (joystick_cal.y_max - joystick_cal.y_center);
    }
    else
    {
        joystick_pos.y_scaled = (y_temp * 100) / (joystick_cal.y_center - joystick_cal.y_min);
    }

    // Clamp values
    if (joystick_pos.x_scaled > 100)
        joystick_pos.x_scaled = 100;
    if (joystick_pos.x_scaled < -100)
        joystick_pos.x_scaled = -100;
    if (joystick_pos.y_scaled > 100)
        joystick_pos.y_scaled = 100;
    if (joystick_pos.y_scaled < -100)
        joystick_pos.y_scaled = -100;

    // Determine direction with deadzone
    joystick_pos.direction = 0;

    if (abs(joystick_pos.x_scaled) > joystick_cal.deadzone ||
        abs(joystick_pos.y_scaled) > joystick_cal.deadzone)
    {

        if (joystick_pos.y_scaled > joystick_cal.deadzone)
        {
            joystick_pos.direction |= LED_UP;
        }
        if (joystick_pos.y_scaled < -joystick_cal.deadzone)
        {
            joystick_pos.direction |= LED_DOWN;
        }
        if (joystick_pos.x_scaled > joystick_cal.deadzone)
        {
            joystick_pos.direction |= LED_RIGHT;
        }
        if (joystick_pos.x_scaled < -joystick_cal.deadzone)
        {
            joystick_pos.direction |= LED_LEFT;
        }
    }
    else
    {
        joystick_pos.direction = LED_CENTER;
    }
}

// Function to update LED indicators
void update_direction_leds()
{
    PORTB = joystick_pos.direction;

    // Add button indicator
    if (joystick_pos.button_pressed)
    {
        PORTB |= (1 << 7); // Light up LED 7 for button press
    }
}

// Function to demonstrate joystick control
void demonstrate_joystick()
{
    puts_USART1("\r\n=== Joystick Demonstration ===\r\n");
    puts_USART1("Move joystick to see LED response\r\n");
    puts_USART1("Press joystick button to activate center LED\r\n");
    puts_USART1("Press any key to exit demo...\r\n");

    while (!is_USART1_received())
    {
        read_joystick_position();
        update_direction_leds();

        // Display position every 500ms
        static uint16_t display_counter = 0;
        if (++display_counter >= 10)
        {
            display_counter = 0;
            char buffer[80];
            sprintf(buffer, "X=%4d Y=%4d Dir=0x%02X Btn=%u\r\n",
                    joystick_pos.x_scaled, joystick_pos.y_scaled,
                    joystick_pos.direction, joystick_pos.button_pressed);
            puts_USART1(buffer);
        }

        _delay_ms(50);
    }

    get_USART1(); // Clear received character
    PORTB = 0x00; // Turn off all LEDs
    puts_USART1("Demo complete\r\n");
}

// Function to handle user commands
void handle_joystick_commands()
{
    if (is_USART1_received())
    {
        char command = get_USART1();

        switch (command)
        {
        case 'r':
        case 'R':
            char buffer[80];
            sprintf(buffer, "Raw: X=%4u Y=%4u Btn=%u\r\n",
                    joystick_pos.x_raw, joystick_pos.y_raw, joystick_pos.button_pressed);
            puts_USART1(buffer);
            break;

        case 's':
        case 'S':
            sprintf(buffer, "Scaled: X=%4d Y=%4d Dir=0x%02X Btn=%u\r\n",
                    joystick_pos.x_scaled, joystick_pos.y_scaled,
                    joystick_pos.direction, joystick_pos.button_pressed);
            puts_USART1(buffer);
            break;

        case 'd':
        case 'D':
            demonstrate_joystick();
            break;

        case 'h':
        case 'H':
        case '?':
            puts_USART1("\r\n=== Joystick Control Help ===\r\n");
            puts_USART1("r/R - Show raw ADC values\r\n");
            puts_USART1("s/S - Show scaled values\r\n");
            puts_USART1("d/D - Run demonstration\r\n");
            puts_USART1("h/? - Show this help\r\n");
            break;

        default:
            puts_USART1("Unknown command. Press 'h' for help.\r\n");
            break;
        }
    }
}

int main(void)
{
    // Initialize system components
    init_devices();
    Uart1_init();
    ADC_init();

    puts_USART1("Joystick Control System Starting...\r\n");
    puts_USART1("Educational analog joystick interface demo\r\n");
    puts_USART1("Learn ADC usage and coordinate mapping!\r\n");

    // Initialize joystick control
    init_joystick_control();

    puts_USART1("\r\nPress 'h' for help or 'd' for demo\r\n");

    while (1)
    {
        // Continuously read joystick
        read_joystick_position();
        update_direction_leds();

        // Handle user commands
        handle_joystick_commands();

        _delay_ms(50);
    }

    return 0;
}