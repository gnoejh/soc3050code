/*
 * Port Configuration - Educational Example
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Master ATmega128 port configuration techniques
 * - Learn DDR, PORT, and PIN register functions
 * - Practice bit manipulation and masking operations
 * - Understand pull-up resistors and input/output modes
 *
 * HARDWARE SETUP:
 * - LEDs connected to PORTB (outputs)
 * - Switches/buttons connected to PORTC (inputs with pull-ups)
 * - PORTD configured for mixed input/output demonstration
 * - UART for configuration monitoring and control
 */

#include "config.h"

// Port configuration structures for educational demonstration
typedef struct
{
    volatile uint8_t *ddr_reg;
    volatile uint8_t *port_reg;
    volatile uint8_t *pin_reg;
    char name[10];
    uint8_t current_config;
} port_config_t;

// Port configuration array
port_config_t ports[] = {
    {&DDRB, &PORTB, &PINB, "PORTB", 0},
    {&DDRC, &PORTC, &PINC, "PORTC", 0},
    {&DDRD, &PORTD, &PIND, "PORTD", 0}};

#define NUM_PORTS (sizeof(ports) / sizeof(ports[0]))

// Demonstration patterns
const uint8_t led_patterns[] = {
    0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, // Single LED
    0x03, 0x06, 0x0C, 0x18, 0x30, 0x60, 0xC0, 0x81, // Two adjacent LEDs
    0x0F, 0x1E, 0x3C, 0x78, 0xF0, 0xE1, 0xC3, 0x87, // Four LEDs
    0xFF, 0x00, 0xAA, 0x55, 0xF0, 0x0F, 0xCC, 0x33  // Full patterns
};

#define NUM_PATTERNS (sizeof(led_patterns) / sizeof(led_patterns[0]))

uint8_t current_pattern = 0;
uint8_t pattern_speed = 5; // Pattern change delay (100ms units)
uint8_t auto_demo_mode = 0;

// Function to initialize port configuration system
void init_port_configuration()
{
    puts_USART1("Initializing Port Configuration System...\r\n");

    // Configure PORTB as output for LEDs
    DDRB = 0xFF;  // All pins as outputs
    PORTB = 0x00; // All LEDs off initially
    ports[0].current_config = 0xFF;

    // Configure PORTC as input with pull-ups for switches
    DDRC = 0x00;  // All pins as inputs
    PORTC = 0xFF; // Enable pull-up resistors
    ports[1].current_config = 0x00;

    // Configure PORTD as mixed (first 4 outputs, last 4 inputs)
    DDRD = 0x0F;  // Pins 0-3 as outputs, 4-7 as inputs
    PORTD = 0xF0; // Pull-ups for input pins
    ports[2].current_config = 0x0F;

    puts_USART1("Port Configuration Ready!\r\n");
    puts_USART1("Commands: 'p'=patterns, 'c'=configure, 's'=status\r\n");
    puts_USART1("         'a'=auto demo, '+'=faster, '-'=slower\r\n");
}

// Function to display port status
void display_port_status()
{
    char buffer[100];

    puts_USART1("\r\n=== PORT CONFIGURATION STATUS ===\r\n");

    for (uint8_t i = 0; i < NUM_PORTS; i++)
    {
        sprintf(buffer, "%s: DDR=0x%02X PORT=0x%02X PIN=0x%02X\r\n",
                ports[i].name,
                *ports[i].ddr_reg,
                *ports[i].port_reg,
                *ports[i].pin_reg);
        puts_USART1(buffer);

        // Explain configuration
        sprintf(buffer, "  Config: ");
        puts_USART1(buffer);

        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (*ports[i].ddr_reg & (1 << bit))
            {
                sprintf(buffer, "P%u=OUT ", bit);
            }
            else
            {
                if (*ports[i].port_reg & (1 << bit))
                {
                    sprintf(buffer, "P%u=IN(PU) ", bit); // Input with pull-up
                }
                else
                {
                    sprintf(buffer, "P%u=IN(FL) ", bit); // Input floating
                }
            }
            puts_USART1(buffer);
        }
        puts_USART1("\r\n");
    }
}

// Function to demonstrate LED patterns
void demonstrate_led_patterns()
{
    if (auto_demo_mode)
    {
        static uint16_t pattern_counter = 0;

        if (++pattern_counter >= (pattern_speed * 20))
        { // 20 * 50ms = 1s base
            pattern_counter = 0;
            current_pattern = (current_pattern + 1) % NUM_PATTERNS;

            PORTB = led_patterns[current_pattern];

            char buffer[60];
            sprintf(buffer, "Pattern %u: 0x%02X\r\n",
                    current_pattern, led_patterns[current_pattern]);
            puts_USART1(buffer);
        }
    }
}

// Function to configure port interactively
void configure_port_interactive()
{
    puts_USART1("\r\n=== Interactive Port Configuration ===\r\n");
    puts_USART1("Select port: B, C, or D: ");

    while (!is_USART1_received())
    {
        _delay_ms(10);
    }
    char port_char = get_USART1();
    put_USART1(port_char);
    puts_USART1("\r\n");

    uint8_t port_index;
    switch (port_char)
    {
    case 'B':
    case 'b':
        port_index = 0;
        break;
    case 'C':
    case 'c':
        port_index = 1;
        break;
    case 'D':
    case 'd':
        port_index = 2;
        break;
    default:
        puts_USART1("Invalid port selection\r\n");
        return;
    }

    char buffer[80];
    sprintf(buffer, "Configuring %s\r\n", ports[port_index].name);
    puts_USART1(buffer);

    puts_USART1("Enter DDR value (hex, e.g., FF): ");

    // Simple hex input (2 characters)
    uint8_t ddr_value = 0;
    for (uint8_t i = 0; i < 2; i++)
    {
        while (!is_USART1_received())
        {
            _delay_ms(10);
        }
        char hex_char = get_USART1();
        put_USART1(hex_char);

        ddr_value <<= 4;
        if (hex_char >= '0' && hex_char <= '9')
        {
            ddr_value |= (hex_char - '0');
        }
        else if (hex_char >= 'A' && hex_char <= 'F')
        {
            ddr_value |= (hex_char - 'A' + 10);
        }
        else if (hex_char >= 'a' && hex_char <= 'f')
        {
            ddr_value |= (hex_char - 'a' + 10);
        }
    }
    puts_USART1("\r\n");

    puts_USART1("Enter PORT value (hex, e.g., 00): ");

    uint8_t port_value = 0;
    for (uint8_t i = 0; i < 2; i++)
    {
        while (!is_USART1_received())
        {
            _delay_ms(10);
        }
        char hex_char = get_USART1();
        put_USART1(hex_char);

        port_value <<= 4;
        if (hex_char >= '0' && hex_char <= '9')
        {
            port_value |= (hex_char - '0');
        }
        else if (hex_char >= 'A' && hex_char <= 'F')
        {
            port_value |= (hex_char - 'A' + 10);
        }
        else if (hex_char >= 'a' && hex_char <= 'f')
        {
            port_value |= (hex_char - 'a' + 10);
        }
    }
    puts_USART1("\r\n");

    // Apply configuration
    *ports[port_index].ddr_reg = ddr_value;
    *ports[port_index].port_reg = port_value;
    ports[port_index].current_config = ddr_value;

    sprintf(buffer, "Applied: %s DDR=0x%02X PORT=0x%02X\r\n",
            ports[port_index].name, ddr_value, port_value);
    puts_USART1(buffer);
}

// Function to demonstrate bit manipulation
void demonstrate_bit_manipulation()
{
    puts_USART1("\r\n=== Bit Manipulation Demonstration ===\r\n");

    // Demonstrate setting individual bits
    puts_USART1("Setting bits one by one on PORTB:\r\n");
    PORTB = 0x00;

    for (uint8_t bit = 0; bit < 8; bit++)
    {
        PORTB |= (1 << bit); // Set bit

        char buffer[50];
        sprintf(buffer, "Bit %u set: PORTB = 0x%02X\r\n", bit, PORTB);
        puts_USART1(buffer);
        _delay_ms(300);
    }

    _delay_ms(500);

    // Demonstrate clearing individual bits
    puts_USART1("Clearing bits one by one:\r\n");

    for (uint8_t bit = 0; bit < 8; bit++)
    {
        PORTB &= ~(1 << bit); // Clear bit

        char buffer[50];
        sprintf(buffer, "Bit %u cleared: PORTB = 0x%02X\r\n", bit, PORTB);
        puts_USART1(buffer);
        _delay_ms(300);
    }

    // Demonstrate toggling
    puts_USART1("Toggling all bits:\r\n");

    for (uint8_t i = 0; i < 8; i++)
    {
        PORTB ^= 0xFF; // Toggle all bits

        char buffer[50];
        sprintf(buffer, "Toggle %u: PORTB = 0x%02X\r\n", i, PORTB);
        puts_USART1(buffer);
        _delay_ms(300);
    }

    PORTB = 0x00;
    puts_USART1("Bit manipulation demo complete\r\n");
}

// Function to handle user commands
void handle_port_commands()
{
    if (is_USART1_received())
    {
        char command = get_USART1();

        switch (command)
        {
        case 'p':
        case 'P':
            current_pattern = (current_pattern + 1) % NUM_PATTERNS;
            PORTB = led_patterns[current_pattern];
            char buffer[60];
            sprintf(buffer, "Manual pattern %u: 0x%02X\r\n",
                    current_pattern, led_patterns[current_pattern]);
            puts_USART1(buffer);
            break;

        case 'c':
        case 'C':
            configure_port_interactive();
            break;

        case 's':
        case 'S':
            display_port_status();
            break;

        case 'a':
        case 'A':
            auto_demo_mode = !auto_demo_mode;
            sprintf(buffer, "Auto demo mode: %s\r\n",
                    auto_demo_mode ? "ON" : "OFF");
            puts_USART1(buffer);
            break;

        case '+':
            if (pattern_speed > 1)
            {
                pattern_speed--;
                sprintf(buffer, "Speed increased (delay: %u00ms)\r\n", pattern_speed);
                puts_USART1(buffer);
            }
            break;

        case '-':
            if (pattern_speed < 20)
            {
                pattern_speed++;
                sprintf(buffer, "Speed decreased (delay: %u00ms)\r\n", pattern_speed);
                puts_USART1(buffer);
            }
            break;

        case 'b':
        case 'B':
            demonstrate_bit_manipulation();
            break;

        case 'h':
        case 'H':
        case '?':
            puts_USART1("\r\n=== Port Configuration Help ===\r\n");
            puts_USART1("p/P - Next LED pattern\r\n");
            puts_USART1("c/C - Configure port interactively\r\n");
            puts_USART1("s/S - Show port status\r\n");
            puts_USART1("a/A - Toggle auto demo mode\r\n");
            puts_USART1("b/B - Bit manipulation demo\r\n");
            puts_USART1("+   - Increase pattern speed\r\n");
            puts_USART1("-   - Decrease pattern speed\r\n");
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

    puts_USART1("Port Configuration System Starting...\r\n");
    puts_USART1("Educational port control and bit manipulation demo\r\n");
    puts_USART1("Learn DDR, PORT, and PIN register usage!\r\n");

    // Initialize port configuration
    init_port_configuration();

    // Show initial status
    display_port_status();

    puts_USART1("\r\nPress 'h' for help or 'a' to start auto demo\r\n");

    while (1)
    {
        // Run automatic pattern demonstration
        demonstrate_led_patterns();

        // Handle user commands
        handle_port_commands();

        _delay_ms(50);
    }

    return 0;
}
iguration
        *ATmega128 Educational Framework
            *
                *This project demonstrates port configuration
                    *and GPIO pin control operations.
                        * /

#include "config.h"

    int main(void)
{
    main_port_blinking();

    return 0;
}