/*
 * Motor Control - Educational Example
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Learn stepper motor control principles
 * - Understand full-step and half-step sequencing
 * - Practice timing control for motor speed
 * - Master digital output patterns for motor drive
 *
 * HARDWARE SETUP:
 * - Stepper motor connected to PORTB (pins 0-3)
 * - Motor driver IC (ULN2003 or similar)
 * - Optional: Motor speed control via potentiometer on ADC
 * - LEDs on PORTC for step sequence visualization
 */

#include "config.h"

// Stepper motor configuration
#define MOTOR_PORT PORTB
#define MOTOR_DDR DDRB
#define MOTOR_PINS 0x0F // Use pins 0-3

// Step sequences for different modes
const uint8_t full_step_sequence[4] = {
    0x01, // Step 1: Coil A
    0x02, // Step 2: Coil B
    0x04, // Step 3: Coil C
    0x08  // Step 4: Coil D
};

const uint8_t half_step_sequence[8] = {
    0x01, // Step 1: A
    0x03, // Step 2: A+B
    0x02, // Step 3: B
    0x06, // Step 4: B+C
    0x04, // Step 5: C
    0x0C, // Step 6: C+D
    0x08, // Step 7: D
    0x09  // Step 8: D+A
};

// Motor control variables
uint8_t current_step = 0;
uint8_t motor_direction = 1; // 1 = clockwise, 0 = counterclockwise
uint16_t step_delay = 10;    // Delay between steps (ms)
uint8_t motor_mode = 0;      // 0 = full step, 1 = half step
uint16_t total_steps = 0;

// Function to initialize motor control
void init_motor_control()
{
    puts_USART1("Initializing Motor Control System...\r\n");

    // Configure motor pins as outputs
    MOTOR_DDR |= MOTOR_PINS;
    MOTOR_PORT &= ~MOTOR_PINS; // Start with all coils off

    // Configure LED visualization pins
    DDRC = 0xFF;  // PORTC as output for step visualization
    PORTC = 0x00; // All LEDs off initially

    puts_USART1("Motor Control Ready!\r\n");
    puts_USART1("Commands: 'f'=forward, 'b'=backward, 's'=stop\r\n");
    puts_USART1("         'm'=mode, '+'=faster, '-'=slower\r\n");
}

// Function to execute one motor step
void motor_step(uint8_t direction)
{
    uint8_t step_pattern;

    if (motor_mode == 0)
    {
        // Full step mode
        if (direction)
        {
            current_step = (current_step + 1) % 4;
        }
        else
        {
            current_step = (current_step + 3) % 4; // Equivalent to -1 mod 4
        }
        step_pattern = full_step_sequence[current_step];
    }
    else
    {
        // Half step mode
        if (direction)
        {
            current_step = (current_step + 1) % 8;
        }
        else
        {
            current_step = (current_step + 7) % 8; // Equivalent to -1 mod 8
        }
        step_pattern = half_step_sequence[current_step];
    }

    // Apply step pattern to motor
    MOTOR_PORT = (MOTOR_PORT & ~MOTOR_PINS) | (step_pattern & MOTOR_PINS);

    // Visualize on LEDs
    PORTC = step_pattern;

    total_steps++;

    // Step timing
    _delay_ms(step_delay);
}

// Function to move motor a specific number of steps
void motor_move_steps(uint16_t steps, uint8_t direction)
{
    char buffer[50];
    sprintf(buffer, "Moving %u steps %s...\r\n",
            steps, direction ? "clockwise" : "counterclockwise");
    puts_USART1(buffer);

    for (uint16_t i = 0; i < steps; i++)
    {
        motor_step(direction);

        // Check for UART commands during movement
        if (is_USART1_received())
        {
            char cmd = get_USART1();
            if (cmd == 's' || cmd == 'S')
            {
                puts_USART1("\r\nMovement stopped by user\r\n");
                break;
            }
        }
    }

    puts_USART1("Movement complete\r\n");
}

// Function to stop motor (turn off all coils)
void motor_stop()
{
    MOTOR_PORT &= ~MOTOR_PINS;
    PORTC = 0x00;
    puts_USART1("Motor stopped\r\n");
}

// Function to demonstrate motor patterns
void demonstrate_motor_patterns()
{
    puts_USART1("\r\n=== Motor Pattern Demonstration ===\r\n");

    // Full step pattern
    puts_USART1("Full Step Pattern:\r\n");
    motor_mode = 0;
    motor_move_steps(16, 1); // One full rotation (assuming 16 steps/rotation)
    _delay_ms(1000);

    // Half step pattern
    puts_USART1("Half Step Pattern:\r\n");
    motor_mode = 1;
    motor_move_steps(32, 1); // One full rotation in half steps
    _delay_ms(1000);

    // Bidirectional movement
    puts_USART1("Bidirectional Movement:\r\n");
    motor_move_steps(16, 1); // Forward
    _delay_ms(500);
    motor_move_steps(16, 0); // Backward

    motor_stop();
    puts_USART1("Demonstration complete\r\n");
}

// Function to handle user commands
void handle_motor_commands()
{
    if (is_USART1_received())
    {
        char command = get_USART1();
        char buffer[60];

        switch (command)
        {
        case 'f':
        case 'F':
            puts_USART1("Moving forward...\r\n");
            motor_move_steps(8, 1);
            break;

        case 'b':
        case 'B':
            puts_USART1("Moving backward...\r\n");
            motor_move_steps(8, 0);
            break;

        case 's':
        case 'S':
            motor_stop();
            break;

        case 'm':
        case 'M':
            motor_mode = !motor_mode;
            sprintf(buffer, "Motor mode: %s\r\n",
                    motor_mode ? "Half Step" : "Full Step");
            puts_USART1(buffer);
            break;

        case '+':
            if (step_delay > 1)
            {
                step_delay--;
                sprintf(buffer, "Speed increased (delay: %ums)\r\n", step_delay);
                puts_USART1(buffer);
            }
            break;

        case '-':
            if (step_delay < 100)
            {
                step_delay++;
                sprintf(buffer, "Speed decreased (delay: %ums)\r\n", step_delay);
                puts_USART1(buffer);
            }
            break;

        case 'd':
        case 'D':
            demonstrate_motor_patterns();
            break;

        case 'i':
        case 'I':
            sprintf(buffer, "Motor Status:\r\n");
            puts_USART1(buffer);
            sprintf(buffer, "Mode: %s\r\n", motor_mode ? "Half Step" : "Full Step");
            puts_USART1(buffer);
            sprintf(buffer, "Speed: %ums delay\r\n", step_delay);
            puts_USART1(buffer);
            sprintf(buffer, "Total steps: %u\r\n", total_steps);
            puts_USART1(buffer);
            sprintf(buffer, "Current position: %u\r\n", current_step);
            puts_USART1(buffer);
            break;

        case 'h':
        case 'H':
        case '?':
            puts_USART1("\r\n=== Motor Control Help ===\r\n");
            puts_USART1("f/F - Move forward (8 steps)\r\n");
            puts_USART1("b/B - Move backward (8 steps)\r\n");
            puts_USART1("s/S - Stop motor\r\n");
            puts_USART1("m/M - Toggle step mode (full/half)\r\n");
            puts_USART1("+   - Increase speed\r\n");
            puts_USART1("-   - Decrease speed\r\n");
            puts_USART1("d/D - Run demonstration\r\n");
            puts_USART1("i/I - Show motor status\r\n");
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

    puts_USART1("Motor Control System Starting...\r\n");
    puts_USART1("Educational stepper motor control demo\r\n");
    puts_USART1("Learn motor sequencing and timing control!\r\n");

    // Initialize motor control
    init_motor_control();

    // Run initial demonstration
    puts_USART1("\r\nRunning initial demonstration...\r\n");
    demonstrate_motor_patterns();

    puts_USART1("\r\nEntering interactive mode. Press 'h' for help.\r\n");

    while (1)
    {
        // Handle user commands
        handle_motor_commands();

        _delay_ms(50);
    }

    return 0;
}