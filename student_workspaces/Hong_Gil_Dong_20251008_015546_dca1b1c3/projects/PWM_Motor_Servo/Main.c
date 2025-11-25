/*
 * =============================================================================
 * PWM SERVO MOTOR CONTROL - EDUCATIONAL DEMONSTRATION
 * =============================================================================
 *
 * PROJECT: PWM_Motor_Servo
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Educational demonstration of servo motor control using PWM signals.
 * Students learn precision timing control and servo positioning systems.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master servo motor PWM requirements and timing
 * 2. Learn 16-bit Timer1 configuration for precise pulse generation
 * 3. Practice servo position control (0-180 degrees)
 * 4. Implement multi-servo coordination systems
 * 5. Understand feedback control and positioning accuracy
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - Servo motors connected to PB5 (OC1A), PB6 (OC1B)
 * - External 5-6V power supply for servos (shared ground)
 * - Position feedback potentiometers (optional)
 * - Control interface via UART
 * - Serial connection for debugging (9600 baud)
 *
 * SERVO PWM SPECIFICATIONS:
 * - Frequency: 50 Hz (20ms period)
 * - Pulse width: 1.0ms (0°) to 2.0ms (180°)
 * - Neutral position: 1.5ms (90°)
 * - Dead band: Typically ±5° for standard servos
 *
 * LEARNING PROGRESSION:
 * - Demo 1: Basic Servo Control
 * - Demo 2: Position Feedback
 * - Demo 3: Multi-Servo Coordination
 * - Demo 4: Advanced Control Applications
 *
 * =============================================================================
 */

#include "config.h"

// Servo PWM calculations for 50Hz (20ms period)
// F_CPU = 7372800 Hz, Prescaler = 8
// Timer frequency = 7372800 / 8 = 921600 Hz
// For 20ms period: TOP = 921600 / 50 - 1 = 18431
#define SERVO_FREQ_HZ 50
#define SERVO_PRESCALER 8
#define TIMER_FREQ (F_CPU / SERVO_PRESCALER)
#define SERVO_TOP (TIMER_FREQ / SERVO_FREQ_HZ - 1) // 18431

// Pulse width calculations (in timer ticks)
// 1.0ms = 921.6 ticks, 2.0ms = 1843.2 ticks
#define SERVO_MIN_PULSE (TIMER_FREQ / 1000)                       // 1ms = 921 ticks
#define SERVO_MAX_PULSE (TIMER_FREQ / 500)                        // 2ms = 1843 ticks
#define SERVO_MID_PULSE ((SERVO_MIN_PULSE + SERVO_MAX_PULSE) / 2) // 1.5ms

// Servo channel definitions
typedef enum
{
    SERVO_A = 0, // OC1A (PB5)
    SERVO_B = 1  // OC1B (PB6)
} servo_channel_t;

/*
 * Initialize Timer1 for servo PWM generation
 * Mode 14: Fast PWM with ICR1 as TOP
 * Frequency: 50Hz (20ms period)
 */
void timer1_servo_init(void)
{
    // Set PB5 (OC1A) and PB6 (OC1B) as outputs
    DDRB |= (1 << PB5) | (1 << PB6);

    // Configure Timer1 for Fast PWM, Mode 14
    // WGM13:0 = 1110 (Fast PWM, TOP=ICR1)
    // COM1A1:0 = 10 (Clear OC1A on compare, set at BOTTOM)
    // COM1B1:0 = 10 (Clear OC1B on compare, set at BOTTOM)
    // CS12:0 = 010 (Prescaler = 8)
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);

    // Set TOP for 50Hz frequency
    ICR1 = SERVO_TOP;

    // Initialize both servos to neutral position (90°)
    OCR1A = SERVO_MID_PULSE;
    OCR1B = SERVO_MID_PULSE;
}

/*
 * Set servo position by angle (0-180 degrees)
 */
void servo_set_angle(servo_channel_t channel, uint8_t angle)
{
    // Constrain angle to valid range
    if (angle > 180)
        angle = 180;

    // Convert angle to pulse width
    // Linear interpolation: pulse = MIN + (angle/180) * (MAX - MIN)
    uint16_t pulse = SERVO_MIN_PULSE +
                     ((uint32_t)angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / 180;

    // Set appropriate channel
    if (channel == SERVO_A)
    {
        OCR1A = pulse;
    }
    else
    {
        OCR1B = pulse;
    }
}

/*
 * Set servo position by pulse width (microseconds)
 * Useful for fine-tuning and calibration
 */
void servo_set_pulse_us(servo_channel_t channel, uint16_t pulse_us)
{
    // Constrain to reasonable range (500-2500 us)
    if (pulse_us < 500)
        pulse_us = 500;
    if (pulse_us > 2500)
        pulse_us = 2500;

    // Convert microseconds to timer ticks
    // pulse_ticks = (pulse_us * TIMER_FREQ) / 1000000
    uint16_t pulse_ticks = ((uint32_t)pulse_us * TIMER_FREQ) / 1000000UL;

    if (channel == SERVO_A)
    {
        OCR1A = pulse_ticks;
    }
    else
    {
        OCR1B = pulse_ticks;
    }
}

/*
 * Smooth servo movement from current to target position
 */
void servo_move_smooth(servo_channel_t channel, uint8_t target_angle, uint16_t duration_ms)
{
    // Get current position (approximate from OCR value)
    uint16_t current_pulse = (channel == SERVO_A) ? OCR1A : OCR1B;
    uint8_t current_angle = ((uint32_t)(current_pulse - SERVO_MIN_PULSE) * 180) /
                            (SERVO_MAX_PULSE - SERVO_MIN_PULSE);

    // Calculate step size
    int16_t angle_diff = target_angle - current_angle;
    uint8_t num_steps = duration_ms / 20; // 20ms per step (50Hz update)
    if (num_steps == 0)
        num_steps = 1;

    float angle_step = (float)angle_diff / num_steps;

    // Move gradually
    for (uint8_t i = 0; i < num_steps; i++)
    {
        float intermediate_angle = current_angle + (angle_step * i);
        servo_set_angle(channel, (uint8_t)intermediate_angle);
        _delay_ms(20);
    }

    // Ensure final position
    servo_set_angle(channel, target_angle);
}

/* ========================================================================
 * DEMO 1: Basic Servo Positioning with UART Control
 * ======================================================================== */
void demo1_basic_positioning(void)
{
    puts_USART1("\r\n=== DEMO 1: Basic Servo Positioning ===\r\n");
    puts_USART1("Commands:\r\n");
    puts_USART1("  a[angle]: Set Servo A (e.g., 'a90' for 90°)\r\n");
    puts_USART1("  b[angle]: Set Servo B (e.g., 'b180' for 180°)\r\n");
    puts_USART1("  0-9: Quick angles (0=0°, 5=90°, 9=180°)\r\n");
    puts_USART1("  q: Return to menu\r\n\r\n");

    char input_buffer[10];
    uint8_t buf_index = 0;

    while (1)
    {
        if (UCSR1A & (1 << RXC1))
        {
            char c = UDR1;
            putch_USART1(c); // Echo

            if (c == '\r' || c == '\n')
            {
                input_buffer[buf_index] = '\0';
                puts_USART1("\r\n");

                // Process command
                if (buf_index > 0)
                {
                    if (input_buffer[0] == 'a' || input_buffer[0] == 'A')
                    {
                        uint8_t angle = atoi(&input_buffer[1]);
                        servo_set_angle(SERVO_A, angle);
                        char msg[50];
                        sprintf(msg, "Servo A → %d°\r\n", angle);
                        puts_USART1(msg);
                    }
                    else if (input_buffer[0] == 'b' || input_buffer[0] == 'B')
                    {
                        uint8_t angle = atoi(&input_buffer[1]);
                        servo_set_angle(SERVO_B, angle);
                        char msg[50];
                        sprintf(msg, "Servo B → %d°\r\n", angle);
                        puts_USART1(msg);
                    }
                    else if (input_buffer[0] >= '0' && input_buffer[0] <= '9')
                    {
                        uint8_t angle = (input_buffer[0] - '0') * 20; // 0→0°, 5→100°, 9→180°
                        servo_set_angle(SERVO_A, angle);
                        servo_set_angle(SERVO_B, angle);
                        char msg[50];
                        sprintf(msg, "Both servos → %d°\r\n", angle);
                        puts_USART1(msg);
                    }
                    else if (input_buffer[0] == 'q' || input_buffer[0] == 'Q')
                    {
                        return;
                    }
                }
                buf_index = 0;
            }
            else if (c == 8 || c == 127)
            { // Backspace
                if (buf_index > 0)
                {
                    buf_index--;
                    puts_USART1(" \b");
                }
            }
            else if (buf_index < sizeof(input_buffer) - 1)
            {
                input_buffer[buf_index++] = c;
            }
        }
    }
}

/* ========================================================================
 * DEMO 2: Servo Sweep Test
 * ======================================================================== */
void demo2_sweep_test(void)
{
    puts_USART1("\r\n=== DEMO 2: Servo Sweep Test ===\r\n");
    puts_USART1("Sweeping servos across full range\r\n");
    puts_USART1("Press any key to stop and return to menu\r\n\r\n");

    while (1)
    {
        // Sweep forward (0° to 180°)
        puts_USART1("Sweeping forward (0° → 180°)...\r\n");
        for (uint8_t angle = 0; angle <= 180; angle += 5)
        {
            servo_set_angle(SERVO_A, angle);
            servo_set_angle(SERVO_B, 180 - angle); // Mirror movement

            char buf[40];
            sprintf(buf, "  A: %3d°  B: %3d°\r\n", angle, 180 - angle);
            puts_USART1(buf);

            _delay_ms(100);

            if (UCSR1A & (1 << RXC1))
            {
                getch_USART1();
                return;
            }
        }

        _delay_ms(500);

        // Sweep backward (180° to 0°)
        puts_USART1("Sweeping backward (180° → 0°)...\r\n");
        for (int16_t angle = 180; angle >= 0; angle -= 5)
        {
            servo_set_angle(SERVO_A, angle);
            servo_set_angle(SERVO_B, 180 - angle); // Mirror movement

            _delay_ms(100);

            if (UCSR1A & (1 << RXC1))
            {
                getch_USART1();
                return;
            }
        }

        _delay_ms(500);
    }
}

/* ========================================================================
 * DEMO 3: Smooth Movement with Acceleration
 * ======================================================================== */
void demo3_smooth_movement(void)
{
    puts_USART1("\r\n=== DEMO 3: Smooth Servo Movement ===\r\n");
    puts_USART1("Demonstrating smooth acceleration/deceleration\r\n");
    puts_USART1("Press any key to stop and return to menu\r\n\r\n");

    uint8_t positions[] = {0, 45, 90, 135, 180, 135, 90, 45};
    uint8_t num_positions = sizeof(positions) / sizeof(positions[0]);

    while (1)
    {
        for (uint8_t i = 0; i < num_positions; i++)
        {
            char buf[50];
            sprintf(buf, "Moving to %d° (smooth)...\r\n", positions[i]);
            puts_USART1(buf);

            servo_move_smooth(SERVO_A, positions[i], 1000); // 1 second transition
            servo_move_smooth(SERVO_B, 180 - positions[i], 1000);

            _delay_ms(500);

            if (UCSR1A & (1 << RXC1))
            {
                getch_USART1();
                return;
            }
        }
    }
}

/* ========================================================================
 * DEMO 4: ADC Joystick Control
 * ======================================================================== */
void demo4_joystick_control(void)
{
    puts_USART1("\r\n=== DEMO 4: Joystick Servo Control ===\r\n");
    puts_USART1("ADC0 controls Servo A, ADC1 controls Servo B\r\n");
    puts_USART1("Joystick X/Y axes map to servo angles\r\n");
    puts_USART1("Press 'q' to return to menu\r\n\r\n");

    Adc_init();

    while (1)
    {
        // Read joystick positions
        uint16_t adc_x = Read_Adc_Data(0);
        uint16_t adc_y = Read_Adc_Data(1);

        // Convert ADC values (0-1023) to servo angles (0-180°)
        uint8_t angle_a = (uint32_t)adc_x * 180 / 1023;
        uint8_t angle_b = (uint32_t)adc_y * 180 / 1023;

        // Set servo positions
        servo_set_angle(SERVO_A, angle_a);
        servo_set_angle(SERVO_B, angle_b);

        // Display status every ~500ms
        static uint8_t display_counter = 0;
        if (++display_counter >= 10)
        {
            char buf[70];
            sprintf(buf, "ADC: X=%4u Y=%4u  |  Servos: A=%3d° B=%3d°\r\n",
                    adc_x, adc_y, angle_a, angle_b);
            puts_USART1(buf);
            display_counter = 0;
        }

        // Check for quit command
        if (UCSR1A & (1 << RXC1))
        {
            char cmd = UDR1;
            if (cmd == 'q' || cmd == 'Q')
            {
                return;
            }
        }

        _delay_ms(50);
    }
}

/* ========================================================================
 * Main Menu System
 * ======================================================================== */
void display_main_menu(void)
{
    puts_USART1("\r\n\r\n");
    puts_USART1("╔════════════════════════════════════════╗\r\n");
    puts_USART1("║   SERVO MOTOR CONTROL - ATmega128     ║\r\n");
    puts_USART1("╚════════════════════════════════════════╝\r\n");
    puts_USART1("\r\n");
    puts_USART1("Select Demo:\r\n");
    puts_USART1("  [1] Basic Positioning (UART Commands)\r\n");
    puts_USART1("  [2] Automatic Sweep Test\r\n");
    puts_USART1("  [3] Smooth Movement Demo\r\n");
    puts_USART1("  [4] Joystick Control (ADC)\r\n");
    puts_USART1("\r\n");
    puts_USART1("Enter selection (1-4): ");
}

int main(void)
{
    // Initialize peripherals
    // init_devices();  // Manual initialization below
    Uart1_init();
    timer1_servo_init();

    // Configure status LEDs
    DDRC = 0xFF;
    PORTC = 0x00;

    // Send startup message
    _delay_ms(500);
    puts_USART1("\r\n\r\n*** Servo Motor Control System ***\r\n");
    puts_USART1("ATmega128 Dual Servo Controller\r\n");
    char buf[80];
    sprintf(buf, "PWM: %dHz, TOP=%u, Pulse: %u-%u ticks\r\n",
            SERVO_FREQ_HZ, SERVO_TOP, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    puts_USART1(buf);

    // Initialize servos to center position
    servo_set_angle(SERVO_A, 90);
    servo_set_angle(SERVO_B, 90);
    puts_USART1("Servos initialized to 90° (neutral)\r\n");

    while (1)
    {
        display_main_menu();

        // Wait for user selection
        char choice = getch_USART1();
        putch_USART1(choice);
        puts_USART1("\r\n");

        switch (choice)
        {
        case '1':
            demo1_basic_positioning();
            break;
        case '2':
            demo2_sweep_test();
            break;
        case '3':
            demo3_smooth_movement();
            break;
        case '4':
            demo4_joystick_control();
            break;
        default:
            puts_USART1("Invalid selection!\r\n");
            _delay_ms(1000);
            break;
        }

        // Return servos to neutral between demos
        servo_set_angle(SERVO_A, 90);
        servo_set_angle(SERVO_B, 90);
        _delay_ms(500);
    }

    return 0;
}
