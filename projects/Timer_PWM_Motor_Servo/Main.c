/*
 * ==============================================================================
 * PWM SERVO MOTOR - DEMO CODE (REFINED)
 * ==============================================================================
 * PROJECT: PWM_Motor_Servo
 * See Slide.md for complete theory and technical details
 *
 * REFACTORED: Now uses _pwm.h library for professional servo control
 * LEARNING: Compare this refined version with manual Timer1 configuration
 * DEMOS: Servo position control, PWM timing, angle positioning
 * ==============================================================================
 */

#include "config.h"

// Servo channel mapping (using PWM library channels)
#define SERVO_A PWM_CH_1A // OC1A (PB5)
#define SERVO_B PWM_CH_1B // OC1B (PB6)

// Keep constants for reference and debugging output
#define SERVO_FREQ_HZ 50
#define TIMER_FREQ (F_CPU / 8) // PWM library uses prescaler 8 for servos
#define SERVO_TOP (TIMER_FREQ / SERVO_FREQ_HZ - 1)
#define SERVO_MIN_PULSE 544  // 0° in microseconds
#define SERVO_MAX_PULSE 2400 // 180° in microseconds
#define SERVO_MID_PULSE 1472 // 90° in microseconds

/*
 * Initialize servos using PWM library
 * EDUCATIONAL NOTE: Compare this simple initialization with the manual
 * Timer1 configuration (TCCR1A, TCCR1B, ICR1, OCR1A/B) that was needed before
 */
void timer1_servo_init(void)
{
    // Initialize both servo channels (library handles all Timer1 configuration)
    PWM_servo_init(SERVO_A);
    PWM_servo_init(SERVO_B);

    // Set both servos to neutral position (90°)
    PWM_servo_set_angle(SERVO_A, 90);
    PWM_servo_set_angle(SERVO_B, 90);
}

/*
 * Set servo position by angle (0-180 degrees)
 * EDUCATIONAL NOTE: Library handles angle-to-pulse conversion automatically
 */
void servo_set_angle(pwm_channel_t channel, uint8_t angle)
{
    PWM_servo_set_angle(channel, angle);
}

/*
 * Set servo position using microsecond pulse width
 * EDUCATIONAL NOTE: Useful for fine-tuning and calibration
 */
void servo_set_pulse_us(pwm_channel_t channel, uint16_t pulse_us)
{
    PWM_servo_set_pulse_us(channel, pulse_us);
}

/*
 * Smooth servo movement from current to target position
 * EDUCATIONAL NOTE: Library function handles all the stepping logic
 * Old version: Manual OCR reading, angle calculation, step loop
 * New version: Single library call with professional ramping
 */
void servo_move_smooth(pwm_channel_t channel, uint8_t target_angle, uint16_t duration_ms)
{
    // Use library's sweep function (goes from current angle to target)
    // Note: We approximate current as 90° since library tracks position internally
    // For precise tracking, we'd need to add PWM_servo_get_angle() to the library
    PWM_servo_sweep(channel, 90, target_angle, duration_ms / 20);
}

/* ========================================================================
 * DEMO 1: Basic Servo Positioning with UART Control
 * ======================================================================== */
void demo1_basic_positioning(void)
{
    puts_USART1("\r\n=== DEMO 1: Basic Servo Positioning ===\r\n");
    puts_USART1("Commands:\r\n");
    puts_USART1("  a[angle]: Set Servo A (e.g., 'a90' for 90째)\r\n");
    puts_USART1("  b[angle]: Set Servo B (e.g., 'b180' for 180째)\r\n");
    puts_USART1("  0-9: Quick angles (0=0째, 5=90째, 9=180째)\r\n");
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
                        sprintf(msg, "Servo A ??%d째\r\n", angle);
                        puts_USART1(msg);
                    }
                    else if (input_buffer[0] == 'b' || input_buffer[0] == 'B')
                    {
                        uint8_t angle = atoi(&input_buffer[1]);
                        servo_set_angle(SERVO_B, angle);
                        char msg[50];
                        sprintf(msg, "Servo B ??%d째\r\n", angle);
                        puts_USART1(msg);
                    }
                    else if (input_buffer[0] >= '0' && input_buffer[0] <= '9')
                    {
                        uint8_t angle = (input_buffer[0] - '0') * 20; // 0??째, 5??00째, 9??80째
                        servo_set_angle(SERVO_A, angle);
                        servo_set_angle(SERVO_B, angle);
                        char msg[50];
                        sprintf(msg, "Both servos ??%d째\r\n", angle);
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
        // Sweep forward (0째 to 180째)
        puts_USART1("Sweeping forward (0째 ??180째)...\r\n");
        for (uint8_t angle = 0; angle <= 180; angle += 5)
        {
            servo_set_angle(SERVO_A, angle);
            servo_set_angle(SERVO_B, 180 - angle); // Mirror movement

            char buf[40];
            sprintf(buf, "  A: %3d째  B: %3d째\r\n", angle, 180 - angle);
            puts_USART1(buf);

            _delay_ms(100);

            if (UCSR1A & (1 << RXC1))
            {
                getch_USART1();
                return;
            }
        }

        _delay_ms(500);

        // Sweep backward (180째 to 0째)
        puts_USART1("Sweeping backward (180째 ??0째)...\r\n");
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
            sprintf(buf, "Moving to %d째 (smooth)...\r\n", positions[i]);
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

        // Convert ADC values (0-1023) to servo angles (0-180째)
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
            sprintf(buf, "ADC: X=%4u Y=%4u  |  Servos: A=%3d째 B=%3d째\r\n",
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
    puts_USART1("?붴븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븮\r\n");
    puts_USART1("??  SERVO MOTOR CONTROL - ATmega128     ??r\n");
    puts_USART1("?싢븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븧?먥븴\r\n");
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
    puts_USART1("Servos initialized to 90째 (neutral)\r\n");

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
