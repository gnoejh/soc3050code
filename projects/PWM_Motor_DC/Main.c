/*
 * =============================================================================
 * PWM DC MOTOR CONTROL - EDUCATIONAL DEMONSTRATION
 * =============================================================================
 *
 * PROJECT: PWM_Motor_DC
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Educational demonstration of PWM-based DC motor control systems.
 * Students learn motor control concepts and power electronics interfacing.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master PWM signal generation for motor control
 * 2. Learn H-bridge driver interfacing
 * 3. Practice speed and direction control algorithms
 * 4. Understand motor dynamics and feedback
 * 5. Implement closed-loop control systems
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - DC motor with H-bridge driver circuit
 * - PWM output on Timer1 (OC1A/OC1B)
 * - Potentiometer for speed control input
 * - Serial connection for monitoring (9600 baud)
 *
 * DOCUMENTATION REFERENCE:
 * ATmega128 Datasheet: https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/2467S.pdf
 * - Timer/Counter1 (pages 86-107)
 * - PWM modes (pages 94-99)
 * - Output Compare pins (page 89)
 *
 * =============================================================================
 * TIMER1 PWM CONTROL REGISTERS - DETAILED REFERENCE FOR STUDENTS
 * =============================================================================
 *
 * REGISTER 1: TCCR1A (Timer/Counter1 Control Register A)
 *
 *    Bit:   7      6      5      4      3      2      1      0
 *    Name: COM1A1 COM1A0 COM1B1 COM1B0 COM1C1 COM1C0 WGM11  WGM10
 *
 * COM1A1:0 (bits 7-6): Compare Output Mode for Channel A (OC1A = PB5)
 *                      00 = Normal port operation, OC1A disconnected
 *                      10 = Non-inverting PWM: Clear OC1A on compare match
 *                           (High duty = high voltage, standard for motors)
 *                      11 = Inverting PWM: Set OC1A on compare match
 *                      Usage: TCCR1A |= (1<<COM1A1);  // Non-inverting
 *
 * COM1B1:0 (bits 5-4): Compare Output Mode for Channel B (OC1B = PB6)
 *                      Same modes as COM1A, for second PWM output
 *                      Useful for dual motor control or H-bridge control
 *
 * WGM11:10 (bits 1-0): Waveform Generation Mode (lower 2 bits)
 *                      Combined with WGM13:12 in TCCR1B to select PWM mode
 *
 * REGISTER 2: TCCR1B (Timer/Counter1 Control Register B)
 *
 *    Bit:   7      6      5      4      3      2      1      0
 *    Name: ICNC1  ICES1   -    WGM13  WGM12   CS12   CS11   CS10
 *
 * WGM13:10 (bits 4-3 + TCCR1A bits 1-0): Waveform Generation Mode
 *          MOTOR CONTROL MODES:
 *
 *          Mode 14: Fast PWM with ICR1 as TOP (MOST FLEXIBLE)
 *                   WGM13:10 = 1110
 *                   TCCR1A = (1<<WGM11)
 *                   TCCR1B = (1<<WGM13)|(1<<WGM12)
 *                   Frequency: F_CPU / (Prescaler * (1 + ICR1))
 *                   Duty cycle: OCR1A / ICR1
 *
 *          Mode 5: Fast PWM, 8-bit (SIMPLE, LOW RESOLUTION)
 *                  WGM13:10 = 0101
 *                  TOP = 0xFF (255)
 *                  Frequency: F_CPU / (Prescaler * 256)
 *                  Duty cycle: OCR1A / 255
 *
 *          Mode 1: Phase Correct PWM, 8-bit (SMOOTH, LOW FREQ)
 *                  WGM13:10 = 0001
 *                  Counts up and down (0→255→0)
 *                  Frequency: F_CPU / (Prescaler * 510)
 *
 * CS12:10 (bits 2-0): Clock Select (Prescaler) - CONTROLS PWM FREQUENCY
 *                     000 = No clock (timer stopped)
 *                     001 = clk/1 (no prescaling) - 16MHz
 *                     010 = clk/8 - 2MHz
 *                     011 = clk/64 - 250kHz
 *                     100 = clk/256 - 62.5kHz
 *                     101 = clk/1024 - 15.625kHz
 *
 * REGISTER 3: OCR1AH/OCR1AL (Output Compare Register 1A - 16-bit)
 *
 * Duty Cycle Control Register:
 * - Sets PWM pulse width
 * - Range: 0 to TOP (ICR1 or mode-dependent maximum)
 * - Duty cycle % = (OCR1A / TOP) × 100%
 *
 * Examples with ICR1=999 (Mode 14):
 *   OCR1A = 0:    0% duty (motor stopped)
 *   OCR1A = 250:  25% duty (slow speed)
 *   OCR1A = 500:  50% duty (medium speed)
 *   OCR1A = 750:  75% duty (fast speed)
 *   OCR1A = 999:  100% duty (maximum speed)
 *
 * CRITICAL: Must write high byte first, then low byte:
 *   OCR1AH = (duty >> 8);
 *   OCR1AL = (duty & 0xFF);
 * Or use 16-bit write: OCR1A = duty;
 *
 * REGISTER 4: ICR1H/ICR1L (Input Capture Register 1 - 16-bit)
 *
 * TOP Value for Mode 14 Fast PWM:
 * - Defines PWM frequency
 * - Higher value = lower frequency, higher resolution
 * - Lower value = higher frequency, lower resolution
 *
 * PWM Frequency Calculation:
 *   F_PWM = F_CPU / (Prescaler × (1 + ICR1))
 *
 * Examples @ 16MHz, Prescaler = 8:
 *   ICR1 = 999:   F_PWM = 16MHz/(8×1000) = 2kHz (good for motors)
 *   ICR1 = 1999:  F_PWM = 16MHz/(8×2000) = 1kHz (smoother, lower freq)
 *   ICR1 = 19999: F_PWM = 16MHz/(8×20000) = 100Hz (audible whine)
 *
 * MOTOR CONTROL RECOMMENDATIONS:
 * - PWM frequency: 1-20kHz (above human hearing ~16kHz preferred)
 * - Small motors: 10-20kHz
 * - Large motors: 1-5kHz
 * - Too high: Switching losses in H-bridge
 * - Too low: Audible noise, rough operation
 *
 * TYPICAL INITIALIZATION FOR DC MOTOR @ 2kHz:
 *
 *   void pwm_motor_init(void) {
 *       // Configure PB5 (OC1A) as output
 *       DDRB |= (1<<PB5);
 *
 *       // Mode 14: Fast PWM, ICR1=TOP, Non-inverting on OC1A
 *       TCCR1A = (1<<COM1A1) | (1<<WGM11);
 *       TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS11);  // Prescaler 8
 *
 *       // Set TOP for 2kHz PWM @ 16MHz
 *       ICR1 = 999;   // F_PWM = 16MHz/(8×1000) = 2000Hz
 *
 *       // Initial duty cycle (stopped)
 *       OCR1A = 0;
 *   }
 *
 * SPEED CONTROL FUNCTION:
 *
 *   void set_motor_speed(uint8_t percent) {
 *       // percent: 0-100
 *       uint16_t duty = (uint32_t)percent * ICR1 / 100;
 *       OCR1A = duty;
 *   }
 *
 * H-BRIDGE CONTROL (L298N typical):
 * - OC1A (PB5): PWM signal to Enable pin
 * - IN1, IN2: Direction control
 *
 *   Forward:  IN1=1, IN2=0, PWM on Enable
 *   Reverse:  IN1=0, IN2=1, PWM on Enable
 *   Brake:    IN1=0, IN2=0 (or both 1)
 *
 * DUAL MOTOR CONTROL:
 * - Use OC1A (PB5) for Motor 1
 * - Use OC1B (PB6) for Motor 2
 * - Independent duty cycles: OCR1A, OCR1B
 * - Same frequency (shared ICR1)
 *
 * ACCELERATION/DECELERATION:
 *
 *   void ramp_speed(uint16_t target_duty, uint8_t step) {
 *       uint16_t current = OCR1A;
 *       while(current != target_duty) {
 *           if(current < target_duty) current += step;
 *           else current -= step;
 *           OCR1A = current;
 *           _delay_ms(10);  // 10ms per step
 *       }
 *   }
 *
 * =============================================================================
 *
 * LEARNING PROGRESSION:
 * - Demo 1: Basic PWM Generation
 * - Demo 2: Motor Speed Control
 * - Demo 3: Direction Control
 * - Demo 4: Acceleration/Deceleration
 * - Demo 5: Closed-Loop Control
 *
 * =============================================================================
 */
*-Resolution : Number of steps in duty cycle(Timer1 = 16 - bit = 65536 steps) * /

#include "config.h"

// Motor control pins
#define MOTOR_PWM_PIN (1 << PB5)  // OC1A - Timer1 PWM output
#define MOTOR_DIR1_PIN (1 << PB6) // Direction control 1
#define MOTOR_DIR2_PIN (1 << PB7) // Direction control 2

// Motor direction definitions
#define MOTOR_FORWARD()           \
    do                            \
    {                             \
        PORTB |= MOTOR_DIR1_PIN;  \
        PORTB &= ~MOTOR_DIR2_PIN; \
    } while (0)
#define MOTOR_REVERSE()           \
    do                            \
    {                             \
        PORTB &= ~MOTOR_DIR1_PIN; \
        PORTB |= MOTOR_DIR2_PIN;  \
    } while (0)
#define MOTOR_BRAKE()                                \
    do                                               \
    {                                                \
        PORTB &= ~(MOTOR_DIR1_PIN | MOTOR_DIR2_PIN); \
    } while (0)

// PWM frequency calculation
// F_PWM = F_CPU / (Prescaler * (1 + TOP))
// Example: 7372800 / (8 * 1000) = 921 Hz
#define PWM_TOP 999 // 10-bit resolution (0-999)

    /*
     * Initialize Timer1 for Fast PWM mode
     * Mode 14: Fast PWM with ICR1 as TOP
     * Prescaler: 8
     * Non-inverting mode on OC1A
     */
    void timer1_pwm_init(void)
{
    // Set PB5 (OC1A) as output
    DDRB |= MOTOR_PWM_PIN | MOTOR_DIR1_PIN | MOTOR_DIR2_PIN;

    // Configure Timer1 for Fast PWM, Mode 14
    // WGM13:0 = 1110 (Fast PWM, TOP=ICR1)
    // COM1A1:0 = 10 (Clear OC1A on compare match, set at BOTTOM)
    // CS12:0 = 010 (Prescaler = 8)
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);

    // Set TOP value for desired PWM frequency
    ICR1 = PWM_TOP;

    // Initialize duty cycle to 0 (motor stopped)
    OCR1A = 0;

    // Initial state: braked
    MOTOR_BRAKE();
}

/*
 * Set motor speed (0-100%)
 */
void motor_set_speed(uint8_t speed_percent)
{
    if (speed_percent > 100)
        speed_percent = 100;

    // Convert percentage to PWM value
    uint16_t pwm_value = ((uint32_t)speed_percent * PWM_TOP) / 100;
    OCR1A = pwm_value;
}

/*
 * Set motor direction and speed
 */
void motor_drive(int8_t speed)
{
    if (speed > 0)
    {
        MOTOR_FORWARD();
        motor_set_speed(speed);
    }
    else if (speed < 0)
    {
        MOTOR_REVERSE();
        motor_set_speed(-speed);
    }
    else
    {
        MOTOR_BRAKE();
        motor_set_speed(0);
    }
}

/* ========================================================================
 * DEMO 1: Basic Speed Control with UART Commands
 * ======================================================================== */
void demo1_basic_speed_control(void)
{
    puts_USART1("\r\n=== DEMO 1: Basic Speed Control ===\r\n");
    puts_USART1("Commands:\r\n");
    puts_USART1("  0-9: Set speed (0=stop, 9=max)\r\n");
    puts_USART1("  f: Forward  r: Reverse  b: Brake\r\n");
    puts_USART1("  q: Return to menu\r\n\r\n");

    uint8_t current_speed = 0;
    char direction = 'f';

    while (1)
    {
        if (UCSR1A & (1 << RXC1))
        {
            char cmd = UDR1;

            if (cmd >= '0' && cmd <= '9')
            {
                current_speed = (cmd - '0') * 10; // 0-90%
                if (direction == 'f')
                {
                    motor_drive(current_speed);
                    puts_USART1("Forward @ ");
                }
                else
                {
                    motor_drive(-current_speed);
                    puts_USART1("Reverse @ ");
                }
                char buf[20];
                sprintf(buf, "%d%%\r\n", current_speed);
                puts_USART1(buf);
            }
            else if (cmd == 'f' || cmd == 'F')
            {
                direction = 'f';
                motor_drive(current_speed);
                puts_USART1("Direction: FORWARD\r\n");
            }
            else if (cmd == 'r' || cmd == 'R')
            {
                direction = 'r';
                motor_drive(-current_speed);
                puts_USART1("Direction: REVERSE\r\n");
            }
            else if (cmd == 'b' || cmd == 'B')
            {
                motor_drive(0);
                current_speed = 0;
                puts_USART1("BRAKED\r\n");
            }
            else if (cmd == 'q' || cmd == 'Q')
            {
                motor_drive(0);
                return;
            }
        }

        // Status LED blink
        static uint16_t counter = 0;
        if (++counter > 10000)
        {
            PORTC ^= 0x01;
            counter = 0;
        }
    }
}

/* ========================================================================
 * DEMO 2: Automatic Speed Ramp (Acceleration/Deceleration)
 * ======================================================================== */
void demo2_speed_ramp(void)
{
    puts_USART1("\r\n=== DEMO 2: Speed Ramping ===\r\n");
    puts_USART1("Demonstrating smooth acceleration and deceleration\r\n");
    puts_USART1("Press any key to continue, 'q' to quit\r\n\r\n");

    while (1)
    {
        // Ramp up forward
        puts_USART1("Ramping UP (Forward)...\r\n");
        MOTOR_FORWARD();
        for (uint8_t speed = 0; speed <= 100; speed += 5)
        {
            motor_set_speed(speed);
            char buf[30];
            sprintf(buf, "Speed: %d%%\r\n", speed);
            puts_USART1(buf);
            _delay_ms(200);

            if (UCSR1A & (1 << RXC1))
            {
                char cmd = UDR1;
                if (cmd == 'q' || cmd == 'Q')
                {
                    motor_drive(0);
                    return;
                }
            }
        }

        _delay_ms(1000);

        // Ramp down
        puts_USART1("Ramping DOWN...\r\n");
        for (int8_t speed = 100; speed >= 0; speed -= 5)
        {
            motor_set_speed(speed);
            char buf[30];
            sprintf(buf, "Speed: %d%%\r\n", speed);
            puts_USART1(buf);
            _delay_ms(200);

            if (UCSR1A & (1 << RXC1))
            {
                char cmd = UDR1;
                if (cmd == 'q' || cmd == 'Q')
                {
                    motor_drive(0);
                    return;
                }
            }
        }

        MOTOR_BRAKE();
        _delay_ms(1000);

        // Reverse direction
        puts_USART1("Ramping UP (Reverse)...\r\n");
        MOTOR_REVERSE();
        for (uint8_t speed = 0; speed <= 100; speed += 5)
        {
            motor_set_speed(speed);
            char buf[30];
            sprintf(buf, "Speed: %d%%\r\n", speed);
            puts_USART1(buf);
            _delay_ms(200);

            if (UCSR1A & (1 << RXC1))
            {
                char cmd = UDR1;
                if (cmd == 'q' || cmd == 'Q')
                {
                    motor_drive(0);
                    return;
                }
            }
        }

        _delay_ms(1000);

        // Ramp down again
        puts_USART1("Ramping DOWN...\r\n");
        for (int8_t speed = 100; speed >= 0; speed -= 5)
        {
            motor_set_speed(speed);
            _delay_ms(200);
        }

        MOTOR_BRAKE();
        puts_USART1("\r\nCycle complete!\r\n\r\n");
        _delay_ms(2000);
    }
}

/* ========================================================================
 * DEMO 3: PWM Frequency Analysis
 * ======================================================================== */
void demo3_pwm_frequency_test(void)
{
    puts_USART1("\r\n=== DEMO 3: PWM Frequency Test ===\r\n");
    puts_USART1("Testing different PWM frequencies\r\n");
    puts_USART1("Listen to motor sound changes\r\n");
    puts_USART1("Press any key to continue, 'q' to quit\r\n\r\n");

    uint16_t frequencies[] = {100, 500, 1000, 2000, 5000, 10000};
    uint8_t num_freqs = sizeof(frequencies) / sizeof(frequencies[0]);

    MOTOR_FORWARD();
    motor_set_speed(50); // 50% speed

    for (uint8_t i = 0; i < num_freqs; i++)
    {
        uint16_t top_value = (F_CPU / (8UL * frequencies[i])) - 1;
        ICR1 = top_value;
        OCR1A = top_value / 2; // 50% duty cycle

        char buf[50];
        sprintf(buf, "Frequency: %u Hz, TOP: %u\r\n", frequencies[i], top_value);
        puts_USART1(buf);

        _delay_ms(3000);

        if (UCSR1A & (1 << RXC1))
        {
            char cmd = UDR1;
            if (cmd == 'q' || cmd == 'Q')
            {
                motor_drive(0);
                ICR1 = PWM_TOP; // Restore default
                return;
            }
        }
    }

    motor_drive(0);
    ICR1 = PWM_TOP; // Restore default
    puts_USART1("\r\nFrequency test complete!\r\n");
}

/* ========================================================================
 * DEMO 4: ADC-Controlled Motor Speed (Potentiometer)
 * ======================================================================== */
void demo4_adc_speed_control(void)
{
    puts_USART1("\r\n=== DEMO 4: Potentiometer Speed Control ===\r\n");
    puts_USART1("Using ADC to read potentiometer for speed control\r\n");
    puts_USART1("ADC0: Speed control (0-1023 → 0-100%)\r\n");
    puts_USART1("Press 'd' to toggle direction, 'q' to quit\r\n\r\n");

    Adc_init();
    char direction = 'f';

    while (1)
    {
        // Read ADC value
        uint16_t adc_value = Read_Adc_Data(0);

        // Convert to speed (0-100%)
        uint8_t speed = (uint32_t)adc_value * 100 / 1023;

        // Apply speed with direction
        if (direction == 'f')
        {
            motor_drive(speed);
        }
        else
        {
            motor_drive(-speed);
        }

        // Display status every ~500ms
        static uint8_t display_counter = 0;
        if (++display_counter >= 10)
        {
            char buf[60];
            sprintf(buf, "ADC: %4u  Speed: %3d%%  Dir: %s\r\n",
                    adc_value, speed, (direction == 'f') ? "FWD" : "REV");
            puts_USART1(buf);
            display_counter = 0;
        }

        // Check for direction toggle command
        if (UCSR1A & (1 << RXC1))
        {
            char cmd = UDR1;
            if (cmd == 'd' || cmd == 'D')
            {
                direction = (direction == 'f') ? 'r' : 'f';
                puts_USART1("\r\nDirection toggled!\r\n\r\n");
            }
            else if (cmd == 'q' || cmd == 'Q')
            {
                motor_drive(0);
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
    puts_USART1("║   DC MOTOR PWM CONTROL - ATmega128    ║\r\n");
    puts_USART1("╚════════════════════════════════════════╝\r\n");
    puts_USART1("\r\n");
    puts_USART1("Select Demo:\r\n");
    puts_USART1("  [1] Basic Speed Control (UART)\r\n");
    puts_USART1("  [2] Automatic Speed Ramping\r\n");
    puts_USART1("  [3] PWM Frequency Test\r\n");
    puts_USART1("  [4] ADC Potentiometer Control\r\n");
    puts_USART1("\r\n");
    puts_USART1("Enter selection (1-4): ");
}

int main(void)
{
    // Initialize peripherals
    // init_devices();  // Manual initialization below
    Uart1_init();
    timer1_pwm_init();

    // Configure status LEDs
    DDRC = 0xFF;
    PORTC = 0x00;

    // Send startup message
    _delay_ms(500);
    puts_USART1("\r\n\r\n*** DC Motor PWM Control System ***\r\n");
    puts_USART1("ATmega128 @ ");
    char buf[30];
    sprintf(buf, "%lu", F_CPU);
    puts_USART1(buf);
    puts_USART1(" Hz\r\n");
    sprintf(buf, "PWM Frequency: %lu Hz\r\n", F_CPU / (8UL * (PWM_TOP + 1)));
    puts_USART1(buf);

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
            demo1_basic_speed_control();
            break;
        case '2':
            demo2_speed_ramp();
            break;
        case '3':
            demo3_pwm_frequency_test();
            break;
        case '4':
            demo4_adc_speed_control();
            break;
        default:
            puts_USART1("Invalid selection!\r\n");
            _delay_ms(1000);
            break;
        }

        // Ensure motor is stopped between demos
        motor_drive(0);
        _delay_ms(500);
    }

    return 0;
}
