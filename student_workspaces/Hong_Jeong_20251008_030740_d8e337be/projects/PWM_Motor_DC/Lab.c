/*
 * =============================================================================
 * PWM MOTOR CONTROL - HANDS-ON LAB EXERCISES
 * =============================================================================
 * ATmega128 Educational Framework - Lab Session
 *
 * OBJECTIVE: Master PWM-based motor control systems
 * DURATION: 75 minutes
 * DIFFICULTY: Intermediate
 *
 * STUDENTS WILL:
 * - Generate PWM signals for motor speed control
 * - Implement acceleration and deceleration profiles
 * - Create closed-loop motor control systems
 * - Build motor-controlled applications
 * - Debug motor control issues
 *
 * HARDWARE REQUIRED:
 * - ATmega128 board
 * - DC motor with H-bridge driver
 * - PWM output on OC1A (PB5)
 * - Direction control pins (PC0, PC1)
 * - Speed potentiometer on ADC2
 * - 4 control buttons
 * - Current sensor (optional)
 *
 * LAB STRUCTURE:
 * - Exercise 1: Basic PWM generation and motor control (20 min)
 * - Exercise 2: Speed ramping and acceleration profiles (20 min)
 * - Exercise 3: Direction control and H-bridge operation (20 min)
 * - Exercise 4: Advanced motor applications (15 min)
 *
 * =============================================================================
 */

#include "config.h"

// Lab configuration - Motor control pins
#define MOTOR_PWM_PIN 5     // PB5 (OC1A)
#define MOTOR_DIR_PIN1 0    // PC0 (Direction control 1)
#define MOTOR_DIR_PIN2 1    // PC1 (Direction control 2)
#define SPEED_POT_CHANNEL 2 // ADC2 (Speed potentiometer)

// Motor control constants
#define PWM_MAX 255
#define PWM_MIN 0
#define RAMP_STEP 5
#define RAMP_DELAY 50

// Motor directions
typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_FORWARD = 1,
    MOTOR_REVERSE = 2,
    MOTOR_BRAKE = 3
} motor_direction_t;

// Global variables for lab exercises
uint16_t lab_score = 0;
uint8_t current_speed = 0;
motor_direction_t current_direction = MOTOR_STOP;

/*
 * =============================================================================
 * HELPER FUNCTIONS
 * =============================================================================
 */

void setup_pwm_timer1(void)
{
    // Configure Timer1 for Fast PWM mode
    TCCR1A = (1 << COM1A1) | (1 << WGM11) | (1 << WGM10); // Fast PWM 8-bit, non-inverting
    TCCR1B = (1 << WGM12) | (1 << CS11);                  // Prescaler 8
    DDRB |= (1 << MOTOR_PWM_PIN);                         // Set PWM pin as output
}

void set_motor_pwm(uint8_t duty_cycle)
{
    OCR1A = duty_cycle;
    current_speed = duty_cycle;
}

void set_motor_direction(motor_direction_t direction)
{
    current_direction = direction;

    switch (direction)
    {
    case MOTOR_STOP:
        PORTC &= ~(1 << MOTOR_DIR_PIN1);
        PORTC &= ~(1 << MOTOR_DIR_PIN2);
        set_motor_pwm(0);
        break;

    case MOTOR_FORWARD:
        PORTC |= (1 << MOTOR_DIR_PIN1);
        PORTC &= ~(1 << MOTOR_DIR_PIN2);
        break;

    case MOTOR_REVERSE:
        PORTC &= ~(1 << MOTOR_DIR_PIN1);
        PORTC |= (1 << MOTOR_DIR_PIN2);
        break;

    case MOTOR_BRAKE:
        PORTC |= (1 << MOTOR_DIR_PIN1);
        PORTC |= (1 << MOTOR_DIR_PIN2);
        set_motor_pwm(0);
        break;
    }
}

/*
 * =============================================================================
 * LAB EXERCISE 1: BASIC PWM GENERATION (20 minutes)
 * =============================================================================
 * OBJECTIVE: Generate PWM signals and control motor speed
 * DIFFICULTY: ★★☆☆☆ (Easy-Medium)
 */

void lab_ex1_pwm_basics(void)
{
    /*
     * CHALLENGE: Generate different PWM duty cycles
     * TASK: Create various PWM signals and observe motor response
     * LEARNING: PWM fundamentals and motor speed control
     */

    puts_USART1("\\r\\n=== Lab 1: PWM Basics ===\\r\\n");
    puts_USART1("Testing various PWM duty cycles\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "PWM MOTOR CONTROL");
    lcd_string(1, 0, "Testing duty cycles");

    setup_pwm_timer1();
    DDRC |= (1 << MOTOR_DIR_PIN1) | (1 << MOTOR_DIR_PIN2); // Direction pins as output

    set_motor_direction(MOTOR_FORWARD);

    // Test different duty cycles
    uint8_t test_speeds[] = {50, 100, 150, 200, 255};
    uint8_t num_tests = sizeof(test_speeds) / sizeof(test_speeds[0]);

    for (uint8_t i = 0; i < num_tests; i++)
    {
        uint8_t speed = test_speeds[i];
        set_motor_pwm(speed);

        char speed_msg[30];
        sprintf(speed_msg, "Speed: %d (%d%%)    ", speed, (speed * 100) / 255);
        lcd_string(3, 0, speed_msg);

        char serial_msg[50];
        sprintf(serial_msg, "PWM Duty Cycle: %d/255 (%d%%)\\r\\n", speed, (speed * 100) / 255);
        puts_USART1(serial_msg);

        _delay_ms(3000); // Run each speed for 3 seconds
    }

    set_motor_direction(MOTOR_STOP);
    puts_USART1("PWM basics test complete!\\r\\n");
    lab_score += 100;
}

void lab_ex1_manual_speed_control(void)
{
    /*
     * CHALLENGE: Manual speed control with potentiometer
     * TASK: Use ADC to read potentiometer and control motor speed
     * LEARNING: Analog input to PWM output mapping
     */

    puts_USART1("\\r\\n=== Lab 1.2: Manual Speed Control ===\\r\\n");
    puts_USART1("Use potentiometer to control motor speed\\r\\n");
    puts_USART1("Press button to exit...\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "MANUAL CONTROL");
    lcd_string(1, 0, "Turn potentiometer");

    set_motor_direction(MOTOR_FORWARD);

    while (!button_pressed(0))
    {
        // Read potentiometer value (0-1023)
        uint16_t pot_value = Read_Adc_Data(SPEED_POT_CHANNEL);

        // Map to PWM range (0-255)
        uint8_t pwm_value = (pot_value * 255L) / 1023;

        set_motor_pwm(pwm_value);

        // Display current values
        char display_msg[20];
        sprintf(display_msg, "ADC:%4d PWM:%3d", pot_value, pwm_value);
        lcd_string(3, 0, display_msg);

        sprintf(display_msg, "Speed: %3d%%    ", (pwm_value * 100) / 255);
        lcd_string(4, 0, display_msg);

        _delay_ms(100);
    }

    set_motor_direction(MOTOR_STOP);
    lab_score += 100;
}

/*
 * =============================================================================
 * LAB EXERCISE 2: SPEED RAMPING (20 minutes)
 * =============================================================================
 * OBJECTIVE: Implement smooth acceleration and deceleration
 * DIFFICULTY: ★★★☆☆ (Medium)
 */

void lab_ex2_acceleration_profiles(void)
{
    /*
     * CHALLENGE: Implement smooth speed ramping
     * TASK: Create acceleration and deceleration curves
     * LEARNING: Rate limiting and smooth control
     */

    puts_USART1("\\r\\n=== Lab 2: Acceleration Profiles ===\\r\\n");
    puts_USART1("Testing smooth speed transitions\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "ACCELERATION TEST");
    lcd_string(1, 0, "Smooth ramping");

    set_motor_direction(MOTOR_FORWARD);

    // Test 1: Linear acceleration
    puts_USART1("Linear acceleration to 200...\\r\\n");
    lcd_string(2, 0, "Linear ramp up");

    for (uint8_t speed = 0; speed <= 200; speed += RAMP_STEP)
    {
        set_motor_pwm(speed);

        char speed_msg[20];
        sprintf(speed_msg, "Speed: %3d", speed);
        lcd_string(4, 0, speed_msg);

        _delay_ms(RAMP_DELAY);
    }

    _delay_ms(2000); // Hold at max speed

    // Test 2: Linear deceleration
    puts_USART1("Linear deceleration to 0...\\r\\n");
    lcd_string(2, 0, "Linear ramp down");

    for (uint8_t speed = 200; speed > 0; speed -= RAMP_STEP)
    {
        set_motor_pwm(speed);

        char speed_msg[20];
        sprintf(speed_msg, "Speed: %3d", speed);
        lcd_string(4, 0, speed_msg);

        _delay_ms(RAMP_DELAY);
    }

    set_motor_direction(MOTOR_STOP);
    lab_score += 150;
}

void lab_ex2_custom_profiles(void)
{
    /*
     * CHALLENGE: Create custom acceleration profiles
     * TASK: Implement S-curve acceleration (slow-fast-slow)
     * LEARNING: Advanced motion control algorithms
     */

    puts_USART1("\\r\\n=== Lab 2.2: S-Curve Profile ===\\r\\n");
    puts_USART1("Implementing S-curve acceleration\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "S-CURVE PROFILE");
    lcd_string(1, 0, "Smooth acceleration");

    set_motor_direction(MOTOR_FORWARD);

    // S-curve: slow start, fast middle, slow end
    uint8_t target_speed = 180;
    uint8_t steps = 36; // Total steps for acceleration

    for (uint8_t i = 0; i <= steps; i++)
    {
        // S-curve formula: smooth acceleration
        float t = (float)i / steps;              // Normalized time (0 to 1)
        float s_curve = t * t * (3.0 - 2.0 * t); // Smooth step function
        uint8_t speed = (uint8_t)(s_curve * target_speed);

        set_motor_pwm(speed);

        char speed_msg[20];
        sprintf(speed_msg, "Speed: %3d", speed);
        lcd_string(3, 0, speed_msg);

        char progress_msg[20];
        sprintf(progress_msg, "Step: %2d/%2d", i, steps);
        lcd_string(4, 0, progress_msg);

        _delay_ms(100);
    }

    _delay_ms(2000); // Hold at target speed

    // S-curve deceleration
    for (uint8_t i = steps; i > 0; i--)
    {
        float t = (float)i / steps;
        float s_curve = t * t * (3.0 - 2.0 * t);
        uint8_t speed = (uint8_t)(s_curve * target_speed);

        set_motor_pwm(speed);

        char speed_msg[20];
        sprintf(speed_msg, "Speed: %3d", speed);
        lcd_string(3, 0, speed_msg);

        _delay_ms(100);
    }

    set_motor_direction(MOTOR_STOP);
    lab_score += 150;
}

/*
 * =============================================================================
 * LAB EXERCISE 3: DIRECTION CONTROL (20 minutes)
 * =============================================================================
 * OBJECTIVE: Master H-bridge direction control
 * DIFFICULTY: ★★★☆☆ (Medium)
 */

void lab_ex3_direction_control(void)
{
    /*
     * CHALLENGE: Control motor direction with H-bridge
     * TASK: Implement forward, reverse, stop, and brake functions
     * LEARNING: H-bridge operation and motor protection
     */

    puts_USART1("\\r\\n=== Lab 3: Direction Control ===\\r\\n");
    puts_USART1("Testing H-bridge direction control\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "DIRECTION CONTROL");
    lcd_string(1, 0, "H-bridge testing");

    uint8_t test_speed = 150;

    // Test forward direction
    puts_USART1("Testing FORWARD direction...\\r\\n");
    lcd_string(3, 0, "Direction: FORWARD ");
    set_motor_direction(MOTOR_FORWARD);
    set_motor_pwm(test_speed);
    _delay_ms(3000);

    // Stop before changing direction
    puts_USART1("Stopping motor...\\r\\n");
    lcd_string(3, 0, "Direction: STOP    ");
    set_motor_direction(MOTOR_STOP);
    _delay_ms(1000);

    // Test reverse direction
    puts_USART1("Testing REVERSE direction...\\r\\n");
    lcd_string(3, 0, "Direction: REVERSE ");
    set_motor_direction(MOTOR_REVERSE);
    set_motor_pwm(test_speed);
    _delay_ms(3000);

    // Test brake
    puts_USART1("Testing BRAKE function...\\r\\n");
    lcd_string(3, 0, "Direction: BRAKE   ");
    set_motor_direction(MOTOR_BRAKE);
    _delay_ms(2000);

    set_motor_direction(MOTOR_STOP);
    lab_score += 150;
}

/*
 * =============================================================================
 * LAB EXERCISE 4: MOTOR APPLICATIONS (15 minutes)
 * =============================================================================
 * OBJECTIVE: Build practical motor control applications
 * DIFFICULTY: ★★★★☆ (Medium-Hard)
 */

void lab_ex4_motor_sequencer(void)
{
    /*
     * CHALLENGE: Create a motor movement sequence
     * TASK: Program a sequence of moves with different speeds and directions
     * LEARNING: Sequence programming and timing control
     */

    puts_USART1("\\r\\n=== Lab 4: Motor Sequencer ===\\r\\n");
    puts_USART1("Running programmed motor sequence\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "MOTOR SEQUENCER");
    lcd_string(1, 0, "Running sequence...");

    // Define sequence: direction, speed, duration (seconds)
    struct
    {
        motor_direction_t direction;
        uint8_t speed;
        uint8_t duration;
        const char *name;
    } sequence[] = {
        {MOTOR_FORWARD, 100, 2, "Slow Forward "},
        {MOTOR_FORWARD, 200, 2, "Fast Forward "},
        {MOTOR_STOP, 0, 1, "Stop         "},
        {MOTOR_REVERSE, 150, 3, "Med Reverse  "},
        {MOTOR_BRAKE, 0, 1, "Brake        "},
        {MOTOR_FORWARD, 180, 2, "Final Forward"}};

    uint8_t num_steps = sizeof(sequence) / sizeof(sequence[0]);

    for (uint8_t step = 0; step < num_steps; step++)
    {
        char step_msg[30];
        sprintf(step_msg, "Step %d/%d", step + 1, num_steps);
        lcd_string(2, 0, step_msg);

        lcd_string(3, 0, sequence[step].name);

        char details[20];
        sprintf(details, "Speed: %3d", sequence[step].speed);
        lcd_string(4, 0, details);

        set_motor_direction(sequence[step].direction);
        if (sequence[step].direction == MOTOR_FORWARD || sequence[step].direction == MOTOR_REVERSE)
        {
            set_motor_pwm(sequence[step].speed);
        }

        char serial_msg[60];
        sprintf(serial_msg, "Step %d: %s Speed=%d Duration=%ds\\r\\n",
                step + 1, sequence[step].name, sequence[step].speed, sequence[step].duration);
        puts_USART1(serial_msg);

        _delay_ms(sequence[step].duration * 1000);
    }

    set_motor_direction(MOTOR_STOP);
    puts_USART1("Motor sequence complete!\\r\\n");
    lab_score += 200;
}

/*
 * =============================================================================
 * LAB MAIN PROGRAM - EXERCISE SELECTION
 * =============================================================================
 */

void show_lab_menu(void)
{
    puts_USART1("\\r\\n");
    puts_USART1("==============================================\\r\\n");
    puts_USART1("     PWM MOTOR CONTROL - LAB EXERCISES       \\r\\n");
    puts_USART1("==============================================\\r\\n");
    puts_USART1("1. PWM Basics & Manual Control               \\r\\n");
    puts_USART1("2. Speed Ramping & Acceleration Profiles     \\r\\n");
    puts_USART1("3. Direction Control & H-Bridge Operation    \\r\\n");
    puts_USART1("4. Motor Sequencer Application               \\r\\n");
    puts_USART1("                                              \\r\\n");
    puts_USART1("0. Run All Exercises                         \\r\\n");
    puts_USART1("X. Exit Lab                                   \\r\\n");
    puts_USART1("==============================================\\r\\n");
    char score_msg[50];
    sprintf(score_msg, "Current Score: %d points\\r\\n", lab_score);
    puts_USART1(score_msg);
    puts_USART1("Select exercise (1-4, 0, X): ");
}

int main(void)
{
    init_devices();

    puts_USART1("\\r\\n*** PWM MOTOR CONTROL LAB SESSION ***\\r\\n");
    puts_USART1("Welcome to hands-on motor control programming!\\r\\n");
    puts_USART1("SAFETY: Ensure motor is properly mounted and area is clear!\\r\\n");

    lcd_clear();
    lcd_string(1, 0, "MOTOR CONTROL LAB");
    lcd_string(2, 0, "Check connections");
    lcd_string(4, 0, "Use Serial Menu");

    while (1)
    {
        show_lab_menu();
        char choice = getch_USART1();
        putch_USART1(choice);
        putch_USART1('\\r');
        putch_USART1('\\n');

        switch (choice)
        {
        case '1':
            lab_ex1_pwm_basics();
            lab_ex1_manual_speed_control();
            break;

        case '2':
            lab_ex2_acceleration_profiles();
            lab_ex2_custom_profiles();
            break;

        case '3':
            lab_ex3_direction_control();
            break;

        case '4':
            lab_ex4_motor_sequencer();
            break;

        case '0':
            puts_USART1("\\r\\n*** RUNNING ALL EXERCISES ***\\r\\n");
            lab_ex1_pwm_basics();
            lab_ex1_manual_speed_control();
            lab_ex2_acceleration_profiles();
            lab_ex2_custom_profiles();
            lab_ex3_direction_control();
            lab_ex4_motor_sequencer();

            char final_buffer[80];
            sprintf(final_buffer, "\\r\\n*** ALL EXERCISES COMPLETE! ***\\r\\nFinal Score: %d points\\r\\n", lab_score);
            puts_USART1(final_buffer);
            break;

        case 'X':
        case 'x':
            puts_USART1("\\r\\nExiting lab. Great work!\\r\\n");
            puts_USART1("Remember: Always disconnect power when done!\\r\\n");
            lcd_clear();
            lcd_string(2, 0, "LAB COMPLETE!");
            char exit_score[30];
            sprintf(exit_score, "Score: %d pts", lab_score);
            lcd_string(3, 0, exit_score);
            set_motor_direction(MOTOR_STOP); // Ensure motor is stopped
            while (1)
                ;

        default:
            puts_USART1("Invalid choice. Please try again.\\r\\n");
        }

        puts_USART1("\\r\\nPress any key to continue...\\r\\n");
        getch_USART1();
    }

    return 0;
}