/*
 * =============================================================================
 * SERVO MOTOR CONTROL - HANDS-ON LAB EXERCISES
 * =============================================================================
 * ATmega128 Educational Framework - Lab Session
 *
 * OBJECTIVE: Master PWM-based servo motor control systems
 * DURATION: 80 minutes
 * DIFFICULTY: Intermediate-Advanced
 *
 * STUDENTS WILL:
 * - Generate precise PWM signals for servo control
 * - Implement position control and smooth movements
 * - Create multi-servo coordination systems
 * - Build servo-based applications and robotics
 * - Debug servo timing and calibration issues
 *
 * HARDWARE REQUIRED:
 * - ATmega128 board
 * - 2-4 standard servo motors (SG90 or similar)
 * - External 5V power supply for servos
 * - PWM outputs on OC1A (PB5), OC1B (PB6), OC3A (PE3)
 * - 4 control buttons and potentiometer
 * - Position feedback LEDs
 *
 * SERVO CONTROL THEORY:
 * - Standard servos: 20ms period (50Hz)
 * - Pulse width: 1ms (0°) to 2ms (180°)
 * - Neutral position: 1.5ms (90°)
 * - Resolution: ~0.1° with 16-bit timer
 *
 * LAB STRUCTURE:
 * - Exercise 1: Single servo PWM generation and calibration (25 min)
 * - Exercise 2: Position control and smooth movements (20 min)
 * - Exercise 3: Multi-servo coordination (20 min)
 * - Exercise 4: Servo-based applications (15 min)
 *
 * =============================================================================
 */

#include "config.h"

// Servo control pins and configuration
#define SERVO1_PIN 5 // PB5 (OC1A)
#define SERVO2_PIN 6 // PB6 (OC1B)
#define SERVO3_PIN 3 // PE3 (OC3A)

// Servo timing constants (for 16-bit timer at 7.3728MHz)
#define SERVO_PERIOD 14745   // 20ms period (7372800/1024/50)
#define SERVO_MIN_PULSE 737  // 1ms pulse width
#define SERVO_MAX_PULSE 1474 // 2ms pulse width
#define SERVO_CENTER 1106    // 1.5ms center position

// Servo position constants
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define SERVO_CENTER_ANGLE 90

// Lab session variables
uint16_t lab_score = 0;
uint16_t servo_movements = 0;
uint8_t current_servo1_angle = 90;
uint8_t current_servo2_angle = 90;
uint8_t current_servo3_angle = 90;

/*
 * =============================================================================
 * SERVO CONTROL FUNCTIONS
 * =============================================================================
 */

void servo_timer_init(void)
{
    // Configure Timer1 for servo control (16-bit, Fast PWM, ICR1 as TOP)
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);            // Non-inverting PWM
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12) | (1 << CS10); // Fast PWM, prescaler 1024
    ICR1 = SERVO_PERIOD;                                              // Set period to 20ms

    // Configure Timer3 for additional servo (16-bit, Fast PWM)
    TCCR3A = (1 << COM3A1) | (1 << WGM31);
    TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS32) | (1 << CS30); // Fast PWM, prescaler 1024
    ICR3 = SERVO_PERIOD;

    // Set servo pins as outputs
    DDRB |= (1 << SERVO1_PIN) | (1 << SERVO2_PIN);
    DDRE |= (1 << SERVO3_PIN);

    // Initialize servos to center position
    OCR1A = SERVO_CENTER; // Servo 1
    OCR1B = SERVO_CENTER; // Servo 2
    OCR3A = SERVO_CENTER; // Servo 3
}

uint16_t angle_to_pulse_width(uint8_t angle)
{
    // Convert angle (0-180°) to pulse width (737-1474)
    if (angle > 180)
        angle = 180;
    return SERVO_MIN_PULSE + ((uint32_t)(angle) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / 180;
}

void set_servo_angle(uint8_t servo_num, uint8_t angle)
{
    uint16_t pulse_width = angle_to_pulse_width(angle);

    switch (servo_num)
    {
    case 1:
        OCR1A = pulse_width;
        current_servo1_angle = angle;
        break;
    case 2:
        OCR1B = pulse_width;
        current_servo2_angle = angle;
        break;
    case 3:
        OCR3A = pulse_width;
        current_servo3_angle = angle;
        break;
    }

    servo_movements++;
}

void move_servo_smooth(uint8_t servo_num, uint8_t target_angle, uint8_t speed_delay)
{
    uint8_t current_angle;

    // Get current angle
    switch (servo_num)
    {
    case 1:
        current_angle = current_servo1_angle;
        break;
    case 2:
        current_angle = current_servo2_angle;
        break;
    case 3:
        current_angle = current_servo3_angle;
        break;
    default:
        return;
    }

    // Move step by step
    if (current_angle < target_angle)
    {
        for (uint8_t angle = current_angle; angle <= target_angle; angle++)
        {
            set_servo_angle(servo_num, angle);
            _delay_ms(speed_delay);
        }
    }
    else
    {
        for (uint8_t angle = current_angle; angle >= target_angle; angle--)
        {
            set_servo_angle(servo_num, angle);
            _delay_ms(speed_delay);
            if (angle == 0)
                break; // Prevent underflow
        }
    }
}

/*
 * =============================================================================
 * LAB EXERCISE 1: SINGLE SERVO CONTROL (25 minutes)
 * =============================================================================
 * OBJECTIVE: Master basic servo PWM generation and calibration
 * DIFFICULTY: ★★☆☆☆ (Easy-Medium)
 */

void lab_ex1_servo_calibration(void)
{
    /*
     * CHALLENGE: Calibrate servo and test position accuracy
     * TASK: Generate proper PWM signals and verify servo response
     * LEARNING: PWM timing, servo mechanics, calibration procedures
     */

    puts_USART1("\\r\\n=== Lab 1: Servo Calibration ===\\r\\n");
    puts_USART1("Calibrating servo motor positions\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "SERVO CALIBRATION");
    lcd_string(1, 0, "Testing positions");

    servo_timer_init();

    // Test key positions
    uint8_t test_angles[] = {0, 45, 90, 135, 180};
    const char *position_names[] = {"0° (Min)", "45° (Low)", "90° (Center)", "135° (High)", "180° (Max)"};
    uint8_t num_positions = sizeof(test_angles) / sizeof(test_angles[0]);

    for (uint8_t i = 0; i < num_positions; i++)
    {
        uint8_t angle = test_angles[i];

        char test_msg[50];
        sprintf(test_msg, "Moving to %s...\\r\\n", position_names[i]);
        puts_USART1(test_msg);

        // Update LCD
        char lcd_msg[20];
        sprintf(lcd_msg, "Pos: %s", position_names[i]);
        lcd_string(3, 0, lcd_msg);

        sprintf(lcd_msg, "Angle: %3d°", angle);
        lcd_string(4, 0, lcd_msg);

        // Move servo to position
        set_servo_angle(1, angle);

        // Show pulse width for debugging
        uint16_t pulse_width = angle_to_pulse_width(angle);
        char debug_msg[50];
        sprintf(debug_msg, "Pulse width: %d (%.2fms)\\r\\n",
                pulse_width, (float)pulse_width * 1024.0 / 7372800.0 * 1000.0);
        puts_USART1(debug_msg);

        _delay_ms(2000); // Hold position for observation
    }

    puts_USART1("Servo calibration complete!\\r\\n");
    lab_score += 100;
}

void lab_ex1_manual_control(void)
{
    /*
     * CHALLENGE: Manual servo control with potentiometer
     * TASK: Map analog input to servo position with real-time feedback
     * LEARNING: Real-time control, analog-to-angle mapping
     */

    puts_USART1("\\r\\n=== Lab 1.2: Manual Control ===\\r\\n");
    puts_USART1("Use potentiometer to control servo position\\r\\n");
    puts_USART1("Press button to exit manual control...\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "MANUAL CONTROL");
    lcd_string(1, 0, "Turn potentiometer");

    while (!button_pressed(0))
    {
        // Read potentiometer (ADC channel 2)
        uint16_t adc_value = Read_Adc_Data(2);

        // Map ADC value (0-1023) to servo angle (0-180)
        uint8_t target_angle = (adc_value * 180L) / 1023;

        // Update servo position
        set_servo_angle(1, target_angle);

        // Display current values
        char display_msg[20];
        sprintf(display_msg, "ADC: %4d", adc_value);
        lcd_string(3, 0, display_msg);

        sprintf(display_msg, "Angle: %3d°", target_angle);
        lcd_string(4, 0, display_msg);

        char pulse_msg[20];
        uint16_t pulse = angle_to_pulse_width(target_angle);
        sprintf(pulse_msg, "Pulse: %4d", pulse);
        lcd_string(5, 0, pulse_msg);

        _delay_ms(50); // Smooth update rate
    }

    lab_score += 100;
}

/*
 * =============================================================================
 * LAB EXERCISE 2: SMOOTH MOVEMENTS (20 minutes)
 * =============================================================================
 * OBJECTIVE: Implement smooth servo movements and motion profiles
 * DIFFICULTY: ★★★☆☆ (Medium)
 */

void lab_ex2_smooth_movements(void)
{
    /*
     * CHALLENGE: Create smooth servo movements between positions
     * TASK: Implement different motion profiles and speeds
     * LEARNING: Motion planning, acceleration profiles, smooth control
     */

    puts_USART1("\\r\\n=== Lab 2: Smooth Movements ===\\r\\n");
    puts_USART1("Testing different movement profiles\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "SMOOTH MOVEMENTS");
    lcd_string(1, 0, "Motion profiles");

    // Test 1: Slow smooth movement
    puts_USART1("Test 1: Slow smooth movement (0° → 180°)\\r\\n");
    lcd_string(3, 0, "Slow movement");
    move_servo_smooth(1, 0, 10); // Start at 0°
    _delay_ms(500);
    move_servo_smooth(1, 180, 20); // Move to 180° slowly
    _delay_ms(1000);

    // Test 2: Fast smooth movement
    puts_USART1("Test 2: Fast smooth movement (180° → 0°)\\r\\n");
    lcd_string(3, 0, "Fast movement");
    move_servo_smooth(1, 0, 5); // Move back quickly
    _delay_ms(1000);

    // Test 3: Oscillation
    puts_USART1("Test 3: Oscillation pattern\\r\\n");
    lcd_string(3, 0, "Oscillation");

    for (uint8_t cycle = 0; cycle < 3; cycle++)
    {
        move_servo_smooth(1, 60, 10);  // 90° → 60°
        move_servo_smooth(1, 120, 10); // 60° → 120°

        char cycle_msg[20];
        sprintf(cycle_msg, "Cycle: %d/3", cycle + 1);
        lcd_string(4, 0, cycle_msg);
    }

    move_servo_smooth(1, 90, 10); // Return to center

    // Test 4: Step response
    puts_USART1("Test 4: Step response analysis\\r\\n");
    lcd_string(3, 0, "Step response");

    uint8_t step_positions[] = {45, 135, 90, 0, 180, 90};
    uint8_t num_steps = sizeof(step_positions) / sizeof(step_positions[0]);

    for (uint8_t i = 0; i < num_steps; i++)
    {
        char step_msg[40];
        sprintf(step_msg, "Step %d: Moving to %d°\\r\\n", i + 1, step_positions[i]);
        puts_USART1(step_msg);

        set_servo_angle(1, step_positions[i]); // Instant movement

        char lcd_msg[20];
        sprintf(lcd_msg, "Step: %3d°", step_positions[i]);
        lcd_string(4, 0, lcd_msg);

        _delay_ms(1000); // Hold position
    }

    puts_USART1("Smooth movement tests complete!\\r\\n");
    lab_score += 150;
}

/*
 * =============================================================================
 * LAB EXERCISE 3: MULTI-SERVO COORDINATION (20 minutes)
 * =============================================================================
 * OBJECTIVE: Control multiple servos simultaneously
 * DIFFICULTY: ★★★★☆ (Medium-Hard)
 */

void lab_ex3_multi_servo_coordination(void)
{
    /*
     * CHALLENGE: Coordinate multiple servos for complex movements
     * TASK: Create synchronized and choreographed servo sequences
     * LEARNING: Multi-axis control, synchronization, coordination algorithms
     */

    puts_USART1("\\r\\n=== Lab 3: Multi-Servo Coordination ===\\r\\n");
    puts_USART1("Coordinating multiple servo motors\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "MULTI-SERVO COORD");
    lcd_string(1, 0, "3-axis control");

    // Initialize all servos to center
    set_servo_angle(1, 90);
    set_servo_angle(2, 90);
    set_servo_angle(3, 90);
    _delay_ms(1000);

    // Choreography 1: Sequential movement
    puts_USART1("Choreography 1: Sequential movement\\r\\n");
    lcd_string(3, 0, "Sequential");

    for (uint8_t i = 0; i < 3; i++)
    {
        // Move each servo in sequence
        move_servo_smooth(1, 45, 15);
        move_servo_smooth(2, 45, 15);
        move_servo_smooth(3, 45, 15);

        _delay_ms(500);

        move_servo_smooth(1, 135, 15);
        move_servo_smooth(2, 135, 15);
        move_servo_smooth(3, 135, 15);

        _delay_ms(500);
    }

    // Return to center
    move_servo_smooth(1, 90, 10);
    move_servo_smooth(2, 90, 10);
    move_servo_smooth(3, 90, 10);

    // Choreography 2: Synchronized movement
    puts_USART1("Choreography 2: Synchronized movement\\r\\n");
    lcd_string(3, 0, "Synchronized");

    for (uint8_t step = 0; step <= 90; step += 5)
    {
        // All servos move together
        set_servo_angle(1, 90 + step / 2); // Servo 1: 90° → 135°
        set_servo_angle(2, 90 - step / 2); // Servo 2: 90° → 45°
        set_servo_angle(3, 90 + step);     // Servo 3: 90° → 180°

        char sync_msg[30];
        sprintf(sync_msg, "Step: %2d", step);
        lcd_string(4, 0, sync_msg);

        _delay_ms(100);
    }

    _delay_ms(1000);

    // Return to center
    for (uint8_t step = 0; step <= 90; step += 5)
    {
        set_servo_angle(1, 135 - step / 2); // Return to 90°
        set_servo_angle(2, 45 + step / 2);  // Return to 90°
        set_servo_angle(3, 180 - step);     // Return to 90°
        _delay_ms(100);
    }

    // Choreography 3: Wave pattern
    puts_USART1("Choreography 3: Wave pattern\\r\\n");
    lcd_string(3, 0, "Wave pattern");

    for (uint8_t wave = 0; wave < 360; wave += 10)
    {
        // Create sinusoidal wave pattern
        float angle1 = 90 + 45 * sin(wave * M_PI / 180);
        float angle2 = 90 + 45 * sin((wave + 120) * M_PI / 180);
        float angle3 = 90 + 45 * sin((wave + 240) * M_PI / 180);

        set_servo_angle(1, (uint8_t)angle1);
        set_servo_angle(2, (uint8_t)angle2);
        set_servo_angle(3, (uint8_t)angle3);

        char wave_msg[20];
        sprintf(wave_msg, "Wave: %3d°", wave);
        lcd_string(4, 0, wave_msg);

        _delay_ms(100);
    }

    // Return all to center
    set_servo_angle(1, 90);
    set_servo_angle(2, 90);
    set_servo_angle(3, 90);

    puts_USART1("Multi-servo coordination complete!\\r\\n");
    lab_score += 200;
}

/*
 * =============================================================================
 * LAB EXERCISE 4: SERVO APPLICATIONS (15 minutes)
 * =============================================================================
 * OBJECTIVE: Build practical servo-based applications
 * DIFFICULTY: ★★★★★ (Hard)
 */

void lab_ex4_servo_applications(void)
{
    /*
     * CHALLENGE: Create a complete servo-based system
     * TASK: Build a pan-tilt camera mount or robotic arm simulation
     * LEARNING: System integration, user interfaces, practical applications
     */

    puts_USART1("\\r\\n=== Lab 4: Servo Applications ===\\r\\n");
    puts_USART1("Building pan-tilt camera mount system\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "PAN-TILT SYSTEM");
    lcd_string(1, 0, "Camera mount");

    // Define pan-tilt system
    uint8_t pan_angle = 90;  // Servo 1: horizontal movement
    uint8_t tilt_angle = 90; // Servo 2: vertical movement

    puts_USART1("Commands: w/s (tilt), a/d (pan), c (center), q (quit)\\r\\n");
    puts_USART1("Control the pan-tilt system with keyboard...\\r\\n");

    // Set initial position
    set_servo_angle(1, pan_angle);
    set_servo_angle(2, tilt_angle);

    char control_char = 0;
    uint8_t movements = 0;

    while (control_char != 'q' && movements < 50)
    {
        // Display current position
        char pos_msg[20];
        sprintf(pos_msg, "Pan: %3d°", pan_angle);
        lcd_string(3, 0, pos_msg);

        sprintf(pos_msg, "Tilt:%3d°", tilt_angle);
        lcd_string(4, 0, pos_msg);

        char serial_msg[50];
        sprintf(serial_msg, "Position - Pan: %d°, Tilt: %d°\\r\\n", pan_angle, tilt_angle);
        puts_USART1(serial_msg);

        puts_USART1("Command: ");
        control_char = getch_USART1();
        putch_USART1(control_char);
        puts_USART1("\\r\\n");

        switch (control_char)
        {
        case 'w': // Tilt up
            if (tilt_angle < 180)
            {
                tilt_angle += 10;
                move_servo_smooth(2, tilt_angle, 5);
                puts_USART1("Tilting up\\r\\n");
                movements++;
            }
            break;

        case 's': // Tilt down
            if (tilt_angle > 0)
            {
                tilt_angle -= 10;
                move_servo_smooth(2, tilt_angle, 5);
                puts_USART1("Tilting down\\r\\n");
                movements++;
            }
            break;

        case 'a': // Pan left
            if (pan_angle > 0)
            {
                pan_angle -= 10;
                move_servo_smooth(1, pan_angle, 5);
                puts_USART1("Panning left\\r\\n");
                movements++;
            }
            break;

        case 'd': // Pan right
            if (pan_angle < 180)
            {
                pan_angle += 10;
                move_servo_smooth(1, pan_angle, 5);
                puts_USART1("Panning right\\r\\n");
                movements++;
            }
            break;

        case 'c': // Center
            pan_angle = 90;
            tilt_angle = 90;
            puts_USART1("Centering...\\r\\n");
            move_servo_smooth(1, pan_angle, 8);
            move_servo_smooth(2, tilt_angle, 8);
            movements += 2;
            break;

        case 'q': // Quit
            puts_USART1("Exiting pan-tilt control\\r\\n");
            break;

        default:
            puts_USART1("Invalid command\\r\\n");
            break;
        }

        _delay_ms(100);
    }

    // Demo sequence: Automatic scanning pattern
    puts_USART1("\\r\\nDemo: Automatic scanning pattern\\r\\n");
    lcd_string(3, 0, "Auto scanning");

    // Horizontal sweep
    for (uint8_t scan = 0; scan < 3; scan++)
    {
        move_servo_smooth(1, 30, 8);  // Pan left
        move_servo_smooth(1, 150, 8); // Pan right
        move_servo_smooth(1, 90, 8);  // Return center

        char scan_msg[20];
        sprintf(scan_msg, "Scan: %d/3", scan + 1);
        lcd_string(4, 0, scan_msg);
    }

    // Vertical sweep
    move_servo_smooth(2, 60, 8);  // Tilt up
    move_servo_smooth(2, 120, 8); // Tilt down
    move_servo_smooth(2, 90, 8);  // Return center

    char final_msg[50];
    sprintf(final_msg, "Pan-tilt system complete! Movements: %d\\r\\n", movements);
    puts_USART1(final_msg);

    if (movements >= 10)
    {
        lab_score += 250;
        puts_USART1("✓ Servo applications mastered!\\r\\n");
    }
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
    puts_USART1("    SERVO MOTOR CONTROL - LAB EXERCISES      \\r\\n");
    puts_USART1("==============================================\\r\\n");
    puts_USART1("1. Single Servo Control & Calibration       \\r\\n");
    puts_USART1("2. Smooth Movements & Motion Profiles        \\r\\n");
    puts_USART1("3. Multi-Servo Coordination                 \\r\\n");
    puts_USART1("4. Servo Applications (Pan-Tilt System)     \\r\\n");
    puts_USART1("                                              \\r\\n");
    puts_USART1("0. Run All Exercises                         \\r\\n");
    puts_USART1("X. Exit Lab                                   \\r\\n");
    puts_USART1("==============================================\\r\\n");
    char score_msg[50];
    sprintf(score_msg, "Current Score: %d points\\r\\n", lab_score);
    puts_USART1(score_msg);
    char move_msg[50];
    sprintf(move_msg, "Servo Movements: %d\\r\\n", servo_movements);
    puts_USART1(move_msg);
    puts_USART1("Select exercise (1-4, 0, X): ");
}

int main(void)
{
    init_devices();

    puts_USART1("\\r\\n*** SERVO MOTOR CONTROL LAB SESSION ***\\r\\n");
    puts_USART1("Welcome to hands-on servo control programming!\\r\\n");
    puts_USART1("SAFETY: Ensure servos have adequate power supply!\\r\\n");
    puts_USART1("Connections: PWM pins to servo signal wires\\r\\n");

    lcd_clear();
    lcd_string(1, 0, "SERVO CONTROL LAB");
    lcd_string(2, 0, "Check servo power");
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
            lab_ex1_servo_calibration();
            lab_ex1_manual_control();
            break;

        case '2':
            lab_ex2_smooth_movements();
            break;

        case '3':
            lab_ex3_multi_servo_coordination();
            break;

        case '4':
            lab_ex4_servo_applications();
            break;

        case '0':
            puts_USART1("\\r\\n*** RUNNING ALL EXERCISES ***\\r\\n");
            lab_ex1_servo_calibration();
            lab_ex1_manual_control();
            lab_ex2_smooth_movements();
            lab_ex3_multi_servo_coordination();
            lab_ex4_servo_applications();

            char final_buffer[80];
            sprintf(final_buffer, "\\r\\n*** ALL EXERCISES COMPLETE! ***\\r\\nFinal Score: %d points\\r\\n", lab_score);
            puts_USART1(final_buffer);
            break;

        case 'X':
        case 'x':
            puts_USART1("\\r\\nExiting lab. Great work on servo control!\\r\\n");
            puts_USART1("Remember: Always center servos before power off!\\r\\n");
            lcd_clear();
            lcd_string(2, 0, "LAB COMPLETE!");
            char exit_score[30];
            sprintf(exit_score, "Score: %d pts", lab_score);
            lcd_string(3, 0, exit_score);
            // Center all servos before exit
            set_servo_angle(1, 90);
            set_servo_angle(2, 90);
            set_servo_angle(3, 90);
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