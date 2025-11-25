/*
 * =============================================================================
 * STEPPER MOTOR CONTROL - HANDS-ON LAB EXERCISES
 * =============================================================================
 * ATmega128 Educational Framework - Lab Session
 *
 * OBJECTIVE: Master stepper motor control and precision positioning
 * DURATION: 85 minutes
 * DIFFICULTY: Advanced
 *
 * STUDENTS WILL:
 * - Control stepper motors with different drive sequences
 * - Implement precise positioning and speed control
 * - Create acceleration profiles and smooth motion
 * - Build multi-axis stepper coordination systems
 * - Debug stepper motor timing and torque issues
 *
 * HARDWARE REQUIRED:
 * - ATmega128 board
 * - Bipolar stepper motor (NEMA 17 or similar)
 * - Stepper driver (A4988, DRV8825, or L298N)
 * - Step/Direction control pins or 4-wire direct control
 * - Position encoder or limit switches (optional)
 * - 4 control buttons and LCD display
 *
 * STEPPER CONTROL METHODS:
 * - Full step: Maximum torque, 200 steps/revolution
 * - Half step: Smoother motion, 400 steps/revolution
 * - Microstepping: Ultra-smooth, 1600+ steps/revolution
 * - Direction control with step pulses
 *
 * LAB STRUCTURE:
 * - Exercise 1: Basic stepper control and step sequences (25 min)
 * - Exercise 2: Speed control and acceleration profiles (20 min)
 * - Exercise 3: Precision positioning and homing (25 min)
 * - Exercise 4: Advanced stepper applications (15 min)
 *
 * =============================================================================
 */

#include "config.h"

// Stepper motor control pins
#define STEP_PIN 0   // PC0 - Step pulse
#define DIR_PIN 1    // PC1 - Direction control
#define ENABLE_PIN 2 // PC2 - Motor enable (low = enabled)

// Alternative: Direct 4-wire control pins (for unipolar steppers)
#define COIL_A1 4 // PC4 - Coil A, wire 1
#define COIL_A2 5 // PC5 - Coil A, wire 2
#define COIL_B1 6 // PC6 - Coil B, wire 1
#define COIL_B2 7 // PC7 - Coil B, wire 2

// Stepper motor constants
#define STEPS_PER_REV 200 // Standard stepper motor
#define MICROSTEPS 16     // Driver microstepping setting
#define FULL_REVOLUTION (STEPS_PER_REV * MICROSTEPS)

// Speed and timing constants
#define MIN_STEP_DELAY 500   // Microseconds (fastest speed)
#define MAX_STEP_DELAY 10000 // Microseconds (slowest speed)
#define ACCEL_STEPS 50       // Steps for acceleration/deceleration

// Lab session variables
uint16_t lab_score = 0;
uint32_t total_steps = 0;
int32_t current_position = 0;  // Current position in steps
uint8_t current_direction = 0; // 0 = CW, 1 = CCW
uint16_t current_speed = 2000; // Current step delay in microseconds

/*
 * =============================================================================
 * STEPPER CONTROL FUNCTIONS
 * =============================================================================
 */

void stepper_pins_init(void)
{
    // Set control pins as outputs
    DDRC |= (1 << STEP_PIN) | (1 << DIR_PIN) | (1 << ENABLE_PIN);

    // Set 4-wire control pins as outputs (for alternative control)
    DDRC |= (1 << COIL_A1) | (1 << COIL_A2) | (1 << COIL_B1) | (1 << COIL_B2);

    // Initialize pins
    PORTC &= ~(1 << STEP_PIN);   // Step pin low
    PORTC &= ~(1 << DIR_PIN);    // Direction CW
    PORTC &= ~(1 << ENABLE_PIN); // Enable motor (active low)

    // Turn off all coils initially
    PORTC &= ~((1 << COIL_A1) | (1 << COIL_A2) | (1 << COIL_B1) | (1 << COIL_B2));
}

void stepper_set_direction(uint8_t direction)
{
    current_direction = direction;
    if (direction)
    {
        PORTC |= (1 << DIR_PIN); // CCW
    }
    else
    {
        PORTC &= ~(1 << DIR_PIN); // CW
    }
    _delay_us(10); // Direction setup time
}

void stepper_single_step(void)
{
    // Generate step pulse
    PORTC |= (1 << STEP_PIN);
    _delay_us(5); // Minimum pulse width
    PORTC &= ~(1 << STEP_PIN);

    // Update position
    if (current_direction)
    {
        current_position--;
    }
    else
    {
        current_position++;
    }

    total_steps++;
}

void stepper_move_steps(uint16_t steps, uint16_t step_delay)
{
    for (uint16_t i = 0; i < steps; i++)
    {
        stepper_single_step();
        _delay_us(step_delay);
    }
}

// Full step sequence for 4-wire control (alternative method)
const uint8_t full_step_sequence[4] = {
    (1 << COIL_A1), // Step 1: A1
    (1 << COIL_B1), // Step 2: B1
    (1 << COIL_A2), // Step 3: A2
    (1 << COIL_B2)  // Step 4: B2
};

// Half step sequence for smoother motion
const uint8_t half_step_sequence[8] = {
    (1 << COIL_A1),                  // Step 1
    (1 << COIL_A1) | (1 << COIL_B1), // Step 2
    (1 << COIL_B1),                  // Step 3
    (1 << COIL_B1) | (1 << COIL_A2), // Step 4
    (1 << COIL_A2),                  // Step 5
    (1 << COIL_A2) | (1 << COIL_B2), // Step 6
    (1 << COIL_B2),                  // Step 7
    (1 << COIL_B2) | (1 << COIL_A1)  // Step 8
};

void stepper_4wire_step(uint8_t step_num, uint8_t half_step_mode)
{
    // Clear all coils first
    PORTC &= ~((1 << COIL_A1) | (1 << COIL_A2) | (1 << COIL_B1) | (1 << COIL_B2));

    if (half_step_mode)
    {
        // Half step mode (8 steps per cycle)
        PORTC |= half_step_sequence[step_num % 8];
    }
    else
    {
        // Full step mode (4 steps per cycle)
        PORTC |= full_step_sequence[step_num % 4];
    }
}

/*
 * =============================================================================
 * MOTION CONTROL FUNCTIONS
 * =============================================================================
 */

void stepper_move_with_accel(uint16_t target_steps, uint16_t max_speed)
{
    uint16_t accel_steps = (target_steps < ACCEL_STEPS * 2) ? target_steps / 2 : ACCEL_STEPS;

    // Acceleration phase
    for (uint16_t i = 0; i < accel_steps; i++)
    {
        uint16_t speed = MAX_STEP_DELAY - (i * (MAX_STEP_DELAY - max_speed)) / accel_steps;
        stepper_single_step();
        _delay_us(speed);
    }

    // Constant speed phase
    uint16_t const_steps = target_steps - (2 * accel_steps);
    for (uint16_t i = 0; i < const_steps; i++)
    {
        stepper_single_step();
        _delay_us(max_speed);
    }

    // Deceleration phase
    for (uint16_t i = 0; i < accel_steps; i++)
    {
        uint16_t speed = max_speed + (i * (MAX_STEP_DELAY - max_speed)) / accel_steps;
        stepper_single_step();
        _delay_us(speed);
    }
}

void stepper_goto_position(int32_t target_position)
{
    int32_t steps_to_move = target_position - current_position;

    if (steps_to_move == 0)
        return;

    // Set direction
    stepper_set_direction(steps_to_move < 0 ? 1 : 0);

    // Move with acceleration
    stepper_move_with_accel(abs(steps_to_move), current_speed);
}

/*
 * =============================================================================
 * LAB EXERCISE 1: BASIC STEPPER CONTROL (25 minutes)
 * =============================================================================
 * OBJECTIVE: Learn stepper motor fundamentals and basic control
 * DIFFICULTY: ★★☆☆☆ (Easy-Medium)
 */

void lab_ex1_basic_stepping(void)
{
    /*
     * CHALLENGE: Control stepper motor with different step sequences
     * TASK: Implement full step and half step control modes
     * LEARNING: Step sequences, timing, motor mechanics
     */

    puts_USART1("\\r\\n=== Lab 1: Basic Stepper Control ===\\r\\n");
    puts_USART1("Testing stepper motor step sequences\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "STEPPER CONTROL");
    lcd_string(1, 0, "Basic stepping");

    stepper_pins_init();

    // Test 1: Step/Direction control method
    puts_USART1("Test 1: Step/Direction control\\r\\n");
    lcd_string(3, 0, "Step/Dir mode");

    stepper_set_direction(0); // Clockwise

    // Rotate clockwise
    puts_USART1("Rotating clockwise (200 steps)...\\r\\n");
    stepper_move_steps(200, 5000); // One full revolution at slow speed
    _delay_ms(1000);

    // Rotate counter-clockwise
    stepper_set_direction(1); // Counter-clockwise
    puts_USART1("Rotating counter-clockwise (200 steps)...\\r\\n");
    stepper_move_steps(200, 5000);
    _delay_ms(1000);

    // Test 2: 4-wire direct control method
    puts_USART1("\\r\\nTest 2: 4-wire direct control\\r\\n");
    lcd_string(3, 0, "4-wire mode");

    // Disable step/dir driver for 4-wire test
    PORTC |= (1 << ENABLE_PIN); // Disable driver

    // Full step sequence
    puts_USART1("Full step sequence (forward)...\\r\\n");
    for (uint16_t step = 0; step < 200; step++)
    {
        stepper_4wire_step(step, 0); // Full step mode

        char step_msg[20];
        sprintf(step_msg, "Step: %d", step % 4);
        lcd_string(4, 0, step_msg);

        _delay_ms(50);
    }

    _delay_ms(1000);

    // Half step sequence
    puts_USART1("Half step sequence (backward)...\\r\\n");
    for (int16_t step = 399; step >= 0; step--)
    {
        stepper_4wire_step(step, 1); // Half step mode

        char step_msg[20];
        sprintf(step_msg, "Step: %d", step % 8);
        lcd_string(4, 0, step_msg);

        _delay_ms(25);
    }

    // Turn off all coils
    PORTC &= ~((1 << COIL_A1) | (1 << COIL_A2) | (1 << COIL_B1) | (1 << COIL_B2));

    // Re-enable step/dir driver
    PORTC &= ~(1 << ENABLE_PIN);

    puts_USART1("Basic stepper control complete!\\r\\n");
    lab_score += 150;
}

/*
 * =============================================================================
 * LAB EXERCISE 2: SPEED CONTROL (20 minutes)
 * =============================================================================
 * OBJECTIVE: Master stepper motor speed and acceleration control
 * DIFFICULTY: ★★★☆☆ (Medium)
 */

void lab_ex2_speed_control(void)
{
    /*
     * CHALLENGE: Implement variable speed and acceleration profiles
     * TASK: Create smooth speed transitions and find maximum speed
     * LEARNING: Timing control, acceleration algorithms, motor limits
     */

    puts_USART1("\\r\\n=== Lab 2: Speed Control ===\\r\\n");
    puts_USART1("Testing different speeds and acceleration\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "SPEED CONTROL");
    lcd_string(1, 0, "Testing speeds");

    stepper_set_direction(0); // Clockwise

    // Test different constant speeds
    uint16_t test_speeds[] = {8000, 4000, 2000, 1000, 500}; // Microsecond delays
    const char *speed_names[] = {"Very Slow", "Slow", "Medium", "Fast", "Very Fast"};
    uint8_t num_speeds = sizeof(test_speeds) / sizeof(test_speeds[0]);

    for (uint8_t i = 0; i < num_speeds; i++)
    {
        char speed_msg[50];
        sprintf(speed_msg, "Speed test %d: %s (%d μs delay)\\r\\n",
                i + 1, speed_names[i], test_speeds[i]);
        puts_USART1(speed_msg);

        lcd_string(3, 0, speed_names[i]);

        char delay_msg[20];
        sprintf(delay_msg, "Delay: %d μs", test_speeds[i]);
        lcd_string(4, 0, delay_msg);

        // Rotate 90 degrees (50 steps) at this speed
        stepper_move_steps(50, test_speeds[i]);
        _delay_ms(1000);
    }

    // Return to start position
    stepper_set_direction(1);      // Counter-clockwise
    stepper_move_steps(250, 2000); // Return to approximately start position

    // Test acceleration profiles
    puts_USART1("\\r\\nTesting acceleration profiles...\\r\\n");
    lcd_string(3, 0, "Acceleration");

    stepper_set_direction(0); // Clockwise

    // Linear acceleration profile
    puts_USART1("Linear acceleration profile:\\r\\n");
    for (uint16_t step = 0; step < 200; step++)
    {
        // Calculate speed: start slow, get faster
        uint16_t speed = MAX_STEP_DELAY - (step * (MAX_STEP_DELAY - MIN_STEP_DELAY)) / 200;
        stepper_single_step();

        if (step % 20 == 0)
        { // Update display every 20 steps
            char accel_msg[20];
            sprintf(accel_msg, "Speed: %d μs", speed);
            lcd_string(4, 0, accel_msg);
        }

        _delay_us(speed);
    }

    _delay_ms(1000);

    // Test deceleration
    puts_USART1("Linear deceleration profile:\\r\\n");
    lcd_string(3, 0, "Deceleration");

    for (uint16_t step = 0; step < 200; step++)
    {
        // Calculate speed: start fast, get slower
        uint16_t speed = MIN_STEP_DELAY + (step * (MAX_STEP_DELAY - MIN_STEP_DELAY)) / 200;
        stepper_single_step();

        if (step % 20 == 0)
        {
            char decel_msg[20];
            sprintf(decel_msg, "Speed: %d μs", speed);
            lcd_string(4, 0, decel_msg);
        }

        _delay_us(speed);
    }

    // Test S-curve acceleration (smooth start and stop)
    puts_USART1("\\r\\nS-curve acceleration profile:\\r\\n");
    lcd_string(3, 0, "S-curve accel");

    for (uint16_t step = 0; step < 200; step++)
    {
        // S-curve profile: slow-fast-slow using sine function
        float t = (float)step / 200.0;               // Normalized time (0 to 1)
        float s_curve = 0.5 * (1.0 - cos(t * M_PI)); // Sine-based S-curve
        uint16_t speed = MAX_STEP_DELAY - (uint16_t)(s_curve * (MAX_STEP_DELAY - MIN_STEP_DELAY));

        stepper_single_step();
        _delay_us(speed);
    }

    puts_USART1("Speed control tests complete!\\r\\n");
    lab_score += 150;
}

/*
 * =============================================================================
 * LAB EXERCISE 3: PRECISION POSITIONING (25 minutes)
 * =============================================================================
 * OBJECTIVE: Implement precise positioning and homing functions
 * DIFFICULTY: ★★★★☆ (Medium-Hard)
 */

void lab_ex3_precision_positioning(void)
{
    /*
     * CHALLENGE: Implement absolute positioning and homing routines
     * TASK: Create precise movement to specific positions
     * LEARNING: Position tracking, coordinate systems, homing algorithms
     */

    puts_USART1("\\r\\n=== Lab 3: Precision Positioning ===\\r\\n");
    puts_USART1("Implementing absolute positioning system\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "PRECISION POS");
    lcd_string(1, 0, "Absolute control");

    // Reset position counter (simulate homing)
    current_position = 0;
    current_speed = 1500; // Medium speed for positioning

    puts_USART1("Homing sequence (resetting position to 0)...\\r\\n");
    lcd_string(3, 0, "Homing...");

    // Simulate homing: move to a reference position
    stepper_set_direction(1);      // Move CCW to "home"
    stepper_move_steps(100, 3000); // Slow movement to home
    current_position = 0;          // Reset position counter

    char pos_msg[30];
    sprintf(pos_msg, "Homed at position: %ld\\r\\n", current_position);
    puts_USART1(pos_msg);

    // Test absolute positioning
    int32_t test_positions[] = {400, -200, 800, 0, -400, 200}; // Various positions
    uint8_t num_positions = sizeof(test_positions) / sizeof(test_positions[0]);

    for (uint8_t i = 0; i < num_positions; i++)
    {
        int32_t target = test_positions[i];

        char move_msg[50];
        sprintf(move_msg, "Moving to position %ld (from %ld)\\r\\n", target, current_position);
        puts_USART1(move_msg);

        // Update LCD
        char lcd_msg[20];
        sprintf(lcd_msg, "Target: %ld", target);
        lcd_string(3, 0, lcd_msg);

        sprintf(lcd_msg, "Current: %ld", current_position);
        lcd_string(4, 0, lcd_msg);

        // Move to target position with acceleration
        stepper_goto_position(target);

        // Verify position
        sprintf(move_msg, "Reached position: %ld\\r\\n", current_position);
        puts_USART1(move_msg);

        sprintf(lcd_msg, "At: %ld", current_position);
        lcd_string(5, 0, lcd_msg);

        _delay_ms(1500); // Pause to observe position
    }

    // Test relative positioning
    puts_USART1("\\r\\nTesting relative positioning...\\r\\n");
    lcd_string(3, 0, "Relative moves");

    int16_t relative_moves[] = {100, -50, 150, -200, 75};
    uint8_t num_moves = sizeof(relative_moves) / sizeof(relative_moves[0]);

    for (uint8_t i = 0; i < num_moves; i++)
    {
        int32_t relative_move = relative_moves[i];
        int32_t target = current_position + relative_move;

        char rel_msg[50];
        sprintf(rel_msg, "Relative move: %+d steps\\r\\n", relative_move);
        puts_USART1(rel_msg);

        stepper_goto_position(target);

        char final_msg[40];
        sprintf(final_msg, "New position: %ld\\r\\n", current_position);
        puts_USART1(final_msg);

        char lcd_msg[20];
        sprintf(lcd_msg, "Pos: %ld", current_position);
        lcd_string(4, 0, lcd_msg);

        _delay_ms(1000);
    }

    // Return to home position
    puts_USART1("\\r\\nReturning to home position...\\r\\n");
    lcd_string(3, 0, "Going home");
    stepper_goto_position(0);

    sprintf(pos_msg, "Final position: %ld (should be 0)\\r\\n", current_position);
    puts_USART1(pos_msg);

    puts_USART1("Precision positioning complete!\\r\\n");
    lab_score += 200;
}

/*
 * =============================================================================
 * LAB EXERCISE 4: ADVANCED APPLICATIONS (15 minutes)
 * =============================================================================
 * OBJECTIVE: Build practical stepper motor applications
 * DIFFICULTY: ★★★★★ (Hard)
 */

void lab_ex4_stepper_applications(void)
{
    /*
     * CHALLENGE: Create a complete stepper-based positioning system
     * TASK: Build an interactive position control system
     * LEARNING: User interfaces, system integration, practical applications
     */

    puts_USART1("\\r\\n=== Lab 4: Stepper Applications ===\\r\\n");
    puts_USART1("Interactive positioning system\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "STEPPER APP");
    lcd_string(1, 0, "Interactive ctrl");

    puts_USART1("Commands:\\r\\n");
    puts_USART1("  + / -     : Move +/- 10 steps\\r\\n");
    puts_USART1("  f / s     : Fast/Slow speed\\r\\n");
    puts_USART1("  h         : Home (goto 0)\\r\\n");
    puts_USART1("  1,2,3,4   : Goto preset positions\\r\\n");
    puts_USART1("  q         : Quit application\\r\\n\\r\\n");

    // Preset positions for quick access
    int32_t presets[] = {0, 200, 400, -200}; // Home, Pos1, Pos2, Pos3

    char command = 0;
    uint8_t interactions = 0;

    while (command != 'q' && interactions < 30)
    {
        // Display current status
        char status[60];
        sprintf(status, "Position: %ld, Speed: %d μs, Steps: %ld\\r\\n",
                current_position, current_speed, total_steps);
        puts_USART1(status);

        char lcd_msg[20];
        sprintf(lcd_msg, "Pos: %ld", current_position);
        lcd_string(3, 0, lcd_msg);

        sprintf(lcd_msg, "Speed: %d", current_speed);
        lcd_string(4, 0, lcd_msg);

        puts_USART1("Command: ");
        command = getch_USART1();
        putch_USART1(command);
        puts_USART1("\\r\\n");

        interactions++;

        switch (command)
        {
        case '+':
            puts_USART1("Moving +10 steps\\r\\n");
            stepper_goto_position(current_position + 10);
            break;

        case '-':
            puts_USART1("Moving -10 steps\\r\\n");
            stepper_goto_position(current_position - 10);
            break;

        case 'f':
            current_speed = MIN_STEP_DELAY;
            puts_USART1("Speed set to FAST\\r\\n");
            break;

        case 's':
            current_speed = 3000;
            puts_USART1("Speed set to SLOW\\r\\n");
            break;

        case 'h':
            puts_USART1("Homing to position 0\\r\\n");
            lcd_string(5, 0, "Homing...");
            stepper_goto_position(0);
            lcd_string(5, 0, "Home    ");
            break;

        case '1':
            sprintf(status, "Moving to preset 1: %ld\\r\\n", presets[1]);
            puts_USART1(status);
            stepper_goto_position(presets[1]);
            break;

        case '2':
            sprintf(status, "Moving to preset 2: %ld\\r\\n", presets[2]);
            puts_USART1(status);
            stepper_goto_position(presets[2]);
            break;

        case '3':
            sprintf(status, "Moving to preset 3: %ld\\r\\n", presets[3]);
            puts_USART1(status);
            stepper_goto_position(presets[3]);
            break;

        case 'q':
            puts_USART1("Exiting stepper application\\r\\n");
            break;

        default:
            puts_USART1("Invalid command\\r\\n");
            break;
        }

        _delay_ms(100);
    }

    // Demo sequence: Automated pattern
    puts_USART1("\\r\\nDemo: Automated movement pattern\\r\\n");
    lcd_string(3, 0, "Auto demo");

    // Create a complex movement pattern
    for (uint8_t cycle = 0; cycle < 3; cycle++)
    {
        char cycle_msg[30];
        sprintf(cycle_msg, "Demo cycle %d/3\\r\\n", cycle + 1);
        puts_USART1(cycle_msg);

        sprintf(cycle_msg, "Cycle: %d", cycle + 1);
        lcd_string(4, 0, cycle_msg);

        // Pattern: Home → Pos1 → Pos2 → Pos3 → Home
        stepper_goto_position(0); // Home
        _delay_ms(500);
        stepper_goto_position(400); // Far position
        _delay_ms(500);
        stepper_goto_position(-300); // Opposite direction
        _delay_ms(500);
        stepper_goto_position(200); // Middle position
        _delay_ms(500);
    }

    // Final return to home
    stepper_goto_position(0);

    char final_msg[60];
    sprintf(final_msg, "Application complete! Interactions: %d, Total steps: %ld\\r\\n",
            interactions, total_steps);
    puts_USART1(final_msg);

    if (interactions >= 5)
    {
        lab_score += 250;
        puts_USART1("✓ Stepper applications mastered!\\r\\n");
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
    puts_USART1("   STEPPER MOTOR CONTROL - LAB EXERCISES     \\r\\n");
    puts_USART1("==============================================\\r\\n");
    puts_USART1("1. Basic Stepper Control & Step Sequences   \\r\\n");
    puts_USART1("2. Speed Control & Acceleration Profiles    \\r\\n");
    puts_USART1("3. Precision Positioning & Homing           \\r\\n");
    puts_USART1("4. Advanced Stepper Applications            \\r\\n");
    puts_USART1("                                              \\r\\n");
    puts_USART1("0. Run All Exercises                         \\r\\n");
    puts_USART1("X. Exit Lab                                   \\r\\n");
    puts_USART1("==============================================\\r\\n");
    char score_msg[50];
    sprintf(score_msg, "Current Score: %d points\\r\\n", lab_score);
    puts_USART1(score_msg);
    char step_msg[50];
    sprintf(step_msg, "Total Steps: %ld, Position: %ld\\r\\n", total_steps, current_position);
    puts_USART1(step_msg);
    puts_USART1("Select exercise (1-4, 0, X): ");
}

int main(void)
{
    init_devices();

    puts_USART1("\\r\\n*** STEPPER MOTOR CONTROL LAB SESSION ***\\r\\n");
    puts_USART1("Welcome to hands-on stepper motor programming!\\r\\n");
    puts_USART1("SAFETY: Ensure stepper driver is properly connected!\\r\\n");
    puts_USART1("Check: Step/Dir pins or 4-wire connections\\r\\n");

    lcd_clear();
    lcd_string(1, 0, "STEPPER LAB");
    lcd_string(2, 0, "Check driver wiring");
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
            lab_ex1_basic_stepping();
            break;

        case '2':
            lab_ex2_speed_control();
            break;

        case '3':
            lab_ex3_precision_positioning();
            break;

        case '4':
            lab_ex4_stepper_applications();
            break;

        case '0':
            puts_USART1("\\r\\n*** RUNNING ALL EXERCISES ***\\r\\n");
            lab_ex1_basic_stepping();
            lab_ex2_speed_control();
            lab_ex3_precision_positioning();
            lab_ex4_stepper_applications();

            char final_buffer[80];
            sprintf(final_buffer, "\\r\\n*** ALL EXERCISES COMPLETE! ***\\r\\nFinal Score: %d points\\r\\n", lab_score);
            puts_USART1(final_buffer);
            break;

        case 'X':
        case 'x':
            puts_USART1("\\r\\nExiting lab. Great work on stepper control!\\r\\n");
            puts_USART1("Remember: Turn off motor power when not in use!\\r\\n");
            lcd_clear();
            lcd_string(2, 0, "LAB COMPLETE!");
            char exit_score[30];
            sprintf(exit_score, "Score: %d pts", lab_score);
            lcd_string(3, 0, exit_score);
            // Disable motor
            PORTC |= (1 << ENABLE_PIN);
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