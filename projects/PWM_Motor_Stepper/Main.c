/*
 * PWM Stepper Motor Control
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Understand stepper motor operation (full-step, half-step, microstepping)
 * - Generate precise step sequences using port manipulation
 * - Control speed through step delay timing
 * - Practice position tracking and homing
 *
 * HARDWARE SETUP:
 * - Bipolar stepper motor (4-wire) with ULN2003/L298N driver
 * - Coil connections: PA0-PA3 (4-wire stepper)
 * - Alternative: Unipolar 28BYJ-48 with ULN2003 driver
 * - UART for control interface
 * - LEDs on PORTC for phase visualization
 *
 * DOCUMENTATION REFERENCE:
 * ATmega128 Datasheet: https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/2467S.pdf
 * - I/O Ports (pages 62-75)
 * - Timer/Counter (pages 77-107)
 *
 * =============================================================================
 * PORT AND TIMER REGISTERS - DETAILED REFERENCE FOR STUDENTS
 * =============================================================================
 *
 * REGISTER 1: DDRx (Data Direction Register) - PORT CONFIGURATION
 *
 *    Bit:   7      6      5      4      3      2      1      0
 *    Name:  DD7    DD6    DD5    DD4    DD3    DD2    DD1    DD0
 *
 * Each bit configures corresponding pin:
 *   0 = Input
 *   1 = Output (required for stepper coil control)
 *
 * For stepper on PORTA (PA0-PA3):
 *   DDRA = 0b00001111;  // PA0-PA3 outputs, PA4-PA7 inputs
 *   or: DDRA |= (1<<PA0)|(1<<PA1)|(1<<PA2)|(1<<PA3);
 *
 * REGISTER 2: PORTx (Port Data Register) - OUTPUT CONTROL
 *
 *    Bit:   7      6      5      4      3      2      1      0
 *    Name:  P7     P6     P5     P4     P3     P2     P1     P0
 *
 * Controls output state:
 *   0 = Low (0V)
 *   1 = High (VCC)
 *
 * Stepper Phase Pattern Examples:
 *   PORTA = 0b00000011;  // Coils A1, A2 ON
 *   PORTA = 0b00000110;  // Coils A2, B1 ON
 *   PORTA = 0b00001100;  // Coils B1, B2 ON
 *   PORTA = 0b00001001;  // Coils B2, A1 ON
 *
 * TIMER REGISTERS FOR STEP TIMING:
 *
 * REGISTER 3: TCCR1B (Timer1 Control Register B) - SPEED CONTROL
 *
 *    Bit:   7      6      5      4      3      2      1      0
 *    Name: ICNC1  ICES1   -    WGM13  WGM12   CS12   CS11   CS10
 *
 * CS12:10: Prescaler for step delay timing
 *          Use CTC mode (WGM12=1) with OCR1A for precise step intervals
 *
 * REGISTER 4: OCR1A (Output Compare Register A) - STEP RATE
 *
 * CTC mode step timing:
 *   Step_Rate = F_CPU / (Prescaler × (1 + OCR1A))
 *
 * Examples @ 16MHz, Prescaler 64:
 *   OCR1A = 249:  Step rate = 16MHz/(64×250) = 1kHz (1ms per step)
 *   OCR1A = 999:  Step rate = 250Hz (4ms per step, smoother)
 *   OCR1A = 3999: Step rate = 62.5Hz (16ms per step, very slow)
 *
 * RPM Calculation:
 *   RPM = (Step_Rate × 60) / Steps_Per_Revolution
 *   Example: 250Hz step rate, 200 steps/rev
 *   RPM = (250 × 60) / 200 = 75 RPM
 *
 * STEPPER CONTROL SEQUENCES:
 *
 * FULL-STEP (4 steps, high torque):
 *   const uint8_t full_step[4] = {
 *       0b0011,  // Phase 1: A1+A2
 *       0b0110,  // Phase 2: A2+B1
 *       0b1100,  // Phase 3: B1+B2
 *       0b1001   // Phase 4: B2+A1
 *   };
 *
 * HALF-STEP (8 steps, smoother):
 *   const uint8_t half_step[8] = {
 *       0b0001,  // A1
 *       0b0011,  // A1+A2
 *       0b0010,  // A2
 *       0b0110,  // A2+B1
 *       0b0100,  // B1
 *       0b1100,  // B1+B2
 *       0b1000,  // B2
 *       0b1001   // B2+A1
 *   };
 *
 * STEPPER CONTROL FUNCTIONS:
 *
 *   Initialize:
 *     void stepper_init(void) {
 *         DDRA |= 0x0F;       // PA0-PA3 outputs
 *         PORTA = 0x00;       // All coils off
 *
 *         // Timer1 CTC for step timing
 *         TCCR1B = (1<<WGM12) | (1<<CS11) | (1<<CS10);  // CTC, prescaler 64
 *         OCR1A = 999;        // 4ms per step
 *     }
 *
 *   Single Step (Full-Step):
 *     void step_once(int8_t direction) {
 *         static uint8_t phase = 0;
 *
 *         PORTA = (PORTA & 0xF0) | full_step[phase];
 *
 *         if(direction > 0) phase = (phase + 1) % 4;  // CW
 *         else phase = (phase + 3) % 4;  // CCW (phase - 1)
 *     }
 *
 *   Move N Steps:
 *     void move_steps(int16_t steps) {
 *         int8_t dir = (steps > 0) ? 1 : -1;
 *         uint16_t count = (steps > 0) ? steps : -steps;
 *
 *         for(uint16_t i = 0; i < count; i++) {
 *             step_once(dir);
 *             _delay_ms(4);  // 4ms per step
 *         }
 *     }
 *
 *   Rotate Angle:
 *     void rotate_angle(int16_t degrees) {
 *         int32_t steps = (int32_t)degrees * STEPS_PER_REV / 360;
 *         move_steps(steps);
 *     }
 *
 * MICROSTEPPING (Advanced, requires PWM):
 *
 * Instead of binary ON/OFF, use PWM for current control:
 *   Phase A current = Imax × sin(θ)
 *   Phase B current = Imax × cos(θ)
 *
 * This requires:
 *   - Timer1 PWM on OC1A, OC1B for current control
 *   - Lookup table or real-time sine calculation
 *   - H-bridge drivers supporting PWM input
 *
 *   Example @ 16 microsteps:
 *     for(uint8_t i = 0; i < 16; i++) {
 *         OCR1A = sin_table[i];       // Phase A
 *         OCR1B = cos_table[i];       // Phase B
 *         _delay_ms(1);
 *     }
 *
 * ACCELERATION PROFILE (S-Curve):
 *
 *   void move_with_accel(int16_t target_steps) {
 *       uint16_t accel_steps = 50;  // Acceleration zone
 *       for(int16_t i = 0; i < target_steps; i++) {
 *           step_once(1);
 *
 *           // Variable delay for acceleration
 *           if(i < accel_steps) {
 *               _delay_ms(10 - i/5);  // Accelerate
 *           } else if(i > target_steps - accel_steps) {
 *               _delay_ms(10 - (target_steps-i)/5);  // Decelerate
 *           } else {
 *               _delay_ms(2);  // Constant speed
 *           }
 *       }
 *   }
 *
 * CRITICAL STEPPER NOTES:
 * 1. Never leave coils energized too long (overheating)
 * 2. Use current-limiting resistors or chopper drivers
 * 3. Maximum step rate limited by motor inductance (~1kHz typical)
 * 4. Half-stepping reduces torque but increases smoothness
 * 5. Acceleration needed for high-speed moves to prevent skipping
 *
 * =============================================================================
 *
 * STEPPER MOTOR CONCEPTS:
 * - Steps per Revolution: Typically 200 (1.8°) or 48 (7.5°)
 * - Full-Step: 4 steps per cycle, maximum torque
 * - Half-Step: 8 steps per cycle, smoother motion
 * - Microstepping: Finer resolution, requires PWM
 * - Holding Torque: Torque when stationary
 */

#include "config.h"

// Stepper motor control pins (Port A)
#define STEPPER_PORT PORTA
#define STEPPER_DDR DDRA
#define COIL_A1 (1 << PA0)
#define COIL_A2 (1 << PA1)
#define COIL_B1 (1 << PA2)
#define COIL_B2 (1 << PA3)

// Stepper motor specifications
#define STEPS_PER_REV 200  // Standard 1.8° stepper
#define GEAR_RATIO 1       // No gearbox
#define STEPS_FULL_CYCLE 4 // Full-step sequence length
#define STEPS_HALF_CYCLE 8 // Half-step sequence length

// Full-step sequence (high torque, 4 steps per cycle)
// Wave drive (one phase on): A, B, A', B'
const uint8_t full_step_sequence_wave[4] = {
    0b0001, // Coil A1
    0b0010, // Coil A2
    0b0100, // Coil B1
    0b1000  // Coil B2
};

// Full-step sequence (two-phase on, higher torque)
const uint8_t full_step_sequence[4] = {
    0b0011, // A1 + A2
    0b0110, // A2 + B1
    0b1100, // B1 + B2
    0b1001  // B2 + A1
};

// Half-step sequence (smoother, 8 steps per cycle)
const uint8_t half_step_sequence[8] = {
    0b0001, // A1
    0b0011, // A1 + A2
    0b0010, // A2
    0b0110, // A2 + B1
    0b0100, // B1
    0b1100, // B1 + B2
    0b1000, // B2
    0b1001  // B2 + A1
};

// Global state variables
volatile int32_t current_position = 0; // Current position in steps
volatile uint8_t current_step_index = 0;
volatile uint8_t stepping_mode = 0; // 0=full, 1=half

/*
 * Initialize stepper motor control
 */
void stepper_init(void)
{
    // Set Port A as output
    STEPPER_DDR = 0xFF;
    STEPPER_PORT = 0x00;

    // Initialize position
    current_position = 0;
    current_step_index = 0;
    stepping_mode = 0;
}

/*
 * Set coil state directly
 */
void stepper_set_coils(uint8_t coil_pattern)
{
    STEPPER_PORT = coil_pattern & 0x0F;

    // Visualize on PORTC LEDs
    PORTC = coil_pattern;
}

/*
 * Release all coils (power off)
 */
void stepper_release(void)
{
    STEPPER_PORT = 0x00;
    PORTC = 0x00;
}

/*
 * Step forward one step
 */
void stepper_step_forward(void)
{
    if (stepping_mode == 0)
    {
        // Full-step mode
        current_step_index = (current_step_index + 1) % STEPS_FULL_CYCLE;
        stepper_set_coils(full_step_sequence[current_step_index]);
    }
    else
    {
        // Half-step mode
        current_step_index = (current_step_index + 1) % STEPS_HALF_CYCLE;
        stepper_set_coils(half_step_sequence[current_step_index]);
    }
    current_position++;
}

/*
 * Step backward one step
 */
void stepper_step_backward(void)
{
    if (stepping_mode == 0)
    {
        // Full-step mode
        if (current_step_index == 0)
        {
            current_step_index = STEPS_FULL_CYCLE - 1;
        }
        else
        {
            current_step_index--;
        }
        stepper_set_coils(full_step_sequence[current_step_index]);
    }
    else
    {
        // Half-step mode
        if (current_step_index == 0)
        {
            current_step_index = STEPS_HALF_CYCLE - 1;
        }
        else
        {
            current_step_index--;
        }
        stepper_set_coils(half_step_sequence[current_step_index]);
    }
    current_position--;
}

/*
 * Move a specified number of steps
 * Negative for reverse direction
 */
void stepper_move_steps(int32_t steps, uint16_t delay_ms)
{
    int32_t steps_to_move = abs(steps);

    for (int32_t i = 0; i < steps_to_move; i++)
    {
        if (steps > 0)
        {
            stepper_step_forward();
        }
        else
        {
            stepper_step_backward();
        }
        // Variable delay using busy wait
        for (uint16_t d = 0; d < delay_ms; d++)
        {
            _delay_ms(1);
        }
    }
}

/*
 * Rotate by degrees
 */
void stepper_rotate_degrees(int16_t degrees, uint16_t delay_ms)
{
    // Calculate steps needed
    int32_t steps = ((int32_t)degrees * STEPS_PER_REV) / 360;

    if (stepping_mode == 1)
    {
        steps *= 2; // Half-step mode has double the resolution
    }

    stepper_move_steps(steps, delay_ms);
}

/*
 * Set stepper mode
 */
void stepper_set_mode(uint8_t mode)
{
    stepping_mode = mode;
    current_step_index = 0;
}

/* ========================================================================
 * DEMO 1: Basic Stepping with UART Control
 * ======================================================================== */
void demo1_basic_stepping(void)
{
    puts_USART1("\r\n=== DEMO 1: Basic Stepping Control ===\r\n");
    puts_USART1("Commands:\r\n");
    puts_USART1("  +/-: Step forward/backward\r\n");
    puts_USART1("  f/r: Rotate forward/reverse 10 steps\r\n");
    puts_USART1("  m: Toggle mode (Full/Half step)\r\n");
    puts_USART1("  p: Show position\r\n");
    puts_USART1("  h: Home (reset position to 0)\r\n");
    puts_USART1("  s: Stop (release coils)\r\n");
    puts_USART1("  q: Return to menu\r\n\r\n");

    char buf[60];
    sprintf(buf, "Mode: %s  Position: 0\r\n",
            stepping_mode ? "Half-Step" : "Full-Step");
    puts_USART1(buf);

    while (1)
    {
        if (UCSR1A & (1 << RXC1))
        {
            char cmd = UDR1;

            switch (cmd)
            {
            case '+':
                stepper_step_forward();
                sprintf(buf, "→ Step forward  Pos: %ld\r\n", current_position);
                puts_USART1(buf);
                break;

            case '-':
                stepper_step_backward();
                sprintf(buf, "← Step backward  Pos: %ld\r\n", current_position);
                puts_USART1(buf);
                break;

            case 'f':
            case 'F':
                puts_USART1("Rotating forward 10 steps...\r\n");
                stepper_move_steps(10, 50);
                sprintf(buf, "Position: %ld\r\n", current_position);
                puts_USART1(buf);
                break;

            case 'r':
            case 'R':
                puts_USART1("Rotating reverse 10 steps...\r\n");
                stepper_move_steps(-10, 50);
                sprintf(buf, "Position: %ld\r\n", current_position);
                puts_USART1(buf);
                break;

            case 'm':
            case 'M':
                stepping_mode = !stepping_mode;
                sprintf(buf, "Mode changed to: %s\r\n",
                        stepping_mode ? "Half-Step" : "Full-Step");
                puts_USART1(buf);
                break;

            case 'p':
            case 'P':
                sprintf(buf, "Current Position: %ld steps\r\n", current_position);
                puts_USART1(buf);
                break;

            case 'h':
            case 'H':
                current_position = 0;
                puts_USART1("Position homed to 0\r\n");
                break;

            case 's':
            case 'S':
                stepper_release();
                puts_USART1("Coils released (motor free)\r\n");
                break;

            case 'q':
            case 'Q':
                stepper_release();
                return;
            }
        }
    }
}

/* ========================================================================
 * DEMO 2: Continuous Rotation Test
 * ======================================================================== */
void demo2_continuous_rotation(void)
{
    puts_USART1("\r\n=== DEMO 2: Continuous Rotation ===\r\n");
    puts_USART1("Testing continuous rotation at different speeds\r\n");
    puts_USART1("Press any key to stop and return to menu\r\n\r\n");

    uint16_t speeds_ms[] = {20, 10, 5, 2, 1};
    char *speed_names[] = {"Slow", "Medium", "Fast", "Very Fast", "Maximum"};
    uint8_t num_speeds = sizeof(speeds_ms) / sizeof(speeds_ms[0]);

    for (uint8_t i = 0; i < num_speeds; i++)
    {
        char buf[60];
        sprintf(buf, "Speed: %s (%u ms/step)\r\n", speed_names[i], speeds_ms[i]);
        puts_USART1(buf);
        puts_USART1("Rotating one full revolution...\r\n");

        for (uint16_t step = 0; step < STEPS_PER_REV; step++)
        {
            stepper_step_forward();
            for (uint16_t d = 0; d < speeds_ms[i]; d++)
            {
                _delay_ms(1);
            }

            if (UCSR1A & (1 << RXC1))
            {
                getch_USART1();
                stepper_release();
                return;
            }
        }

        puts_USART1("Complete!\r\n\r\n");
        _delay_ms(1000);
    }

    stepper_release();
}

/* ========================================================================
 * DEMO 3: Position Control - Move to Specific Angles
 * ======================================================================== */
void demo3_position_control(void)
{
    puts_USART1("\r\n=== DEMO 3: Position Control ===\r\n");
    puts_USART1("Moving to specific angles\r\n");
    puts_USART1("Press any key to stop and return to menu\r\n\r\n");

    int16_t target_angles[] = {0, 90, 180, 270, 360, 270, 180, 90, 0, -90, 0};
    uint8_t num_positions = sizeof(target_angles) / sizeof(target_angles[0]);

    current_position = 0; // Home position

    for (uint8_t i = 0; i < num_positions; i++)
    {
        char buf[60];
        sprintf(buf, "Moving to %d°...\r\n", target_angles[i]);
        puts_USART1(buf);

        stepper_rotate_degrees(target_angles[i], 5);

        sprintf(buf, "Position: %ld steps (%d°)\r\n",
                current_position,
                (int16_t)((current_position * 360L) / STEPS_PER_REV));
        puts_USART1(buf);

        _delay_ms(1000);

        if (UCSR1A & (1 << RXC1))
        {
            getch_USART1();
            stepper_release();
            return;
        }
    }

    stepper_release();
    puts_USART1("\r\nPosition control demo complete!\r\n");
}

/* ========================================================================
 * DEMO 4: Full-Step vs Half-Step Comparison
 * ======================================================================== */
void demo4_mode_comparison(void)
{
    puts_USART1("\r\n=== DEMO 4: Stepping Mode Comparison ===\r\n");
    puts_USART1("Comparing Full-Step and Half-Step modes\r\n");
    puts_USART1("Press any key to stop and return to menu\r\n\r\n");

    // Full-step test
    puts_USART1("--- FULL-STEP MODE ---\r\n");
    puts_USART1("One complete revolution (200 steps)\r\n");
    stepper_set_mode(0);
    current_position = 0;

    for (uint16_t i = 0; i < STEPS_PER_REV; i++)
    {
        stepper_step_forward();
        _delay_ms(10);

        if (i % 50 == 0)
        {
            char buf[40];
            sprintf(buf, "Progress: %u/%u steps\r\n", i, STEPS_PER_REV);
            puts_USART1(buf);
        }

        if (UCSR1A & (1 << RXC1))
        {
            getch_USART1();
            stepper_release();
            return;
        }
    }

    puts_USART1("Full-step complete!\r\n\r\n");
    _delay_ms(2000);

    // Half-step test
    puts_USART1("--- HALF-STEP MODE ---\r\n");
    puts_USART1("One complete revolution (400 steps)\r\n");
    stepper_set_mode(1);
    current_position = 0;

    for (uint16_t i = 0; i < STEPS_PER_REV * 2; i++)
    {
        stepper_step_forward();
        _delay_ms(5);

        if (i % 100 == 0)
        {
            char buf[40];
            sprintf(buf, "Progress: %u/%u steps\r\n", i, STEPS_PER_REV * 2);
            puts_USART1(buf);
        }

        if (UCSR1A & (1 << RXC1))
        {
            getch_USART1();
            stepper_release();
            return;
        }
    }

    puts_USART1("Half-step complete!\r\n");
    puts_USART1("\r\nComparison:\r\n");
    puts_USART1("  Full-Step: Higher torque, faster, audible steps\r\n");
    puts_USART1("  Half-Step: Smoother motion, finer resolution, quieter\r\n");

    stepper_release();
}

/* ========================================================================
 * Main Menu System
 * ======================================================================== */
void display_main_menu(void)
{
    puts_USART1("\r\n\r\n");
    puts_USART1("╔════════════════════════════════════════╗\r\n");
    puts_USART1("║  STEPPER MOTOR CONTROL - ATmega128    ║\r\n");
    puts_USART1("╚════════════════════════════════════════╝\r\n");
    puts_USART1("\r\n");
    puts_USART1("Select Demo:\r\n");
    puts_USART1("  [1] Basic Stepping Control\r\n");
    puts_USART1("  [2] Continuous Rotation Test\r\n");
    puts_USART1("  [3] Position Control (Angles)\r\n");
    puts_USART1("  [4] Full vs Half-Step Comparison\r\n");
    puts_USART1("\r\n");
    puts_USART1("Enter selection (1-4): ");
}

int main(void)
{
    // Initialize peripherals
    // init_devices();  // Manual initialization below
    Uart1_init();
    stepper_init();

    // Configure status LEDs
    DDRC = 0xFF;
    PORTC = 0x00;

    // Send startup message
    _delay_ms(500);
    puts_USART1("\r\n\r\n*** Stepper Motor Control System ***\r\n");
    puts_USART1("ATmega128 Stepper Controller\r\n");
    char buf[60];
    sprintf(buf, "Motor: %d steps/rev, %s mode\r\n",
            STEPS_PER_REV, stepping_mode ? "Half-Step" : "Full-Step");
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
            demo1_basic_stepping();
            break;
        case '2':
            demo2_continuous_rotation();
            break;
        case '3':
            demo3_position_control();
            break;
        case '4':
            demo4_mode_comparison();
            break;
        default:
            puts_USART1("Invalid selection!\r\n");
            _delay_ms(1000);
            break;
        }

        // Ensure motor is released between demos
        stepper_release();
        current_position = 0;
        _delay_ms(500);
    }

    return 0;
}
