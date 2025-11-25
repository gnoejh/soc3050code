/*
 * =============================================================================
 * PWM DC MOTOR CONTROL - PURE PWM DEMO CODE
 * =============================================================================
 * PROJECT: PWM_Motor_DC
 * See Slide.md for complete lecture content and theory
 *
 * DEMOS (Pure PWM - No UART/ADC):
 * 1. Basic PWM Test - Fixed duty cycle PWM generation
 * 2. Speed Ramp - Smooth acceleration/deceleration
 * 3. Direction Control - Forward/Reverse with H-bridge
 * 4. Speed Sweep - Automatic speed variation
 * 5. PWM Duty Cycle Test - Multiple duty levels
 * 6. Random Rotation - Random speed and direction changes
 * =============================================================================
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>

// Hardware mapping
// PB5 (OC1A): PWM output to motor driver - MUST BE CONNECTED IN CIRCUIT!
// PB6: Direction control 1 (IN1)
// PB7: Direction control 2 (IN2)

#define MOTOR_OC1A_PIN (1 << PB5)
#define MOTOR_DIR1_PIN (1 << PB6)
#define MOTOR_DIR2_PIN (1 << PB7)

/*
 * =============================================================================
 * MOTOR COLOR INDICATOR (SimulIDE Visual Feedback)
 * =============================================================================
 * The DC motor component in SimulIDE displays a colored pie chart showing
 * the motor's operating state in real-time:
 *
 * GREEN SEGMENT:
 *   - Motor running efficiently with normal load
 *   - Moderate current draw
 *   - Optimal operating condition
 *   - Indicates healthy motor operation
 *
 * YELLOW/ORANGE SEGMENT:
 *   - Medium load condition
 *   - Motor working harder than normal
 *   - Increased current consumption
 *   - May indicate mechanical resistance
 *
 * RED SEGMENT:
 *   - Heavy load or near-stall condition
 *   - Very high current draw
 *   - Motor struggling to overcome load
 *   - Risk of overheating
 *   - When fully red: Motor stalled (blocked rotor)
 *
 * HOW TO INTERPRET:
 *   All green          → Light/no load, motor spinning freely
 *   Green + Orange     → Normal working condition, moderate load
 *   Mostly red         → Motor overloaded, may stall
 *   Static red (100%)  → Motor completely stalled, maximum current
 *
 * The color indicator helps visualize:
 *   1. Effect of PWM duty cycle on motor load
 *   2. Starting torque requirements (need high duty initially)
 *   3. Load variations during operation
 *   4. Difference between forward and reverse directions
 *
 * NOTE: Colors change based on:
 *   - PWM duty cycle (OCR1A value)
 *   - Mechanical load on motor shaft
 *   - Motor speed (RPM)
 *   - Current draw from power supply
 * =============================================================================
 */

/*
 * =============================================================================
 * TIMER1 PWM REGISTER REFERENCE - FAST PWM MODE 14 (TOP = ICR1)
 * =============================================================================
 *
 * TCCR1A - Timer/Counter1 Control Register A
 * +-----+-----+-----+-----+-----+-----+-----+-----+
 * | COM1A1 COM1A0 COM1B1 COM1B0 COM1C1 COM1C0 WGM11  WGM10 |
 * |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
 * +-----+-----+-----+-----+-----+-----+-----+-----+
 * OUR SETTING: 0xA2 = 1010 0010
 *              COM1A1=1, COM1A0=0 (non-inverting PWM on OC1A)
 *              WGM11=1 (part of mode 14)
 *
 * TCCR1B - Timer/Counter1 Control Register B
 * +-----+-----+-----+-----+-----+-----+-----+-----+
 * | ICNC1 ICES1  —   WGM13 WGM12  CS12  CS11  CS10|
 * |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
 * +-----+-----+-----+-----+-----+-----+-----+-----+
 * OUR SETTING: 0x1A = 0001 1010
 *              WGM13=1, WGM12=1 (mode 14: Fast PWM, TOP=ICR1)
 *              CS11=1 (prescaler = 8)
 *              ICNC1=0, ICES1=0 (not used in PWM mode)
 *
 * -----------------------------------------------------------------------------
 * WGM (WAVEFORM GENERATION MODE) - Complete Table
 * -----------------------------------------------------------------------------
 * Combines WGM13:12 (TCCR1B) + WGM11:10 (TCCR1A)
 *
 * Mode | WGM13 WGM12 WGM11 WGM10 | Description           | TOP    | Update | TOV1
 * -----|-------------------------|-----------------------|--------|--------|------
 *   0  |   0     0     0     0   | Normal                | 0xFFFF | Immed  | MAX
 *   1  |   0     0     0     1   | PWM Phase Correct 8   | 0x00FF | TOP    | BOTTOM
 *   2  |   0     0     1     0   | PWM Phase Correct 9   | 0x01FF | TOP    | BOTTOM
 *   3  |   0     0     1     1   | PWM Phase Correct 10  | 0x03FF | TOP    | BOTTOM
 *   4  |   0     1     0     0   | CTC                   | OCR1A  | Immed  | MAX
 *   5  |   0     1     0     1   | Fast PWM 8-bit        | 0x00FF | BOTTOM | TOP
 *   6  |   0     1     1     0   | Fast PWM 9-bit        | 0x01FF | BOTTOM | TOP
 *   7  |   0     1     1     1   | Fast PWM 10-bit       | 0x03FF | BOTTOM | TOP
 *   8  |   1     0     0     0   | PWM Phase Freq Corr   | ICR1   | BOTTOM | BOTTOM
 *   9  |   1     0     0     1   | PWM Phase Freq Corr   | OCR1A  | BOTTOM | BOTTOM
 *  10  |   1     0     1     0   | PWM Phase Correct     | ICR1   | TOP    | BOTTOM
 *  11  |   1     0     1     1   | PWM Phase Correct     | OCR1A  | TOP    | BOTTOM
 *  12  |   1     1     0     0   | CTC                   | ICR1   | Immed  | MAX
 *  13  |   1     1     0     1   | Reserved              | —      | —      | —
 * ★14★ |   1     1     1     0   | Fast PWM              | ICR1   | BOTTOM | TOP
 *  15  |   1     1     1     1   | Fast PWM              | OCR1A  | BOTTOM | TOP
 *
 * ★ WE USE MODE 14: Fast PWM with TOP=ICR1
 *   - Flexible frequency: Set ICR1 to any value for desired PWM frequency
 *   - OCR1A sets duty cycle (0 to ICR1)
 *   - Updates at BOTTOM (prevents glitches)
 *   - TOV1 flag set at TOP
 *
 * -----------------------------------------------------------------------------
 * COM (COMPARE OUTPUT MODE) - For Fast PWM Modes
 * -----------------------------------------------------------------------------
 * COM1A[1:0] bits control OC1A pin behavior on compare match
 *
 * COM1A1 | COM1A0 | Description                        | Use Case
 * -------|--------|------------------------------------|------------------
 *   0    |   0    | Normal port, OC1A disconnected    | GPIO mode
 *   0    |   1    | WGM-dependent (toggle in some)    | Special modes
 *   1    |   0    | Clear on match (non-inverting)    | Standard PWM ★
 *   1    |   1    | Set on match (inverting PWM)      | Inverted output
 *
 * ★ WE USE COM1A=10 (Non-Inverting Fast PWM):
 *   - OC1A cleared when TCNT1 = OCR1A (output goes LOW)
 *   - OC1A set at BOTTOM (TCNT1 = 0, output goes HIGH)
 *   - Result: HIGH pulse width = OCR1A timer ticks
 *   - Duty % = (OCR1A / ICR1) × 100
 *
 * Inverting Mode (COM1A=11):
 *   - OC1A set when TCNT1 = OCR1A (output goes HIGH)
 *   - OC1A cleared at BOTTOM (output goes LOW)
 *   - Result: Inverted waveform
 *   - Duty % = 100 - ((OCR1A / ICR1) × 100)
 *
 * -----------------------------------------------------------------------------
 * ICNC1 & ICES1 - Input Capture Noise Canceler and Edge Select
 * -----------------------------------------------------------------------------
 * These bits control the Input Capture Unit (ICP1 pin) - used for frequency
 * measurement or external event timing. NOT used in PWM generation modes.
 *
 * ICNC1 (Bit 7) - Input Capture Noise Canceler:
 *   0 = Disabled (capture on first edge)
 *   1 = Enabled (requires 4 successive samples before capture)
 *   Purpose: Filters noise on ICP1 pin by requiring stable signal
 *   Effect: Adds 4 clock cycle delay but prevents false triggers
 *
 * ICES1 (Bit 6) - Input Capture Edge Select:
 *   0 = Falling edge triggers capture (HIGH→LOW transition)
 *   1 = Rising edge triggers capture (LOW→HIGH transition)
 *   Purpose: Choose which edge stores TCNT1 into ICR1
 *   Common use: Measure pulse width or frequency of external signals
 *
 * Note: When using ICR1 as TOP for PWM (Mode 14), these bits have no effect
 * on PWM operation. ICR1 is write-only in this mode, not used for capture.
 *
 * -----------------------------------------------------------------------------
 * CS (CLOCK SELECT) - Prescaler Selection
 * -----------------------------------------------------------------------------
 * CS12:10 Clock Select (Prescaler):
 *   000 = No clock (timer stopped)
 *   001 = clk/1 (no prescaling)
 *   010 = clk/8     ← WE USE THIS
 *   011 = clk/64
 *   100 = clk/256
 *   101 = clk/1024
 *   110 = External clock T1 (falling edge)
 *   111 = External clock T1 (rising edge)
 *
 * -----------------------------------------------------------------------------
 * REGISTER VALUES - Our Configuration
 * -----------------------------------------------------------------------------
 * ICR1 - Input Capture Register (16-bit, use ICR1H:ICR1L)
 * Sets PWM period (TOP value)
 * OUR SETTING: 1999 (0x07CF)
 * PWM Frequency = F_CPU / (Prescaler × (1 + TOP))
 *               = 16000000 / (8 × 2000) = 1000 Hz
 *
 * OCR1A - Output Compare Register A (16-bit, use OCR1AH:OCR1AL)
 * Sets PWM duty cycle
 * Duty % = (OCR1A / ICR1) × 100
 * Examples:
 *   OCR1A =    0 →   0% duty (always LOW)
 *   OCR1A =  499 →  25% duty
 *   OCR1A =  999 →  50% duty
 *   OCR1A = 1499 →  75% duty
 *   OCR1A = 1999 → 100% duty (always HIGH)
 *
 * -----------------------------------------------------------------------------
 * MEMORY TIP: "CATS"
 * -----------------------------------------------------------------------------
 *   C = COM bits (output mode)
 *   A = Always check WGM bits (waveform mode)
 *   T = TOP value in ICR1 (period)
 *   S = Speed with CS bits (prescaler)
 *
 * =============================================================================
 */

// Motor direction macros
#define MOTOR_FORWARD()       \
  do                          \
  {                           \
    PORTB |= MOTOR_DIR1_PIN;  \
    PORTB &= ~MOTOR_DIR2_PIN; \
  } while (0)
#define MOTOR_REVERSE()       \
  do                          \
  {                           \
    PORTB &= ~MOTOR_DIR1_PIN; \
    PORTB |= MOTOR_DIR2_PIN;  \
  } while (0)
#define MOTOR_BRAKE()                            \
  do                                             \
  {                                              \
    PORTB &= ~(MOTOR_DIR1_PIN | MOTOR_DIR2_PIN); \
  } while (0)

// Function prototypes
void demo1_basic_pwm_test(void);
void demo2_speed_ramp(void);
void demo3_direction_control(void);
void demo4_speed_sweep(void);
void demo5_pwm_duty_cycle_test(void);
void demo6_random_rotation(void);

// ===== MAIN - SELECT YOUR DEMO =====
int main(void)
{
  // Uncomment ONE demo to run:
  // demo1_basic_pwm_test();         // Basic PWM generation
  // demo2_speed_ramp(); // Speed ramp up/down
  // demo3_direction_control();      // Forward/reverse test
  // demo4_speed_sweep();            // Automatic speed variation
  // demo5_pwm_duty_cycle_test();    // Multiple duty levels
  demo6_random_rotation(); // Random speed/direction

  while (1)
  {
  }
  return 0;
}

// ===== DEMO 1: BASIC PWM TEST =====
// Generate PWM signal at fixed duty cycles
// Tests Timer1 Fast PWM mode on OC1A (PB5)
void demo1_basic_pwm_test(void)
{
  // Configure pins: PB5, PB6, PB7
  DDRB |= MOTOR_OC1A_PIN | MOTOR_DIR1_PIN | MOTOR_DIR2_PIN;
  MOTOR_FORWARD();

  // Timer1: Fast PWM, TOP=ICR1, prescaler=8
  // PWM frequency = F_CPU/(prescaler*(1+TOP))
  // 16000000/(8*2000) = 1000 Hz
  ICR1 = 1999;
  OCR1A = 1999; // Start at 100% to overcome starting torque
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);

  // Test different duty cycles: 100%, 75%, 50%, 25% (reverse order)
  uint16_t duties[] = {1999, 1499, 999, 499};

  while (1)
  {
    for (uint8_t i = 0; i < 4; i++)
    {
      OCR1A = duties[i];
      _delay_ms(2000);
    }
  }
}

// ===== DEMO 2: SPEED RAMP =====
// Smooth acceleration and deceleration
// Demonstrates gradual PWM duty cycle changes
void demo2_speed_ramp(void)
{
  DDRB |= MOTOR_OC1A_PIN | MOTOR_DIR1_PIN | MOTOR_DIR2_PIN;
  MOTOR_FORWARD();

  ICR1 = 1999;
  OCR1A = 0;
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);

  while (1)
  {
    // Ramp up 0% to 100%
    for (uint16_t duty = 0; duty <= 1999; duty += 20)
    {
      OCR1A = duty;
      _delay_ms(50);
    }

    _delay_ms(1000);

    // Ramp down 100% to 0%
    for (int16_t duty = 1999; duty >= 0; duty -= 20)
    {
      OCR1A = duty;
      _delay_ms(50);
    }

    _delay_ms(1000);
  }
}

// ===== DEMO 3: DIRECTION CONTROL =====
// Forward and reverse operation with H-bridge
// Tests direction control pins (PB6, PB7)
void demo3_direction_control(void)
{
  DDRB |= MOTOR_OC1A_PIN | MOTOR_DIR1_PIN | MOTOR_DIR2_PIN;

  ICR1 = 1999;
  OCR1A = 999; // 50% duty cycle
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);

  while (1)
  {
    // Forward at 50% speed
    MOTOR_FORWARD();
    _delay_ms(3000);

    // Brake
    MOTOR_BRAKE();
    _delay_ms(1000);

    // Reverse at 50% speed
    MOTOR_REVERSE();
    _delay_ms(3000);

    // Brake
    MOTOR_BRAKE();
    _delay_ms(1000);
  }
}

// ===== DEMO 4: SPEED SWEEP =====
// Continuous speed variation forward and reverse
// Combines speed control with direction changes
void demo4_speed_sweep(void)
{
  DDRB |= MOTOR_OC1A_PIN | MOTOR_DIR1_PIN | MOTOR_DIR2_PIN;

  ICR1 = 1999;
  OCR1A = 0;
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);

  while (1)
  {
    // Forward sweep
    MOTOR_FORWARD();
    for (uint16_t duty = 0; duty <= 1999; duty += 10)
    {
      OCR1A = duty;
      _delay_ms(20);
    }
    for (int16_t duty = 1999; duty >= 0; duty -= 10)
    {
      OCR1A = duty;
      _delay_ms(20);
    }

    MOTOR_BRAKE();
    _delay_ms(500);

    // Reverse sweep
    MOTOR_REVERSE();
    for (uint16_t duty = 0; duty <= 1999; duty += 10)
    {
      OCR1A = duty;
      _delay_ms(20);
    }
    for (int16_t duty = 1999; duty >= 0; duty -= 10)
    {
      OCR1A = duty;
      _delay_ms(20);
    }

    MOTOR_BRAKE();
    _delay_ms(500);
  }
}

// ===== DEMO 5: PWM DUTY CYCLE TEST =====
// Test multiple duty cycle levels
// Similar to demo1 but includes 0% and uses loop
void demo5_pwm_library_test(void)
{
  DDRB |= MOTOR_OC1A_PIN | MOTOR_DIR1_PIN | MOTOR_DIR2_PIN;
  MOTOR_FORWARD();

  // Manual PWM setup (same as demo1)
  ICR1 = 1999;
  OCR1A = 0;
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);

  uint16_t duties[] = {0, 499, 999, 1499, 1999}; // 0%, 25%, 50%, 75%, 100%

  while (1)
  {
    for (uint8_t i = 0; i < 5; i++)
    {
      OCR1A = duties[i];
      _delay_ms(2000);
    }
  }
}

// ===== DEMO 6: RANDOM ROTATION =====
// Random speed and direction changes using simple pseudo-random generator
// Demonstrates unpredictable motor behavior
void demo6_random_rotation(void)
{
  DDRB |= MOTOR_OC1A_PIN | MOTOR_DIR1_PIN | MOTOR_DIR2_PIN;

  // Initialize PWM
  ICR1 = 1999;
  OCR1A = 0;
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);

  // Simple pseudo-random number generator seed
  // Uses TCNT1 as entropy source (timer counter changes unpredictably at start)
  uint16_t random_seed = TCNT1;

  while (1)
  {
    // Linear Congruential Generator (LCG) for pseudo-random numbers
    // Formula: seed = (seed * 1103515245 + 12345) % 65536
    random_seed = (random_seed * 25173UL + 13849UL) & 0xFFFF;

    // Extract random speed (0-1999) from upper bits
    uint16_t random_speed = (random_seed >> 4) % 2000;

    // Extract random direction from lower bit
    if (random_seed & 0x0001)
    {
      MOTOR_FORWARD();
    }
    else
    {
      MOTOR_REVERSE();
    }

    // Apply random speed
    OCR1A = random_speed;

    // Random duration (1-3 seconds) using fixed delay loop
    uint16_t random_duration = 10 + ((random_seed >> 6) % 20); // 10-30 iterations
    for (uint16_t i = 0; i < random_duration; i++)
    {
      _delay_ms(100); // 100ms * 10-30 = 1-3 seconds
    }

    // Brief brake between changes
    MOTOR_BRAKE();
    _delay_ms(200);
  }
}
