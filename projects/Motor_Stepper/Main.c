/*
 * ==============================================================================
 * STEPPER MOTOR - DEMO CODE
 * ==============================================================================
 * PROJECT: Motor_Stepper
 * See Slide.md for complete theory and technical details
 *
 * DEMOS: Stepper motor control, step sequencing, speed control
 * ==============================================================================
 */

#include "config.h"

// Stepper motor control pins (Port B)
#define STEPPER_PORT PORTB
#define STEPPER_DDR DDRB
#define COIL_A1 (1 << PB0)
#define COIL_A2 (1 << PB1)
#define COIL_B1 (1 << PB2)
#define COIL_B2 (1 << PB3)

// Stepper motor specifications
// NOTE: For SimulIDE simulation, set "Steps per Rotation" to 200 in the stepper
// motor component properties If using SimulIDE's default 4-step model, change
// STEPS_PER_REV to 4 below
#define STEPS_PER_REV 200 // Standard 1.8° stepper (200 steps = 360°)
// #define STEPS_PER_REV 4  // Uncomment this for SimulIDE's 4-step model
#define GEAR_RATIO 1       // No gearbox
#define STEPS_FULL_CYCLE 4 // Full-step sequence length (electrical steps)
#define STEPS_HALF_CYCLE 8 // Half-step sequence length (electrical steps)

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
void stepper_init(void) {
  // Set Port B as output
  STEPPER_DDR = 0xFF;
  // Initialize: stepper coils OFF, LEDs OFF (active low: HIGH = OFF)
  STEPPER_PORT =
      0xF0; // Upper nibble HIGH = LEDs OFF, lower nibble LOW = coils OFF

  // Initialize position
  current_position = 0;
  current_step_index = 0;
  stepping_mode = 0;
}

/*
 * Set coil state directly
 * Lower nibble (PB0-PB3): Stepper motor coils
 * Upper nibble (PB4-PB7): LEDs (preserved)
 */
void stepper_set_coils(uint8_t coil_pattern) {
  // Update only lower nibble for stepper, preserve upper nibble for LEDs
  STEPPER_PORT = (STEPPER_PORT & 0xF0) | (coil_pattern & 0x0F);
}

/*
 * Release all coils (power off)
 * Preserves LED states on upper nibble
 */
void stepper_release(void) {
  STEPPER_PORT = STEPPER_PORT & 0xF0; // Clear only stepper coils, keep LEDs
}

/*
 * Step forward one step
 */
void stepper_step_forward(void) {
  if (stepping_mode == 0) {
    // Full-step mode
    current_step_index = (current_step_index + 1) % STEPS_FULL_CYCLE;
    stepper_set_coils(full_step_sequence[current_step_index]);
  } else {
    // Half-step mode
    current_step_index = (current_step_index + 1) % STEPS_HALF_CYCLE;
    stepper_set_coils(half_step_sequence[current_step_index]);
  }
  current_position++;
}

/*
 * Step backward one step
 */
void stepper_step_backward(void) {
  if (stepping_mode == 0) {
    // Full-step mode
    if (current_step_index == 0) {
      current_step_index = STEPS_FULL_CYCLE - 1;
    } else {
      current_step_index--;
    }
    stepper_set_coils(full_step_sequence[current_step_index]);
  } else {
    // Half-step mode
    if (current_step_index == 0) {
      current_step_index = STEPS_HALF_CYCLE - 1;
    } else {
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
void stepper_move_steps(int32_t steps, uint16_t delay_ms) {
  int32_t steps_to_move = abs(steps);

  for (int32_t i = 0; i < steps_to_move; i++) {
    if (steps > 0) {
      stepper_step_forward();
    } else {
      stepper_step_backward();
    }
    // Variable delay using busy wait
    for (uint16_t d = 0; d < delay_ms; d++) {
      _delay_ms(1);
    }
  }
}

/*
 * Rotate by degrees
 */
void stepper_rotate_degrees(int16_t degrees, uint16_t delay_ms) {
  // Calculate steps needed
  int32_t steps = ((int32_t)degrees * STEPS_PER_REV) / 360;

  if (stepping_mode == 1) {
    steps *= 2; // Half-step mode has double the resolution
  }

  stepper_move_steps(steps, delay_ms);
}

/*
 * Set stepper mode
 */
void stepper_set_mode(uint8_t mode) {
  stepping_mode = mode;
  current_step_index = 0;
}

/* ========================================================================
 * DEMO 1: Basic Forward/Backward Stepping
 * ======================================================================== */
void demo1_basic_stepping(void) {
  // LED indicator: Demo 1 active (PB4 LOW = LED ON, active low)
  STEPPER_PORT = (STEPPER_PORT & 0xEF);

  // Forward 50 steps
  stepper_move_steps(50, 20);
  _delay_ms(500);

  // Backward 50 steps
  stepper_move_steps(-50, 20);
  _delay_ms(500);

  // Forward 100 steps
  stepper_move_steps(100, 15);
  _delay_ms(500);

  // Backward 100 steps
  stepper_move_steps(-100, 15);
  _delay_ms(1000);

  stepper_release();
  STEPPER_PORT =
      STEPPER_PORT | 0xF0; // Turn all LEDs OFF (set upper nibble HIGH)
}

/* ========================================================================
 * DEMO 2: Continuous Rotation at Different Speeds
 * ======================================================================== */
void demo2_continuous_rotation(void) {
  // LED indicator: Demo 2 active (PB5 LOW = LED ON, active low)
  STEPPER_PORT = (STEPPER_PORT & 0xDF);

  uint16_t speeds_ms[] = {20, 10, 5, 2, 1};
  uint8_t num_speeds = sizeof(speeds_ms) / sizeof(speeds_ms[0]);

  for (uint8_t i = 0; i < num_speeds; i++) {
    // Show speed index on LEDs (PB4-PB7), preserve stepper coils
    // Active low: invert the pattern (0 = LED ON, 1 = LED OFF)
    uint8_t led_pattern = ~((i + 2) << 4) & 0xF0;
    STEPPER_PORT = (STEPPER_PORT & 0x0F) | led_pattern;

    // Rotate one full revolution at current speed
    for (uint16_t step = 0; step < STEPS_PER_REV; step++) {
      stepper_step_forward();
      for (uint16_t d = 0; d < speeds_ms[i]; d++) {
        _delay_ms(1);
      }
    }

    _delay_ms(1000); // Pause between speed changes
  }

  stepper_release();
  STEPPER_PORT =
      STEPPER_PORT | 0xF0; // Turn all LEDs OFF (set upper nibble HIGH)
}

/* ========================================================================
 * DEMO 3: Position Control - Move to Specific Angles
 * ======================================================================== */
void demo3_position_control(void) {
  // LED indicator: Demo 3 active (PB6 LOW = LED ON, active low)
  STEPPER_PORT = (STEPPER_PORT & 0xBF);

  int16_t target_angles[] = {0, 90, 180, 270, 360, 270, 180, 90, 0, -90, 0};
  uint8_t num_positions = sizeof(target_angles) / sizeof(target_angles[0]);

  current_position = 0; // Home position

  for (uint8_t i = 0; i < num_positions; i++) {
    // Show position index on LEDs (PB4-PB7), preserve stepper coils
    // Active low: invert the pattern (0 = LED ON, 1 = LED OFF)
    uint8_t led_pattern = ~((i % 16) << 4) & 0xF0;
    STEPPER_PORT = (STEPPER_PORT & 0x0F) | led_pattern;

    stepper_rotate_degrees(target_angles[i], 5);
    _delay_ms(1000); // Hold position
  }

  stepper_release();
  STEPPER_PORT =
      STEPPER_PORT | 0xF0; // Turn all LEDs OFF (set upper nibble HIGH)
}

/* ========================================================================
 * DEMO 4: Full-Step vs Half-Step Comparison
 * ======================================================================== */
void demo4_mode_comparison(void) {
  // LED indicator: Demo 4 active (PB7 LOW = LED ON, active low)
  STEPPER_PORT = (STEPPER_PORT & 0x7F);

  // Full-step test
  STEPPER_PORT =
      (STEPPER_PORT & 0x7F); // Full-step mode indicator (PB7 LOW = ON)
  stepper_set_mode(0);
  current_position = 0;

  // One complete revolution in full-step mode (200 steps)
  for (uint16_t i = 0; i < STEPS_PER_REV; i++) {
    stepper_step_forward();
    _delay_ms(10);
  }

  _delay_ms(2000); // Pause between modes

  // Half-step test
  STEPPER_PORT =
      (STEPPER_PORT & 0x6F); // Half-step mode indicator (PB7+PB4 LOW = ON)
  stepper_set_mode(1);
  current_position = 0;

  // One complete revolution in half-step mode (400 steps)
  for (uint16_t i = 0; i < STEPS_PER_REV * 2; i++) {
    stepper_step_forward();
    _delay_ms(5);
  }

  _delay_ms(2000);
  stepper_release();
  STEPPER_PORT =
      STEPPER_PORT | 0xF0; // Turn all LEDs OFF (set upper nibble HIGH)
}

/* ========================================================================
 * DEMO 5: Wave Drive vs Two-Phase Drive Comparison
 * ======================================================================== */
void demo5_drive_mode_comparison(void) {
  // LED indicator: Demo 5 active (PB4 LOW = LED ON, active low)
  STEPPER_PORT = (STEPPER_PORT & 0xEF);

  // Wave drive (one phase on) - using full_step_sequence_wave
  STEPPER_PORT = (STEPPER_PORT & 0xEF); // PB4 LOW = LED ON
  current_position = 0;
  current_step_index = 0;

  // Rotate 50 steps using wave drive
  for (uint16_t i = 0; i < 50; i++) {
    current_step_index = (current_step_index + 1) % STEPS_FULL_CYCLE;
    // Update only stepper coils, preserve LEDs
    STEPPER_PORT = (STEPPER_PORT & 0xF0) |
                   (full_step_sequence_wave[current_step_index] & 0x0F);
    _delay_ms(20);
  }

  _delay_ms(1000);

  // Two-phase drive (two phases on) - using full_step_sequence
  STEPPER_PORT = (STEPPER_PORT & 0xDF); // LED indicator (PB5 LOW = LED ON)
  stepper_set_mode(0);
  current_position = 0;

  // Rotate 50 steps using two-phase drive
  for (uint16_t i = 0; i < 50; i++) {
    stepper_step_forward();
    _delay_ms(20);
  }

  _delay_ms(1000);
  stepper_release();
  STEPPER_PORT =
      STEPPER_PORT | 0xF0; // Turn all LEDs OFF (set upper nibble HIGH)
}

/* ========================================================================
 * DEMO 6: Random Rotation - Random movements in random directions
 * ======================================================================== */
void demo6_random_rotation(void) {
  // LED indicator: Demo 6 active (PB4+PB5 LOW = LEDs ON, active low)
  STEPPER_PORT = (STEPPER_PORT & 0xCF);

  // Simple pseudo-random number generator seed (using position as seed)
  uint16_t random_seed = (uint16_t)current_position;
  if (random_seed == 0) {
    random_seed = 12345; // Default seed
  }

  // Random rotation demo: perform 10 random movements
  uint8_t num_movements = 10;
  current_position = 0; // Start from home

  for (uint8_t i = 0; i < num_movements; i++) {
    // Update LED pattern to show movement number (active low)
    uint8_t led_pattern = ~((i % 16) << 4) & 0xF0;
    STEPPER_PORT = (STEPPER_PORT & 0x0F) | led_pattern;

    // Generate pseudo-random number (Linear Feedback Shift Register)
    random_seed = (random_seed << 1) ^ (random_seed & 0x8000 ? 0x1021 : 0);

    // Random step count: 10 to 100 steps
    int32_t random_steps = 10 + (random_seed % 91);

    // Random direction: positive (forward) or negative (backward)
    if (random_seed & 0x0100) {
      random_steps = -random_steps; // Reverse direction
    }

    // Random speed: 5ms to 30ms per step
    uint16_t random_delay = 5 + (random_seed % 26);

    // Move random steps at random speed
    stepper_move_steps(random_steps, random_delay);

    // Hold position for a moment
    _delay_ms(500);

    // Update seed for next iteration
    random_seed += current_position;
  }

  // Return to home position
  STEPPER_PORT = (STEPPER_PORT & 0xCF); // Show return indicator
  stepper_move_steps(-current_position, 10);
  _delay_ms(1000);

  stepper_release();
  STEPPER_PORT =
      STEPPER_PORT | 0xF0; // Turn all LEDs OFF (set upper nibble HIGH)
}

/* ========================================================================
 * DEMO 7: Sinusoidal Motion - Smooth sine wave motion pattern
 * ======================================================================== */
void demo7_sinusoidal_motion(void) {
  // LED indicator: Demo 7 active (PB6+PB7 LOW = LEDs ON, active low)
  STEPPER_PORT = (STEPPER_PORT & 0x3F);

  // Sine lookup table (0-90 degrees, scaled 0-100 for amplitude)
  // Approximate sine values: sin(angle) * 100
  // sin(0°)=0, sin(30°)=50, sin(45°)=71, sin(60°)=87, sin(90°)=100
  const uint8_t sine_table[91] = {
      0,   2,   3,   5,   7,   9,   11,  13,  14,  16,  17,  19,  21,
      22,  24,  26,  28,  29,  31,  33,  34,  36,  38,  39,  41,  43,
      45,  46,  48,  50,  52,  53,  55,  57,  59,  60,  62,  64,  65,
      67,  69,  71,  72,  74,  76,  77,  79,  81,  82,  84,  86,  87,
      89,  91,  92,  94,  95,  97,  98,  100, 100, 100, 100, 100, 100,
      100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
      100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};

  current_position = 0; // Start from home
  stepper_set_mode(0);  // Use full-step mode

  // Sinusoidal motion: oscillate back and forth following sine wave
  uint16_t amplitude_steps = 50; // Maximum steps from center (50 steps ≈ 90°)
  uint16_t base_delay = 15;      // Base delay in ms
  uint8_t cycles = 3;            // Number of complete sine wave cycles
  uint8_t steps_per_degree = 2;  // Resolution: 2 steps per degree

  for (uint8_t cycle = 0; cycle < cycles; cycle++) {
    // Update LED to show cycle number (active low)
    uint8_t led_pattern = ~((cycle % 4) << 4) & 0xF0;
    STEPPER_PORT = (STEPPER_PORT & 0x0F) | led_pattern;

    // Complete sine wave cycle: 0° to 360°
    for (uint16_t angle = 0; angle <= 360; angle += steps_per_degree) {
      // Get sine value (0-90 degrees, mirrored for other quadrants)
      uint8_t sine_idx;
      int8_t sign = 1;

      if (angle <= 90) {
        sine_idx = angle;
        sign = 1; // Positive
      } else if (angle <= 180) {
        sine_idx = 180 - angle;
        sign = 1; // Positive
      } else if (angle <= 270) {
        sine_idx = angle - 180;
        sign = -1; // Negative
      } else {
        sine_idx = 360 - angle;
        sign = -1; // Negative
      }

      // Calculate target position: amplitude * sin(angle)
      int32_t target_pos =
          (sign * (int32_t)amplitude_steps * sine_table[sine_idx]) / 100;

      // Calculate speed: faster at zero crossings, slower at peaks
      // Speed is proportional to cosine (derivative of sine)
      uint8_t cosine_idx = (sine_idx <= 90) ? (90 - sine_idx) : (sine_idx - 90);
      uint8_t speed_factor = sine_table[cosine_idx]; // Use cosine for speed
      uint16_t delay = base_delay + ((100 - speed_factor) * 10 / 100);

      // Move one step toward target
      if (target_pos > current_position) {
        stepper_step_forward();
      } else if (target_pos < current_position) {
        stepper_step_backward();
      }

      // Variable delay using busy wait loop
      for (uint16_t d = 0; d < delay; d++) {
        _delay_ms(1);
      }
    }

    _delay_ms(500); // Pause between cycles
  }

  // Return to home position
  STEPPER_PORT = (STEPPER_PORT & 0x3F); // Show return indicator
  stepper_move_steps(-current_position, 10);
  _delay_ms(1000);

  stepper_release();
  STEPPER_PORT =
      STEPPER_PORT | 0xF0; // Turn all LEDs OFF (set upper nibble HIGH)
}

// ===== MAIN - SELECT YOUR DEMO =====
int main(void) {
  // Initialize stepper motor (PORTB lower nibble for coils, upper nibble for
  // LEDs)
  stepper_init();

  // Startup delay
  _delay_ms(500);

  // Uncomment ONE demo to run:
  demo1_basic_stepping(); // Basic forward/backward stepping
  // demo2_continuous_rotation(); // Continuous rotation at different speeds
  // demo3_position_control(); // Position control - move to specific angles
  // demo4_mode_comparison(); // Full-step vs Half-step comparison
  // demo5_drive_mode_comparison(); // Wave drive vs Two-phase drive
  // demo6_random_rotation(); // Random rotation movements
  // demo7_sinusoidal_motion(); // Sinusoidal motion

  while (1) {
    // Demo functions run until completion
    // After completion, motor is released and LEDs are off
  }

  return 0;
}
