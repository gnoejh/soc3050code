/*
 * =============================================================================
 * EDUCATIONAL ATmega128 PWM LIBRARY - HEADER FILE
 * =============================================================================
 *
 * COURSE: SOC 3050 - Embedded Systems and IoT
 * AUTHOR: Professor Kim
 *
 * PURPOSE:
 * Dedicated PWM (Pulse Width Modulation) abstraction library for ATmega128.
 * Provides high-level interface for servo control, motor control, LED dimming,
 * and general PWM signal generation. Builds on top of general timer library.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master PWM concepts and duty cycle control
 * 2. Learn servo motor control principles
 * 3. Understand DC motor speed control
 * 4. Practice LED brightness modulation
 * 5. Explore frequency and resolution tradeoffs
 * 6. Bridge timer hardware to practical PWM applications
 *
 * PWM OVERVIEW:
 * PWM = Pulse Width Modulation - digital signal with variable duty cycle
 * Duty Cycle = percentage of time signal is HIGH vs LOW
 * Applications: motor control, LED dimming, audio generation, power control
 *
 * ATMEGA128 PWM CAPABILITIES:
 * - Timer0: 1 PWM channel (OC0 - PB4)
 * - Timer1: 3 PWM channels (OC1A/B/C - PB5/6/7)
 * - Timer2: 1 PWM channel (OC2 - PB7, conflicts with Timer1)
 * - Timer3: 3 PWM channels (OC3A/B/C - PE3/4/5)
 *
 * LEARNING PROGRESSION:
 * Assembly → C → Python → IoT
 * Direct registers → PWM abstraction → PWM objects → Remote control
 *
 * =============================================================================
 */

#ifndef _PWM_H_
#define _PWM_H_

#include <stdint.h>
#include "_timer.h"

/*
 * =============================================================================
 * PWM CHANNEL IDENTIFICATION
 * =============================================================================
 */

/* PWM channel enumeration - maps to timer output compare pins */
typedef enum
{
    PWM_CH_0 = 0,  // Timer0 OC0 (PB4)
    PWM_CH_1A = 1, // Timer1 OC1A (PB5)
    PWM_CH_1B = 2, // Timer1 OC1B (PB6)
    PWM_CH_1C = 3, // Timer1 OC1C (PB7) - conflicts with Timer2
    PWM_CH_2 = 4,  // Timer2 OC2 (PB7) - conflicts with OC1C
    PWM_CH_3A = 5, // Timer3 OC3A (PE3)
    PWM_CH_3B = 6, // Timer3 OC3B (PE4)
    PWM_CH_3C = 7  // Timer3 OC3C (PE5)
} pwm_channel_t;

/*
 * =============================================================================
 * PWM MODE CONFIGURATION
 * =============================================================================
 */

/* PWM mode selection */
typedef enum
{
    PWM_MODE_FAST = 0,         // Fast PWM - maximum frequency
    PWM_MODE_PHASE_CORRECT = 1 // Phase Correct PWM - cleaner signal
} pwm_mode_t;

/*
 * =============================================================================
 * STANDARD PWM FREQUENCIES
 * =============================================================================
 */

/* Common PWM frequencies for different applications */
#define PWM_FREQ_SERVO_50HZ 50     // Standard servo PWM (20ms period)
#define PWM_FREQ_SERVO_100HZ 100   // Fast servo PWM (10ms period)
#define PWM_FREQ_MOTOR_1KHZ 1000   // DC motor control (inaudible)
#define PWM_FREQ_MOTOR_20KHZ 20000 // High-frequency motor control
#define PWM_FREQ_LED_1KHZ 1000     // LED dimming (flicker-free)
#define PWM_FREQ_LED_10KHZ 10000   // High-frequency LED dimming
#define PWM_FREQ_AUDIO_440HZ 440   // Audio tone (A4 note)

/*
 * =============================================================================
 * SERVO MOTOR CONSTANTS
 * =============================================================================
 */

/* Standard servo timing (microseconds) */
#define SERVO_PULSE_MIN_US 544     // Minimum pulse width (0°)
#define SERVO_PULSE_CENTER_US 1472 // Center pulse width (90°)
#define SERVO_PULSE_MAX_US 2400    // Maximum pulse width (180°)
#define SERVO_PERIOD_US 20000      // Standard servo period (50Hz)

/* Servo angle limits */
#define SERVO_ANGLE_MIN 0     // Minimum angle (degrees)
#define SERVO_ANGLE_CENTER 90 // Center angle (degrees)
#define SERVO_ANGLE_MAX 180   // Maximum angle (degrees)

/*
 * =============================================================================
 * CORE PWM FUNCTIONS
 * =============================================================================
 */

/**
 * @brief Initialize PWM channel with specified frequency and mode
 *
 * @param channel PWM channel to initialize
 * @param frequency_hz PWM frequency in Hz
 * @param mode PWM mode (FAST or PHASE_CORRECT)
 * @return 1 if successful, 0 if failed
 *
 * EDUCATIONAL NOTE: Sets up timer, configures pins, calculates prescaler.
 * Automatically selects best prescaler for desired frequency.
 */
uint8_t PWM_init(pwm_channel_t channel, uint32_t frequency_hz, pwm_mode_t mode);

/**
 * @brief Start PWM output on channel
 *
 * @param channel PWM channel to start
 *
 * EDUCATIONAL NOTE: Connects OC pin to timer output.
 */
void PWM_start(pwm_channel_t channel);

/**
 * @brief Stop PWM output on channel
 *
 * @param channel PWM channel to stop
 *
 * EDUCATIONAL NOTE: Disconnects OC pin, output goes LOW.
 */
void PWM_stop(pwm_channel_t channel);

/**
 * @brief Set PWM duty cycle as percentage
 *
 * @param channel PWM channel to configure
 * @param duty_percent Duty cycle (0-100%)
 *
 * EDUCATIONAL NOTE: 0% = always LOW, 100% = always HIGH, 50% = square wave.
 */
void PWM_set_duty_cycle(pwm_channel_t channel, uint8_t duty_percent);

/**
 * @brief Set PWM duty cycle using raw timer value
 *
 * @param channel PWM channel to configure
 * @param duty_value Raw OCR value (0-255 for 8-bit, 0-65535 for 16-bit)
 *
 * EDUCATIONAL NOTE: Direct control for precise timing.
 */
void PWM_set_duty_cycle_raw(pwm_channel_t channel, uint16_t duty_value);

/**
 * @brief Get current duty cycle percentage
 *
 * @param channel PWM channel to read
 * @return Current duty cycle (0-100%)
 */
uint8_t PWM_get_duty_cycle(pwm_channel_t channel);

/**
 * @brief Set PWM frequency (reconfigures timer)
 *
 * @param channel PWM channel to reconfigure
 * @param frequency_hz New PWM frequency in Hz
 * @return 1 if successful, 0 if failed
 *
 * EDUCATIONAL NOTE: May affect other channels on same timer.
 */
uint8_t PWM_set_frequency(pwm_channel_t channel, uint32_t frequency_hz);

/**
 * @brief Get current PWM frequency
 *
 * @param channel PWM channel to check
 * @return Current frequency in Hz
 */
uint32_t PWM_get_frequency(pwm_channel_t channel);

/*
 * =============================================================================
 * SERVO MOTOR CONTROL FUNCTIONS
 * =============================================================================
 */

/**
 * @brief Initialize PWM channel for servo motor control
 *
 * @param channel PWM channel to use for servo
 * @return 1 if successful, 0 if failed
 *
 * EDUCATIONAL NOTE: Configures for 50Hz PWM (20ms period).
 * Standard servo control frequency.
 */
uint8_t PWM_servo_init(pwm_channel_t channel);

/**
 * @brief Set servo angle in degrees
 *
 * @param channel PWM channel connected to servo
 * @param angle_degrees Servo angle (0-180°)
 *
 * EDUCATIONAL NOTE:
 * - 0° = ~544μs pulse width
 * - 90° = ~1472μs pulse width
 * - 180° = ~2400μs pulse width
 */
void PWM_servo_set_angle(pwm_channel_t channel, uint8_t angle_degrees);

/**
 * @brief Set servo position using pulse width in microseconds
 *
 * @param channel PWM channel connected to servo
 * @param pulse_us Pulse width in microseconds (544-2400)
 *
 * EDUCATIONAL NOTE: Direct control for non-standard servos.
 */
void PWM_servo_set_pulse_us(pwm_channel_t channel, uint16_t pulse_us);

/**
 * @brief Sweep servo from start to end angle
 *
 * @param channel PWM channel connected to servo
 * @param start_angle Starting angle (0-180°)
 * @param end_angle Ending angle (0-180°)
 * @param step_delay_ms Delay between steps in milliseconds
 *
 * EDUCATIONAL NOTE: Smooth servo movement demonstration.
 */
void PWM_servo_sweep(pwm_channel_t channel, uint8_t start_angle, uint8_t end_angle, uint16_t step_delay_ms);

/**
 * @brief Center servo to 90° position
 *
 * @param channel PWM channel connected to servo
 */
void PWM_servo_center(pwm_channel_t channel);

/*
 * =============================================================================
 * DC MOTOR CONTROL FUNCTIONS
 * =============================================================================
 */

/**
 * @brief Initialize PWM channel for DC motor control
 *
 * @param channel PWM channel for motor speed control
 * @param frequency_hz PWM frequency (recommended: 1kHz-20kHz)
 * @return 1 if successful, 0 if failed
 *
 * EDUCATIONAL NOTE: Higher frequency = smoother motor operation.
 * 1kHz is inaudible, 20kHz is optimal for most motors.
 */
uint8_t PWM_motor_init(pwm_channel_t channel, uint32_t frequency_hz);

/**
 * @brief Set motor speed as percentage
 *
 * @param channel PWM channel connected to motor driver
 * @param speed_percent Motor speed (0-100%)
 *
 * EDUCATIONAL NOTE: 0% = stopped, 100% = full speed.
 * Requires H-bridge or motor driver circuit.
 */
void PWM_motor_set_speed(pwm_channel_t channel, uint8_t speed_percent);

/**
 * @brief Gradually ramp motor speed from current to target
 *
 * @param channel PWM channel connected to motor
 * @param target_speed Target speed (0-100%)
 * @param ramp_time_ms Time to reach target in milliseconds
 *
 * EDUCATIONAL NOTE: Prevents sudden current spikes and mechanical stress.
 */
void PWM_motor_ramp_speed(pwm_channel_t channel, uint8_t target_speed, uint16_t ramp_time_ms);

/**
 * @brief Stop motor (set speed to 0%)
 *
 * @param channel PWM channel connected to motor
 */
void PWM_motor_stop(pwm_channel_t channel);

/**
 * @brief Emergency brake (short both motor terminals)
 *
 * @param channel PWM channel connected to motor
 *
 * EDUCATIONAL NOTE: Requires motor driver with brake capability.
 */
void PWM_motor_brake(pwm_channel_t channel);

/*
 * =============================================================================
 * LED DIMMING FUNCTIONS
 * =============================================================================
 */

/**
 * @brief Initialize PWM channel for LED brightness control
 *
 * @param channel PWM channel connected to LED
 * @param frequency_hz PWM frequency (recommended: 1kHz-10kHz)
 * @return 1 if successful, 0 if failed
 *
 * EDUCATIONAL NOTE: >500Hz prevents visible flickering.
 * Higher frequency = smoother dimming, more power consumption.
 */
uint8_t PWM_led_init(pwm_channel_t channel, uint32_t frequency_hz);

/**
 * @brief Set LED brightness as percentage
 *
 * @param channel PWM channel connected to LED
 * @param brightness_percent Brightness (0-100%)
 *
 * EDUCATIONAL NOTE: 0% = off, 100% = full brightness.
 * Human eye perceives brightness logarithmically.
 */
void PWM_led_set_brightness(pwm_channel_t channel, uint8_t brightness_percent);

/**
 * @brief Set LED brightness with gamma correction
 *
 * @param channel PWM channel connected to LED
 * @param brightness_percent Perceived brightness (0-100%)
 *
 * EDUCATIONAL NOTE: Applies gamma curve for linear perceived brightness.
 * Better matches human eye response.
 */
void PWM_led_set_brightness_gamma(pwm_channel_t channel, uint8_t brightness_percent);

/**
 * @brief Fade LED from current to target brightness
 *
 * @param channel PWM channel connected to LED
 * @param target_brightness Target brightness (0-100%)
 * @param fade_time_ms Fade duration in milliseconds
 *
 * EDUCATIONAL NOTE: Smooth brightness transition.
 */
void PWM_led_fade(pwm_channel_t channel, uint8_t target_brightness, uint16_t fade_time_ms);

/**
 * @brief Pulse LED (breathe effect)
 *
 * @param channel PWM channel connected to LED
 * @param period_ms Pulse period in milliseconds
 * @param cycles Number of pulses (0 = infinite)
 *
 * EDUCATIONAL NOTE: Creates breathing/pulsing effect.
 */
void PWM_led_pulse(pwm_channel_t channel, uint16_t period_ms, uint16_t cycles);

/*
 * =============================================================================
 * ADVANCED PWM FUNCTIONS
 * =============================================================================
 */

/**
 * @brief Set PWM phase shift (for multi-channel coordination)
 *
 * @param channel PWM channel to phase shift
 * @param phase_degrees Phase shift in degrees (0-360°)
 *
 * EDUCATIONAL NOTE: Useful for multi-phase motor control.
 */
void PWM_set_phase_shift(pwm_channel_t channel, uint16_t phase_degrees);

/**
 * @brief Synchronize multiple PWM channels to same frequency
 *
 * @param channels Array of PWM channels to synchronize
 * @param num_channels Number of channels
 * @param frequency_hz Common frequency in Hz
 * @return 1 if successful, 0 if failed
 *
 * EDUCATIONAL NOTE: Ensures phase-locked PWM outputs.
 */
uint8_t PWM_synchronize_channels(pwm_channel_t *channels, uint8_t num_channels, uint32_t frequency_hz);

/**
 * @brief Generate single PWM pulse (one-shot mode)
 *
 * @param channel PWM channel to use
 * @param pulse_width_us Pulse width in microseconds
 *
 * EDUCATIONAL NOTE: Useful for trigger signals, ultrasonic sensors.
 */
void PWM_generate_pulse(pwm_channel_t channel, uint16_t pulse_width_us);

/*
 * =============================================================================
 * DIAGNOSTIC AND UTILITY FUNCTIONS
 * =============================================================================
 */

/**
 * @brief Get PWM resolution in bits
 *
 * @param channel PWM channel to check
 * @return Resolution in bits (8 or 16)
 *
 * EDUCATIONAL NOTE: 8-bit = 256 steps, 16-bit = 65536 steps.
 */
uint8_t PWM_get_resolution_bits(pwm_channel_t channel);

/**
 * @brief Calculate maximum achievable frequency for resolution
 *
 * @param channel PWM channel
 * @param resolution_bits Desired resolution (8-16)
 * @return Maximum frequency in Hz
 *
 * EDUCATIONAL NOTE: Higher resolution = lower max frequency.
 * Tradeoff between precision and speed.
 */
uint32_t PWM_calculate_max_frequency(pwm_channel_t channel, uint8_t resolution_bits);

/**
 * @brief Get timer associated with PWM channel
 *
 * @param channel PWM channel
 * @return Timer ID (TIMER_0, TIMER_1, etc.)
 */
timer_id_t PWM_get_timer(pwm_channel_t channel);

/**
 * @brief Check if PWM channel is active
 *
 * @param channel PWM channel to check
 * @return 1 if active, 0 if stopped
 */
uint8_t PWM_is_active(pwm_channel_t channel);

/**
 * @brief Display PWM channel configuration
 *
 * @param channel PWM channel to display
 *
 * EDUCATIONAL NOTE: Prints frequency, duty cycle, mode, timer info.
 */
void PWM_print_config(pwm_channel_t channel);

/*
 * =============================================================================
 * EDUCATIONAL USAGE EXAMPLES
 * =============================================================================
 *
 * EXAMPLE 1: Servo Motor Control
 *   PWM_servo_init(PWM_CH_1A);           // Initialize on Timer1A
 *   PWM_start(PWM_CH_1A);
 *   PWM_servo_set_angle(PWM_CH_1A, 90);  // Center position
 *   _delay_ms(1000);
 *   PWM_servo_sweep(PWM_CH_1A, 0, 180, 20); // Sweep 0-180°
 *
 * EXAMPLE 2: DC Motor Speed Control
 *   PWM_motor_init(PWM_CH_3A, PWM_FREQ_MOTOR_20KHZ);
 *   PWM_start(PWM_CH_3A);
 *   PWM_motor_set_speed(PWM_CH_3A, 50);     // 50% speed
 *   _delay_ms(2000);
 *   PWM_motor_ramp_speed(PWM_CH_3A, 100, 1000); // Ramp to full speed
 *
 * EXAMPLE 3: LED Dimming
 *   PWM_led_init(PWM_CH_0, PWM_FREQ_LED_1KHZ);
 *   PWM_start(PWM_CH_0);
 *   PWM_led_fade(PWM_CH_0, 100, 500);       // Fade to full brightness
 *   _delay_ms(1000);
 *   PWM_led_pulse(PWM_CH_0, 2000, 5);       // Pulse 5 times
 *
 * EXAMPLE 4: Multi-Channel Synchronization
 *   pwm_channel_t rgb_leds[] = {PWM_CH_1A, PWM_CH_1B, PWM_CH_1C};
 *   PWM_synchronize_channels(rgb_leds, 3, PWM_FREQ_LED_10KHZ);
 *   PWM_start(PWM_CH_1A); PWM_start(PWM_CH_1B); PWM_start(PWM_CH_1C);
 *   PWM_led_set_brightness(PWM_CH_1A, 100); // Red full
 *   PWM_led_set_brightness(PWM_CH_1B, 50);  // Green half
 *   PWM_led_set_brightness(PWM_CH_1C, 25);  // Blue quarter
 *
 * EXAMPLE 5: Stepper Motor Control (2-phase)
 *   PWM_motor_init(PWM_CH_3A, PWM_FREQ_MOTOR_1KHZ); // Phase A
 *   PWM_motor_init(PWM_CH_3B, PWM_FREQ_MOTOR_1KHZ); // Phase B
 *   PWM_set_phase_shift(PWM_CH_3B, 90);             // 90° phase shift
 *   PWM_start(PWM_CH_3A); PWM_start(PWM_CH_3B);
 *
 * =============================================================================
 */

#endif /* _PWM_H_ */
