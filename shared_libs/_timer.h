/*
 * =============================================================================
 * EDUCATIONAL ATmega128 GENERAL TIMER LIBRARY - HEADER FILE
 * =============================================================================
 *
 * COURSE: SOC 3050 - Embedded Systems and IoT
 * AUTHOR: Professor Kim
 *
 * PURPOSE:
 * Comprehensive timer library for ATmega128 supporting all timer/counters
 * (Timer0, Timer1, Timer2, Timer3) with all operating modes. Replaces the
 * Timer2-specific library with a unified, professional-quality interface.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master timer/counter concepts across all ATmega128 timers
 * 2. Understand timer modes: Normal, CTC, Fast PWM, Phase Correct PWM
 * 3. Learn prescaler selection and timing calculations
 * 4. Practice interrupt-driven programming with callbacks
 * 5. Explore input capture for frequency/pulse measurement (Timer1)
 * 6. Bridge assembly register access to C abstraction
 *
 * ATMEGA128 TIMER OVERVIEW:
 * - Timer0: 8-bit general purpose timer
 * - Timer1: 16-bit advanced timer with input capture
 * - Timer2: 8-bit timer with asynchronous capability
 * - Timer3: 16-bit advanced timer (similar to Timer1)
 *
 * LEARNING PROGRESSION:
 * Assembly → C → Python → IoT
 * Direct registers → Structured functions → Threading → Async programming
 *
 * =============================================================================
 */

#ifndef _TIMER_H_
#define _TIMER_H_

#include <avr/io.h>
#include <stdint.h>

/*
 * =============================================================================
 * TIMER IDENTIFICATION AND SELECTION
 * =============================================================================
 */

/* Timer selection constants */
typedef enum
{
    TIMER_0 = 0, // 8-bit Timer0
    TIMER_1 = 1, // 16-bit Timer1 with input capture
    TIMER_2 = 2, // 8-bit Timer2 with async capability
    TIMER_3 = 3  // 16-bit Timer3
} timer_id_t;

/* Timer type constants */
#define TIMER_TYPE_8BIT 8
#define TIMER_TYPE_16BIT 16

/*
 * =============================================================================
 * TIMER OPERATING MODES
 * =============================================================================
 */

/* Timer mode enumeration */
typedef enum
{
    TIMER_MODE_NORMAL = 0,            // Normal mode: count to MAX, overflow
    TIMER_MODE_CTC,                   // Clear Timer on Compare Match
    TIMER_MODE_FAST_PWM,              // Fast PWM mode
    TIMER_MODE_PHASE_CORRECT_PWM,     // Phase Correct PWM mode
    TIMER_MODE_PHASE_FREQ_CORRECT_PWM // Phase and Frequency Correct PWM (16-bit only)
} timer_mode_t;

/*
 * =============================================================================
 * PRESCALER CONFIGURATION
 * =============================================================================
 */

/* Prescaler values for Timer0 and Timer1/3 */
typedef enum
{
    TIMER_PRESCALER_STOP = 0,     // Timer stopped
    TIMER_PRESCALER_1 = 1,        // No prescaling (F_CPU)
    TIMER_PRESCALER_8 = 2,        // F_CPU / 8
    TIMER_PRESCALER_64 = 3,       // F_CPU / 64
    TIMER_PRESCALER_256 = 4,      // F_CPU / 256
    TIMER_PRESCALER_1024 = 5,     // F_CPU / 1024
    TIMER_PRESCALER_EXT_FALL = 6, // External clock on falling edge
    TIMER_PRESCALER_EXT_RISE = 7  // External clock on rising edge
} timer_prescaler_t;

/* Prescaler values for Timer2 (different from Timer0/1/3) */
typedef enum
{
    TIMER2_PRESCALER_STOP = 0, // Timer stopped
    TIMER2_PRESCALER_1 = 1,    // No prescaling
    TIMER2_PRESCALER_8 = 2,    // F_CPU / 8
    TIMER2_PRESCALER_32 = 3,   // F_CPU / 32
    TIMER2_PRESCALER_64 = 4,   // F_CPU / 64
    TIMER2_PRESCALER_128 = 5,  // F_CPU / 128
    TIMER2_PRESCALER_256 = 6,  // F_CPU / 256
    TIMER2_PRESCALER_1024 = 7  // F_CPU / 1024
} timer2_prescaler_t;

/* Prescaler divisor lookup (for calculations) */
#define TIMER_PRESCALER_VALUE_1 1
#define TIMER_PRESCALER_VALUE_8 8
#define TIMER_PRESCALER_VALUE_64 64
#define TIMER_PRESCALER_VALUE_256 256
#define TIMER_PRESCALER_VALUE_1024 1024

/*
 * =============================================================================
 * COMPARE OUTPUT MODES (for PWM and CTC)
 * =============================================================================
 */

/* Compare output mode for OC pins */
typedef enum
{
    TIMER_COM_DISCONNECT = 0, // OC pin disconnected
    TIMER_COM_TOGGLE = 1,     // Toggle OC on compare match
    TIMER_COM_CLEAR = 2,      // Clear OC on compare match
    TIMER_COM_SET = 3         // Set OC on compare match
} timer_compare_output_t;

/* Compare register selection for 16-bit timers */
typedef enum
{
    TIMER_COMPARE_A = 0, // Compare register A
    TIMER_COMPARE_B = 1, // Compare register B
    TIMER_COMPARE_C = 2  // Compare register C (Timer1/3 only)
} timer_compare_register_t;

/*
 * =============================================================================
 * INPUT CAPTURE CONFIGURATION (Timer1 only)
 * =============================================================================
 */

/* Input capture edge selection */
typedef enum
{
    TIMER_IC_FALLING_EDGE = 0, // Capture on falling edge
    TIMER_IC_RISING_EDGE = 1   // Capture on rising edge
} timer_input_capture_edge_t;

/* Input capture noise canceler */
#define TIMER_IC_NOISE_CANCEL_OFF 0
#define TIMER_IC_NOISE_CANCEL_ON 1

/*
 * =============================================================================
 * INTERRUPT CALLBACK FUNCTION TYPES
 * =============================================================================
 */

/* Callback function pointer types */
typedef void (*timer_callback_t)(void);             // Simple callback
typedef void (*timer_capture_callback_t)(uint16_t); // Capture callback with value

/*
 * =============================================================================
 * CORE TIMER FUNCTIONS
 * =============================================================================
 */

/**
 * @brief Initialize timer with specified mode and prescaler
 *
 * @param timer Timer to initialize (TIMER_0, TIMER_1, TIMER_2, TIMER_3)
 * @param mode Operating mode (NORMAL, CTC, FAST_PWM, etc.)
 * @param prescaler Prescaler value
 *
 * EDUCATIONAL NOTE: This replaces separate Timer0_init(), Timer1_init(), etc.
 */
void Timer_init(timer_id_t timer, timer_mode_t mode, uint16_t prescaler);

/**
 * @brief Start timer (begin counting)
 *
 * @param timer Timer to start
 * @param prescaler Prescaler value to use
 */
void Timer_start(timer_id_t timer, uint16_t prescaler);

/**
 * @brief Stop timer (halt counting)
 *
 * @param timer Timer to stop
 */
void Timer_stop(timer_id_t timer);

/**
 * @brief Reset timer counter to zero
 *
 * @param timer Timer to reset
 */
void Timer_reset(timer_id_t timer);

/**
 * @brief Get current timer count value
 *
 * @param timer Timer to read
 * @return Current count (8-bit or 16-bit depending on timer)
 */
uint16_t Timer_get_count(timer_id_t timer);

/**
 * @brief Set timer count value
 *
 * @param timer Timer to set
 * @param value New count value
 */
void Timer_set_count(timer_id_t timer, uint16_t value);

/**
 * @brief Check if timer is running
 *
 * @param timer Timer to check
 * @return 1 if running, 0 if stopped
 */
uint8_t Timer_is_running(timer_id_t timer);

/*
 * =============================================================================
 * COMPARE MATCH FUNCTIONS (CTC and PWM modes)
 * =============================================================================
 */

/**
 * @brief Set compare match value
 *
 * @param timer Timer to configure
 * @param compare_reg Compare register (A, B, or C)
 * @param value Compare value
 *
 * EDUCATIONAL NOTE: In CTC mode, timer resets when count reaches this value.
 * In PWM mode, this sets the duty cycle.
 */
void Timer_set_compare_value(timer_id_t timer, timer_compare_register_t compare_reg, uint16_t value);

/**
 * @brief Get compare match value
 *
 * @param timer Timer to read
 * @param compare_reg Compare register to read
 * @return Current compare value
 */
uint16_t Timer_get_compare_value(timer_id_t timer, timer_compare_register_t compare_reg);

/**
 * @brief Set compare output mode (for OC pins)
 *
 * @param timer Timer to configure
 * @param compare_reg Compare register (A or B)
 * @param mode Output mode (DISCONNECT, TOGGLE, CLEAR, SET)
 */
void Timer_set_compare_output_mode(timer_id_t timer, timer_compare_register_t compare_reg, timer_compare_output_t mode);

/*
 * =============================================================================
 * INPUT CAPTURE FUNCTIONS (Timer1 only)
 * =============================================================================
 */

/**
 * @brief Initialize input capture on Timer1
 *
 * @param edge Capture edge (RISING or FALLING)
 * @param noise_cancel Enable noise canceler (0 or 1)
 *
 * EDUCATIONAL NOTE: Input capture measures external signal timing.
 * Used for frequency measurement, pulse width detection, etc.
 */
void Timer1_input_capture_init(timer_input_capture_edge_t edge, uint8_t noise_cancel);

/**
 * @brief Get last input capture value
 *
 * @return Captured timer value (16-bit)
 */
uint16_t Timer1_get_capture_value(void);

/**
 * @brief Set input capture edge
 *
 * @param edge RISING or FALLING edge
 */
void Timer1_set_capture_edge(timer_input_capture_edge_t edge);

/**
 * @brief Measure frequency using input capture
 *
 * @return Frequency in Hz (0 if no signal or timeout)
 *
 * EDUCATIONAL NOTE: Blocks until two edges are captured.
 * Returns frequency calculated from captured period.
 */
uint32_t Timer1_measure_frequency_Hz(void);

/**
 * @brief Measure pulse width using input capture
 *
 * @return Pulse width in microseconds
 *
 * EDUCATIONAL NOTE: Measures time between rising and falling edges.
 */
uint32_t Timer1_measure_pulse_width_us(void);

/*
 * =============================================================================
 * INTERRUPT MANAGEMENT
 * =============================================================================
 */

/**
 * @brief Enable overflow interrupt
 *
 * @param timer Timer to enable interrupt for
 */
void Timer_enable_overflow_interrupt(timer_id_t timer);

/**
 * @brief Disable overflow interrupt
 *
 * @param timer Timer to disable interrupt for
 */
void Timer_disable_overflow_interrupt(timer_id_t timer);

/**
 * @brief Enable compare match interrupt
 *
 * @param timer Timer to enable interrupt for
 * @param compare_reg Compare register (A, B, or C)
 */
void Timer_enable_compare_interrupt(timer_id_t timer, timer_compare_register_t compare_reg);

/**
 * @brief Disable compare match interrupt
 *
 * @param timer Timer to disable interrupt for
 * @param compare_reg Compare register (A, B, or C)
 */
void Timer_disable_compare_interrupt(timer_id_t timer, timer_compare_register_t compare_reg);

/**
 * @brief Enable input capture interrupt (Timer1 only)
 */
void Timer1_enable_capture_interrupt(void);

/**
 * @brief Disable input capture interrupt (Timer1 only)
 */
void Timer1_disable_capture_interrupt(void);

/*
 * =============================================================================
 * INTERRUPT CALLBACK REGISTRATION
 * =============================================================================
 */

/**
 * @brief Register overflow interrupt callback
 *
 * @param timer Timer to register callback for
 * @param callback Function to call on overflow interrupt
 *
 * EDUCATIONAL NOTE: Modern callback-based interrupt handling.
 * Cleaner than writing ISR directly in application code.
 */
void Timer_register_overflow_callback(timer_id_t timer, timer_callback_t callback);

/**
 * @brief Register compare match interrupt callback
 *
 * @param timer Timer to register callback for
 * @param compare_reg Compare register (A, B, or C)
 * @param callback Function to call on compare match
 */
void Timer_register_compare_callback(timer_id_t timer, timer_compare_register_t compare_reg, timer_callback_t callback);

/**
 * @brief Register input capture callback (Timer1 only)
 *
 * @param callback Function to call on capture event (receives captured value)
 */
void Timer1_register_capture_callback(timer_capture_callback_t callback);

/*
 * =============================================================================
 * TIMING CALCULATION HELPERS
 * =============================================================================
 */

/**
 * @brief Calculate prescaler for desired frequency
 *
 * @param timer Timer to calculate for
 * @param desired_freq_hz Desired frequency in Hz
 * @return Best prescaler value (0 if impossible)
 *
 * EDUCATIONAL NOTE: Helps students understand prescaler selection.
 */
uint16_t Timer_calculate_prescaler(timer_id_t timer, uint32_t desired_freq_hz);

/**
 * @brief Calculate compare value for desired period
 *
 * @param timer Timer to calculate for
 * @param prescaler Prescaler value being used
 * @param period_ms Desired period in milliseconds
 * @return Compare value (0 if impossible)
 *
 * EDUCATIONAL NOTE: Used in CTC mode for precise timing.
 */
uint16_t Timer_calculate_compare_value(timer_id_t timer, uint16_t prescaler, uint32_t period_ms);

/**
 * @brief Calculate actual frequency from prescaler and compare value
 *
 * @param timer Timer configuration
 * @param prescaler Prescaler value
 * @param compare_value Compare match value
 * @return Actual frequency in Hz
 */
uint32_t Timer_calculate_actual_frequency(timer_id_t timer, uint16_t prescaler, uint16_t compare_value);

/**
 * @brief Get timer resolution in microseconds
 *
 * @param timer Timer to check
 * @param prescaler Current prescaler
 * @return Timer tick period in microseconds
 */
float Timer_get_resolution_us(timer_id_t timer, uint16_t prescaler);

/*
 * =============================================================================
 * MILLISECOND TIMING SYSTEM (High-level abstraction)
 * =============================================================================
 */

/**
 * @brief Initialize millisecond timing system using Timer0
 *
 * EDUCATIONAL NOTE: Provides millis() functionality like Arduino.
 * Sets up Timer0 for 1ms interrupt-driven timing.
 */
void Timer_millis_init(void);

/**
 * @brief Get system uptime in milliseconds
 *
 * @return Milliseconds since Timer_millis_init() called
 */
uint32_t Timer_millis(void);

/**
 * @brief Get system uptime in microseconds (approximate)
 *
 * @return Microseconds since Timer_millis_init() called
 */
uint32_t Timer_micros(void);

/**
 * @brief Delay for specified milliseconds (blocking)
 *
 * @param ms Milliseconds to delay
 *
 * EDUCATIONAL NOTE: Non-busy-wait delay using timer system.
 */
void Timer_delay_ms(uint32_t ms);

/*
 * =============================================================================
 * TASK SCHEDULER (Educational multi-tasking)
 * =============================================================================
 */

#define TIMER_MAX_TASKS 8

/* Task structure */
typedef struct
{
    timer_callback_t callback; // Function to call
    uint32_t interval_ms;      // Task interval in milliseconds
    uint32_t last_run_ms;      // Last execution time
    uint8_t enabled;           // Task enabled flag
} timer_task_t;

/**
 * @brief Initialize task scheduler
 *
 * @param timer Timer to use for scheduling (typically TIMER_2)
 */
void Timer_scheduler_init(timer_id_t timer);

/**
 * @brief Add task to scheduler
 *
 * @param callback Function to execute
 * @param interval_ms Execution interval in milliseconds
 * @return Task ID (0-7) or 0xFF if scheduler full
 */
uint8_t Timer_scheduler_add_task(timer_callback_t callback, uint32_t interval_ms);

/**
 * @brief Remove task from scheduler
 *
 * @param task_id Task ID to remove
 */
void Timer_scheduler_remove_task(uint8_t task_id);

/**
 * @brief Enable task
 *
 * @param task_id Task ID to enable
 */
void Timer_scheduler_enable_task(uint8_t task_id);

/**
 * @brief Disable task
 *
 * @param task_id Task ID to disable
 */
void Timer_scheduler_disable_task(uint8_t task_id);

/**
 * @brief Update scheduler (call from main loop)
 *
 * EDUCATIONAL NOTE: Checks all tasks and executes those whose intervals have elapsed.
 */
void Timer_scheduler_update(void);

/*
 * =============================================================================
 * BACKWARD COMPATIBILITY WITH OLD TIMER2 LIBRARY
 * =============================================================================
 */

/* Legacy Timer2 functions - redirect to new API */
#define Timer2_init() Timer_init(TIMER_2, TIMER_MODE_NORMAL, TIMER2_PRESCALER_64)
#define Timer2_start() Timer_start(TIMER_2, TIMER2_PRESCALER_64)
#define Timer2_stop() Timer_stop(TIMER_2)

/* Legacy global variables (deprecated - use new API) */
extern volatile uint32_t system_milliseconds; // Use Timer_millis() instead

/*
 * =============================================================================
 * EDUCATIONAL USAGE EXAMPLES
 * =============================================================================
 *
 * EXAMPLE 1: Basic Timer Overflow Interrupt
 *   Timer_init(TIMER_0, TIMER_MODE_NORMAL, TIMER_PRESCALER_64);
 *   Timer_enable_overflow_interrupt(TIMER_0);
 *   sei(); // Enable global interrupts
 *
 *   // ISR handled automatically via callback
 *
 * EXAMPLE 2: CTC Mode for Precise Timing
 *   Timer_init(TIMER_1, TIMER_MODE_CTC, TIMER_PRESCALER_256);
 *   Timer_set_compare_value(TIMER_1, TIMER_COMPARE_A, 62499); // 1 second at 16MHz
 *   Timer_enable_compare_interrupt(TIMER_1, TIMER_COMPARE_A);
 *   Timer_register_compare_callback(TIMER_1, TIMER_COMPARE_A, my_1hz_task);
 *   sei();
 *
 * EXAMPLE 3: Input Capture Frequency Measurement
 *   Timer1_input_capture_init(TIMER_IC_RISING_EDGE, 1);
 *   uint32_t freq = Timer1_measure_frequency_Hz();
 *   printf("Frequency: %lu Hz\n", freq);
 *
 * EXAMPLE 4: Millisecond System (Arduino-style)
 *   Timer_millis_init();
 *   sei();
 *
 *   uint32_t start = Timer_millis();
 *   // Do something
 *   uint32_t elapsed = Timer_millis() - start;
 *
 * EXAMPLE 5: Task Scheduler
 *   Timer_scheduler_init(TIMER_2);
 *   Timer_scheduler_add_task(blink_led, 500);    // Blink every 500ms
 *   Timer_scheduler_add_task(read_sensor, 1000); // Read every 1s
 *   sei();
 *
 *   while(1) {
 *       Timer_scheduler_update(); // Execute scheduled tasks
 *   }
 *
 * =============================================================================
 */

#endif /* _TIMER_H_ */
