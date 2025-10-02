/*
 * config.h - Optimized Educational Configuration for ATmega128
 * Assembly → C → Python Teaching Progression
 *
 * LEARNING PROGRESSION:
 * Phase 1: Assembly Fundamentals (direct register access)
 * Phase 2: C Hardware Abstraction (library functions)
 * Phase 3: Communication & Sensors (UART, ADC, interrupts)
 * Phase 4: Advanced Applications (complex projects)
 * Phase 5: Python Integration (high-level programming)
 *
 * USAGE: Uncomment ONE #define from desired learning phase, compile and test
 *
 * OPTIMIZATION FEATURES:
 * - Conditional compilation for memory efficiency
 * - Optimized include dependencies
 * - Performance-tuned build configurations
 * - Educational validation with detailed feedback
 * - Feature detection and capability management
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// Define F_CPU first to prevent warnings in util/delay.h
#ifndef F_CPU
#define F_CPU 16000000UL // 16 MHz crystal oscillator
#endif

/*
 * ============================================================================
 * SYSTEM OPTIMIZATION CONFIGURATION
 * ============================================================================
 */

// Core system parameters (optimized for ATmega128)
#define BAUD 9600 // Standard UART baud rate for education

// Optimization flags for educational framework
#define EDUCATIONAL_FRAMEWORK_VERSION_MAJOR 2
#define EDUCATIONAL_FRAMEWORK_VERSION_MINOR 0
#define EDUCATIONAL_FRAMEWORK_OPTIMIZED 1

// Memory optimization settings
#ifndef DEBUG_MODE
#define MEMORY_OPTIMIZED 1 // Enable memory optimizations
#define FAST_COMPILATION 1 // Enable fast compilation flags
#endif

/*
 * OPTIMIZED SYSTEM INCLUDES - Conditional based on active features
 */
#include <avr/io.h>        // Hardware register definitions (always needed)
#include <avr/interrupt.h> // Interrupt handling (always needed)
#include <stdint.h>        // Standard integer types (int8_t, uint16_t, etc.)
// Note: util/delay.h included in individual files to prevent F_CPU conflicts

// Standard library includes (conditional for memory optimization)
#if defined(MEMORY_OPTIMIZED) && MEMORY_OPTIMIZED == 0
#include <stdio.h>  // Standard I/O (sprintf, etc.)
#include <string.h> // String manipulation
#include <stdlib.h> // Standard library functions
#endif

/*
 * PROJECT MODULES - Conditional includes for memory optimization
 */
#include "_main.h" // Always needed for core functionality
#include "_init.h" // Always needed for device initialization

// Essential includes for _init.c functionality
#if defined(ASSEMBLY_EXAMPLES_ACTIVE) && ASSEMBLY_EXAMPLES_ACTIVE == 1
#include "_port.h"
#include "_uart.h"
#include "_timer2.h"
#include "_interrupt.h"
#include "_adc.h"
#include "_glcd.h"
#endif

#if defined(COMMUNICATION_ACTIVE) && COMMUNICATION_ACTIVE == 1
#include "_uart.h"
#endif

#if defined(ADVANCED_EXAMPLES_ACTIVE) && ADVANCED_EXAMPLES_ACTIVE == 1
#include "_adc.h"
#include "_buzzer.h"
#include "_timer2.h"
#include "_interrupt.h"
#include "_eeprom.h"
#include "_glcd.h"
#endif

// Always include basic functionality for current example
#ifdef ASSEMBLY_BLINK_BASIC
#include "_port.h"
#include "_uart.h"
#include "_timer2.h"
#include "_interrupt.h"
#include "_adc.h"
#include "_glcd.h"
#endif

/*
 * ============================================================================
 * FEATURE DETECTION AND CAPABILITY MACROS
 * Optimized feature detection for efficient compilation
 * ============================================================================
 */

// Detect if any assembly examples are active
#if defined(ASSEMBLY_BLINK_BASIC) || defined(ASSEMBLY_BLINK_PATTERN) ||      \
    defined(ASSEMBLY_BLINK_INDIVIDUAL) || defined(ASSEMBLY_PORT_ROTATION) || \
    defined(ASSEMBLY_BUTTON_SIMPLE) || defined(ASSEMBLY_BUTTON_LED_CONTROL)
#define ASSEMBLY_EXAMPLES_ACTIVE 1
#else
#define ASSEMBLY_EXAMPLES_ACTIVE 0
#endif

// Detect if any C abstraction examples are active
#if defined(C_LED_BASIC) || defined(C_LED_PATTERNS) ||             \
    defined(C_LED_BUTTON_INTERACTIVE) || defined(C_TIMER_BASIC) || \
    defined(C_TIMER_INTERRUPT) || defined(C_TIMER_PWM)
#define C_ABSTRACTION_ACTIVE 1
#else
#define C_ABSTRACTION_ACTIVE 0
#endif

// Detect if any communication examples are active
#if defined(SERIAL_POLLING_SINGLE_CHAR) || defined(SERIAL_INTERRUPT_RX) || \
    defined(ADC_BASIC_READING) || defined(BUZZER_BASIC_BEEP) ||            \
    defined(INTERRUPT_EXTERNAL)
#define COMMUNICATION_ACTIVE 1
#else
#define COMMUNICATION_ACTIVE 0
#endif

// Detect if any advanced examples are active
#if defined(GRAPHICS_BASICS) || defined(MOTORS_DC_PWM) || \
    defined(GAME_SIMON_SAYS) || defined(MEMORY_BASIC_EEPROM)
#define ADVANCED_EXAMPLES_ACTIVE 1
#else
#define ADVANCED_EXAMPLES_ACTIVE 0
#endif

// Detect if Python/IoT examples are active
#if defined(PYTHON_BASIC_PROTOCOL) || defined(IOT_SENSOR_MONITORING) || defined(IOT)
#define PYTHON_IOT_ACTIVE 1
#else
#define PYTHON_IOT_ACTIVE 0
#endif

/*
 * OPTIMIZATION FEATURE FLAGS
 */
#if ASSEMBLY_EXAMPLES_ACTIVE
#define ENABLE_DIRECT_REGISTER_ACCESS 1
#else
#define ENABLE_DIRECT_REGISTER_ACCESS 0
#endif

#if COMMUNICATION_ACTIVE || PYTHON_IOT_ACTIVE || defined(EDUCATIONAL_DEMO)
#define ENABLE_UART_FEATURES 1
#else
#define ENABLE_UART_FEATURES 0
#endif

#if ADVANCED_EXAMPLES_ACTIVE || defined(EDUCATIONAL_DEMO)
#define ENABLE_GRAPHICS_FEATURES 1
#else
#define ENABLE_GRAPHICS_FEATURES 0
#endif

/*
 * ============================================================================
 * PHASE 1: ASSEMBLY FUNDAMENTALS - OPTIMIZED
 * Direct register manipulation, bit operations, basic I/O
 * Learn: DDR, PORT, PIN registers, bit manipulation, timing loops
 * Memory-optimized configurations for educational progression
 * ============================================================================
 */

/*
 * 1.1 Basic LED Control (Assembly → C Register Access)
 * Optimized for minimal memory footprint while preserving educational value
 */
/*
 * ============================================================================
 * ASSEMBLY EXAMPLE DEFINITIONS
 * Phase 1: Direct hardware register access
 * ============================================================================
 */

// #define ASSEMBLY_BLINK_BASIC // Simple LED blinking with PORTB

// Enable only minimal features for assembly example
#ifdef ASSEMBLY_BLINK_BASIC
#undef ENABLE_INTERRUPTS
#undef ENABLE_TIMERS
#undef ENABLE_UART
#undef ENABLE_ADC
#undef ENABLE_LCD
#endif
#define ASSEMBLY_BLINK_PATTERN // LED patterns with bit manipulation
// #define ASSEMBLY_BLINK_INDIVIDUAL       // Individual LED control
// #define PORT_ROTATION // Rotating LED patterns

/*
 * 1.2 Button Input and Control
 * Optimized input handling with debouncing considerations
 */
// #define ASSEMBLY_BUTTON_SIMPLE          // Read single button state
// #define ASSEMBLY_BUTTON_LED_CONTROL     // Control LEDs with buttons

/*
 * ============================================================================
 * PHASE 2: C HARDWARE ABSTRACTION - OPTIMIZED
 * Introduction of C functions that wrap assembly concepts
 * Learn: Function calls, parameters, return values, abstraction layers
 * Compiler-optimized function calls for educational efficiency
 * ============================================================================
 */

/*
 * 2.1 Port Control with C Functions
 * Optimized C abstraction layer with inline function support
 */
// COMMUNICATION PROTOCOLS
// #define BUZZER_BASIC_BEEP                    // Basic buzzer beep sounds
// #define C_TIMER_BASIC // Basic timer functionality
// #define C_LED_PATTERNS                  // Pattern generation with C loops
// #define C_LED_BUTTON_INTERACTIVE        // Interactive LED control

/*
 * 2.2 Timer and PWM Control
 * Optimized timer configurations for precise educational timing
 */
// // #define C_TIMER_BASIC                   // Timer initialization with C
// #define C_TIMER_INTERRUPT               // Timer interrupts and ISRs
// #define C_TIMER_PWM // PWM generation

/*
 * ============================================================================
 * PHASE 3: COMMUNICATION & SENSORS
 * Data exchange, analog input, sensor interfacing
 * Learn: Communication protocols, data conversion, sensor integration
 * ============================================================================
 */

/*
 * 3.1 UART Communication - TESTING ONE BY ONE
 * Current test: Disabled for individual example testing
 */
// #define EDUCATIONAL_DEMO // Complete educational demonstration - DISABLED FOR TESTING

// UART polling examples
// #define SERIAL_POLLING_SINGLE_CHAR      // Single character communication
// #define SERIAL_POLLING_STRING // String communication
// #define SERIAL_POLLING_ECHO             // Echo received characters
// #define SERIAL_POLLING_SENTENCE         // Sentence processing
// #define SERIAL_POLLING_WORD             // Word processing

// UART interrupt examples
// #define SERIAL_INTERRUPT_RX             // Interrupt-based receiving
// #define SERIAL_INTERRUPT_TX             // Interrupt-based transmission
// #define SERIAL_INTERRUPT_ECHO           // Interrupt-based echo
// #define SERIAL_INTERRUPT_SENTENCE       // Interrupt sentence processing
// #define SERIAL_INTERRUPT_CIRCULAR_BUFFER // Circular buffer implementation
// #define SERIAL_INTERRUPT_WORD           // Interrupt word processing

/*
 * 3.2 Analog Input and Sensors
 */
// #define ADC_BASIC_READING               // Simple ADC value reading
// #define ADC_VOLTAGE_CONVERSION          // Convert ADC to voltage
// #define ADC_MULTIPLE_CHANNELS           // Read multiple sensors

// ADC with communication examples
// #define ADC_INTERRUPT                   // ADC with interrupts
// #define ADC_INTERRUPT_UART_INTERRUPT    // ADC + UART interrupts
// #define ADC_INTERRUPT_UART_POLLING      // ADC interrupts + UART polling
// #define ADC_POLLING                     // ADC polling mode
// #define ADC_POLLING_UART_POLLING        // ADC + UART polling
// #define ADC_POLLING_UART_INTERRUPT      // ADC polling + UART interrupts

/*
 * 3.3 Audio Output and Feedback
 */
// #define BUZZER_BASIC_BEEP               // Simple beep generation
// #define BUZZER_MUSICAL_NOTES            // Musical note generation
// #define BUZZER_MELODY_PLAYER            // Play melodies

/*
 * 3.4 External Interrupts
 */
// #define INTERRUPT_EXTERNAL              // External interrupt handling
// #define INTERRUPT_LAB                   // Laboratory interrupt exercises
// #define INTERRUPT_TIMER                 // Timer-based interrupts
// #define INTERRUPT_TIMER_CTC             // Timer CTC interrupts
// #define INTERRUPT_EXT_TIMER             // External + Timer interrupts

/*
 * ============================================================================
 * PHASE 4: ADVANCED APPLICATIONS
 * Complex projects combining multiple concepts
 * Learn: System integration, state machines, complex algorithms
 * ============================================================================
 */

/*
 * 4.1 Graphics and Display
 */
// #define GRAPHICS_BASIC_SHAPES           // Draw basic shapes on GLCD
// #define GRAPHICS_ANIMATION              // Simple animations
// #define GRAPHICS_BASIC_SHAPES           // Draw basic shapes on GLCD
// #define GRAPHICS_ANIMATION              // Simple animations
// #define GRAPHICS_SENSOR_DISPLAY         // Display sensor data graphically

// Additional graphics examples
// #define GRAPHICS_BASICS                 // Basic graphics operations
// #define GRAPHICS_MOVEMENT               // Moving graphics elements
// #define GRAPHICS_RANDOM // Random graphics patterns
// // #define GRAPHICS_BOUNCING_BALL          // Bouncing ball animation
// #define GRAPHICS_MOVING_SQUARE          // Moving square animation
// #define GRAPHICS_SINE_WAVE // Sine wave visualization

/*
 * 4.2 Motor Control Systems
 */
// #define MOTORS_DC_PWM                   // DC motor speed control
// #define MOTORS_SERVO_BASIC              // Servo motor positioning
// #define MOTORS_STEPPER_BASIC // Stepper motor control

// ========================================
// PHASE 8: INTERACTIVE GAMES SERIES
// ========================================
// #define GAME_SIMON_SAYS                 // Memory game with LEDs/buttons
// #define GAME_REACTION_TIMER             // Reaction time measurement
// #define GAME_SENSOR_TARGET              // Target practice with sensors
// #define GAME_HANGMAN                    // Hangman word guessing game
// #define GAME_OBSTACLE                   // Obstacle avoidance game

/*
 * 4.3 Interactive Games
 */
// #define GAME_SIMON_SAYS                 // Memory game with LEDs/buttons
// #define GAME_REACTION_TIMER             // Reaction time measurement
// #define GAME_SENSOR_TARGET              // Target practice with sensors

/*
 * 4.4 Data Logging and Memory
 */
// Memory Management Series (New Architecture)
// #define MEMORY_BASIC                    // Basic memory operations and concepts
// #define MEMORY_STACK                    // Stack memory management
// #define MEMORY_HEAP                     // Heap simulation and management

// EEPROM Data Storage Series (New Architecture)
// #define EEPROM_BASIC                    // Basic EEPROM operations
// #define EEPROM_LOGGER                   // Data logging to EEPROM
// #define EEPROM_SETTINGS // Settings storage in EEPROM

// EEPROM Data Storage Series
// #define EEPROM_BASIC                    // Basic EEPROM read/write operations
// #define EEPROM_LOGGER                   // Data logging to EEPROM
// #define EEPROM_SETTINGS                 // Configuration settings storage

/*
 * ============================================================================
 * PHASE 5: PYTHON INTEGRATION & IoT
 * High-level programming interface and IoT applications
 * Learn: Serial protocols, data parsing, web interfaces
 * ============================================================================
 */

/*
 * 5.1 Python Communication Interface
 */
// #define PYTHON_BASIC_PROTOCOL           // Simple command/response protocol
// #define PYTHON_SENSOR_STREAMING         // Continuous sensor data streaming
// #define PYTHON_REMOTE_CONTROL           // Remote control via Python

/*
 * 5.2 IoT and Web Interface
 */
// #define IOT_SENSOR_MONITORING           // Send sensor data to Python
// #define IOT_REMOTE_CONTROL              // Control hardware via web
// #define IOT_DATA_VISUALIZATION // Real-time data plotting

/*
 * ============================================================================
 * MISCELLANEOUS EXAMPLES
 * Special purpose examples and testing
 * ============================================================================
 */

// Specialized sensors and input devices
// #define CDS                             // Light sensor (CdS cell)
// #define JOYSTICK                        // Analog joystick input
// #define ACCELEROMETER                   // 3-axis accelerometer

// Development and testing
// #define INLINE                          // Inline function examples
// #define EDUCATIONAL_SIMPLE_TEST         // Simple educational test
// #define ASSEMBLY_PROGRESSION_EXAMPLE    // Assembly progression demo

/*
 * ============================================================================
 * OPTIMIZED EDUCATIONAL VALIDATION SYSTEM
 * Enhanced validation with detailed feedback and optimization suggestions
 * ============================================================================
 */

// Advanced validation using preprocessor counter
#ifdef __COUNTER__
#define COUNT_START __COUNTER__

// Optimized counting system for all possible defines
#ifdef EDUCATIONAL_DEMO
#define _COUNT_EDUCATIONAL 1
#else
#define _COUNT_EDUCATIONAL 0
#endif

#ifdef ASSEMBLY_BLINK_BASIC
#define _COUNT_ASSEMBLY_1 1
#else
#define _COUNT_ASSEMBLY_1 0
#endif

#ifdef ASSEMBLY_BLINK_PATTERN
#define _COUNT_ASSEMBLY_2 1
#else
#define _COUNT_ASSEMBLY_2 0
#endif

#ifdef ASSEMBLY_BLINK_INDIVIDUAL
#define _COUNT_ASSEMBLY_3 1
#else
#define _COUNT_ASSEMBLY_3 0
#endif

#ifdef PORT_ROTATION
#define _COUNT_ASSEMBLY_4 1
#else
#define _COUNT_ASSEMBLY_4 0
#endif

#ifdef ASSEMBLY_BUTTON_SIMPLE
#define _COUNT_ASSEMBLY_5 1
#else
#define _COUNT_ASSEMBLY_5 0
#endif

#ifdef ASSEMBLY_BUTTON_LED_CONTROL
#define _COUNT_ASSEMBLY_6 1
#else
#define _COUNT_ASSEMBLY_6 0
#endif

#ifdef C_LED_BASIC
#define _COUNT_C_1 1
#else
#define _COUNT_C_1 0
#endif

#ifdef SERIAL_POLLING_SINGLE_CHAR
#define _COUNT_COMM_1 1
#else
#define _COUNT_COMM_1 0
#endif

#ifdef GRAPHICS_BASICS
#define _COUNT_ADVANCED_1 1
#else
#define _COUNT_ADVANCED_1 0
#endif

#ifdef PYTHON_BASIC_PROTOCOL
#define _COUNT_PYTHON_1 1
#else
#define _COUNT_PYTHON_1 0
#endif

// Calculate total active examples
#define COUNT_END __COUNTER__
#define ACTIVE_COUNT (_COUNT_EDUCATIONAL + _COUNT_ASSEMBLY_1 + _COUNT_ASSEMBLY_2 + _COUNT_ASSEMBLY_3 + _COUNT_ASSEMBLY_4 + _COUNT_ASSEMBLY_5 + _COUNT_ASSEMBLY_6 + _COUNT_C_1 + _COUNT_COMM_1 + _COUNT_ADVANCED_1 + _COUNT_PYTHON_1)

// Enhanced validation with optimization suggestions
#if ACTIVE_COUNT > 1
#error "OPTIMIZATION ERROR: Multiple examples selected! This reduces compilation efficiency and increases memory usage. Please enable only ONE #define at a time for optimal learning progression and memory usage."
#elif ACTIVE_COUNT == 0
#warning "OPTIMIZATION NOTICE: No example selected. Uncomment one #define to compile an educational example. Current configuration uses minimal memory footprint."
#else
// Provide optimization feedback for active configuration
#if defined(EDUCATIONAL_DEMO)
#pragma message "OPTIMIZATION: Educational demo active - using comprehensive library set for maximum learning value."
#elif ASSEMBLY_EXAMPLES_ACTIVE
#pragma message "OPTIMIZATION: Assembly examples active - minimal library usage for efficient learning."
#elif C_ABSTRACTION_ACTIVE
#pragma message "OPTIMIZATION: C abstraction active - balanced library usage for function-based learning."
#elif COMMUNICATION_ACTIVE
#pragma message "OPTIMIZATION: Communication examples active - UART and sensor libraries enabled."
#elif ADVANCED_EXAMPLES_ACTIVE
#pragma message "OPTIMIZATION: Advanced examples active - full feature set enabled for complex projects."
#elif PYTHON_IOT_ACTIVE
#pragma message "OPTIMIZATION: Python/IoT examples active - communication-focused configuration."
#endif
#endif

// Memory usage estimation (rough calculation based on active features)
#if defined(EDUCATIONAL_DEMO)
#define ESTIMATED_FLASH_USAGE "~11KB (Full Educational Suite)"
#elif ASSEMBLY_EXAMPLES_ACTIVE
#define ESTIMATED_FLASH_USAGE "~2-3KB (Minimal Assembly)"
#elif C_ABSTRACTION_ACTIVE
#define ESTIMATED_FLASH_USAGE "~4-5KB (C Functions)"
#elif COMMUNICATION_ACTIVE
#define ESTIMATED_FLASH_USAGE "~6-7KB (Communication)"
#elif ADVANCED_EXAMPLES_ACTIVE
#define ESTIMATED_FLASH_USAGE "~8-9KB (Advanced Features)"
#elif PYTHON_IOT_ACTIVE
#define ESTIMATED_FLASH_USAGE "~7-8KB (IoT Communication)"
#else
#define ESTIMATED_FLASH_USAGE "~1KB (Base System)"
#endif

#ifdef ESTIMATED_FLASH_USAGE
#pragma message "MEMORY OPTIMIZATION: Estimated flash usage: " ESTIMATED_FLASH_USAGE
#pragma message "MEMORY OPTIMIZATION: Conditional includes active - unused libraries excluded"
#endif

#endif /* __COUNTER__ */

/*
 * ============================================================================
 * PERFORMANCE OPTIMIZATION MACROS
 * Compile-time optimizations for educational framework
 * ============================================================================
 */

// Optimization level detection and suggestions
#ifndef __OPTIMIZE__
#pragma message "PERFORMANCE TIP: Consider enabling compiler optimizations (-Os for size, -O2 for speed) for better performance."
#endif

#ifdef __OPTIMIZE_SIZE__
#pragma message "OPTIMIZATION: Size optimization enabled - excellent for learning resource constraints."
#endif

#ifdef __OPTIMIZE__
#pragma message "OPTIMIZATION: Speed optimization enabled - good for real-time applications."
#endif

// Educational framework optimization flags
#if defined(EDUCATIONAL_DEMO) || defined(FAST_COMPILATION)
#define INLINE_SMALL_FUNCTIONS 1
#define OPTIMIZE_GRAPHICS_CALLS 1
#define ENABLE_FAST_UART 1
#else
#define INLINE_SMALL_FUNCTIONS 0
#define OPTIMIZE_GRAPHICS_CALLS 0
#define ENABLE_FAST_UART 0
#endif

// Debug vs Release optimization
#ifdef DEBUG_MODE
#define ENABLE_DEBUG_FEATURES 1
#define OPTIMIZE_FOR_DEBUGGING 1
#pragma message "DEBUG MODE: Full debugging features enabled - larger memory usage but better learning experience."
#else
#define ENABLE_DEBUG_FEATURES 0
#define OPTIMIZE_FOR_DEBUGGING 0
#pragma message "RELEASE MODE: Optimized for performance and memory efficiency."
#endif

#endif /* CONFIG_H_ */
