/*
 * config.h - Educational Configuration for ATmega128 Learning Progression
 * Assembly → C → Python Teaching Sequence
 *
 * USAGE INSTRUCTIONS:
 * 1. Uncomment ONE #define from the desired learning phase
 * 2. Compile with build.ps1 or build.bat
 * 3. Program with program.ps1 or program.bat
 * 4. Progress through phases in order for optimal learning
 *
 * LEARNING PROGRESSION:
 * Phase 1: Assembly Fundamentals (direct register access)
 * Phase 2: C Hardware Abstraction (library functions)
 * Phase 3: Communication & Sensors (data exchange)
 * Phase 4: Advanced Applications (complex projects)
 * Phase 5: Python Integration (high-level programming)
 */

#ifndef CONFIG_H_
#define CONFIG_H_

/*
 * SYSTEM CONFIGURATION
 * Fixed settings for ATmega128 development board
 */
#define F_CPU 16000000UL // 16 MHz crystal oscillator
#define BAUD 9600        // Standard UART baud rate for education

/*
 * SYSTEM INCLUDES
 * Required libraries for all examples
 */
#include <avr/io.h>        // Hardware register definitions
#include <avr/interrupt.h> // Interrupt handling
#include <util/delay.h>    // Delay functions
#include <stdio.h>         // Standard I/O (sprintf, etc.)
#include <string.h>        // String manipulation
#include <stdlib.h>        // Standard library functions

/*
 * PROJECT MODULES
 * Choose optimized or original library versions
 */

// Use optimized educational libraries (recommended for new students)
#define USE_OPTIMIZED_LIBRARIES

#ifdef USE_OPTIMIZED_LIBRARIES
#include "_main.h"
#include "_port_optimized.h"
#include "_uart_optimized.h"
#include "_init_optimized.h"
#include "_adc_optimized.h"
#include "_buzzer_optimized.h"
#include "_timer2.h"    // TO DO: Create optimized version
#include "_interrupt.h" // TO DO: Create optimized version
#include "_eeprom.h"    // TO DO: Create optimized version
#include "_glcd.h"      // TO DO: Create simplified version
#else
// Original libraries (for compatibility with existing code)
#include "_main.h"
#include "_port.h"
#include "_uart.h"
#include "_init.h"
#include "_adc.h"
#include "_buzzer.h"
#include "_timer2.h"
#include "_interrupt.h"
#include "_eeprom.h"
#include "_glcd.h"
#endif

/*
 * ============================================================================
 * PHASE 1: ASSEMBLY FUNDAMENTALS
 * Direct register manipulation, bit operations, basic I/O
 * Students learn: DDR, PORT, PIN registers, bit manipulation, timing loops
 * ============================================================================
 */

/*
 * 1.1 Basic Port Control (Assembly → C Register Access)
 * Learn: DDRB, PORTB, bit manipulation, basic timing
 */
// #define ASSEMBLY_BLINK_BASIC            // Simple LED blinking with PORTB
// #define ASSEMBLY_BLINK_PATTERN          // LED patterns with bit manipulation
// #define ASSEMBLY_BLINK_INDIVIDUAL       // Individual LED control with bit operations
// #define ASSEMBLY_PORT_ROTATION          // Rotating LED patterns

/*
 * 1.2 Button Input (Assembly → C Input Reading)
 * Learn: PIN registers, pull-up resistors, input debouncing
 */
// #define ASSEMBLY_BUTTON_SIMPLE          // Read single button state
// #define ASSEMBLY_BUTTON_MULTIPLE        // Read multiple button states
// #define ASSEMBLY_BUTTON_LED_CONTROL     // Control LEDs with buttons

/*
 * 1.3 Timer Basics (Assembly → C Timer Registers)
 * Learn: TCNT, TCCR, timer prescalers, overflow concepts
 */
// #define ASSEMBLY_TIMER_BASIC            // Basic timer configuration
// #define ASSEMBLY_TIMER_OVERFLOW         // Timer overflow detection
// #define ASSEMBLY_TIMER_PRESCALER        // Different prescaler settings

/*
 * ============================================================================
 * PHASE 2: C HARDWARE ABSTRACTION
 * Introduction of C functions that wrap assembly concepts
 * Students learn: Function calls, parameters, return values, abstraction layers
 * ============================================================================
 */

/*
 * 2.1 Port Control with C Functions
 * Learn: Function calls vs direct register access, abstraction benefits
 */
// #define C_LED_BASIC                     // LED control using C functions
// #define C_LED_PATTERNS                  // Pattern generation with C loops
// #define C_LED_BUTTON_INTERACTIVE        // Interactive LED control

/*
 * 2.2 Timer Abstraction
 * Learn: C timer initialization, interrupt service routines
 */
// #define C_TIMER_BASIC                   // Timer initialization with C
// #define C_TIMER_INTERRUPT               // Timer interrupts and ISRs
// #define C_TIMER_PWM                     // PWM generation for analog output

/*
 * 2.3 Basic Communication
 * Learn: UART registers → C communication functions
 */
// #define C_UART_BASIC                    // Basic UART initialization and usage
// #define C_UART_ECHO                     // Character echo (currently active)
// #define C_UART_STRINGS                  // String transmission and reception

// CURRENTLY ACTIVE EXAMPLE
#define SERIAL_POLLING_ECHO // Echo received characters back

/*
 * ============================================================================
 * PHASE 3: COMMUNICATION & SENSORS
 * Data exchange, analog input, sensor interfacing
 * Students learn: Communication protocols, data conversion, sensor integration
 * ============================================================================
 */

/*
 * 3.1 UART Communication Patterns
 * Learn: Different communication methods, data formatting
 */
// #define UART_POLLING_CHAR               // Single character communication
// #define UART_POLLING_STRING             // String communication
// #define UART_POLLING_SENTENCE           // Sentence processing
// #define UART_INTERRUPT_RX               // Interrupt-based receiving
// #define UART_INTERRUPT_TX               // Interrupt-based transmission
// #define UART_INTERRUPT_FULL             // Full duplex interrupt communication

/*
 * 3.2 Analog Input and Sensors
 * Learn: ADC configuration, voltage references, sensor interfacing
 */
// #define ADC_BASIC_READING               // Simple ADC value reading
// #define ADC_VOLTAGE_CONVERSION          // Convert ADC to voltage
// #define ADC_MULTIPLE_CHANNELS           // Read multiple sensors
// #define ADC_UART_DISPLAY                // Display sensor data via UART
// #define ADC_LED_BARGRAPH                // Show analog values on LEDs

/*
 * 3.3 Audio Output and Feedback
 * Learn: PWM for audio, frequency generation, timing concepts
 */
// #define BUZZER_BASIC_BEEP               // Simple beep generation
// #define BUZZER_MUSICAL_NOTES            // Musical note generation
// #define BUZZER_MELODY_PLAYER            // Play melodies from arrays
// #define BUZZER_FREQUENCY_SWEEP          // Frequency sweep demonstration

/*
 * ============================================================================
 * PHASE 4: ADVANCED APPLICATIONS
 * Complex projects combining multiple concepts
 * Students learn: System integration, state machines, complex algorithms
 * ============================================================================
 */

/*
 * 4.1 Graphics and Display
 * Learn: Pixel manipulation, graphics algorithms, visual feedback
 */
// #define GRAPHICS_BASIC_SHAPES           // Draw basic shapes on GLCD
// #define GRAPHICS_ANIMATION              // Simple animations
// #define GRAPHICS_SENSOR_DISPLAY         // Display sensor data graphically
// #define GRAPHICS_MENU_SYSTEM            // Interactive menu system

/*
 * 4.2 Motor Control
 * Learn: PWM control, stepper motors, servo positioning
 */
// #define MOTORS_DC_PWM                   // DC motor speed control
// #define MOTORS_SERVO_BASIC              // Servo motor positioning
// #define MOTORS_STEPPER_BASIC            // Stepper motor control
// #define MOTORS_ADC_CONTROL              // Motor control via analog input

/*
 * 4.3 Interactive Games
 * Learn: Game logic, user input handling, score keeping
 */
// #define GAME_SIMON_SAYS                 // Memory game with LEDs and buttons
// #define GAME_REACTION_TIMER             // Reaction time measurement
// #define GAME_SENSOR_TARGET              // Target practice with sensors
// #define GAME_PONG_SIMPLE                // Simple Pong game

/*
 * 4.4 Data Logging and Memory
 * Learn: EEPROM usage, data persistence, logging systems
 */
// #define MEMORY_BASIC_EEPROM             // Basic EEPROM read/write
// #define MEMORY_SENSOR_LOGGING           // Log sensor data to EEPROM
// #define MEMORY_SETTINGS_STORAGE         // Store configuration settings

/*
 * ============================================================================
 * PHASE 5: PYTHON INTEGRATION
 * High-level programming interface and IoT applications
 * Students learn: Serial protocols, data parsing, web interfaces
 * ============================================================================
 */

/*
 * 5.1 Python Communication Interface
 * Learn: Structured data exchange, protocol design
 */
// #define PYTHON_BASIC_PROTOCOL           // Simple command/response protocol
// #define PYTHON_JSON_COMMUNICATION       // JSON-formatted data exchange
// #define PYTHON_SENSOR_STREAMING         // Continuous sensor data streaming
// #define PYTHON_REMOTE_CONTROL           // Remote control via Python

/*
 * 5.2 IoT and Web Interface
 * Learn: Internet connectivity concepts, web dashboards
 */
// #define IOT_SENSOR_MONITORING           // Send sensor data to Python web server
// #define IOT_REMOTE_CONTROL              // Control hardware via web interface
// #define IOT_DATA_VISUALIZATION          // Real-time data plotting
// #define IOT_ALERT_SYSTEM                // Send alerts via Python

/*
 * ============================================================================
 * EXPERIMENTAL AND ADVANCED FEATURES
 * For advanced students and special projects
 * ============================================================================
 */

/*
 * Assembly Language Integration
 * Learn: Mixing C and assembly, performance optimization
 */
// #define ADVANCED_ASM_INLINE             // Inline assembly in C code
// #define ADVANCED_ASM_FUNCTIONS          // Pure assembly functions
// #define ADVANCED_ASM_OPTIMIZATION       // Performance-critical assembly code

/*
 * System-Level Programming
 * Learn: Bootloaders, watchdog timers, power management
 */
// #define SYSTEM_WATCHDOG                 // Watchdog timer usage
// #define SYSTEM_SLEEP_MODES              // Power management
// #define SYSTEM_BOOTLOADER               // Custom bootloader concepts

/*
 * ============================================================================
 * LEGACY EXAMPLES (For Compatibility)
 * These maintain compatibility with existing course materials
 * ============================================================================
 */

// Original example names (mapped to new educational structure)
// #define BLINK_PORT                      // → ASSEMBLY_BLINK_BASIC
// #define BLINK_PIN                       // → ASSEMBLY_BLINK_INDIVIDUAL
// #define SERIAL_POLLING_SINGLE_CHAR      // → UART_POLLING_CHAR
// #define SERIAL_POLLING_STRING           // → UART_POLLING_STRING
// #define ADC_POLLING                     // → ADC_BASIC_READING
// #define GRAPHICS_BASICS                 // → GRAPHICS_BASIC_SHAPES
// #define SOUND                           // → BUZZER_BASIC_BEEP
// #define GAME_HANGMAN                    // → GAME_SIMON_SAYS
// #define IOT                             // → IOT_SENSOR_MONITORING

/*
 * ============================================================================
 * EDUCATIONAL CONFIGURATION VALIDATION
 * Ensure only one example is selected at a time
 * ============================================================================
 */

// Count active defines to ensure only one is selected
#ifdef __COUNTER__
#define COUNT_DEFINES_START __COUNTER__

// Add counter increments for each active define here
// This will be expanded as examples are implemented

#define COUNT_DEFINES_END __COUNTER__
#define ACTIVE_DEFINES (COUNT_DEFINES_END - COUNT_DEFINES_START)

#if ACTIVE_DEFINES > 1
#error "Multiple examples selected! Please enable only ONE #define at a time."
#elif ACTIVE_DEFINES == 0
#warning "No example selected. Please uncomment one #define to compile an example."
#endif
#endif

/*
 * EDUCATIONAL NOTES FOR INSTRUCTORS:
 *
 * 1. PROGRESSION SEQUENCE:
 *    Start with Phase 1 (Assembly) → Phase 2 (C) → Phase 3 (Communication)
 *    → Phase 4 (Applications) → Phase 5 (Python)
 *
 * 2. LIBRARY SELECTION:
 *    USE_OPTIMIZED_LIBRARIES recommended for new students
 *    Original libraries for compatibility with existing materials
 *
 * 3. EXAMPLE COMPLEXITY:
 *    Each phase builds on previous knowledge
 *    Examples within each phase progress from simple to complex
 *
 * 4. CUSTOMIZATION:
 *    Instructors can modify F_CPU and BAUD for different hardware
 *    Additional examples can be added following the naming convention
 *
 * 5. ASSESSMENT:
 *    Students should demonstrate mastery of each phase before advancing
 *    Phase completion can be verified through practical exercises
 */

#endif /* CONFIG_H_ */