
/*
 * _timer2.h - ATmega128 Timer2 Library Header
 * Educational Version for Assembly→C→Python Learning Progression
 */

#ifndef _TIMER2_H_
#define _TIMER2_H_

/*
 * Core Timer2 Functions - Basic Timing Operations
 */
void Timer2_init(void);  // Initialize Timer2 for 1ms interrupts
void Timer2_start(void); // Start Timer2 operation
void Timer2_stop(void);  // Stop Timer2 operation

/*
 * Advanced Timer2 Functions - Custom Timing Control
 */
void Timer2_set_prescaler(unsigned char prescaler);   // Change timer frequency
void Timer2_set_period_ms(unsigned int period_ms);    // Set timer period in milliseconds
unsigned long Timer2_get_milliseconds(void);          // Get system uptime in ms
unsigned char Timer2_delay_ms(unsigned int delay_ms); // Non-blocking delay function

/*
 * Task Management Functions - Real-Time Scheduling
 */
unsigned char Timer2_check_task1(void); // Check and clear Task 1 flag
unsigned char Timer2_check_task2(void); // Check and clear Task 2 flag
unsigned char Timer2_check_task3(void); // Check and clear Task 3 flag

/*
 * Global Variables for Educational Use
 * These demonstrate timer-based programming and real-time scheduling
 */
extern volatile unsigned int Count_Of_Timer2; // Main timer counter
extern volatile unsigned int Task1_Of_Timer2; // Task 1 ready flag
extern volatile unsigned int Task2_Of_Timer2; // Task 2 ready flag
extern volatile unsigned int Task3_Of_Timer2; // Task 3 ready flag
extern unsigned int Time_Of_Timer2;           // Task 1 interval (default 500ms)
extern unsigned int Time2_Of_Timer2;          // Task 2 interval (default 100ms)
extern unsigned int Time3_Of_Timer2;          // Task 3 interval (default 1000ms)

extern volatile unsigned long system_milliseconds; // System uptime counter
extern volatile unsigned int timer2_prescaler;     // Current prescaler setting
extern unsigned char timer2_start_value;           // Current timer start value

/*
 * Timer2 Constants for Educational Reference
 */
#define TIMER2_MAX_COUNT 255        // Maximum 8-bit timer value
#define TIMER2_OVERFLOW_FREQ 250000 // Timer frequency with prescaler 64 (16MHz/64)
#define TIMER2_1MS_TICKS 250        // Ticks needed for 1ms (250000Hz / 1000)

/*
 * Prescaler Constants (redefined for header access)
 */
#define TIMER2_STOP 0x00          // Timer stopped
#define TIMER2_PRESCALE_1 0x01    // No prescaling (16MHz)
#define TIMER2_PRESCALE_8 0x02    // Prescaler 8 (2MHz)
#define TIMER2_PRESCALE_32 0x03   // Prescaler 32 (500kHz)
#define TIMER2_PRESCALE_64 0x04   // Prescaler 64 (250kHz) - default
#define TIMER2_PRESCALE_128 0x05  // Prescaler 128 (125kHz)
#define TIMER2_PRESCALE_256 0x06  // Prescaler 256 (62.5kHz)
#define TIMER2_PRESCALE_1024 0x07 // Prescaler 1024 (15.625kHz)

/*
 * Common Timing Intervals (in timer ticks at 1ms per tick)
 */
#define TIMER2_INTERVAL_10MS 10   // 10 milliseconds
#define TIMER2_INTERVAL_50MS 50   // 50 milliseconds
#define TIMER2_INTERVAL_100MS 100 // 100 milliseconds
#define TIMER2_INTERVAL_250MS 250 // 250 milliseconds
#define TIMER2_INTERVAL_500MS 500 // 500 milliseconds (default)
#define TIMER2_INTERVAL_1SEC 1000 // 1 second
#define TIMER2_INTERVAL_2SEC 2000 // 2 seconds
#define TIMER2_INTERVAL_5SEC 5000 // 5 seconds

/*
 * Task Priority Definitions for Educational Use
 */
#define TIMER2_TASK_HIGH_FREQ TIMER2_INTERVAL_10MS    // High frequency tasks
#define TIMER2_TASK_MEDIUM_FREQ TIMER2_INTERVAL_100MS // Medium frequency tasks
#define TIMER2_TASK_LOW_FREQ TIMER2_INTERVAL_1SEC     // Low frequency tasks

/*
 * Function Prototypes for Educational Examples
 * These are implemented in separate main_*.c files
 */
void main_timer2_basic_timing(void);    // Basic timer usage example
void main_timer2_multi_task(void);      // Multi-task scheduling example
void main_timer2_pwm_simulation(void);  // Software PWM using timer
void main_timer2_stopwatch(void);       // Stopwatch application
void main_timer2_real_time_clock(void); // Real-time clock example
void main_timer2_scheduler(void);       // Task scheduler example

/*
 * Interrupt callback to be called from the application's ISR(TIMER2_OVF_vect)
 */
void Timer2_ovf_handler(void);

#endif // _TIMER2_H_
