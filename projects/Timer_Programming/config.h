/*
 * Configuration Header - Timer Programming
 * ATmega128 Educational Framework
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
// stdio.h removed - causes UART init issues in SimulIDE
// #include <stdio.h>
// #include <string.h>

// Include shared library headers (compiler uses -I../../shared_libs)
// Note: Main.c uses minimal local init for educational clarity
// #include "_init.h"    // Not needed for Main.c - using local simple_init()

// Lab.c uses full integration - includes all libraries
#include "_init.h"    // System initialization
#include "_uart.h"    // Serial communication
#include "_glcd.h"    // Graphics LCD
#include "_port.h"    // Button handling

// Forward declarations for demo functions (defined in Main.c)
void simple_init(void); // Local minimal initialization (Main.c only)
void demo0_led_test(void);
void demo1_normal_polling(void);
void demo2_normal_interrupt(void);
void demo3_ctc_polling(void);
void demo4_ctc_interrupt(void);
void demo5_fast_pwm(void);
void demo6_phase_correct_pwm(void);
void demo7_prescaler_comparison(void);
void demo8_multitask_interrupt(void);

#endif /* CONFIG_H_ */