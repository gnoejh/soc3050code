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

// Include shared library headers
// #include "_timer2.h"  // Not needed - using Timer0 only
#include "_init.h"
// #include "_uart.h"    // Removed - causes SimulIDE crashes

// External function declarations for demo programs
extern void demo1_normal_mode(void);
extern void demo2_ctc_mode(void);
extern void demo3_fast_pwm(void);
extern void demo4_phase_correct_pwm(void);
extern void demo5_prescaler_comparison(void);
extern void demo6_interrupt_multitask(void);
extern void demo7_timer1_advanced(void);

#endif /* CONFIG_H_ */