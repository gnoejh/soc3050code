/*
 * Configuration Header - Port_Configuration Project (Level 3: Advanced)
 * ATmega128 Educational Framework
 * Advanced Port Control with Serial Monitoring
 *
 * This project teaches ADVANCED concepts:
 * - Multiple port coordination
 * - Interactive configuration via UART
 * - Complex pattern generation
 * - Real-time monitoring and debugging
 * - Professional system architecture
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#ifndef F_CPU
#define F_CPU 16000000UL // 16MHz crystal
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

// UART Configuration
// For 16MHz crystal, 9600 baud is reliable
// UBRR = (16000000 / (16 * 9600)) - 1 = 103
#define BAUD 9600
#define UART_BAUD 9600
#define UART_UBRR_VALUE ((F_CPU / (16UL * UART_BAUD)) - 1)

#endif /* CONFIG_H_ */
