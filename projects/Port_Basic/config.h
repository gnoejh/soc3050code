/*
 * Configuration Header - Port Project (Level 1: Basic)
 * ATmega128 Educational Framework
 * Fundamental Port Programming Concepts
 *
 * This project teaches BASIC port programming:
 * - DDR register (Data Direction Register)
 * - PORT register (Output data and pull-up control)
 * - PIN register (Input reading)
 * - Simple bit manipulation
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#ifndef F_CPU
#define F_CPU 16000000UL // 16MHz crystal
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#endif /* CONFIG_H_ */