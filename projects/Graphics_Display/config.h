/*
 * Configuration Header - Graphics Display
 * ATmega128 Educational Framework
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// F_CPU is defined by build script (-DF_CPU=16000000UL)
// BAUD is defined by build script (-DBAUD=9600)

// Enable Comprehensive Graphics Test
#define GRAPHICS_COMPREHENSIVE_TEST

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <time.h>

// Include shared library headers
#include "../../shared_libs/_port.h"
#include "../../shared_libs/_init.h"
#include "../../shared_libs/_glcd.h"
#include "../../shared_libs/_buzzer.h"

#endif /* CONFIG_H_ */
