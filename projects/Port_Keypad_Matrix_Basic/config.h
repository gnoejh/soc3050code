#ifndef CONFIG_H
#define CONFIG_H

#define F_CPU 7372800UL
#define BAUD 9600

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include "../../shared_libs/_uart.h"
#include "../../shared_libs/_glcd.h"

// LCD declarations (if not using full _glcd library)
void lcd_init(void);
void lcd_clear(void);
void lcd_goto(uint8_t row, uint8_t col);
void lcd_puts(const char *str);
void lcd_puts_at(uint8_t row, uint8_t col, const char *str);
void lcd_data(uint8_t data);

#endif
