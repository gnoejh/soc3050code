/*
 * Enhanced UART Library Header
 * ATmega128 Educational Framework
 */

#ifndef UART_ENHANCED_H_
#define UART_ENHANCED_H_

#include <stdint.h>

// Error codes
#define UART_NO_ERROR 0x00
#define UART_FRAME_ERROR 0x01
#define UART_DATA_OVERRUN 0x02
#define UART_PARITY_ERROR 0x04
#define UART_BUFFER_OVERFLOW 0x08
#define UART_TIMEOUT_ERROR 0x10

// Enhanced initialization
uint8_t uart_enhanced_init(uint32_t baud_rate, uint8_t data_bits, uint8_t parity, uint8_t stop_bits);

// Enhanced communication
uint8_t uart_enhanced_receive(uint8_t *data, uint16_t timeout_ms);
uint8_t uart_enhanced_transmit(uint8_t data);
uint8_t uart_enhanced_transmit_string(const char *str);
int uart_enhanced_printf(const char *format, ...);

// Buffer management
uint8_t uart_enhanced_rx_available(void);
uint8_t uart_enhanced_tx_free(void);

// Error handling
uint8_t uart_enhanced_get_error_flags(void);
void uart_enhanced_clear_error_flags(void);
uint8_t uart_enhanced_get_last_error(void);

// Statistics
uint16_t uart_enhanced_get_bytes_received(void);
uint16_t uart_enhanced_get_bytes_transmitted(void);
void uart_enhanced_reset_statistics(void);

// Configuration
uint32_t uart_enhanced_get_baud_rate(void);
void uart_enhanced_get_config(uint32_t *baud, uint8_t *data_bits, uint8_t *parity, uint8_t *stop_bits);

// Diagnostics
void uart_enhanced_print_status(void);
void uart_enhanced_test_baud_rates(void);
void uart_enhanced_demo(void);

// Backward compatibility
void Uart1_init(void);
uint8_t is_USART1_received(void);
uint8_t get_USART1(void);
void put_USART1(uint8_t data);
void puts_USART1(const char *str);

#endif /* UART_ENHANCED_H_ */