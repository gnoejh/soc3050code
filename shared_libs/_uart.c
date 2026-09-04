/*
 * _uart.c - ATmega128 UART Communication Library
 * Educational Version for Assembly→C→Python Learning Progression
 *
 * LEARNING OBJECTIVES:
 * 1. Understand UART register configuration (UBRR, UCSR, UDR)
 * 2. Learn interrupt vs polling communication methods
 * 3. Bridge assembly register access to C abstraction
 * 4. Prepare for Python serial communication interface
 *
 * ASSEMBLY EQUIVALENT CONCEPTS:
 * - UBRR1H = high_byte  ≡  LDI R16, high_byte; STS UBRR1H, R16
 * - UDR1 = data         ≡  LDI R16, data; STS UDR1, R16
 * - Check UDRE1 flag    ≡  LDS R16, UCSR1A; SBRS R16, UDRE1
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include "_main.h"
#include "_uart.h"

// Only compile UART functions if not using self-contained assembly example
#ifndef ASSEMBLY_BLINK_BASIC

/*
 * Forward declarations for interrupt handler functions
 */
void uart_rx_interrupt_handler(void);

/*
 * UART Communication Variables
 * These demonstrate C variable usage vs assembly register manipulation
 */
unsigned char uart_command = 0;	   // Command byte from serial input
unsigned char uart_input_data = 0; // Buffer for received data
unsigned char uart_state = 0;	   // Communication state tracker
unsigned int uart_baud_register;   // Calculated baud rate register value

// Standard communication strings
char uart_newline[] = {"\r\n"}; // Carriage return + line feed
char uart_tab[] = {"\t"};		// Tab character

/*
 * Uart1_init() - Initialize UART1 for 8N1 communication
 *
 * EDUCATIONAL NOTES:
 * - UART = Universal Asynchronous Receiver/Transmitter
 * - 8N1 = 8 data bits, No parity, 1 stop bit (most common)
 * - Baud rate = bits per second (9600 is slow but reliable)
 *
 * REGISTER CONFIGURATION DETAILS:
 * - UCSR1A: UART Control and Status Register A
 * - UCSR1B: UART Control and Status Register B
 * - UCSR1C: UART Control and Status Register C
 * - UBRR1H/L: UART Baud Rate Register (High/Low bytes)
 * - UDR1: UART Data Register (transmit/receive buffer)
 */
void Uart1_init(void)
{
	// No interrupt manager: direct RX interrupt

	// Step 1: Configure UART Control Register A
	// Assembly equivalent: LDI R16, 0x00; STS UCSR1A, R16
	UCSR1A = 0x00; // U2X=0 for standard baud rate calculation

	// Step 2: Configure character format (8 data bits)
	// Assembly equivalent: LDI R16, 0x06; STS UCSR1C, R16
	UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); // UCSZ11:10 = 11 for 8-bit character size

	// Step 3: Enable transmitter and receiver with interrupt
	// Assembly equivalent: LDI R16, 0x98; STS UCSR1B, R16
	UCSR1B = (1 << RXCIE1) | (1 << RXEN1) | (1 << TXEN1);
	// RXCIE1 = RX Complete Interrupt Enable
	// RXEN1  = Receiver Enable
	// TXEN1  = Transmitter Enable

	// Step 4: Calculate and set baud rate
	// Formula: UBRR = (F_CPU / (16 * BAUD)) - 1
	// For 16MHz and 9600 baud: UBRR = (16000000 / (16 * 9600)) - 1 = 103
	uart_baud_register = F_CPU / 16 / BAUD - 1;

	// Assembly equivalent:
	// LDI R16, HIGH(103); STS UBRR1H, R16
	// LDI R16, LOW(103); STS UBRR1L, R16
	UBRR1H = (uart_baud_register >> 8); // High byte of baud rate register
	UBRR1L = uart_baud_register;		// Low byte of baud rate register
}

/*
 * putch_USART1() - Transmit single character via UART1
 *
 * EDUCATIONAL NOTES:
 * - UDRE1 = UART Data Register Empty flag
 * - Must wait for transmitter to be ready before sending next byte
 * - Polling method: continuously check flag until ready
 *
 * ASSEMBLY EQUIVALENT:
 * wait_loop:
 *   LDS R16, UCSR1A       ; Load UART status
 *   SBRS R16, UDRE1       ; Skip if UDRE1 bit is set
 *   RJMP wait_loop        ; Jump back if not ready
 *   STS UDR1, R17         ; Store data to transmit register
 */
void putch_USART1(char data)
{
	// Wait until transmitter is ready (UDRE1 flag set)
	while (!(UCSR1A & (1 << UDRE1)))
		; // Poll until data register empty

	// Send the character
	UDR1 = data; // Write data to UART Data Register
}

/*
 * puts_USART1() - Transmit string via UART1
 *
 * EDUCATIONAL NOTES:
 * - Strings in C are null-terminated (end with '\0')
 * - Pointer arithmetic: str++ moves to next character
 * - Demonstrates C abstraction over assembly character-by-character transmission
 */
void puts_USART1(char *str)
{
	while (*str != 0)
	{						// Continue until null terminator
		putch_USART1(*str); // Send current character
		str++;				// Move pointer to next character
	}
}

/*
 * getch_USART1() - Receive single character via UART1 (polling)
 *
 * EDUCATIONAL NOTES:
 * - RXC1 = Receive Complete flag
 * - Polling method: wait until character received
 * - Returns received character from UDR1 register
 */
unsigned char getch_USART1(void)
{
	// Wait until character received (RXC1 flag set)
	while (!(UCSR1A & (1 << RXC1)))
		; // Poll until receive complete

	// Return received character
	return UDR1; // Read data from UART Data Register
}

/*
 * Educational Helper Functions
 * These bridge C concepts to Python string/number handling
 */

// Convert and transmit unsigned integer as decimal
void USART1_print_decimal(unsigned int number)
{
	char buffer[6];				   // Buffer for decimal string (max 65535)
	sprintf(buffer, "%u", number); // Convert number to string
	puts_USART1(buffer);		   // Send string via UART
}

// Convert and transmit unsigned char as hexadecimal
void USART1_print_hex(unsigned char number)
{
	char buffer[3];					 // Buffer for hex string (max FF)
	sprintf(buffer, "%02X", number); // Convert to 2-digit hex string
	puts_USART1(buffer);			 // Send string via UART
}

// Send newline sequence (carriage return + line feed)
void USART1_print_newline(void)
{
	puts_USART1(uart_newline); // Send "\r\n"
}

// Educational echo function for testing
void USART1_echo_char(void)
{
	unsigned char received_char;
	received_char = getch_USART1(); // Receive character
	putch_USART1(received_char);	// Echo it back immediately
}

/*
 * UART Interrupt Service Routine - Now using centralized interrupt manager
 * Advanced topic: interrupt-driven communication vs polling
 */

volatile unsigned char uart_rx_buffer = 0; // Buffer for interrupt-received data
volatile unsigned char uart_rx_flag = 0;   // Flag indicating new data received

/*
 * UART RX interrupt handler function (callback)
 * This function will be registered with the interrupt manager
 */
void uart_rx_interrupt_handler(void)
{
	uart_rx_buffer = UDR1;		   // Read received character
	uart_rx_flag = 1;			   // Set flag for main program
	uart_command = uart_rx_buffer; // Store for command processing
}

/*
 * Check if new data received via interrupt
 * Returns: 1 if new data available, 0 if no new data
 */
unsigned char USART1_data_available(void)
{
	if (uart_rx_flag)
	{
		uart_rx_flag = 0; // Clear flag
		return 1;		  // Data available
	}
	return 0; // No new data
}

/*
 * Get data from interrupt buffer
 * Should only be called after USART1_data_available() returns 1
 */
unsigned char USART1_get_data(void)
{
	return uart_rx_buffer; // Return buffered data
}

/*
 * EDUCATIONAL PROGRESSION NOTES:
 *
 * 1. ASSEMBLY LEVEL (Direct Register Access):
 *    Students learn: LDS R16, UCSR1A; SBRS R16, UDRE1
 *    Understanding hardware flags and polling loops
 *
 * 2. C ABSTRACTION LEVEL (This Library):
 *    Students learn: putch_USART1('A'); puts_USART1("Hello");
 *    Understanding function calls and string handling
 *
 * 3. PYTHON LEVEL (High-Level Interface):
 *    Students learn: atmega.serial.write("Hello\n")
 *    Understanding object-oriented interfaces and automatic buffering
 *
 * PROGRESSION TOPICS:
 * - Polling vs Interrupt-driven communication
 * - ASCII character encoding and string handling
 * - Data formatting (decimal, hexadecimal, binary)
 * - Communication protocols and error handling
 */

/*
 * LEGACY COMPATIBILITY FUNCTIONS
 * These maintain compatibility with existing educational examples
 */

// Legacy variable names for backward compatibility
unsigned char command = 0;		   // Alias for uart_command
unsigned char InputSerialData = 0; // Legacy buffer variable
unsigned char InputSirialData = 0; // Legacy typo maintained for compatibility
unsigned char Uart1_State = 0;	   // Legacy state variable
unsigned int ubrr;				   // Legacy baud rate variable
char Enter[] = {"\r\n"};		   // Legacy newline string
char Tap[] = {"\t"};			   // Legacy tab string

/*
 * Legacy Number Formatting Functions
 * These provide the exact same interface as the original Korean-commented versions
 */

// ASCII to HEX conversion table (legacy name)
unsigned char TABLE[16] = {'0', '1', '2', '3', '4', '5', '6', '7',
						   '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

/*
 * USART1_putchdecu() - Send unsigned integer as decimal
 * Legacy function maintaining exact original interface
 */
void USART1_putchdecu(unsigned int dt)
{
	unsigned int tmp = dt;

	putch_USART1(TABLE[tmp / 10000]);
	tmp %= 10000;
	putch_USART1(TABLE[tmp / 1000]);
	tmp %= 1000;
	putch_USART1(TABLE[tmp / 100]);
	tmp %= 100;
	putch_USART1(TABLE[tmp / 10]);
	putch_USART1(TABLE[tmp % 10]);
}

/*
 * USART1_putchuchar() - Send unsigned char as decimal
 * Legacy function maintaining exact original interface
 */
void USART1_putchuchar(unsigned char dt)
{
	unsigned char tmp = dt;

	putch_USART1(TABLE[tmp / 100]);
	tmp %= 100;
	putch_USART1(TABLE[tmp / 10]);
	putch_USART1(TABLE[tmp % 10]);
}

/*
 * USART1_putchdecs() - Send signed integer as decimal
 * Legacy function maintaining exact original interface
 */
void USART1_putchdecs(signed int dt)
{
	signed int tmp = dt;

	if (tmp >= 0)
	{
		putch_USART1('+');
		putch_USART1(TABLE[tmp / 10000]);
		tmp %= 10000;
		putch_USART1(TABLE[tmp / 1000]);
		tmp %= 1000;
		putch_USART1(TABLE[tmp / 100]);
		tmp %= 100;
		putch_USART1(TABLE[tmp / 10]);
		putch_USART1(TABLE[tmp % 10]);
	}
	else
	{
		tmp = -tmp;
		putch_USART1('-');
		putch_USART1(TABLE[tmp / 10000]);
		tmp %= 10000;
		putch_USART1(TABLE[tmp / 1000]);
		tmp %= 1000;
		putch_USART1(TABLE[tmp / 100]);
		tmp %= 100;
		putch_USART1(TABLE[tmp / 10]);
		putch_USART1(TABLE[tmp % 10]);
	}
}

/*
 * USART1_putchlongs() - Send long integer as decimal
 * Legacy function for 32-bit number support
 */
void USART1_putchlongs(long dt)
{
	long tmp = dt;
	long divisor = 1000000000; // Start with billions

	// Skip leading zeros
	while (divisor > 1 && tmp < divisor)
	{
		divisor /= 10;
	}

	// Output each digit
	while (divisor >= 1)
	{
		putch_USART1(TABLE[tmp / divisor]);
		tmp %= divisor;
		divisor /= 10;
	}
}

/*
 * USART1_puthex() - Send byte as hexadecimal
 * Legacy function maintaining exact original interface
 */
void USART1_puthex(unsigned char dt)
{
	unsigned char high_nibble = dt >> 4;
	unsigned char low_nibble = dt & 0x0F;

	putch_USART1(TABLE[high_nibble]);
	putch_USART1(TABLE[low_nibble]);
}

/*
 * H2C() - Character to hex conversion
 * Legacy function maintaining exact original interface
 */
unsigned int H2C(unsigned char ch)
{
	unsigned char high, low;
	unsigned int s;

	high = ch >> 4;
	low = ch & 0x0f;
	s = TABLE[high];
	s <<= 8;
	s |= TABLE[low];

	return s;
}

/*
 * Note: No ISR definitions are provided here. Applications should define
 * ISR(USART1_RX_vect) locally and call uart_rx_interrupt_handler() if desired.
 */

/*
 * Educational main function prototypes (implemented in separate main_*.c files)
 * These are declared here for header compatibility
 */
void main_serial_general_word(void) { /* Implemented in main_serial_general_word.c */ }

#endif // !ASSEMBLY_BLINK_BASIC
void main_serial_sentence(void) { /* Implemented in main_serial_sentence.c */ }
void main_serial_polling_single_char(void) { /* Implemented in main_serial_polling_single_char.c */ }
void Serial_Main(void) { /* Implemented in main_serial.c */ }