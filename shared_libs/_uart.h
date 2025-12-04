/*
 * _uart.h - ATmega128 UART Communication Library Header
 * Educational Version for Assembly→C→Python Learning Progression
 */

#ifndef _UART_H_
#define _UART_H_

/*
 * Core UART Functions - Basic Communication
 */
void Uart1_init(void); // Initialize UART1 for 8N1 at configured baud rate

// Single character communication (polling method)
void putch_USART1(char data);     // Send single character
unsigned char getch_USART1(void); // Receive single character (blocking)

// String communication (polling method)
void puts_USART1(char *str); // Send null-terminated string

/*
 * Legacy Number Formatting Functions - Exact Original Interface
 * These maintain compatibility with existing educational examples
 */
void USART1_putchdecu(unsigned int dt);   // Send unsigned int as decimal
void USART1_putchuchar(unsigned char dt); // Send unsigned char as decimal
void USART1_putchdecs(signed int dt);     // Send signed int as decimal
void USART1_putchlongs(long dt);          // Send long as decimal
void USART1_puthex(unsigned char dt);     // Send byte as hexadecimal
unsigned int H2C(unsigned char ch);       // Character to hex conversion

/*
 * Educational Helper Functions - Modern Interface
 */
void USART1_print_decimal(unsigned int number); // Print number as decimal
void USART1_print_hex(unsigned char number);    // Print number as hexadecimal
void USART1_print_newline(void);                // Send carriage return + line feed

/*
 * Interactive Functions - For Learning Exercises
 */
void USART1_echo_char(void); // Echo received character back

/*
 * Interrupt-Based Communication (Advanced Topic)
 */
unsigned char USART1_data_available(void); // Check if new data received
unsigned char USART1_get_data(void);       // Get data from interrupt buffer

/*
 * Global Variables for Educational Use and Legacy Compatibility
 */
extern unsigned char command;         // Last command received (legacy)
extern unsigned char InputSerialData; // Last data byte received (legacy)
extern unsigned char InputSirialData; // Alternative spelling (legacy)
extern unsigned char Uart1_State;     // Communication state (legacy)
extern unsigned int ubrr;             // Baud rate register value (legacy)

extern unsigned char uart_command;      // Modern command variable
extern unsigned char uart_input_data;   // Modern input buffer
extern unsigned char uart_state;        // Modern state tracker
extern unsigned int uart_baud_register; // Modern baud rate value

extern char Enter[];        // "\r\n" string (legacy)
extern char Tap[];          // "\t" string (legacy)
extern char uart_newline[]; // "\r\n" string (modern)
extern char uart_tab[];     // "\t" string (modern)

/*
 * UART Register Educational Constants
 * These help students understand ATmega128 UART registers
 */
#define UART_8BIT_CHAR ((1 << UCSZ11) | (1 << UCSZ10))                // 8-bit character size
#define UART_ENABLE_ALL ((1 << RXCIE1) | (1 << RXEN1) | (1 << TXEN1)) // RX+TX+Interrupt
#define UART_ENABLE_POLL ((1 << RXEN1) | (1 << TXEN1))                // RX+TX without interrupt

/*
 * Common Baud Rates for Educational Reference
 * Formula: UBRR = (F_CPU / (16 * BAUD)) - 1
 */
#define BAUD 9600     // Default baud rate for educational projects
#define BAUD_2400 416 // For 16MHz: (16000000/(16*2400))-1
#define BAUD_4800 207 // For 16MHz: (16000000/(16*4800))-1
#define BAUD_9600 103 // For 16MHz: (16000000/(16*9600))-1
#define BAUD_19200 51 // For 16MHz: (16000000/(16*19200))-1
#define BAUD_38400 25 // For 16MHz: (16000000/(16*38400))-1

/*
 * ASCII Character Constants for Educational Use
 */
#define ASCII_CR 0x0D        // Carriage Return '\r'
#define ASCII_LF 0x0A        // Line Feed '\n'
#define ASCII_TAB 0x09       // Tab '\t'
#define ASCII_SPACE 0x20     // Space ' '
#define ASCII_BACKSPACE 0x08 // Backspace
#define ASCII_ESC 0x1B       // Escape

/*
 * Function Prototypes for Educational Examples
 * These are implemented in separate main_*.c files or provided as stubs
 */
void main_serial_polling_single_char(void); // Basic character I/O
void main_serial_polling_echo(void);        // Echo received characters
void main_serial_polling_string(void);      // String communication
void main_serial_interrupt_rx(void);        // Interrupt-based receive
void main_serial_interrupt_tx(void);        // Interrupt-based transmit
void main_serial_general_word(void);        // General word processing
void main_serial_sentence(void);            // Sentence processing
void Serial_Main(void);                     // Main serial demonstration

/*
 * Note: ISRs are not declared or defined in this header or library.
 * Applications should define ISR(USART1_RX_vect) locally if using RX interrupts.
 */

#endif // _UART_H_