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
 * =============================================================================
 * ENHANCED UART FUNCTIONS - PROFESSIONAL COMMUNICATION
 * =============================================================================
 */

/*
 * Formatted Output Functions - Printf-Style
 */
void USART1_printf(const char *format, ...);                  // Printf-style formatted output
void USART1_print_int(signed int value);                      // Print signed integer
void USART1_print_uint(unsigned int value);                   // Print unsigned integer
void USART1_print_long(signed long value);                    // Print signed long
void USART1_print_float(float value, unsigned char decimals); // Print floating point

/*
 * String Reception Functions - Buffered Input
 */
unsigned char USART1_read_line(char *buffer, unsigned char max_len);   // Read until newline
unsigned char USART1_read_string(char *buffer, unsigned char max_len); // Read with timeout
unsigned char USART1_available(void);                                  // Check data available
void USART1_flush_rx(void);                                            // Clear receive buffer

/*
 * Receive Buffer with Timeout
 */
typedef struct
{
    char buffer[128];        // Receive buffer
    unsigned char head;      // Buffer head index
    unsigned char tail;      // Buffer tail index
    unsigned char count;     // Number of bytes in buffer
    unsigned int timeout_ms; // Timeout in milliseconds
    unsigned char overflow;  // Overflow flag
} UART_RxBuffer;

void UART_Buffer_Init(UART_RxBuffer *buf, unsigned int timeout_ms);                        // Initialize buffer
unsigned char UART_Buffer_Get_Char(UART_RxBuffer *buf, char *ch);                          // Get character from buffer
unsigned char UART_Buffer_Get_Line(UART_RxBuffer *buf, char *line, unsigned char max_len); // Get line
void UART_Buffer_Clear(UART_RxBuffer *buf);                                                // Clear buffer

/*
 * Command Parsing Functions
 */
typedef struct
{
    char command[32];       // Command string
    char args[4][32];       // Up to 4 arguments
    unsigned char num_args; // Number of arguments parsed
    unsigned char valid;    // Command valid flag
} UART_Command;

unsigned char UART_Parse_Command(char *input, UART_Command *cmd);       // Parse command string
unsigned char UART_Match_Command(UART_Command *cmd, const char *match); // Match command
signed int UART_Get_Arg_Int(UART_Command *cmd, unsigned char arg_num);  // Get integer argument
float UART_Get_Arg_Float(UART_Command *cmd, unsigned char arg_num);     // Get float argument

/*
 * Binary Data Transfer Functions
 */
void USART1_write_bytes(const unsigned char *data, unsigned int length);                                 // Send binary data
unsigned int USART1_read_bytes(unsigned char *buffer, unsigned int max_length, unsigned int timeout_ms); // Read binary

/*
 * Packet Framing Functions
 */
typedef struct
{
    unsigned char start_byte; // Packet start delimiter
    unsigned char end_byte;   // Packet end delimiter
    unsigned char data[64];   // Packet data
    unsigned char length;     // Data length
    unsigned char checksum;   // Packet checksum
    unsigned char valid;      // Packet valid flag
} UART_Packet;

void UART_Packet_Init(UART_Packet *pkt, unsigned char start, unsigned char end); // Initialize packet
void UART_Packet_Add_Byte(UART_Packet *pkt, unsigned char byte);                 // Add data byte
unsigned char UART_Packet_Calculate_Checksum(UART_Packet *pkt);                  // Calculate checksum
void UART_Packet_Send(UART_Packet *pkt);                                         // Send packet
unsigned char UART_Packet_Receive(UART_Packet *pkt, unsigned int timeout_ms);    // Receive packet

/*
 * Checksum and CRC Functions
 */
unsigned char UART_Checksum_XOR(const unsigned char *data, unsigned char length); // XOR checksum
unsigned char UART_Checksum_Sum(const unsigned char *data, unsigned char length); // Sum checksum
unsigned int UART_CRC16(const unsigned char *data, unsigned char length);         // CRC-16 checksum

/*
 * Stream Processing Functions
 */
void USART1_print_buffer_hex(const unsigned char *buffer, unsigned char length);   // Print buffer as hex
void USART1_print_buffer_ascii(const unsigned char *buffer, unsigned char length); // Print buffer as ASCII

/*
 * Data Encoding Functions
 */
void USART1_encode_base64(const unsigned char *input, unsigned char in_len, char *output); // Base64 encode
unsigned char USART1_decode_base64(const char *input, unsigned char *output);              // Base64 decode

/*
 * Interactive Menu Functions
 */
typedef struct
{
    const char *title;         // Menu title
    const char *options[10];   // Menu options (up to 10)
    unsigned char num_options; // Number of options
} UART_Menu;

void UART_Menu_Init(UART_Menu *menu, const char *title);        // Initialize menu
void UART_Menu_Add_Option(UART_Menu *menu, const char *option); // Add menu option
void UART_Menu_Display(UART_Menu *menu);                        // Display menu
unsigned char UART_Menu_Get_Selection(UART_Menu *menu);         // Get user selection

/*
 * Status and Statistics
 */
typedef struct
{
    unsigned long tx_count;      // Transmitted bytes
    unsigned long rx_count;      // Received bytes
    unsigned int tx_errors;      // Transmission errors
    unsigned int rx_errors;      // Reception errors
    unsigned int overflow_count; // Buffer overflows
    unsigned int frame_errors;   // Framing errors
    unsigned int parity_errors;  // Parity errors
} UART_Statistics;

extern UART_Statistics uart_stats;
void UART_Reset_Statistics(void); // Reset counters
void UART_Print_Statistics(void); // Print statistics

/*
 * Advanced Configuration
 */
void UART_Set_Baud_Rate(unsigned long baud);        // Change baud rate
void UART_Enable_Parity(unsigned char parity_type); // Enable parity
void UART_Set_Stop_Bits(unsigned char stop_bits);   // Set stop bits

/*
 * Flow Control Functions
 */
void UART_Enable_RTS_CTS(void);         // Enable hardware flow control
unsigned char UART_Check_CTS(void);     // Check Clear To Send
void UART_Set_RTS(unsigned char state); // Set Request To Send

/*
 * Interrupt Service Routine Declarations
 */
void __vector_30(void) __attribute__((signal, __INTR_ATTRS)); // USART1_RX_vect
void __vector_18(void) __attribute__((signal, __INTR_ATTRS)); // USART0_RX_vect (compatibility)

#endif // _UART_H_