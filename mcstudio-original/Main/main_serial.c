/*
 * main_serial.c
 *
 * Created: 2024-10-21
 * Author: hjeong
 * 
 * This file demonstrates various UART programming techniques for the ATmega128,
 * including character-level, sentence-level, and word-level communication,
 * using both polling and interrupts.
 */

#include "config.h" // Include configuration file for example selection

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

// Define CPU frequency and baud rate for UART communication
#define F_CPU 16000000UL
#define BAUD 9600

/*
 * UART1 Control Register Summary:
 * ----------------------------------------------------------------------------
 * Register     | Bit(s)          | Description
 * ----------------------------------------------------------------------------
 * UBRR1H/L     | High/Low Byte   | Baud rate setting (F_CPU / (16 * BAUD) - 1)
 * ----------------------------------------------------------------------------
 * UCSR1A       | UDRE1           | Data Register Empty Flag (ready to send)
 *              | RXC1            | Receive Complete Flag (data received)
 *              | TXC1            | Transmit Complete Flag
 * ----------------------------------------------------------------------------
 * UCSR1B       | RXEN1           | Enable Receiver
 *              | TXEN1           | Enable Transmitter
 *              | RXCIE1          | Enable Receive Complete Interrupt
 *              | TXCIE1          | Enable Transmit Complete Interrupt
 *              | UDRIE1          | Enable Data Register Empty Interrupt
 * ----------------------------------------------------------------------------
 * UCSR1C       | UCSZ10, UCSZ11  | Character Size (8-bit when both are set)
 *              | USBS1           | Stop Bit Selection (1 bit = 0, 2 bits = 1)
 *              | UPM10, UPM11    | Parity Mode (None = 00, Even = 10, Odd = 11)
 * ----------------------------------------------------------------------------
 * UDR1         | Data Register   | Holds data to be sent/received
 * ----------------------------------------------------------------------------
 */

// ---------------------------------------------------------------
// UART Initialization
// ---------------------------------------------------------------
void UART1_init(void) {
    UBRR1H = (F_CPU / 16 / BAUD - 1) >> 8;  // Set the high byte of the UBRR register
    UBRR1L = (F_CPU / 16 / BAUD - 1);       // Set the low byte of the UBRR register
    UCSR1B = (1 << RXEN1) | (1 << TXEN1);   // Enable receiver and transmitter
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); // 8-bit data, 1 stop bit, no parity
}

// ---------------------------------------------------------------
// Polling: Single Character Transmission and Reception
// ---------------------------------------------------------------
#ifdef SERIAL_POLLING_SINGLE_CHAR
void UART1_transmit(unsigned char data) {
    while (!(UCSR1A & (1 << UDRE1)));  // Wait until buffer is empty
    UDR1 = data;                       // Transmit data
}

unsigned char UART1_receive(void) {
    while (!(UCSR1A & (1 << RXC1)));  // Wait until data is received
    return UDR1;                      // Return received data
}

void main_serial_polling_single_char(void) {
    UART1_init();                     // Initialize UART
    UART1_transmit('A');              // Send character 'A'
    while (1) {
        unsigned char data = UART1_receive();  // Receive a character
        UART1_transmit(data);                  // Echo it back
    }
}
#endif

// ---------------------------------------------------------------
// Polling: String Transmission and Reception
// ---------------------------------------------------------------
#ifdef SERIAL_POLLING_STRING
void UART1_send_string(const char *str) {
    while (*str) {
        while (!(UCSR1A & (1 << UDRE1)));  // Wait until buffer is empty
        UDR1 = *str++;                     // Transmit each character
    }
}

void main_serial_polling_string(void) {
    UART1_init();                                // Initialize UART
    UART1_send_string("Hello, UART with Polling!\n");  // Send a string
    while (1) {
        // Additional tasks can go here
    }
}
#endif

// ---------------------------------------------------------------
// Polling: Echo Functionality
// ---------------------------------------------------------------
#ifdef SERIAL_POLLING_ECHO
void UART1_transmit(unsigned char data) {
	while (!(UCSR1A & (1 << UDRE1)));  // Wait until buffer is empty
	UDR1 = data;                       // Transmit data
}

unsigned char UART1_receive(void) {
	while (!(UCSR1A & (1 << RXC1)));  // Wait until data is received
	return UDR1;                      // Return received data
}

void main_serial_polling_echo(void) {
    UART1_init();  // Initialize UART
    while (1) {
        unsigned char data = UART1_receive();  // Receive a character
        UART1_transmit(data);                  // Echo it back
    }
}
#endif

// ---------------------------------------------------------------
// Polling: Sentence-Level Communication
// ---------------------------------------------------------------
#ifdef SERIAL_POLLING_SENTENCE
#define MAX_LINE_SIZE 128
void UART1_transmit(unsigned char data) {
	while (!(UCSR1A & (1 << UDRE1)));  // Wait until buffer is empty
	UDR1 = data;                       // Transmit data
}

unsigned char UART1_receive(void) {
	while (!(UCSR1A & (1 << RXC1)));  // Wait until data is received
	return UDR1;                      // Return received data
}

void UART1_send_string(const char *str) {
	while (*str) {
		while (!(UCSR1A & (1 << UDRE1)));  // Wait until buffer is empty
		UDR1 = *str++;                     // Transmit each character
	}
}

void process_line(const char *line) {
    UART1_send_string("Processed: ");
    UART1_send_string(line);
    UART1_send_string("\n");
}

void main_serial_polling_sentence(void) {
    UART1_init();
    char buffer[MAX_LINE_SIZE];
    unsigned char index = 0;

    UART1_send_string("Type a sentence (end with \\n or \\r):\n");
    while (1) {
        char received = UART1_receive();
        if (received == '\n' || received == '\r') {
            buffer[index] = '\0';
            process_line(buffer);
            index = 0;
        } else if (index < MAX_LINE_SIZE - 1) {
            buffer[index++] = received;
        }
    }
}
#endif

// ---------------------------------------------------------------
// Interrupt: Receive Complete (RX)
// ---------------------------------------------------------------
#ifdef SERIAL_INTERRUPT_RX
volatile unsigned char received_data = 0;

void UART1_init_interrupt_rx(void) {
    UART1_init();
    UCSR1B |= (1 << RXCIE1);
}

ISR(USART1_RX_vect) {
    received_data = UDR1;
}

void main_serial_interrupt_rx(void) {
    UART1_init_interrupt_rx();
    sei();
    while (1) {
        // Process received_data in the main loop if needed
    }
}
#endif

// ---------------------------------------------------------------
// Interrupt: Transmit Complete (TX)
// ---------------------------------------------------------------
#ifdef SERIAL_INTERRUPT_TX
volatile unsigned char data_ready = 0;

void UART1_init_interrupt_tx(void) {
    UART1_init();
    UCSR1B |= (1 << TXCIE1);
}

ISR(USART1_TX_vect) {
    data_ready = 0;  // Transmission complete
}

void UART1_transmit_interrupt(unsigned char data) {
    while (data_ready);  // Wait if previous transmission is ongoing
    data_ready = 1;
    UDR1 = data;  // Start transmission
}

void main_serial_interrupt_tx(void) {
    UART1_init_interrupt_tx();
    sei();
    UART1_transmit_interrupt('A');
    while (1) {
        // Main loop is free for other tasks
		UART1_transmit_interrupt('Z');
    }
}
#endif

// ---------------------------------------------------------------
// Interrupt: Echo (RX + TX)
// ---------------------------------------------------------------
#ifdef SERIAL_INTERRUPT_ECHO
volatile unsigned char rx_data = 0;

void UART1_init_interrupt_echo(void) {
    UART1_init();
    UCSR1B |= (1 << RXCIE1);
}

ISR(USART1_RX_vect) {
    rx_data = UDR1;  // Receive data
    while (!(UCSR1A & (1 << UDRE1)));  // Wait until the buffer is empty
    UDR1 = rx_data;  // Echo back
}

void main_serial_interrupt_echo(void) {
    UART1_init_interrupt_echo();
    sei();
    while (1) {
        // Main loop is free for other tasks
    }
}
#endif

// ---------------------------------------------------------------
// Circular Buffer Example
// ---------------------------------------------------------------
#ifdef SERIAL_INTERRUPT_CIRCULAR_BUFFER
#define BUFFER_SIZE 64
volatile char tx_buffer[BUFFER_SIZE];
volatile char rx_buffer[BUFFER_SIZE];
volatile unsigned char tx_write_index = 0, tx_read_index = 0;
volatile unsigned char rx_write_index = 0, rx_read_index = 0;

void UART1_init_interrupt_buffer(void) {
    UART1_init();
    UCSR1B |= (1 << RXCIE1) | (1 << UDRIE1);  // Enable RX and TX interrupts
}

ISR(USART1_RX_vect) {
    char received_data = UDR1;
    unsigned char next_index = (rx_write_index + 1) % BUFFER_SIZE;
    if (next_index != rx_read_index) {
        rx_buffer[rx_write_index] = received_data;
        rx_write_index = next_index;
    }
}

ISR(USART1_UDRE_vect) {
    if (tx_read_index != tx_write_index) {
        UDR1 = tx_buffer[tx_read_index];
        tx_read_index = (tx_read_index + 1) % BUFFER_SIZE;
    } else {
        UCSR1B &= ~(1 << UDRIE1);  // Disable TX interrupt
    }
}

void UART1_transmit_buffer(unsigned char data) {
    unsigned char next_index = (tx_write_index + 1) % BUFFER_SIZE;
    while (next_index == tx_read_index);  // Wait if buffer is full
    tx_buffer[tx_write_index] = data;
    tx_write_index = next_index;
    UCSR1B |= (1 << UDRIE1);  // Enable TX interrupt
}

unsigned char UART1_receive_buffer(void) {
    while (rx_read_index == rx_write_index);  // Wait if buffer is empty
    char data = rx_buffer[rx_read_index];
    rx_read_index = (rx_read_index + 1) % BUFFER_SIZE;
    return data;
}

void main_serial_interrupt_circular_buffer(void) {
    UART1_init_interrupt_buffer();
    sei();
    while (1) {
        char data = UART1_receive_buffer();
        UART1_transmit_buffer(data);
    }
}
#endif

// ---------------------------------------------------------------
// Word-Level Communication (Polling)
// ---------------------------------------------------------------
#ifdef SERIAL_POLLING_WORD
#define MAX_WORD_SIZE 64
char word_buffer[MAX_WORD_SIZE];
unsigned char word_index = 0;

void UART1_transmit(unsigned char data) {
	while (!(UCSR1A & (1 << UDRE1)));  // Wait until buffer is empty
	UDR1 = data;                       // Transmit data
}

unsigned char UART1_receive(void) {
	while (!(UCSR1A & (1 << RXC1)));  // Wait until data is received
	return UDR1;                      // Return received data
}

void UART1_send_string(const char *str) {
	while (*str) {
		while (!(UCSR1A & (1 << UDRE1)));  // Wait until buffer is empty
		UDR1 = *str++;                     // Transmit each character
	}
}


void process_word(const char *word) {
    UART1_send_string("Word received: ");
    UART1_send_string(word);
    UART1_send_string("\n");
}

void main_serial_polling_word(void) {
    UART1_init();
    UART1_send_string("Enter words separated by spaces:\n");
    while (1) {
        char c = UART1_receive();
        if (c == ' ' || c == '\n' || c == '\r') {
            word_buffer[word_index] = '\0';
            process_word(word_buffer);
            word_index = 0;
        } else if (word_index < MAX_WORD_SIZE - 1) {
            word_buffer[word_index++] = c;
        }
    }
}
#endif

// ---------------------------------------------------------------
// Word-Level Communication (Interrupt)
// ---------------------------------------------------------------
#ifdef SERIAL_INTERRUPT_WORD
#define MAX_WORD_SIZE 64
volatile char word_buffer_interrupt[MAX_WORD_SIZE];
volatile unsigned char word_index_interrupt = 0;
volatile unsigned char word_ready = 0;

// Send a string via UART
void UART1_send_string(const char *str) {
	while (*str) {
		while (!(UCSR1A & (1 << UDRE1)));  // Wait until buffer is empty
		UDR1 = *str++;                     // Transmit each character
	}
}

// Process a received word
void process_word(const char *word) {
	UART1_send_string("Word received: ");
	UART1_send_string(word);
	UART1_send_string("\n");
}

// Initialize UART for word-level interrupt
void UART1_init_word_interrupt(void) {
	UART1_init();                // Initialize UART baud rate and frame format
	UCSR1B |= (1 << RXCIE1);     // Enable RX interrupt
}

// ISR for receiving characters
ISR(USART1_RX_vect) {
	char c = UDR1;  // Read received character
	if (c == ' ' || c == '\n' || c == '\r') {
		word_buffer_interrupt[word_index_interrupt] = '\0';  // Null-terminate the word
		word_ready = 1;  // Set word ready flag
		word_index_interrupt = 0;  // Reset word index
		} else if (word_index_interrupt < MAX_WORD_SIZE - 1) {
		word_buffer_interrupt[word_index_interrupt++] = c;  // Store character
	}
}

// Main function for interrupt-based word processing
void main_serial_interrupt_word(void) {
	UART1_init_word_interrupt();  // Initialize UART with interrupt
	sei();  // Enable global interrupts
	UART1_send_string("Word-based interrupt started. Enter words:\n");
	while (1) {
		if (word_ready) {
			word_ready = 0;  // Clear word ready flag
			process_word((const char *)word_buffer_interrupt);  // Process the word
		}
	}
}
#endif
