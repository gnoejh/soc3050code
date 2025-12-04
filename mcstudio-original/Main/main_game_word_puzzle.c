



#include "config.h"

#ifdef GAME_GUESS_SECRET_WORD

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>

#define MYUBRR F_CPU/16/BAUD-1 // Calculate UBRR value

// UART buffer sizes
#define TX_BUFFER_SIZE 128
#define RX_BUFFER_SIZE 32

// UART buffers
volatile char tx_buffer[TX_BUFFER_SIZE];
volatile uint8_t tx_write_pos = 0;
volatile uint8_t tx_read_pos = 0;

// Secret word list
const char* secret_words[] = {
	"banana", "apple", "grapes", "orange", "lemon",
	"mango", "peach", "pear", "kiwi", "plum",
	"cherry", "strawberry", "blueberry", "pineapple", "papaya"
};
#define NUM_WORDS (sizeof(secret_words) / sizeof(secret_words[0]))
#define MAX_WORD_LENGTH 20

// Function prototypes
void init_UART(uint16_t ubrr_value);
void send_string(const char* str);
void write_tx_buffer(char c);
void start_game();
void process_guess(const char* guess);
void provide_hint();
void reset_game();

// Game variables
char current_secret[MAX_WORD_LENGTH];
char revealed_word[MAX_WORD_LENGTH];
uint8_t attempts;

// UART initialization
void init_UART(uint16_t ubrr_value) {
	UBRR1H = (uint8_t)(ubrr_value >> 8);
	UBRR1L = (uint8_t)ubrr_value;
	UCSR1B = (1 << RXEN1) | (1 << TXEN1);  // Enable receiver and transmitter
	UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);  // Set frame format: 8 data, 1 stop bit
	UCSR1B |= (1 << RXCIE1);  // Enable RX interrupt
}

// Send a string via UART
void send_string(const char* str) {
	while (*str) {
		write_tx_buffer(*str++);
	}
}

// Write data to the transmission buffer
void write_tx_buffer(char c) {
	if ((tx_write_pos + 1) % TX_BUFFER_SIZE != tx_read_pos) {
		tx_buffer[tx_write_pos] = c;
		tx_write_pos = (tx_write_pos + 1) % TX_BUFFER_SIZE;
	}
	UCSR1B |= (1 << UDRIE1);  // Enable data register empty interrupt
}

// ISR for UART data register empty
ISR(USART1_UDRE_vect) {
	if (tx_read_pos != tx_write_pos) {
		UDR1 = tx_buffer[tx_read_pos];
		tx_read_pos = (tx_read_pos + 1) % TX_BUFFER_SIZE;
		} else {
		UCSR1B &= ~(1 << UDRIE1);  // Disable interrupt if nothing to send
	}
}

// ISR for UART receive complete
ISR(USART1_RX_vect) {
	static char buffer[RX_BUFFER_SIZE];
	static uint8_t index = 0;

	char data = UDR1;

	if (data == '\r' || data == '\n') {
		if (index > 0) {
			buffer[index] = '\0';  // Null-terminate the string
			process_guess(buffer);
			index = 0;  // Reset buffer index
		}
		} else if (index < RX_BUFFER_SIZE - 1) {
		buffer[index++] = data;  // Store received character
	}
}

// Start the game
void start_game() {
	// Choose a random secret word
	srand(TCNT1);  // Use timer as random seed
	int random_index = rand() % NUM_WORDS;
	strcpy(current_secret, secret_words[random_index]);

	// Initialize revealed word
	memset(revealed_word, '_', strlen(current_secret));
	revealed_word[strlen(current_secret)] = '\0';

	// Reset attempts
	attempts = 0;

	// Send game instructions
	send_string("Welcome to Guess the Secret Word!\r\n");
	send_string("I have chosen a word. Start guessing!\r\n");
	provide_hint();
}

// Process the user's guess
void process_guess(const char* guess) {
	attempts++;
	if (strcasecmp(guess, current_secret) == 0) {
		char success_msg[64];
		snprintf(success_msg, sizeof(success_msg), "Congratulations! You guessed it in %d attempts.\r\n", attempts);
		send_string(success_msg);
		send_string("Starting a new game...\r\n");
		reset_game();
		start_game();
		} else {
		send_string("Incorrect guess! Try again.\r\n");
		provide_hint();
	}
}

// Provide a hint to the user
void provide_hint() {
	// Reveal one more letter
	for (size_t i = 0; i < strlen(current_secret); i++) {
		if (revealed_word[i] == '_') {
			revealed_word[i] = current_secret[i];
			break;
		}
	}

	char hint_msg[64];
	snprintf(hint_msg, sizeof(hint_msg), "Hint: %s\r\n", revealed_word);
	send_string(hint_msg);
}

// Reset the game variables
void reset_game() {
	memset(current_secret, 0, MAX_WORD_LENGTH);
	memset(revealed_word, 0, MAX_WORD_LENGTH);
	attempts = 0;
}

// Main function for the UART-based game
void main_game_guess_secret_word(void) {
	uint16_t ubrr = MYUBRR;
	init_UART(ubrr);  // Initialize UART
	sei();  // Enable global interrupts

	start_game();  // Start the game

	while (1) {
		// Main loop (waiting for user input via UART)
	}
}

#endif
