#include "config.h"

#ifdef GAME_PUZZLE

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define MYUBRR F_CPU/16/BAUD-1 // Calculate UBRR value

// UART buffer sizes
#define TX_BUFFER_SIZE 128
#define RX_BUFFER_SIZE 10

// UART buffers
volatile char tx_buffer[TX_BUFFER_SIZE];
volatile uint8_t tx_write_pos = 0;
volatile uint8_t tx_read_pos = 0;

// Game parameters
#define LOWER_BOUND 1
#define UPPER_BOUND 100

// Game variables
int secret_number;
int attempts;

// Function prototypes
void init_UART(uint16_t ubrr_value);
void send_string(const char* str);
void write_tx_buffer(char c);
void start_game();
void process_guess(const char* guess);
void reset_game();

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
			process_guess(buffer);  // Process the guess
			index = 0;  // Reset buffer index
		}
		} else if (index < RX_BUFFER_SIZE - 1) {
		buffer[index++] = data;  // Store received character
	}
}

// Start the game
void start_game() {
	// Generate a random number
	srand(TCNT1);  // Use timer as a random seed
	secret_number = (rand() % (UPPER_BOUND - LOWER_BOUND + 1)) + LOWER_BOUND;

	// Reset attempts
	attempts = 0;

	// Send game instructions
	send_string("Welcome to the Number Guessing Game!\r\n");
	send_string("I have chosen a number between 1 and 100.\r\n");
	send_string("Try to guess it!\r\n");
}

// Process the user's guess
void process_guess(const char* guess) {
	int guessed_number = atoi(guess);  // Convert guess to integer
	attempts++;

	if (guessed_number < secret_number) {
		send_string("Too low! Try again.\r\n");
		} else if (guessed_number > secret_number) {
		send_string("Too high! Try again.\r\n");
		} else {
		char success_msg[64];
		snprintf(success_msg, sizeof(success_msg), "Congratulations! You guessed it in %d attempts.\r\n", attempts);
		send_string(success_msg);
		send_string("Starting a new game...\r\n");
		reset_game();
		start_game();
	}
}

// Reset the game variables
void reset_game() {
	secret_number = 0;
	attempts = 0;
}

// Main function for the UART-based number guessing game
void main_game_puzzle(void) {
	uint16_t ubrr = MYUBRR;
	init_UART(ubrr);  // Initialize UART
	sei();  // Enable global interrupts

	start_game();  // Start the game

	while (1) {
		// Main loop (waiting for user input via UART)
	}
}

#endif


////////////////////////
#ifdef GAME_SCRAMBLE

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define MYUBRR F_CPU/16/BAUD-1 // Calculate UBRR value

// UART buffer sizes
#define TX_BUFFER_SIZE 128
#define RX_BUFFER_SIZE 32

// UART buffers
volatile char tx_buffer[TX_BUFFER_SIZE];
volatile uint8_t tx_write_pos = 0;
volatile uint8_t tx_read_pos = 0;

// Word list for the game
const char* word_list[] = {
	"banana", "apple", "grapes", "orange", "lemon",
	"mango", "peach", "pear", "kiwi", "plum",
	"cherry", "strawberry", "blueberry", "pineapple", "papaya"
};
#define NUM_WORDS (sizeof(word_list) / sizeof(word_list[0]))
#define MAX_WORD_LENGTH 20

// Game variables
char secret_word[MAX_WORD_LENGTH];
char scrambled_word[MAX_WORD_LENGTH];
uint8_t attempts;

// Function prototypes
void init_UART(uint16_t ubrr_value);
void send_string(const char* str);
void write_tx_buffer(char c);
void start_game();
void scramble_word(const char* word, char* scrambled);
void process_guess(const char* guess);
void reset_game();

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
			process_guess(buffer);  // Process the guess
			index = 0;  // Reset buffer index
		}
		} else if (index < RX_BUFFER_SIZE - 1) {
		buffer[index++] = data;  // Store received character
	}
}

// Start the Word Scramble game
void start_game() {
	// Choose a random word
	srand(TCNT1);  // Use timer as a random seed
	int random_index = rand() % NUM_WORDS;
	strcpy(secret_word, word_list[random_index]);

	// Scramble the word
	scramble_word(secret_word, scrambled_word);

	// Reset attempts
	attempts = 0;

	// Send game instructions
	send_string("Welcome to the Word Scramble Game!\r\n");
	char scramble_msg[64];
	snprintf(scramble_msg, sizeof(scramble_msg), "Unscramble this word: %s\r\n", scrambled_word);
	send_string(scramble_msg);
}

// Scramble a word randomly
void scramble_word(const char* word, char* scrambled) {
	size_t len = strlen(word);
	char temp[len + 1];
	strcpy(temp, word);

	for (size_t i = 0; i < len; i++) {
		size_t rand_index = rand() % len;
		scrambled[i] = temp[rand_index];
		temp[rand_index] = temp[--len];  // Remove the used character
	}
	scrambled[strlen(word)] = '\0';
}

// Process the user's guess
void process_guess(const char* guess) {
	attempts++;
	if (strcasecmp(guess, secret_word) == 0) {
		char success_msg[64];
		snprintf(success_msg, sizeof(success_msg), "Congratulations! You solved it in %d attempts.\r\n", attempts);
		send_string(success_msg);
		send_string("Starting a new game...\r\n");
		reset_game();
		start_game();
		} else {
		send_string("Incorrect! Try again.\r\n");
		char scramble_msg[64];
		snprintf(scramble_msg, sizeof(scramble_msg), "Hint: %s\r\n", scrambled_word);
		send_string(scramble_msg);
	}
}

// Reset the game variables
void reset_game() {
	memset(secret_word, 0, MAX_WORD_LENGTH);
	memset(scrambled_word, 0, MAX_WORD_LENGTH);
	attempts = 0;
}

// Main function for the Word Scramble game
void main_game_scramble(void) {
	uint16_t ubrr = MYUBRR;
	init_UART(ubrr);  // Initialize UART
	sei();  // Enable global interrupts

	start_game();  // Start the game

	while (1) {
		// Main loop (waiting for user input via UART)
	}
}

#endif

