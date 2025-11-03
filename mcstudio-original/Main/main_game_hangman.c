#include "config.h"

#ifdef GAME_HANGMAN
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>

#define F_CPU 16000000UL  // Assuming a clock speed of 16 MHz
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

// UART function prototypes
void uart_init(unsigned int ubrr);
void uart_transmit(char data);
char uart_receive(void);
void uart_send_string(const char* str);

// Hangman game function prototypes
void init_game();
void play_game();
void draw_hangman(int attempts);
void reveal_letter(char guess);
int is_word_guessed();
int is_game_over();

// Hangman game parameters
#define MAX_ATTEMPTS 6
#define NUM_WORDS 10
#define MAX_WORD_LENGTH 10

// Hangman game variables
char secret_word[MAX_WORD_LENGTH] = {0};
char guessed_word[MAX_WORD_LENGTH] = {0};
int attempts_remaining;

// Word list
const char* word_list[NUM_WORDS] = {
	"banana", "apple", "grapes", "orange", "lemon",
	"mango", "papaya", "coconut", "melon", "peach"
};

// Initialize UART
void uart_init(unsigned int ubrr) {
	// Set baud rate
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	// Enable receiver and transmitter
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	// Set frame format: 8data, 1stop bit
	UCSR0C = (3<<UCSZ00);
}

// Transmit a character over UART
void uart_transmit(char data) {
	// Wait for empty transmit buffer
	while (!(UCSR0A & (1<<UDRE0)));
	// Put data into buffer, sends the data
	UDR0 = data;
}

// Receive a character over UART (blocking)
char uart_receive(void) {
	// Wait for data to be received
	while (!(UCSR0A & (1<<RXC0)));
	// Get and return received data from buffer
	return UDR0;
}

// Send a string over UART
void uart_send_string(const char* str) {
	while (*str) {
		uart_transmit(*str++);
	}
}

void init_game() {
	// Pick a random word from the list
	srand(0); // TODO: Replace 0 with a real seed value, like from a timer or external input
	int rand_index = rand() % NUM_WORDS;
	strncpy(secret_word, word_list[rand_index], MAX_WORD_LENGTH);

	// Initialize guessed_word with underscores
	for (int i = 0; i < strlen(secret_word); i++) {
		guessed_word[i] = '_';
	}
	guessed_word[strlen(secret_word)] = '\0';

	attempts_remaining = MAX_ATTEMPTS;

	uart_send_string("Welcome to Hangman! Guess the word:\r\n");
	uart_send_string(guessed_word);
	uart_send_string("\r\n");
}

void play_game() {
	while (!is_game_over()) {
		uart_send_string("Enter a letter: ");
		char guess = uart_receive();
		uart_transmit(guess); // Echo the input back to the user
		uart_send_string("\r\n");

		reveal_letter(guess);
		uart_send_string(guessed_word);
		uart_send_string("\r\n");

		draw_hangman(attempts_remaining);

		if (is_word_guessed()) {
			uart_send_string("Congratulations! You've won!\r\n");
			break;
		}
	}

	if (attempts_remaining == 0) {
		uart_send_string("Game over! The word was: ");
		uart_send_string(secret_word);
		uart_send_string("\r\n");
	}
}

void draw_hangman(int attempts) {
	// Placeholder for drawing the hangman
	char buffer[30];
	snprintf(buffer, 30, "Attempts remaining: %d\r\n", attempts);
	uart_send_string(buffer);
}

void reveal_letter(char guess) {
	int found = 0;
	for (int i = 0; i < strlen(secret_word); i++) {
		if (tolower(secret_word[i]) == tolower(guess)) {
			guessed_word[i] = secret_word[i];
			found = 1;
		}
	}
	if (!found) {
		attempts_remaining--;
	}
}

int is_word_guessed() {
	return strcmp(secret_word, guessed_word) == 0;
}

int is_game_over() {
	return attempts_remaining == 0 || is_word_guessed();
}

void main_game_hangman(void) {
	uart_init(MYUBRR);

	init_game();
	play_game();

	while (1) {
		// Optionally put the MCU to sleep or run other tasks
	}

}
#endif