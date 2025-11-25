#include "config.h"

#ifdef GAME_WORD_PUZZLE

#define MYUBRR F_CPU/16/BAUD-1 // Calculate UBRR value

// UART buffer sizes
#define TX_BUFFER_SIZE 128  // Increase this to a higher value to accommodate your message

// UART buffers
volatile char tx_buffer[TX_BUFFER_SIZE];
volatile uint8_t tx_write_pos = 0;
volatile uint8_t tx_read_pos = 0;

// Function prototypes
void init_UART(uint16_t ubrr_value);
void uart_transmit(char data);
void send_string(const char* str);
uint8_t is_tx_buffer_empty();
void write_tx_buffer(char c);

// Word list
// Word list expanded to 20 fruits
const char* word_list[] = {
	"banana", "apple", "grapes", "orange", "lemon",
	"mango", "peach", "pear", "kiwi", "plum",
	"pomegranate", "cherry", "strawberry", "blackberry", "blueberry",
	"pineapple", "papaya", "melon", "coconut", "lime"
};
#define NUM_WORDS (sizeof(word_list) / sizeof(word_list[0]))
#define MAX_WORD_LENGTH 15  // Updated to accommodate longer fruit names like "pomegranate"

// Current guessed word
char guessed_word[MAX_WORD_LENGTH] = {0};

// UART and other utility functions would remain unchanged

// Function to ask the user to think of a word
void ask_for_word() {
	send_string("Think of a fruit: banana, apple, grapes, orange, lemon, "
	"mango, peach, pear, kiwi, plum, pomegranate, cherry, "
	"strawberry, blackberry, blueberry, pineapple, papaya, melon, "
	"coconut, or lime.\r\n");
}


// Function to reset the guessed word
void reset_guessed_word() {
	for (int i = 0; i < MAX_WORD_LENGTH; i++) {
		guessed_word[i] = '\0';
	}
}

// Function to send a character to the UART
void uart_transmit(char data) {
	// Wait for empty transmit buffer
	while (!(UCSR1A & (1<<UDRE1)));
	// Put data into buffer, sends the data
	UDR1 = data;
}

// Function to check if the transmission buffer is empty
uint8_t is_tx_buffer_empty() {
	return tx_read_pos == tx_write_pos;
}

// Function to write data to the transmission buffer
void write_tx_buffer(char c) {
	if ((tx_write_pos + 1) % TX_BUFFER_SIZE != tx_read_pos) {
		tx_buffer[tx_write_pos] = c;
		tx_write_pos = (tx_write_pos + 1) % TX_BUFFER_SIZE;
	}
	// Enable data register empty interrupt
	UCSR1B |= (1<<UDRIE1);
}

// ISR for UART data register empty
ISR(USART1_UDRE_vect) {
	if (!is_tx_buffer_empty()) {
		UDR1 = tx_buffer[tx_read_pos];
		tx_read_pos = (tx_read_pos + 1) % TX_BUFFER_SIZE;
		} else {
		// Disable the interrupt if nothing to send
		UCSR1B &= ~(1<<UDRIE1);
	}
}

// Function to send a string through UART
void send_string(const char* str) {
	while (*str) {
		write_tx_buffer(*str++);
	}
}

// Function to process the user's answer
void process_answer(char* answer) {
	static int current_word = 0;  // Index of the word being guessed
	static int letter_index = 0;  // Index of the letter in the word being guessed

	if (answer[0] == 'y') {
		guessed_word[letter_index] = word_list[current_word][letter_index];
		letter_index++;
	}

	// If the current word is completely guessed or the answer was 'n'
	if (guessed_word[letter_index] == '\0' || answer[0] == 'n') {
		if (current_word < NUM_WORDS - 1) {
			// Move to the next word in the list
			current_word++;
			} else {
			// Start again from the first word
			current_word = 0;
		}
		// Reset the guessed word
		reset_guessed_word();
		letter_index = 0;
	}

	// Send next guess or success message
	if (guessed_word[letter_index] != '\0') {
		char success_msg[64];
		snprintf(success_msg, sizeof(success_msg), "The word is %s!\r\n", guessed_word);
		send_string(success_msg);
		// Start a new round
		ask_for_word();
		} else {
		char guess_msg[64];
		snprintf(guess_msg, sizeof(guess_msg), "Does your word have '%c' at position %d? (y/n)\r\n", word_list[current_word][letter_index], letter_index + 1);
		send_string(guess_msg);
	}
}

// ISR for UART receive complete
ISR(USART1_RX_vect) {
	static char buffer[10];
	static uint8_t i = 0;

	char data = UDR1;

	// Check for end of line (\r or \n)
	if (data == '\r' || data == '\n') {
		if (i > 0) { // Only if buffer has data
			buffer[i] = '\0'; // Null-terminate the string
			process_answer(buffer);
			i = 0; // Reset index
		}
		} else if (i < sizeof(buffer) - 1) { // Save data to buffer
		buffer[i++] = data;
	}
}

void init_UART(uint16_t ubrr_value) {
	// Set baud rate
	UBRR1H = (uint8_t)(ubrr_value>>8);
	UBRR1L = (uint8_t)ubrr_value;
	// Enable receiver and transmitter
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);
	// Set frame format: 8data, 1stop bit
	UCSR1C = (3<<UCSZ10);
	// Enable RX interrupt
	UCSR1B |= (1<<RXCIE1);
}


void main_game_word_puzzle(void) {
	// Set baud rate
	uint16_t ubrr = MYUBRR;
	init_UART(ubrr); // Initialize UART with the baud rate

	// Enable global interrupts
	sei();

	// Start the game
	ask_for_word();

	// Main loop
	while (1) {
		// Main loop can be used to implement additional tasks
	}

}

#endif
