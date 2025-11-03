/*
 * =============================================================================
 * SERIAL COMMUNICATION - COMPREHENSIVE EDUCATIONAL MATRIX
 * =============================================================================
 *
 * PROJECT: Serial_Communications
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Systematic comparison of POLLING vs INTERRUPT methods across three data
 * granularities: CHARACTER, WORD, and SENTENCE. All demos use ECHO pattern
 * (RX + TX) for consistent learning and direct method comparison.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master both polling and interrupt-driven UART communication
 * 2. Understand data granularity: character → word → sentence progression
 * 3. Learn real ISR programming (no wrappers, direct register access)
 * 4. Compare CPU efficiency: polling blocks vs interrupts free CPU
 * 5. Implement circular buffers and protocol parsing
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - UART1 connection for serial communication
 * - Serial terminal (9600 baud, 8N1)
 *
 * DOCUMENTATION REFERENCE:
 * ATmega128 Datasheet: https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/2467S.pdf
 * - USART section (pages 169-194)
 * - USART registers (pages 186-194)
 * - Interrupt vectors (pages 44-46)
 *
 * =============================================================================
 * PEDAGOGICAL STRUCTURE: 2×3 MATRIX (6 DEMOS)
 * =============================================================================
 *
 * ┌─────────────────┬──────────────────────┬──────────────────────┐
 * │   Data Type     │   POLLING METHOD     │  INTERRUPT METHOD    │
 * │                 │   (Simple/Blocking)  │  (Efficient/Complex) │
 * ├─────────────────┼──────────────────────┼──────────────────────┤
 * │ 1. CHARACTER    │ Demo 1: Polling      │ Demo 4: Interrupt    │
 * │    (single)     │ Character Echo       │ Character Echo       │
 * │    Basic I/O    │ • getch/putch        │ • ISR RX/TX          │
 * │                 │ • CPU blocks         │ • CPU continues      │
 * ├─────────────────┼──────────────────────┼──────────────────────┤
 * │ 2. WORD         │ Demo 2: Polling      │ Demo 5: Interrupt    │
 * │    (space delim)│ Word Echo            │ Word Echo            │
 * │    Buffering    │ • Manual buffer      │ • Circular buffer    │
 * │                 │ • Simple parsing     │ • ISR buffering      │
 * ├─────────────────┼──────────────────────┼──────────────────────┤
 * │ 3. SENTENCE     │ Demo 3: Polling      │ Demo 6: Interrupt    │
 * │    (line delim) │ Sentence Echo        │ Sentence Echo        │
 * │    Protocol     │ • Line buffering     │ • Full duplex ISR    │
 * │                 │ • Command parsing    │ • Command protocol   │
 * └─────────────────┴──────────────────────┴──────────────────────┘
 *
 * CRITICAL EDUCATIONAL POINTS FOR STUDENTS:
 *
 * 1. ECHO PATTERN (RX + TX TOGETHER):
 *    - All demos receive AND transmit (realistic communication)
 *    - Students see complete data flow: input → process → output
 *    - Enables direct polling vs interrupt comparison
 *
 * 2. REAL ISR PROGRAMMING:
 *    - No wrapper functions or callback managers
 *    - Direct ISR(USART1_RX_vect) and ISR(USART1_UDRE_vect)
 *    - Students see actual interrupt vector names
 *    - Learn proper ISR syntax and timing constraints
 *
 * 3. DIRECT REGISTER PROGRAMMING:
 *    - UCSR1B |= (1 << RXCIE1) to enable RX interrupts
 *    - UCSR1B |= (1 << UDRIE1) to enable TX interrupts
 *    - UDR1 register for data read/write
 *    - sei() and cli() for global interrupt control
 *
 * 4. VOLATILE VARIABLES:
 *    - All ISR-accessed variables marked volatile
 *    - Students learn why volatile is essential
 *    - Understand shared data between ISR and main()
 *
 * 5. CIRCULAR BUFFER IMPLEMENTATION:
 *    - Head/tail pointers for FIFO operation
 *    - Buffer overflow detection and handling
 *    - Atomic operations for data integrity
 *
 * 6. PERFORMANCE COMPARISON:
 *    - Polling blocks CPU (inefficient but simple)
 *    - Interrupts free CPU (efficient but complex)
 *    - Real-time responsiveness differences
 *
 * LEARNING PROGRESSION:
 * Start with Demo 1 (simplest: polling single character)
 * Progress to Demo 6 (most complex: interrupt sentence protocol)
 * Compare horizontally: same data type, different method
 * Compare vertically: same method, increasing complexity
 *
 * =============================================================================
 * UART CONTROL REGISTERS - DETAILED REFERENCE FOR STUDENTS
 * =============================================================================
 *
 * REGISTER 1: UCSR1A (USART Control and Status Register A)
 * ┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
 * │ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
 * ├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
 * │Name │RXC1 │TXC1 │UDRE1│ FE1 │DOR1 │UPE1 │U2X1 │MPCM1│
 * └─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
 *
 * RXC1  (bit 7): Receive Complete Flag (read-only)
 *                Set when data received in UDR1, cleared by reading UDR1
 *                Use: while (!(UCSR1A & (1<<RXC1))); // Wait for data
 *
 * TXC1  (bit 6): Transmit Complete Flag
 *                Set when entire frame sent, cleared by writing 1 to it
 *                Use: Check if transmission fully completed
 *
 * UDRE1 (bit 5): Data Register Empty Flag (read-only)
 *                Set when UDR1 ready for new data, cleared by writing UDR1
 *                Use: while (!(UCSR1A & (1<<UDRE1))); // Wait until ready
 *
 * FE1   (bit 4): Frame Error (read-only)
 *                Set if stop bit not detected correctly
 *
 * DOR1  (bit 3): Data OverRun (read-only)
 *                Set if new data received before old data was read
 *
 * UPE1  (bit 2): Parity Error (read-only)
 *                Set if parity check failed (when parity enabled)
 *
 * U2X1  (bit 1): Double Transmission Speed
 *                1 = double speed mode (reduces baud error)
 *                Use: UCSR1A = (1<<U2X1); // Enable U2X for better accuracy
 *
 * MPCM1 (bit 0): Multi-processor Communication Mode
 *                Usually 0 for normal operation
 *
 * -----------------------------------------------------------------------------
 *
 * REGISTER 2: UCSR1B (USART Control and Status Register B)
 * ┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
 * │ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
 * ├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
 * │Name │RXCIE│TXCIE│UDRIE│RXEN1│TXEN1│UCSZ │ RXB8│ TXB8│
 * │     │  1  │  1  │  1  │     │     │  12 │   1 │   1 │
 * └─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
 *
 * RXCIE1 (bit 7): RX Complete Interrupt Enable **[INTERRUPT MODE]**
 *                 1 = Enable interrupt when RXC1 flag set (data received)
 *                 Use: UCSR1B |= (1<<RXCIE1); // Enable RX interrupt
 *                 ISR: ISR(USART1_RX_vect) { data = UDR1; }
 *
 * TXCIE1 (bit 6): TX Complete Interrupt Enable
 *                 1 = Enable interrupt when TXC1 flag set (transmission done)
 *                 Rarely used (UDRIE1 is more common for continuous TX)
 *
 * UDRIE1 (bit 5): Data Register Empty Interrupt Enable **[INTERRUPT MODE]**
 *                 1 = Enable interrupt when UDRE1 flag set (ready for data)
 *                 Use: UCSR1B |= (1<<UDRIE1); // Enable TX interrupt
 *                 ISR: ISR(USART1_UDRE_vect) { UDR1 = tx_data; }
 *                 CRITICAL: Disable when buffer empty to prevent infinite ISR!
 *
 * RXEN1  (bit 4): Receiver Enable **[REQUIRED]**
 *                 1 = Enable UART receiver
 *                 Use: UCSR1B |= (1<<RXEN1); // Must set to receive data
 *
 * TXEN1  (bit 3): Transmitter Enable **[REQUIRED]**
 *                 1 = Enable UART transmitter
 *                 Use: UCSR1B |= (1<<TXEN1); // Must set to transmit data
 *
 * UCSZ12 (bit 2): Character Size bit 2 (combine with UCSR1C bits)
 *                 For 8-bit data: UCSZ12=0, UCSZ11=1, UCSZ10=1
 *
 * RXB81  (bit 1): 9th receive data bit (for 9-bit mode)
 * TXB81  (bit 0): 9th transmit data bit (for 9-bit mode)
 *
 * COMMON USAGE PATTERNS:
 * Polling Mode:  UCSR1B = (1<<RXEN1) | (1<<TXEN1);
 * Interrupt Mode: UCSR1B = (1<<RXCIE1) | (1<<RXEN1) | (1<<TXEN1);
 *                 Later add: UCSR1B |= (1<<UDRIE1); // When data to send
 *
 * -----------------------------------------------------------------------------
 *
 * REGISTER 3: UCSR1C (USART Control and Status Register C)
 * ┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
 * │ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
 * ├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
 * │Name │UMSEL│ UPM1│ UPM1│USBS1│UCSZ1│UCSZ1│UCPOL│     │
 * │     │  1  │  1  │  0  │     │  1  │  0  │  1  │     │
 * └─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
 *
 * UMSEL1 (bit 7): USART Mode Select
 *                 0 = Asynchronous mode (standard UART)
 *                 1 = Synchronous mode (requires clock)
 *
 * UPM11:0 (bits 5-4): Parity Mode
 *                     00 = No parity (most common: 8N1)
 *                     10 = Even parity
 *                     11 = Odd parity
 *
 * USBS1 (bit 3): Stop Bit Select
 *                0 = 1 stop bit (standard)
 *                1 = 2 stop bits
 *
 * UCSZ11:10 (bits 2-1): Character Size (with UCSZ12 in UCSR1B)
 *                       011 = 8-bit data (UCSZ12=0, UCSZ11=1, UCSZ10=1)
 *                       Use: UCSR1C = (1<<UCSZ11) | (1<<UCSZ10); // 8-bit
 *
 * UCPOL1 (bit 0): Clock Polarity (synchronous mode only)
 *
 * COMMON USAGE (8N1 format):
 * UCSR1C = (1<<UCSZ11) | (1<<UCSZ10); // 8-bit, no parity, 1 stop bit
 *
 * -----------------------------------------------------------------------------
 *
 * REGISTER 4: UBRR1H and UBRR1L (Baud Rate Registers)
 * ┌─────────────┬─────────────────────────────────────┐
 * │  Register   │          Purpose                    │
 * ├─────────────┼─────────────────────────────────────┤
 * │ UBRR1H      │ High 4 bits of baud rate divisor    │
 * │ UBRR1L      │ Low 8 bits of baud rate divisor     │
 * └─────────────┴─────────────────────────────────────┘
 *
 * BAUD RATE CALCULATION (U2X1=1, double speed mode):
 * UBRR = (F_CPU / (8 * BAUD)) - 1
 *
 * Example for 9600 baud @ 16MHz:
 * UBRR = (16000000 / (8 * 9600)) - 1 = 207.33 ≈ 207
 * Error = ((16000000/(8*(207+1)))/9600 - 1) × 100% = 0.16% (acceptable)
 *
 * Common Values (F_CPU=16MHz, U2X=1):
 * ┌───────────┬────────┬────────────┐
 * │ Baud Rate │  UBRR  │   Error    │
 * ├───────────┼────────┼────────────┤
 * │   2400    │  832   │   0.0%     │
 * │   4800    │  416   │   0.0%     │
 * │   9600    │  207   │   0.16%    │
 * │  19200    │  103   │   0.16%    │
 * │  38400    │   51   │   0.16%    │
 * │  57600    │   34   │  -0.79%    │
 * │ 115200    │   16   │   2.12%    │
 * └───────────┴────────┴────────────┘
 *
 * Usage:
 * uint16_t ubrr = 207; // For 9600 baud
 * UBRR1H = (ubrr >> 8);  // High byte
 * UBRR1L = ubrr;         // Low byte
 *
 * -----------------------------------------------------------------------------
 *
 * REGISTER 5: UDR1 (USART Data Register)
 * ┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
 * │ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
 * ├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
 * │Data │ D7  │ D6  │ D5  │ D4  │ D3  │ D2  │ D1  │ D0  │
 * └─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
 *
 * This is the actual data register - read to receive, write to transmit
 *
 * RX Usage (Polling):
 *   while (!(UCSR1A & (1<<RXC1))); // Wait for data
 *   char data = UDR1;               // Read received byte
 *
 * TX Usage (Polling):
 *   while (!(UCSR1A & (1<<UDRE1))); // Wait until ready
 *   UDR1 = data;                    // Send byte
 *
 * RX Usage (Interrupt):
 *   ISR(USART1_RX_vect) {
 *       char data = UDR1; // Reading UDR1 clears RXC1 flag
 *       // Process data...
 *   }
 *
 * TX Usage (Interrupt):
 *   ISR(USART1_UDRE_vect) {
 *       UDR1 = next_byte; // Writing UDR1 clears UDRE1 flag
 *   }
 *
 * =============================================================================
 * INTERRUPT SYSTEM - DETAILED REFERENCE FOR STUDENTS
 * =============================================================================
 *
 * GLOBAL INTERRUPT CONTROL
 * ┌──────────────────┬─────────────────────────────────────────┐
 * │   Function       │   Description                           │
 * ├──────────────────┼─────────────────────────────────────────┤
 * │ sei()            │ Set Global Interrupt Enable (I-bit)     │
 * │                  │ MUST call after enabling RXCIE1/UDRIE1  │
 * │                  │ Use: sei(); // Enable interrupts        │
 * ├──────────────────┼─────────────────────────────────────────┤
 * │ cli()            │ Clear Global Interrupt Enable           │
 * │                  │ Disables ALL interrupts temporarily     │
 * │                  │ Use: cli(); // Critical section         │
 * └──────────────────┴─────────────────────────────────────────┘
 *
 * UART INTERRUPT VECTORS (ATmega128)
 * ┌───────────────────────┬──────────────┬───────────────────────┐
 * │ Interrupt Vector      │ Trigger      │ Enable Bit            │
 * ├───────────────────────┼──────────────┼───────────────────────┤
 * │ ISR(USART1_RX_vect)   │ RXC1 flag    │ RXCIE1 (UCSR1B bit 7) │
 * │                       │ Data arrived │ ISR called when byte  │
 * │                       │ in UDR1      │ received              │
 * ├───────────────────────┼──────────────┼───────────────────────┤
 * │ ISR(USART1_UDRE_vect) │ UDRE1 flag   │ UDRIE1 (UCSR1B bit 5) │
 * │                       │ UDR1 empty   │ ISR called when ready │
 * │                       │ and ready    │ to send next byte     │
 * ├───────────────────────┼──────────────┼───────────────────────┤
 * │ ISR(USART1_TX_vect)   │ TXC1 flag    │ TXCIE1 (UCSR1B bit 6) │
 * │                       │ Frame fully  │ Rarely used           │
 * │                       │ transmitted  │                       │
 * └───────────────────────┴──────────────┴───────────────────────┘
 *
 * ISR PROGRAMMING RULES (CRITICAL FOR STUDENTS):
 *
 * 1. KEEP ISR SHORT AND FAST
 *    - ISRs block other interrupts (unless nested)
 *    - Do minimal work: read/write data, update flags, return quickly
 *    - DON'T: Call printf(), delay(), complex calculations
 *    - DO: Read UDR1, write to buffer, set flags
 *
 * 2. USE VOLATILE FOR SHARED VARIABLES
 *    volatile uint8_t rx_buffer[32];
 *    volatile uint8_t rx_head = 0, rx_tail = 0;
 *    - Prevents compiler optimization that breaks ISR/main() communication
 *    - ALL variables accessed by both ISR and main() MUST be volatile
 *
 * 3. ATOMIC OPERATIONS FOR MULTI-BYTE DATA
 *    cli();                    // Disable interrupts
 *    uint16_t count = counter; // Read multi-byte variable
 *    sei();                    // Re-enable interrupts
 *
 * 4. FLAG CLEARING IS AUTOMATIC FOR UART
 *    - Reading UDR1 clears RXC1 flag automatically
 *    - Writing UDR1 clears UDRE1 flag automatically
 *    - No manual flag clearing needed!
 *
 * 5. DISABLE UDRIE1 WHEN NO DATA TO SEND
 *    ISR(USART1_UDRE_vect) {
 *        if (tx_head == tx_tail) {
 *            UCSR1B &= ~(1<<UDRIE1); // DISABLE interrupt when buffer empty
 *            return;
 *        }
 *        UDR1 = tx_buffer[tx_tail++];
 *    }
 *    - UDRIE1 fires CONTINUOUSLY when UDR1 empty
 *    - Must disable to prevent infinite ISR calls
 *
 * INITIALIZATION SEQUENCE (INTERRUPT MODE):
 *
 * Step 1: Configure UART hardware
 *   UCSR1A = (1<<U2X1);                    // Double speed
 *   UCSR1C = (1<<UCSZ11) | (1<<UCSZ10);   // 8-bit data
 *   UBRR1H = (UBRR >> 8);                  // Baud rate high
 *   UBRR1L = UBRR;                         // Baud rate low
 *
 * Step 2: Enable RX/TX and RX interrupt
 *   UCSR1B = (1<<RXCIE1) | (1<<RXEN1) | (1<<TXEN1);
 *   // Note: UDRIE1 NOT enabled yet (enable only when data to send)
 *
 * Step 3: Enable global interrupts
 *   sei(); // Now ISRs can fire
 *
 * Step 4: To send data later
 *   tx_buffer[tx_head++] = data;    // Put data in buffer
 *   UCSR1B |= (1<<UDRIE1);          // Enable TX interrupt
 *
 * =============================================================================
 * CIRCULAR BUFFER IMPLEMENTATION - FOR INTERRUPT MODE
 * =============================================================================
 *
 * PURPOSE: Decouple ISR from main() for efficient buffering
 *
 * STRUCTURE:
 * ┌───┬───┬───┬───┬───┬───┬───┬───┐
 * │ 0 │ 1 │ 2 │ 3 │ 4 │ 5 │ 6 │ 7 │ ... buffer[32]
 * └───┴───┴───┴───┴───┴───┴───┴───┘
 *   ↑               ↑
 *  tail            head
 *  (read)         (write)
 *
 * VARIABLES (ALL MUST BE VOLATILE):
 *   volatile uint8_t buffer[BUFFER_SIZE];
 *   volatile uint8_t head = 0; // Next write position
 *   volatile uint8_t tail = 0; // Next read position
 *
 * OPERATIONS:
 *
 * Write (by ISR):
 *   buffer[head] = data;
 *   head = (head + 1) % BUFFER_SIZE; // Wrap around
 *
 * Read (by main):
 *   if (head != tail) {              // Check not empty
 *       data = buffer[tail];
 *       tail = (tail + 1) % BUFFER_SIZE;
 *   }
 *
 * Check if empty:
 *   if (head == tail) // Buffer empty
 *
 * Check if full:
 *   if (((head + 1) % BUFFER_SIZE) == tail) // Buffer full
 *
 * =============================================================================
 */

#include "config.h"
#include <avr/pgmspace.h> // For PROGMEM string storage in flash

// Forward declaration for UART functions
void putch_USART1(char c);

// Helper function to send strings from flash memory
void puts_USART1_P(const char *str)
{
    char c;
    while ((c = pgm_read_byte(str++)))
    {
        putch_USART1(c);
    }
}

// Function prototypes for 6 educational demos (2×3 matrix)
void simple_init_serial(void);

// POLLING METHOD DEMOS (Left column: Simple but CPU-blocking)
void demo_polling_char_echo(void);     // Demo 1: Single character echo (polling)
void demo_polling_word_echo(void);     // Demo 2: Word echo with space delimiter (polling)
void demo_polling_sentence_echo(void); // Demo 3: Sentence echo with line delimiter (polling)

// INTERRUPT METHOD DEMOS (Right column: Complex but CPU-efficient)
void demo_interrupt_char_echo(void);     // Demo 4: Single character echo (interrupt)
void demo_interrupt_word_echo(void);     // Demo 5: Word echo with circular buffer (interrupt)
void demo_interrupt_sentence_echo(void); // Demo 6: Sentence echo with full duplex (interrupt)

/*
 * =============================================================================
 * DEMO SELECTION GUIDE - 2×3 PEDAGOGICAL MATRIX
 * =============================================================================
 *
 * USAGE: In main(), uncomment ONE demo to run. Compare across rows or columns:
 *
 * ┌──────────────────────────────────────────────────────────────────┐
 * │                  POLLING (Simple/Blocking)                       │
 * ├──────────────────────────────────────────────────────────────────┤
 * │ Demo 1: demo_polling_char_echo()                                 │
 * │         Single character echo - learn basic getch/putch          │
 * │         Shows how CPU blocks waiting for each character          │
 * │                                                                  │
 * │ Demo 2: demo_polling_word_echo()                                 │
 * │         Word echo (space-delimited) - learn manual buffering     │
 * │         Shows polling with simple parsing logic                  │
 * │                                                                  │
 * │ Demo 3: demo_polling_sentence_echo()                             │
 * │         Sentence echo (line-delimited) - learn protocol design   │
 * │         Shows polling with command processing                    │
 * └──────────────────────────────────────────────────────────────────┘
 *
 * ┌──────────────────────────────────────────────────────────────────┐
 * │                INTERRUPT (Efficient/Non-blocking)                │
 * ├──────────────────────────────────────────────────────────────────┤
 * │ Demo 4: demo_interrupt_char_echo()                               │
 * │         Single character echo - learn ISR programming            │
 * │         Shows how CPU continues other work during RX/TX          │
 * │         Compare with Demo 1 to see efficiency difference!        │
 * │                                                                  │
 * │ Demo 5: demo_interrupt_word_echo()                               │
 * │         Word echo with ISR - learn circular buffer management    │
 * │         Shows interrupt-driven parsing and buffering             │
 * │         Compare with Demo 2 to see CPU freedom!                  │
 * │                                                                  │
 * │ Demo 6: demo_interrupt_sentence_echo()                           │
 * │         Sentence echo with ISR - learn full duplex protocol      │
 * │         Shows advanced ISR with command processing               │
 * │         Compare with Demo 3 to see true non-blocking I/O!        │
 * └──────────────────────────────────────────────────────────────────┘
 *
 * LEARNING STRATEGY:
 * - Horizontal comparison: Compare Demo 1 vs Demo 4 (same granularity)
 * - Vertical progression: Go from Demo 1 → Demo 2 → Demo 3 (increasing complexity)
 * - Full mastery: Understand all 6 demos and their trade-offs
 */

// Simple initialization function (no LCD needed for serial communication)
void simple_init_serial(void)
{
    // Initialize basic I/O ports
    PORTA = 0xFF;
    DDRA = 0x00; // Set PORTA as input with pull-ups
    PORTB = 0x00;
    DDRB = 0xFF; // Set PORTB as output

    // Note: UART initialization is handled by each individual demo
    // This allows clean demonstration of different initialization approaches
} /*
   * =============================================================================
   * EDUCATIONAL UART FUNCTIONS - DIRECT REGISTER PROGRAMMING
   * =============================================================================
   * These functions show students the exact register operations for UART communication.
   * Students learn to work directly with UART registers without wrapper functions.
   */

// Initialize UART1 for 9600 baud, 8N1 format - EDUCATIONAL VERSION
void init_uart_polling(void)
{
    // Step 1: Configure UART Control Register A (U2X=1 for better accuracy)
    UCSR1A = UART_U2X_ENABLE; // U2X=1 for double-speed mode

    // Step 2: Configure character format (8 data bits, No parity, 1 stop bit = 8N1)
    UCSR1C = UART_8BIT_CHAR; // UCSZ11:10 = 11 for 8-bit character size

    // Step 3: Enable transmitter and receiver (NO INTERRUPTS for polling)
    UCSR1B = UART_ENABLE_RX_TX; // RXEN1 and TXEN1

    // Step 4: Calculate and set baud rate
    // Formula with U2X=1: UBRR = (F_CPU / (8 * BAUD)) - 1
    // For 16MHz and 9600 baud: UBRR = (16000000 / (8 * 9600)) - 1 = 207
    unsigned int baud_register = UART_BAUD_REGISTER;
    UBRR1H = (baud_register >> 8); // High byte of baud rate register
    UBRR1L = baud_register;        // Low byte of baud rate register

    // Step 5: Allow UART hardware to stabilize (CRITICAL for first character)
    _delay_ms(10); // Small delay for UART register stabilization
}

// Send single character via UART1 - EDUCATIONAL VERSION
void putch_USART1(char c)
{
    // Wait for transmit buffer to be empty
    // Students learn to check UDRE1 (USART Data Register Empty) flag
    while (!(UCSR1A & (1 << UDRE1)))
    {
        // Busy wait - this is polling!
        // CPU is blocked here until transmitter is ready
    }

    // Put data into buffer, sends the data
    UDR1 = c;
}

// Send string via UART1 - EDUCATIONAL VERSION
void puts_USART1(const char *str)
{
    while (*str != '\0')
    {
        putch_USART1(*str);
        str++;
    }
}

// Receive single character via UART1 - EDUCATIONAL VERSION
char getch_USART1(void)
{
    // Wait for data to be received
    // Students learn to check RXC1 (Receive Complete) flag
    while (!(UCSR1A & (1 << RXC1)))
    {
        // Busy wait - this is polling!
        // CPU is blocked here until data arrives
    }

    // Get and return received data from buffer
    return UDR1;
}

// Check if data is available for reading - EDUCATIONAL VERSION
unsigned char data_available_USART1(void)
{
    // Students learn to check RXC1 flag without blocking
    return (UCSR1A & (1 << RXC1)) ? 1 : 0;
}

/*
 * =============================================================================
 * POLLING-BASED SERIAL COMMUNICATION DEMOS
 * =============================================================================
 * These demos use polling (busy-waiting) to check for received data.
 * Advantages: Simple to understand and implement
 * Disadvantages: CPU is blocked while waiting, inefficient
 */

/*
 * =============================================================================
 * DEMO 1: POLLING CHARACTER ECHO
 * =============================================================================
 * DATA GRANULARITY: Single character
 * METHOD: Polling (CPU blocks on each character)
 *
 * LEARNING FOCUS:
 * - Basic UART functions: getch_USART1() and putch_USART1()
 * - Understanding CPU blocking during polling
 * - Simple echo pattern: receive → send back
 *
 * COMPARE WITH: Demo 4 (interrupt character echo) to see CPU efficiency difference
 */
void demo_polling_char_echo(void)
{
    // Initialize UART for polling communication
    init_uart_polling();

    // Store strings in flash memory to save SRAM
    puts_USART1_P(PSTR("\r\n=== DEMO 1: Polling Char Echo ===\r\n"));
    puts_USART1_P(PSTR("Polling: CPU blocks. Type chars, press 'q' to quit.\r\n\r\n"));

    unsigned int char_count = 0;
    char received;

    while (1)
    {
        // EDUCATIONAL POINT: CPU is BLOCKED here waiting for character
        // No other work can be done during this wait!
        received = getch_USART1(); // <-- CPU BLOCKS HERE
        char_count++;

        // Exit condition
        if (received == 'q' || received == 'Q')
        {
            break;
        }

        // Echo character back (also blocks during transmission)
        putch_USART1(received); // <-- CPU BLOCKS HERE TOO

        // Show statistics periodically
        if ((char_count % 10) == 0)
        {
            puts_USART1(" [");
            putch_USART1('0' + (char_count / 10) % 10);
            putch_USART1('0' + char_count % 10);
            puts_USART1(" chars, CPU blocked every time]");
        }
    }

    puts_USART1("\r\n\r\n[DEMO 1 COMPLETE]\r\n");
    puts_USART1("Total characters echoed: ");
    putch_USART1('0' + (char_count / 10) % 10);
    putch_USART1('0' + char_count % 10);
    puts_USART1("\r\nCPU was blocked ");
    putch_USART1('0' + (char_count / 10) % 10);
    putch_USART1('0' + char_count % 10);
    puts_USART1(" times waiting for I/O\r\n");
    puts_USART1("Compare this with Demo 4 (interrupt method)!\r\n\r\n");
}

/*
 * =============================================================================
 * DEMO 2: POLLING WORD ECHO
 * =============================================================================
 * DATA GRANULARITY: Word (space-delimited)
 * METHOD: Polling with manual buffer
 *
 * LEARNING FOCUS:
 * - Manual buffer management for word assembly
 * - Character-by-character polling with parsing
 * - Space as word delimiter
 * - CPU still blocks but now processes complete words
 *
 * COMPARE WITH: Demo 5 (interrupt word echo) to see buffer efficiency
 */
void demo_polling_word_echo(void)
{
    puts_USART1_P(PSTR("\r\n=== DEMO 2: Polling Word Echo ===\r\n"));
    puts_USART1_P(PSTR("Polling: words echo on space. Type 'quit' to exit.\r\n\r\n"));

    char word_buffer[32];
    unsigned char word_index = 0;
    char received;
    unsigned int word_count = 0;

    while (1)
    {
        // POLLING: CPU blocks waiting for each character
        received = getch_USART1(); // <-- CPU BLOCKS

        // Echo the character for user feedback
        putch_USART1(received);

        // Check for word delimiter (space or enter)
        if (received == ' ' || received == '\r' || received == '\n')
        {
            if (word_index > 0)
            {
                word_buffer[word_index] = '\0'; // Null-terminate the word
                word_count++;

                // Check for quit command
                if (strcmp(word_buffer, "quit") == 0)
                {
                    puts_USART1("\r\n[Exiting Demo 2]\r\n");
                    break;
                }

                // Echo the complete word back
                puts_USART1(" → ECHO: [");
                puts_USART1(word_buffer);
                puts_USART1("] ");

                // Show statistics
                if ((word_count % 5) == 0)
                {
                    puts_USART1(" (");
                    putch_USART1('0' + (word_count / 10) % 10);
                    putch_USART1('0' + word_count % 10);
                    puts_USART1(" words, CPU blocked for each char)");
                }

                puts_USART1("\r\n");

                // Reset buffer for next word
                word_index = 0;
            }
        }
        // Handle backspace
        else if (received == '\b' || received == 127)
        {
            if (word_index > 0)
            {
                word_index--;
                puts_USART1(" \b"); // Erase character from screen
            }
        }
        // Add character to word buffer
        else if (word_index < 31 && received >= ' ')
        {
            word_buffer[word_index++] = received;
        }
    }

    puts_USART1("\r\n[DEMO 2 COMPLETE]\r\n");
    puts_USART1("Total words echoed: ");
    putch_USART1('0' + (word_count / 10) % 10);
    putch_USART1('0' + word_count % 10);
    puts_USART1("\r\nCPU blocked on every character, echoed complete words\r\n");
    puts_USART1("Compare this with Demo 5 (interrupt word echo)!\r\n\r\n");
}

/*
 * =============================================================================
 * DEMO 3: POLLING SENTENCE ECHO
 * =============================================================================
 * DATA GRANULARITY: Sentence (line-delimited, multi-word)
 * METHOD: Polling with line buffering
 *
 * LEARNING FOCUS:
 * - Line buffering (collecting until Enter pressed)
 * - Multi-word parsing and protocol design
 * - Complete line echo pattern
 * - CPU blocked but processes full sentences
 *
 * COMPARE WITH: Demo 6 (interrupt sentence echo) for full duplex efficiency
 */
void demo_polling_sentence_echo(void)
{
    puts_USART1_P(PSTR("\r\n=== DEMO 3: Polling Sentence Echo ===\r\n"));
    puts_USART1_P(PSTR("Polling: sentences echo on Enter. Type 'quit' to exit.\r\n\r\n"));

    char line_buffer[64];
    unsigned char line_index = 0;
    char received;
    unsigned int line_count = 0;

    puts_USART1("Type sentence> ");

    while (1)
    {
        // POLLING: CPU blocks waiting for each character
        received = getch_USART1(); // <-- CPU BLOCKS

        // Echo character for user feedback
        putch_USART1(received);

        // Check for line delimiter (Enter key)
        if (received == '\r' || received == '\n')
        {
            if (line_index > 0)
            {
                line_buffer[line_index] = '\0'; // Null-terminate
                line_count++;

                // Check for quit command
                if (strcmp(line_buffer, "quit") == 0)
                {
                    puts_USART1("\r\n[Exiting Demo 3]\r\n");
                    break;
                }

                // Echo the complete sentence back
                puts_USART1("\r\n→ SENTENCE ECHO: \"");
                puts_USART1(line_buffer);
                puts_USART1("\"\r\n");

                // Show statistics every 3 sentences
                if ((line_count % 3) == 0)
                {
                    puts_USART1("   [");
                    putch_USART1('0' + (line_count / 10) % 10);
                    putch_USART1('0' + line_count % 10);
                    puts_USART1(" sentences, ");
                    putch_USART1('0' + (line_index / 10) % 10);
                    putch_USART1('0' + line_index % 10);
                    puts_USART1(" chars, CPU blocked on each]\r\n");
                }

                puts_USART1("Type sentence> ");

                // Reset buffer for next sentence
                line_index = 0;
            }
        }
        // Handle backspace
        else if (received == '\b' || received == 127)
        {
            if (line_index > 0)
            {
                line_index--;
                puts_USART1(" \b"); // Erase character from screen
            }
        }
        // Add character to line buffer
        else if (line_index < 63 && received >= ' ')
        {
            line_buffer[line_index++] = received;
        }
        // Buffer overflow warning
        else if (line_index >= 63)
        {
            puts_USART1("\r\n[BUFFER FULL - Press Enter]\r\n");
        }
    }

    puts_USART1("\r\n[DEMO 3 COMPLETE]\r\n");
    puts_USART1("Total sentences echoed: ");
    putch_USART1('0' + (line_count / 10) % 10);
    putch_USART1('0' + line_count % 10);
    puts_USART1("\r\nCPU blocked for every character, echoed complete sentences\r\n");
    puts_USART1("This is the most common polling pattern for command-line interfaces\r\n");
    puts_USART1("Compare this with Demo 6 (interrupt sentence echo)!\r\n\r\n");
}

/*
 * =============================================================================
 * INTERRUPT COMMUNICATION GLOBAL VARIABLES
 * =============================================================================
 */

// Receive buffer and control variables
#define RX_BUFFER_SIZE 32 // Reduced from 64 to save memory
volatile char rx_buffer[RX_BUFFER_SIZE];
volatile unsigned char rx_head = 0;
volatile unsigned char rx_tail = 0;
volatile unsigned char rx_overflow = 0;

// Transmit buffer and control variables
#define TX_BUFFER_SIZE 32 // Reduced from 64 to save memory
volatile char tx_buffer[TX_BUFFER_SIZE];
volatile unsigned char tx_head = 0;
volatile unsigned char tx_tail = 0;
volatile unsigned char tx_busy = 0;

// Communication status and control
volatile unsigned char new_command_received = 0;
volatile unsigned char communication_mode = 0;
volatile unsigned char error_count = 0;

// Command processing variables
volatile char command_buffer[16]; // Reduced from 32 to save memory
volatile unsigned char command_length = 0;
volatile unsigned char command_ready = 0;

/*
 * =============================================================================
 * CUSTOM INTERRUPT HANDLERS FOR SERIAL_INTERRUPT PROJECT
 * =============================================================================
 */

/*
 * =============================================================================
 * EDUCATIONAL INTERRUPT SERVICE ROUTINES
 * =============================================================================
 * These are the actual ISRs that students must learn to write.
 * No wrappers or managers - direct hardware programming!
 */

/*
 * =============================================================================
 * EDUCATIONAL INTERRUPT SERVICE ROUTINES
 * =============================================================================
 * These are the actual ISRs that students must learn to write.
 * No wrappers or managers - direct hardware programming!
 */

/*
 * USART1 Receive Complete Interrupt
 * This ISR is called automatically when a character is received
 * Students learn: ISR syntax, volatile variables, circular buffers
 */
ISR(USART1_RX_vect)
{
    char received = UDR1; // Read the received character
    unsigned char next_head = (rx_head + 1) % RX_BUFFER_SIZE;

    // Check for buffer overflow
    if (next_head != rx_tail)
    {
        rx_buffer[rx_head] = received;
        rx_head = next_head;
    }
    else
    {
        rx_overflow = 1; // Flag overflow for debugging
        error_count++;
    }
}

/*
 * USART1 Data Register Empty Interrupt
 * This ISR is called when the transmit buffer is ready for next character
 * Students learn: TX interrupts, automatic transmission, buffer management
 */
ISR(USART1_UDRE_vect)
{
    if (tx_head != tx_tail)
    {
        // Send next character from buffer
        UDR1 = tx_buffer[tx_tail];
        tx_tail = (tx_tail + 1) % TX_BUFFER_SIZE;
    }
    else
    {
        // Buffer empty - disable this interrupt
        UCSR1B &= ~(1 << UDRIE1);
        tx_busy = 0;
    }
}

/*
 * =============================================================================
 * INTERRUPT COMMUNICATION SETUP FUNCTIONS
 * =============================================================================
 */

/*
 * Initialize UART1 with proper interrupt configuration
 * Students learn: Direct register programming, interrupt enable bits
 */
void init_uart_interrupts(void)
{
    // EDUCATIONAL UART INITIALIZATION - Direct Register Programming
    // Students learn the exact steps for UART setup:

    // Step 1: Configure UART Control Register A (U2X=1 for better accuracy)
    UCSR1A = UART_U2X_ENABLE; // U2X=1 for double-speed mode

    // Step 2: Configure character format (8 data bits, No parity, 1 stop bit = 8N1)
    UCSR1C = UART_8BIT_CHAR; // UCSZ11:10 = 11 for 8-bit character size

    // Step 3: Enable transmitter and receiver
    UCSR1B = UART_ENABLE_RX_TX; // RXEN1 and TXEN1

    // Step 4: Calculate and set baud rate
    // Formula with U2X=1: UBRR = (F_CPU / (8 * BAUD)) - 1
    // For 16MHz and 9600 baud: UBRR = (16000000 / (8 * 9600)) - 1 = 207
    unsigned int baud_register = UART_BAUD_REGISTER;
    UBRR1H = (baud_register >> 8); // High byte of baud rate register
    UBRR1L = baud_register;        // Low byte of baud rate register

    // Step 5: Allow UART hardware to stabilize (CRITICAL for first character)
    _delay_ms(10); // Small delay for UART register stabilization

    // NOW THE EDUCATIONAL PART: Direct interrupt setup
    // Students learn these exact register operations:

    // Enable RX Complete Interrupt
    UCSR1B |= (1 << RXCIE1);

    // Enable global interrupts (students must understand this!)
    sei();

    // Clear our educational buffers
    rx_head = rx_tail = 0;
    tx_head = tx_tail = 0;
    rx_overflow = 0;
    tx_busy = 0;
    error_count = 0;
}

/*
 * Send character using interrupt-driven transmission
 */
unsigned char send_char_interrupt(char data)
{
    unsigned char next_head = (tx_head + 1) % TX_BUFFER_SIZE;

    // Check if buffer full
    if (next_head == tx_tail)
    {
        return 0; // Buffer full
    }

    // Add to buffer
    tx_buffer[tx_head] = data;
    tx_head = next_head;

    // Enable transmit interrupt if not busy
    if (!tx_busy)
    {
        tx_busy = 1;
        UCSR1B |= (1 << UDRIE1);
    }

    return 1; // Success
}

/*
 * Send string using interrupt-driven transmission
 */
void send_string_interrupt(const char *str)
{
    while (*str)
    {
        while (!send_char_interrupt(*str))
            ; // Wait if buffer full
        str++;
    }
}

/*
 * Check if characters available in RX buffer
 */
unsigned char chars_available(void)
{
    return (rx_head != rx_tail);
}

/*
 * Get character from RX buffer
 */
char get_char_from_buffer(void)
{
    char data;

    if (rx_head == rx_tail)
    {
        return 0; // Buffer empty
    }

    data = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;

    return data;
}

/*
 * =============================================================================
 * DEMONSTRATION MODE FUNCTIONS
 * =============================================================================
 */

/*
 * =============================================================================
 * SERIAL COMMUNICATION DEMO FUNCTIONS
 * =============================================================================
 */

/*
 * =============================================================================
 * DEMO 4: INTERRUPT CHARACTER ECHO
 * =============================================================================
 * DATA GRANULARITY: Single character
 * METHOD: Interrupt-driven with ISR (CPU non-blocking)
 *
 * LEARNING FOCUS:
 * - Real ISR programming: ISR(USART1_RX_vect) and ISR(USART1_UDRE_vect)
 * - Circular buffer for RX and TX
 * - CPU continues other work while ISRs handle I/O
 * - Volatile variables for ISR-main communication
 *
 * COMPARE WITH: Demo 1 (polling character echo) to see CPU freedom!
 */
void demo_interrupt_char_echo(void)
{
    // EDUCATIONAL: Initialize interrupt-based UART (see ISRs above!)
    init_uart_interrupts();

    // Send initial messages using polling (before interrupts fully active)
    puts_USART1_P(PSTR("\r\n=== DEMO 4: Interrupt Char Echo ===\r\n"));
    puts_USART1_P(PSTR("Interrupt: CPU free! ISRs handle I/O. Press 'q' to quit.\r\n\r\n"));

    _delay_ms(100); // Let initial messages complete

    // EDUCATIONAL: Show students the difference - CPU is free to do other work!
    unsigned int counter = 0;
    char received;

    while (1)
    {
        // EDUCATIONAL POINT: CPU can do other work while ISR handles serial data!
        // This counter proves the CPU is not blocked waiting for serial data
        counter++;
        if ((counter % 20000) == 0)
        {
            // Show that CPU is free to do other tasks
            PORTB = ~PORTB; // Toggle LEDs to show CPU activity
        }

        // EDUCATIONAL: Check if our ISR has received data
        if (chars_available())
        {
            received = get_char_from_buffer();

            // Echo the character back using interrupt-driven TX
            while (!send_char_interrupt(received))
                ; // Wait if TX buffer full

            if (received == 'q' || received == 'Q')
            {
                break; // Exit demo
            }
        }

        // Show ISR buffer status periodically for debugging
        if ((counter % 100000) == 0)
        {
            if (rx_overflow)
            {
                puts_USART1("[ISR BUFFER OVERFLOW - too much data!]\r\n");
                rx_overflow = 0;
            }
        }
    }

    puts_USART1("\r\nInterrupt Demo 4 completed.\r\n");
    puts_USART1("Key Learning: CPU was free to count and toggle LEDs while ISRs handled all serial data!\r\n");
    puts_USART1("Compare this efficiency with polling demos above.\r\n");
}

/*
 * =============================================================================
 * DEMO 5: INTERRUPT WORD ECHO
 * =============================================================================
 * DATA GRANULARITY: Word (space-delimited)
 * METHOD: Interrupt-driven with ISR circular buffer
 *
 * LEARNING FOCUS:
 * - Word assembly in ISR buffer
 * - Space delimiter parsing with interrupts
 * - Non-blocking word collection
 * - Compare efficiency with Demo 2 (polling word echo)
 *
 * COMPARE WITH: Demo 2 (polling word echo) to see buffer management difference
 */
void demo_interrupt_word_echo(void)
{
    // Initialize UART for interrupt-based communication
    init_uart_interrupts();

    puts_USART1_P(PSTR("\r\n=== DEMO 5: Interrupt Word Echo ===\r\n"));
    puts_USART1_P(PSTR("Interrupt: words via ISR. Type 'quit' to exit.\r\n\r\n"));

    _delay_ms(100); // Let messages transmit

    char word_buffer[32];
    unsigned char word_index = 0;
    char received;
    unsigned int word_count = 0;
    unsigned int cpu_counter = 0;

    while (1)
    {
        // EDUCATIONAL: CPU is FREE to do other work!
        cpu_counter++;
        if ((cpu_counter % 50000) == 0)
        {
            // This proves CPU is not blocked!
            // In polling, this would never execute while waiting for input
        }

        // Check ISR buffer (non-blocking!)
        if (chars_available())
        {
            received = get_char_from_buffer();

            // Echo character back via ISR
            send_char_interrupt(received);

            // Word delimiter check
            if (received == ' ' || received == '\r' || received == '\n')
            {
                if (word_index > 0)
                {
                    word_buffer[word_index] = '\0';
                    word_count++;

                    // Check quit command
                    if (strcmp(word_buffer, "quit") == 0)
                    {
                        send_string_interrupt("\r\n[Exiting Demo 5]\r\n");
                        break;
                    }

                    // Echo complete word
                    send_string_interrupt(" → ECHO: [");
                    send_string_interrupt(word_buffer);
                    send_string_interrupt("]");

                    if ((word_count % 5) == 0)
                    {
                        send_string_interrupt(" (");
                        send_char_interrupt('0' + (word_count / 10) % 10);
                        send_char_interrupt('0' + word_count % 10);
                        send_string_interrupt(" words, CPU was FREE!)");
                    }
                    send_string_interrupt("\r\n");

                    word_index = 0;
                }
            }
            // Backspace
            else if (received == '\b' || received == 127)
            {
                if (word_index > 0)
                {
                    word_index--;
                    send_string_interrupt(" \b");
                }
            }
            // Add to buffer
            else if (word_index < 31 && received >= ' ')
            {
                word_buffer[word_index++] = received;
            }
        }

        // Small delay (CPU still free during this!)
        _delay_ms(5);
    }

    send_string_interrupt("\r\n[DEMO 5 COMPLETE]\r\n");
    send_string_interrupt("Words echoed: ");
    send_char_interrupt('0' + (word_count / 10) % 10);
    send_char_interrupt('0' + word_count % 10);
    send_string_interrupt("\r\nISRs handled ALL I/O, CPU was free!\r\n");
    send_string_interrupt("Compare with Demo 2 (polling word echo)!\r\n\r\n");
}

/*
 * =============================================================================
 * DEMO 6: INTERRUPT SENTENCE ECHO
 * =============================================================================
 * DATA GRANULARITY: Sentence (line-delimited, multi-word)
 * METHOD: Interrupt full duplex with command protocol
 *
 * LEARNING FOCUS:
 * - Full duplex ISR communication
 * - Line buffering with interrupts
 * - Command protocol design with ISR
 * - Complete non-blocking sentence processing
 *
 * COMPARE WITH: Demo 3 (polling sentence echo) for maximum efficiency gain
 */
void demo_interrupt_sentence_echo(void)
{
    // Initialize UART for interrupt-based communication
    init_uart_interrupts();

    puts_USART1_P(PSTR("\r\n=== DEMO 6: Interrupt Sentence Echo ===\r\n"));
    puts_USART1_P(PSTR("Interrupt: sentences via ISR. Type 'quit' to exit.\r\n\r\n"));

    _delay_ms(100);

    char line_buffer[64];
    unsigned char line_index = 0;
    char received;
    unsigned int line_count = 0;
    unsigned int cpu_counter = 0;

    send_string_interrupt("Type sentence> ");

    while (1)
    {
        // EDUCATIONAL: CPU is FREE to do other work!
        cpu_counter++;
        if ((cpu_counter % 50000) == 0)
        {
            // CPU continues working while ISRs handle I/O
        }

        // Check ISR buffer (non-blocking!)
        if (chars_available())
        {
            received = get_char_from_buffer();

            // Echo via ISR
            send_char_interrupt(received);

            // Line delimiter check
            if (received == '\r' || received == '\n')
            {
                if (line_index > 0)
                {
                    line_buffer[line_index] = '\0';
                    line_count++;

                    // Check quit
                    if (strcmp(line_buffer, "quit") == 0)
                    {
                        send_string_interrupt("\r\n[Exiting Demo 6]\r\n");
                        break;
                    }

                    // Echo complete sentence
                    send_string_interrupt("\r\n→ SENTENCE ECHO: \"");
                    send_string_interrupt(line_buffer);
                    send_string_interrupt("\"\r\n");

                    if ((line_count % 3) == 0)
                    {
                        send_string_interrupt("   [");
                        send_char_interrupt('0' + (line_count / 10) % 10);
                        send_char_interrupt('0' + line_count % 10);
                        send_string_interrupt(" sentences, CPU was FREE!]\r\n");
                    }

                    send_string_interrupt("Type sentence> ");
                    line_index = 0;
                }
            }
            // Backspace
            else if (received == '\b' || received == 127)
            {
                if (line_index > 0)
                {
                    line_index--;
                    send_string_interrupt(" \b");
                }
            }
            // Add to buffer
            else if (line_index < 63 && received >= ' ')
            {
                line_buffer[line_index++] = received;
            }
            // Buffer full
            else if (line_index >= 63)
            {
                send_string_interrupt("\r\n[BUFFER FULL - Press Enter]\r\n");
            }
        }

        // Small delay (CPU still free!)
        _delay_ms(5);
    }

    send_string_interrupt("\r\n[DEMO 6 COMPLETE]\r\n");
    send_string_interrupt("Sentences echoed: ");
    send_char_interrupt('0' + (line_count / 10) % 10);
    send_char_interrupt('0' + line_count % 10);
    send_string_interrupt("\r\nFull duplex ISR: Maximum efficiency!\r\n");
    send_string_interrupt("Compare with Demo 3 (polling sentence)!\r\n\r\n");
}

/*
 * =============================================================================
 * MAIN PROGRAM ENTRY POINT
 * =============================================================================
 */

int main(void)
{
    // Initialize basic system (each demo will initialize its own UART)
    simple_init_serial();

    // Wait a moment for system stability
    _delay_ms(1000);

    // Initialize UART for initial message before demo selection
    init_uart_polling();

    puts_USART1_P(PSTR("\r\n=== SERIAL COMMUNICATION - 2x3 MATRIX ===\r\n"));
    puts_USART1_P(PSTR("Edit main() to uncomment ONE demo.\r\n\r\n"));

    _delay_ms(1000);

    // ====================================================================
    // 2×3 EDUCATIONAL MATRIX: Select ONE demo to run
    // ====================================================================
    //
    // ┌──────────────────────────────────────────────────────────────┐
    // │ POLLING METHOD (Simple, CPU blocks)                         │
    // ├──────────────────────────────────────────────────────────────┤
    // │ Demo 1: Character Echo    → demo_polling_char_echo()        │
    // │ Demo 2: Word Echo          → demo_polling_word_echo()        │
    // │ Demo 3: Sentence Echo      → demo_polling_sentence_echo()    │
    // └──────────────────────────────────────────────────────────────┘
    //
    // ┌──────────────────────────────────────────────────────────────┐
    // │ INTERRUPT METHOD (Complex, CPU free)                        │
    // ├──────────────────────────────────────────────────────────────┤
    // │ Demo 4: Character Echo ISR → demo_interrupt_char_echo()      │
    // │ Demo 5: Word Echo ISR      → demo_interrupt_word_echo()      │
    // │ Demo 6: Sentence Echo ISR  → demo_interrupt_sentence_echo()  │
    // └──────────────────────────────────────────────────────────────┘

    // ========== POLLING TRACK (Character → Word → Sentence) ==========
    // demo_polling_char_echo(); // Demo 1: Polling character echo ← ACTIVE
    // demo_polling_word_echo();      // Demo 2: Polling word echo
    // demo_polling_sentence_echo();  // Demo 3: Polling sentence echo

    // ======== INTERRUPT TRACK (Character → Word → Sentence) =========
    // demo_interrupt_char_echo(); // Demo 4: Interrupt character echo
    // demo_interrupt_word_echo(); // Demo 5: Interrupt word echo (NEW!)
    demo_interrupt_sentence_echo(); // Demo 6: Interrupt sentence echo

    puts_USART1_P(PSTR("\r\n=== SUMMARY ===\r\n"));
    puts_USART1_P(PSTR("Polling: Simple but blocks CPU\r\n"));
    puts_USART1_P(PSTR("Interrupt: Complex but CPU-efficient\r\n"));
    puts_USART1_P(PSTR("Learn 1-3 first, then 4-6. Compare pairs.\r\n"));

    // Keep LED blinking to show program is running
    while (1)
    {
        PORTB ^= 0x01; // Toggle LED to show CPU is free
        _delay_ms(500);
    }

    return 0;
}
