/*
 * ==============================================================================
 * HELLO, ATmega128 - the first program of the course
 * ==============================================================================
 * PROJECT: 00_Introduction
 * See Slide.md for the lecture: embedded processors, the development
 * environment, and the ATmega128 architecture.
 *
 * This program exists to prove three things on the first day:
 *
 *   1. The toolchain works.  If a banner appears on the serial monitor, then
 *      avr-gcc compiled, the linker linked, avr-objcopy produced a valid HEX
 *      image, and SimulIDE loaded it into the MCU model.
 *   2. The board is what the notes say it is.  The LEDs are on PORT B and are
 *      active low; the serial port is USART1; PD0 is a button pulled up to the
 *      rail.  You can check all three from the chair you are sitting in.
 *   3. Registers are the whole interface.  Nothing here calls a driver.  Every
 *      line below writes a number into an address the datasheet names, which is
 *      what "bare metal" means in practice.
 *
 * Nothing from shared_libs is linked, on purpose: the first program should have
 * no layer in it you have not read.
 * ==============================================================================
 */

#include "config.h"

/* ---------------------------------------------------------------------------
 * USART1 - the board's serial port, PD2 (RXD1) and PD3 (TXD1)
 *
 * Three registers configure it and one carries the data:
 *   UBRR1H/L  divisor that sets the bit rate
 *   UCSR1B    enables the transmitter and the receiver
 *   UCSR1C    frame format - here 8 data bits, no parity, 1 stop bit
 *   UDR1      write a byte to send it, read it to take a received byte
 * ------------------------------------------------------------------------ */
static void uart_init(void)
{
    /* 16000000 / (16 * 9600) - 1 = 103.17 -> 103, an error of 0.2 %, well
     * inside the ~2 % a UART frame tolerates. */
    uint16_t ubrr = (uint16_t)((F_CPU / (16UL * BAUD)) - 1UL);

    UBRR1H = (uint8_t)(ubrr >> 8);
    UBRR1L = (uint8_t)ubrr;
    UCSR1B = (1 << RXEN1) | (1 << TXEN1);
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
}

static void uart_putc(char c)
{
    /* UDRE1 = "USART Data Register Empty".  Polling it is the simplest form of
     * flow control there is: do nothing at all until the hardware is ready. */
    while (!(UCSR1A & (1 << UDRE1)))
        ;
    UDR1 = (uint8_t)c;
}

static void uart_puts(const char *s)
{
    while (*s)
        uart_putc(*s++);
}

/* ---------------------------------------------------------------------------
 * A look at the chip from the inside
 *
 * Every peripheral register is just a data-space address.  Printing those
 * addresses turns the memory map on the slides into something you can check.
 * The I/O address is the data address minus 0x20, because the register file
 * occupies 0x0000-0x001F and the I/O space starts immediately after it.
 * ------------------------------------------------------------------------ */
static void report_address(const char *name, volatile uint8_t *reg)
{
    char line[64];
    uint16_t data_addr = (uint16_t)(uintptr_t)reg;

    if (data_addr < 0x60)
        sprintf(line, "  %-5s  data 0x%04X   I/O 0x%02X\r\n",
                name, data_addr, (unsigned)(data_addr - 0x20));
    else
        sprintf(line, "  %-5s  data 0x%04X   I/O  -  (needs LDS/STS)\r\n",
                name, data_addr);

    uart_puts(line);
}

static void report_architecture(void)
{
    /* Linker symbols, not variables: their *addresses* are the numbers.
     * __data_start is the first byte of SRAM the program uses, __heap_start
     * the first byte after every static object. */
    extern uint8_t __data_start;
    extern uint8_t __heap_start;

    uint16_t statics_lo = (uint16_t)(uintptr_t)&__data_start;
    uint16_t statics_hi = (uint16_t)(uintptr_t)&__heap_start;
    char line[64];

    uart_puts("\r\n");
    uart_puts("==============================================\r\n");
    uart_puts(" Hello, ATmega128 - SOC3050, lesson 00\r\n");
    uart_puts("==============================================\r\n");

    sprintf(line, " Clock          %lu Hz, %u ns per cycle\r\n",
            (unsigned long)F_CPU, (unsigned)(1000000000UL / F_CPU));
    uart_puts(line);
    sprintf(line, " Serial         USART1, %lu baud 8N1, UBRR1 = %u\r\n",
            (unsigned long)BAUD, (unsigned)((F_CPU / (16UL * BAUD)) - 1UL));
    uart_puts(line);
    uart_puts(" Flash          131072 bytes = 65536 words of 16 bits\r\n");
    uart_puts(" EEPROM         4096 bytes, in an address space of its own\r\n");
    sprintf(line, " SRAM           %u bytes at 0x%04X - 0x%04X\r\n",
            (unsigned)(RAMEND - statics_lo + 1), statics_lo, (unsigned)RAMEND);
    uart_puts(line);
    sprintf(line, " Statics        0x%04X - 0x%04X, %u bytes\r\n",
            statics_lo, (unsigned)(statics_hi - 1),
            (unsigned)(statics_hi - statics_lo));
    uart_puts(line);
    sprintf(line, " Stack          from 0x%04X, growing downwards\r\n",
            (unsigned)RAMEND);
    uart_puts(line);
    sprintf(line, " Free RAM       %u bytes between the two\r\n",
            (unsigned)(RAMEND - statics_hi + 1));
    uart_puts(line);

    uart_puts("\r\n Where some of the registers live:\r\n");
    report_address("SREG", &SREG);   /* status register, top of the I/O space */
    report_address("DDRB", &DDRB);   /* direction of the LED port             */
    report_address("PORTB", &PORTB); /* what we drive onto it                 */
    report_address("PINB", &PINB);   /* what is actually there                */
    report_address("UDR1", &UDR1);   /* USART1 sits in the extended I/O area  */

    uart_puts("\r\n Watch the LEDs on PORT B. Hold the PD0 button to pause.\r\n");
    uart_puts("----------------------------------------------\r\n");
}

/* ---------------------------------------------------------------------------
 * main - the shape almost every bare-metal program has
 *
 *     set the hardware up once, then loop forever
 *
 * There is no operating system to return to, so main() never ends.  If it did,
 * avr-libc would drop the CPU into an infinite loop anyway, because a processor
 * always has to be executing something.
 * ------------------------------------------------------------------------ */
int main(void)
{
    uint8_t position = 0;
    uint16_t laps = 0;
    char line[64];

    /* All 8 LED pins become outputs.  A 1 bit in a DDR means "output". */
    LED_DDR = 0xFF;
    LED_WRITE(0x00); /* bit set = lit, so 0x00 is all off */

    /* The button pin becomes an input with its internal pull-up enabled: the
     * DDR bit 0 for input, the PORT bit 1 for the pull-up.  Without the pull-up
     * the pin floats and reads noise. */
    BTN_DDR &= (uint8_t) ~(1 << BTN_PAUSE);
    BTN_PORT |= (uint8_t)(1 << BTN_PAUSE);

    uart_init();
    report_architecture();

    while (1)
    {
        /* One lit LED, walked along the port.  LED_WRITE does the inversion,
         * so this line reads the way it looks: bit set means that LED is on. */
        LED_WRITE(1 << position);

        if (BTN_PRESSED(BTN_PAUSE))
        {
            uart_puts("PD0 held - paused\r\n");
            while (BTN_PRESSED(BTN_PAUSE))
                ; /* the entire CPU waits here.  Lesson 02 explains why that
                   * is a problem, and lesson 05 gives the alternative. */
        }

        _delay_ms(120);

        position++;
        if (position > 7)
        {
            position = 0;
            laps++;
            sprintf(line, "lap %u - %lu ms spent in _delay_ms doing nothing\r\n",
                    laps, (unsigned long)laps * 8UL * 120UL);
            uart_puts(line);
        }
    }
}
