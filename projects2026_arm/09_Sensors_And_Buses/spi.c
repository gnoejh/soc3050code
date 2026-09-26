/*
 * spi.c - SPI1 master, polled   (SOC3050 lesson 09)
 *
 * SPI is the other bus, and the opposite trade to I2C: four wires instead of
 * two, no addresses, no acknowledge - and ten or more times faster.  A device
 * is chosen by pulling ITS chip-select line low; everything else is shared.
 *
 *   I2C: 2 wires, address in every transfer, ACK per byte, 400 kHz here
 *   SPI: SCK + MOSI (+ MISO) + one CS per device, no ACK,  6 MHz here
 *
 * Every SPI transfer is also a receive: a byte goes out on MOSI while one
 * comes in on MISO, clocked by the same edges.  The MAX7219 sends nothing
 * back, but the received frame must still be read out of DR, or RXNE stays
 * set and the receive FIFO fills.
 */
#include "stm32c031xx.h"
#include "gpio.h"
#include "spi.h"

#define SCK_PIN   5u       /* PA5, AF0 */
#define MOSI_PIN  7u       /* PA7, AF0 */
#define CS_PIN    0u       /* PB0, GPIO */
#define AF_SPI1   0u
#define SPIN_LIMIT 100000u

void spi_init(void)
{
    RCC->IOPENR  |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;
    RCC->APBENR2 |= RCC_APBENR2_SPI1EN;

    pin_af(GPIOA, SCK_PIN,  AF_SPI1);
    pin_af(GPIOA, MOSI_PIN, AF_SPI1);
    pin_mode(GPIOB, CS_PIN, MODE_OUTPUT);
    pin_high(GPIOB, CS_PIN);                         /* idle: nobody selected */

    /* 16-bit frames: DS = 1111.  The MAX7219 takes exactly 16 bits per
     * command, register in the high byte, data in the low. */
    SPI1->CR2 = (0xFu << SPI_CR2_DS_Pos);

    /* Master, mode 0 (CPOL = CPHA = 0), MSB first, 48 MHz / 8 = 6 MHz -
     * inside the MAX7219's 10 MHz.  SSM + SSI: the chip select is ours to
     * drive as a GPIO, not the peripheral's NSS pin. */
    SPI1->CR1 = SPI_CR1_MSTR | (2u << SPI_CR1_BR_Pos) | SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR1 |= SPI_CR1_SPE;
}

int spi_send16(uint16_t frame)
{
    /* DR is accessed as 16 bits.  The data-packing logic of this SPI looks
     * at the WIDTH of each access, so a 32-bit write is not the same thing. */
    volatile uint16_t *dr = (volatile uint16_t *)&SPI1->DR;
    uint32_t i;

    pin_low(GPIOB, CS_PIN);
    for (i = 0; i < SPIN_LIMIT && !(SPI1->SR & SPI_SR_TXE); i++) { }
    *dr = frame;
    for (i = 0; i < SPIN_LIMIT && !(SPI1->SR & SPI_SR_RXNE); i++) { }
    (void)*dr;                                       /* discard what came back */
    for (i = 0; i < SPIN_LIMIT && (SPI1->SR & SPI_SR_BSY); i++) { }
    pin_high(GPIOB, CS_PIN);                         /* the rising edge LATCHES it */
    return i < SPIN_LIMIT ? 0 : -1;
}
