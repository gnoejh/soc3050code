//
//#include "config.h"
//
//
///* Atmega ADC Registers
//=====================================================
//| ADC Registers and Bit Description                 |
//=====================================================
//| Register | Bit      | Description                 |
//|----------|----------|-----------------------------|
//| ADCSRA   | ADEN     | ADC Enable                 |
//|          | ADSC     | ADC Start Conversion       |
//|          | ADATE    | Auto Trigger Enable        |
//|          | ADIF     | ADC Interrupt Flag         |
//|          | ADIE     | ADC Interrupt Enable       |
//|          | ADPS2    | ADC Prescaler Select Bit 2 |
//|          | ADPS1    | ADC Prescaler Select Bit 1 |
//|          | ADPS0    | ADC Prescaler Select Bit 0 |
//|----------|----------|-----------------------------|
//| ADMUX    | REFS1    | Reference Selection Bit 1  |
//|          | REFS0    | Reference Selection Bit 0  |
//|          | ADLAR    | ADC Left Adjust Result     |
//|          | MUX3     | Analog Channel Selection 3 |
//|          | MUX2     | Analog Channel Selection 2 |
//|          | MUX1     | Analog Channel Selection 1 |
//|          | MUX0     | Analog Channel Selection 0 |
//=====================================================
//
//Example Configuration:
//1. Set ADEN in ADCSRA to enable the ADC.
//2. Set ADSC in ADCSRA to start conversion.
//3. Configure ADMUX to select the input channel and reference voltage.
//4. Optionally set ADIE in ADCSRA to enable interrupts.
//5. Use the ADIF bit to check conversion completion.
//6. Adjust the ADC prescaler using ADPS2, ADPS1, and ADPS0 for clock scaling.
//=====================================================
//*/
//
//#ifdef ADC_POLLING
//void main_adc_polling(void)
//{
	//// Initialize devices (user-defined function)
	//init_devices();
//
	//// Disable ADC initially for configuration
	//ADCSRA = 0x00;
//
	//// Configure ADC:
	//// - Select ADC input channel 0 (MUX0 = 0)
	//// - Set Voltage Reference to Internal 2.56V with external capacitor on AREF
	//ADMUX = (1 << REFS1) | (1 << REFS0);
//
	//// Enable ADC and set prescaler to Fosc/8 for conversion time
	//ADCSRA = (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0);
//
	//unsigned int adc = 0;
//
	//while (1)
	//{
		//adc = 0;
//
		//// Start ADC conversion
		//ADCSRA |= (1 << ADSC);
//
		//// Wait for the ADC conversion to complete
		//while (!(ADCSRA & (1 << ADIF)));
//
		//// Clear the ADIF flag manually (required in polling mode)
		//ADCSRA |= (1 << ADIF);
//
		//// Combine ADCL and ADCH to get 10-bit ADC result
		//adc = (ADCL + (ADCH * 256)) / 10;
//
		//// Add a delay for stability
		//_delay_ms(100);
//
		//// Clear LCD and screen buffer
		//lcd_clear();
		//ScreenBuffer_clear();
//
		//// Display custom text and ADC value on LCD
		//lcd_string(0, 0, " 12345 Hong Jeong");
		//lcd_string(2, 0, "ADC0 Potentiometer");
		//lcd_string(3, 0, "ADC0 Polling");
//
		//// Display graphical representation of ADC value (line gauge)
		//GLCD_Rectangle(50, 0, 60, adc);
//
		//// Display numeric ADC value at specified position
		//lcd_xy(4, 0);
		//GLCD_4DigitDecimal(adc);
	//}
//}
//#endif
//
//
//#ifdef ADC_INTERRUPT
//void main_adc_interrupt(void)
//{
	//// Initialize devices (user-defined function)
	//init_devices();
//
	//// Disable ADC initially for configuration
	//ADCSRA = 0x00;
//
	//// Configure ADC:
	//// - Select ADC input channel 0 (MUX0 = 0)
	//// - Use AREF as the voltage reference
	//ADMUX = (1 << REFS0);
//
	//// Enable ADC, start the first conversion, enable ADC interrupt,
	//// and set prescaler to Fosc/8 for conversion time
	//ADCSRA = (1 << ADSC) | (1 << ADIE) | (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0);
//
	//// Enable global interrupts
	//sei();
//
	//// Infinite loop to keep the program running
	//while (1);
//}
//
//// ADC Conversion Complete Interrupt Service Routine
//SIGNAL(ADC_vect)
//{
	//static unsigned int adc = 0;
//
	//// Combine ADCL and ADCH to get the 10-bit ADC result
	//adc = (ADCL + (ADCH * 256)) / 10;
//
	//// Add a delay for stability
	//_delay_ms(100);
//
	//// Clear LCD and screen buffer
	//lcd_clear();
	//ScreenBuffer_clear();
//
	//// Display custom text and ADC value on LCD
	//lcd_string(0, 0, " 12345 Hong Jeong");
	//lcd_string(2, 0, "ADC0 Potentiometer");
	//lcd_string(3, 0, "ADC0 Interrupt");
//
	//// Display graphical representation of ADC value (line gauge)
	//GLCD_Rectangle(50, 0, 60, adc);
//
	//// Display numeric ADC value at specified position
	//lcd_xy(4, 0);
	//GLCD_4DigitDecimal(adc);
//
	//// Restart ADC conversion by setting the ADSC bit
	//ADCSRA |= (1 << ADSC);
//}
//#endif
//
//
//#ifdef ADC_POLLING_UART_POLLING
///*
//=====================================================
//| 3. ADC Polling with UART Polling                  |
//=====================================================
//| UART Registers:                                   |
//|---------------------------------------------------|
//| UCSRnA: RXCn TXCn UDREn - - - - -                |
//| UCSRnB: RXCIEn TXCIEn UDRIEn RXENn TXENn - - -   |
//| UCSRnC: - - - - - UCSZn1 UCSZn0 -                |
//|---------------------------------------------------|
//| ADC Registers:                                    |
//|---------------------------------------------------|
//| ADMUX: REFS1 REFS0 - - MUX3 MUX2 MUX1 MUX0       |
//| ADCSRA: ADEN ADSC - ADIF ADIE ADPS2 ADPS1 ADPS0  |
//=====================================================
//*/
//
//#define F_CPU 16000000UL
//#define BAUD 9600
//
//// Function prototypes
//void usart_send(unsigned char ch);
//void main_adc_polling_uart_polling(void);
//
//void main_adc_polling_uart_polling(void)
//{
	//// UART1 Initialization: 9600 Baud Rate, 8 Data Bits, No Parity, 1 Stop Bit (8N1)
	//UBRR1H = (F_CPU / 16 / BAUD - 1) >> 8;  // Set baud rate high byte
	//UBRR1L = (F_CPU / 16 / BAUD - 1);       // Set baud rate low byte
	//UCSR1B = (1 << TXEN1);                  // Enable transmission
	//UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); // Set frame format to 8-bit data
//
	//// ADC Initialization
	//ADMUX = (1 << REFS1) | (1 << REFS0); // 2.56V Internal Voltage Reference, channel 0
	//ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC, Prescaler = 128
//
	//while (1)
	//{
		//// Start ADC conversion
		//ADCSRA |= (1 << ADSC);
//
		//// Wait for conversion to complete
		//while (!(ADCSRA & (1 << ADIF)));
//
		//// Clear ADC Interrupt Flag (required in polling mode)
		//ADCSRA |= (1 << ADIF);
//
		//// Read ADC value (10-bit resolution)
		//int a = ADCL;
		//a |= (ADCH << 8);
//
		//// Convert negative value to positive and send '-'
		//if (a < 0)
		//{
			//usart_send('-');
			//a = -a;
		//}
//
		//// Transmit each digit of the ADC value
		//usart_send((a / 1000) + '0');  // Thousands place
		//a %= 1000;
		//usart_send((a / 100) + '0');   // Hundreds place
		//a %= 100;
		//usart_send((a / 10) + '0');    // Tens place
		//a %= 10;
		//usart_send(a + '0');           // Units place
		//usart_send('\r');              // Carriage return
//
		//// Delay for stability
		//_delay_ms(100);
	//}
//}
//
//// USART send function: Transmit a single character
//void usart_send(unsigned char ch)
//{
	//// Wait until the data register is empty
	//while (!(UCSR1A & (1 << UDRE1)));
	//// Load data into the USART data register to send
	//UDR1 = ch;
//}
//#endif
//
//#ifdef ADC_INTERRUPT_UART_POLLING
///* 4. ADC Interrupt UART Polling */
///* UART and ADC Registers ('-' means default 0)
//UART Registers:
//UCSRnA: RXCn TXCn UDREn - - - - -
//UCSRnB: RXCIEn TXCIEn UDRIEn RXENn TXENn - - -
//UCSRnC: - - - - - UCSZn1 UCSZn0 -
//
//ADC Registers:
//ADMUX: REFS1 REFS0 - - MUX3 MUX2 MUX1 MUX0
//ADCSA: ADEN ADSC - ADIF ADIE ADPS2 ADPS1 ADPS0
//*/
//
//#define F_CPU 16000000UL
//#define BAUD 9600
//void main_adc_interrupt_uart_polling (void)
//{
	//cli();
	//// UART1: 9600 8N1
	//UBRR1H = (F_CPU/16/BAUD-1)>>8;		// UBRR
	//UBRR1L = F_CPU/16/BAUD-1;
	//UCSR1B = (1<<TXEN1);				// Transmission enable
	//UCSR1C = (1<<UCSZ11)|(1<<UCSZ10);	// 8-bit
//
	//// ADC Initialization
	//ADMUX = (1<<REFS1)|(1<<REFS0);  // 0xC0, 2.56V Vref, right-justified, channel 0
	//ADCSRA = (1<<ADSC)|(1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // 0x8F
//
	//sei();
//
	//while (1);
//}
//
//void usart_send (unsigned char ch)
//{
	//while (! (UCSR1A & (1<<UDRE1))); 	// polling transmission complete?
	//UDR1 = ch;							// transmit ch
//}
//
//SIGNAL(ADC_vect)					// ADC interrupt
//{
	//int a = ADCL;					// ADC register
	//a = a | (ADCH<<8);
//
	//if(a < 0)
	//{
		//usart_send('-');
		//a *= -1;
	//}
//
	//usart_send((a/1000)+'0');
	//a = a % 1000;
	//usart_send((a/100)+'0');
	//a = a % 100;
	//usart_send((a/10)+'0');
	//a = a % 10;
	//usart_send((a)+'0');
	//usart_send('\r');
//
	//ADCSRA |=(1<<ADSC);				// start conversion
//}
//#endif
//
//
//#ifdef ADC_INTERRUPT_UART_INTERRUPT
///*
//=====================================================
//| Register Table for ADC and UART Programming       |
//=====================================================
//| ADC Registers:                                    |
//|---------------------------------------------------|
//| Register | Bit      | Description                 |
//|----------|----------|-----------------------------|
//| ADMUX    | REFS1    | Reference Selection Bit 1   |
//|          | REFS0    | Reference Selection Bit 0   |
//|          | -        | Reserved                    |
//|          | -        | Reserved                    |
//|          | MUX3     | Analog Channel Selection 3  |
//|          | MUX2     | Analog Channel Selection 2  |
//|          | MUX1     | Analog Channel Selection 1  |
//|          | MUX0     | Analog Channel Selection 0  |
//|---------------------------------------------------|
//| ADCSRA   | ADEN     | ADC Enable                  |
//|          | ADSC     | ADC Start Conversion        |
//|          | -        | Reserved                    |
//|          | ADIF     | ADC Interrupt Flag          |
//|          | ADIE     | ADC Interrupt Enable        |
//|          | ADPS2    | ADC Prescaler Select Bit 2  |
//|          | ADPS1    | ADC Prescaler Select Bit 1  |
//|          | ADPS0    | ADC Prescaler Select Bit 0  |
//|---------------------------------------------------|
//
//| UART Registers:                                   |
//|---------------------------------------------------|
//| Register | Bit      | Description                 |
//|----------|----------|-----------------------------|
//| UCSRnA   | RXCn     | USART Receive Complete      |
//|          | TXCn     | USART Transmit Complete     |
//|          | UDREn    | Data Register Empty         |
//|          | -        | Reserved                    |
//|          | -        | Reserved                    |
//|          | -        | Reserved                    |
//|          | -        | Reserved                    |
//|          | -        | Reserved                    |
//|---------------------------------------------------|
//| UCSRnB   | RXCIEn   | RX Complete Interrupt Enable|
//|          | TXCIEn   | TX Complete Interrupt Enable|
//|          | UDRIEn   | Data Register Empty Int En  |
//|          | RXENn    | Receiver Enable             |
//|          | TXENn    | Transmitter Enable          |
//|          | -        | Reserved                    |
//|          | -        | Reserved                    |
//|---------------------------------------------------|
//| UCSRnC   | UCSZn1   | Character Size Bit 1        |
//|          | UCSZn0   | Character Size Bit 0        |
//|---------------------------------------------------|
//| UBRRnH   | [7:0]    | High Byte of Baud Rate      |
//|---------------------------------------------------|
//| UBRRnL   | [7:0]    | Low Byte of Baud Rate       |
//=====================================================
//
//Note:
//1. Set `ADEN` in `ADCSRA` to enable the ADC.
//2. Use `ADMUX` to select the reference voltage and input channel.
//3. Set `RXENn` and `TXENn` in `UCSRnB` to enable UART receiver and transmitter.
//4. Enable `ADIE` in `ADCSRA` and `RXCIEn` in `UCSRnB` to enable ADC and UART interrupts.
//*/
//
//// Global variables
//unsigned char ch = 'x'; // UART character buffer
//int a;                  // ADC result storage
//
//void main_adc_interrupt_uart_interrupt(void)
//{
	//init_devices(); // User-defined initialization function
//
	//// UART1 Initialization: 9600 Baud Rate, 8 Data Bits, No Parity, 1 Stop Bit (8N1)
	//UBRR1H = (F_CPU / 16 / BAUD - 1) >> 8;  // Set baud rate high byte
	//UBRR1L = (F_CPU / 16 / BAUD - 1);       // Set baud rate low byte
	//UCSR1B = (1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1); // Enable TX, RX, and RX interrupt
	//UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); // Set frame format to 8-bit data
//
	//// ADC Initialization
	//ADMUX = (1 << REFS1) | (1 << REFS0); // Use 2.56V Internal Voltage Reference, channel 0
	//ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC with interrupt, Prescaler = 128
//
	//sei(); // Enable global interrupts
//
	//ADCSRA |= (1 << ADSC); // Start first ADC conversion
//
	//DDRB = 0xFF;  // Set PORTB as output
	//PORTB = 0xFF; // Initialize PORTB to HIGH
//
	//while (1); // Infinite loop
//}
//
//// UART Transmit Function (Polling)
//void usart_send(unsigned char ch)
//{
	//while (!(UCSR1A & (1 << UDRE1))); // Wait until UART data register is empty
	//UDR1 = ch;                        // Transmit the character
	//PORTB ^= (1 << 7);                // Toggle PORTB7 for debugging
//}
//
//// ADC Conversion Complete Interrupt
//ISR(ADC_vect)
//{
	//// Read ADC result (10-bit resolution)
	//a = ADCL;
	//a |= (ADCH << 8);
//
	//lcd_clear(); // Clear LCD
	//lcd_string(0, 0, " 12345 Hong Jeong");
	//lcd_string(2, 0, "ADC0 Potentiometer");
	//ScreenBuffer_clear();
	//GLCD_Rectangle(50, 0, 60, a / 10); // Display ADC value as a graphical bar
	//lcd_xy(4, 0);
	//GLCD_4DigitDecimal(a); // Display ADC value numerically
//
	//// Transmit ADC result over UART
	//ch = (a / 1000) + '0'; usart_send(ch);
	//a %= 1000;
	//ch = (a / 100) + '0'; usart_send(ch);
	//a %= 100;
	//ch = (a / 10) + '0'; usart_send(ch);
	//a %= 10;
	//ch = a + '0'; usart_send(ch);
	//usart_send('\r'); // Carriage return
//
	//// Restart ADC conversion
	//ADCSRA |= (1 << ADSC);
//}
//
//// UART RX Complete Interrupt
//ISR(USART1_RX_vect)
//{
	//unsigned char control = UDR1; // Read received character
	//if (control == 's')
	//{
		//PORTB ^= (1 << 0);        // Toggle PORTB0
		//ADCSRA |= (1 << ADSC);    // Start ADC conversion
	//}
//}
//
//#endif
//

#include "config.h"

/*
#####################################################
#            ATmega ADC Registers Overview          #
#####################################################

The following registers are used for configuring the ADC (Analog-to-Digital Converter) and UART functionalities on an ATmega microcontroller.

#####################################################
#                 ADC Registers                     #
#####################################################

+-----------+----------+----------------------------------+
| Register  | Bit      | Description                      |
+===========+==========+==================================+
| **ADCSRA**| **ADEN** | ADC Enable                       |
|           +----------+----------------------------------+
|           | **ADSC** | ADC Start Conversion             |
|           +----------+----------------------------------+
|           | **ADATE**| Auto Trigger Enable              |
|           +----------+----------------------------------+
|           | **ADIF** | ADC Interrupt Flag               |
|           +----------+----------------------------------+
|           | **ADIE** | ADC Interrupt Enable             |
|           +----------+----------------------------------+
|           | **ADPS2**| ADC Prescaler Select Bit 2       |
|           +----------+----------------------------------+
|           | **ADPS1**| ADC Prescaler Select Bit 1       |
|           +----------+----------------------------------+
|           | **ADPS0**| ADC Prescaler Select Bit 0       |
+-----------+----------+----------------------------------+
| **ADMUX** | **REFS1**| Reference Selection Bit 1        |
|           +----------+----------------------------------+
|           | **REFS0**| Reference Selection Bit 0        |
|           +----------+----------------------------------+
|           | **ADLAR**| ADC Left Adjust Result           |
|           +----------+----------------------------------+
|           | **MUX3** | Analog Channel Selection Bit 3   |
|           +----------+----------------------------------+
|           | **MUX2** | Analog Channel Selection Bit 2   |
|           +----------+----------------------------------+
|           | **MUX1** | Analog Channel Selection Bit 1   |
|           +----------+----------------------------------+
|           | **MUX0** | Analog Channel Selection Bit 0   |
+-----------+----------+----------------------------------+

### ADC Configuration Steps:

1. **Enable the ADC** by setting the `ADEN` bit in the `ADCSRA` register.
2. **Start a conversion** by setting the `ADSC` bit in the `ADCSRA` register.
3. **Configure the input channel and reference voltage** using the `ADMUX` register.
4. **Enable ADC interrupts** by setting the `ADIE` bit in the `ADCSRA` register (optional).
5. **Monitor the `ADIF` bit** to check if the conversion is complete.
6. **Set the ADC prescaler** by configuring `ADPS2`, `ADPS1`, and `ADPS0` bits in the `ADCSRA` register to adjust the ADC clock frequency.

#####################################################
#                 UART Registers                     #
#####################################################

+-----------+----------+----------------------------------+
| Register  | Bit      | Description                      |
+===========+==========+==================================+
| **UCSRnA**| **RXCn** | USART Receive Complete           |
|           +----------+----------------------------------+
|           | **TXCn** | USART Transmit Complete          |
|           +----------+----------------------------------+
|           | **UDREn**| Data Register Empty              |
+-----------+----------+----------------------------------+
| **UCSRnB**| **RXCIEn**| RX Complete Interrupt Enable    |
|           +----------+----------------------------------+
|           | **TXCIEn**| TX Complete Interrupt Enable    |
|           +----------+----------------------------------+
|           | **UDRIEn**| Data Register Empty Int Enable  |
|           +----------+----------------------------------+
|           | **RXENn**| Receiver Enable                  |
|           +----------+----------------------------------+
|           | **TXENn**| Transmitter Enable               |
+-----------+----------+----------------------------------+
| **UCSRnC**| **UCSZn1**| Character Size Bit 1            |
|           +----------+----------------------------------+
|           | **UCSZn0**| Character Size Bit 0            |
+-----------+----------+----------------------------------+
| **UBRRnH**| [7:0]    | High Byte of Baud Rate           |
+-----------+----------+----------------------------------+
| **UBRRnL**| [7:0]    | Low Byte of Baud Rate            |
+-----------+----------+----------------------------------+

### UART Configuration Steps:

1. **Set the baud rate** by configuring `UBRRnH` and `UBRRnL` registers.
2. **Enable UART transmitter and receiver** by setting `TXENn` and `RXENn` bits in the `UCSRnB` register.
3. **Configure frame format** (e.g., 8 data bits, no parity, 1 stop bit) by setting `UCSZn1` and `UCSZn0` bits in the `UCSRnC` register.
4. **Enable UART interrupts** by setting `RXCIEn` and `TXCIEn` bits in the `UCSRnB` register (optional).

#####################################################
#             Example Implementations                #
#####################################################
*/

/*-----------------------------------*
 * 1. ADC Polling Implementation     *
 *-----------------------------------*/
#ifdef ADC_POLLING
void main_adc_polling(void)
{
    // Initialize devices (user-defined function)
    init_devices();

    // Disable ADC initially for configuration
    ADCSRA = 0x00;

    // Configure ADC:
    // - Use Internal 2.56V Reference with external capacitor at AREF pin
    // - Select ADC input channel 0 (MUX[3:0] = 0b0000)
    ADMUX = (1 << REFS1) | (1 << REFS0);

    // Enable ADC and set prescaler to division factor of 8
    ADCSRA = (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0);

    unsigned int adc_value = 0;

    while (1)
    {
        // Start ADC conversion
        ADCSRA |= (1 << ADSC);

        // Wait for the ADC conversion to complete
        while (!(ADCSRA & (1 << ADIF)));

        // Clear the ADIF flag by writing a 1 (required in polling mode)
        ADCSRA |= (1 << ADIF);

        // Read the ADC result (10-bit value)
        adc_value = ADC; // ADC = ADCL + (ADCH << 8)

        // Process the ADC value (e.g., scaling, averaging)
        adc_value = adc_value / 10;

        // Delay for stability
        _delay_ms(100);

        // Clear LCD and screen buffer
        lcd_clear();
        ScreenBuffer_clear();

        // Display custom text and ADC value on LCD
        lcd_string(0, 0, " 12345 Hong Jeong");
        lcd_string(2, 0, "ADC0 Potentiometer");
        lcd_string(3, 0, "ADC0 Polling");

        // Display graphical representation of ADC value (line gauge)
        GLCD_Rectangle(50, 0, 60, adc_value);

        // Display numeric ADC value at specified position
        lcd_xy(4, 0);
        GLCD_4DigitDecimal(adc_value);
    }
}
#endif

/*-----------------------------------*
 * 2. ADC Interrupt Implementation   *
 *-----------------------------------*/
#ifdef ADC_INTERRUPT
void main_adc_interrupt(void)
{
    // Initialize devices (user-defined function)
    init_devices();

    // Disable ADC initially for configuration
    ADCSRA = 0x00;

    // Configure ADC:
    // - Use AREF as the voltage reference
    // - Select ADC input channel 0 (MUX[3:0] = 0b0000)
    ADMUX = (1 << REFS0);

    // Enable ADC, start the first conversion, enable ADC interrupt,
    // and set prescaler to division factor of 8
    ADCSRA = (1 << ADSC) | (1 << ADIE) | (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0);

    // Enable global interrupts
    sei();

    // Infinite loop to keep the program running
    while (1);
}

// ADC Conversion Complete Interrupt Service Routine
ISR(ADC_vect)
{
    static unsigned int adc_value = 0;

    // Read the ADC result (10-bit value)
    adc_value = ADC; // ADC = ADCL + (ADCH << 8)
    adc_value = adc_value / 10;

    // Delay for stability
    _delay_ms(100);

    // Clear LCD and screen buffer
    lcd_clear();
    ScreenBuffer_clear();

    // Display custom text and ADC value on LCD
    lcd_string(0, 0, " 12345 Hong Jeong");
    lcd_string(2, 0, "ADC0 Potentiometer");
    lcd_string(3, 0, "ADC0 Interrupt");

    // Display graphical representation of ADC value (line gauge)
    GLCD_Rectangle(50, 0, 60, adc_value);

    // Display numeric ADC value at specified position
    lcd_xy(4, 0);
    GLCD_4DigitDecimal(adc_value);

    // Restart ADC conversion by setting the ADSC bit
    ADCSRA |= (1 << ADSC);
}
#endif

/*-----------------------------------------------*
 * 3. ADC Polling with UART Polling Implementation *
 *-----------------------------------------------*/
#ifdef ADC_POLLING_UART_POLLING
#define F_CPU 16000000UL
#define BAUD 9600

// Function prototypes
void usart_send(unsigned char ch);

void main_adc_polling_uart_polling(void)
{
    // UART1 Initialization: 9600 Baud Rate, 8 Data Bits, No Parity, 1 Stop Bit (8N1)
    UBRR1H = (F_CPU / 16 / BAUD - 1) >> 8;  // Set baud rate high byte
    UBRR1L = (F_CPU / 16 / BAUD - 1);       // Set baud rate low byte
    UCSR1B = (1 << TXEN1);                  // Enable transmitter
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); // Set frame format to 8-bit data

    // ADC Initialization
    ADMUX = (1 << REFS1) | (1 << REFS0); // 2.56V Internal Voltage Reference, channel 0
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC, Prescaler = 128

    while (1)
    {
        // Start ADC conversion
        ADCSRA |= (1 << ADSC);

        // Wait for conversion to complete
        while (!(ADCSRA & (1 << ADIF)));

        // Clear ADC Interrupt Flag (required in polling mode)
        ADCSRA |= (1 << ADIF);

        // Read ADC value (10-bit resolution)
        int adc_value = ADC; // ADC = ADCL + (ADCH << 8)

        // Transmit ADC value over UART
        usart_send((adc_value / 1000) + '0');  // Thousands place
        adc_value %= 1000;
        usart_send((adc_value / 100) + '0');   // Hundreds place
        adc_value %= 100;
        usart_send((adc_value / 10) + '0');    // Tens place
        adc_value %= 10;
        usart_send(adc_value + '0');           // Units place
        usart_send('\r');                      // Carriage return

        // Delay for stability
        _delay_ms(100);
    }
}

// USART send function: Transmit a single character
void usart_send(unsigned char ch)
{
    // Wait until the data register is empty
    while (!(UCSR1A & (1 << UDRE1)));
    // Load data into the USART data register to send
    UDR1 = ch;
}
#endif

/*--------------------------------------------------*
 * 4. ADC Interrupt with UART Polling Implementation *
 *--------------------------------------------------*/
#ifdef ADC_INTERRUPT_UART_POLLING
#define F_CPU 16000000UL
#define BAUD 9600

void usart_send(unsigned char ch);

void main_adc_interrupt_uart_polling(void)
{
    cli(); // Disable global interrupts

    // UART1 Initialization: 9600 Baud Rate, 8 Data Bits, No Parity, 1 Stop Bit (8N1)
    UBRR1H = (F_CPU / 16 / BAUD - 1) >> 8;  // Set baud rate high byte
    UBRR1L = (F_CPU / 16 / BAUD - 1);       // Set baud rate low byte
    UCSR1B = (1 << TXEN1);                  // Enable transmitter
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); // Set frame format to 8-bit data

    // ADC Initialization
    ADMUX = (1 << REFS1) | (1 << REFS0); // 2.56V Internal Voltage Reference, channel 0
    ADCSRA = (1 << ADSC) | (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    sei(); // Enable global interrupts

    // Infinite loop
    while (1);
}

// USART send function: Transmit a single character
void usart_send(unsigned char ch)
{
    while (!(UCSR1A & (1 << UDRE1))); // Wait until data register is empty
    UDR1 = ch;                        // Transmit character
}

// ADC Conversion Complete Interrupt Service Routine
ISR(ADC_vect)
{
    int adc_value = ADC; // ADC = ADCL + (ADCH << 8)

    // Transmit ADC value over UART
    usart_send((adc_value / 1000) + '0');
    adc_value %= 1000;
    usart_send((adc_value / 100) + '0');
    adc_value %= 100;
    usart_send((adc_value / 10) + '0');
    adc_value %= 10;
    usart_send(adc_value + '0');
    usart_send('\r');

    // Start next ADC conversion
    ADCSRA |= (1 << ADSC);
}
#endif

/*------------------------------------------------------*
 * 5. ADC Interrupt with UART Interrupt Implementation  *
 *------------------------------------------------------*/
#ifdef ADC_INTERRUPT_UART_INTERRUPT
// Global variables
volatile unsigned char uart_char = 'x'; // UART character buffer
volatile int adc_value;                 // ADC result storage

void usart_send(unsigned char ch);

void main_adc_interrupt_uart_interrupt(void)
{
    init_devices(); // User-defined initialization function

    // UART1 Initialization: 9600 Baud Rate, 8 Data Bits, No Parity, 1 Stop Bit (8N1)
    UBRR1H = (F_CPU / 16 / BAUD - 1) >> 8;  // Set baud rate high byte
    UBRR1L = (F_CPU / 16 / BAUD - 1);       // Set baud rate low byte
    UCSR1B = (1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1); // Enable TX, RX, and RX Complete Interrupt
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); // Set frame format to 8-bit data

    // ADC Initialization
    ADMUX = (1 << REFS1) | (1 << REFS0); // Use 2.56V Internal Voltage Reference, channel 0
    ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC with interrupt, Prescaler = 128

    sei(); // Enable global interrupts

    ADCSRA |= (1 << ADSC); // Start first ADC conversion

    DDRB = 0xFF;  // Set PORTB as output (for debugging LEDs)
    PORTB = 0xFF; // Initialize PORTB to HIGH

    while (1)
    {
        // Main loop can perform other tasks
    }
}

// USART send function: Transmit a single character
void usart_send(unsigned char ch)
{
    while (!(UCSR1A & (1 << UDRE1))); // Wait until UART data register is empty
    UDR1 = ch;                        // Transmit the character
    PORTB ^= (1 << 7);                // Toggle PORTB7 (for debugging)
}

// ADC Conversion Complete Interrupt Service Routine
ISR(ADC_vect)
{
    // Read ADC result (10-bit value)
    adc_value = ADC; // ADC = ADCL + (ADCH << 8)

    // Update LCD display
    lcd_clear(); // Clear LCD
    lcd_string(0, 0, " 12345 Hong Jeong");
    lcd_string(2, 0, "ADC0 Potentiometer");
    ScreenBuffer_clear();
    GLCD_Rectangle(50, 0, 60, adc_value / 10); // Display ADC value as a graphical bar
    lcd_xy(4, 0);
    GLCD_4DigitDecimal(adc_value); // Display ADC value numerically

    // Transmit ADC result over UART
    usart_send((adc_value / 1000) + '0');
    adc_value %= 1000;
    usart_send((adc_value / 100) + '0');
    adc_value %= 100;
    usart_send((adc_value / 10) + '0');
    adc_value %= 10;
    usart_send(adc_value + '0');
    usart_send('\r'); // Carriage return

    // Restart ADC conversion
    ADCSRA |= (1 << ADSC);
}

// UART Receive Complete Interrupt Service Routine
ISR(USART1_RX_vect)
{
    unsigned char received_char = UDR1; // Read received character
    if (received_char == 's')
    {
        PORTB ^= (1 << 0);        // Toggle PORTB0 (for debugging)
        ADCSRA |= (1 << ADSC);    // Start ADC conversion
    }
}
#endif

/*------------------------------------------------------*
 * 6. ADC Polling with UART Interrupt Implementation    *
 *------------------------------------------------------*/
#ifdef ADC_POLLING_UART_INTERRUPT
#define F_CPU 16000000UL
#define BAUD 9600

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Global variables
volatile unsigned char uart_buffer[5]; // Buffer to store ADC value as string
volatile uint8_t uart_index = 0;       // Index for UART transmission

void main_adc_polling_uart_interrupt(void)
{
    cli(); // Disable global interrupts during initialization

    // **UART Initialization**: 9600 Baud Rate, 8 Data Bits, No Parity, 1 Stop Bit (8N1)
    UBRR1H = (F_CPU / 16 / BAUD - 1) >> 8;  // Set baud rate high byte
    UBRR1L = (F_CPU / 16 / BAUD - 1);       // Set baud rate low byte
    UCSR1B = (1 << TXEN1);                  // Corrected: Enable transmitter only
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); // Set frame format: 8 data bits, no parity, 1 stop bit

    // **ADC Initialization**
    // - Use 2.56V Internal Voltage Reference (REFS1 and REFS0 set)
    // - Select ADC input channel 0 (MUX[3:0] = 0b0000)
    // - Enable ADC
    // - Set prescaler to division factor of 128 (ADPS[2:0] = 0b111)
    ADMUX = (1 << REFS1) | (1 << REFS0); // ADMUX = 0xC0
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // ADCSRA = 0x87

    sei(); // Enable global interrupts

    while (1)
    {
        // **Start ADC Conversion**
        ADCSRA |= (1 << ADSC); // Start ADC conversion

        // **Wait for ADC Conversion to Complete**
        while (ADCSRA & (1 << ADSC)); // Wait until ADSC bit is cleared

        // **Read ADC Value**
        uint16_t adc_value = ADC; // Read ADC value (ADCL and ADCH)

        // **Convert ADC Value to String**
        uart_buffer[0] = (adc_value / 1000) + '0';       // Thousands place
        adc_value %= 1000;
        uart_buffer[1] = (adc_value / 100) + '0';        // Hundreds place
        adc_value %= 100;
        uart_buffer[2] = (adc_value / 10) + '0';         // Tens place
        adc_value %= 10;
        uart_buffer[3] = adc_value + '0';                // Units place
        uart_buffer[4] = '\r';                           // Carriage return

        // **Reset UART Index and Enable Data Register Empty Interrupt**
        uart_index = 0;
        UCSR1B |= (1 << UDRIE1); // Enable UART Data Register Empty Interrupt

        // **Delay Before Next ADC Conversion**
        _delay_ms(100); // Adjust as needed
    }
}

// **UART Data Register Empty Interrupt Service Routine**
ISR(USART1_UDRE_vect)
{
    if (uart_index < 5)
    {
        UDR1 = uart_buffer[uart_index++]; // Send next character
    }
    else
    {
        UCSR1B &= ~(1 << UDRIE1); // Disable Data Register Empty Interrupt
    }
}
#endif

