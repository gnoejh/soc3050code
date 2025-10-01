/*
 * educational_demo.c - Working Educational Framework Demonstration
 * Shows Assembly → C → Python progression concepts using existing stable libraries
 */

#include "config.h"

#ifdef EDUCATIONAL_DEMO

/*
 * Demo 1: Register Access Progression
 * Shows the evolution from assembly to C to high-level concepts
 */
void demo_register_progression(void)
{
    // Send educational header
    puts_USART1("\r\n=== Educational Progression Demo ===\r\n");
    puts_USART1("ATmega128 Assembly → C → Python Learning Framework\r\n\r\n");

    // Phase 1: Direct Register Manipulation (Assembly style)
    puts_USART1("Phase 1: Assembly-Style Register Access\r\n");
    puts_USART1("DDRB = 0xFF;  // Configure PORT B as output\r\n");
    puts_USART1("PORTB = 0x00; // Turn LEDs ON (active LOW)\r\n");

    DDRB = 0xFF;  // Set all PORTB pins as output
    PORTB = 0x00; // Turn all LEDs ON (active LOW)
    _delay_ms(1000);

    puts_USART1("LEDs ON for 1 second...\r\n");

    // Phase 2: C Abstraction with Functions
    puts_USART1("\r\nPhase 2: C Function Abstraction\r\n");
    puts_USART1("Port_init(); // Initialize ports using library\r\n");

    Port_init(); // Use library initialization
    _delay_ms(500);

    // Phase 3: Pattern Generation with Algorithms
    puts_USART1("\r\nPhase 3: Algorithmic Pattern Generation\r\n");
    puts_USART1("for(i=0; i<8; i++) { PORTB = patterns[i]; }\r\n");

    unsigned char patterns[] = {0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F, 0xFF};

    for (unsigned char i = 0; i < 8; i++)
    {
        PORTB = ~patterns[i]; // Invert because LEDs are active LOW
        _delay_ms(200);
    }

    puts_USART1("\r\nProgression complete!\r\n");
}

/*
 * Demo 2: Communication Protocol Evolution
 * Shows UART usage from basic to structured communication
 */
void demo_communication_evolution(void)
{
    puts_USART1("\r\n=== Communication Evolution Demo ===\r\n");

    // Basic character echo
    puts_USART1("Level 1: Basic Character Echo\r\n");
    puts_USART1("Type a character: ");

    // Wait for input
    while (!(UCSR1A & (1 << RXC1)))
        ;
    char received = UDR1;

    // Echo back
    while (!(UCSR1A & (1 << UDRE1)))
        ;
    UDR1 = received;

    puts_USART1("\r\nLevel 2: Structured Response\r\n");
    puts_USART1("RESPONSE:Character_");
    putch_USART1(received);
    puts_USART1("_received\r\n");

    puts_USART1("\r\nLevel 3: Python-Ready Protocol\r\n");
    puts_USART1("DATA:{'type':'char','value':'");
    putch_USART1(received);
    puts_USART1("','timestamp':1234567890}\r\n");
}

/*
 * Demo 3: Sensor Integration Progression
 * Shows analog input from raw values to meaningful data
 */
void demo_sensor_progression(void)
{
    puts_USART1("\r\n=== Sensor Integration Demo ===\r\n");

    // Read ADC channel 0
    unsigned int adc_value = Read_Adc_Data(0);

    // Level 1: Raw ADC value
    puts_USART1("Level 1: Raw ADC = ");
    USART1_putchdecu(adc_value);
    puts_USART1("\r\n");

    // Level 2: Voltage conversion
    float voltage = (adc_value * 5.0) / 1023.0;
    puts_USART1("Level 2: Voltage = ");
    // Simple voltage display (integer part)
    USART1_putchdecu((unsigned int)voltage);
    puts_USART1(".");
    USART1_putchdecu((unsigned int)((voltage - (unsigned int)voltage) * 100));
    puts_USART1("V\r\n");

    // Level 3: Sensor interpretation
    puts_USART1("Level 3: Sensor Status = ");
    if (voltage < 1.0)
    {
        puts_USART1("LOW");
    }
    else if (voltage < 3.0)
    {
        puts_USART1("MEDIUM");
    }
    else
    {
        puts_USART1("HIGH");
    }
    puts_USART1("\r\n");

    // Level 4: Python-ready JSON
    puts_USART1("Level 4: JSON = {\"adc\":");
    USART1_putchdecu(adc_value);
    puts_USART1(",\"voltage\":");
    USART1_putchdecu((unsigned int)(voltage * 100)); // Send as integer * 100
    puts_USART1(",\"status\":\"");
    if (voltage < 1.0)
    {
        puts_USART1("low");
    }
    else if (voltage < 3.0)
    {
        puts_USART1("medium");
    }
    else
    {
        puts_USART1("high");
    }
    puts_USART1("\"}\r\n");
}

/*
 * Interactive Educational Menu
 */
void educational_menu(void)
{
    puts_USART1("\r\n=== ATmega128 Educational Framework ===\r\n");
    puts_USART1("Assembly → C → Python Learning Progression\r\n\r\n");
    puts_USART1("Choose a demonstration:\r\n");
    puts_USART1("1 - Register Access Progression\r\n");
    puts_USART1("2 - Communication Evolution\r\n");
    puts_USART1("3 - Sensor Integration\r\n");
    puts_USART1("R - Repeat menu\r\n");
    puts_USART1("\r\nEnter choice: ");

    while (1)
    {
        if (UCSR1A & (1 << RXC1))
        {
            char choice = UDR1;

            // Echo choice
            while (!(UCSR1A & (1 << UDRE1)))
                ;
            UDR1 = choice;
            puts_USART1("\r\n");

            switch (choice)
            {
            case '1':
                demo_register_progression();
                break;

            case '2':
                demo_communication_evolution();
                break;

            case '3':
                demo_sensor_progression();
                break;

            case 'R':
            case 'r':
                educational_menu();
                return;

            default:
                puts_USART1("Invalid choice. Try again: ");
                continue;
            }

            puts_USART1("\r\nPress 'R' for menu or any key to continue...\r\n");

            // Wait for next input
            while (!(UCSR1A & (1 << RXC1)))
                ;
            char next = UDR1;

            if (next == 'R' || next == 'r')
            {
                educational_menu();
                return;
            }

            puts_USART1("Enter choice: ");
        }

        // LED activity indicator
        static unsigned char led_counter = 0;
        led_counter++;
        if (led_counter > 100)
        {
            led_counter = 0;
            PORTB ^= 0x01; // Toggle LED 0 to show system is alive
        }

        _delay_ms(10);
    }
}

/*
 * Main educational demonstration function
 */
void main_educational_demo(void)
{
    // Initialize system
    init_devices();

    // Brief startup sequence
    PORTB = 0xFF; // All LEDs OFF
    _delay_ms(500);

    // LED startup animation
    for (unsigned char i = 0; i < 8; i++)
    {
        PORTB = ~(1 << i); // Turn on LED i
        _delay_ms(100);
    }
    PORTB = 0xFF; // All LEDs OFF

    // Start interactive menu
    educational_menu();
}

#endif // EDUCATIONAL_DEMO