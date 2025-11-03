/*
 * =============================================================================
 * I2C MASTER COMMUNICATION - EDUCATIONAL DEMONSTRATION
 * =============================================================================
 *
 * PROJECT: I2C_Master_Basic
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Educational demonstration of I2C/TWI master communication protocols.
 * Students learn two-wire interface concepts and multi-device communication.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master I2C protocol fundamentals (Start, Stop, ACK, NACK)
 * 2. Learn TWI register configuration and timing
 * 3. Practice 7-bit device addressing schemes
 * 4. Implement read/write operations with I2C devices
 * 5. Handle bus arbitration and error conditions
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - I2C slave devices (EEPROM, RTC, sensors)
 * - SCL and SDA lines with 4.7K pull-up resistors
 * - Serial connection for debugging (9600 baud)
 * - Optional: I2C bus analyzer for protocol verification
 *
 * LEARNING PROGRESSION:
 * - Demo 1: I2C Master Configuration
 * - Demo 2: Device Detection and Addressing
 * - Demo 3: Single Byte Read/Write Operations
 * - Demo 4: Multi-Byte Data Transfers
 * - Demo 5: Advanced Protocol Handling
 *
 * =============================================================================
 */
 * - LEDs on PORTC for status
 *
 * I2C CONCEPTS:
 * - Multi-Master: Multiple masters can control the bus
 * - 7-bit Addressing: Supports 128 unique device addresses
 * - ACK/NACK: Receiver acknowledges each byte
 * - Start Condition: SDA falls while SCL is high
 * - Stop Condition: SDA rises while SCL is high
 * - Clock Stretching: Slave can hold SCL low to slow master
 * - Standard: 100 kHz, Fast: 400 kHz, High-speed: 3.4 MHz
 */

#include "config.h"

 // I2C/TWI Status Codes
#define TWI_START 0x08        // Start condition transmitted
#define TWI_REP_START 0x10    // Repeated start condition transmitted
#define TWI_MT_SLA_ACK 0x18   // SLA+W transmitted, ACK received
#define TWI_MT_SLA_NACK 0x20  // SLA+W transmitted, NACK received
#define TWI_MT_DATA_ACK 0x28  // Data transmitted, ACK received
#define TWI_MT_DATA_NACK 0x30 // Data transmitted, NACK received
#define TWI_MR_SLA_ACK 0x40   // SLA+R transmitted, ACK received
#define TWI_MR_SLA_NACK 0x48  // SLA+R transmitted, NACK received
#define TWI_MR_DATA_ACK 0x50  // Data received, ACK returned
#define TWI_MR_DATA_NACK 0x58 // Data received, NACK returned

 // I2C Operation result codes
#define I2C_SUCCESS 0
#define I2C_ERROR_NODEV 1
#define I2C_ERROR_TIMEOUT 2
#define I2C_ERROR_NACK 3

/*
 * Initialize I2C/TWI interface
 * SCL Frequency = F_CPU / (16 + 2 * TWBR * Prescaler)
 * For 100kHz @ 7.3728MHz: TWBR = 32, Prescaler = 1
 */
void i2c_init(void)
 {
     // Set bit rate register for 100kHz
     // SCL_freq = F_CPU / (16 + 2 * TWBR * Prescaler)
     // 100000 = 7372800 / (16 + 2 * TWBR * 1)
     // TWBR = 32
     TWBR = 32;

     // Prescaler = 1 (TWPS1:0 = 00)
     TWSR = 0x00;

     // Enable TWI
     TWCR = (1 << TWEN);
 }

 /*
  * Send START condition
  */
 uint8_t i2c_start(void)
 {
     // Send START condition
     TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

     // Wait for TWINT flag (operation complete)
     uint16_t timeout = 0;
     while (!(TWCR & (1 << TWINT)))
     {
         if (++timeout > 10000)
             return I2C_ERROR_TIMEOUT;
     }

     // Check status code
     uint8_t status = TWSR & 0xF8;
     if (status != TWI_START && status != TWI_REP_START)
     {
         return I2C_ERROR_NACK;
     }

     return I2C_SUCCESS;
 }

 /*
  * Send STOP condition
  */
 void i2c_stop(void)
 {
     TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);

     // Wait for STOP to complete
     _delay_us(100);
 }

 /*
  * Write one byte to I2C bus
  */
 uint8_t i2c_write(uint8_t data)
 {
     // Load data into register
     TWDR = data;

     // Start transmission
     TWCR = (1 << TWINT) | (1 << TWEN);

     // Wait for completion
     uint16_t timeout = 0;
     while (!(TWCR & (1 << TWINT)))
     {
         if (++timeout > 10000)
             return I2C_ERROR_TIMEOUT;
     }

     // Check status
     uint8_t status = TWSR & 0xF8;
     if (status != TWI_MT_SLA_ACK && status != TWI_MT_DATA_ACK)
     {
         return I2C_ERROR_NACK;
     }

     return I2C_SUCCESS;
 }

 /*
  * Read one byte from I2C bus
  * send_ack: 1 = send ACK, 0 = send NACK
  */
 uint8_t i2c_read(uint8_t *data, uint8_t send_ack)
 {
     // Start reception with ACK/NACK
     if (send_ack)
     {
         TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
     }
     else
     {
         TWCR = (1 << TWINT) | (1 << TWEN);
     }

     // Wait for completion
     uint16_t timeout = 0;
     while (!(TWCR & (1 << TWINT)))
     {
         if (++timeout > 10000)
             return I2C_ERROR_TIMEOUT;
     }

     // Read received data
     *data = TWDR;

     // Check status
     uint8_t status = TWSR & 0xF8;
     if (send_ack && status != TWI_MR_DATA_ACK)
         return I2C_ERROR_NACK;
     if (!send_ack && status != TWI_MR_DATA_NACK)
         return I2C_ERROR_NACK;

     return I2C_SUCCESS;
 }

 /*
  * Write byte to I2C device at specific register
  */
 uint8_t i2c_write_register(uint8_t device_addr, uint8_t reg_addr, uint8_t data)
 {
     uint8_t result;

     // Start condition
     result = i2c_start();
     if (result != I2C_SUCCESS)
     {
         i2c_stop();
         return result;
     }

     // Send device address + write bit
     result = i2c_write((device_addr << 1) | 0x00);
     if (result != I2C_SUCCESS)
     {
         i2c_stop();
         return result;
     }

     // Send register address
     result = i2c_write(reg_addr);
     if (result != I2C_SUCCESS)
     {
         i2c_stop();
         return result;
     }

     // Send data
     result = i2c_write(data);
     if (result != I2C_SUCCESS)
     {
         i2c_stop();
         return result;
     }

     // Stop condition
     i2c_stop();

     return I2C_SUCCESS;
 }

 /*
  * Read byte from I2C device at specific register
  */
 uint8_t i2c_read_register(uint8_t device_addr, uint8_t reg_addr, uint8_t *data)
 {
     uint8_t result;

     // Start condition
     result = i2c_start();
     if (result != I2C_SUCCESS)
     {
         i2c_stop();
         return result;
     }

     // Send device address + write bit
     result = i2c_write((device_addr << 1) | 0x00);
     if (result != I2C_SUCCESS)
     {
         i2c_stop();
         return result;
     }

     // Send register address
     result = i2c_write(reg_addr);
     if (result != I2C_SUCCESS)
     {
         i2c_stop();
         return result;
     }

     // Repeated start
     result = i2c_start();
     if (result != I2C_SUCCESS)
     {
         i2c_stop();
         return result;
     }

     // Send device address + read bit
     result = i2c_write((device_addr << 1) | 0x01);
     if (result != I2C_SUCCESS)
     {
         i2c_stop();
         return result;
     }

     // Read data (send NACK after last byte)
     result = i2c_read(data, 0);
     if (result != I2C_SUCCESS)
     {
         i2c_stop();
         return result;
     }

     // Stop condition
     i2c_stop();

     return I2C_SUCCESS;
 }

 /*
  * Scan I2C bus for devices
  */
 uint8_t i2c_scan(uint8_t *devices_found, uint8_t max_devices)
 {
     uint8_t count = 0;

     puts_USART1("Scanning I2C bus (addresses 0x08 - 0x77)...\r\n");

     for (uint8_t addr = 0x08; addr < 0x78; addr++)
     {
         // Try to access device
         if (i2c_start() == I2C_SUCCESS)
         {
             if (i2c_write((addr << 1) | 0x00) == I2C_SUCCESS)
             {
                 // Device responded
                 if (count < max_devices)
                 {
                     devices_found[count++] = addr;
                 }

                 char buf[50];
                 sprintf(buf, "  Found device at 0x%02X\r\n", addr);
                 puts_USART1(buf);
             }
             i2c_stop();
         }
     }

     return count;
 }

 /* ========================================================================
  * DEMO 1: I2C Bus Scanner
  * ======================================================================== */
 void demo1_bus_scanner(void)
 {
     puts_USART1("\r\n=== DEMO 1: I2C Bus Scanner ===\r\n");
     puts_USART1("Scanning for I2C devices...\r\n\r\n");

     uint8_t devices[16];
     uint8_t count = i2c_scan(devices, 16);

     char buf[60];
     sprintf(buf, "\r\nScan complete! Found %u device(s)\r\n", count);
     puts_USART1(buf);

     if (count > 0)
     {
         puts_USART1("\r\nDevice addresses:\r\n");
         for (uint8_t i = 0; i < count; i++)
         {
             sprintf(buf, "  Device %u: 0x%02X (7-bit)\r\n", i + 1, devices[i]);
             puts_USART1(buf);
         }
         PORTC = count; // Show count on LEDs
     }
     else
     {
         puts_USART1("\r\nNo devices found. Check connections and pull-ups!\r\n");
         PORTC = 0xFF; // All LEDs on = error
     }

     puts_USART1("\r\nPress any key to continue...");
     getch_USART1();
 }

 /* ========================================================================
  * DEMO 2: Register Read/Write Test
  * ======================================================================== */
 void demo2_register_test(void)
 {
     puts_USART1("\r\n=== DEMO 2: Register Read/Write Test ===\r\n");
     puts_USART1("Enter device address (hex, e.g., 50): ");

     // Read device address
     char addr_str[3];
     for (uint8_t i = 0; i < 2; i++)
     {
         addr_str[i] = getch_USART1();
         putch_USART1(addr_str[i]);
     }
     addr_str[2] = '\0';

     uint8_t device_addr = 0;
     for (uint8_t i = 0; i < 2; i++)
     {
         device_addr <<= 4;
         char c = addr_str[i];
         if (c >= '0' && c <= '9')
             device_addr |= (c - '0');
         else if (c >= 'A' && c <= 'F')
             device_addr |= (c - 'A' + 10);
         else if (c >= 'a' && c <= 'f')
             device_addr |= (c - 'a' + 10);
     }

     puts_USART1("\r\n\r\nEnter register address (hex, e.g., 00): ");

     // Read register address
     char reg_str[3];
     for (uint8_t i = 0; i < 2; i++)
     {
         reg_str[i] = getch_USART1();
         putch_USART1(reg_str[i]);
     }
     reg_str[2] = '\0';

     uint8_t reg_addr = 0;
     for (uint8_t i = 0; i < 2; i++)
     {
         reg_addr <<= 4;
         char c = reg_str[i];
         if (c >= '0' && c <= '9')
             reg_addr |= (c - '0');
         else if (c >= 'A' && c <= 'F')
             reg_addr |= (c - 'A' + 10);
         else if (c >= 'a' && c <= 'f')
             reg_addr |= (c - 'a' + 10);
     }

     puts_USART1("\r\n\r\nReading register...\r\n");

     uint8_t data;
     uint8_t result = i2c_read_register(device_addr, reg_addr, &data);

     char buf[80];
     if (result == I2C_SUCCESS)
     {
         sprintf(buf, "Register 0x%02X = 0x%02X (%u decimal)\r\n",
                 reg_addr, data, data);
         puts_USART1(buf);
         PORTC = data; // Show on LEDs
     }
     else
     {
         sprintf(buf, "Error reading register! Code: %u\r\n", result);
         puts_USART1(buf);
         PORTC = 0xFF;
     }

     puts_USART1("\r\nPress any key to continue...");
     getch_USART1();
 }

 /* ========================================================================
  * DEMO 3: Multi-Byte Sequential Read
  * ======================================================================== */
 void demo3_sequential_read(void)
 {
     puts_USART1("\r\n=== DEMO 3: Sequential Read Test ===\r\n");
     puts_USART1("Reading multiple bytes from I2C device\r\n");
     puts_USART1("Enter device address (hex): ");

     // Get device address (simplified for demo)
     char addr_input[3];
     for (uint8_t i = 0; i < 2; i++)
     {
         addr_input[i] = getch_USART1();
         putch_USART1(addr_input[i]);
     }

     uint8_t device_addr = 0x50; // Example: 24C256 EEPROM
     puts_USART1("\r\n\r\nReading 16 bytes starting from address 0x00...\r\n\r\n");

     uint8_t buffer[16];
     uint8_t errors = 0;

     for (uint8_t i = 0; i < 16; i++)
     {
         uint8_t result = i2c_read_register(device_addr, i, &buffer[i]);
         if (result != I2C_SUCCESS)
         {
             errors++;
         }
         _delay_ms(10);
     }

     // Display hex dump
     puts_USART1("Address: 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\r\n");
     puts_USART1("Data:    ");

     for (uint8_t i = 0; i < 16; i++)
     {
         char buf[10];
         sprintf(buf, "%02X ", buffer[i]);
         puts_USART1(buf);
     }

     puts_USART1("\r\n\r\nASCII: ");
     for (uint8_t i = 0; i < 16; i++)
     {
         if (buffer[i] >= 32 && buffer[i] <= 126)
         {
             putch_USART1(buffer[i]);
         }
         else
         {
             putch_USART1('.');
         }
     }

     char msg[50];
     sprintf(msg, "\r\n\r\nRead complete! Errors: %u\r\n", errors);
     puts_USART1(msg);

     puts_USART1("\r\nPress any key to continue...");
     getch_USART1();
 }

 /* ========================================================================
  * DEMO 4: I2C Clock Speed Test
  * ======================================================================== */
 void demo4_speed_test(void)
 {
     puts_USART1("\r\n=== DEMO 4: I2C Speed Test ===\r\n");
     puts_USART1("Testing different I2C clock speeds\r\n\r\n");

     // Test speeds (TWBR values)
     uint8_t twbr_values[] = {72, 32, 12, 2}; // ~50kHz, 100kHz, 200kHz, 400kHz
     char *speed_names[] = {"50 kHz", "100 kHz", "200 kHz", "400 kHz"};

     uint8_t device_addr = 0x50; // Test with EEPROM or similar

     for (uint8_t s = 0; s < 4; s++)
     {
         TWBR = twbr_values[s];

         char buf[60];
         sprintf(buf, "Testing %s (TWBR=%u)...\r\n", speed_names[s], twbr_values[s]);
         puts_USART1(buf);

         // Measure time for 100 reads
         uint16_t start_time = TCNT1;

         uint8_t data;
         uint8_t success_count = 0;
         for (uint8_t i = 0; i < 100; i++)
         {
             if (i2c_read_register(device_addr, 0x00, &data) == I2C_SUCCESS)
             {
                 success_count++;
             }
         }

         uint16_t elapsed = TCNT1 - start_time;

         sprintf(buf, "  Time: %u ticks, Success: %u/100\r\n\r\n",
                 elapsed, success_count);
         puts_USART1(buf);

         _delay_ms(500);
     }

     // Restore default speed
     TWBR = 32;

     puts_USART1("Speed test complete!\r\n");
     puts_USART1("\r\nPress any key to continue...");
     getch_USART1();
 }

 /* ========================================================================
  * Main Menu System
  * ======================================================================== */
 void display_main_menu(void)
 {
     puts_USART1("\r\n\r\n");
     puts_USART1("╔════════════════════════════════════════╗\r\n");
     puts_USART1("║   I2C/TWI MASTER BASIC - ATmega128    ║\r\n");
     puts_USART1("╚════════════════════════════════════════╝\r\n");
     puts_USART1("\r\n");
     puts_USART1("Select Demo:\r\n");
     puts_USART1("  [1] I2C Bus Scanner\r\n");
     puts_USART1("  [2] Register Read/Write Test\r\n");
     puts_USART1("  [3] Multi-Byte Sequential Read\r\n");
     puts_USART1("  [4] I2C Speed Test\r\n");
     puts_USART1("\r\n");
     puts_USART1("Enter selection (1-4): ");
 }

 int main(void)
 {
     // Initialize peripherals
     Uart1_init();
     i2c_init();

     // Configure Timer1 for timing measurements
     TCCR1B = (1 << CS10);

     // Configure status LEDs
     DDRC = 0xFF;
     PORTC = 0x00;

     // Send startup message
     _delay_ms(500);
     puts_USART1("\r\n\r\n*** I2C/TWI Master Communication ***\r\n");
     puts_USART1("ATmega128 I2C Learning System\r\n");
     puts_USART1("Default: 100 kHz, 7-bit addressing\r\n");
     puts_USART1("Pins: PD0=SCL, PD1=SDA (need 4.7K pull-ups!)\r\n");

     while (1)
     {
         display_main_menu();

         // Wait for user selection
         char choice = getch_USART1();
         putch_USART1(choice);
         puts_USART1("\r\n");

         switch (choice)
         {
         case '1':
             demo1_bus_scanner();
             break;
         case '2':
             demo2_register_test();
             break;
         case '3':
             demo3_sequential_read();
             break;
         case '4':
             demo4_speed_test();
             break;
         default:
             puts_USART1("Invalid selection!\r\n");
             _delay_ms(1000);
             break;
         }

         _delay_ms(500);
     }

     return 0;
 }
