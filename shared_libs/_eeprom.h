/*
 * =============================================================================
 * EDUCATIONAL ATmega128 EEPROM LIBRARY - HEADER FILE
 * =============================================================================
 *
 * COURSE: SOC 3050 - Embedded Systems and IoT
 * AUTHOR: Professor Kim
 *
 * PURPOSE:
 * Educational header for ATmega128 EEPROM (Electrically Erasable Programmable
 * Read-Only Memory) operations. This library provides comprehensive EEPROM
 * functionality with safety features and educational documentation.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Understand non-volatile memory concepts and applications
 * 2. Learn EEPROM programming techniques and limitations
 * 3. Implement data persistence and configuration storage
 * 4. Explore memory management and wear leveling strategies
 * 5. Practice data integrity and error detection methods
 *
 * HARDWARE FEATURES:
 * - 4KB (4096 bytes) EEPROM memory on ATmega128
 * - 100,000 write/erase cycles minimum endurance
 * - 3.3ms typical write time, 4 clock cycles read time
 * - Individual byte-level write capability
 * - Built-in hardware write protection during programming
 *
 * LEARNING PROGRESSION:
 * Assembly → C → Python → IoT
 * Direct registers → Structured functions → Object serialization → Remote storage
 *
 * =============================================================================
 */

#ifndef _EEPROM_H_
#define _EEPROM_H_

/*
 * =============================================================================
 * EEPROM HARDWARE CONSTANTS AND DEFINITIONS
 * =============================================================================
 */

/* ATmega128 EEPROM specifications */
#define EEPROM_SIZE 4096        // Total EEPROM size in bytes
#define EEPROM_MAX_ADDRESS 4095 // Maximum valid address (0-based)
#define EEPROM_WRITE_TIME_MS 4  // Approximate write time in milliseconds
#define EEPROM_ENDURANCE 100000 // Minimum write/erase cycles

/* EEPROM operation result codes */
#define EEPROM_SUCCESS 0     // Operation completed successfully
#define EEPROM_ERROR_ADDR 1  // Invalid address error
#define EEPROM_ERROR_BUSY 2  // EEPROM busy error
#define EEPROM_ERROR_TEST 3  // Self-test failed
#define EEPROM_ERROR_BLOCK 4 // Block operation error

/*
 * =============================================================================
 * BASIC EEPROM OPERATIONS
 * =============================================================================
 */

/*
 * FUNDAMENTAL FUNCTION: Write Single Byte to EEPROM
 *
 * PURPOSE: Store a single byte at specified EEPROM address
 * PARAMETERS:
 *   EE_Addr - EEPROM address (0 to EEPROM_MAX_ADDRESS)
 *   EE_Data - Data byte to write (0-255)
 *
 * EDUCATIONAL NOTES:
 * - Uses ATmega128 EEPROM hardware registers
 * - Implements proper write sequence timing
 * - Includes safety checks and error handling
 */
void In_EEPROM_Write(unsigned int EE_Addr, unsigned char EE_Data);

/*
 * FUNDAMENTAL FUNCTION: Read Single Byte from EEPROM
 *
 * PURPOSE: Retrieve a single byte from specified EEPROM address
 * PARAMETERS:
 *   EE_Addr - EEPROM address (0 to EEPROM_MAX_ADDRESS)
 * RETURNS: Data byte read from EEPROM (0-255)
 *
 * EDUCATIONAL NOTES:
 * - Fast operation (4 clock cycles)
 * - No wear on memory cells during read
 * - Includes address validation
 */
unsigned char In_EEPROM_Read(unsigned int EE_Addr);

/*
 * =============================================================================
 * ADVANCED EEPROM OPERATIONS
 * =============================================================================
 */

/*
 * SYSTEM FUNCTION: Initialize EEPROM System
 *
 * PURPOSE: Set up EEPROM for safe operation and verify functionality
 * EDUCATIONAL VALUE: System initialization and self-testing procedures
 */
void EEPROM_init(void);

/*
 * BLOCK OPERATIONS: Multi-Byte Data Handling
 *
 * PURPOSE: Store and retrieve arrays or structures efficiently
 * EDUCATIONAL VALUE: Bulk data operations and memory management
 */
void EEPROM_write_block(unsigned int start_addr, unsigned char *data, unsigned int length);
void EEPROM_read_block(unsigned int start_addr, unsigned char *buffer, unsigned int length);

/*
 * DATA TYPE OPERATIONS: Structured Data Storage
 *
 * PURPOSE: Handle common data types with proper byte ordering
 * EDUCATIONAL VALUE: Multi-byte data handling and endianness concepts
 */
void EEPROM_write_int(unsigned int address, unsigned int value);
unsigned int EEPROM_read_int(unsigned int address);
void EEPROM_write_string(unsigned int address, const char *str);
void EEPROM_read_string(unsigned int address, char *buffer, unsigned int max_length);

/*
 * MEMORY MANAGEMENT: Region Operations
 *
 * PURPOSE: Initialize and manage EEPROM memory regions
 * EDUCATIONAL VALUE: Memory organization and bulk operations
 */
void EEPROM_clear_region(unsigned int start_addr, unsigned int length, unsigned char fill_value);

/*
 * =============================================================================
 * DATA INTEGRITY AND DIAGNOSTICS
 * =============================================================================
 */

/*
 * DIAGNOSTIC FUNCTION: Get EEPROM Usage Statistics
 *
 * PURPOSE: Provide usage statistics for debugging and optimization
 * EDUCATIONAL VALUE: System monitoring and performance analysis
 */
void EEPROM_get_stats(unsigned int *writes, unsigned int *reads, unsigned char *last_error);

/*
 * VERIFICATION FUNCTION: Data Integrity Check
 *
 * PURPOSE: Compare EEPROM contents with expected data
 * EDUCATIONAL VALUE: Data integrity verification and error detection
 */
unsigned char EEPROM_verify_block(unsigned int address, unsigned char *expected_data, unsigned int length);

/*
 * DEBUGGING FUNCTION: Memory Content Display
 *
 * PURPOSE: Display EEPROM contents for debugging purposes
 * EDUCATIONAL VALUE: Memory inspection and debugging techniques
 */
void EEPROM_dump_region(unsigned int start_addr, unsigned int length);

/*
 * ERROR DETECTION: Checksum Calculation
 *
 * PURPOSE: Generate simple checksum for data integrity
 * EDUCATIONAL VALUE: Error detection and data validation methods
 */
unsigned char EEPROM_calculate_checksum(unsigned int address, unsigned int length);

/*
 * =============================================================================
 * PRACTICAL APPLICATION EXAMPLES
 * =============================================================================
 */

/*
 * CONFIGURATION MANAGEMENT: System Settings Storage
 *
 * PURPOSE: Example of storing and loading system configuration
 * EDUCATIONAL VALUE: Practical EEPROM usage with validation
 */
void EEPROM_store_config(unsigned char brightness, unsigned char volume, unsigned char mode);
unsigned char EEPROM_load_config(unsigned char *brightness, unsigned char *volume, unsigned char *mode);

/*
 * WEAR LEVELING: Write Distribution Techniques
 *
 * PURPOSE: Demonstrate simple wear leveling for frequently updated data
 * EDUCATIONAL VALUE: Memory lifetime extension and advanced techniques
 */
void EEPROM_wear_level_write(unsigned char data);
unsigned char EEPROM_wear_level_read(void);

/*
 * =============================================================================
 * EDUCATIONAL USAGE EXAMPLES AND LEARNING OBJECTIVES
 * =============================================================================
 *
 * BASIC EXAMPLE:
 *   EEPROM_init();                           // Initialize system
 *   In_EEPROM_Write(100, 0x42);             // Store byte
 *   unsigned char data = In_EEPROM_Read(100); // Retrieve byte
 *
 * CONFIGURATION EXAMPLE:
 *   EEPROM_store_config(brightness, volume, mode);  // Save settings
 *   EEPROM_load_config(&brightness, &volume, &mode); // Load settings
 *
 * BLOCK OPERATION EXAMPLE:
 *   unsigned char buffer[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
 *   EEPROM_write_block(200, buffer, 16);     // Store array
 *   EEPROM_read_block(200, buffer, 16);      // Retrieve array
 *
 * LEARNING OBJECTIVES ACHIEVED:
 * 1. ✓ Non-volatile memory programming and timing
 * 2. ✓ Data persistence across power cycles
 * 3. ✓ Memory management and address validation
 * 4. ✓ Multi-byte data storage techniques
 * 5. ✓ Error detection and data integrity
 * 6. ✓ Wear leveling and memory optimization
 * 7. ✓ Configuration management patterns
 * 8. ✓ Real-world embedded system applications
 *
 * =============================================================================
 */

#endif /* _EEPROM_H_ */
