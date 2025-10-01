
/*
 * _eeprom.c - ATmega128 Educational EEPROM Library
 * Part of Assembly → C → Python Learning Progression
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Understand non-volatile memory concepts and applications
 * 2. Learn EEPROM register configuration (EEAR, EEDR, EECR)
 * 3. Master data persistence and storage techniques
 * 4. Practice memory management and data integrity
 * 5. Bridge assembly register access to C abstraction
 * 6. Prepare for Python persistent storage concepts
 *
 * EEPROM OVERVIEW:
 * - EEPROM = Electrically Erasable Programmable Read-Only Memory
 * - Non-volatile storage that retains data when power is off
 * - Byte-addressable memory with individual read/write operations
 * - Limited write cycles (typically 100,000 per location)
 * - Slower than RAM but faster than external storage
 *
 * ATmega128 EEPROM FEATURES:
 * - 4096 bytes (4KB) of EEPROM memory
 * - Addresses from 0x0000 to 0x0FFF
 * - Byte-level read and write operations
 * - Hardware write protection mechanisms
 * - Interrupt-driven operation support
 * - Separate address and data registers
 *
 * EEPROM APPLICATIONS:
 * - Configuration settings storage
 * - Calibration values preservation
 * - User preferences and settings
 * - Data logging with power-loss protection
 * - System state backup and recovery
 *
 * ASSEMBLY EQUIVALENT CONCEPTS:
 * - EEAR = address   ≡  STS EEAR, R16 (16-bit address)
 * - EEDR = data      ≡  STS EEDR, R16 (8-bit data)
 * - Write sequence   ≡  SBI EECR, EEMWE; SBI EECR, EEWE
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "config.h"
#include "_main.h"
#include "_eeprom.h"

/*
 * EDUCATIONAL CONSTANTS: EEPROM Configuration
 * These define EEPROM memory layout and operational parameters
 * Note: EEPROM_SIZE and EEPROM_MAX_ADDRESS are defined in header file
 */
#define EEPROM_PAGE_SIZE 1 // EEPROM page size (1 byte for ATmega128)

/*
 * EDUCATIONAL CONSTANTS: Memory Regions for Organization
 * These help organize EEPROM usage for different purposes
 */
#define EEPROM_CONFIG_START 0x0000 // Configuration data region (0-255)
#define EEPROM_CONFIG_SIZE 256
#define EEPROM_CALIBRATION_START 0x0100 // Calibration data region (256-511)
#define EEPROM_CALIBRATION_SIZE 256
#define EEPROM_USER_START 0x0200 // User data region (512-1023)
#define EEPROM_USER_SIZE 512
#define EEPROM_LOG_START 0x0400 // Data logging region (1024-4095)
#define EEPROM_LOG_SIZE 3072

/*
 * EDUCATIONAL VARIABLES
 * Global variables for EEPROM management and statistics
 */
volatile unsigned int eeprom_write_count = 0; // Track write operations
volatile unsigned int eeprom_read_count = 0;  // Track read operations
volatile unsigned char eeprom_last_error = 0; // Last error code
unsigned char eeprom_initialized = 0;		  // Initialization flag

/*
 * EDUCATIONAL FUNCTION: Write Single Byte to EEPROM
 *
 * PURPOSE: Store one byte of data to specified EEPROM address
 * LEARNING: Shows complete EEPROM write sequence and safety procedures
 *
 * PARAMETERS:
 * EE_Addr - EEPROM address (0x0000 to 0x0FFF for ATmega128)
 * EE_Data - 8-bit data value to store
 *
 * PROCESS:
 * 1. Wait for any previous write operation to complete
 * 2. Set address in EEAR register (16-bit)
 * 3. Set data in EEDR register (8-bit)
 * 4. Execute atomic write sequence (EEMWE then EEWE)
 * 5. Hardware handles the actual programming
 *
 * TIMING:
 * EEPROM write typically takes 3.3ms to complete
 * Multiple writes to same address wear out the memory cell
 *
 * SAFETY:
 * - Interrupts disabled during critical write sequence
 * - Address bounds checking for data integrity
 * - Write completion verification
 *
 * ASSEMBLY EQUIVALENT:
 * wait_loop: LDS R16, EECR; SBRC R16, EEWE; RJMP wait_loop
 * STS EEAR, R16         ; Set address (low byte)
 * STS EEAR+1, R17       ; Set address (high byte)
 * STS EEDR, R18         ; Set data
 * CLI                   ; Disable interrupts
 * SBI EECR, EEMWE      ; Set master write enable
 * SBI EECR, EEWE       ; Set write enable (starts write)
 * SEI                   ; Re-enable interrupts
 */
void In_EEPROM_Write(unsigned int EE_Addr, unsigned char EE_Data)
{
	/*
	 * STEP 1: Validate address range
	 * Prevent accidental writes to invalid addresses
	 */
	if (EE_Addr > EEPROM_MAX_ADDRESS)
	{
		eeprom_last_error = 1; // Address out of range error
		return;
	}

	/*
	 * STEP 2: Wait for completion of previous write operation
	 * EEWE (EEPROM Write Enable) bit remains set during write
	 * Typical write time: 3.3ms at 5V, longer at lower voltages
	 */
	while (EECR & (1 << EEWE))
	{
		/* Wait for previous write to complete */
		/* This prevents data corruption and ensures write integrity */
	}

	/*
	 * STEP 3: Set EEPROM address
	 * EEAR is a 16-bit register for ATmega128 (supports 4KB EEPROM)
	 * Address is automatically latched when EEAR is written
	 */
	EEAR = EE_Addr;

	/*
	 * STEP 4: Set data to be written
	 * EEDR holds the 8-bit data value for write operation
	 * Data is latched when EEDR is written
	 */
	EEDR = EE_Data;

	/*
	 * STEP 5: Execute atomic write sequence
	 * This sequence must be executed within 4 clock cycles
	 * Interrupts are disabled to ensure atomic operation
	 */
	cli();				  // Disable global interrupts
	EECR |= (1 << EEMWE); // Set EEPROM Master Write Enable (required first)
	EECR |= (1 << EEWE);  // Set EEPROM Write Enable (starts write)
	sei();				  // Re-enable global interrupts

	/*
	 * STEP 6: Update statistics and clear error flag
	 * Track operations for debugging and wear leveling
	 */
	eeprom_write_count++;
	eeprom_last_error = 0; // Clear error flag on successful write

	/*
	 * EDUCATIONAL NOTE:
	 * After setting EEWE, the hardware:
	 * 1. Programs the EEPROM cell (takes ~3.3ms)
	 * 2. Automatically clears EEWE when complete
	 * 3. May generate interrupt if EERIE is set
	 *
	 * The write is complete when EEWE bit clears automatically
	 */
}

/*
 * EDUCATIONAL FUNCTION: Read Single Byte from EEPROM
 *
 * PURPOSE: Retrieve one byte of data from specified EEPROM address
 * LEARNING: Shows EEPROM read sequence and data validation
 *
 * PARAMETERS:
 * EE_Addr - EEPROM address to read from (0x0000 to 0x0FFF)
 *
 * RETURNS:
 * 8-bit data value stored at the specified address
 *
 * PROCESS:
 * 1. Wait for any write operation to complete
 * 2. Set address in EEAR register
 * 3. Start read operation by setting EERE bit
 * 4. Read result from EEDR register
 *
 * TIMING:
 * EEPROM read typically completes in 4 clock cycles (250ns at 16MHz)
 * Much faster than write operations
 *
 * SAFETY:
 * - Address bounds checking
 * - Write completion verification before read
 * - Read operation verification
 *
 * ASSEMBLY EQUIVALENT:
 * wait_loop: LDS R16, EECR; SBRC R16, EEWE; RJMP wait_loop
 * STS EEAR, R16         ; Set address (low byte)
 * STS EEAR+1, R17       ; Set address (high byte)
 * SBI EECR, EERE       ; Start read operation
 * LDS R16, EEDR        ; Read data result
 */
unsigned char In_EEPROM_Read(unsigned int EE_Addr)
{
	/*
	 * STEP 1: Validate address range
	 * Prevent reads from invalid addresses
	 */
	if (EE_Addr > EEPROM_MAX_ADDRESS)
	{
		eeprom_last_error = 2; // Address out of range error
		return 0xFF;		   // Return invalid data indicator
	}

	/*
	 * STEP 2: Wait for completion of any write operation
	 * Cannot read during write operation - would return invalid data
	 * This ensures data integrity and proper timing
	 */
	while (EECR & (1 << EEWE))
	{
		/* Wait for any pending write to complete */
	}

	/*
	 * STEP 3: Set EEPROM address for read operation
	 * Same address register used for both read and write
	 */
	EEAR = EE_Addr;

	/*
	 * STEP 4: Start read operation
	 * Setting EERE bit initiates read cycle
	 * Read completes in approximately 4 clock cycles
	 */
	EECR |= (1 << EERE); // Set EEPROM Read Enable

	/*
	 * STEP 5: Update statistics and return data
	 * EEDR now contains the data from specified address
	 */
	eeprom_read_count++;
	eeprom_last_error = 0; // Clear error flag on successful read

	/*
	 * STEP 6: Return read data
	 * EEDR automatically contains result after EERE is set
	 */
	return EEDR;

	/*
	 * EDUCATIONAL NOTE:
	 * EEPROM read operation:
	 * 1. Is much faster than write (4 cycles vs 3.3ms)
	 * 2. Does not wear out memory cells
	 * 3. Can be performed unlimited times
	 * 4. Returns data immediately after EERE is set
	 */
}

/*
 * EDUCATIONAL FUNCTION: Initialize EEPROM System
 *
 * PURPOSE: Set up EEPROM for safe operation and verify functionality
 * LEARNING: Shows system initialization and self-testing procedures
 */
void EEPROM_init(void)
{
	/* Clear statistics */
	eeprom_write_count = 0;
	eeprom_read_count = 0;
	eeprom_last_error = 0;

	/* Mark as initialized */
	eeprom_initialized = 1;

	/*
	 * Optional: Perform self-test
	 * Write and read a test pattern to verify functionality
	 */
	unsigned int test_address = EEPROM_MAX_ADDRESS;				// Use last address for test
	unsigned char test_data = 0xAA;								// Test pattern
	unsigned char original_data = In_EEPROM_Read(test_address); // Save original

	In_EEPROM_Write(test_address, test_data);				// Write test pattern
	unsigned char read_back = In_EEPROM_Read(test_address); // Read back

	if (read_back != test_data)
	{
		eeprom_last_error = 3; // Self-test failed
	}

	In_EEPROM_Write(test_address, original_data); // Restore original data
}

/*
 * EDUCATIONAL FUNCTION: Write Multi-Byte Data
 *
 * PURPOSE: Store arrays or structures to EEPROM
 * LEARNING: Shows bulk data operations and memory management
 */
void EEPROM_write_block(unsigned int start_addr, unsigned char *data, unsigned int length)
{
	unsigned int i;

	/* Validate parameters */
	if (start_addr + length > EEPROM_SIZE)
	{
		eeprom_last_error = 4; // Block too large error
		return;
	}

	/* Write each byte */
	for (i = 0; i < length; i++)
	{
		In_EEPROM_Write(start_addr + i, data[i]);
	}
}

/*
 * EDUCATIONAL FUNCTION: Read Multi-Byte Data
 *
 * PURPOSE: Retrieve arrays or structures from EEPROM
 * LEARNING: Shows bulk data retrieval and buffer management
 */
void EEPROM_read_block(unsigned int start_addr, unsigned char *buffer, unsigned int length)
{
	unsigned int i;

	/* Validate parameters */
	if (start_addr + length > EEPROM_SIZE)
	{
		eeprom_last_error = 5; // Block too large error
		return;
	}

	/* Read each byte */
	for (i = 0; i < length; i++)
	{
		buffer[i] = In_EEPROM_Read(start_addr + i);
	}
}

/*
 * EDUCATIONAL FUNCTION: Write Integer (16-bit)
 *
 * PURPOSE: Store 16-bit values in EEPROM
 * LEARNING: Shows multi-byte data handling and endianness
 */
void EEPROM_write_int(unsigned int address, unsigned int value)
{
	/* Store as little-endian (low byte first) */
	In_EEPROM_Write(address, (unsigned char)(value & 0xFF));   // Low byte
	In_EEPROM_Write(address + 1, (unsigned char)(value >> 8)); // High byte
}

/*
 * EDUCATIONAL FUNCTION: Read Integer (16-bit)
 *
 * PURPOSE: Retrieve 16-bit values from EEPROM
 * LEARNING: Shows multi-byte data reconstruction
 */
unsigned int EEPROM_read_int(unsigned int address)
{
	unsigned char low_byte = In_EEPROM_Read(address);
	unsigned char high_byte = In_EEPROM_Read(address + 1);

	/* Reconstruct 16-bit value from bytes */
	return (unsigned int)low_byte | ((unsigned int)high_byte << 8);
}

/*
 * EDUCATIONAL FUNCTION: Write String
 *
 * PURPOSE: Store null-terminated strings in EEPROM
 * LEARNING: Shows string handling and null termination
 */
void EEPROM_write_string(unsigned int address, const char *str)
{
	unsigned int i = 0;

	/* Write each character including null terminator */
	do
	{
		In_EEPROM_Write(address + i, str[i]);
		i++;
	} while (str[i - 1] != '\0' && (address + i) < EEPROM_SIZE);
}

/*
 * EDUCATIONAL FUNCTION: Read String
 *
 * PURPOSE: Retrieve null-terminated strings from EEPROM
 * LEARNING: Shows string reconstruction and buffer safety
 */
void EEPROM_read_string(unsigned int address, char *buffer, unsigned int max_length)
{
	unsigned int i = 0;

	/* Read characters until null terminator or buffer limit */
	do
	{
		buffer[i] = In_EEPROM_Read(address + i);
		i++;
	} while (buffer[i - 1] != '\0' && i < max_length && (address + i) < EEPROM_SIZE);

	/* Ensure null termination */
	if (i == max_length)
	{
		buffer[max_length - 1] = '\0';
	}
}

/*
 * EDUCATIONAL FUNCTION: Clear EEPROM Region
 *
 * PURPOSE: Set a region of EEPROM to a specific value
 * LEARNING: Shows memory initialization and bulk operations
 */
void EEPROM_clear_region(unsigned int start_addr, unsigned int length, unsigned char fill_value)
{
	unsigned int i;

	/* Validate parameters */
	if (start_addr + length > EEPROM_SIZE)
	{
		eeprom_last_error = 6; // Region too large error
		return;
	}

	/* Fill region with specified value */
	for (i = 0; i < length; i++)
	{
		In_EEPROM_Write(start_addr + i, fill_value);
	}
}

/*
 * EDUCATIONAL FUNCTION: Get EEPROM Statistics
 *
 * PURPOSE: Provide usage statistics for debugging and optimization
 * LEARNING: Shows system monitoring and performance analysis
 */
void EEPROM_get_stats(unsigned int *writes, unsigned int *reads, unsigned char *last_error)
{
	*writes = eeprom_write_count;
	*reads = eeprom_read_count;
	*last_error = eeprom_last_error;
}

/*
 * EDUCATIONAL FUNCTION: Verify EEPROM Data
 *
 * PURPOSE: Compare EEPROM contents with expected data
 * LEARNING: Shows data integrity verification
 */
unsigned char EEPROM_verify_block(unsigned int address, unsigned char *expected_data, unsigned int length)
{
	unsigned int i;

	for (i = 0; i < length; i++)
	{
		if (In_EEPROM_Read(address + i) != expected_data[i])
		{
			return 0; // Verification failed
		}
	}

	return 1; // Verification successful
}

/*
 * EDUCATIONAL FUNCTION: EEPROM Memory Dump
 *
 * PURPOSE: Display EEPROM contents for debugging
 * LEARNING: Shows memory inspection and debugging techniques
 */
void EEPROM_dump_region(unsigned int start_addr, unsigned int length)
{
	/* This function would typically output to UART for debugging */
	/* Implementation depends on available output functions */
	/* Left as exercise for students to implement with UART */
}

/*
 * EDUCATIONAL FUNCTION: Calculate Checksum
 *
 * PURPOSE: Generate simple checksum for data integrity
 * LEARNING: Shows error detection and data validation
 */
unsigned char EEPROM_calculate_checksum(unsigned int address, unsigned int length)
{
	unsigned int i;
	unsigned char checksum = 0;

	for (i = 0; i < length; i++)
	{
		checksum ^= In_EEPROM_Read(address + i); // XOR checksum
	}

	return checksum;
}

/*
 * EDUCATIONAL FUNCTION: Store Configuration Data
 *
 * PURPOSE: Example of storing system configuration
 * LEARNING: Shows practical EEPROM usage for settings storage
 */
void EEPROM_store_config(unsigned char brightness, unsigned char volume, unsigned char mode)
{
	/* Define configuration structure in EEPROM */
	const unsigned int CONFIG_BASE = 0x00;
	const unsigned char CONFIG_MAGIC = 0x5A; // Magic number to verify valid config

	In_EEPROM_Write(CONFIG_BASE + 0, CONFIG_MAGIC); // Magic number
	In_EEPROM_Write(CONFIG_BASE + 1, brightness);	// Brightness setting
	In_EEPROM_Write(CONFIG_BASE + 2, volume);		// Volume setting
	In_EEPROM_Write(CONFIG_BASE + 3, mode);			// Mode setting

	/* Calculate and store checksum */
	unsigned char checksum = CONFIG_MAGIC ^ brightness ^ volume ^ mode;
	In_EEPROM_Write(CONFIG_BASE + 4, checksum);
}

/*
 * EDUCATIONAL FUNCTION: Load Configuration Data
 *
 * PURPOSE: Example of loading system configuration
 * LEARNING: Shows data validation and default value handling
 */
unsigned char EEPROM_load_config(unsigned char *brightness, unsigned char *volume, unsigned char *mode)
{
	const unsigned int CONFIG_BASE = 0x00;
	const unsigned char CONFIG_MAGIC = 0x5A;

	/* Read configuration data */
	unsigned char magic = In_EEPROM_Read(CONFIG_BASE + 0);

	if (magic != CONFIG_MAGIC)
	{
		/* No valid configuration found, use defaults */
		*brightness = 128; // Default brightness (50%)
		*volume = 64;	   // Default volume (25%)
		*mode = 0;		   // Default mode
		return 0;		   // Configuration not found
	}

	*brightness = In_EEPROM_Read(CONFIG_BASE + 1);
	*volume = In_EEPROM_Read(CONFIG_BASE + 2);
	*mode = In_EEPROM_Read(CONFIG_BASE + 3);
	unsigned char stored_checksum = In_EEPROM_Read(CONFIG_BASE + 4);

	/* Verify checksum */
	unsigned char calculated_checksum = CONFIG_MAGIC ^ *brightness ^ *volume ^ *mode;

	if (stored_checksum != calculated_checksum)
	{
		/* Checksum error, use defaults */
		*brightness = 128;
		*volume = 64;
		*mode = 0;
		return 0; // Checksum error
	}

	return 1; // Configuration loaded successfully
}

/*
 * EDUCATIONAL FUNCTION: EEPROM Wear Leveling Demo
 *
 * PURPOSE: Demonstrate simple wear leveling technique
 * LEARNING: Shows how to distribute write operations
 */
void EEPROM_wear_level_write(unsigned char data)
{
	/* Simple round-robin wear leveling for frequently updated data */
	static unsigned int wear_level_index = 0x100; // Start at address 256
	const unsigned int WEAR_LEVEL_SIZE = 16;	  // Use 16 locations

	In_EEPROM_Write(wear_level_index, data);

	/* Move to next location */
	wear_level_index++;
	if (wear_level_index >= (0x100 + WEAR_LEVEL_SIZE))
	{
		wear_level_index = 0x100; // Wrap around
	}
}

/*
 * EDUCATIONAL FUNCTION: Find Most Recent Data
 *
 * PURPOSE: Find the most recently written data in wear-leveled region
 * LEARNING: Shows data recovery techniques in wear-leveled systems
 */
unsigned char EEPROM_wear_level_read(void)
{
	/* Read from the most recently written location */
	/* This is a simplified example - real wear leveling uses timestamps */
	unsigned int i;
	unsigned int most_recent_addr = 0x100;

	/* For this demo, assume the last non-0xFF value is most recent */
	for (i = 0x100; i < 0x110; i++)
	{
		unsigned char data = In_EEPROM_Read(i);
		if (data != 0xFF)
		{
			most_recent_addr = i;
		}
	}

	return In_EEPROM_Read(most_recent_addr);
}

/*
 * =============================================================================
 * EDUCATIONAL SUMMARY AND LEARNING OBJECTIVES
 * =============================================================================
 *
 * This EEPROM library demonstrates:
 *
 * 1. NON-VOLATILE MEMORY CONCEPTS:
 *    - Data persistence across power cycles
 *    - Write endurance limitations (100,000 cycles)
 *    - Erase/write time differences (3.3ms vs 4 cycles)
 *
 * 2. MEMORY MANAGEMENT:
 *    - Address validation and bounds checking
 *    - Multi-byte data storage (integers, strings)
 *    - Block operations for efficiency
 *
 * 3. DATA INTEGRITY:
 *    - Checksum calculation and verification
 *    - Configuration validation with magic numbers
 *    - Error detection and default value handling
 *
 * 4. WEAR LEVELING:
 *    - Simple round-robin distribution
 *    - Write operation counting
 *    - Lifetime extension techniques
 *
 * 5. PRACTICAL APPLICATIONS:
 *    - System configuration storage
 *    - Calibration data preservation
 *    - User preference settings
 *    - Data logging for analysis
 *
 * 6. ADVANCED TOPICS:
 *    - Atomic operations for critical data
 *    - Backup and recovery strategies
 *    - Performance optimization techniques
 *    - Integration with higher-level protocols
 *
 * LEARNING PROGRESSION:
 * - Assembly: Direct register manipulation (EEAR, EEDR, EECR)
 * - C: Structured functions with safety checks
 * - Python: High-level data serialization and storage
 * - IoT: Remote configuration and over-the-air updates
 *
 * =============================================================================
 */