/*
 * =============================================================================
 * I2C SENSORS - HANDS-ON LAB EXERCISES
 * =============================================================================
 * ATmega128 Educational Framework - Lab Session
 *
 * OBJECTIVE: Master I2C communication and sensor integration
 * DURATION: 90 minutes
 * DIFFICULTY: Advanced
 *
 * STUDENTS WILL:
 * - Configure I2C master communication protocol
 * - Interface with multiple I2C sensors simultaneously
 * - Implement sensor data fusion and calibration
 * - Debug I2C communication issues
 * - Create sensor-based applications
 *
 * HARDWARE REQUIRED:
 * - ATmega128 board
 * - I2C bus with pull-up resistors (4.7kΩ)
 * - Temperature sensor (LM75A or DS1621)
 * - Accelerometer (ADXL345 or MPU6050)
 * - EEPROM (24C64 or similar)
 * - Real-time clock (DS1307 or DS3231)
 * - 4 push buttons for interaction
 * - Status LEDs
 *
 * I2C ADDRESSES:
 * - LM75A Temperature: 0x48
 * - ADXL345 Accelerometer: 0x53
 * - 24C64 EEPROM: 0x50
 * - DS1307 RTC: 0x68
 *
 * LAB STRUCTURE:
 * - Exercise 1: I2C Bus scanning and device detection (20 min)
 * - Exercise 2: Temperature sensor reading and calibration (25 min)
 * - Exercise 3: Accelerometer data acquisition and processing (25 min)
 * - Exercise 4: Multi-sensor data logging system (20 min)
 *
 * =============================================================================
 */

#include "config.h"

// I2C device addresses
#define LM75A_ADDR 0x48   // Temperature sensor
#define ADXL345_ADDR 0x53 // Accelerometer
#define EEPROM_ADDR 0x50  // EEPROM memory
#define DS1307_ADDR 0x68  // Real-time clock

// I2C register definitions
#define LM75A_TEMP_REG 0x00
#define LM75A_CONFIG_REG 0x01

#define ADXL345_DEVID_REG 0x00
#define ADXL345_POWER_CTL 0x2D
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0 0x32

// Lab session variables
uint16_t lab_score = 0;
uint8_t devices_found = 0;
uint8_t sensor_errors = 0;

/*
 * =============================================================================
 * I2C COMMUNICATION FUNCTIONS
 * =============================================================================
 */

void i2c_init(void)
{
    // Set I2C bit rate to 100kHz (TWBR calculation for 7.3728MHz)
    TWBR = 32;
    TWSR = 0x00; // Prescaler = 1

    // Enable I2C
    TWCR = (1 << TWEN);
}

uint8_t i2c_start(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)))
        ;
    return (TWSR & 0xF8);
}

void i2c_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    while (TWCR & (1 << TWSTO))
        ;
}

uint8_t i2c_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)))
        ;
    return (TWSR & 0xF8);
}

uint8_t i2c_read_ack(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)))
        ;
    return TWDR;
}

uint8_t i2c_read_nack(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)))
        ;
    return TWDR;
}

/*
 * =============================================================================
 * SENSOR-SPECIFIC FUNCTIONS
 * =============================================================================
 */

uint8_t i2c_device_exists(uint8_t address)
{
    uint8_t status;

    status = i2c_start();
    if (status != 0x08)
        return 0; // Start condition failed

    status = i2c_write((address << 1) | 0x00); // Write mode
    i2c_stop();

    return (status == 0x18); // ACK received
}

int16_t read_lm75a_temperature(void)
{
    uint8_t temp_high, temp_low;
    uint8_t status;

    // Start I2C communication
    status = i2c_start();
    if (status != 0x08)
        return -999; // Error indicator

    // Send device address with write bit
    status = i2c_write((LM75A_ADDR << 1) | 0x00);
    if (status != 0x18)
    {
        i2c_stop();
        return -999;
    }

    // Send temperature register address
    status = i2c_write(LM75A_TEMP_REG);
    if (status != 0x28)
    {
        i2c_stop();
        return -999;
    }

    // Restart for read operation
    status = i2c_start();
    if (status != 0x10)
    {
        i2c_stop();
        return -999;
    }

    // Send device address with read bit
    status = i2c_write((LM75A_ADDR << 1) | 0x01);
    if (status != 0x40)
    {
        i2c_stop();
        return -999;
    }

    // Read temperature data
    temp_high = i2c_read_ack();
    temp_low = i2c_read_nack();
    i2c_stop();

    // Convert to temperature (0.5°C resolution)
    int16_t temperature = (temp_high << 8) | temp_low;
    temperature = temperature >> 7; // Right shift for 0.5°C resolution

    return temperature; // Temperature * 2 (to avoid float)
}

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} accel_data_t;

uint8_t init_adxl345(void)
{
    uint8_t status;

    // Check device ID
    status = i2c_start();
    if (status != 0x08)
        return 0;

    i2c_write((ADXL345_ADDR << 1) | 0x00);
    i2c_write(ADXL345_DEVID_REG);

    i2c_start();
    i2c_write((ADXL345_ADDR << 1) | 0x01);
    uint8_t device_id = i2c_read_nack();
    i2c_stop();

    if (device_id != 0xE5)
        return 0; // Wrong device ID

    // Configure power control - measurement mode
    i2c_start();
    i2c_write((ADXL345_ADDR << 1) | 0x00);
    i2c_write(ADXL345_POWER_CTL);
    i2c_write(0x08); // Measurement mode
    i2c_stop();

    // Configure data format - ±2g range
    i2c_start();
    i2c_write((ADXL345_ADDR << 1) | 0x00);
    i2c_write(ADXL345_DATA_FORMAT);
    i2c_write(0x00); // ±2g, 10-bit resolution
    i2c_stop();

    return 1; // Success
}

accel_data_t read_adxl345_data(void)
{
    accel_data_t accel = {0, 0, 0};
    uint8_t data[6];

    // Read 6 bytes starting from DATAX0
    i2c_start();
    i2c_write((ADXL345_ADDR << 1) | 0x00);
    i2c_write(ADXL345_DATAX0);

    i2c_start();
    i2c_write((ADXL345_ADDR << 1) | 0x01);

    for (uint8_t i = 0; i < 5; i++)
    {
        data[i] = i2c_read_ack();
    }
    data[5] = i2c_read_nack();
    i2c_stop();

    // Combine bytes (little endian)
    accel.x = (int16_t)((data[1] << 8) | data[0]);
    accel.y = (int16_t)((data[3] << 8) | data[2]);
    accel.z = (int16_t)((data[5] << 8) | data[4]);

    return accel;
}

/*
 * =============================================================================
 * LAB EXERCISE 1: I2C BUS SCANNING (20 minutes)
 * =============================================================================
 * OBJECTIVE: Learn I2C communication basics and device detection
 * DIFFICULTY: ★★☆☆☆ (Easy-Medium)
 */

void lab_ex1_i2c_bus_scan(void)
{
    /*
     * CHALLENGE: Scan the I2C bus for connected devices
     * TASK: Detect all I2C devices and display their addresses
     * LEARNING: I2C addressing, ACK/NACK responses
     */

    puts_USART1("\\r\\n=== Lab 1: I2C Bus Scanning ===\\r\\n");
    puts_USART1("Scanning I2C bus for devices...\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "I2C BUS SCANNER");
    lcd_string(1, 0, "Detecting devices...");

    i2c_init();
    devices_found = 0;

    puts_USART1("Address scan results:\\r\\n");
    puts_USART1("Addr  Status    Device Type\\r\\n");
    puts_USART1("----  ------    -----------\\r\\n");

    for (uint8_t addr = 0x08; addr < 0x78; addr++)
    {
        if (i2c_device_exists(addr))
        {
            devices_found++;

            char msg[50];
            sprintf(msg, "0x%02X  Found     ", addr);
            puts_USART1(msg);

            // Identify common devices
            switch (addr)
            {
            case LM75A_ADDR:
                puts_USART1("LM75A Temperature\\r\\n");
                break;
            case ADXL345_ADDR:
                puts_USART1("ADXL345 Accelerometer\\r\\n");
                break;
            case EEPROM_ADDR:
                puts_USART1("24C64 EEPROM\\r\\n");
                break;
            case DS1307_ADDR:
                puts_USART1("DS1307 RTC\\r\\n");
                break;
            default:
                puts_USART1("Unknown Device\\r\\n");
                break;
            }

            char lcd_msg[20];
            sprintf(lcd_msg, "0x%02X: Found", addr);
            lcd_string(2 + (devices_found % 4), 0, lcd_msg);
        }

        _delay_ms(10); // Small delay between scans
    }

    char summary[50];
    sprintf(summary, "\\r\\nScan complete! Found %d devices\\r\\n", devices_found);
    puts_USART1(summary);

    sprintf(summary, "Devices: %d", devices_found);
    lcd_string(6, 0, summary);

    if (devices_found >= 2)
    {
        lab_score += 100;
        puts_USART1("✓ Multiple devices detected!\\r\\n");
    }
    else
    {
        puts_USART1("⚠ Check connections if devices missing\\r\\n");
    }
}

void lab_ex1_i2c_communication_test(void)
{
    /*
     * CHALLENGE: Test basic I2C read/write operations
     * TASK: Verify I2C communication reliability
     * LEARNING: I2C error handling and status checking
     */

    puts_USART1("\\r\\n=== Lab 1.2: Communication Test ===\\r\\n");
    puts_USART1("Testing I2C communication reliability\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "I2C COMM TEST");
    lcd_string(1, 0, "Testing reads...");

    uint8_t test_count = 0;
    uint8_t success_count = 0;

    // Test communication with each found device
    for (uint8_t addr = 0x08; addr < 0x78; addr++)
    {
        if (i2c_device_exists(addr))
        {
            test_count++;

            char test_msg[40];
            sprintf(test_msg, "Testing 0x%02X... ", addr);
            puts_USART1(test_msg);

            // Perform multiple read attempts
            uint8_t attempts = 5;
            uint8_t successes = 0;

            for (uint8_t i = 0; i < attempts; i++)
            {
                if (i2c_device_exists(addr))
                {
                    successes++;
                }
                _delay_ms(50);
            }

            if (successes == attempts)
            {
                puts_USART1("✓ PASS\\r\\n");
                success_count++;
            }
            else
            {
                sprintf(test_msg, "⚠ UNSTABLE (%d/%d)\\r\\n", successes, attempts);
                puts_USART1(test_msg);
            }
        }
    }

    char result[50];
    sprintf(result, "Communication test: %d/%d devices stable\\r\\n", success_count, test_count);
    puts_USART1(result);

    if (success_count >= test_count)
    {
        lab_score += 100;
        puts_USART1("✓ All devices communicating reliably!\\r\\n");
    }
}

/*
 * =============================================================================
 * LAB EXERCISE 2: TEMPERATURE SENSOR (25 minutes)
 * =============================================================================
 * OBJECTIVE: Master analog sensor interfacing and calibration
 * DIFFICULTY: ★★★☆☆ (Medium)
 */

void lab_ex2_temperature_reading(void)
{
    /*
     * CHALLENGE: Read and display temperature data
     * TASK: Implement continuous temperature monitoring
     * LEARNING: Sensor data conversion and filtering
     */

    puts_USART1("\\r\\n=== Lab 2: Temperature Reading ===\\r\\n");
    puts_USART1("Reading LM75A temperature sensor\\r\\n");
    puts_USART1("Press button to stop monitoring...\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "TEMPERATURE MONITOR");
    lcd_string(1, 0, "LM75A Sensor");

    if (!i2c_device_exists(LM75A_ADDR))
    {
        puts_USART1("❌ LM75A not found!\\r\\n");
        lcd_string(3, 0, "Sensor not found!");
        return;
    }

    uint8_t reading_count = 0;
    int32_t temp_sum = 0;
    int16_t temp_min = 1000;
    int16_t temp_max = -1000;

    while (!button_pressed(0) && reading_count < 50)
    {
        int16_t temp_raw = read_lm75a_temperature();

        if (temp_raw != -999) // Valid reading
        {
            reading_count++;
            temp_sum += temp_raw;

            if (temp_raw < temp_min)
                temp_min = temp_raw;
            if (temp_raw > temp_max)
                temp_max = temp_raw;

            // Display current temperature (divide by 2 for actual temp)
            char temp_msg[30];
            sprintf(temp_msg, "Temp: %d.%d°C     ", temp_raw / 2, (temp_raw % 2) * 5);
            lcd_string(3, 0, temp_msg);

            sprintf(temp_msg, "Reading #%d", reading_count);
            lcd_string(4, 0, temp_msg);

            char serial_msg[50];
            sprintf(serial_msg, "Reading %d: %d.%d°C\\r\\n", reading_count, temp_raw / 2, (temp_raw % 2) * 5);
            puts_USART1(serial_msg);
        }
        else
        {
            sensor_errors++;
            puts_USART1("❌ Temperature read error\\r\\n");
            lcd_string(5, 0, "Read error!");
        }

        _delay_ms(500);
    }

    // Display statistics
    if (reading_count > 0)
    {
        int16_t temp_avg = temp_sum / reading_count;

        char stats[60];
        sprintf(stats, "\\r\\nTemperature Statistics (%d readings):\\r\\n", reading_count);
        puts_USART1(stats);

        sprintf(stats, "Average: %d.%d°C\\r\\n", temp_avg / 2, (temp_avg % 2) * 5);
        puts_USART1(stats);

        sprintf(stats, "Minimum: %d.%d°C\\r\\n", temp_min / 2, (temp_min % 2) * 5);
        puts_USART1(stats);

        sprintf(stats, "Maximum: %d.%d°C\\r\\n", temp_max / 2, (temp_max % 2) * 5);
        puts_USART1(stats);

        lab_score += 150;
    }
}

/*
 * =============================================================================
 * LAB EXERCISE 3: ACCELEROMETER (25 minutes)
 * =============================================================================
 * OBJECTIVE: Master multi-axis sensor data processing
 * DIFFICULTY: ★★★★☆ (Medium-Hard)
 */

void lab_ex3_accelerometer_setup(void)
{
    /*
     * CHALLENGE: Initialize and configure ADXL345
     * TASK: Set up accelerometer for data acquisition
     * LEARNING: Sensor configuration and validation
     */

    puts_USART1("\\r\\n=== Lab 3: Accelerometer Setup ===\\r\\n");
    puts_USART1("Initializing ADXL345 accelerometer\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "ACCELEROMETER INIT");
    lcd_string(1, 0, "ADXL345 Setup");

    if (!i2c_device_exists(ADXL345_ADDR))
    {
        puts_USART1("❌ ADXL345 not found at 0x53!\\r\\n");
        lcd_string(3, 0, "Device not found!");
        return;
    }

    if (init_adxl345())
    {
        puts_USART1("✓ ADXL345 initialized successfully\\r\\n");
        lcd_string(3, 0, "Init successful!");
        lab_score += 100;
    }
    else
    {
        puts_USART1("❌ ADXL345 initialization failed\\r\\n");
        lcd_string(3, 0, "Init failed!");
        return;
    }

    // Test initial reading
    accel_data_t accel = read_adxl345_data();

    char accel_msg[50];
    sprintf(accel_msg, "Initial: X=%d Y=%d Z=%d\\r\\n", accel.x, accel.y, accel.z);
    puts_USART1(accel_msg);

    sprintf(accel_msg, "X:%d Y:%d Z:%d", accel.x, accel.y, accel.z);
    lcd_string(5, 0, accel_msg);
}

void lab_ex3_motion_detection(void)
{
    /*
     * CHALLENGE: Implement motion detection and tilt sensing
     * TASK: Process accelerometer data for motion analysis
     * LEARNING: Signal processing and threshold detection
     */

    puts_USART1("\\r\\n=== Lab 3.2: Motion Detection ===\\r\\n");
    puts_USART1("Monitoring motion and tilt...\\r\\n");
    puts_USART1("Press button to stop monitoring...\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "MOTION DETECTOR");
    lcd_string(1, 0, "Move the board!");

    accel_data_t baseline = read_adxl345_data();
    uint8_t motion_count = 0;
    uint16_t motion_threshold = 50; // Adjust based on sensitivity needed

    while (!button_pressed(0) && motion_count < 100)
    {
        accel_data_t current = read_adxl345_data();

        // Calculate motion magnitude
        int16_t delta_x = current.x - baseline.x;
        int16_t delta_y = current.y - baseline.y;
        int16_t delta_z = current.z - baseline.z;

        uint16_t motion_magnitude = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);

        // Display current readings
        char accel_msg[20];
        sprintf(accel_msg, "X:%4d Y:%4d", current.x, current.y);
        lcd_string(2, 0, accel_msg);

        sprintf(accel_msg, "Z:%4d Mag:%3d", current.z, motion_magnitude);
        lcd_string(3, 0, accel_msg);

        // Motion detection
        if (motion_magnitude > motion_threshold)
        {
            motion_count++;
            lcd_string(4, 0, "*** MOTION! ***");

            char motion_msg[50];
            sprintf(motion_msg, "Motion #%d: Magnitude=%d\\r\\n", motion_count, motion_magnitude);
            puts_USART1(motion_msg);

            // Update baseline after significant motion
            baseline = current;
            _delay_ms(500); // Debounce motion detection
        }
        else
        {
            lcd_string(4, 0, "   Stable...   ");
        }

        // Tilt analysis
        if (abs(current.x) > 200)
        {
            lcd_string(5, 0, "Tilt: X-axis");
        }
        else if (abs(current.y) > 200)
        {
            lcd_string(5, 0, "Tilt: Y-axis");
        }
        else
        {
            lcd_string(5, 0, "Level        ");
        }

        _delay_ms(100);
    }

    char summary[50];
    sprintf(summary, "\\r\\nMotion detection complete: %d events\\r\\n", motion_count);
    puts_USART1(summary);

    if (motion_count >= 5)
    {
        lab_score += 150;
        puts_USART1("✓ Motion detection working!\\r\\n");
    }
}

/*
 * =============================================================================
 * LAB EXERCISE 4: MULTI-SENSOR DATA LOGGER (20 minutes)
 * =============================================================================
 * OBJECTIVE: Integrate multiple sensors in a complete application
 * DIFFICULTY: ★★★★★ (Hard)
 */

void lab_ex4_data_logger(void)
{
    /*
     * CHALLENGE: Create a multi-sensor data logging system
     * TASK: Collect and log data from all connected sensors
     * LEARNING: Sensor fusion and data management
     */

    puts_USART1("\\r\\n=== Lab 4: Multi-Sensor Data Logger ===\\r\\n");
    puts_USART1("Logging data from all sensors\\r\\n");
    puts_USART1("Press button to stop logging...\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "DATA LOGGER");
    lcd_string(1, 0, "Multi-sensor");

    // Check available sensors
    uint8_t temp_available = i2c_device_exists(LM75A_ADDR);
    uint8_t accel_available = i2c_device_exists(ADXL345_ADDR);

    char sensor_status[30];
    sprintf(sensor_status, "Temp:%s Accel:%s",
            temp_available ? "OK" : "NO",
            accel_available ? "OK" : "NO");
    lcd_string(2, 0, sensor_status);

    if (accel_available)
    {
        init_adxl345();
    }

    puts_USART1("\\r\\nData Log Format:\\r\\n");
    puts_USART1("Entry, Time, Temp(°C), AccelX, AccelY, AccelZ\\r\\n");
    puts_USART1("---------------------------------------------\\r\\n");

    uint16_t log_entry = 0;
    uint32_t start_time = 0; // Simple time counter

    while (!button_pressed(0) && log_entry < 200)
    {
        log_entry++;
        start_time += 1; // Increment time counter

        // Read temperature
        int16_t temperature = -999;
        if (temp_available)
        {
            temperature = read_lm75a_temperature();
        }

        // Read accelerometer
        accel_data_t accel = {0, 0, 0};
        if (accel_available)
        {
            accel = read_adxl345_data();
        }

        // Log to serial port
        char log_msg[80];
        if (temp_available && accel_available)
        {
            sprintf(log_msg, "%3d, %5ld, %d.%d, %5d, %5d, %5d\\r\\n",
                    log_entry, start_time,
                    temperature / 2, (temperature % 2) * 5,
                    accel.x, accel.y, accel.z);
        }
        else if (temp_available)
        {
            sprintf(log_msg, "%3d, %5ld, %d.%d, -----, -----, -----\\r\\n",
                    log_entry, start_time,
                    temperature / 2, (temperature % 2) * 5);
        }
        else if (accel_available)
        {
            sprintf(log_msg, "%3d, %5ld, -----, %5d, %5d, %5d\\r\\n",
                    log_entry, start_time,
                    accel.x, accel.y, accel.z);
        }
        else
        {
            sprintf(log_msg, "%3d, %5ld, -----, -----, -----, -----\\r\\n",
                    log_entry, start_time);
        }
        puts_USART1(log_msg);

        // Update LCD display
        char lcd_msg[20];
        sprintf(lcd_msg, "Entry: %3d", log_entry);
        lcd_string(3, 0, lcd_msg);

        if (temp_available && temperature != -999)
        {
            sprintf(lcd_msg, "T:%d.%d°C", temperature / 2, (temperature % 2) * 5);
            lcd_string(4, 0, lcd_msg);
        }

        if (accel_available)
        {
            sprintf(lcd_msg, "A:%d,%d,%d    ", accel.x / 10, accel.y / 10, accel.z / 10);
            lcd_string(5, 0, lcd_msg);
        }

        _delay_ms(1000); // Log every second
    }

    char final_msg[50];
    sprintf(final_msg, "\\r\\nData logging complete: %d entries\\r\\n", log_entry);
    puts_USART1(final_msg);

    if (log_entry >= 10)
    {
        lab_score += 200;
        puts_USART1("✓ Data logging successful!\\r\\n");
    }
}

/*
 * =============================================================================
 * LAB MAIN PROGRAM - EXERCISE SELECTION
 * =============================================================================
 */

void show_lab_menu(void)
{
    puts_USART1("\\r\\n");
    puts_USART1("==============================================\\r\\n");
    puts_USART1("      I2C SENSORS - LAB EXERCISES            \\r\\n");
    puts_USART1("==============================================\\r\\n");
    puts_USART1("1. I2C Bus Scanning & Communication Test     \\r\\n");
    puts_USART1("2. Temperature Sensor (LM75A)                \\r\\n");
    puts_USART1("3. Accelerometer (ADXL345)                   \\r\\n");
    puts_USART1("4. Multi-Sensor Data Logger                  \\r\\n");
    puts_USART1("                                              \\r\\n");
    puts_USART1("0. Run All Exercises                         \\r\\n");
    puts_USART1("X. Exit Lab                                   \\r\\n");
    puts_USART1("==============================================\\r\\n");
    char score_msg[50];
    sprintf(score_msg, "Current Score: %d points\\r\\n", lab_score);
    puts_USART1(score_msg);
    puts_USART1("Select exercise (1-4, 0, X): ");
}

int main(void)
{
    init_devices();

    puts_USART1("\\r\\n*** I2C SENSORS LAB SESSION ***\\r\\n");
    puts_USART1("Welcome to hands-on I2C sensor programming!\\r\\n");
    puts_USART1("Ensure all sensors are connected with pull-up resistors!\\r\\n");

    lcd_clear();
    lcd_string(1, 0, "I2C SENSORS LAB");
    lcd_string(2, 0, "Check connections");
    lcd_string(4, 0, "Use Serial Menu");

    while (1)
    {
        show_lab_menu();
        char choice = getch_USART1();
        putch_USART1(choice);
        putch_USART1('\\r');
        putch_USART1('\\n');

        switch (choice)
        {
        case '1':
            lab_ex1_i2c_bus_scan();
            lab_ex1_i2c_communication_test();
            break;

        case '2':
            lab_ex2_temperature_reading();
            break;

        case '3':
            lab_ex3_accelerometer_setup();
            lab_ex3_motion_detection();
            break;

        case '4':
            lab_ex4_data_logger();
            break;

        case '0':
            puts_USART1("\\r\\n*** RUNNING ALL EXERCISES ***\\r\\n");
            lab_ex1_i2c_bus_scan();
            lab_ex1_i2c_communication_test();
            lab_ex2_temperature_reading();
            lab_ex3_accelerometer_setup();
            lab_ex3_motion_detection();
            lab_ex4_data_logger();

            char final_buffer[80];
            sprintf(final_buffer, "\\r\\n*** ALL EXERCISES COMPLETE! ***\\r\\nFinal Score: %d points\\r\\n", lab_score);
            puts_USART1(final_buffer);
            break;

        case 'X':
        case 'x':
            puts_USART1("\\r\\nExiting lab. Great work on I2C!\\r\\n");
            puts_USART1("Remember: I2C requires proper pull-up resistors!\\r\\n");
            lcd_clear();
            lcd_string(2, 0, "LAB COMPLETE!");
            char exit_score[30];
            sprintf(exit_score, "Score: %d pts", lab_score);
            lcd_string(3, 0, exit_score);
            while (1)
                ;

        default:
            puts_USART1("Invalid choice. Please try again.\\r\\n");
        }

        puts_USART1("\\r\\nPress any key to continue...\\r\\n");
        getch_USART1();
    }

    return 0;
}