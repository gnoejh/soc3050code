/*
 * =============================================================================
 * I2C MULTI-SENSOR INTERFACE - EDUCATIONAL DEMONSTRATION
 * =============================================================================
 *
 * PROJECT: I2C_Sensors_Multi
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Educational demonstration of interfacing multiple I2C sensors simultaneously.
 * Students learn sensor integration, data fusion, and multi-device I2C communication.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master multi-device I2C communication protocols
 * 2. Learn sensor data fusion and processing techniques
 * 3. Practice different data format handling (raw, scaled, multi-byte)
 * 4. Implement sensor calibration and validation
 * 5. Understand I2C addressing and arbitration
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - Multiple I2C sensors: MPU6050, BMP180/280, HMC5883L, DS18B20
 * - I2C connections: PD0 (SCL), PD1 (SDA) with 4.7K pull-ups
 * - LCD display for sensor data visualization
 * - LEDs for sensor status indication
 * - Serial connection for data logging (9600 baud)
 *
 * SUPPORTED SENSORS:
 * - MPU6050: 6-axis Gyro/Accelerometer (0x68/0x69)
 * - BMP180/BMP280: Temperature/Pressure (0x77/0x76)
 * - HMC5883L: 3-axis Magnetometer (0x1E)
 * - DS18B20: Temperature Sensor (0x48-0x4F)
 *
 * LEARNING PROGRESSION:
 * - Demo 1: Individual Sensor Interface
 * - Demo 2: Multi-Sensor Data Acquisition
 * - Demo 3: Sensor Data Fusion
 * - Demo 4: Environmental Monitoring System
 *
 * =============================================================================
 */

#include "config.h"

// Common I2C sensor addresses
#define MPU6050_ADDR 0x68  // Gyro/Accelerometer
#define BMP180_ADDR 0x77   // Barometric pressure
#define HMC5883L_ADDR 0x1E // Magnetometer
#define DS18B20_ADDR 0x48  // Temperature (example)

// MPU6050 Registers
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_TEMP_OUT_H 0x41

// BMP180 Registers
#define BMP180_CONTROL 0xF4
#define BMP180_RESULT 0xF6
#define BMP180_ID 0xD0

// HMC5883L Registers
#define HMC5883L_CONFIG_A 0x00
#define HMC5883L_CONFIG_B 0x01
#define HMC5883L_MODE 0x02
#define HMC5883L_DATA_X_H 0x03

// Sensor data structures
typedef struct
{
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t temperature;
    uint8_t present;
} mpu6050_data_t;

typedef struct
{
    int32_t temperature; // 0.1°C resolution
    int32_t pressure;    // Pa
    uint8_t present;
} bmp180_data_t;

typedef struct
{
    int16_t mag_x, mag_y, mag_z;
    uint8_t present;
} hmc5883l_data_t;

// Global sensor data
mpu6050_data_t mpu6050;
bmp180_data_t bmp180;
hmc5883l_data_t hmc5883l;

// I2C functions
void i2c_init(void)
{
    TWBR = 32; // 100kHz @ 7.3728MHz
    TWSR = 0x00;
    TWCR = (1 << TWEN);
}

uint8_t i2c_start(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)))
        ;
    uint8_t status = TWSR & 0xF8;
    if (status != 0x08 && status != 0x10)
        return 1;
    return 0;
}

void i2c_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    _delay_us(100);
}

uint8_t i2c_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)))
        ;
    uint8_t status = TWSR & 0xF8;
    if (status != 0x18 && status != 0x28)
        return 1;
    return 0;
}

uint8_t i2c_read(uint8_t *data, uint8_t send_ack)
{
    if (send_ack)
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    else
        TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)))
        ;
    *data = TWDR;
    return 0;
}

/*
 * Generic register read/write functions
 */
uint8_t i2c_read_register(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data)
{
    if (i2c_start())
    {
        i2c_stop();
        return 1;
    }
    if (i2c_write((dev_addr << 1) | 0x00))
    {
        i2c_stop();
        return 1;
    }
    if (i2c_write(reg_addr))
    {
        i2c_stop();
        return 1;
    }
    if (i2c_start())
    {
        i2c_stop();
        return 1;
    }
    if (i2c_write((dev_addr << 1) | 0x01))
    {
        i2c_stop();
        return 1;
    }
    if (i2c_read(data, 0))
    {
        i2c_stop();
        return 1;
    }
    i2c_stop();
    return 0;
}

uint8_t i2c_write_register(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    if (i2c_start())
    {
        i2c_stop();
        return 1;
    }
    if (i2c_write((dev_addr << 1) | 0x00))
    {
        i2c_stop();
        return 1;
    }
    if (i2c_write(reg_addr))
    {
        i2c_stop();
        return 1;
    }
    if (i2c_write(data))
    {
        i2c_stop();
        return 1;
    }
    i2c_stop();
    return 0;
}

uint8_t i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buffer, uint8_t count)
{
    if (i2c_start())
    {
        i2c_stop();
        return 1;
    }
    if (i2c_write((dev_addr << 1) | 0x00))
    {
        i2c_stop();
        return 1;
    }
    if (i2c_write(reg_addr))
    {
        i2c_stop();
        return 1;
    }
    if (i2c_start())
    {
        i2c_stop();
        return 1;
    }
    if (i2c_write((dev_addr << 1) | 0x01))
    {
        i2c_stop();
        return 1;
    }

    for (uint8_t i = 0; i < count - 1; i++)
    {
        i2c_read(&buffer[i], 1); // ACK
    }
    i2c_read(&buffer[count - 1], 0); // NACK on last byte

    i2c_stop();
    return 0;
}

/* ========================================================================
 * MPU6050 Functions
 * ======================================================================== */
uint8_t mpu6050_init(void)
{
    uint8_t who_am_i;

    // Check device ID
    if (i2c_read_register(MPU6050_ADDR, MPU6050_WHO_AM_I, &who_am_i) != 0)
    {
        return 1;
    }

    if (who_am_i != 0x68)
    {
        return 1;
    }

    // Wake up device (clear sleep bit)
    i2c_write_register(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
    _delay_ms(100);

    mpu6050.present = 1;
    return 0;
}

void mpu6050_read_all(void)
{
    if (!mpu6050.present)
        return;

    uint8_t data[14];

    // Read all sensor data at once
    if (i2c_read_bytes(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, data, 14) == 0)
    {
        mpu6050.accel_x = (data[0] << 8) | data[1];
        mpu6050.accel_y = (data[2] << 8) | data[3];
        mpu6050.accel_z = (data[4] << 8) | data[5];
        mpu6050.temperature = (data[6] << 8) | data[7];
        mpu6050.gyro_x = (data[8] << 8) | data[9];
        mpu6050.gyro_y = (data[10] << 8) | data[11];
        mpu6050.gyro_z = (data[12] << 8) | data[13];
    }
}

/* ========================================================================
 * BMP180 Functions (Simplified)
 * ======================================================================== */
uint8_t bmp180_init(void)
{
    uint8_t chip_id;

    if (i2c_read_register(BMP180_ADDR, BMP180_ID, &chip_id) != 0)
    {
        return 1;
    }

    if (chip_id != 0x55)
    {
        return 1;
    }

    bmp180.present = 1;
    return 0;
}

void bmp180_read_temperature(void)
{
    if (!bmp180.present)
        return;

    // Start temperature measurement
    i2c_write_register(BMP180_ADDR, BMP180_CONTROL, 0x2E);
    _delay_ms(5);

    // Read raw temperature
    uint8_t data[2];
    if (i2c_read_bytes(BMP180_ADDR, BMP180_RESULT, data, 2) == 0)
    {
        int16_t raw = (data[0] << 8) | data[1];
        // Simplified conversion (real conversion uses calibration data)
        bmp180.temperature = (raw * 10) / 32; // Approximation
    }
}

/* ========================================================================
 * HMC5883L Functions
 * ======================================================================== */
uint8_t hmc5883l_init(void)
{
    // Configure: 8 samples average, 15 Hz, normal measurement
    if (i2c_write_register(HMC5883L_ADDR, HMC5883L_CONFIG_A, 0x70) != 0)
    {
        return 1;
    }

    // Gain setting
    if (i2c_write_register(HMC5883L_ADDR, HMC5883L_CONFIG_B, 0xA0) != 0)
    {
        return 1;
    }

    // Continuous measurement mode
    if (i2c_write_register(HMC5883L_ADDR, HMC5883L_MODE, 0x00) != 0)
    {
        return 1;
    }

    hmc5883l.present = 1;
    return 0;
}

void hmc5883l_read(void)
{
    if (!hmc5883l.present)
        return;

    uint8_t data[6];

    if (i2c_read_bytes(HMC5883L_ADDR, HMC5883L_DATA_X_H, data, 6) == 0)
    {
        // HMC5883L data order: X, Z, Y
        hmc5883l.mag_x = (data[0] << 8) | data[1];
        hmc5883l.mag_z = (data[2] << 8) | data[3];
        hmc5883l.mag_y = (data[4] << 8) | data[5];
    }
}

/* ========================================================================
 * DEMO 1: Sensor Discovery and Information
 * ======================================================================== */
void demo1_sensor_discovery(void)
{
    puts_USART1("\r\n=== DEMO 1: Sensor Discovery ===\r\n\r\n");

    mpu6050.present = 0;
    bmp180.present = 0;
    hmc5883l.present = 0;

    puts_USART1("Scanning I2C bus for sensors...\r\n\r\n");

    // Try MPU6050
    puts_USART1("MPU6050 (Gyro/Accel) at 0x68: ");
    if (mpu6050_init() == 0)
    {
        puts_USART1("FOUND!\r\n");
        PORTC |= 0x01;
    }
    else
    {
        puts_USART1("Not detected\r\n");
    }

    // Try BMP180
    puts_USART1("BMP180 (Pressure) at 0x77: ");
    if (bmp180_init() == 0)
    {
        puts_USART1("FOUND!\r\n");
        PORTC |= 0x02;
    }
    else
    {
        puts_USART1("Not detected\r\n");
    }

    // Try HMC5883L
    puts_USART1("HMC5883L (Magnetometer) at 0x1E: ");
    if (hmc5883l_init() == 0)
    {
        puts_USART1("FOUND!\r\n");
        PORTC |= 0x04;
    }
    else
    {
        puts_USART1("Not detected\r\n");
    }

    puts_USART1("\r\nSensor Summary:\r\n");
    char buf[60];
    sprintf(buf, "  Active sensors: %u\r\n",
            mpu6050.present + bmp180.present + hmc5883l.present);
    puts_USART1(buf);

    puts_USART1("\r\nPress any key to continue...");
    getch_USART1();
}

/* ========================================================================
 * DEMO 2: Real-Time Multi-Sensor Display
 * ======================================================================== */
void demo2_realtime_display(void)
{
    puts_USART1("\r\n=== DEMO 2: Real-Time Sensor Data ===\r\n");
    puts_USART1("Press any key to stop\r\n\r\n");

    while (1)
    {
        char buf[100];

        // Clear screen (simplified)
        puts_USART1("\r                                                      ");

        // Read and display MPU6050
        if (mpu6050.present)
        {
            mpu6050_read_all();

            // Convert to g (±2g range, 16384 LSB/g)
            float ax = mpu6050.accel_x / 16384.0;
            float ay = mpu6050.accel_y / 16384.0;
            float az = mpu6050.accel_z / 16384.0;

            sprintf(buf, "\rAccel: X=%+.2fg Y=%+.2fg Z=%+.2fg  ", ax, ay, az);
            puts_USART1(buf);

            // Temperature (340 LSB/°C, 0 at 36.53°C)
            float temp = (mpu6050.temperature / 340.0) + 36.53;
            sprintf(buf, "Temp: %.1f°C  ", temp);
            puts_USART1(buf);
        }

        // Read and display BMP180
        if (bmp180.present)
        {
            bmp180_read_temperature();
            sprintf(buf, "BMP: %.1f°C  ", bmp180.temperature / 10.0);
            puts_USART1(buf);
        }

        // Read and display HMC5883L
        if (hmc5883l.present)
        {
            hmc5883l_read();
            sprintf(buf, "Mag: X=%d Y=%d Z=%d",
                    hmc5883l.mag_x, hmc5883l.mag_y, hmc5883l.mag_z);
            puts_USART1(buf);
        }

        // Animate LED
        PORTC = (PORTC & 0xF0) | ((PORTC << 1) & 0x0F);
        if (!(PORTC & 0x0F))
            PORTC |= 0x01;

        _delay_ms(200);

        if (UCSR1A & (1 << RXC1))
        {
            getch_USART1();
            puts_USART1("\r\n\r\nStopped.\r\n");
            return;
        }
    }
}

/* ========================================================================
 * DEMO 3: Motion Detection (MPU6050)
 * ======================================================================== */
void demo3_motion_detection(void)
{
    puts_USART1("\r\n=== DEMO 3: Motion Detection ===\r\n");

    if (!mpu6050.present)
    {
        puts_USART1("MPU6050 not available!\r\n");
        puts_USART1("Press any key to continue...");
        getch_USART1();
        return;
    }

    puts_USART1("Monitoring for motion...\r\n");
    puts_USART1("Press any key to stop\r\n\r\n");

    // Read baseline
    mpu6050_read_all();
    int16_t baseline_x = mpu6050.accel_x;
    int16_t baseline_y = mpu6050.accel_y;
    int16_t baseline_z = mpu6050.accel_z;

    uint16_t motion_count = 0;

    while (1)
    {
        mpu6050_read_all();

        // Calculate differences
        int16_t diff_x = mpu6050.accel_x - baseline_x;
        int16_t diff_y = mpu6050.accel_y - baseline_y;
        int16_t diff_z = mpu6050.accel_z - baseline_z;

        // Simple threshold detection
        uint16_t magnitude = abs(diff_x) + abs(diff_y) + abs(diff_z);

        if (magnitude > 2000)
        { // Threshold
            motion_count++;
            char buf[80];
            sprintf(buf, "\rMotion detected! Count: %u  Magnitude: %u  ",
                    motion_count, magnitude);
            puts_USART1(buf);

            // Flash LED
            PORTC = 0xFF;
            _delay_ms(50);
            PORTC = 0x00;
        }
        else
        {
            puts_USART1("\rMonitoring...              ");
            PORTC = 0x01;
        }

        _delay_ms(50);

        if (UCSR1A & (1 << RXC1))
        {
            getch_USART1();
            puts_USART1("\r\n\r\nMotion detection stopped.\r\n");
            char buf[60];
            sprintf(buf, "Total motions detected: %u\r\n", motion_count);
            puts_USART1(buf);
            return;
        }
    }
}

/* ========================================================================
 * DEMO 4: Data Logging
 * ======================================================================== */
void demo4_data_logging(void)
{
    puts_USART1("\r\n=== DEMO 4: Sensor Data Logging ===\r\n");
    puts_USART1("Logging 20 samples at 1 Hz\r\n\r\n");

    puts_USART1("Time,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z,Temp_MPU,Temp_BMP,Mag_X,Mag_Y,Mag_Z\r\n");

    for (uint8_t i = 0; i < 20; i++)
    {
        // Read all sensors
        if (mpu6050.present)
            mpu6050_read_all();
        if (bmp180.present)
            bmp180_read_temperature();
        if (hmc5883l.present)
            hmc5883l_read();

        // Output CSV format
        char buf[150];
        sprintf(buf, "%u,%d,%d,%d,%d,%d,%d,%d,%ld,%d,%d,%d\r\n",
                i,
                mpu6050.present ? mpu6050.accel_x : 0,
                mpu6050.present ? mpu6050.accel_y : 0,
                mpu6050.present ? mpu6050.accel_z : 0,
                mpu6050.present ? mpu6050.gyro_x : 0,
                mpu6050.present ? mpu6050.gyro_y : 0,
                mpu6050.present ? mpu6050.gyro_z : 0,
                mpu6050.present ? mpu6050.temperature : 0,
                bmp180.present ? bmp180.temperature : 0,
                hmc5883l.present ? hmc5883l.mag_x : 0,
                hmc5883l.present ? hmc5883l.mag_y : 0,
                hmc5883l.present ? hmc5883l.mag_z : 0);
        puts_USART1(buf);

        // Toggle LED
        PORTC ^= 0xFF;

        _delay_ms(1000);
    }

    puts_USART1("\r\nLogging complete!\r\n");
    puts_USART1("Data can be copied to CSV file for analysis\r\n");

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
    puts_USART1("║   I2C Multi-Sensor - ATmega128        ║\r\n");
    puts_USART1("╚════════════════════════════════════════╝\r\n");
    puts_USART1("\r\n");
    puts_USART1("Select Demo:\r\n");
    puts_USART1("  [1] Sensor Discovery\r\n");
    puts_USART1("  [2] Real-Time Display\r\n");
    puts_USART1("  [3] Motion Detection\r\n");
    puts_USART1("  [4] Data Logging (CSV)\r\n");
    puts_USART1("\r\n");
    puts_USART1("Enter selection (1-4): ");
}

int main(void)
{
    // Initialize peripherals
    Uart1_init();
    i2c_init();

    // Configure status LEDs
    DDRC = 0xFF;
    PORTC = 0x00;

    // Send startup message
    _delay_ms(500);
    puts_USART1("\r\n\r\n*** I2C Multi-Sensor Interface ***\r\n");
    puts_USART1("Supports: MPU6050, BMP180, HMC5883L\r\n");

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
            demo1_sensor_discovery();
            break;
        case '2':
            demo2_realtime_display();
            break;
        case '3':
            demo3_motion_detection();
            break;
        case '4':
            demo4_data_logging();
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
