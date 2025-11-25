/*
 * =============================================================================
 * ACCELEROMETER SENSOR - HANDS-ON LAB EXERCISES
 * =============================================================================
 * ATmega128 Educational Framework - Lab Session
 *
 * OBJECTIVE: Master accelerometer interfacing and motion sensing
 * DURATION: 90 minutes
 * DIFFICULTY: Advanced
 *
 * STUDENTS WILL:
 * - Interface with digital accelerometer sensors (ADXL345, MPU6050)
 * - Process 3-axis acceleration data and calculate motion parameters
 * - Implement motion detection and gesture recognition algorithms
 * - Create tilt sensing and orientation detection systems
 * - Build practical accelerometer-based applications
 *
 * HARDWARE REQUIRED:
 * - ATmega128 board
 * - ADXL345 digital accelerometer (I2C interface)
 * - Alternative: MPU6050 (accelerometer + gyroscope)
 * - I2C pull-up resistors (4.7kΩ)
 * - 8 LEDs for motion visualization
 * - Buzzer for motion alerts
 * - Push buttons for calibration
 *
 * ACCELEROMETER THEORY:
 * - Measures acceleration in 3 axes (X, Y, Z)
 * - Sensitivity: ±2g, ±4g, ±8g, ±16g ranges
 * - Applications: Tilt, motion, shock, vibration detection
 * - Data filtering and processing algorithms
 *
 * LAB STRUCTURE:
 * - Exercise 1: Sensor initialization and data acquisition (25 min)
 * - Exercise 2: Motion detection and threshold processing (25 min)
 * - Exercise 3: Tilt sensing and orientation detection (25 min)
 * - Exercise 4: Advanced motion applications (15 min)
 *
 * =============================================================================
 */

#include "config.h"
#include <math.h>

// ADXL345 I2C address and registers
#define ADXL345_ADDR 0x53
#define ADXL345_DEVID 0x00
#define ADXL345_POWER_CTL 0x2D
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_BW_RATE 0x2C
#define ADXL345_DATAX0 0x32
#define ADXL345_DATAY0 0x34
#define ADXL345_DATAZ0 0x36

// Motion detection constants
#define MOTION_THRESHOLD 100 // Motion detection threshold
#define GRAVITY_1G 256       // 1G in raw sensor units (±2g range)
#define TILT_THRESHOLD 200   // Tilt detection threshold
#define SHAKE_THRESHOLD 400  // Shake detection threshold

// Lab session variables
uint16_t lab_score = 0;
uint32_t motion_events = 0;
uint16_t calibration_samples = 0;

// Accelerometer data structure
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    float magnitude;
    float pitch;
    float roll;
} accel_data_t;

// Calibration offsets
int16_t offset_x = 0;
int16_t offset_y = 0;
int16_t offset_z = 0;

/*
 * =============================================================================
 * I2C AND SENSOR FUNCTIONS
 * =============================================================================
 */

void i2c_init(void)
{
    TWBR = 32;          // 100kHz I2C clock
    TWSR = 0x00;        // Prescaler = 1
    TWCR = (1 << TWEN); // Enable I2C
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

uint8_t adxl345_read_register(uint8_t reg)
{
    i2c_start();
    i2c_write((ADXL345_ADDR << 1) | 0x00); // Write mode
    i2c_write(reg);
    i2c_start();
    i2c_write((ADXL345_ADDR << 1) | 0x01); // Read mode
    uint8_t data = i2c_read_nack();
    i2c_stop();
    return data;
}

void adxl345_write_register(uint8_t reg, uint8_t data)
{
    i2c_start();
    i2c_write((ADXL345_ADDR << 1) | 0x00); // Write mode
    i2c_write(reg);
    i2c_write(data);
    i2c_stop();
}

uint8_t adxl345_init(void)
{
    i2c_init();

    // Check device ID
    uint8_t device_id = adxl345_read_register(ADXL345_DEVID);
    if (device_id != 0xE5)
    {
        return 0; // Wrong device
    }

    // Configure accelerometer
    adxl345_write_register(ADXL345_DATA_FORMAT, 0x00); // ±2g range, 10-bit resolution
    adxl345_write_register(ADXL345_BW_RATE, 0x0A);     // 100Hz data rate
    adxl345_write_register(ADXL345_POWER_CTL, 0x08);   // Measurement mode

    _delay_ms(100); // Allow sensor to settle
    return 1;       // Success
}

accel_data_t adxl345_read_data(void)
{
    accel_data_t data = {0, 0, 0, 0.0, 0.0, 0.0};

    // Read 6 bytes starting from DATAX0
    i2c_start();
    i2c_write((ADXL345_ADDR << 1) | 0x00);
    i2c_write(ADXL345_DATAX0);

    i2c_start();
    i2c_write((ADXL345_ADDR << 1) | 0x01);

    uint8_t x_low = i2c_read_ack();
    uint8_t x_high = i2c_read_ack();
    uint8_t y_low = i2c_read_ack();
    uint8_t y_high = i2c_read_ack();
    uint8_t z_low = i2c_read_ack();
    uint8_t z_high = i2c_read_nack();
    i2c_stop();

    // Combine bytes (little endian) and apply calibration
    data.x = (int16_t)((x_high << 8) | x_low) - offset_x;
    data.y = (int16_t)((y_high << 8) | y_low) - offset_y;
    data.z = (int16_t)((z_high << 8) | z_low) - offset_z;

    // Calculate magnitude
    data.magnitude = sqrt(data.x * data.x + data.y * data.y + data.z * data.z);

    // Calculate tilt angles (pitch and roll)
    data.pitch = atan2(data.y, sqrt(data.x * data.x + data.z * data.z)) * 180.0 / M_PI;
    data.roll = atan2(-data.x, data.z) * 180.0 / M_PI;

    return data;
}

/*
 * =============================================================================
 * MOTION VISUALIZATION FUNCTIONS
 * =============================================================================
 */

void visualize_motion_leds(accel_data_t data)
{
    // Map acceleration to LED patterns
    // LEDs 0-3 for X-axis (left/right), LEDs 4-7 for Y-axis (forward/back)

    // Clear all LEDs
    PORTB &= 0xF0;

    // X-axis visualization
    if (abs(data.x) > TILT_THRESHOLD)
    {
        if (data.x > 0)
        {
            PORTB |= 0x03; // LEDs 0,1 for positive X
        }
        else
        {
            PORTB |= 0x0C; // LEDs 2,3 for negative X
        }
    }

    // Y-axis visualization
    if (abs(data.y) > TILT_THRESHOLD)
    {
        if (data.y > 0)
        {
            PORTB |= 0x30; // LEDs 4,5 for positive Y
        }
        else
        {
            PORTB |= 0xC0; // LEDs 6,7 for negative Y
        }
    }
}

void motion_alert(uint8_t intensity)
{
    // Generate buzzer alert based on motion intensity
    for (uint8_t i = 0; i < intensity; i++)
    {
        PORTD |= (1 << 6); // Buzzer on
        _delay_ms(50);
        PORTD &= ~(1 << 6); // Buzzer off
        _delay_ms(50);
    }
}

/*
 * =============================================================================
 * LAB EXERCISE 1: SENSOR INITIALIZATION (25 minutes)
 * =============================================================================
 * OBJECTIVE: Initialize accelerometer and verify data acquisition
 * DIFFICULTY: ★★☆☆☆ (Easy-Medium)
 */

void lab_ex1_sensor_initialization(void)
{
    /*
     * CHALLENGE: Initialize ADXL345 and verify proper operation
     * TASK: Configure sensor settings and test data reading
     * LEARNING: I2C communication, sensor configuration, data validation
     */

    puts_USART1("\\r\\n=== Lab 1: Sensor Initialization ===\\r\\n");
    puts_USART1("Initializing ADXL345 accelerometer\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "ACCEL INIT");
    lcd_string(1, 0, "ADXL345 Setup");

    if (!adxl345_init())
    {
        puts_USART1("❌ ADXL345 not found!\\r\\n");
        lcd_string(3, 0, "Sensor not found!");
        return;
    }

    puts_USART1("✓ ADXL345 initialized successfully\\r\\n");
    lcd_string(3, 0, "Init successful!");

    // Verify communication with multiple reads
    puts_USART1("Verifying sensor communication...\\r\\n");

    for (uint8_t i = 0; i < 10; i++)
    {
        accel_data_t data = adxl345_read_data();

        char data_msg[60];
        sprintf(data_msg, "Read %d: X=%d, Y=%d, Z=%d, Mag=%.1f\\r\\n",
                i + 1, data.x, data.y, data.z, data.magnitude);
        puts_USART1(data_msg);

        // Display on LCD
        char lcd_msg[20];
        sprintf(lcd_msg, "X:%4d Y:%4d", data.x, data.y);
        lcd_string(4, 0, lcd_msg);

        sprintf(lcd_msg, "Z:%4d M:%.0f", data.z, data.magnitude);
        lcd_string(5, 0, lcd_msg);

        _delay_ms(500);
    }

    puts_USART1("Sensor communication verified!\\r\\n");
    lab_score += 100;
}

void lab_ex1_calibration(void)
{
    /*
     * CHALLENGE: Calibrate accelerometer for zero offset
     * TASK: Measure sensor bias and calculate calibration offsets
     * LEARNING: Sensor calibration, statistical averaging, offset correction
     */

    puts_USART1("\\r\\n=== Lab 1.2: Sensor Calibration ===\\r\\n");
    puts_USART1("Place sensor flat and level for calibration\\r\\n");
    puts_USART1("Press button when ready...\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "CALIBRATION");
    lcd_string(1, 0, "Place flat & level");
    lcd_string(3, 0, "Press button");

    while (!button_pressed(0))
    {
        _delay_ms(100);
    }

    puts_USART1("Calibrating... (100 samples)\\r\\n");
    lcd_string(3, 0, "Calibrating...");

    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    calibration_samples = 100;

    for (uint16_t i = 0; i < calibration_samples; i++)
    {
        accel_data_t data = adxl345_read_data();

        sum_x += data.x;
        sum_y += data.y;
        sum_z += data.z;

        if (i % 10 == 0)
        {
            char progress[20];
            sprintf(progress, "Sample: %d", i);
            lcd_string(4, 0, progress);
        }

        _delay_ms(50);
    }

    // Calculate offsets (Z should read ~+1g when flat)
    offset_x = sum_x / calibration_samples;
    offset_y = sum_y / calibration_samples;
    offset_z = (sum_z / calibration_samples) - GRAVITY_1G; // Subtract 1G for Z-axis

    char cal_msg[80];
    sprintf(cal_msg, "Calibration complete!\\r\\nOffsets: X=%d, Y=%d, Z=%d\\r\\n",
            offset_x, offset_y, offset_z);
    puts_USART1(cal_msg);

    lcd_string(3, 0, "Calibrated!");
    char offset_msg[20];
    sprintf(offset_msg, "X:%d Y:%d Z:%d", offset_x, offset_y, offset_z);
    lcd_string(4, 0, offset_msg);

    // Test calibration
    puts_USART1("Testing calibration...\\r\\n");
    _delay_ms(2000);

    for (uint8_t i = 0; i < 5; i++)
    {
        accel_data_t data = adxl345_read_data();

        char test_msg[60];
        sprintf(test_msg, "Cal Test %d: X=%d, Y=%d, Z=%d\\r\\n",
                i + 1, data.x, data.y, data.z);
        puts_USART1(test_msg);

        _delay_ms(1000);
    }

    lab_score += 150;
}

/*
 * =============================================================================
 * LAB EXERCISE 2: MOTION DETECTION (25 minutes)
 * =============================================================================
 * OBJECTIVE: Implement motion detection and threshold processing
 * DIFFICULTY: ★★★☆☆ (Medium)
 */

void lab_ex2_motion_detection(void)
{
    /*
     * CHALLENGE: Detect motion events using acceleration thresholds
     * TASK: Implement motion detection with configurable sensitivity
     * LEARNING: Threshold processing, motion algorithms, event detection
     */

    puts_USART1("\\r\\n=== Lab 2: Motion Detection ===\\r\\n");
    puts_USART1("Move the accelerometer to trigger motion events\\r\\n");
    puts_USART1("Press button to stop monitoring...\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "MOTION DETECTOR");
    lcd_string(1, 0, "Move to trigger");

    // Initialize LED pins
    DDRB |= 0xFF;     // All LEDs as outputs
    DDRD |= (1 << 6); // Buzzer pin as output

    accel_data_t baseline = adxl345_read_data();
    uint16_t motion_count = 0;
    uint16_t readings = 0;

    while (!button_pressed(0) && readings < 500)
    {
        accel_data_t current = adxl345_read_data();
        readings++;

        // Calculate motion magnitude
        int16_t delta_x = current.x - baseline.x;
        int16_t delta_y = current.y - baseline.y;
        int16_t delta_z = current.z - baseline.z;

        float motion_magnitude = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);

        // Display current data
        char data_msg[20];
        sprintf(data_msg, "X:%4d Y:%4d", current.x, current.y);
        lcd_string(3, 0, data_msg);

        sprintf(data_msg, "Z:%4d M:%.0f", current.z, motion_magnitude);
        lcd_string(4, 0, data_msg);

        // Motion detection
        if (motion_magnitude > MOTION_THRESHOLD)
        {
            motion_count++;
            motion_events++;

            char motion_msg[50];
            sprintf(motion_msg, "Motion #%d: Magnitude=%.1f\\r\\n", motion_count, motion_magnitude);
            puts_USART1(motion_msg);

            // Visual and audio feedback
            visualize_motion_leds(current);

            if (motion_magnitude > SHAKE_THRESHOLD)
            {
                lcd_string(5, 0, "*** SHAKE! ***");
                motion_alert(3); // Strong alert
            }
            else
            {
                lcd_string(5, 0, "* Motion *");
                motion_alert(1); // Gentle alert
            }

            // Update baseline after motion
            baseline = current;
            _delay_ms(500); // Debounce
        }
        else
        {
            lcd_string(5, 0, "Stable");
            PORTB &= 0xF0; // Clear motion LEDs
        }

        _delay_ms(100);
    }

    char summary[60];
    sprintf(summary, "\\r\\nMotion detection complete: %d events in %d readings\\r\\n",
            motion_count, readings);
    puts_USART1(summary);

    if (motion_count >= 5)
    {
        lab_score += 150;
        puts_USART1("✓ Motion detection working!\\r\\n");
    }
}

/*
 * =============================================================================
 * LAB EXERCISE 3: TILT SENSING (25 minutes)
 * =============================================================================
 * OBJECTIVE: Implement tilt and orientation detection
 * DIFFICULTY: ★★★★☆ (Medium-Hard)
 */

void lab_ex3_tilt_sensing(void)
{
    /*
     * CHALLENGE: Detect tilt angles and orientation changes
     * TASK: Calculate pitch and roll angles, detect orientation
     * LEARNING: Trigonometry, angle calculation, orientation algorithms
     */

    puts_USART1("\\r\\n=== Lab 3: Tilt Sensing ===\\r\\n");
    puts_USART1("Tilt the accelerometer to see orientation\\r\\n");
    puts_USART1("Press button to stop monitoring...\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "TILT SENSOR");
    lcd_string(1, 0, "Orientation detect");

    uint16_t tilt_readings = 0;
    uint8_t orientation_changes = 0;
    uint8_t last_orientation = 0; // 0=flat, 1=left, 2=right, 3=forward, 4=back

    while (!button_pressed(0) && tilt_readings < 300)
    {
        accel_data_t data = adxl345_read_data();
        tilt_readings++;

        // Display angles
        char angle_msg[20];
        sprintf(angle_msg, "Pitch: %+.1f°", data.pitch);
        lcd_string(3, 0, angle_msg);

        sprintf(angle_msg, "Roll:  %+.1f°", data.roll);
        lcd_string(4, 0, angle_msg);

        // Determine orientation
        uint8_t current_orientation = 0;
        const char *orientation_names[] = {"Flat", "Left", "Right", "Forward", "Back", "Upside"};

        if (abs(data.z) < 50)
        {                            // Nearly vertical
            current_orientation = 5; // Upside down or vertical
        }
        else if (abs(data.pitch) < 15 && abs(data.roll) < 15)
        {
            current_orientation = 0; // Flat
        }
        else if (data.roll > 30)
        {
            current_orientation = 1; // Tilted left
        }
        else if (data.roll < -30)
        {
            current_orientation = 2; // Tilted right
        }
        else if (data.pitch > 30)
        {
            current_orientation = 3; // Tilted forward
        }
        else if (data.pitch < -30)
        {
            current_orientation = 4; // Tilted back
        }

        // Check for orientation change
        if (current_orientation != last_orientation)
        {
            orientation_changes++;

            char orient_msg[50];
            sprintf(orient_msg, "Orientation change #%d: %s\\r\\n",
                    orientation_changes, orientation_names[current_orientation]);
            puts_USART1(orient_msg);

            last_orientation = current_orientation;
        }

        lcd_string(5, 0, orientation_names[current_orientation]);

        // LED pattern based on orientation
        PORTB &= 0xF0; // Clear LEDs
        switch (current_orientation)
        {
        case 1:
            PORTB |= 0x01;
            break; // Left - LED 0
        case 2:
            PORTB |= 0x02;
            break; // Right - LED 1
        case 3:
            PORTB |= 0x04;
            break; // Forward - LED 2
        case 4:
            PORTB |= 0x08;
            break; // Back - LED 3
        case 0:
            PORTB |= 0x0F;
            break; // Flat - All LEDs
        case 5:
            PORTB |= 0xF0;
            break; // Upside - Different pattern
        }

        // Alert for extreme tilt
        if (abs(data.pitch) > 60 || abs(data.roll) > 60)
        {
            motion_alert(2);
        }

        _delay_ms(200);
    }

    char tilt_summary[60];
    sprintf(tilt_summary, "\\r\\nTilt sensing complete: %d orientation changes\\r\\n",
            orientation_changes);
    puts_USART1(tilt_summary);

    if (orientation_changes >= 3)
    {
        lab_score += 200;
        puts_USART1("✓ Tilt sensing working!\\r\\n");
    }
}

/*
 * =============================================================================
 * LAB EXERCISE 4: ADVANCED APPLICATIONS (15 minutes)
 * =============================================================================
 * OBJECTIVE: Build practical accelerometer applications
 * DIFFICULTY: ★★★★★ (Hard)
 */

void lab_ex4_gesture_recognition(void)
{
    /*
     * CHALLENGE: Implement simple gesture recognition
     * TASK: Detect specific motion patterns and gestures
     * LEARNING: Pattern recognition, signal processing, gesture algorithms
     */

    puts_USART1("\\r\\n=== Lab 4: Gesture Recognition ===\\r\\n");
    puts_USART1("Perform gestures to test recognition\\r\\n");
    puts_USART1("Gestures: Shake, Tap, Flip\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "GESTURE RECOG");
    lcd_string(1, 0, "Shake, Tap, Flip");

    uint8_t gestures_detected = 0;
    uint16_t gesture_readings = 0;

    // Gesture detection variables
    accel_data_t gesture_buffer[10];
    uint8_t buffer_index = 0;

    while (gestures_detected < 10 && gesture_readings < 400)
    {
        accel_data_t data = adxl345_read_data();
        gesture_readings++;

        // Store in circular buffer
        gesture_buffer[buffer_index] = data;
        buffer_index = (buffer_index + 1) % 10;

        // Display current acceleration
        char accel_msg[20];
        sprintf(accel_msg, "X:%4d Y:%4d", data.x, data.y);
        lcd_string(3, 0, accel_msg);

        sprintf(accel_msg, "Z:%4d M:%.0f", data.z, data.magnitude);
        lcd_string(4, 0, accel_msg);

        // Gesture 1: Shake detection (rapid back-and-forth motion)
        if (data.magnitude > SHAKE_THRESHOLD)
        {
            // Check for oscillating pattern in buffer
            uint8_t shake_count = 0;
            for (uint8_t i = 1; i < 10; i++)
            {
                if (abs(gesture_buffer[i].x - gesture_buffer[i - 1].x) > 200)
                {
                    shake_count++;
                }
            }

            if (shake_count >= 5)
            {
                puts_USART1("🤝 SHAKE gesture detected!\\r\\n");
                lcd_string(5, 0, "SHAKE detected!");
                motion_alert(4);
                gestures_detected++;
                _delay_ms(1000); // Debounce
            }
        }

        // Gesture 2: Tap detection (sharp acceleration spike)
        if (data.magnitude > 600 && data.magnitude < 1000)
        {
            puts_USART1("👆 TAP gesture detected!\\r\\n");
            lcd_string(5, 0, "TAP detected!");
            motion_alert(2);
            gestures_detected++;
            _delay_ms(1000); // Debounce
        }

        // Gesture 3: Flip detection (Z-axis inversion)
        static int16_t last_z = 0;
        if (last_z != 0 && ((last_z > 200 && data.z < -200) || (last_z < -200 && data.z > 200)))
        {
            puts_USART1("🔄 FLIP gesture detected!\\r\\n");
            lcd_string(5, 0, "FLIP detected!");
            motion_alert(3);
            gestures_detected++;
            _delay_ms(1500); // Longer debounce for flip
        }
        last_z = data.z;

        // Update gesture counter
        char gesture_msg[20];
        sprintf(gesture_msg, "Gestures: %d", gestures_detected);
        lcd_string(2, 0, gesture_msg);

        _delay_ms(100);
    }

    char final_msg[60];
    sprintf(final_msg, "\\r\\nGesture recognition complete! Detected: %d gestures\\r\\n",
            gestures_detected);
    puts_USART1(final_msg);

    if (gestures_detected >= 5)
    {
        lab_score += 250;
        puts_USART1("✓ Gesture recognition mastered!\\r\\n");
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
    puts_USART1("   ACCELEROMETER SENSOR - LAB EXERCISES      \\r\\n");
    puts_USART1("==============================================\\r\\n");
    puts_USART1("1. Sensor Initialization & Calibration      \\r\\n");
    puts_USART1("2. Motion Detection & Threshold Processing   \\r\\n");
    puts_USART1("3. Tilt Sensing & Orientation Detection     \\r\\n");
    puts_USART1("4. Advanced Gesture Recognition             \\r\\n");
    puts_USART1("                                              \\r\\n");
    puts_USART1("0. Run All Exercises                         \\r\\n");
    puts_USART1("X. Exit Lab                                   \\r\\n");
    puts_USART1("==============================================\\r\\n");
    char score_msg[50];
    sprintf(score_msg, "Current Score: %d points\\r\\n", lab_score);
    puts_USART1(score_msg);
    char event_msg[50];
    sprintf(event_msg, "Motion Events: %ld\\r\\n", motion_events);
    puts_USART1(event_msg);
    puts_USART1("Select exercise (1-4, 0, X): ");
}

int main(void)
{
    init_devices();

    puts_USART1("\\r\\n*** ACCELEROMETER SENSOR LAB SESSION ***\\r\\n");
    puts_USART1("Welcome to hands-on accelerometer programming!\\r\\n");
    puts_USART1("Ensure ADXL345 is connected via I2C with pull-ups\\r\\n");

    lcd_clear();
    lcd_string(1, 0, "ACCELEROMETER LAB");
    lcd_string(2, 0, "Check I2C wiring");
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
            lab_ex1_sensor_initialization();
            lab_ex1_calibration();
            break;

        case '2':
            lab_ex2_motion_detection();
            break;

        case '3':
            lab_ex3_tilt_sensing();
            break;

        case '4':
            lab_ex4_gesture_recognition();
            break;

        case '0':
            puts_USART1("\\r\\n*** RUNNING ALL EXERCISES ***\\r\\n");
            lab_ex1_sensor_initialization();
            lab_ex1_calibration();
            lab_ex2_motion_detection();
            lab_ex3_tilt_sensing();
            lab_ex4_gesture_recognition();

            char final_buffer[80];
            sprintf(final_buffer, "\\r\\n*** ALL EXERCISES COMPLETE! ***\\r\\nFinal Score: %d points\\r\\n", lab_score);
            puts_USART1(final_buffer);
            break;

        case 'X':
        case 'x':
            puts_USART1("\\r\\nExiting lab. Great work on accelerometer!\\r\\n");
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