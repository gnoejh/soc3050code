/*
 * =============================================================================
 * ACCELEROMETER SENSOR INTERFACE - EDUCATIONAL DEMONSTRATION
 * =============================================================================
 *
 * PROJECT: Accelerometer
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Educational demonstration of 3-axis accelerometer interfacing and motion detection.
 * Students learn multi-channel sensor processing and motion analysis algorithms.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master 3-axis accelerometer operation and calibration
 * 2. Learn multi-channel ADC reading techniques
 * 3. Practice motion detection and threshold algorithms
 * 4. Implement real-time sensor data processing
 * 5. Analyze acceleration vectors and orientation
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - 3-axis analog accelerometer (ADXL335 or similar)
 * - X-axis connected to ADC2, Y-axis to ADC3, Z-axis to ADC4
 * - LEDs on PORTB for motion indication
 * - Serial connection for data analysis (9600 baud)
 *
 * LEARNING PROGRESSION:
 * - Demo 1: Single Axis Acceleration Reading
 * - Demo 2: 3-Axis Vector Calculation
 * - Demo 3: Motion Detection and Thresholds
 * - Demo 4: Orientation Determination
 * - Demo 5: Real-Time Motion Analysis
 *
 * =============================================================================
 */
 * - UART for data logging
 */

#include "config.h"

int main(void)
 {
     // Initialize system components
     init_devices(); // Initialize all peripherals

     // Initialize UART for data output
     Uart1_init(); // 9600 baud serial communication

     // Send startup message
     puts_USART1("3-Axis Accelerometer Started\r\n");
     puts_USART1("Reading X, Y, Z acceleration values...\r\n");

     uint16_t x_axis, y_axis, z_axis;
     uint16_t x_prev = 512, y_prev = 512, z_prev = 512; // Assume 512 = center
     char buffer[80];
     uint8_t motion_detected = 0;

     while (1)
     {
         // Read accelerometer values from ADC channels 2, 3, 4
         x_axis = Adc_read_ch(2); // X-axis
         y_axis = Adc_read_ch(3); // Y-axis
         z_axis = Adc_read_ch(4); // Z-axis

         // Detect motion (simple threshold-based)
         if (abs(x_axis - x_prev) > 50 ||
             abs(y_axis - y_prev) > 50 ||
             abs(z_axis - z_prev) > 50)
         {
             motion_detected = 1;
             PORTB = 0x00; // Turn on all LEDs (active LOW)
         }
         else
         {
             motion_detected = 0;
             PORTB = 0xFF; // Turn off all LEDs
         }

         // Send data via UART
         sprintf(buffer, "X:%u Y:%u Z:%u Motion:%s\r\n",
                 x_axis, y_axis, z_axis,
                 motion_detected ? "YES" : "NO");
         puts_USART1(buffer);

         // Analyze orientation (simplified)
         if (z_axis > 700)
         {
             puts_USART1("Orientation: FACE UP\r\n");
         }
         else if (z_axis < 300)
         {
             puts_USART1("Orientation: FACE DOWN\r\n");
         }
         else if (x_axis > 700)
         {
             puts_USART1("Orientation: TILTED RIGHT\r\n");
         }
         else if (x_axis < 300)
         {
             puts_USART1("Orientation: TILTED LEFT\r\n");
         }
         else
         {
             puts_USART1("Orientation: LEVEL\r\n");
         }

         // Store previous values for motion detection
         x_prev = x_axis;
         y_prev = y_axis;
         z_prev = z_axis;

         // Wait before next reading
         _delay_ms(200);
     }

     return 0;
 }