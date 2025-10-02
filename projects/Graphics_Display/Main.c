/*
 * Graphics Display - Educational Example
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Understand GLCD (Graphic LCD) operation
 * - Learn bitmap graphics programming
 * - Practice text and shape drawing
 * - Master display control algorithms
 *
 * HARDWARE SETUP:
 * - Connect GLCD to appropriate pins (see config.h)
 * - Ensure proper power supply and contrast adjustment
 * - Optional: buttons for user interaction
 */

#include "config.h"

int main(void)
{
    // Initialize system components
    init_devices(); // Initialize all peripherals

    // Initialize GLCD
    glcd_init(); // Initialize graphic LCD

    // Clear display and show startup message
    glcd_clear();
    glcd_write_string("ATmega128 GLCD", 0, 0);
    glcd_write_string("Graphics Demo", 0, 1);
    _delay_ms(2000);

    uint8_t frame = 0;
    uint8_t x, y;

    while (1)
    {
        // Clear display for new frame
        glcd_clear();

        // Draw title
        glcd_write_string("Frame: ", 0, 0);
        glcd_write_char('0' + (frame % 10), 42, 0);

        // Draw animated patterns based on frame number
        switch (frame % 4)
        {
        case 0:
            // Draw horizontal lines
            for (y = 2; y < 6; y++)
            {
                for (x = 0; x < 84; x += 2)
                {
                    glcd_set_pixel(x, y * 8);
                }
            }
            glcd_write_string("Horizontal", 0, 7);
            break;

        case 1:
            // Draw vertical lines
            for (x = 0; x < 84; x += 8)
            {
                for (y = 16; y < 48; y++)
                {
                    glcd_set_pixel(x, y);
                }
            }
            glcd_write_string("Vertical", 0, 7);
            break;

        case 2:
            // Draw diagonal pattern
            for (x = 0; x < 64; x++)
            {
                glcd_set_pixel(x, 16 + (x / 2));
                glcd_set_pixel(x, 32 - (x / 4));
            }
            glcd_write_string("Diagonal", 0, 7);
            break;

        case 3:
            // Draw rectangle outline
            for (x = 10; x < 74; x++)
            {
                glcd_set_pixel(x, 16); // Top
                glcd_set_pixel(x, 40); // Bottom
            }
            for (y = 16; y < 41; y++)
            {
                glcd_set_pixel(10, y); // Left
                glcd_set_pixel(74, y); // Right
            }
            glcd_write_string("Rectangle", 0, 7);
            break;
        }

        // Update display
        glcd_update();

        // Visual feedback on LEDs
        PORTB = ~(1 << (frame % 8));

        // Next frame
        frame++;
        _delay_ms(1000);
    }

    return 0;
}