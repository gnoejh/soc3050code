/*
 * Task Template for RTOS ATmega128
 *
 * Use this template to create your own tasks.
 * Copy this code and modify it for your needs.
 */

// =============================================================================
// TASK TEMPLATE - Copy and modify this
// =============================================================================

// Task X: [Your Task Name]
// Description: [What does this task do?]
// Hardware: [Which pins/peripherals used?]
// Update Rate: [How often does it run? e.g., 100ms]
void task_template(void)
{
    // Static variables keep their values between task executions
    static uint32_t last_update = 0; // Timing control
    static uint8_t counter = 0;      // Example state variable
    static bool led_state = false;   // Example flag

    // Timing check - run this code every 100ms (100 ticks)
    if ((system_ticks - last_update) >= 100)
    {
        // ===== YOUR CODE HERE =====

        // Example 1: Toggle an LED
        if (led_state)
        {
            PORTB |= (1 << PB0); // LED OFF (active-LOW)
        }
        else
        {
            PORTB &= ~(1 << PB0); // LED ON (active-LOW)
        }
        led_state = !led_state;

        // Example 2: Read a sensor
        uint16_t sensor_value = adc_read(0); // Read ADC channel 0

        // Example 3: Send data via UART
        uart_puts("[MyTask] Count: ");
        uart_print_num(counter);
        uart_puts(" | Sensor: ");
        uart_print_num(sensor_value);
        uart_puts("\r\n");

        // Example 4: Update GLCD
        ks0108_set_cursor(6, 0);
        ks0108_puts("MyTask:");
        ks0108_putchar('0' + (counter / 10));
        ks0108_putchar('0' + (counter % 10));

        // Increment counter
        counter++;
        if (counter > 99)
            counter = 0;

        // ===== END YOUR CODE =====

        // Update timing - IMPORTANT: Don't forget this!
        last_update = system_ticks;
    }
}

// =============================================================================
// TO USE THIS TASK:
// =============================================================================
// 1. Copy this function to Main.c (before main() function)
// 2. Rename the function (e.g., task_temperature, task_alarm, etc.)
// 3. Modify the code inside the timing check
// 4. In main(), add this line in the task initialization section:
//
//    rtos_create_task(task_template, "Template", PRIORITY_NORMAL, true);
//    uart_puts("[OK] Task 10: Template\r\n");
//
// 5. If needed, increase MAX_TASKS at the top of Main.c
// 6. Build and test!
// =============================================================================

// =============================================================================
// COMMON PATTERNS
// =============================================================================

// Pattern 1: Button Input with Debouncing
void task_button_example(void)
{
    static uint8_t last_button_state = 0xFF;
    static uint32_t last_debounce = 0;

    uint8_t current_state = PIND;

    if ((system_ticks - last_debounce) >= 50) // 50ms debounce
    {
        // Check if button 0 was pressed (falling edge)
        if ((last_button_state & (1 << PD0)) && !(current_state & (1 << PD0)))
        {
            uart_puts("Button 0 pressed!\r\n");
            buzzer_beep(50);
        }

        last_button_state = current_state;
        last_debounce = system_ticks;
    }
}

// Pattern 2: ADC Reading and Processing
void task_adc_example(void)
{
    static uint32_t last_read = 0;

    if ((system_ticks - last_read) >= 500) // Every 500ms
    {
        uint16_t raw = adc_read(0);            // Read ADC (0-1023)
        uint16_t percent = (raw * 100) / 1023; // Convert to percentage
        uint16_t voltage = (raw * 500) / 1023; // Convert to mV (0-5000mV)

        uart_puts("ADC: ");
        uart_print_num(raw);
        uart_puts(" (");
        uart_print_num(percent);
        uart_puts("%)\r\n");

        last_read = system_ticks;
    }
}

// Pattern 3: LED Pattern Generator
void task_led_pattern_example(void)
{
    static uint32_t last_update = 0;
    static uint8_t step = 0;

    if ((system_ticks - last_update) >= 100) // Every 100ms
    {
        // Create different patterns based on step
        switch (step)
        {
        case 0:
            PORTC = 0xFE;
            break; // 11111110
        case 1:
            PORTC = 0xFD;
            break; // 11111101
        case 2:
            PORTC = 0xFB;
            break; // 11111011
        case 3:
            PORTC = 0xF7;
            break; // 11110111
        case 4:
            PORTC = 0xEF;
            break; // 11101111
        case 5:
            PORTC = 0xDF;
            break; // 11011111
        case 6:
            PORTC = 0xBF;
            break; // 10111111
        case 7:
            PORTC = 0x7F;
            break; // 01111111
        }

        step = (step + 1) % 8;
        last_update = system_ticks;
    }
}

// Pattern 4: State Machine
void task_state_machine_example(void)
{
    typedef enum
    {
        STATE_IDLE,
        STATE_ACTIVE,
        STATE_ALARM,
        STATE_COOLDOWN
    } State;

    static State current_state = STATE_IDLE;
    static uint32_t state_timer = 0;

    switch (current_state)
    {
    case STATE_IDLE:
        // Wait for trigger
        if (!(PIND & (1 << PD0))) // Button pressed
        {
            current_state = STATE_ACTIVE;
            state_timer = system_ticks;
            uart_puts("State: ACTIVE\r\n");
        }
        break;

    case STATE_ACTIVE:
        // Blink LED for 3 seconds
        if ((system_ticks % 200) < 100)
            PORTB &= ~(1 << PB0); // LED ON
        else
            PORTB |= (1 << PB0); // LED OFF

        if ((system_ticks - state_timer) >= 3000)
        {
            current_state = STATE_ALARM;
            state_timer = system_ticks;
            uart_puts("State: ALARM\r\n");
        }
        break;

    case STATE_ALARM:
        // Sound buzzer and flash LED
        buzzer_beep(100);
        current_state = STATE_COOLDOWN;
        state_timer = system_ticks;
        break;

    case STATE_COOLDOWN:
        // Wait 2 seconds
        if ((system_ticks - state_timer) >= 2000)
        {
            current_state = STATE_IDLE;
            uart_puts("State: IDLE\r\n");
        }
        break;
    }
}

// Pattern 5: Moving Average Filter
void task_filter_example(void)
{
    static uint32_t last_read = 0;
    static uint16_t samples[10] = {0};
    static uint8_t sample_index = 0;

    if ((system_ticks - last_read) >= 100) // Every 100ms
    {
        // Read new sample
        samples[sample_index] = adc_read(0);
        sample_index = (sample_index + 1) % 10;

        // Calculate average
        uint32_t sum = 0;
        for (uint8_t i = 0; i < 10; i++)
        {
            sum += samples[i];
        }
        uint16_t average = sum / 10;

        uart_puts("Filtered: ");
        uart_print_num(average);
        uart_puts("\r\n");

        last_read = system_ticks;
    }
}

// =============================================================================
// HARDWARE REFERENCE QUICK GUIDE
// =============================================================================
/*
PORT ASSIGNMENTS (Simulator110.simu):
- Port B (PORTB): LEDs (PB0-PB7, active-LOW)
  - PB0-PB6: LED sequence
  - PB7: Heartbeat LED
  - PB5: PWM output (Timer1 OC1A)

- Port D (PORTD): Buttons (PD0-PD7, active-LOW with pull-ups)
  - PD0-PD7: 8 push buttons
  - PD2: UART1 RX
  - PD3: UART1 TX

- Port F (PORTF): ADC inputs
  - PF0: Potentiometer/ADC Channel 0
  - PF1-PF7: Additional ADC channels

- Port G (PORTG):
  - PG4: Buzzer output

TIMER USAGE:
- Timer0: RTOS system tick (1ms)
- Timer1: PWM motor control (available)
- Timer2: Available
- Timer3: Available

ADC:
- 8 channels (0-7)
- 10-bit resolution (0-1023)
- Reference: AVcc (5V)

UART1:
- 9600 baud
- 8 data bits, 1 stop bit, no parity

GLCD (KS0108):
- 128x64 pixels
- 8 lines, 21 columns (text mode)
- Connected via Port A (data), Port E (control)
*/
