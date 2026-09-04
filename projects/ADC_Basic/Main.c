/*
 * ==============================================================================
 * ADC MULTI-SENSOR - DEMO CODE WITH GLCD DISPLAY
 * ==============================================================================
 * PROJECT: ADC_Basic
 * See Slide.md for complete ADC theory and technical details
 *
 * DEMOS: ADC reading with multiple sensors (Potentiometer, Joystick)
 * DISPLAY: KS0108 GLCD (128x64) for sensor data visualization
 * FORMAT: Matches concise educational style used in Motor_Stepper
 * ==============================================================================
 */

#include "config.h"
#include <stdlib.h> // For abs()

// --- Hardware Mapping
// --------------------------------------------------------- ADC0 (PORTF0):
// Potentiometer - variable voltage input ADC3 (PORTF3): KY023 Joystick VRX
// (X-axis) ADC4 (PORTF4): KY023 Joystick VRY (Y-axis) PORTD0: KY023 Joystick SW
// (switch button, digital input) GLCD: KS0108 128x64 connected via PORTA
// (data), PORTE (control)

#define ADC_CH_POT 0   // Potentiometer on ADC0
#define ADC_CH_JOY_X 3 // Joystick X-axis on ADC3
#define ADC_CH_JOY_Y 4 // Joystick Y-axis on ADC4

#define JOY_SW_PIN (1 << PD0) // Joystick switch button

// ADC specifications
#define ADC_MAX_VALUE 1023      // 10-bit ADC maximum value
#define ADC_REF_VOLTAGE_mV 5000 // AVCC reference = 5V = 5000mV
#define ADC_LSB_mV 4.883f       // LSB = 5000mV / 1024 = 4.883mV

// Joystick calibration (typical center values)
#define JOY_CENTER_X 512
#define JOY_CENTER_Y 512
#define JOY_DEADZONE 50 // Dead zone around center

// GLCD display layout
#define GLCD_BAR_X 70      // X position for bar graphs
#define GLCD_BAR_WIDTH 50  // Width of bar graph
#define GLCD_BAR_HEIGHT 50 // Height of bar graph

/*
 * Initialize ADC, sensors, and GLCD
 */
void adc_sensors_init(void)
{
  // Use init_devices() like Graphics_Display and legacy code
  init_devices();

  lcd_clear();
  lcd_string(0, 0, "ADC Sensor System");
  lcd_string(1, 0, "Initializing...");
  _delay_ms(500);

  // Initialize ADC using library
  Adc_init();

  // Configure joystick switch as input with pull-up
  DDRD &= ~JOY_SW_PIN; // Input
  PORTD |= JOY_SW_PIN; // Enable pull-up

  lcd_clear();
  lcd_string(0, 0, "ADC System Ready");
  _delay_ms(1000);
}

/*
 * Read potentiometer value (0-1023)
 */
uint16_t read_potentiometer(void) { return Read_Adc_Data(ADC_CH_POT); }

/*
 * Read potentiometer as percentage (0-100%)
 */
uint8_t read_potentiometer_percent(void)
{
  uint16_t raw = read_potentiometer();
  return (uint8_t)((raw * 100UL) / ADC_MAX_VALUE);
}

/*
 * Read potentiometer voltage in millivolts
 */
uint16_t read_potentiometer_voltage_mV(void)
{
  uint16_t raw = read_potentiometer();
  return (uint16_t)((raw * ADC_REF_VOLTAGE_mV) / ADC_MAX_VALUE);
}

/*
 * Read joystick X-axis (0-1023)
 */
uint16_t read_joystick_x(void) { return Read_Adc_Data(ADC_CH_JOY_X); }

/*
 * Read joystick Y-axis (0-1023)
 */
uint16_t read_joystick_y(void) { return Read_Adc_Data(ADC_CH_JOY_Y); }

/*
 * Read joystick switch state
 * Returns 1 if pressed, 0 if released (active low with pull-up)
 */
uint8_t read_joystick_switch(void)
{
  return (PIND & JOY_SW_PIN) ? 0 : 1; // Inverted: LOW = pressed
}

/*
 * Get joystick direction
 * Returns: 0=center, 1=up, 2=down, 3=left, 4=right
 */
uint8_t get_joystick_direction(void)
{
  uint16_t x = read_joystick_x();
  uint16_t y = read_joystick_y();

  int16_t dx = (int16_t)x - JOY_CENTER_X;
  int16_t dy = (int16_t)y - JOY_CENTER_Y;

  // Check dead zone
  if (abs(dx) < JOY_DEADZONE && abs(dy) < JOY_DEADZONE)
  {
    return 0; // Center
  }

  // Determine direction based on larger component
  if (abs(dx) > abs(dy))
  {
    return (dx > 0) ? 4 : 3; // Right or Left
  }
  else
  {
    return (dy > 0) ? 2 : 1; // Down or Up
  }
}

/*
 * Read all ADC channels and return in array
 */
void read_all_adc_channels(uint16_t *results, uint8_t num_channels)
{
  for (uint8_t i = 0; i < num_channels; i++)
  {
    results[i] = Read_Adc_Data(i);
  }
}

/*
 * Draw vertical bar graph on GLCD
 * x: X position (0-127)
 * y: Y position (top of bar, 0-63)
 * height: Bar height in pixels (0-63)
 * width: Bar width in pixels
 */
void glcd_draw_bar(uint8_t x, uint8_t y, uint8_t height, uint8_t width)
{
  if (height == 0)
    return;
  ks0108_fill_rect(x, y - height, width, height, KS0108_PIXEL_ON);
}

/*
 * Clear bar graph area
 */
void glcd_clear_bar_area(uint8_t x, uint8_t y, uint8_t width, uint8_t height)
{
  ks0108_fill_rect(x, y - height, width, height, KS0108_PIXEL_OFF);
}

/*
 * Draw horizontal bar graph (for joystick visualization)
 */
void glcd_draw_hbar(uint8_t x, uint8_t y, uint8_t length, uint8_t height)
{
  if (length == 0)
    return;
  ks0108_fill_rect(x, y, length, height, KS0108_PIXEL_ON);
}

/* ========================================================================
 * HELPER: Display number without printf (avoids ELPM issues)
 * ======================================================================== */
void display_number(uint8_t row, uint8_t col, uint16_t value)
{
  char buffer[6];
  uint8_t i = 0;

  // Convert to string manually (avoid sprintf/printf)
  if (value == 0)
  {
    buffer[i++] = '0';
  }
  else
  {
    uint16_t temp = value;
    uint8_t digits[5];
    uint8_t num_digits = 0;

    while (temp > 0)
    {
      digits[num_digits++] = temp % 10;
      temp /= 10;
    }

    // Reverse digits
    for (int8_t j = num_digits - 1; j >= 0; j--)
    {
      buffer[i++] = digits[j] + '0';
    }
  }

  buffer[i] = '\0';
  lcd_string(row, col, buffer);
}

/* ========================================================================
 * DEMO 1: Basic Potentiometer Reading with GLCD
 * ======================================================================== */
void demo1_potentiometer_basic(void)
{
  // Draw static page layout once using new API like Graphics_Display
  ks0108_clear_screen();
  ScreenBuffer_clear();

  // Configure PORTF0 as input
  DDRF &= ~(1 << PF0);  // Set PF0 as input
  PORTF &= ~(1 << PF0); // Disable pull-up

  // Initialize ADC with external AREF (12V)
  // NOTE: AREF pin (62) connected to 12V in SimulIDE
  // 1kΩ potentiometer on ADC0
  ADCSRA = 0x00; // Disable ADC
  ADMUX = 0x00;  // External AREF, right-adjust, ADC0
  ACSR = 0x80;   // Disable analog comparator
  ADCSRA = 0x87; // Enable ADC, prescaler=128 (16MHz/128=125kHz)

  _delay_ms(100); // Wait for reference to stabilize

  // Dummy conversion
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC))
    ;

  // Title with decorative line
  ks0108_puts_at(0, 0, "=== ADC Demo ===");

  // Draw labels with better formatting
  ks0108_puts_at(2, 0, "ADC Value:");
  ks0108_puts_at(3, 0, "Voltage  :");
  ks0108_puts_at(4, 0, "Percent  :");

  // Draw unit labels
  ks0108_puts_at(3, 16, "mV");
  ks0108_puts_at(4, 16, "%");

  while (1)
  {
    // Read ADC channel 0 (potentiometer)
    ADMUX = 0x00;          // External AREF (12V), right-adjust, ADC0
    ADCSRA |= (1 << ADSC); // Start conversion
    while (ADCSRA & (1 << ADSC))
      ; // Wait for conversion complete

    uint8_t low = ADCL;
    uint8_t high = ADCH;
    uint16_t adc_value = low | (high << 8);

    // Calculate voltage in mV (12V reference, 10-bit ADC)
    uint32_t voltage_mV = (adc_value * 12000UL) / 1024;

    // Calculate percentage (0-100%)
    uint16_t percentage = (adc_value * 100UL) / 1023;

    // Calculate voltage in Volts with 2 decimal places
    uint16_t volts = voltage_mV / 1000;
    uint16_t millivolts = voltage_mV % 1000;

    // Display ADC value (0-1023)
    display_number(2, 11, adc_value);

    // Display voltage in V.VV format
    display_number(3, 11, volts);
    ks0108_puts_at(3, 13, ".");
    display_number(3, 14, millivolts / 100);       // First decimal
    display_number(3, 15, (millivolts / 10) % 10); // Second decimal

    // Display percentage
    display_number(4, 11, percentage);

    _delay_ms(200); // Faster update for smoother display
  }
} /* ========================================================================
   * DEMO 2: Joystick Position Reading with GLCD
   * ======================================================================== */
void demo2_joystick_reading(void)
{
  ks0108_clear_screen();
  ScreenBuffer_clear();

  // Title with decorative line
  ks0108_puts_at(0, 0, "== Joystick ==");

  // Labels with better alignment
  ks0108_puts_at(2, 0, "X-Axis  :");
  ks0108_puts_at(3, 0, "Y-Axis  :");
  ks0108_puts_at(4, 0, "Switch  :");
  ks0108_puts_at(5, 0, "Direction:");

  for (uint16_t i = 0; i < 100; i++)
  {
    uint16_t x = read_joystick_x();
    uint16_t y = read_joystick_y();
    uint8_t sw = read_joystick_switch();
    uint8_t dir = get_joystick_direction();

    // Display X-axis value
    display_number(2, 11, x);

    // Display Y-axis value
    display_number(3, 11, y);

    // Display switch state
    ks0108_puts_at(4, 11, sw ? "PRESS" : "-----");

    // Display direction
    const char *dir_names[] = {"CENTER", "UP    ", "DOWN  ", "LEFT  ", "RIGHT "};
    ks0108_puts_at(5, 11, dir_names[dir]);

    _delay_ms(200);
  }
}

/* ========================================================================
 * DEMO 3: Multi-Channel ADC Scanning with GLCD
 * ======================================================================== */
void demo3_multi_channel_scan(void)
{
  ks0108_clear_screen();
  ScreenBuffer_clear();

  // Title
  ks0108_puts_at(0, 0, "= Multi-Channel =");

  uint16_t adc_values[8] = {0};

  for (uint16_t cycle = 0; cycle < 60; cycle++)
  {
    // Read all 8 ADC channels
    read_all_adc_channels(adc_values, 8);

    // Display channel values in organized grid
    for (uint8_t ch = 0; ch < 8; ch++)
    {
      if (ch < 4)
      {
        // Channels 0-3 on left
        ks0108_puts_at(2 + ch, 0, "CH");
        display_number(2 + ch, 2, ch);
        ks0108_puts_at(2 + ch, 3, ":");
        display_number(2 + ch, 5, adc_values[ch]);
      }
      else
      {
        // Channels 4-7 on right
        ks0108_puts_at(2 + (ch - 4), 11, "CH");
        display_number(2 + (ch - 4), 13, ch);
        ks0108_puts_at(2 + (ch - 4), 14, ":");
        display_number(2 + (ch - 4), 16, adc_values[ch]);
      }
    }

    _delay_ms(200);
  }
}

/* ========================================================================
 * DEMO 4: Averaged ADC Reading (Noise Reduction) with GLCD
 * ======================================================================== */
void demo4_averaged_reading(void)
{
  ks0108_clear_screen();
  ScreenBuffer_clear();

  // Title
  ks0108_puts_at(0, 0, "== Averaging ==");

  // Labels with alignment
  ks0108_puts_at(2, 0, "Single   :");
  ks0108_puts_at(3, 0, "Averaged :");
  ks0108_puts_at(5, 0, "Noise    :");

  while (1)
  {
    // Single reading
    uint16_t single = read_potentiometer();

    // Manual averaging (8 samples for noise reduction)
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
      sum += read_potentiometer();
      _delay_ms(1);
    }
    uint16_t averaged = sum / 8;

    // Calculate difference (noise level)
    int16_t diff = (int16_t)single - (int16_t)averaged;
    uint16_t abs_diff = (diff < 0) ? -diff : diff;

    // Display values
    display_number(2, 11, single);
    display_number(3, 11, averaged);
    display_number(5, 11, abs_diff);

    _delay_ms(300);
  }
}

/* ========================================================================
 * DEMO 5: Interactive Joystick Control with GLCD
 * ======================================================================== */
void demo5_joystick_interactive(void)
{
  // Calibration: Read joystick center position at startup
  _delay_ms(100);
  uint16_t center_x = read_joystick_x();
  uint16_t center_y = read_joystick_y();

  // Clear screen completely - use only buffer clear
  for (uint8_t row = 0; row < 8; row++)
  {
    for (uint8_t col = 0; col < 21; col++)
    {
      ks0108_puts_at(row, col, " ");
    }
  }
  _delay_ms(100);

  // Full screen cursor movement (21 columns x 8 rows)
  // Start at center position
  uint8_t cursor_col = 10; // Center column (0-20)
  uint8_t cursor_row = 4;  // Center row (0-7)

  while (1)
  {
    uint16_t x = read_joystick_x();
    uint16_t y = read_joystick_y();
    uint8_t sw = read_joystick_switch();

    // Apply calibration offset and map to screen with expanded range
    // Offset from center, then scale to screen dimensions
    int16_t offset_x = (int16_t)x - (int16_t)center_x;
    int16_t offset_y = (int16_t)y - (int16_t)center_y;

    // Map offset with slightly expanded scaling to ensure cursor touches edges
    // Scale: ±256 offset -> ±11 columns (touches 0-20), ±5 rows (touches 0-7)
    int8_t col_offset = (offset_x * 17) / 256; // Expanded to touch edges
    int8_t row_offset = (offset_y * 8) / 256;  // Expanded to touch edges

    // Calculate new position with bounds checking
    int8_t new_col = 10 + col_offset; // Start from center (10)
    int8_t new_row = 4 + row_offset;  // Start from center (4), NOT inverted

    // Clamp to screen boundaries
    if (new_col < 0)
      new_col = 0;
    if (new_col > 20)
      new_col = 20;
    if (new_row < 0)
      new_row = 0;
    if (new_row > 7)
      new_row = 7;

    // Clear previous cursor position
    ks0108_puts_at(cursor_row, cursor_col, " ");

    // Update position
    cursor_col = new_col;
    cursor_row = new_row;

    // Draw cursor at new position (different character if switch pressed)
    ks0108_puts_at(cursor_row, cursor_col, sw ? "X" : "*");

    _delay_ms(50);
  }
}

/* ========================================================================
 * DEMO 6: Resistive Touchpad with GLCD
 * ======================================================================== */
void demo6_touchpad_display(void)
{
  ks0108_clear_screen();
  ScreenBuffer_clear();

  // Title
  ks0108_puts_at(0, 0, "== TouchPad ==");

  // Labels with alignment
  ks0108_puts_at(2, 0, "X Pos   :");
  ks0108_puts_at(3, 0, "Y Pos   :");
  ks0108_puts_at(5, 0, "Note: Click on");
  ks0108_puts_at(6, 0, "touchpad in");
  ks0108_puts_at(7, 0, "SimulIDE");

  while (1)
  {
    // Method 1: Read X position
    // Set F1(XR) as output HIGH, F2(XL) as output LOW
    // Read Y-axis pins as ADC to get X position
    DDRF |= (1 << PF1) | (1 << PF2);    // F1,F2 as output
    DDRF &= ~((1 << PF5) | (1 << PF6)); // F5,F6 as input
    PORTF |= (1 << PF1);                // F1 HIGH
    PORTF &= ~(1 << PF2);               // F2 LOW
    _delay_us(10);
    uint16_t x_pos = Read_Adc_Data(5); // Read F5 for X position

    // Method 2: Read Y position
    // Set F5(YL) as output LOW, F6(YR) as output HIGH
    // Read X-axis pins as ADC to get Y position
    DDRF |= (1 << PF5) | (1 << PF6);    // F5,F6 as output
    DDRF &= ~((1 << PF1) | (1 << PF2)); // F1,F2 as input
    PORTF &= ~(1 << PF5);               // F5 LOW
    PORTF |= (1 << PF6);                // F6 HIGH
    _delay_us(10);
    uint16_t y_pos = Read_Adc_Data(1); // Read F1 for Y position

    // Display positions
    display_number(2, 11, x_pos);
    display_number(3, 11, y_pos);

    _delay_ms(100);
  }
} /* ========================================================================
   * Main Program
   * ======================================================================== */
int main(void)
{
  // Initialize system
  adc_sensors_init();

  // Configure PORTB LEDs as output (optional, for status)
  DDRB = 0xFF;
  PORTB = 0xFF; // LEDs off

  // Main loop: Uncomment ONE demo to run
  while (1)
  {
    // Demo 1: Basic Potentiometer Reading with GLCD
    // demo1_potentiometer_basic();
    // _delay_ms(2000);

    // Demo 2: Joystick Position Reading with GLCD
    // demo2_joystick_reading();
    // _delay_ms(2000);

    // Demo 3: Multi-Channel ADC Scanning with GLCD
    // demo3_multi_channel_scan();
    // _delay_ms(2000);

    // Demo 4: Averaged ADC Reading (Noise Reduction) with GLCD
    // demo4_averaged_reading();
    // _delay_ms(2000);

    // Demo 5: Interactive Joystick Control with GLCD
    // demo5_joystick_interactive();
    // _delay_ms(2000);

    // Demo 6: Resistive Touchpad with GLCD
    demo6_touchpad_display();
    _delay_ms(2000);
  }

  return 0;
}
