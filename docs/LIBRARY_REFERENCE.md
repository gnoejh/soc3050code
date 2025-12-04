# Library Reference

Complete API reference for enhanced ATmega128 libraries.

---

## 📚 Enhanced Libraries

All shared libraries are in `shared_libs/` with enhanced functionality:

| Library | Functions | Features |
|---------|-----------|----------|
| **GLCD** | 13 | Graphics, icons, shapes, text |
| **ADC** | 30+ | 10-bit conversion, scaling, filtering |
| **UART** | 40+ | Serial communication, formatting |
| **Port** | 8 | GPIO initialization and control |
| **Timer** | 10 | Delays, PWM, timing |
| **Interrupt** | 6 | ISR management |
| **EEPROM** | 4 | Non-volatile storage |

**Total: 83+ enhanced functions**

---

## 🎨 GLCD Library (Graphics LCD)

### Basic Display Functions

```c
void GLCD_init(void);                          // Initialize 128x64 GLCD
void GLCD_clear(void);                         // Clear entire screen
void GLCD_setPixel(uint8_t x, uint8_t y);     // Set single pixel
void GLCD_clearPixel(uint8_t x, uint8_t y);   // Clear single pixel
```

### Drawing Functions

```c
void GLCD_drawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void GLCD_drawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h);
void GLCD_fillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h);
void GLCD_drawCircle(uint8_t x0, uint8_t y0, uint8_t radius);
```

### Text Functions

```c
void GLCD_putChar(char c);                     // Display character
void GLCD_puts(char *str);                     // Display string
void GLCD_printf(const char *fmt, ...);        // Formatted output
void GLCD_setFont(uint8_t size);              // Set font size
```

### Icon Functions (13 Built-in Icons)

```c
void GLCD_drawIcon_Heart(uint8_t x, uint8_t y);
void GLCD_drawIcon_Star(uint8_t x, uint8_t y);
void GLCD_drawIcon_Battery(uint8_t x, uint8_t y, uint8_t level); // 0-100%
void GLCD_drawIcon_Signal(uint8_t x, uint8_t y, uint8_t bars);   // 0-4
void GLCD_drawIcon_Temp(uint8_t x, uint8_t y);
void GLCD_drawIcon_Light(uint8_t x, uint8_t y);
void GLCD_drawIcon_Sound(uint8_t x, uint8_t y);
void GLCD_drawIcon_Warning(uint8_t x, uint8_t y);
void GLCD_drawIcon_Check(uint8_t x, uint8_t y);
void GLCD_drawIcon_Cross(uint8_t x, uint8_t y);
```

**Example:**
```c
GLCD_init();
GLCD_clear();
GLCD_drawIcon_Heart(10, 10);
GLCD_puts("I ♥ Embedded!");
GLCD_drawIcon_Battery(100, 0, 75);  // 75% battery
```

---

## 📊 ADC Library (Analog-to-Digital Converter)

### Initialization

```c
void ADC_init(void);                           // Initialize ADC
void ADC_setReference(uint8_t ref);           // AVCC, AREF, Internal
void ADC_setPrescaler(uint8_t prescaler);     // Clock divider
```

### Basic Reading

```c
uint16_t ADC_read(uint8_t channel);           // Read 10-bit value (0-1023)
uint16_t ADC_readAverage(uint8_t channel, uint8_t samples);  // Averaged
```

### Voltage Conversion

```c
float ADC_toVoltage(uint16_t adc_value);      // Convert to volts (0-5V)
uint16_t ADC_toMillivolts(uint16_t adc_value); // Convert to mV
```

### Sensor Functions

```c
// Temperature (LM35 sensor - 10mV/°C)
float ADC_readTemperature(uint8_t channel);   // Returns °C

// Light (CDS photoresistor)
uint16_t ADC_readLight(uint8_t channel);      // Returns lux approximation

// Potentiometer (returns 0-100%)
uint8_t ADC_readPotPercent(uint8_t channel);

// Generic scaling
int16_t ADC_scale(uint16_t adc_value, int16_t min, int16_t max);
```

### Advanced Features

```c
// Multi-channel scanning
void ADC_scanChannels(uint8_t* channels, uint16_t* results, uint8_t count);

// Filtering
uint16_t ADC_medianFilter(uint8_t channel, uint8_t samples);
uint16_t ADC_movingAverage(uint8_t channel, uint8_t window);

// Calibration
void ADC_calibrate(uint8_t channel, uint16_t known_mv);
uint16_t ADC_readCalibrated(uint8_t channel);

// Threshold detection
uint8_t ADC_isAboveThreshold(uint8_t channel, uint16_t threshold);
uint8_t ADC_isBelowThreshold(uint8_t channel, uint16_t threshold);
```

**Example:**
```c
ADC_init();
ADC_setReference(ADC_REF_AVCC);  // Use 5V reference

uint16_t raw = ADC_read(0);                // Raw value
float voltage = ADC_toVoltage(raw);        // Convert to V
float temp = ADC_readTemperature(1);       // Read temp sensor
uint8_t light = ADC_readPotPercent(2);     // Read as percentage
```

---

## 📡 UART Library (Serial Communication)

### Initialization

```c
void UART_init(uint32_t baud);                // Initialize UART (9600, 115200, etc.)
void UART_setBaudRate(uint32_t baud);        // Change baud rate
```

### Basic I/O

```c
void UART_putChar(char c);                    // Send single character
char UART_getChar(void);                      // Receive character (blocking)
uint8_t UART_available(void);                // Check if data available
```

### String Functions

```c
void UART_putString(const char* str);        // Send string
void UART_getString(char* buffer, uint8_t maxLen); // Receive string
void UART_puts(const char* str);             // Send string + newline
```

### Formatted Output

```c
void UART_printf(const char* fmt, ...);       // Printf-style formatting
void UART_printInt(int16_t num);             // Print integer
void UART_printHex(uint8_t num);             // Print hex (0x00-0xFF)
void UART_printBinary(uint8_t num);          // Print binary (0b00000000)
```

### Advanced Features

```c
// Number parsing
int16_t UART_parseInt(void);                  // Read integer from serial
float UART_parseFloat(void);                  // Read float from serial

// Buffer management
void UART_flush(void);                        // Clear receive buffer
uint8_t UART_peek(void);                      // Look at next char without removing

// Utilities
void UART_printMenu(const char* items[], uint8_t count);
uint8_t UART_getSelection(uint8_t min, uint8_t max);
```

**Example:**
```c
UART_init(9600);
UART_puts("Enter temperature:");
int temp = UART_parseInt();
UART_printf("Temperature: %d°C\n", temp);

if (temp > 30) {
    UART_puts("🔥 HOT!");
}
```

---

## 🔌 Port Library (GPIO)

```c
void PORT_init(void);                         // Initialize all ports
void PORT_setOutput(volatile uint8_t* port, uint8_t pin);
void PORT_setInput(volatile uint8_t* port, uint8_t pin);
void PORT_togglePin(volatile uint8_t* port, uint8_t pin);
uint8_t PORT_readPin(volatile uint8_t* port, uint8_t pin);
```

**Example:**
```c
PORT_init();
PORT_setOutput(&DDRB, 0);  // PB0 as output (LED)
PORT_setInput(&DDRC, 0);   // PC0 as input (button)

while(1) {
    if (PORT_readPin(&PINC, 0)) {
        PORT_togglePin(&PORTB, 0);  // Toggle LED
    }
}
```

---

## ⏱️ Timer Library

```c
void TIMER0_init(void);                       // Initialize Timer0
void TIMER0_delay_ms(uint16_t ms);           // Blocking delay
void TIMER0_PWM(uint8_t dutyCycle);          // PWM output (0-255)

void TIMER1_init(void);                       // 16-bit timer
void TIMER1_setFrequency(uint16_t hz);       // Set output frequency
void TIMER1_servo(uint8_t angle);            // Servo control (0-180°)
```

---

## ⚡ Interrupt Library

```c
void INT0_init(uint8_t mode);                 // External INT0 (PD0)
void INT1_init(uint8_t mode);                 // External INT1 (PD1)
// Modes: RISING, FALLING, BOTH, LOW

void TIMER0_overflow_enable(void);
void TIMER1_overflow_enable(void);
```

**Example ISR:**
```c
#include <avr/interrupt.h>

volatile uint8_t button_pressed = 0;

ISR(INT0_vect) {
    button_pressed = 1;  // Set flag
}

int main(void) {
    INT0_init(FALLING);
    sei();  // Enable global interrupts
    
    while(1) {
        if (button_pressed) {
            // Handle button press
            button_pressed = 0;
        }
    }
}
```

---

## 💾 EEPROM Library

```c
void EEPROM_write(uint16_t addr, uint8_t data);
uint8_t EEPROM_read(uint16_t addr);
void EEPROM_writeBlock(uint16_t addr, uint8_t* data, uint16_t len);
void EEPROM_readBlock(uint16_t addr, uint8_t* buffer, uint16_t len);
```

**Example:**
```c
// Save high score
uint16_t highScore = 1250;
EEPROM_write(0, highScore >> 8);    // MSB
EEPROM_write(1, highScore & 0xFF);  // LSB

// Load high score
uint16_t loaded = (EEPROM_read(0) << 8) | EEPROM_read(1);
```

---

## 🎯 Common Patterns

### Button Debouncing

```c
uint8_t button_debounce(volatile uint8_t* port, uint8_t pin) {
    static uint8_t state = 0;
    static uint16_t counter = 0;
    
    if (PORT_readPin(port, pin)) {
        if (counter < 100) counter++;
        if (counter >= 50 && !state) {
            state = 1;
            return 1;  // Button just pressed
        }
    } else {
        if (counter > 0) counter--;
        if (counter == 0) state = 0;
    }
    return 0;
}
```

### Multi-Sensor Dashboard

```c
void display_dashboard(void) {
    GLCD_clear();
    
    // Temperature
    float temp = ADC_readTemperature(0);
    GLCD_drawIcon_Temp(0, 0);
    GLCD_printf("%.1f°C", temp);
    
    // Light level
    uint8_t light = ADC_readPotPercent(1);
    GLCD_drawIcon_Light(0, 20);
    GLCD_printf("%d%%", light);
    
    // Battery
    uint16_t voltage = ADC_toMillivolts(ADC_read(2));
    uint8_t percent = (voltage - 3000) / 20;  // 3.0V-5.0V → 0-100%
    GLCD_drawIcon_Battery(0, 40, percent);
    GLCD_printf("%d%%", percent);
}
```

### Serial Menu System

```c
void show_menu(void) {
    const char* menu[] = {
        "1. Read Sensors",
        "2. Configure",
        "3. View Logs",
        "4. Exit"
    };
    UART_printMenu(menu, 4);
    
    uint8_t choice = UART_getSelection(1, 4);
    
    switch(choice) {
        case 1: read_sensors(); break;
        case 2: configure(); break;
        case 3: view_logs(); break;
        case 4: return;
    }
}
```

---

## 📖 Usage in Projects

All projects in `projects/` use these enhanced libraries:

```c
// In your Main.c or Lab.c
#include "_glcd.h"
#include "_adc.h"
#include "_uart.h"

int main(void) {
    // Initialize libraries
    GLCD_init();
    ADC_init();
    UART_init(9600);
    
    // Use functions
    GLCD_puts("Hello!");
    float temp = ADC_readTemperature(0);
    UART_printf("Temp: %.1f°C\n", temp);
    
    while(1) {
        // Your code here
    }
}
```

**Build Command:**
```bash
avr-gcc -mmcu=atmega128 -DF_CPU=16000000UL -Os -Wall \
    -I. -I../../shared_libs \
    Main.c \
    ../../shared_libs/_glcd.c \
    ../../shared_libs/_adc.c \
    ../../shared_libs/_uart.c \
    -o Main.elf
```

---

## 🛠️ Customization

Libraries are designed to be extensible:

1. **Add new functions** in `shared_libs/_*.c`
2. **Declare in headers** `shared_libs/_*.h`
3. **Rebuild all projects** `.\cli-build-all.ps1`

**Example - Add custom ADC filter:**
```c
// In _adc.c
uint16_t ADC_kalmanFilter(uint8_t channel) {
    static float estimate = 0;
    static float error = 10;
    
    float measurement = ADC_read(channel);
    float gain = error / (error + 5);  // Kalman gain
    estimate = estimate + gain * (measurement - estimate);
    error = (1 - gain) * error;
    
    return (uint16_t)estimate;
}
```

---

## ✅ Summary

**Enhanced Libraries:**
- 83+ functions across 7 libraries
- Production-ready code for education
- Consistent API across all projects
- Optimized for ATmega128

**Key Features:**
- GLCD with 13 icons
- ADC with filtering and calibration
- UART with printf-style formatting
- Comprehensive error handling
- Well-documented and tested

**Used in:**
- 35 progressive projects
- 65 lab exercises
- All curriculum materials

---

*Last Updated: October 2025*  
*83+ Functions | 7 Libraries | Full ATmega128 Support*

© 2025 Prof. Hong Jeaong, IUT (Inha University in Tashkent)  
All rights reserved for educational purposes.  
**Contact:** [linkedin.com/in/gnoejh53](https://linkedin.com/in/gnoejh53)
