# Library Usage Guide
**Educational ATmega128 Framework - Professional Quality Libraries**

**Version:** 3.0  
**Last Updated:** November 2025  
**Purpose:** Guide for selecting and using refined shared libraries

---

## Overview

The shared library ecosystem has been completely refined to **professional quality**. This guide helps you choose the right library for your project and understand the library hierarchy.

### Library Quality Grades

| Library | Grade | Status | Purpose |
|---------|-------|--------|---------|
| **_timer.h** | A+ | ⭐ **NEW** | General timer (all Timer0/1/2/3) |
| **_pwm.h** | A+ | ⭐ **NEW** | PWM abstraction (servo, motor, LED) |
| **_eeprom.h** | A+ | ✅ Perfect | EEPROM operations |
| **_interrupt.h** | A | ✅ Excellent | Interrupt management |
| **_glcd.h** | A+ | ⭐ **UPGRADED** | Graphics LCD (KS0108) |
| **_adc.h** | A | ⭐ **ENHANCED** | ADC with calibration |
| **_port.h** | A | ⭐ **ENHANCED** | Port control with debouncing |
| **uart_enhanced.h** | A+ | ✅ Advanced | UART with error handling |
| **_uart.h** | B+ | ✅ Good | Basic UART (educational) |
| **_buzzer.h** | A- | ✅ Very good | Sound generation |
| **_init.h** | B | ✅ Adequate | System initialization |

### Legacy Libraries (Backward Compatibility)

| Library | Status | Replacement |
|---------|--------|-------------|
| **_timer2.h** | ❌ REMOVED | Use **_timer.h** |
| **_glcd_legacy.h** | 📦 Kept | Use **_glcd.h** |
| **ks0108_complete.h** | ❌ REMOVED | Renamed to **_glcd.h** |

---

## Library Selection Decision Tree

### 🎯 **Need Timing Functions?**

**Q: Which timer do I need?**

- **Any timer** (Timer0, 1, 2, or 3) → Use **`_timer.h`**
  - ✅ General purpose, all modes
  - ✅ Normal, CTC, Fast PWM, Phase Correct PWM
  - ✅ Input capture (Timer1)
  - ✅ Millisecond system (Arduino-like)
  - ✅ Task scheduler

- **Legacy Timer2 code** → Update to **`_timer.h`**
  ```c
  // OLD (Timer2-specific):
  Timer2_init();
  Timer2_start();
  
  // NEW (General timer):
  Timer_init(TIMER_2, TIMER_MODE_NORMAL, TIMER_PRESCALER_64);
  Timer_start(TIMER_2, TIMER_PRESCALER_64);
  ```

**Backward Compatibility:** Old `Timer2_init()` macro redirects to new API.

---

### 🎯 **Need PWM Output?**

**Q: What application?**

- **Servo motor control** → Use **`_pwm.h`** servo functions
  ```c
  PWM_servo_init(PWM_CH_1A);
  PWM_start(PWM_CH_1A);
  PWM_servo_set_angle(PWM_CH_1A, 90);  // Center position
  PWM_servo_sweep(PWM_CH_1A, 0, 180, 20);  // Sweep
  ```

- **DC motor speed control** → Use **`_pwm.h`** motor functions
  ```c
  PWM_motor_init(PWM_CH_3A, PWM_FREQ_MOTOR_20KHZ);
  PWM_start(PWM_CH_3A);
  PWM_motor_set_speed(PWM_CH_3A, 75);  // 75% speed
  PWM_motor_ramp_speed(PWM_CH_3A, 100, 1000);  // Ramp to 100%
  ```

- **LED brightness control** → Use **`_pwm.h`** LED functions
  ```c
  PWM_led_init(PWM_CH_0, PWM_FREQ_LED_1KHZ);
  PWM_start(PWM_CH_0);
  PWM_led_set_brightness_gamma(PWM_CH_0, 50);  // 50% with gamma
  PWM_led_fade(PWM_CH_0, 100, 500);  // Fade to full
  PWM_led_pulse(PWM_CH_0, 2000, 5);  // Breathe effect
  ```

- **General PWM** → Use **`_pwm.h`** core functions
  ```c
  PWM_init(PWM_CH_1B, 1000, PWM_MODE_FAST);  // 1kHz
  PWM_start(PWM_CH_1B);
  PWM_set_duty_cycle(PWM_CH_1B, 50);  // 50% duty
  ```

**Available PWM Channels:**
- `PWM_CH_0` (Timer0, PB4)
- `PWM_CH_1A/1B/1C` (Timer1, PB5/6/7)
- `PWM_CH_2` (Timer2, PB7 - conflicts with 1C)
- `PWM_CH_3A/3B/3C` (Timer3, PE3/4/5)

---

### 🎯 **Need Graphics Display?**

**Q: Which GLCD library?**

- **Production projects** → Use **`_glcd.h`** (NEW standard)
  - ✅ Complete KS0108 implementation
  - ✅ Professional documentation
  - ✅ Pixel graphics, shapes, text
  - ✅ Printf support
  - ✅ Hardware specifications
  ```c
  #include "_glcd.h"
  
  ks0108_init();
  ks0108_clear_screen();
  ks0108_puts_at(0, 0, "Hello World");
  ks0108_draw_circle(64, 32, 20, KS0108_PIXEL_ON);
  ks0108_printf("Value: %d", sensor_value);
  ```

- **Legacy projects** → Can use **`_glcd_legacy.h`**
  - 📦 Kept for backward compatibility
  - ⚠️ Minimal documentation (Korean comments)
  - ⚠️ Basic features only
  - 🔄 **Recommended:** Migrate to **`_glcd.h`**

**Migration Path:**
1. Replace `#include "_glcd_legacy.h"` with `#include "_glcd.h"`
2. Update function names: `lcd_init()` → `ks0108_init()`
3. Use new features (printf, shapes, etc.)

---

### 🎯 **Need ADC Measurements?**

**Q: What level of precision?**

- **Basic sensor reading** → Use **`_adc.h`** basic functions
  ```c
  ADC_init();
  unsigned int raw = Read_Adc(ADC_CHANNEL_0);
  unsigned int voltage_mV = Read_Adc_Voltage(ADC_CHANNEL_0);
  ```

- **Temperature/light sensors** → Use **`_adc.h`** sensor helpers
  ```c
  int temp_C = Read_Temperature_Celsius(ADC_CHANNEL_0);
  unsigned char light = Read_Light_Level(ADC_CHANNEL_1);  // 0-100%
  ```

- **Multi-channel scanning** → Use **`_adc.h`** scanning
  ```c
  unsigned char channels[] = {0, 1, 2, 3};
  unsigned int results[4];
  Scan_Adc_Channels(channels, results, 4);
  ```

- **High-precision measurements** → Use **enhanced ADC features**
  ```c
  ADC_init();
  
  // Calibration
  ADC_calibrate(5000);  // Known 5V reference
  
  // Precision control
  ADC_set_precision(10);  // 10-bit mode
  
  // Free-running mode
  ADC_start_free_running(ADC_CHANNEL_0);
  while(1) {
      unsigned int value = ADC_get_last_result();
      // Process continuous data
  }
  ```

- **Differential measurements** → Use **differential mode**
  ```c
  // Measure ADC0 - ADC1 with 10x gain
  int diff = ADC_read_differential(0, 1, 10);
  // Useful for: bridge sensors, thermocouples
  ```

**Enhanced ADC Features:**
- ✅ `ADC_calibrate()` - Known voltage calibration
- ✅ `ADC_set_precision()` - 8-bit or 10-bit
- ✅ `ADC_start_free_running()` - Continuous mode
- ✅ `ADC_read_differential()` - Differential inputs
- ✅ `ADC_set_reference()` - AREF, AVCC, or 2.56V internal

---

### 🎯 **Need Serial Communication?**

**Q: Basic or advanced features?**

- **Educational basics** → Use **`_uart.h`**
  - ✅ Simple polling interface
  - ✅ Number formatting helpers
  - ✅ Good for learning
  - ✅ Legacy compatible
  ```c
  Uart1_init();
  putch_USART1('A');
  puts_USART1("Hello\r\n");
  USART1_print_decimal(1234);
  unsigned char c = getch_USART1();  // Blocking read
  ```

- **Production projects** → Use **`uart_enhanced.h`**
  - ✅ Printf support
  - ✅ Error handling (frame, parity, overrun)
  - ✅ Configurable parameters
  - ✅ Statistics and diagnostics
  - ✅ Timeout support
  ```c
  uart_enhanced_init(9600, 8, 0, 1);  // 9600 8N1
  uart_enhanced_printf("Temp: %d°C\r\n", temp);
  
  uint8_t data;
  if(uart_enhanced_receive(&data, 1000) == UART_NO_ERROR) {
      // Process data
  }
  
  uint8_t errors = uart_enhanced_get_error_flags();
  uart_enhanced_print_status();  // Diagnostic output
  ```

**When to use which:**
- **Learning UART basics** → `_uart.h`
- **Professional applications** → `uart_enhanced.h`
- **Error handling needed** → `uart_enhanced.h`
- **Printf needed** → `uart_enhanced.h`

---

### 🎯 **Need Button Input?**

**Q: Reliable or educational?**

- **Educational demos (quick & dirty)** → Use **basic port functions**
  ```c
  Port_init();
  if(button_pressed(0)) {
      led_on(0);
  }
  // WARNING: May detect bounce as multiple presses
  ```

- **Reliable applications** → Use **enhanced debounced functions**
  ```c
  Port_init();
  
  // Set debounce delay (default 20ms)
  port_set_debounce_delay(30);  // 30ms for noisy switches
  
  // Debounced reading
  if(button_pressed_debounced(0)) {
      led_toggle(0);
      button_wait_release(0);  // Prevent multiple detection
  }
  
  // Wait for any button
  unsigned char button = wait_for_any_button_debounced();
  printf("Button %d pressed\n", button);
  ```

**Enhanced Port Features:**
- ✅ `button_pressed_debounced()` - Software debouncing
- ✅ `button_wait_release()` - Debounced release wait
- ✅ `port_enable_pullup()` - Internal pull-up management
- ✅ `port_read/write_masked()` - Bit masking operations
- ✅ `read_buttons_debounced()` - All buttons debounced
- ✅ Configurable debounce delay (5-100ms)

**Debouncing Algorithm:**
1. Read button state
2. Wait debounce delay (default 20ms)
3. Read again
4. If both match → stable state
5. Otherwise → repeat

---

## Library Combinations

### Common Project Patterns

#### **1. Servo Control with Serial Monitor**
```c
#include "_pwm.h"
#include "uart_enhanced.h"

void main(void) {
    // Initialize
    PWM_servo_init(PWM_CH_1A);
    PWM_start(PWM_CH_1A);
    uart_enhanced_init(9600, 8, 0, 1);
    
    // Control loop
    while(1) {
        uart_enhanced_printf("Enter angle (0-180): ");
        uint8_t angle = read_number();  // Your input function
        
        PWM_servo_set_angle(PWM_CH_1A, angle);
        uart_enhanced_printf("Servo moved to %d°\r\n", angle);
    }
}
```

#### **2. ADC Data Logger with EEPROM**
```c
#include "_adc.h"
#include "_eeprom.h"
#include "_timer.h"

void main(void) {
    ADC_init();
    ADC_calibrate(5000);  // Calibrate to 5V
    EEPROM_init();
    Timer_millis_init();
    sei();
    
    uint16_t address = 0;
    while(1) {
        // Read and store every 1 second
        if((Timer_millis() % 1000) == 0) {
            uint16_t value = Read_Adc(ADC_CHANNEL_0);
            EEPROM_write_int(address, value);
            address += 2;
            
            if(address >= EEPROM_SIZE) break;  // Full
        }
    }
}
```

#### **3. LCD Dashboard with Multiple Sensors**
```c
#include "_glcd.h"
#include "_adc.h"
#include "_timer.h"

void main(void) {
    ks0108_init();
    ADC_init();
    Timer_millis_init();
    sei();
    
    while(1) {
        // Update display every 100ms
        if((Timer_millis() % 100) == 0) {
            int temp = Read_Temperature_Celsius(0);
            uint8_t light = Read_Light_Level(1);
            uint16_t voltage = Read_Adc_Voltage(2);
            
            ks0108_clear_screen();
            ks0108_printf("Temp:  %d°C", temp);
            ks0108_set_cursor(1, 0);
            ks0108_printf("Light: %d%%", light);
            ks0108_set_cursor(2, 0);
            ks0108_printf("Volt:  %d mV", voltage);
        }
    }
}
```

#### **4. Multi-Channel PWM Motor Control**
```c
#include "_pwm.h"
#include "_port.h"

void main(void) {
    Port_init();
    
    // Initialize 2 motors
    PWM_motor_init(PWM_CH_3A, PWM_FREQ_MOTOR_20KHZ);
    PWM_motor_init(PWM_CH_3B, PWM_FREQ_MOTOR_20KHZ);
    PWM_start(PWM_CH_3A);
    PWM_start(PWM_CH_3B);
    
    while(1) {
        // Control with buttons
        if(button_pressed_debounced(0)) {
            PWM_motor_ramp_speed(PWM_CH_3A, 100, 500);  // Accelerate
        }
        if(button_pressed_debounced(1)) {
            PWM_motor_ramp_speed(PWM_CH_3A, 0, 500);    // Decelerate
        }
        if(button_pressed_debounced(2)) {
            PWM_motor_brake(PWM_CH_3A);  // Emergency stop
        }
    }
}
```

---

## Migration Guides

### Migrating from Timer2-Specific to General Timer

**Old Code (_timer2.h):**
```c
#include "_timer2.h"

Timer2_init();
Timer2_start();

// Task-based timing
if(Task1_Of_Timer2) {
    // Do task 1
    Task1_Of_Timer2 = 0;
}

// Millisecond timing
uint32_t ms = system_milliseconds;
```

**New Code (_timer.h):**
```c
#include "_timer.h"

// Option 1: Direct replacement (backward compatible)
Timer2_init();  // Macro redirects to Timer_init()
Timer2_start(); // Macro redirects to Timer_start()

// Option 2: Use new API explicitly
Timer_init(TIMER_2, TIMER_MODE_NORMAL, TIMER2_PRESCALER_64);
Timer_start(TIMER_2, TIMER2_PRESCALER_64);

// Task-based timing → Use scheduler
Timer_scheduler_init(TIMER_2);
Timer_scheduler_add_task(task1_function, 500);  // Every 500ms
Timer_scheduler_update();  // Call from main loop

// Millisecond timing → Use millis system
Timer_millis_init();  // Uses Timer0
uint32_t ms = Timer_millis();
```

**Advantages of new API:**
- ✅ Can use ANY timer (not just Timer2)
- ✅ Modern callback-based interrupts
- ✅ Task scheduler (up to 8 tasks)
- ✅ Arduino-like millis()/micros()
- ✅ All timer modes available

---

### Migrating from Basic GLCD to Professional GLCD

**Old Code (_glcd_legacy.h):**
```c
#include "_glcd.h"  // Old version

lcd_init();
lcd_clear();
lcd_string(0, 0, "Hello");
GLCD_Line(0, 0, 127, 63);
GLCD_Circle(64, 32, 20);
```

**New Code (_glcd.h):**
```c
#include "_glcd.h"  // New version (ks0108_complete)

ks0108_init();
ks0108_clear_screen();
ks0108_puts_at(0, 0, "Hello");
ks0108_draw_line(0, 0, 127, 63, KS0108_PIXEL_ON);
ks0108_draw_circle(64, 32, 20, KS0108_PIXEL_ON);

// NEW features not available in old library:
ks0108_printf("Temp: %d°C", temp);  // Printf support!
ks0108_fill_rect(10, 10, 50, 30, KS0108_PIXEL_ON);
ks0108_set_pixel(x, y, KS0108_PIXEL_XOR);  // XOR mode
```

**Function Name Mapping:**
| Old Function | New Function | Notes |
|--------------|--------------|-------|
| `lcd_init()` | `ks0108_init()` | More specific name |
| `lcd_clear()` | `ks0108_clear_screen()` | Clearer intent |
| `lcd_string(x,y,s)` | `ks0108_puts_at(line,col,s)` | Different coordinates |
| `GLCD_Line()` | `ks0108_draw_line()` | Consistent naming |
| `GLCD_Circle()` | `ks0108_draw_circle()` | + fill option |
| `GLCD_Rectangle()` | `ks0108_draw_rect()` | + fill option |
| N/A | `ks0108_printf()` | ⭐ NEW feature |

**Backward Compatibility:**
- Old `_glcd_legacy.h` still available
- Use `#include "_glcd_legacy.h"` if needed
- **Recommended:** Migrate to new API for full features

---

## Best Practices

### 1. **Include Order**
```c
// Standard AVR headers first
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Framework main header
#include "_main.h"

// Framework libraries (alphabetical)
#include "_adc.h"
#include "_glcd.h"
#include "_port.h"
#include "_pwm.h"
#include "_timer.h"
#include "_uart.h"
```

### 2. **Initialize in Correct Order**
```c
void main(void) {
    // 1. Basic I/O first
    Port_init();
    
    // 2. Timing systems
    Timer_millis_init();  // For delays/timing
    sei();  // Enable interrupts early
    
    // 3. Communication
    uart_enhanced_init(9600, 8, 0, 1);
    
    // 4. Peripherals
    ADC_init();
    PWM_init(...);
    ks0108_init();
    
    // 5. Calibration/config
    ADC_calibrate(5000);
    
    // Main loop
    while(1) {
        // ...
    }
}
```

### 3. **Use Descriptive Names**
```c
// BAD:
PWM_init(1, 50, 0);
ADC_init();
int x = Read_Adc(0);

// GOOD:
PWM_servo_init(PWM_CH_1A);
ADC_init();
ADC_calibrate(AVCC_REFERENCE);
int temperature_C = Read_Temperature_Celsius(TEMPERATURE_SENSOR_ADC);
```

### 4. **Check Return Values**
```c
// BAD:
PWM_init(PWM_CH_1A, 50, PWM_MODE_FAST);

// GOOD:
if(!PWM_init(PWM_CH_1A, 50, PWM_MODE_FAST)) {
    uart_enhanced_printf("ERROR: PWM init failed\r\n");
    while(1);  // Halt
}
```

### 5. **Use Constants, Not Magic Numbers**
```c
// BAD:
ADC_set_precision(10);
PWM_servo_set_angle(ch, 90);

// GOOD:
ADC_set_precision(ADC_PRECISION_10BIT);  // Or just document it
PWM_servo_set_angle(servo_channel, SERVO_ANGLE_CENTER);
```

---

## Common Pitfalls

### ❌ **Using Wrong Timer for PWM**
```c
// WRONG: Timer2 conflicts with PWM_CH_1C
Timer_init(TIMER_2, ...);
PWM_init(PWM_CH_1C, ...);  // Uses Timer1, but pin PB7 conflicts!

// CORRECT: Use Timer3 for separate PWM
PWM_init(PWM_CH_3A, ...);  // Timer3, no conflicts
```

### ❌ **Forgetting to Enable Interrupts**
```c
// WRONG:
Timer_millis_init();
uint32_t ms = Timer_millis();  // Always returns 0!

// CORRECT:
Timer_millis_init();
sei();  // Enable global interrupts!
uint32_t ms = Timer_millis();  // Works
```

### ❌ **Not Debouncing Buttons**
```c
// WRONG: Will toggle multiple times per press
if(button_pressed(0)) {
    led_toggle(0);
}

// CORRECT:
if(button_pressed_debounced(0)) {
    led_toggle(0);
    button_wait_release(0);  // Wait for release
}
```

### ❌ **Using Blocking Delays with Interrupts**
```c
// WRONG: Blocks interrupt processing
ISR(TIMER0_OVF_vect) {
    _delay_ms(100);  // DON'T DO THIS IN ISR!
}

// CORRECT: Set flag, process in main
volatile uint8_t flag = 0;
ISR(TIMER0_OVF_vect) {
    flag = 1;  // Just set flag
}

void main(void) {
    while(1) {
        if(flag) {
            _delay_ms(100);  // OK in main loop
            flag = 0;
        }
    }
}
```

---

## Quick Reference

### Timer Functions
| Function | Purpose |
|----------|---------|
| `Timer_init()` | Initialize timer with mode/prescaler |
| `Timer_start/stop()` | Start/stop counting |
| `Timer_set_compare_value()` | Set compare match value (CTC/PWM) |
| `Timer1_input_capture_init()` | Setup input capture |
| `Timer_millis()` | Get milliseconds since init |
| `Timer_scheduler_add_task()` | Add scheduled task |

### PWM Functions
| Function | Purpose |
|----------|---------|
| `PWM_servo_init/set_angle()` | Servo control (0-180°) |
| `PWM_motor_init/set_speed()` | DC motor control (0-100%) |
| `PWM_led_init/set_brightness()` | LED dimming (0-100%) |
| `PWM_set_duty_cycle()` | General PWM control |

### ADC Functions
| Function | Purpose |
|----------|---------|
| `Read_Adc()` | Basic 10-bit read |
| `Read_Adc_Voltage()` | Convert to millivolts |
| `ADC_calibrate()` | Calibrate with known voltage |
| `ADC_read_differential()` | Differential measurement |
| `ADC_start_free_running()` | Continuous conversion |

### Port Functions
| Function | Purpose |
|----------|---------|
| `button_pressed_debounced()` | Debounced button read |
| `button_wait_release()` | Wait for button release |
| `port_enable_pullup()` | Enable internal pull-up |
| `port_read/write_masked()` | Bit masking operations |

---

## Version History

### Version 3.0 (November 2025) - **MAJOR REFINEMENT**
- ⭐ **NEW:** General Timer library (`_timer.h`)
- ⭐ **NEW:** PWM library (`_pwm.h`)
- ⭐ **UPGRADED:** GLCD to professional version
- ✨ **ENHANCED:** ADC with calibration features
- ✨ **ENHANCED:** Port with debouncing
- ❌ **REMOVED:** Timer2-specific library
- 📚 **ADDED:** This comprehensive usage guide

### Version 2.0 (Previous)
- Initial educational library collection
- Basic Timer2, ADC, Port, UART libraries
- EEPROM and Interrupt libraries

---

## Support and Feedback

**Questions?** Check:
1. Header file comments (comprehensive documentation)
2. This usage guide
3. `SHARED_LIBS_ASSESSMENT.md` for library analysis
4. Example projects in `projects/` folder

**Found a bug?** Report with:
- Library name and function
- Expected vs actual behavior
- Minimal code to reproduce

**Feature request?** Document:
- Use case and educational value
- Proposed API design
- Priority and impact

---

**Last Updated:** November 2025  
**Framework Version:** 3.0  
**Status:** Professional Quality - Ready for Curriculum Development ✅
