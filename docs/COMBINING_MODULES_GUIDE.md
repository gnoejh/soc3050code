# Combining Multiple Modules - Student Guide

## Quick Start

Want to combine Timer + UART + LCD in one project? Here's how!

## Step 1: Create Your Project Folder

```
projects/
└── My_Project/          ← Create this folder
    ├── Main.c           ← Your combined code
    └── config.h         ← Configuration header
```

## Step 2: Setup config.h

```c
#ifndef CONFIG_H_
#define CONFIG_H_

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Include the libraries you need
#include "_init.h"
#include "_uart.h"
#include "_glcd.h"
// #include "_timer.h"  // Add more as needed

#endif
```

## Step 3: Combine Code from Multiple Projects

### Example: Timer + UART + LED

```c
#include "config.h"

// Global variables
volatile uint16_t timer_ticks = 0;
volatile uint8_t uart_data_ready = 0;
volatile uint8_t received_byte = 0;

// ISR from Timer_Programming project
ISR(TIMER0_OVF_vect) {
    timer_ticks++;
    TCNT0 = 6;  // Reload for 1ms timing
    
    // Every 1000ms, toggle LED
    if (timer_ticks >= 1000) {
        timer_ticks = 0;
        PORTB ^= (1 << 0);
    }
}

// ISR from Serial_Communications project
ISR(USART1_RX_vect) {
    received_byte = UDR1;
    uart_data_ready = 1;
}

int main(void) {
    // Setup from Timer_Programming
    DDRB = 0xFF;              // LED port
    PORTB = 0xFF;             // LEDs OFF
    
    // Timer0 setup (from Timer_Programming)
    TCCR0 = (1 << CS02) | (1 << CS00);  // Prescaler 1024
    TIMSK = (1 << TOIE0);                // Enable overflow interrupt
    TCNT0 = 6;                           // Initial value
    
    // UART setup (from Serial_Communications)
    UBRR1H = 0;
    UBRR1L = 103;                        // 9600 baud @ 16MHz
    UCSR1B = (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1);
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
    
    sei();  // Enable global interrupts
    
    while (1) {
        if (uart_data_ready) {
            uart_data_ready = 0;
            
            // Echo received byte
            while (!(UCSR1A & (1 << UDRE1)));
            UDR1 = received_byte;
            
            // Toggle LED on receive
            PORTB ^= (1 << 1);
        }
    }
}
```

## Common Peripheral Combinations

### 1. Timer + LCD (Dashboard)

**Use Cases:** Real-time clock display, sensor logging

**ISRs Needed:**
- `TIMER1_COMPA_vect` - 1 second tick
- No UART ISRs

**Example:**
```c
ISR(TIMER1_COMPA_vect) {
    seconds++;
    if (seconds >= 60) {
        seconds = 0;
        minutes++;
    }
    update_display_flag = 1;
}
```

### 2. UART + Sensors (Data Logging)

**Use Cases:** Send sensor data to PC

**ISRs Needed:**
- `ADC_vect` - ADC conversion complete
- `USART1_UDRE_vect` - UART transmit ready (optional)

**Example:**
```c
ISR(ADC_vect) {
    uint16_t adc_value = ADC;
    send_to_uart(adc_value);
}
```

### 3. Timer + Interrupt + Motor (Robot Control)

**Use Cases:** PWM motor control with button input

**ISRs Needed:**
- `TIMER1_COMPA_vect` - PWM generation
- `INT0_vect` - Button press
- `INT1_vect` - Emergency stop

**Example:**
```c
ISR(INT0_vect) {
    // Button pressed - increase speed
    if (motor_speed < 255) motor_speed += 10;
}

ISR(TIMER1_COMPA_vect) {
    // Update PWM duty cycle
    OCR1A = motor_speed;
}
```

## ISR Conflict Resolution

### Problem: Two Projects Use Same ISR

**Example:** Both `Timer_Programming` and `Power_Wakeup_Optimization` use `TIMER0_OVF_vect`

**Solution 1: Use Different Timer**

```c
// Instead of TIMER0, use TIMER1
ISR(TIMER1_COMPA_vect) {
    // Same logic, different timer
}
```

**Solution 2: Merge ISR Logic**

```c
ISR(TIMER0_OVF_vect) {
    // Logic from Timer_Programming
    timer_ticks++;
    
    // Logic from Power_Wakeup_Optimization  
    if (sleep_mode_enabled) {
        wake_up_counter++;
    }
    
    // Common reload
    TCNT0 = 6;
}
```

**Solution 3: Use Flags and Poll**

```c
// Don't use ISR - poll the flag in main loop
while (1) {
    if (TIFR & (1 << TOV0)) {
        TIFR = (1 << TOV0);  // Clear flag
        // Handle overflow
    }
}
```

## Available Resources

### Timers
- **Timer0** (8-bit): General purpose timing
- **Timer1** (16-bit): PWM, precise timing
- **Timer2** (8-bit): RTC, low-power wakeup
- **Timer3** (16-bit): Additional PWM (rarely used in our projects)

### External Interrupts
- **INT0-INT7**: 8 external interrupt pins
  - Most projects only use INT0, INT1
  - INT2-INT7 are available for your use

### Communication
- **USART0**: Available (not used in most projects)
- **USART1**: Used by Serial_Communications
- **SPI**: Used by SPI_* projects
- **I2C/TWI**: Used by I2C_* projects

## Build and Test

### Build Your Combined Project

```powershell
cd projects\My_Project
powershell -ExecutionPolicy Bypass -File ..\..\tools\cli\cli-build-project.ps1 -ProjectDir . -SourceFile Main.c
```

### Simulate in SimulIDE

```powershell
powershell -ExecutionPolicy Bypass -File ..\..\tools\simulide\cli-simulide.ps1 -ProjectDir .
```

## Troubleshooting

### Error: "multiple definition of ISR"

**Cause:** You included a `.c` file from shared_libs that defines an ISR

**Fix:** 
1. Check `config.h` - remove unnecessary includes
2. Copy ISR code to your Main.c instead of including the library

### Error: "undefined reference to function"

**Cause:** You're calling a function from a library but didn't include the `.c` file in compilation

**Fix:** Add the library to your build command or include it in your Main.c

### Peripheral Not Working

**Cause:** Initialization order or missing configuration

**Fix:**
1. Check that you initialized ALL peripherals before `sei()`
2. Verify clock settings match (F_CPU)
3. Check pin configurations (DDRx, PORTx)

## Best Practices

✅ **DO:**
- Start with one peripheral, add more gradually
- Test each peripheral independently first
- Document which ISRs you're using
- Use meaningful variable names
- Comment your merged code clearly

❌ **DON'T:**
- Copy entire Main.c files - extract only what you need
- Use ISRs from multiple projects without merging them
- Mix different F_CPU values
- Forget to enable global interrupts (`sei()`)

## Example Projects to Try

### Beginner
1. **LED + Button**: Timer_Programming + Interrupt
2. **UART Echo**: Serial_Communications only
3. **LCD Clock**: Timer_Programming + Graphics_Display

### Intermediate  
4. **Temperature Logger**: ADC_Basic + Serial_Communications + Timer
5. **Servo Controller**: PWM_Motor_Servo + Interrupt
6. **Sensor Dashboard**: LCD + ADC + Timer

### Advanced
7. **Robot Control**: PWM + Interrupt + UART + Timer
8. **Multi-Sensor System**: I2C + ADC + LCD + Timer
9. **Data Logger**: RTC + EEPROM + UART + Sensors

## Need Help?

1. Check individual project README files
2. Review the main PROJECT_MODULARITY_GUIDE.md
3. Look at Slide.md files for theory
4. Ask Professor Hong Jeong

---

**Remember:** Each peripheral is independent. You're just combining initialization code and ISRs into one Main.c file. Start simple, test often! 🚀
