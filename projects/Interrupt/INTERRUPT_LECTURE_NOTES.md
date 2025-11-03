# ATmega128 Polling vs Interrupt - Educational Reference

## Overview
This document provides comprehensive educational material for understanding polling vs interrupt concepts on the ATmega128 microcontroller.

## ATmega128 External Interrupt System

### Complete Interrupt Vector Table
| Vector | Name    | Pin | Description                | Address | Priority |
|--------|---------|-----|----------------------------|---------|----------|
| 1      | RESET   | -   | Reset Vector               | 0x0000  | Highest  |
| 2      | INT0    | PD0 | External Interrupt 0       | 0x0004  | 2        |
| 3      | INT1    | PD1 | External Interrupt 1       | 0x0006  | 3        |
| 4      | INT2    | PD2 | External Interrupt 2       | 0x0008  | 4        |
| 5      | INT3    | PD3 | External Interrupt 3       | 0x000A  | 5        |
| 6      | INT4    | PE4 | External Interrupt 4       | 0x000C  | 6        |
| 7      | INT5    | PE5 | External Interrupt 5       | 0x000E  | 7        |
| 8      | INT6    | PE6 | External Interrupt 6       | 0x0010  | 8        |
| 9      | INT7    | PE7 | External Interrupt 7       | 0x0012  | 9        |

### Pin Configuration Details
| Interrupt | Pin | Port | Bit | Alternative Functions |
|-----------|-----|------|-----|-----------------------|
| INT0      | PD0 | D    | 0   | SCL, RXD1            |
| INT1      | PD1 | D    | 1   | SDA, TXD1            |
| INT2      | PD2 | D    | 2   | RXD1, INT2           |
| INT3      | PD3 | D    | 3   | TXD1, INT3           |
| INT4      | PE4 | E    | 4   | OC3B, INT4           |
| INT5      | PE5 | E    | 5   | OC3C, INT5           |
| INT6      | PE6 | E    | 6   | T3, INT6             |
| INT7      | PE7 | E    | 7   | IC3, INT7            |

### Control Registers Reference

#### EICRA - External Interrupt Control Register A (0x6A)
```
Bit:  7     6     5     4     3     2     1     0
     ISC31 ISC30 ISC21 ISC20 ISC11 ISC10 ISC01 ISC00
     └─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
      INT3        INT2        INT1        INT0
```

#### EICRB - External Interrupt Control Register B (0x5A)
```
Bit:  7     6     5     4     3     2     1     0
     ISC71 ISC70 ISC61 ISC60 ISC51 ISC50 ISC41 ISC40
     └─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
      INT7        INT6        INT5        INT4
```

#### EIMSK - External Interrupt Mask Register (0x59)
```
Bit:  7     6     5     4     3     2     1     0
     INT7  INT6  INT5  INT4  INT3  INT2  INT1  INT0
     └─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

#### EIFR - External Interrupt Flag Register (0x58)
```
Bit:  7     6     5     4     3     2     1     0
    INTF7 INTF6 INTF5 INTF4 INTF3 INTF2 INTF1 INTF0
    └─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

### Interrupt Sense Control Settings
| ISCn1 | ISCn0 | Mode Description                |
|-------|-------|---------------------------------|
| 0     | 0     | Low level triggers interrupt    |
| 0     | 1     | Any logical change triggers     |
| 1     | 0     | Falling edge triggers (H→L)     |
| 1     | 1     | Rising edge triggers (L→H)      |

## Polling vs Interrupt Detailed Comparison

### Technical Characteristics
| Aspect                | Polling                      | Interrupt                    |
|-----------------------|------------------------------|------------------------------|
| **Response Time**     | Variable (loop dependent)    | Fixed (2-4 clock cycles)    |
| **CPU Utilization**  | 100% (continuous checking)   | Event-driven (efficient)    |
| **Power Consumption** | High (CPU always active)    | Low (CPU can sleep)          |
| **Determinism**      | Predictable timing           | Asynchronous events          |
| **Event Detection**  | May miss short events        | Guaranteed detection         |
| **Code Complexity**  | Simple linear flow           | Requires ISR management      |
| **Debugging**        | Easy to trace execution      | Complex (context switching)  |
| **Multiple Events**  | Sequential checking          | Priority-based handling      |
| **Real-time Response**| Limited by loop time         | Immediate hardware response  |

### Implementation Complexity
| Task                 | Polling Implementation       | Interrupt Implementation     |
|----------------------|------------------------------|------------------------------|
| **Setup**           | Simple port configuration    | Register configuration       |
| **Main Logic**      | while(1) with checks         | Event handlers (ISR)         |
| **Event Handling**  | Direct in main loop          | Separate ISR functions       |
| **Shared Data**     | Regular variables            | Volatile variables required  |
| **Timing Control** | Explicit delays              | Hardware timing              |

## Programming Examples

### Polling Implementation Pattern
```c
void polling_example(void) {
    // Configure ports
    DDRD &= ~(1 << PD1);        // PD1 as input
    PORTD |= (1 << PD1);        // Enable pull-up
    
    while (1) {
        if (!(PIND & (1 << PD1))) {  // Check button state
            // Button pressed - handle immediately
            handle_button_press();
        }
        // Continue with other tasks
        other_tasks();
        _delay_ms(10);  // Polling interval
    }
}
```

### Interrupt Implementation Pattern
```c
volatile uint8_t button_flag = 0;

ISR(INT0_vect) {
    button_flag = 1;            // Set flag for main processing
    // Keep ISR short and fast!
}

void interrupt_example(void) {
    // Configure interrupt
    EICRA |= (1 << ISC01);      // Falling edge
    EICRA &= ~(1 << ISC00);
    EIMSK |= (1 << INT0);       // Enable INT0
    sei();                      // Global interrupt enable
    
    while (1) {
        if (button_flag) {
            button_flag = 0;    // Clear flag
            handle_button_press(); // Process in main context
        }
        // CPU can do other tasks or sleep
        other_tasks();
    }
}
```

## Educational Experiments

### Experiment 1: Response Time Analysis
1. **Polling with different delays:**
   - Try `_delay_ms(1)`, `_delay_ms(50)`, `_delay_ms(200)`
   - Observe how delay affects button responsiveness
   - Measure maximum delay before missing button presses

2. **Interrupt response:**
   - Compare interrupt response time regardless of main loop timing
   - Add heavy processing in main loop and test interrupt response

### Experiment 2: Edge Detection Modes
```c
// Test different interrupt modes:
// 1. Low level (ISC01=0, ISC00=0)
EICRA &= ~((1 << ISC01) | (1 << ISC00));

// 2. Any change (ISC01=0, ISC00=1)  
EICRA &= ~(1 << ISC01);
EICRA |= (1 << ISC00);

// 3. Falling edge (ISC01=1, ISC00=0)
EICRA |= (1 << ISC01);
EICRA &= ~(1 << ISC00);

// 4. Rising edge (ISC01=1, ISC00=1)
EICRA |= ((1 << ISC01) | (1 << ISC00));
```

### Experiment 3: Multiple Interrupts
```c
// Configure multiple external interrupts
void setup_multiple_interrupts(void) {
    // INT0 on falling edge
    EICRA |= (1 << ISC01);
    EICRA &= ~(1 << ISC00);
    
    // INT1 on rising edge  
    EICRA |= ((1 << ISC11) | (1 << ISC10));
    
    // Enable both interrupts
    EIMSK |= ((1 << INT0) | (1 << INT1));
    sei();
}

ISR(INT0_vect) { /* Handle INT0 */ }
ISR(INT1_vect) { /* Handle INT1 */ }
```

## Common Pitfalls and Best Practices

### ISR Programming Guidelines
1. **Keep ISRs short and fast**
   - Avoid delays, heavy calculations, or lengthy operations
   - Set flags for main loop processing instead

2. **Use volatile for shared variables**
   ```c
   volatile uint8_t shared_flag;    // Correct
   uint8_t shared_flag;             // Wrong - may be optimized away
   ```

3. **Disable interrupts for critical sections**
   ```c
   cli();                  // Disable interrupts
   critical_operation();   // Atomic operation
   sei();                  // Re-enable interrupts
   ```

### Polling Best Practices
1. **Choose appropriate polling intervals**
   - Too fast: Wastes CPU cycles
   - Too slow: May miss events

2. **Use state machines for complex logic**
   ```c
   typedef enum {
       BUTTON_IDLE,
       BUTTON_PRESSED,
       BUTTON_HELD
   } button_state_t;
   ```

## Register Value Analysis

### Common PORTB Values (LEDs)
| Binary    | Hex  | Decimal | LED States (Active Low)    |
|-----------|------|---------|----------------------------|
| 11111111  | 0xFF | 255     | All LEDs OFF              |
| 11111110  | 0xFE | 254     | LED0 ON, others OFF       |
| 11110111  | 0xF7 | 247     | LED3 ON, others OFF       |
| 10101010  | 0xAA | 170     | LEDs 0,2,4,6 ON           |
| 01010101  | 0x55 | 85      | LEDs 1,3,5,7 ON           |

### Common PIND Values (Buttons with Pull-ups)
| Binary    | Hex  | Decimal | Button States (Active Low) |
|-----------|------|---------|----------------------------|
| 11111111  | 0xFF | 255     | No buttons pressed         |
| 11111110  | 0xFE | 254     | PD0 pressed               |
| 11111101  | 0xFD | 253     | PD1 pressed               |
| 11111100  | 0xFC | 252     | PD0 and PD1 pressed       |

## Conclusion
Understanding both polling and interrupt methods is crucial for embedded systems programming. Each approach has its place:
- **Use polling** for simple, predictable systems with few inputs
- **Use interrupts** for responsive systems, multiple event sources, or power-sensitive applications

The key is choosing the right approach based on your specific requirements and constraints.