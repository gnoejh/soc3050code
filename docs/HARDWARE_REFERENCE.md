# ATmega128 Educational Kit - Hardware Reference

**CRITICAL REFERENCE**: This document defines the exact hardware configuration for all educational projects in the SOC 3050 framework. All project code must match these specifications.

## 📋 **Core System Specifications**

### **Microcontroller**
- **IC**: ATmega128 (TQFP 64-pin package)
- **Clock**: 16MHz crystal oscillator
- **Architecture**: 8-bit AVR RISC
- **Flash**: 128KB program memory
- **SRAM**: 4KB data memory
- **EEPROM**: 4KB non-volatile memory

### **Power Supply**
- **Operating Voltage**: 5V DC
- **Logic Levels**: TTL (0V = LOW, 5V = HIGH)

---

## 🖥️ **Display Components**

### **Graphics LCD (GLCD)**
- **Model**: JHD12864E (128x64 pixels)
- **Controller**: KS0108 compatible
- **Interface**: 8-bit parallel data bus

**Pin Connections:**
```
GLCD Pin    ATmega128 Pin    Function
--------    -------------    --------
RS          PE4 (Pin 14)     Register Select
RW          GND              Read/Write (tied to GND for write-only)
E           PE5 (Pin 1)      Enable
CS1         PE7 (Pin 16)     Chip Select 1 (left half)
CS2         PE6 (Pin 17)     Chip Select 2 (right half)
D0-D7       PA0-PA7          8-bit Data Bus (PORTA)
VDD         +5V              Power Supply
VSS         GND              Ground
V0          Variable         Contrast Control (potentiometer)
```

**Educational Note**: Students learn parallel interface programming and dual-controller LCD management.

---

## 💡 **Output Components**

### **LED Array**
- **Location**: PORTB (PB0-PB7)
- **Configuration**: 8 LEDs, active LOW (sink current)
- **Pin Mapping**:
  ```
  LED0 = PB0 (Pin 8)   LED4 = PB4 (Pin 12)
  LED1 = PB1 (Pin 9)   LED5 = PB5 (Pin 13)
  LED2 = PB2 (Pin 10)  LED6 = PB6 (Pin 14)
  LED3 = PB3 (Pin 11)  LED7 = PB7 (Pin 15)
  ```
- **Control**: `PORTB = 0x00` = ALL LEDs ON, `PORTB = 0xFF` = ALL LEDs OFF

### **PWM Outputs**
- **Available Pins**: PB4, PB5, PB6, PB7
- **Timer Sources**:
  - PB4 (OC0): Timer0 Compare Output
  - PB5 (OC1A): Timer1 Compare Output A
  - PB6 (OC1B): Timer1 Compare Output B  
  - PB7 (OC2): Timer2 Compare Output

### **Buzzer**
- **Pin**: PG4 (Pin 17)
- **Type**: Active buzzer (digital on/off control)
- **Control**: `PORTG |= (1<<4)` = ON, `PORTG &= ~(1<<4)` = OFF
- **Educational Use**: Sound feedback, alarm systems, debugging audio cues

---

## 🔘 **Input Components**

### **Push Button Switches**
- **Location**: PORTD (PD0-PD7)
- **Configuration**: Active LOW with internal pull-ups
- **Pin Mapping**:
  ```
  SW0 = PD0 (Pin 18)   SW4 = PD4 (Pin 22)
  SW1 = PD1 (Pin 19)   SW5 = PD5 (Pin 23)
  SW2 = PD2 (Pin 20)   SW6 = PD6 (Pin 24)
  SW3 = PD3 (Pin 21)   SW7 = PD7 (Pin 25)
  ```
- **Reading**: `if (!(PIND & (1<<0)))` = SW0 pressed
- **Debouncing**: Required in software (hardware has minimal debouncing)

### **Variable Resistor (Potentiometer)**
- **Pin**: ADC0 (PF0)
- **Range**: 0-5V analog input
- **Resolution**: 10-bit ADC (0-1023 values)
- **Educational Use**: Analog input demonstration, voltage measurement, control input

---

## 🔌 **Communication Interfaces**

### **Serial Communication (UART)**
- **UART0**: Primary serial interface
  - **RX**: PE0 (Pin 2)
  - **TX**: PE1 (Pin 3)
  - **Standard Baud**: 9600 bps (for educational consistency)
  - **Format**: 8N1 (8 data bits, no parity, 1 stop bit)

- **UART1**: Secondary serial interface
  - **RX**: PD2 (Pin 27)
  - **TX**: PD3 (Pin 28)
  - **Standard Baud**: 9600 bps (for educational consistency)
  - **Format**: 8N1 (8 data bits, no parity, 1 stop bit)

### **External Interrupts**
- **Available**: INT0-INT7 on PORTD and PORTE
- **Primary Educational Use**: 
  - INT0 (PD0): Button interrupt demos
  - INT1 (PD1): Secondary interrupt source
  - PE2-PE7: Additional interrupt inputs

---

## 🤖 **Motor Control Systems**

### **Stepper Motor Interface**
- **Type**: 4-phase unipolar stepper motor
- **Control Pins**:
  ```
  Phase A+ = PB0    Common = +5V
  Phase A- = PB1    
  Phase B+ = PB2    
  Phase B- = PB3    
  ```
- **Educational Focus**: Sequential control, precise positioning, timing

### **DC Motor Control**
- **Pin**: PB4 (with PWM capability)
- **Driver**: H-bridge or transistor driver circuit
- **Speed Control**: PWM duty cycle (0-100%)
- **Educational Use**: Speed control, direction control, motor dynamics

### **Servo Motor Interface**
- **Signal Pin**: PB5 (OC1A - Timer1 Compare Output A)
- **Power**: External 5V supply
- **Control**: PWM signal (50Hz, 1-2ms pulse width)
- **Position Range**: 0-180 degrees
- **Educational Focus**: Precise angular control, PWM generation, feedback systems

---

## 📡 **Sensor Interfaces**

### **Ultrasonic Distance Sensor**
- **Echo Pin**: PD1 (input capture)
- **Trigger Pin**: Controlled via potentiometer input
- **Type**: HC-SR04 compatible
- **Range**: 2cm - 400cm
- **Educational Use**: Distance measurement, input capture, timing calculations

### **Analog Sensor Input**
- **Primary ADC**: PF0 (ADC0)
- **Additional ADC Channels**: PF1-PF7 available for expansion
- **Reference Voltage**: 5V (AVCC)
- **Educational Sensors**: Light sensors, temperature sensors, pressure sensors

---

## ⚡ **Power and Signal Specifications**

### **Digital I/O Characteristics**
- **VIL (Input Low)**: 0V - 1.5V
- **VIH (Input High)**: 3.0V - 5V
- **VOL (Output Low)**: 0V - 0.6V @ 20mA
- **VOH (Output High)**: 4.2V - 5V @ 20mA
- **Maximum Current per Pin**: 40mA
- **Maximum Current per Port**: 100mA

### **ADC Specifications**
- **Resolution**: 10-bit (0-1023)
- **Reference Options**: AREF, AVCC, Internal 2.56V
- **Conversion Time**: 13-260 μs (depending on prescaler)
- **Input Impedance**: 100MΩ

---

## 🎯 **Educational Project Mapping**

### **Beginner Projects (Direct Port Control)**
- **LED Control**: PORTB manipulation
- **Button Reading**: PORTD input with pull-ups
- **Buzzer Control**: PORTG bit manipulation

### **Intermediate Projects (Timers & PWM)**
- **PWM Generation**: Timer1/Timer2 with OC1A, OC1B, OC2
- **Servo Control**: 50Hz PWM on PB5
- **DC Motor Speed**: Variable PWM on PB4

### **Advanced Projects (Interrupts & Communication)**
- **External Interrupts**: INT0-INT7 for responsive input
- **Timer Interrupts**: Periodic events and real-time control
- **Serial Communication**: UART0 for PC interface
- **ADC Sampling**: Analog sensor reading and processing

### **Expert Projects (System Integration)**
- **Stepper Motor Control**: 4-phase sequencing with timing
- **GLCD Graphics**: Parallel interface programming
- **Multi-sensor Systems**: ADC multiplexing and data fusion
- **Real-time Control**: Interrupt-driven servo/motor coordination

---

## 🔧 **Development Environment**

### **Programming Interface**
- **ISP Header**: 6-pin standard ISP connector
- **Bootloader**: Optional Arduino-compatible bootloader
- **Programming Tools**: Atmel Studio, avrdude, VS Code with AVR extension

### **Debugging Tools**
- **Serial Monitor**: Primary debugging via UART0
- **LED Status**: Visual debugging via PORTB LEDs
- **Oscilloscope Points**: PWM outputs, interrupt pins, timer outputs

---

## ⚠️ **Important Educational Notes**

### **Clock Configuration**
- **CRITICAL**: All projects assume 16MHz crystal
- **F_CPU must be set to 16000000UL in all code**
- **UART baud rate calculations depend on 16MHz**
- **Timer prescaler calculations assume 16MHz**

### **Pin Usage Conflicts**
- **Avoid**: Using PORTB for input when LEDs are connected
- **Caution**: GLCD uses entire PORTA for data bus
- **Note**: Some pins have dual functions (PWM + LED)

### **Educational Safety**
- **Current Limits**: Respect 40mA per pin maximum
- **Voltage Levels**: All I/O is 5V TTL logic
- **ESD Protection**: Handle kit with proper anti-static precautions

---

## 📚 **Code Templates**

### **Standard Initialization**
```c
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

void init_hardware(void) {
    // LED setup (PORTB as output)
    DDRB = 0xFF;
    PORTB = 0xFF;  // All LEDs off
    
    // Button setup (PORTD as input with pull-ups)
    DDRD = 0x00;
    PORTD = 0xFF;  // Enable pull-ups
    
    // Buzzer setup
    DDRG |= (1<<4);
    PORTG &= ~(1<<4);  // Buzzer off
}
```

### **GLCD Initialization**
```c
void init_glcd(void) {
    // Data bus (PORTA) as output
    DDRA = 0xFF;
    
    // Control pins as output
    DDRE |= (1<<4) | (1<<5) | (1<<6) | (1<<7);
    
    // Initialize GLCD controllers
    // (specific initialization sequence required)
}
```

---

## 🚨 **Critical Issues for Educational Framework**

### **Common Mistakes to Avoid**
1. **F_CPU Configuration**: Always verify `#define F_CPU 16000000UL` in all projects
2. **UART Baud Rate**: Stick to 9600 baud for educational consistency
3. **LED Logic**: Remember LEDs are active LOW (0 = ON, 1 = OFF)
4. **Button Reading**: Use pull-ups and check for LOW state when pressed
5. **Interrupt Vectors**: Use exact vector names like `ISR(INT0_vect)`, not wrappers

### **Pin Conflict Warnings**
- **PORTB**: Cannot use as input when LEDs are connected (output only)
- **PORTA**: Reserved for GLCD data bus in graphics projects
- **PG1**: Tied to GND for GLCD, cannot be used as I/O
- **PE0/PE1**: Reserved for UART communication

### **Educational Progression Validation**
Before creating any new project, verify:
- [ ] Hardware connections match this reference
- [ ] Pin assignments don't conflict with kit layout
- [ ] F_CPU and baud rates are correct for 16MHz
- [ ] ISR implementations use real vectors, not wrappers
- [ ] Code examples work with actual kit hardware

This hardware reference ensures all educational projects are developed with accurate pin assignments and realistic hardware constraints, providing students with authentic embedded systems programming experience.