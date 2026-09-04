# Lab Exercise: Interrupt-Based Serial Communication with LCD Display

**Course**: SOC 3050 - Embedded Systems and Applications  
**Lab**: Serial Communications - Q&A System  
**File**: Lab.c  
**Date**: October 2025

---

## 📋 Lab Overview

This hands-on laboratory exercise teaches interrupt-based serial communication by implementing an interactive Question & Answer (Q&A) system that displays questions and answers on a graphical LCD while communicating via VS Code's serial monitor.

### Learning Objectives

By completing this lab, students will:

1. ✅ Master interrupt-driven UART communication (ISR programming)
2. ✅ Understand circular buffer management for asynchronous I/O
3. ✅ Integrate multiple peripherals (UART + GLCD) in real-time
4. ✅ Design command protocols for embedded systems
5. ✅ Practice non-blocking programming techniques
6. ✅ Display student information on LCD as required

---

## 🎯 Lab Requirements

### Mandatory Features

- [x] **Student Information Display**: Name and ID must appear on LCD at startup
- [x] **Interrupt-Based Communication**: Use ISR for UART RX/TX (no polling!)
- [x] **Question Display**: Receive questions via serial, display on LCD
- [x] **Answer Display**: Receive answers via serial, display on LCD below question
- [x] **Command Processing**: Handle special commands (INFO, STATS, HELP, etc.)
- [x] **Error Handling**: Validate input format and provide feedback
- [x] **Non-Blocking Operation**: CPU remains free (LED toggles show idle time)

---

## 🔧 Hardware Setup

### Required Components

- **ATmega128** microcontroller @ 16MHz
- **KS0108 Graphic LCD** (128x64 pixels)
- **USB-TTL Serial Adapter** for UART1 connection
- **VS Code Serial Monitor** (9600 baud, 8N1)

### Hardware Connections

```
ATmega128 UART1:
- PD2 (RXD1) → Serial TX from computer
- PD3 (TXD1) → Serial RX to computer
- GND → Common ground

KS0108 LCD:
- Data: PORTA (PA0-PA7)
- Control: PORTC, PORTG (CS1, CS2, RS, R/W, E)

LED Indicators (PORTB):
- PB0 → LED0 (RX Activity)
- PB1 → LED1 (TX Activity)
- PB2 → LED2 (Command Processing)
- PB3 → LED3 (Errors)
- PB4 → LED4 (System Idle/Running)
```

---

## 🚀 Building and Programming

### Step 1: Configure Student Information

Open `Lab.c` and edit the student information section:

```c
// ========== STUDENT INFORMATION - FILL THIS IN! ==========
#define STUDENT_NAME "Hong Gil Dong"  // TODO: Enter your name
#define STUDENT_ID   "2025123456"      // TODO: Enter your student ID
#define LAB_DATE     "2025-10-21"      // TODO: Today's date
// =========================================================
```

### Step 2: Build the Project

```batch
cd w:\soc3050code\projects\Serial_Communications
build_lab.bat
```

Expected output:
```
Building Serial Communications Lab for ATmega128...
Build successful! Creating HEX file...
Build completed! Files: Lab.elf, Lab.hex
```

### Step 3: Program Hardware

```batch
cd w:\soc3050code\projects\Serial_Communications
..\..\tools\avr-toolchain\bin\avrdude.exe -c arduino -p m128 -P COM3 -U flash:w:Lab.hex:i
```

Or use VS Code task: **Build and Program Current Project**

### Step 4: Open Serial Monitor

1. Open VS Code Serial Monitor
2. Configure: **9600 baud, 8N1**
3. Connect to your COM port (usually COM3)

---

## 📡 Communication Protocol

### Command Format

| Command | Format | Description | Example |
|---------|--------|-------------|---------|
| **Question** | `Q: <text>` | Send question to student | `Q: What is an ISR?` |
| **Answer** | `A: <text>` | Student answers question | `A: Interrupt Service Routine` |
| **Info** | `CMD:INFO` | Show student info on LCD | `CMD:INFO` |
| **Stats** | `CMD:STATS` | Show Q&A statistics | `CMD:STATS` |
| **Clear** | `CMD:CLEAR` | Clear LCD display | `CMD:CLEAR` |
| **Reset** | `CMD:RESET` | Reset session counters | `CMD:RESET` |
| **Help** | `CMD:HELP` | Show command reference | `CMD:HELP` |

### Protocol Rules

1. **Question must come before answer**: Send `Q:` first, then `A:`
2. **Case-insensitive**: `q:`, `Q:`, `cmd:`, `CMD:` all work
3. **Line ending**: Press Enter to send command
4. **Echo enabled**: Your typed characters appear in serial monitor
5. **Prompts**: System shows `>` when ready for input

---

## 🧪 Testing Procedure

### Test 1: Initial Setup ✓

```
EXPECTED OUTPUT (Serial Monitor):
=============================================
  Serial Communication Lab - Q&A System
  SOC 3050 - Embedded Systems Lab
=============================================

Student: Hong Gil Dong
ID: 2025123456
Date: 2025-10-21

Lab Features:
- Interrupt-based serial communication
- GLCD display integration
- Real-time Q&A system
- LED indicators (RX/TX/Activity/Error)

LED Indicators:
  LED0 (PB0) - RX activity
  LED1 (PB1) - TX activity
  LED2 (PB2) - Command processing
  LED3 (PB3) - Errors

Type CMD:HELP for command list
Ready for questions!

>
```

```
EXPECTED OUTPUT (LCD):
┌────────────────────┐
│ Student Info:      │
│                    │
│ Name: Hong Gil Dong│
│ ID: 2025123456     │
│                    │
│ Date: 2025-10-21   │
│                    │
│ Ready for Q&A      │
└────────────────────┘
```

**✓ Verify**: Name and ID appear correctly on both serial and LCD

**✓ Verify LEDs**: At startup, all LEDs (LED0-LED3) should flash briefly, then turn off

---

### Test 2: LED Indicators ✓

**Test RX LED (LED0):**
```
> Type any text slowly...
```

**Expected**: LED0 (PB0) toggles rapidly with each character typed

**Test TX LED (LED1):**
```
> Send any command and watch responses
```

**Expected**: LED1 (PB1) toggles as system sends response text

**Test Activity LED (LED2):**
```
> Q: Test question
```

**Expected**: LED2 (PB2) toggles when command is processed

**Test Error LED (LED3):**
```
> A: Answer without question
```

**Expected**: LED3 (PB3) toggles on error

**Test Idle LED (LED4):**
```
> Leave system idle for a few seconds
```

**Expected**: LED4 (PB4) blinks slowly showing CPU is free

**✓ Verify**: All 5 LEDs working independently

---

### Test 3: Basic Q&A Flow ✓

**Send Question:**
```
> Q: What is an ISR?
```

**Expected Serial Response:**
```
>>> Question 01 received: What is an ISR?
>>> Please enter answer (A: <your answer>)
>
```

**Expected LCD Display:**
```
┌────────────────────┐
│ Question 01:       │
│                    │
│ What is an ISR?    │
│                    │
│                    │
│                    │
│ Waiting answer...  │
│                    │
└────────────────────┘
```

**Expected LED Activity:**
- LED0 toggles as you type
- LED2 toggles when processing command
- LED1 toggles as system responds

**Send Answer:**
```
> A: Interrupt Service Routine
```

**Expected Serial Response:**
```
>>> Answer recorded: Interrupt Service Routine
>>> Send next question or type CMD:STATS
>
```

**Expected LCD Display:**
```
┌────────────────────┐
│ Question 01:       │
│                    │
│ What is an ISR?    │
│                    │
│                    │
│                    │
│ Answer:            │
│ Interrupt Service  │
└────────────────────┘
```

**✓ Verify**: Question and answer both appear on LCD

**✓ Verify LEDs**: LED0, LED1, and LED2 toggled during this interaction

---

### Test 4: Command Processing ✓

**Test CMD:STATS:**
```
> CMD:STATS
```

**Expected Serial:**
```
>>> Session Statistics:
    Questions: 01
    Answers: 01
>
```

**Expected LCD:**
```
┌────────────────────┐
│ Session Stats:     │
│                    │
│ Questions: 01      │
│ Answers: 01        │
│                    │
│ Status: Complete   │
│                    │
│                    │
└────────────────────┘
```

**Test CMD:INFO:**
```
> CMD:INFO
```

**Expected**: Returns to student information screen

**Test CMD:HELP:**
```
> CMD:HELP
```

**Expected Serial:**
```
=== Lab Command Reference ===
Q: <text>       - Send question
A: <text>       - Send answer
CMD:INFO        - Show student info
CMD:STATS       - Show statistics
CMD:CLEAR       - Clear LCD
CMD:RESET       - Reset session
CMD:HELP        - This help
==============================
>
```

**✓ Verify**: All commands execute correctly

---

### Test 5: Error Handling ✓

**Test Answer Before Question:**
```
> A: This should fail
```

**Expected:**
```
>>> ERROR: No question pending. Send Q: first!
>
```

**Expected LED**: LED3 (Error) toggles

**Test Invalid Format:**
```
> This is wrong
```

**Expected:**
```
>>> ERROR: Invalid format. Use Q: or A: or CMD:
>
```

**Expected LED**: LED3 (Error) toggles

**✓ Verify**: Errors are caught and reported properly, LED3 indicates errors

---

### Test 6: Multiple Q&A ✓

**Send 3 Questions:**
```
> Q: What does UART stand for?
>>> Question 01 received: What does UART stand for?
>>> Please enter answer (A: <your answer>)
> A: Universal Asynchronous Receiver Transmitter
>>> Answer recorded: Universal Asynchronous Receiver Transmitter
>>> Send next question or type CMD:STATS

> Q: What is the ATmega128 clock frequency?
>>> Question 02 received: What is the ATmega128 clock frequency?
>>> Please enter answer (A: <your answer>)
> A: 16 MHz
>>> Answer recorded: 16 MHz
>>> Send next question or type CMD:STATS

> Q: What baud rate does this lab use?
>>> Question 03 received: What baud rate does this lab use?
>>> Please enter answer (A: <your answer>)
> A: 9600
>>> Answer recorded: 9600
>>> Send next question or type CMD:STATS

> CMD:STATS
>>> Session Statistics:
    Questions: 03
    Answers: 03
>
```

**✓ Verify**: Question counter increments correctly

---

## 🔍 Code Analysis Section

### Understanding the Interrupt System

#### ISR for UART Receive (ISR(USART1_RX_vect))

```c
ISR(USART1_RX_vect)
{
    char received = UDR1;  // Read character (clears RXC1 flag)
    uint8_t next_head = (rx_head + 1) % RX_BUFFER_SIZE;
    
    // Check for buffer overflow
    if (next_head != rx_tail) {
        rx_buffer[rx_head] = received;
        rx_head = next_head;
    } else {
        rx_overflow = 1;  // Flag overflow
    }
}
```

**Key Concepts:**
1. **Automatic Trigger**: ISR called when character arrives (RXC1 flag set)
2. **Reading UDR1**: Automatically clears RXC1 flag
3. **Circular Buffer**: Modulo arithmetic wraps around buffer
4. **Overflow Detection**: Prevents buffer overrun corruption
5. **Volatile Variables**: `rx_head` must be volatile for ISR/main() sharing

#### ISR for UART Transmit (ISR(USART1_UDRE_vect))

```c
ISR(USART1_UDRE_vect)
{
    if (tx_head != tx_tail) {
        UDR1 = tx_buffer[tx_tail];  // Send next character
        tx_tail = (tx_tail + 1) % TX_BUFFER_SIZE;
    } else {
        UCSR1B &= ~(1 << UDRIE1);  // Disable interrupt when done
        tx_busy = 0;
    }
}
```

**Key Concepts:**
1. **Automatic Trigger**: ISR called when UDR1 ready for next byte (UDRE1 flag set)
2. **Self-Disabling**: Must disable interrupt when buffer empty (prevents infinite ISR)
3. **Writing UDR1**: Automatically clears UDRE1 flag
4. **Busy Flag**: `tx_busy` prevents re-enabling already-active interrupt

---

### Understanding Circular Buffers

#### Visual Representation

```
RX Buffer (SIZE = 8):
   tail        head
    ↓           ↓
[H][e][l][l][o][ ][ ][ ]
 0  1  2  3  4  5  6  7

Reading: uart_getchar() reads rx_buffer[tail++]
Writing: ISR writes rx_buffer[head++]
Empty: head == tail
Full: (head + 1) % SIZE == tail
```

#### Why Circular Buffers?

1. **Fixed Memory**: No dynamic allocation needed
2. **Fast**: O(1) enqueue/dequeue operations
3. **No Data Movement**: Just move head/tail pointers
4. **ISR-Safe**: Single-producer, single-consumer pattern
5. **Wrap-Around**: Modulo arithmetic reuses buffer space

---

### Understanding UART Register Configuration

#### UCSR1A - Status Register

```c
UCSR1A = (1 << U2X1);  // Enable double-speed mode
```

**U2X1 = 1**: Double-speed mode for better baud rate accuracy
- Normal mode: UBRR = (F_CPU / (16 * BAUD)) - 1
- U2X mode: UBRR = (F_CPU / (8 * BAUD)) - 1
- At 16MHz, 9600 baud: UBRR = 207 (very accurate with U2X=1)

#### UCSR1B - Control Register

```c
UCSR1B = (1 << RXCIE1) | (1 << RXEN1) | (1 << TXEN1);
```

- **RXCIE1**: Enable RX Complete Interrupt (triggers ISR on character received)
- **RXEN1**: Enable UART receiver hardware
- **TXEN1**: Enable UART transmitter hardware
- **UDRIE1**: (set later) Enable UDR Empty Interrupt for TX

#### UCSR1C - Frame Format

```c
UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
```

- **UCSZ11:10 = 11**: 8-bit character size
- **No parity** (default)
- **1 stop bit** (default)
- Result: **8N1** format (industry standard)

---

### Understanding LCD Integration

#### LCD Coordinate System

```
KS0108 Display: 128x64 pixels, organized as 8 pages (8 pixels tall)
Text mode: 21 characters × 8 lines

X-axis: 0-20 (characters)
Y-axis: 0-7 (text lines)

lcd_string(0, 0, "Hello");  // Top-left corner
lcd_string(0, 7, "Bottom"); // Bottom-left corner
```

#### LCD Function Usage

```c
lcd_clear();                    // Clear entire display
lcd_string(0, 0, "Question:");  // Print at (0,0)
lcd_xy(10, 2);                  // Move cursor to (10,2)
lcd_char('A');                  // Print single character
GLCD_2DigitDecimal(42);         // Print "42" (numeric)
```

---

## 📊 Grading Rubric

### Functionality (60 points)

| Requirement | Points | Verification |
|-------------|--------|--------------|
| Student info displayed on LCD at startup | 10 | Visual check on LCD |
| Interrupt-based RX (no polling) | 10 | Code review: ISR(USART1_RX_vect) |
| Interrupt-based TX (no polling) | 10 | Code review: ISR(USART1_UDRE_vect) |
| Questions display on LCD | 10 | Send Q: command, verify LCD |
| Answers display on LCD | 10 | Send A: command, verify LCD |
| Command processing (INFO, STATS, etc.) | 5 | Test all commands |
| Error handling (invalid input) | 5 | Test error cases |

### Code Quality (20 points)

| Requirement | Points | Verification |
|-------------|--------|--------------|
| Proper volatile variables for ISR | 5 | Code review |
| Circular buffer implementation | 5 | Code review |
| Clear comments and documentation | 5 | Code review |
| No magic numbers (use #define) | 5 | Code review |

### Testing & Validation (20 points)

| Requirement | Points | Verification |
|-------------|--------|--------------|
| All test cases completed | 10 | Test log submission |
| Error cases tested | 5 | Test log submission |
| Multiple Q&A tested | 5 | Test log submission |

---

## 🐛 Troubleshooting

### Problem: LCD shows garbage characters

**Causes:**
- LCD not initialized properly
- Timing issues with GLCD

**Solutions:**
```c
// Ensure initialization order:
glcd_port_init();  // Configure ports first
lcd_init();        // Then initialize LCD controller
lcd_clear();       // Clear display
_delay_ms(100);    // Allow time to stabilize
```

### Problem: No serial communication

**Causes:**
- Wrong COM port selected
- Baud rate mismatch
- Missing sei() call

**Solutions:**
```c
// Check UART initialization:
init_uart_lab();   // Sets up UART + ISR
sei();             // MUST enable global interrupts!
```

### Problem: Characters lost during reception

**Causes:**
- RX buffer too small
- Processing takes too long
- Buffer overflow

**Solutions:**
```c
#define RX_BUFFER_SIZE 128  // Increase if needed

// Check overflow flag:
if (rx_overflow) {
    uart_puts(">>> WARNING: RX buffer overflow!\r\n");
    rx_overflow = 0;
}
```

### Problem: TX interrupt runs forever

**Causes:**
- Forgot to disable UDRIE1 when buffer empty

**Solutions:**
```c
ISR(USART1_UDRE_vect)
{
    if (tx_head != tx_tail) {
        UDR1 = tx_buffer[tx_tail];
        tx_tail = (tx_tail + 1) % TX_BUFFER_SIZE;
    } else {
        // CRITICAL: Disable interrupt when done!
        UCSR1B &= ~(1 << UDRIE1);
        tx_busy = 0;
    }
}
```

---

## 📚 Additional Resources

### Reference Documents

- [UART Interrupt Flow Diagrams](UART_INTERRUPT_FLOW_DIAGRAMS.md)
- [UART Register Quick Reference](UART_REGISTER_QUICK_REFERENCE.md)
- [Peripherals Guide](../Assembly_C/Peripherals.md) - Section 6: USART

### Related Projects

- **Main.c**: Educational demo with 6 serial examples (polling vs interrupt)
- **Serial_interrupt**: Another interrupt-based serial project

### ATmega128 Datasheet Sections

- **Section 20**: USART (Universal Synchronous/Asynchronous Receiver Transmitter)
- **Section 4**: Interrupts and Interrupt Handling
- **Pages 173-194**: USART register descriptions

---

## ✅ Lab Completion Checklist

Before submitting, verify all requirements:

- [ ] **Code compiles without errors** (`build_lab.bat` successful)
- [ ] **Student name and ID correctly filled in** (check #define section)
- [ ] **LCD displays student info at startup** (visual verification)
- [ ] **Questions appear on LCD** (test Q: command)
- [ ] **Answers appear on LCD below questions** (test A: command)
- [ ] **All commands work** (INFO, STATS, CLEAR, RESET, HELP)
- [ ] **Error handling tested** (answer before question, invalid format)
- [ ] **Multiple Q&A tested** (at least 3 questions)
- [ ] **Code uses interrupts, not polling** (ISR functions present)
- [ ] **LED toggles showing CPU idle** (LED0 blinks on PORTB)
- [ ] **Test log completed** (document all test results)
- [ ] **Code properly commented** (explain ISR logic)

---

## 📝 Submission Requirements

Submit the following files:

1. **Lab.c** - Your completed source code with student info filled in
2. **Lab.hex** - Compiled binary file
3. **Test_Log.txt** - Results from all test procedures
4. **Screenshots** - LCD showing:
   - Student information screen
   - Question display
   - Answer display
   - Statistics screen

---

## 🎓 Learning Outcomes Assessment

After completing this lab, you should be able to:

1. ✅ Explain how UART interrupts work in ATmega128
2. ✅ Implement circular buffers for asynchronous I/O
3. ✅ Configure UART registers (UCSR1A/B/C, UBRR1H/L)
4. ✅ Write interrupt service routines (ISR syntax and rules)
5. ✅ Integrate UART and GLCD peripherals in one system
6. ✅ Design command protocols for embedded communication
7. ✅ Debug real-time embedded systems
8. ✅ Use volatile variables correctly for ISR/main() sharing

---

**Professor Hong Jeong**  
Department of Computer Engineering  
2025 Fall Semester  
SOC 3050 - Embedded Systems and Applications

---

*This lab is part of a comprehensive embedded systems curriculum teaching practical hardware programming skills using real ATmega128 hardware and SimulIDE simulation.*
