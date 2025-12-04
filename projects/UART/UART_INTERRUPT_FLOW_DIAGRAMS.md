# UART Interrupt Flow Diagrams
**Visual Guide for Students - ATmega128 USART1**

---

## 📥 Receive (RX) Interrupt Flow

```
┌─────────────────────────────────────────────────────────────┐
│                    UART HARDWARE                            │
│                                                             │
│  Serial Data In → Shift Register → UDR1 Register          │
│                                         │                   │
│                                         ▼                   │
│                                    RXC1 Flag = 1           │
└──────────────────────────────────────┬──────────────────────┘
                                       │
                            ┌──────────▼──────────┐
                            │ Is RXCIE1 enabled?  │
                            │  (UCSR1B bit 7)     │
                            └──────────┬──────────┘
                                       │
                         ┌─────────────┴─────────────┐
                         │ YES                       │ NO
                         ▼                           ▼
              ┌──────────────────┐        ┌────────────────┐
              │ Is sei() called? │        │ Nothing happens│
              │ (Global I enable)│        │ Must poll RXC1 │
              └────────┬─────────┘        └────────────────┘
                       │
              ┌────────┴────────┐
              │ YES             │ NO
              ▼                 ▼
   ┌──────────────────┐   ┌──────────┐
   │ ISR(USART1_RX_   │   │ ISR won't│
   │ vect) FIRES!     │   │ execute  │
   └────────┬─────────┘   └──────────┘
            │
            ▼
   ┌────────────────────────────────┐
   │ char data = UDR1;              │  ◄── Reading UDR1 clears RXC1!
   │                                │
   │ rx_buffer[rx_head] = data;    │
   │ rx_head = (rx_head+1) % SIZE; │
   └────────┬───────────────────────┘
            │
            ▼
   ┌────────────────┐
   │ Return to main │  ◄── ISR must be SHORT and FAST!
   └────────────────┘
```

---

## 📤 Transmit (TX) Interrupt Flow

```
┌─────────────────────────────────────────────────────────────┐
│                      MAIN PROGRAM                           │
│                                                             │
│  1. tx_buffer[tx_head++] = data;  // Put data in buffer   │
│  2. UCSR1B |= (1<<UDRIE1);        // Enable TX interrupt   │
└──────────────────────────────────┬──────────────────────────┘
                                   │
                   ┌───────────────▼────────────────┐
                   │     UART HARDWARE              │
                   │                                │
                   │  UDR1 Register Empty?          │
                   │  UDRE1 Flag = 1               │
                   └──────────┬─────────────────────┘
                              │
                   ┌──────────▼──────────┐
                   │ Is UDRIE1 enabled?  │
                   │  (UCSR1B bit 5)     │
                   └──────────┬──────────┘
                              │
                ┌─────────────┴─────────────┐
                │ YES                       │ NO
                ▼                           ▼
     ┌──────────────────┐        ┌────────────────┐
     │ Is sei() called? │        │ Nothing happens│
     │ (Global I enable)│        │ Must poll UDRE1│
     └────────┬─────────┘        └────────────────┘
              │
     ┌────────┴────────┐
     │ YES             │ NO
     ▼                 ▼
┌──────────────────┐   ┌──────────┐
│ ISR(USART1_UDRE_ │   │ ISR won't│
│ vect) FIRES!     │   │ execute  │
└────────┬─────────┘   └──────────┘
         │
         ▼
┌────────────────────────────────────┐
│ if (tx_head == tx_tail) {          │
│   UCSR1B &= ~(1<<UDRIE1);          │  ◄── CRITICAL: Disable when empty!
│   return;                          │      Otherwise infinite ISR loop!
│ }                                  │
│                                    │
│ UDR1 = tx_buffer[tx_tail];         │  ◄── Writing UDR1 clears UDRE1!
│ tx_tail = (tx_tail+1) % SIZE;      │
└────────┬───────────────────────────┘
         │
         ▼
┌────────────────┐
│ Return to main │  ◄── ISR must be SHORT and FAST!
└────────┬───────┘
         │
         │  ┌──────────────────────────────────┐
         └──┤ UDR1 sends data via shift register│
            │ → UDRE1=1 again → ISR fires again │
            │ (loop until buffer empty)         │
            └───────────────────────────────────┘
```

---

## 🔄 Circular Buffer Visualization

### Initial State (Empty Buffer)
```
┌───┬───┬───┬───┬───┬───┬───┬───┐
│   │   │   │   │   │   │   │   │  rx_buffer[8]
└───┴───┴───┴───┴───┴───┴───┴───┘
  ↑
  │
head = 0 (ISR writes here)
tail = 0 (main reads here)

Empty when: head == tail
```

### After Receiving 3 Characters ('A', 'B', 'C')
```
┌───┬───┬───┬───┬───┬───┬───┬───┐
│ A │ B │ C │   │   │   │   │   │
└───┴───┴───┴───┴───┴───┴───┴───┘
  ↑       ↑
  │       │
 tail    head = 3 (next write position)
 = 0     

Data available: (head != tail) → TRUE
Characters available: 3
```

### After main() Reads 1 Character
```
┌───┬───┬───┬───┬───┬───┬───┬───┐
│ A │ B │ C │   │   │   │   │   │
└───┴───┴───┴───┴───┴───┴───┴───┘
      ↑   ↑
      │   │
    tail head = 3
    = 1

Read 'A', tail advanced
Characters still available: 2 ('B' and 'C')
```

### Wrap-Around Example (Buffer Full)
```
┌───┬───┬───┬───┬───┬───┬───┬───┐
│ H │ B │ C │ D │ E │ F │ G │   │
└───┴───┴───┴───┴───┴───┴───┴───┘
  ↑                           ↑
  │                           │
head = 0 (wrapped!)          tail = 1

Next write would be at index 0
Buffer is almost full: ((head+1) % 8) == tail
```

---

## ⚙️ Complete RX/TX System Interaction

```
┌──────────────────────────────────────────────────────────────┐
│                        MAIN PROGRAM                          │
│                                                              │
│  while(1) {                                                  │
│    // Non-blocking! CPU free to do other work               │
│                                                              │
│    if (rx_head != rx_tail) {          ◄── Check RX buffer  │
│      char c = rx_buffer[rx_tail++];                         │
│      process(c);                                            │
│                                                              │
│      // Echo back                                           │
│      tx_buffer[tx_head++] = c;                              │
│      UCSR1B |= (1<<UDRIE1);          ◄── Enable TX ISR     │
│    }                                                         │
│                                                              │
│    do_other_work();  // ◄── CPU is FREE! Not blocked!      │
│  }                                                           │
└─────────┬──────────────────────────────┬─────────────────────┘
          │                              │
          │ RX Data Arrives              │ TX Data Ready
          ▼                              ▼
┌─────────────────────┐        ┌─────────────────────┐
│ ISR(USART1_RX_vect) │        │ ISR(USART1_UDRE_    │
│                     │        │      _vect)         │
│ char c = UDR1;      │        │                     │
│ rx_buffer[rx_head]  │        │ if (buffer_empty)   │
│   = c;              │        │   UCSR1B &=         │
│ rx_head++;          │        │     ~(1<<UDRIE1);   │
│                     │        │ else                │
│ // FAST! Return     │        │   UDR1 = tx_buffer  │
│                     │        │     [tx_tail++];    │
└─────────────────────┘        └─────────────────────┘
     ▲                              ▲
     │                              │
     │ Hardware triggers            │ Hardware triggers
     │ when byte received           │ when UDR1 empty
     │                              │
┌────┴──────────────────────────────┴────┐
│          UART1 HARDWARE                │
│                                        │
│  RX Pin ──→ Shift Reg ──→ UDR1 ──→ RXC1│
│                                        │
│  TX Pin ←── Shift Reg ←── UDR1 ←─ UDRE1│
└────────────────────────────────────────┘
```

---

## 🎯 Key Concepts for Students

### 1. Hardware vs Software Separation
```
┌────────────────────────────────────┐
│  HARDWARE (Automatic)              │
├────────────────────────────────────┤
│ • Shift registers                  │
│ • UDR1 register                    │
│ • Flag generation (RXC1, UDRE1)    │
│ • Baud rate timing                 │
└────────────────────────────────────┘
         ▲          │
         │          ▼
┌────────────────────────────────────┐
│  SOFTWARE (You Program)            │
├────────────────────────────────────┤
│ • Enable interrupts (RXCIE1/UDRIE1)│
│ • ISR handlers                     │
│ • Circular buffers                 │
│ • Data processing                  │
└────────────────────────────────────┘
```

### 2. Polling vs Interrupt Comparison

**POLLING (Demo 1-3):**
```
main() {
  while(1) {
    while(!(UCSR1A & (1<<RXC1)));  ◄── CPU BLOCKED HERE!
    char c = UDR1;
    
    while(!(UCSR1A & (1<<UDRE1))); ◄── CPU BLOCKED HERE!
    UDR1 = c;
    
    // Cannot do other work while waiting!
  }
}
```

**INTERRUPT (Demo 4-6):**
```
ISR(USART1_RX_vect) {     ─┐
  buffer[head++] = UDR1;   │ Hardware calls ISR
}                          ─┘ automatically!

main() {
  while(1) {
    if (chars_available()) {
      process_data();
    }
    
    do_led_blinking();        ◄── CPU FREE for other work!
    read_sensors();           ◄── Multitasking possible!
    update_display();         ◄── While ISRs handle I/O!
  }
}
```

### 3. Flag Auto-Clearing (Important!)

**RX Flag Cleared by Reading:**
```
ISR(USART1_RX_vect) {
    char c = UDR1;        ◄── Reading UDR1 automatically clears RXC1!
    // No manual clearing needed
}
```

**TX Flag Cleared by Writing:**
```
ISR(USART1_UDRE_vect) {
    UDR1 = data;          ◄── Writing UDR1 automatically clears UDRE1!
    // No manual clearing needed
}
```

### 4. Critical UDRIE1 Management

**❌ WRONG (Infinite ISR Loop!):**
```c
ISR(USART1_UDRE_vect) {
    UDR1 = tx_buffer[tx_tail++];
    // UDRIE1 still enabled, but buffer empty
    // ISR fires again immediately → INFINITE LOOP!
}
```

**✅ CORRECT:**
```c
ISR(USART1_UDRE_vect) {
    if (tx_head == tx_tail) {           // Check if empty
        UCSR1B &= ~(1<<UDRIE1);         // DISABLE interrupt!
        return;
    }
    UDR1 = tx_buffer[tx_tail++];
}
```

---

## 📝 Study Exercises

1. **Trace the RX path:** 
   - Serial bit arrives → Shift register → UDR1 → RXC1=1 → ISR fires → Buffer stores → main() reads

2. **Why volatile?**
   - ISR changes `rx_head`, main() reads it → Compiler might cache value
   - `volatile` tells compiler "this can change unexpectedly"

3. **Why sei()?**
   - RXCIE1/UDRIE1 enable specific interrupts
   - But global I-bit (set by sei()) must also be enabled!
   - Both conditions required: `RXCIE1=1 AND I-bit=1`

4. **Buffer full scenario:**
   - What happens if ISR writes faster than main() reads?
   - Need to check: `((head+1) % SIZE) != tail` before writing

---

**💡 Test Your Understanding:**

1. Can ISR fire if `RXCIE1=1` but `sei()` not called? **NO**
2. Does reading UDR1 clear RXC1? **YES**
3. Should UDRIE1 stay enabled when buffer empty? **NO**
4. Can main() and ISR access same variable? **Only if volatile**
5. Should ISR do complex processing? **NO - keep it short!**

---

*For complete code examples, see Main.c demos 1-6!*
*Last Updated: October 19, 2025*
