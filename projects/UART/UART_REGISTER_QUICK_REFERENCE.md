# UART Register Quick Reference for Students
**ATmega128 USART1 - SOC 3050 Embedded Systems**

---

## 📋 Complete Register Map

### 1️⃣ **UCSR1A** - Status & Control Register A
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ 7   │ 6   │ 5   │ 4   │ 3   │ 2   │ 1   │ 0   │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│RXC1 │TXC1 │UDRE1│ FE1 │DOR1 │UPE1 │U2X1 │MPCM1│
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

| Bit | Name | Function | Usage |
|-----|------|----------|-------|
| 7 | **RXC1** | Receive Complete Flag | `while(!(UCSR1A & (1<<RXC1)));` Wait for RX |
| 5 | **UDRE1** | Data Register Empty | `while(!(UCSR1A & (1<<UDRE1)));` Wait for TX ready |
| 1 | **U2X1** | Double Speed Mode | `UCSR1A = (1<<U2X1);` Enable for accuracy |

**Key Point:** RXC1 and UDRE1 are READ-ONLY flags for polling!

---

### 2️⃣ **UCSR1B** - Control Register B (The Important One!)
```
┌──────┬──────┬──────┬──────┬──────┬──────┬─────┬─────┐
│ 7    │ 6    │ 5    │ 4    │ 3    │ 2    │ 1   │ 0   │
├──────┼──────┼──────┼──────┼──────┼──────┼─────┼─────┤
│RXCIE1│TXCIE1│UDRIE1│RXEN1 │TXEN1 │UCSZ12│RXB81│TXB81│
└──────┴──────┴──────┴──────┴──────┴──────┴─────┴─────┘
```

| Bit | Name | Function | Critical Usage |
|-----|------|----------|----------------|
| 7 | **RXCIE1** | 🔔 RX Interrupt Enable | `UCSR1B \|= (1<<RXCIE1);` Enable RX ISR |
| 5 | **UDRIE1** | 🔔 TX Interrupt Enable | `UCSR1B \|= (1<<UDRIE1);` Enable TX ISR |
| 4 | **RXEN1** | ✅ Receiver Enable | **REQUIRED** to receive data |
| 3 | **TXEN1** | ✅ Transmitter Enable | **REQUIRED** to transmit data |

#### Common Patterns:

**Polling Mode:**
```c
UCSR1B = (1<<RXEN1) | (1<<TXEN1);  // Enable RX/TX only
```

**Interrupt Mode:**
```c
UCSR1B = (1<<RXCIE1) | (1<<RXEN1) | (1<<TXEN1);  // Enable RX interrupt
sei();  // MUST enable global interrupts!

// Later, to send data:
UCSR1B |= (1<<UDRIE1);  // Enable TX interrupt when buffer has data
```

⚠️ **CRITICAL:** Must disable UDRIE1 when no data to send!
```c
ISR(USART1_UDRE_vect) {
    if (buffer_empty) {
        UCSR1B &= ~(1<<UDRIE1);  // DISABLE to prevent infinite ISR!
        return;
    }
    UDR1 = next_byte;
}
```

---

### 3️⃣ **UCSR1C** - Control Register C (Frame Format)
```
┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬─────┐
│ 7    │ 6    │ 5    │ 4    │ 3    │ 2    │ 1    │ 0   │
├──────┼──────┼──────┼──────┼──────┼──────┼──────┼─────┤
│UMSEL1│  -   │UPM11 │UPM10 │USBS1 │UCSZ11│UCSZ10│  -  │
└──────┴──────┴──────┴──────┴──────┴──────┴──────┴─────┘
```

**Standard 8N1 Format (8 data bits, No parity, 1 stop bit):**
```c
UCSR1C = (1<<UCSZ11) | (1<<UCSZ10);  // That's it!
```

| Setting | UCSZ11 | UCSZ10 | UCSZ12 | Result |
|---------|--------|--------|--------|--------|
| **8-bit** | 1 | 1 | 0 | Standard (8N1) |

---

### 4️⃣ **UBRR1H & UBRR1L** - Baud Rate Registers

**Formula (with U2X1=1):**
```
UBRR = (F_CPU / (8 × BAUD)) - 1
```

**Common Values @ 16MHz:**

| Baud Rate | UBRR Value | Error % | Code |
|-----------|------------|---------|------|
| 2400 | 832 | 0.0% | `uint16_t ubrr = 832;` |
| 4800 | 416 | 0.0% | `uint16_t ubrr = 416;` |
| **9600** | **207** | **0.16%** | `uint16_t ubrr = 207;` ⭐ |
| 19200 | 103 | 0.16% | `uint16_t ubrr = 103;` |
| 38400 | 51 | 0.16% | `uint16_t ubrr = 51;` |
| 57600 | 34 | -0.79% | `uint16_t ubrr = 34;` |

**Usage:**
```c
uint16_t ubrr = 207;  // 9600 baud
UBRR1H = (ubrr >> 8);  // High byte
UBRR1L = ubrr;         // Low byte
```

---

### 5️⃣ **UDR1** - Data Register (The Actual Data!)

**Receive (Polling):**
```c
while (!(UCSR1A & (1<<RXC1)));  // Wait for data
char data = UDR1;                // Read clears RXC1 flag
```

**Transmit (Polling):**
```c
while (!(UCSR1A & (1<<UDRE1))); // Wait until ready
UDR1 = 'A';                     // Write clears UDRE1 flag
```

**Receive (Interrupt):**
```c
ISR(USART1_RX_vect) {
    char data = UDR1;  // Reading clears RXC1 automatically
    // Process data...
}
```

**Transmit (Interrupt):**
```c
ISR(USART1_UDRE_vect) {
    UDR1 = next_byte;  // Writing clears UDRE1 automatically
}
```

---

## 🔔 Interrupt Vectors

### Available ISRs:

| ISR Vector | Triggered By | Enable Bit | Common Use |
|------------|--------------|------------|------------|
| `ISR(USART1_RX_vect)` | Data received (RXC1=1) | RXCIE1 | **Primary RX handler** ⭐ |
| `ISR(USART1_UDRE_vect)` | UDR1 empty (UDRE1=1) | UDRIE1 | **Primary TX handler** ⭐ |
| `ISR(USART1_TX_vect)` | Frame sent (TXC1=1) | TXCIE1 | Rarely used |

### Interrupt Setup Sequence:

```c
// Step 1: Hardware configuration
UCSR1A = (1<<U2X1);                      // Double speed
UCSR1C = (1<<UCSZ11) | (1<<UCSZ10);     // 8-bit
UBRR1H = (207 >> 8);
UBRR1L = 207;

// Step 2: Enable RX/TX and RX interrupt
UCSR1B = (1<<RXCIE1) | (1<<RXEN1) | (1<<TXEN1);

// Step 3: Enable global interrupts
sei();  // ⚠️ CRITICAL - Nothing works without this!

// Step 4: Enable TX interrupt only when needed
if (data_to_send) {
    UCSR1B |= (1<<UDRIE1);  // Enable TX ISR
}
```

---

## 📝 ISR Programming Rules

### ✅ DO:
1. **Keep ISR SHORT** - Read/write data, update buffers, return fast
2. **Use VOLATILE** - All ISR-shared variables MUST be volatile
3. **Disable UDRIE1** - When TX buffer empty (prevents infinite ISR)
4. **Let hardware clear flags** - UDR1 read/write clears flags automatically

### ❌ DON'T:
1. **Call printf() in ISR** - Too slow, breaks timing
2. **Use delays in ISR** - Blocks everything
3. **Forget sei()** - Global interrupts must be enabled
4. **Leave UDRIE1 enabled** - Will cause infinite ISR loop when buffer empty

---

## 🔄 Circular Buffer Pattern (For Interrupts)

```c
// Declare (ALL must be volatile!)
volatile uint8_t rx_buffer[32];
volatile uint8_t rx_head = 0;  // ISR writes here
volatile uint8_t rx_tail = 0;  // main() reads here

// ISR writes to buffer
ISR(USART1_RX_vect) {
    rx_buffer[rx_head] = UDR1;
    rx_head = (rx_head + 1) % 32;  // Wrap around
}

// main() reads from buffer
char get_char(void) {
    while (rx_head == rx_tail);  // Wait for data
    char c = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % 32;
    return c;
}

// Check if data available
int chars_available(void) {
    return (rx_head != rx_tail);
}
```

---

## 📊 Complete Initialization Examples

### Polling Mode (Simple):
```c
void init_uart_polling(void) {
    // 1. Baud rate (9600)
    uint16_t ubrr = 207;
    UBRR1H = (ubrr >> 8);
    UBRR1L = ubrr;
    
    // 2. Frame format (8N1)
    UCSR1C = (1<<UCSZ11) | (1<<UCSZ10);
    
    // 3. Enable double speed
    UCSR1A = (1<<U2X1);
    
    // 4. Enable RX and TX (NO interrupts)
    UCSR1B = (1<<RXEN1) | (1<<TXEN1);
}
```

### Interrupt Mode (Efficient):
```c
void init_uart_interrupt(void) {
    // 1. Baud rate (9600)
    uint16_t ubrr = 207;
    UBRR1H = (ubrr >> 8);
    UBRR1L = ubrr;
    
    // 2. Frame format (8N1)
    UCSR1C = (1<<UCSZ11) | (1<<UCSZ10);
    
    // 3. Enable double speed
    UCSR1A = (1<<U2X1);
    
    // 4. Enable RX interrupt, RX, and TX
    UCSR1B = (1<<RXCIE1) | (1<<RXEN1) | (1<<TXEN1);
    // Note: UDRIE1 enabled later when data to send
    
    // 5. Enable global interrupts
    sei();  // ⚠️ REQUIRED!
}
```

---

## 🎯 Study Tips

1. **Start with Polling** (Demos 1-3) - Understand the registers first
2. **Then Learn Interrupts** (Demos 4-6) - Build on polling knowledge
3. **Compare Same Granularity** - Demo 1 vs 4, 2 vs 5, 3 vs 6
4. **Memorize UCSR1B** - This is the most important register!
5. **Practice ISR Rules** - Volatile, short code, disable UDRIE1

---

## 🔍 Debugging Checklist

### Data Not Received?
- ☑️ Is RXEN1 enabled? `UCSR1B |= (1<<RXEN1);`
- ☑️ Checking RXC1 flag? `while(!(UCSR1A & (1<<RXC1)));`
- ☑️ Baud rate correct? (207 for 9600 @ 16MHz)

### Data Not Transmitted?
- ☑️ Is TXEN1 enabled? `UCSR1B |= (1<<TXEN1);`
- ☑️ Checking UDRE1 flag? `while(!(UCSR1A & (1<<UDRE1)));`
- ☑️ Writing to UDR1? `UDR1 = data;`

### Interrupts Not Firing?
- ☑️ Called sei()? **CRITICAL!**
- ☑️ Is RXCIE1/UDRIE1 enabled?
- ☑️ Variables declared volatile?
- ☑️ ISR name correct? `ISR(USART1_RX_vect)`

---

**📚 See Main.c for complete register tables and working examples!**

*Last Updated: October 19, 2025*
