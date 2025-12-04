# Centralized Interrupt Management System

## Problem Solved

Previously, multiple shared libraries (_uart.c, _interrupt.c, etc.) defined their own ISR (Interrupt Service Routine) handlers, causing linker conflicts when multiple libraries were used together:

```
error: multiple definition of '__vector_30'
```

This occurred because each library was defining the same interrupt vector (e.g., USART1_RX_vect).

## Solution: Centralized Interrupt Manager

We implemented a centralized interrupt management system that:

1. **Single ISR Definition**: Only `_interrupt_manager.c` defines the actual ISR handlers
2. **Callback Registration**: Libraries register callback functions instead of defining ISRs
3. **Priority Support**: Callbacks can have priorities for execution order
4. **Scalable Architecture**: Easy to add new interrupt sources without conflicts

## Architecture

### Core Files
- `shared_libs/_interrupt_manager.h` - Header with callback registration functions
- `shared_libs/_interrupt_manager.c` - Implementation with actual ISR definitions
- Modified `shared_libs/_uart.c` - Uses callback registration instead of direct ISR

### Usage Pattern

**Old Approach (Caused Conflicts):**
```c
// In _uart.c
ISR(USART1_RX_vect) {
    // UART handling code
}

// In _interrupt.c  
ISR(USART1_RX_vect) {  // CONFLICT!
    // Other handling code
}
```

**New Approach (No Conflicts):**
```c
// In _uart.c
void uart_rx_interrupt_handler(void) {
    // UART handling code
}

void Uart1_init(...) {
    // Register callback with priority
    register_uart1_rx_callback(uart_rx_interrupt_handler, 10);
}

// In _interrupt_manager.c - Only one ISR definition
ISR(USART1_RX_vect) {
    dispatch_uart1_rx_callbacks();  // Calls all registered callbacks
}
```

## Benefits

1. **No More ISR Conflicts**: Multiple libraries can use interrupts simultaneously
2. **Maintainable**: Easy to add new interrupt sources
3. **Flexible**: Callbacks can be registered/unregistered dynamically
4. **Priority Support**: Critical handlers can run first
5. **Educational**: Students can see how real embedded systems manage interrupts

## Example Usage

```c
#include "_interrupt_manager.h"

// Define your interrupt handler
void my_uart_handler(void) {
    // Your interrupt handling code
}

int main(void) {
    // Register your handler with priority 5
    register_uart1_rx_callback(my_uart_handler, 5);
    
    // Initialize UART (will register its own internal callbacks)
    Uart1_init(9600);
    
    // Enable global interrupts
    sei();
    
    // Your main code...
}
```

## Build System Integration

The build system automatically includes the interrupt manager when building projects that need it. The modified `cli-build-project.ps1` detects interrupt usage and includes the necessary files.

## Testing

Successfully tested with:
- ✅ Serial communication at 9600 baud
- ✅ Multiple interrupt handlers in same project  
- ✅ VS Code Serial Monitor integration
- ✅ Build system automation

This solution provides a permanent, scalable foundation for complex embedded projects using multiple interrupt sources.