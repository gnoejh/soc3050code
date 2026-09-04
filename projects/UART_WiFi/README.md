# ESP-01 / Bluetooth AT Command Interface

## Overview
Dual UART system for ATmega128 demonstrating ESP-01 WiFi module or HC-05/HC-06 Bluetooth module communication with comprehensive AT command handling.

## Hardware Setup

### UART0 - ESP-01/Bluetooth Module (115200 baud)
- **PE0 (RXD0)** ← ESP TX (direct connection, 3.3V logic)
- **PE1 (TXD0)** → ESP RX (use voltage divider: 5V → 3.3V)
  - Suggested: 4.7kΩ + 10kΩ resistor divider
- **VCC** → 3.3V regulated (≥500mA for ESP-01)
- **CH_PD (EN)** → 3.3V via 10kΩ pull-up
- **RST** → 3.3V via 10kΩ pull-up
- **GND** → Common ground

### UART1 - Debug/Monitor (9600 baud)
- **PD2 (RXD1)** ← PC Serial TX
- **PD3 (TXD1)** → PC Serial RX
- Already connected via SerialPort-1706 in Simulator110.simu

## Features

### Dual UART Management
- **UART0**: High-speed ESP-01/Bluetooth communication (115200 baud)
- **UART1**: Debug monitoring and user interface (9600 baud)
- Non-blocking ring buffer implementation for both UARTs
- Interrupt-driven TX/RX on both channels

### AT Command Framework
- Automated initialization sequence
- Command queue with timeout handling
- Response parser with pattern matching
- Retry logic for failed commands
- State machine for connection management

### Operating Modes

#### 1. Initialization Mode
Automatically runs AT command sequence:
```
AT          → Test communication
AT+GMR      → Check firmware version
AT+CWMODE=1 → Set WiFi station mode (ESP-01)
```

#### 2. Interactive Mode
Available commands from debug UART:
- `b` - Enter bridge mode
- `t` - Test AT communication
- `r` - Reset module
- `i` - Get module info (IP address)

#### 3. Bridge Mode
Transparent bidirectional forwarding:
- UART1 (PC) ↔ UART0 (ESP/Bluetooth)
- Press Ctrl+C three times to exit
- Useful for manual AT command testing

## AT Command Examples

### ESP-01 WiFi Commands
```
AT+CWMODE=1                          // Station mode
AT+CWJAP="MySSID","MyPassword"       // Join AP
AT+CIFSR                             // Get IP address
AT+CIPSTART="TCP","server.com",80    // Open TCP connection
AT+CIPSEND=20                        // Send 20 bytes
AT+CIPCLOSE                          // Close connection
```

### HC-05/HC-06 Bluetooth Commands
```
AT                    // Test (should return OK)
AT+NAME=MyDevice      // Set device name
AT+BAUD4              // Set 9600 baud (HC-05)
AT+PIN1234            // Set pairing PIN
AT+VERSION            // Get firmware version
```

## Code Structure

### Interrupt Service Routines
- `USART0_RX_vect` - ESP-01 receive interrupt
- `USART0_UDRE_vect` - ESP-01 transmit interrupt
- `USART1_UDRE_vect` - Debug transmit interrupt
- `TIMER0_COMP_vect` - 1ms system tick for timeouts

### State Machine
```
STATE_INIT → STATE_WAIT_READY → STATE_TEST_AT → STATE_CHECK_VERSION
    → STATE_SET_MODE → STATE_CONNECT_AP → STATE_CONNECTED
    → STATE_BRIDGE_MODE (user controlled)
```

### Key Functions
- `uart0_*` - UART0 (ESP) communication functions
- `uart1_*` - UART1 (Debug) communication functions
- `esp_send_cmd()` - Send AT command with debug echo
- `esp_wait_for_response()` - Wait for specific response with timeout
- `esp_state_machine()` - Main state machine handler

## Building and Testing

### Build
```powershell
# In VS Code, press Ctrl+Shift+B or run:
.\tools\cli\cli-build-project.ps1 -ProjectDir "projects\Bluethooth" -SourceFile "Main.c"
```

### Simulate
```powershell
# Load into SimulIDE
.\tools\simulide\cli-simulide.ps1 -ProjectDir "projects\Bluethooth"
```

### Hardware Upload
```powershell
.\tools\cli\cli-program-project.ps1 -ProjectDir "projects\Bluethooth" -Programmer arduino -Port COM3
```

## Usage Tips

### Testing Without ESP-01
- Use bridge mode to test UART communication
- Connect loopback (PE0 ↔ PE1) to echo commands
- Monitor debug UART for transmitted data

### Debugging
- All ESP-01 traffic is echoed to debug UART
- Timeout values can be adjusted in `CMD_TIMEOUT_MS`
- Use logic analyzer on PE0/PE1 to verify signal levels

### Common Issues

**No response from module:**
- Check power supply (ESP needs stable 3.3V, ≥300mA burst)
- Verify baud rate (try 9600 if 115200 fails)
- Check voltage divider on TXD0 → ESP RX

**Garbled characters:**
- Check crystal frequency (code assumes 16 MHz)
- Verify baud rate calculation: UBRR = (F_CPU / (16 * BAUD)) - 1
- Try enabling U2X mode for better accuracy

**Module resets repeatedly:**
- Insufficient power supply capacitance
- Add 100µF bulk + 0.1µF ceramic near ESP VCC/GND

## Learning Objectives

1. **Dual UART Configuration** - Managing independent serial channels
2. **Interrupt-Driven I/O** - Non-blocking serial communication
3. **Ring Buffer Implementation** - Efficient data queuing
4. **State Machine Design** - Command sequencing and flow control
5. **Protocol Handling** - AT command framing and parsing
6. **Timeout Management** - Reliable error recovery
7. **Cross-Channel Debugging** - Using one UART to monitor another

## Extensions

### Add WiFi Connection
Uncomment and customize in `STATE_CONNECT_AP`:
```c
esp_send_cmd("AT+CWJAP=\"YourSSID\",\"YourPassword\"");
if (esp_wait_for_response("WIFI CONNECTED")) {
    uart1_puts_P(PSTR("[OK] Connected to AP\r\n"));
}
```

### Add TCP Client
```c
esp_send_cmd("AT+CIPSTART=\"TCP\",\"192.168.1.100\",8080");
esp_wait_for_response("OK");
esp_send_cmd("AT+CIPSEND=5");
esp_wait_for_response(">");
uart0_puts("HELLO");
```

### Add Sensor Telemetry
- Read ADC values
- Format as JSON/CSV
- Send via `AT+CIPSEND`
- Log to EEPROM on connection failure

## References
- ATmega128 Datasheet (USART section)
- ESP-01 AT Command Set (v2.2+)
- HC-05/HC-06 Bluetooth module datasheets
- Serial_Communications project for UART patterns
