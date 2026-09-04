# Testing Guide - ESP-01 AT Command Interface

## Quick Start Test (3 Methods)

### Method 1: Software Loopback Test (Easiest - No Circuit Changes)

This tests your code logic without needing ESP-01 connected.

**Step 1**: Temporarily modify Main.c to echo UART0 back to itself:

Add this at the end of `main()` before the while loop:
```c
// TEST MODE: Internal loopback
uart1_puts_P(PSTR("[TEST] Running in loopback mode\r\n"));
while(1) {
    // Echo UART1 to UART0
    if (uart1_available()) {
        char c = uart1_getc();
        uart0_putc(c);
        uart1_putc(c); // Echo back
    }
    // Echo UART0 to UART1
    if (uart0_available()) {
        char c = uart0_getc();
        uart1_putc(c);
    }
}
```

**Step 2**: Build and simulate:
```powershell
# Build
Ctrl+Shift+B

# Simulate
.\tools\simulide\cli-simulide.ps1 -ProjectDir "projects\Bluethooth"
```

**Step 3**: Open Serial Monitor (if SimulIDE opens SerialPort automatically) or use the TCP bridge:
```powershell
# In another terminal, run the serial client
.\tools\cli\cli-run-serial-tcp-client.ps1
```

**Step 4**: Type commands and verify echo:
```
Type: AT
Should see: AT echoed back twice (once from your typing, once from loopback)
```

---

### Method 2: Simulator with Mock AT Responses

**Option A: Edit Simulator ESP-01 to auto-respond**

SimulIDE's ESP-01 component has `Debug="true"` mode that shows serial traffic. You can:

1. Open `Simulator110.simu` in SimulIDE
2. Right-click ESP-01 component
3. Check if it has "Serial Monitor" option
4. The ESP-01 will show traffic but may not auto-respond

**Option B: Python AT Command Emulator (Recommended)**

Create a Python script that acts as ESP-01:

**File**: `tools/test_esp_emulator.py`
```python
import serial
import time
import sys

# Adjust COM port to match your SerialPort in SimulIDE
PORT = 'COM10'  # Change to your virtual COM port
BAUD = 115200

print(f"ESP-01 Emulator starting on {PORT} @ {BAUD} baud...")
try:
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    print("Connected! Waiting for AT commands...\n")
except:
    print(f"ERROR: Cannot open {PORT}")
    print("Check that SimulIDE is running and SerialPort is configured")
    sys.exit(1)

while True:
    line = ser.readline()
    if not line:
        continue
    
    text = line.decode('utf-8', errors='ignore').strip()
    if not text:
        continue
    
    print(f"RX: {text}")
    
    # Simulate ESP-01 responses
    if text == 'AT':
        ser.write(b'OK\r\n')
        print("TX: OK")
        
    elif text.startswith('AT+GMR'):
        ser.write(b'AT version:1.7.4.0(May 11 2020 19:13:04)\r\n')
        ser.write(b'SDK version:3.0.4(9532cba)\r\n')
        ser.write(b'OK\r\n')
        print("TX: Version info + OK")
        
    elif text.startswith('AT+CWMODE'):
        ser.write(b'OK\r\n')
        print("TX: OK")
        
    elif text.startswith('AT+CWJAP'):
        time.sleep(0.5)  # Simulate connection delay
        ser.write(b'WIFI CONNECTED\r\n')
        ser.write(b'WIFI GOT IP\r\n')
        ser.write(b'OK\r\n')
        print("TX: WIFI CONNECTED + OK")
        
    elif text.startswith('AT+CIFSR'):
        ser.write(b'+CIFSR:STAIP,"192.168.1.100"\r\n')
        ser.write(b'+CIFSR:STAMAC,"18:fe:34:a1:23:45"\r\n')
        ser.write(b'OK\r\n')
        print("TX: IP info + OK")
        
    elif text.startswith('AT+RST'):
        ser.write(b'OK\r\n')
        time.sleep(1)
        ser.write(b'\r\nready\r\n')
        print("TX: OK + ready")
        
    else:
        ser.write(b'ERROR\r\n')
        print(f"TX: ERROR (unknown command)")
```

**Run it**:
```powershell
python tools/test_esp_emulator.py
```

---

### Method 3: Bridge Mode Interactive Test (Real AT Commands)

**Step 1**: Build and run in SimulIDE

**Step 2**: Wait for initialization to complete (watch debug output)

**Step 3**: When you see the menu:
```
Commands available:
  'b' - Enter bridge mode
  't' - Test AT
  'r' - Reset module
  'i' - Info
```

**Step 4**: Press `b` to enter bridge mode

**Step 5**: Now you can type AT commands directly:
```
AT                  → Should return: OK
AT+GMR              → Firmware version
AT+CWMODE=1         → Set WiFi station mode
AT+CWMODE?          → Query current mode
```

**Step 6**: Press `Ctrl+C` three times to exit bridge mode

---

## Understanding the Test Flow

### What You'll See (Debug UART Output)

```
========================================
  ESP-01/Bluetooth AT Interface
  ATmega128 Dual UART System
========================================
UART0 (PE0/PE1): ESP-01 @ 115200 baud
UART1 (PD2/PD3): Debug  @ 9600 baud
========================================

=== ESP-01/Bluetooth Init ===
[STATE] Testing AT...
[CMD] AT
OK                              ← Response from ESP-01
[OK] AT responded
[STATE] Checking version...
[CMD] AT+GMR
AT version:1.7.4.0...          ← Version info
OK
[OK] Version check complete
[STATE] Setting mode...
[CMD] AT+CWMODE=1
OK
[OK] Mode set to Station

[READY] Module initialized!
Commands available:
  'b' - Enter bridge mode
  't' - Test AT
  'r' - Reset module
  'i' - Info
```

### Common AT Commands to Test

```
# Basic Communication
AT                              → Test connection
AT+GMR                          → Get version
AT+RST                          → Soft reset

# WiFi Mode (ESP-01)
AT+CWMODE?                      → Query mode
AT+CWMODE=1                     → Station mode
AT+CWMODE=2                     → AP mode
AT+CWMODE=3                     → Station+AP

# WiFi Connection (ESP-01)
AT+CWLAP                        → List available APs
AT+CWJAP="SSID","password"      → Join AP
AT+CWQAP                        → Disconnect from AP
AT+CIFSR                        → Get IP address

# TCP/IP (ESP-01)
AT+CIPSTART="TCP","google.com",80     → Open connection
AT+CIPSEND=20                   → Send 20 bytes
AT+CIPCLOSE                     → Close connection

# Bluetooth (HC-05/HC-06)
AT                              → Test (9600 baud)
AT+NAME=MyDevice                → Set name
AT+BAUD4                        → Set 9600 baud
AT+PIN1234                      → Set PIN
```

---

## Troubleshooting

### "No response from module"

**Check 1**: Is ESP-01 connected in simulator?
- Open Simulator110.simu
- Look for ESP-01 component position
- Verify wires connect to PE0 (RXD0) and PE1 (TXD0)

**Check 2**: Check baud rate match
- ESP-01 default: 115200 (some older: 9600)
- Your code: 115200 (line 48: `#define ESP_BAUD 115200UL`)
- To test 9600: Change to `#define ESP_BAUD 9600UL` and rebuild

**Check 3**: Enable ESP-01 SerialMon in simulator
- Edit Simulator110.simu line 143:
- Change `SerialMon="false"` to `SerialMon="true"`

### "Timeout waiting for OK"

**Solution**: Increase timeout
```c
#define CMD_TIMEOUT_MS 5000   // Line 51: Change from 2000 to 5000
```

### "Garbled characters"

**Check 1**: Verify F_CPU matches your crystal
```c
#define F_CPU 16000000UL      // Line 41: Must match hardware
```

**Check 2**: Try U2X mode for better baud accuracy
Add to `uart0_init()`:
```c
UCSR0A = (1 << U2X0);  // Before setting UBRR0
// Then use: ubrr = (F_CPU / (8UL * ESP_BAUD)) - 1;
```

---

## Circuit Wiring (For Hardware Testing)

### ESP-01 Module Pinout
```
    [=====]
    | ANT |
    [=====]
    
    VCC  ●    TX  ●
    RST  ●    CH  ●
    EN   ●    IO2 ●
    IO0  ●    IO0 ●
    GND  ●    RX  ●
```

### Connections to ATmega128
```
ESP-01          ATmega128       Notes
------          ---------       -----
VCC     →       3.3V            Regulated! ≥500mA
GND     →       GND             Common ground
TX      →       PE0 (RXD0)      Direct OK (3.3V → 5V tolerant)
RX      ←       PE1 (TXD0)      Need voltage divider!
CH_PD   →       3.3V            10kΩ pull-up
RST     →       3.3V            10kΩ pull-up
IO0     →       3.3V            10kΩ pull-up (boot mode)
IO2     →       3.3V            10kΩ pull-up
```

### Voltage Divider for PE1 → ESP RX (5V → 3.3V)
```
PE1 (5V) ──[4.7kΩ]──┬── ESP RX (3.3V max)
                     │
                  [10kΩ]
                     │
                    GND

Output = 5V × (10k / (4.7k + 10k)) = 3.4V ✓
```

---

## Next Steps After Basic Test

1. **Add WiFi Connection**:
   - Edit `STATE_CONNECT_AP` case
   - Add your SSID and password
   - Test connection and IP acquisition

2. **Add TCP Client**:
   ```c
   esp_send_cmd("AT+CIPSTART=\"TCP\",\"192.168.1.100\",8080");
   esp_wait_for_response("OK");
   ```

3. **Send Sensor Data**:
   - Read ADC values
   - Format as JSON
   - Send via AT+CIPSEND

4. **Add EEPROM Logging**:
   - Log data when WiFi down
   - Upload buffered data on reconnect

---

## Quick Reference: Serial Monitor Setup

### Using SimulIDE SerialPort
The simulator has `SerialPort-1706` connected to PD2/PD3 (UART1).
- Right-click SerialPort → Properties
- Set Port: COM10 (or your virtual COM)
- Baud: 9600

### Using External Terminal
```powershell
# Install Python pyserial if needed
pip install pyserial

# Create simple monitor
python -c "import serial; s=serial.Serial('COM10',9600); [print(s.readline().decode(),end='') for _ in iter(int,1)]"
```

### Using VS Code Serial Monitor Extension
1. Install "Serial Monitor" extension
2. Click Serial Monitor icon in status bar
3. Select COM10, 9600 baud
4. Click "Start Monitoring"

---

## Expected Test Results

✅ **Success Indicators**:
- Debug UART shows initialization messages
- AT commands receive "OK" responses
- State machine progresses through states
- Bridge mode allows direct command entry
- Timeouts don't trigger (or retry succeeds)

❌ **Failure Indicators**:
- "[TIMEOUT] Retrying..." loops forever
- No output on debug UART
- Garbled characters
- Immediate crash/reset

---

## Files to Check

- `Main.c` - Your firmware
- `config.h` - F_CPU and includes
- `Simulator110.simu` - ESP-01 wiring
- `README.md` - Architecture overview
- Build output: `Main.hex` - Flash this to hardware

Ready to test! Start with **Method 1 (Loopback)** to verify UART code works, then move to **Method 2 (Python Emulator)** for realistic AT responses.
