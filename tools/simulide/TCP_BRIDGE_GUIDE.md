# SimulIDE TCP Bridge Setup Guide

## 🌐 Why TCP Instead of com0com?

TCP provides:
- ✅ More reliable data transmission
- ✅ No driver issues or conflicts
- ✅ Better performance
- ✅ Cross-platform compatibility
- ✅ Easy debugging

## 📋 Setup Steps

### 1. SimulIDE Configuration
The `Simulator110.simu` file is already configured:
- SerialPort-1706: `tcp://localhost:1234`
- Baud rate: 9600
- This means SimulIDE acts as a **TCP server** on port 1234

### 2. Start the Bridge

**Option A: Using PowerShell (Recommended)**
```bash
# From workspace root
.\tools\simulide\start-tcp-bridge.bat

# Or directly
powershell -ExecutionPolicy Bypass -File tools\simulide\tcp-serial-bridge.ps1
```

**Option B: Using Python**
```bash
# Install pyserial if needed
pip install pyserial

# Run bridge
python tools\simulide\tcp-serial-bridge.py --port COM4 --baud 9600 --tcp 1234
```

### 3. Connection Flow

```
VS Code Serial Monitor (COM4, 9600)
    ↕
TCP Bridge Script
    ↕
TCP localhost:1234
    ↕
SimulIDE SerialPort-1706 (UART1)
    ↕
ATmega128 UART1 (PORTD2/D3)
```

## 🚀 Usage

**IMPORTANT: Follow this exact order!**

1. **Close VS Code Serial Monitor** if it's open (must be closed first!)
2. **Start SimulIDE** with the simulation
3. **Run the TCP bridge** (see step 2 above) - this opens COM4
4. **Open VS Code Serial Monitor** on COM4 at 9600 baud
5. **Test communication!**

**Common Error:** If you get "Access denied to COM4", close VS Code Serial Monitor and restart the bridge.

## 🔧 Customization

### Change TCP Port
Edit `Simulator110.simu`:
```xml
Port="tcp://localhost:5000"
```

Then run bridge with:
```bash
python tcp-serial-bridge.py --tcp 5000
```

### Change Serial Port
```bash
python tcp-serial-bridge.py --port COM3
```

### Change Baud Rate
Update both:
1. Firmware: `config.h` → `#define BAUD 115200`
2. Bridge: `--baud 115200`

## 🐛 Troubleshooting

**Bridge won't connect:**
- Make sure SimulIDE is running first
- Check if port 1234 is already in use: `netstat -an | findstr 1234`

**VS Code Serial Monitor won't open COM4:**
- Close any other programs using COM4
- Check Device Manager that COM4 exists

**Still garbled text:**
- Verify baud rates match everywhere (firmware, SimulIDE, bridge, VS Code)
- Try U2X=1 mode in firmware for better accuracy

## ✅ Advantages Over com0com

| Feature | com0com | TCP Bridge |
|---------|---------|------------|
| Reliability | ⚠️ Poor | ✅ Excellent |
| Setup | Complex | Simple |
| Baud Rate Issues | Common | None |
| Debugging | Difficult | Easy |
| Performance | Limited | High |

## 📝 Notes

- The bridge script shows all data flowing in both directions
- You can run multiple bridges on different TCP ports
- TCP has no baud rate limitations (virtual)
- SimulIDE's TCP mode is more stable than COM port emulation
