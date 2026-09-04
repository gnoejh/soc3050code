# 🎓 Project Launcher Dashboard - User Guide

© 2025 Prof. Hong Jeaong, IUT (Inha University in Tashkent)  
Contact: linkedin.com/in/gnoejh53

## Overview

The **Project Launcher Dashboard** is an educational web interface for teaching ATmega128 microcontroller programming. It provides:

- **Project Browser**: Browse all 35+ educational projects
- **Device Selection**: Target SimulIDE simulator or physical hardware
- **One-Click Build & Run**: Compile and deploy projects instantly
- **Live Monitoring**: Real-time LED states, ADC values, serial output
- **Demo Selection**: Choose specific demo functions from projects

## Installation

### Prerequisites

1. **Python 3.8+** installed
2. **SimulIDE** installed in `simulators/SimulIDE_1.1.0-SR1_Win64/`
3. **AVR-GCC toolchain** installed (for building)
4. **Required Python packages**:

```bash
pip install -r requirements.txt
```

Required packages:
- Flask
- Flask-SocketIO
- pyserial
- python-socketio

### Quick Start

1. **Navigate to dashboards directory**:
   ```bash
   cd w:\soc3050code\dashboards
   ```

2. **Run the dashboard**:
   ```bash
   python project_launcher_dashboard.py
   ```

3. **Open in browser**:
   - Local: http://localhost:5001
   - Network: http://<your-ip>:5001

## User Interface Guide

### 1. Header

**Left Side**: Dashboard title and institution name  
**Right Side**: Connection status indicators
- 🟢 Green dot = Connected
- 🔴 Red dot = Disconnected
- Two indicators: WebSocket connection + Device connection

### 2. Left Sidebar: Project Browser

**Search Box**: Filter projects by name  
**Project List**: Click any project to select it
- Shows project name
- Shows number of available demos
- Active project highlighted in blue

### 3. Main Panel

#### Device Selection
- **🖥️ SimulIDE**: Virtual simulator (recommended for beginners)
- **🔌 Physical Kit**: Real ATmega128 hardware via USB
- **🔍 Scan Devices**: Auto-detect available devices

#### Project Details
When a project is selected, you'll see:
- **Project name and description**
- **🎯 Learning Objectives**: What you'll learn
- **🎮 Available Demos**: Click to select specific demo function

#### Build & Run Controls
- **🔨 Build**: Compile the project (creates .hex file)
- **▶️ Build & Run**: Compile and deploy to selected device
- **⏹️ Stop**: Stop running program (if supported)

#### 📝 Build Output
- Shows compilation progress
- Success/error messages
- Command output from AVR-GCC

### 4. Right Panel: Live Monitor

#### 💡 LED States (PORTB)
- Visual representation of 8 LEDs (bits 7-0)
- Red glow = LED ON
- Gray = LED OFF
- Updates in real-time

#### 📈 ADC Values
- Shows analog sensor readings
- Channel 0 and Channel 1 displayed
- Progress bars show value (0-1023)
- Updates continuously when ADC is used

#### 📟 Serial Monitor
- Displays serial output from ATmega128
- Send commands by typing and clicking "Send"
- Timestamps on all messages
- Auto-scrolls to latest output

## Usage Workflows

### Workflow 1: Run a Project in SimulIDE

1. **Select Device**: Click "🖥️ SimulIDE"
2. **Browse Projects**: Click a project in left sidebar (e.g., "01_Port")
3. **Select Demo** (optional): Click a demo if multiple available
4. **Build & Run**: Click "▶️ Build & Run"
5. **Watch Output**: Build output appears in console
6. **SimulIDE Opens**: Simulator launches automatically with your program
7. **Monitor Live**: See LED states, serial output in right panel

### Workflow 2: Run on Physical Hardware

1. **Connect Kit**: Plug in ATmega128 board via USB
2. **Select Device**: Click "🔌 Physical Kit"
3. **Scan Devices**: Click "🔍 Scan Devices" to detect COM port
4. **Browse Projects**: Select a project
5. **Build & Run**: Click "▶️ Build & Run"
6. **Program Uploads**: HEX file uploads to hardware
7. **Monitor Live**: See results in monitor panel

### Workflow 3: Educational Teaching

**Instructor Workflow**:
1. Project on screen for students to see
2. Select a project to demonstrate (e.g., "03_Serial_Polling_Single_Char")
3. Explain learning objectives shown in project details
4. Build project and show build output
5. Run in SimulIDE for visualization
6. Point out LED changes in monitor panel
7. Show serial communication in terminal
8. Students follow along on their own dashboards

**Student Workflow**:
1. Open dashboard on their computer
2. Follow along with instructor's selected project
3. Try different demos independently
4. Experiment with serial commands
5. Observe real-time feedback
6. Build understanding through hands-on practice

## Features Explained

### Auto-Device Detection

The dashboard automatically detects:
- **SimulIDE Virtual Ports**: Usually CNCA0/CNCB0 (Windows) or /dev/pts/* (Linux)
- **Physical Hardware**: Standard COM ports (COM1-COM20)
- **Automatic Connection**: Connects when device is found

### Real-Time Monitoring

**LED States**:
- Parses PORTB output from serial
- Recognizes patterns like "PORTB: 0xFF" or "LED[7] = ON"
- Updates visual LED display instantly

**ADC Values**:
- Parses ADC readings from serial
- Recognizes "ADC0: 512" or "Channel 1 = 768"
- Shows progress bars and numeric values

**Serial Data**:
- All serial output captured
- Timestamped for debugging
- Searchable console

### Project Parsing

Automatically extracts from Main.c:
- **Description**: From header comments
- **Learning Objectives**: From objective comments
- **Demo Functions**: Functions matching `void demo*_*(void)` pattern
- **Project Metadata**: Lab files, existing HEX files

## Troubleshooting

### Dashboard Won't Start

**Problem**: `ModuleNotFoundError: No module named 'flask'`  
**Solution**: Install requirements: `pip install -r requirements.txt`

**Problem**: `Port 5001 already in use`  
**Solution**: Close other application using port 5001, or change port in `project_launcher_dashboard.py`

### Build Fails

**Problem**: "Build script not found"  
**Solution**: Ensure `tools/cli/cli-build-project.ps1` exists

**Problem**: "AVR-GCC not found"  
**Solution**: Install AVR toolchain and add to PATH

**Problem**: Build timeout  
**Solution**: Increase timeout in config file (default 60s)

### SimulIDE Won't Launch

**Problem**: "SimulIDE not found"  
**Solution**: Check `simulators/SimulIDE_1.1.0-SR1_Win64/simulide.exe` exists

**Problem**: Circuit file error  
**Solution**: Verify `Simulator110.simu` file is valid

### Device Not Detected

**Problem**: "No devices found"  
**Solution**: 
- For SimulIDE: Launch simulator first
- For hardware: Check USB cable, install drivers
- Run "🔍 Scan Devices" multiple times

**Problem**: "Connection failed"  
**Solution**: Close other programs using the COM port

### No Serial Data

**Problem**: Monitor panel shows nothing  
**Solution**:
- Verify program uses UART/serial output
- Check baudrate (default 9600)
- Ensure device is connected
- Try reconnecting

## Configuration

Edit `config/project_launcher_config.json` to customize:

```json
{
  "port": 5001,              // Dashboard port
  "serial": {
    "baudrate": 9600,        // Serial baudrate
    "auto_reconnect": true   // Auto-reconnect on disconnect
  },
  "simulide": {
    "auto_launch": true,     // Auto-launch SimulIDE
    "wait_for_port": 3       // Wait time for virtual port (seconds)
  },
  "build": {
    "timeout": 60            // Build timeout (seconds)
  }
}
```

## Tips & Best Practices

### For Instructors

1. **Pre-build Projects**: Build all projects before class to save time
2. **Use SimulIDE First**: Demonstrate with simulator before hardware
3. **Show Build Output**: Explain compiler messages to students
4. **Live Debugging**: Use serial monitor to show debugging techniques
5. **Multiple Demos**: Use demo selection to show variations

### For Students

1. **Start Simple**: Begin with "01_Port" or "03_Serial_Polling_Single_Char"
2. **Read Objectives**: Understand what you'll learn before running
3. **Watch Monitor**: Pay attention to LED and serial output
4. **Experiment**: Try different demos, send serial commands
5. **Build First**: Always build before running to catch errors

### For Developers

1. **Add Comments**: Document your Main.c with objectives and descriptions
2. **Use Demo Functions**: Create multiple `demo*_*()` functions
3. **Serial Output**: Print PORTB and ADC values for monitoring
4. **Follow Patterns**: Match expected serial formats for auto-parsing
5. **Test Both Devices**: Verify projects work on SimulIDE and hardware

## Serial Data Format Recommendations

For best auto-parsing in the dashboard:

**LED Output**:
```c
printf("PORTB: 0x%02X\n", PORTB);  // Preferred
printf("LED States: %d%d%d%d%d%d%d%d\n", ...);
```

**ADC Output**:
```c
printf("ADC0: %d\n", adc_value);  // Preferred
printf("Channel %d = %d\n", ch, val);
```

**Sensor Output**:
```c
printf("Temperature: %d C\n", temp);
printf("Distance: %d cm\n", dist);
```

## Network Access

To access from other devices on network:

1. **Find your IP**:
   ```bash
   ipconfig  # Windows
   ifconfig  # Linux/Mac
   ```

2. **Allow through firewall** (Windows):
   - Windows Defender Firewall
   - Allow Python through firewall
   - Allow port 5001

3. **Access from devices**:
   - Students: http://<instructor-ip>:5001
   - Tablets/phones: Same URL

## Advanced Features

### Custom Build Options

Modify build script to add custom flags:
```powershell
# In tools/cli/cli-build-project.ps1
$CFLAGS = "-mmcu=atmega128 -DF_CPU=7372800UL -Os -Wall"
```

### Multiple Serial Ports

Connect to specific port:
```javascript
// In project_launcher.js
fetch('/api/device/connect', {
  method: 'POST',
  body: JSON.stringify({ port: 'COM3', baudrate: 9600 })
})
```

### Custom Parsers

Add custom data parsing in `common/data_parser.py`:
```python
def parse_custom_sensor(self, data):
    # Your custom parsing logic
    return parsed_data
```

## FAQ

**Q: Can I run multiple projects simultaneously?**  
A: No, only one project can run at a time per dashboard instance. However, you can run multiple dashboard instances on different ports.

**Q: Does this work with other AVR microcontrollers?**  
A: The dashboard is designed for ATmega128 but can be adapted for other AVRs by modifying build scripts and configs.

**Q: Can I add my own projects?**  
A: Yes! Add your project folder to `projects/` with a `Main.c` file. It will appear automatically.

**Q: How do I share this with students remotely?**  
A: Run dashboard on a server accessible to students, or have each student run it locally. For remote teaching, consider using ngrok or similar tunneling service.

**Q: Can I customize the interface?**  
A: Yes! Edit `static/css/project_launcher.css` for styling, or `templates/project_launcher.html` for layout.

## Support

For issues, questions, or contributions:
- **Contact**: Prof. Hong Jeaong
- **LinkedIn**: linkedin.com/in/gnoejh53
- **Institution**: Inha University in Tashkent

## License

© 2025 Prof. Hong Jeaong, IUT (Inha University in Tashkent)  
All rights reserved for educational purposes.

---

**Happy Teaching and Learning! 🎓**
