# ATmega128 Development Environment
## VS Code + Microchip Studio Integration

A comprehensive embedded systems development environment that provides seamless integration between VS Code and Microchip Studio for ATmega128 microcontroller development.

## 🎯 Overview

This project demonstrates professional embedded development workflows with:
- **Dual IDE Support**: Works in both VS Code and Microchip Studio
- **Automated Build System**: One-click compilation and programming
- **Serial Communication**: Real-time monitoring and control
- **High-Level Integration**: Python, web dashboards, and IoT capabilities
- **Modular Examples**: 50+ embedded programming examples

## 🛠️ System Requirements

### Software
- **VS Code** with extensions:
  - Serial Monitor (`ms-vscode.vscode-serial-monitor`)
- **Microchip Studio 7.0** (or Atmel Studio)
- **AVRDUDESS** for direct programming
- **Python 3.x** (optional, for high-level integration)

### Hardware
- **ATmega128** microcontroller
- **Programmer**: Arduino as ISP, USBasp, AVRISP mkII, or STK500
- **Serial Interface**: USB-to-Serial adapter or built-in UART

## 📁 Project Structure

```
soc3050code/
├── Main/
│   ├── config.h              # Configuration & example selection
│   ├── Main.c                # Entry point & dispatcher
│   ├── main_*.c              # Example implementations
│   ├── _*.c/_*.h             # Hardware abstraction modules
│   ├── build.bat/.ps1        # Build scripts
│   ├── program.bat/.ps1      # Programming scripts
│   ├── serial.ps1            # Serial monitor
│   └── workflow.ps1          # Complete automation
├── .vscode/
│   ├── tasks.json            # VS Code build tasks
│   ├── keybindings.json      # Keyboard shortcuts
│   └── launch.json           # Debug configuration
├── python_interface.py       # Python integration example
├── web_dashboard.py          # Web dashboard example
└── README.md                 # This file
```

## 🚀 Quick Start

### 1. Build & Program
```bash
# Option 1: VS Code (Recommended)
F7                    # Build project
F8                    # Program ATmega128
Ctrl+Shift+M         # Open serial monitor

# Option 2: Command Line
.\build.bat          # Build
.\program.bat        # Program
.\serial.ps1         # Monitor

# Option 3: Complete Workflow
.\workflow.ps1       # Build → Program → Monitor (all-in-one)
```

### 2. Select Examples
Edit `config.h` and uncomment desired example:
```c
// Uncomment ONE example at a time
//#define BLINK_PORT                    // LED blinking patterns
#define SERIAL_POLLING_ECHO            // Serial echo test
//#define GAME_HANGMAN                  // Interactive hangman game
//#define GRAPHICS_BOUNCING_BALL        // LCD graphics demo
```

## 📚 Available Examples

### 🔌 **Serial Communication**
- `SERIAL_POLLING_ECHO` - Echo received characters
- `SERIAL_POLLING_STRING` - String communication
- `SERIAL_INTERRUPT_RX` - Interrupt-driven reception
- `SERIAL_INTERRUPT_TX` - Interrupt-driven transmission

### 💡 **LED Control**
- `BLINK_PORT` - Simple LED blinking
- `BLINK_PIN` - Rotating LED pattern with button control
- `PORT_ROTATION` - Sequential LED patterns

### 🎮 **Games**
- `GAME_HANGMAN` - Word guessing game via UART
- `GAME_OBSTACLE` - Obstacle avoidance game
- `GAME_PUZZLE` - Interactive puzzle game
- `GAME_PONG_UART_CONTROL` - Pong controlled via serial

### 📺 **Graphics (LCD)**
- `GRAPHICS_BASICS` - Basic LCD operations
- `GRAPHICS_BOUNCING_BALL` - Animated bouncing ball
- `GRAPHICS_MOVING_SQUARE` - Moving geometric shapes
- `GRAPHICS_SINE_WAVE` - Mathematical curve plotting

### ⚙️ **Motors**
- `MOTORS_FULLSTEP` - Stepper motor full-step control
- `MOTORS_HALFSTEP` - Stepper motor half-step control
- `MOTORS_SERVO` - Servo motor positioning
- `MOTORS_PWM_FAST` - DC motor speed control

### 🔊 **Sound Generation**
- `SOUND` - Basic tone generation
- `SOUND_ATARI` - Retro Atari-style sounds
- `SOUND_TWINKLE` - Musical melody playback

### ⏰ **Timers**
- `TIMER_COUNTER` - Basic timer/counter operations
- `TIMER_CTC` - Clear Timer on Compare mode
- `TIMER_FASTPWM` - Fast PWM generation
- `TIMER_NORMAL` - Normal timer mode

### 🔔 **Interrupts**
- `INTERRUPT_EXTERNAL` - External interrupt handling
- `INTERRUPT_TIMER` - Timer-based interrupts
- `INTERRUPT_EXT_TIMER` - Combined external/timer interrupts

### 📊 **Analog Sensors**
- `ADC_POLLING` - Basic ADC reading
- `ADC_INTERRUPT` - Interrupt-driven ADC
- `CDS` - Light sensor reading
- `JOYSTICK` - Analog joystick input

### 💾 **Memory Operations**
- `MEMORY_EEPROM` - EEPROM read/write operations
- `MEMORY_PROGRAM` - Program memory access

### 🌐 **IoT Features**
- `IOT` - Internet connectivity basics

## 🔧 Build System

### Automated Compilation
The build system automatically includes required source files based on active configuration:

```batch
# build.bat - Includes all common modules
Main.c main_serial.c main_blink.c main_game_*.c 
_uart.c _init.c _interrupt.c _timer2.c _adc.c _glcd.c _port.c
```

### Memory Usage Examples
- **Blink**: ~228 bytes
- **Serial Echo**: ~3,840 bytes  
- **Hangman Game**: ~6,270 bytes

## 📡 Programming Options

### AVRDUDE Direct Programming
```bash
# Basic programming
.\program.bat

# Advanced options
.\program.ps1 -ComPort COM5 -Programmer usbasp
.\program.ps1 -ListPorts                        # Show available ports
.\program.ps1 -Help                            # Show all options
```

### Supported Programmers
- `arduino` - Arduino as ISP
- `usbasp` - USBasp programmer
- `avrisp` - AVRISP mkII
- `stk500v2` - STK500 v2
- `dragon_isp` - AVR Dragon

## 📱 High-Level Integration

### Python Interface
```python
# Basic communication
from python_interface import ATmega128Interface
atmega = ATmega128Interface('COM3')
atmega.send_command("LED_ON")
data = atmega.read_data()
```

### Web Dashboard
```bash
# Start web interface
python web_dashboard.py
# Open: http://localhost:5000
```

Features:
- **Real-time monitoring** - Live sensor data
- **Interactive control** - Send commands via web
- **Data visualization** - Charts and graphs
- **Data logging** - Export to JSON/CSV

### IoT Applications

#### 1. **Home Automation**
```
ATmega128 → Serial → Python → Web Dashboard
[Sensors]     ↑       ↑        ↑
              |       |        └─ User interface
              |       └─ Logic processing  
              └─ Hardware control
```

#### 2. **Data Acquisition System**
```
ATmega128 → Serial → Python → Database/Cloud
[ADC/Sensors] → Real-time → Processing → Storage/Analysis
```

#### 3. **Remote Monitoring**
```
Mobile App → REST API → Python → Serial → ATmega128
[User] → HTTP requests → Flask → UART → [Hardware]
```

#### 4. **Machine Learning Integration**
```python
# Collect training data
sensor_data = collect_atmega_data(hours=24)

# Train model
model = train_anomaly_detection(sensor_data)

# Real-time prediction
current_reading = atmega.read_sensors()
anomaly_score = model.predict(current_reading)
if anomaly_score > threshold:
    atmega.send_command("ALERT_MODE")
```

## 🌐 Cloud Integration Examples

### ThingSpeak Integration
```python
import requests

def upload_to_thingspeak(temp, humidity):
    url = "https://api.thingspeak.com/update"
    payload = {
        'api_key': 'YOUR_API_KEY',
        'field1': temp,
        'field2': humidity
    }
    requests.post(url, data=payload)

# Use with ATmega128
atmega = ATmega128Interface()
while True:
    data = atmega.read_data()
    if data and 'TEMP' in data.parsed:
        upload_to_thingspeak(data.parsed['TEMP'], data.parsed['HUMID'])
```

### AWS IoT Integration
```python
import boto3
import json

def send_to_aws_iot(data):
    client = boto3.client('iot-data')
    client.publish(
        topic='atmega128/sensors',
        qos=1,
        payload=json.dumps(data)
    )
```

### MQTT Publishing
```python
import paho.mqtt.client as mqtt

def publish_sensor_data(topic, data):
    client = mqtt.Client()
    client.connect("mqtt.broker.com", 1883, 60)
    client.publish(topic, json.dumps(data))
    client.disconnect()
```

## 📊 Data Processing Examples

### Real-time Plotting
```python
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def animate_sensor_data():
    fig, ax = plt.subplots()
    times, temps = [], []
    
    def update(frame):
        data = atmega.read_data()
        if data and 'TEMP' in data.parsed:
            times.append(datetime.now())
            temps.append(data.parsed['TEMP'])
            
            ax.clear()
            ax.plot(times[-50:], temps[-50:])  # Last 50 points
            ax.set_title('Real-time Temperature')
    
    ani = animation.FuncAnimation(fig, update, interval=1000)
    plt.show()
```

### Data Analysis
```python
import pandas as pd
import numpy as np

def analyze_sensor_trends():
    # Load logged data
    df = pd.read_json('atmega128_log.json')
    
    # Extract temperature data
    temps = [entry['parsed']['TEMP'] for entry in df 
             if 'TEMP' in entry.get('parsed', {})]
    
    # Statistical analysis
    mean_temp = np.mean(temps)
    std_temp = np.std(temps)
    
    # Trend detection
    trend = np.polyfit(range(len(temps)), temps, 1)[0]
    
    return {
        'mean': mean_temp,
        'std': std_temp,
        'trend': 'increasing' if trend > 0 else 'decreasing'
    }
```

## 🤖 Advanced Applications

### Machine Learning Pipeline
```python
from sklearn.ensemble import IsolationForest
from sklearn.preprocessing import StandardScaler

def setup_anomaly_detection():
    # Collect baseline data
    normal_data = collect_normal_sensor_data(days=7)
    
    # Prepare model
    scaler = StandardScaler()
    model = IsolationForest(contamination=0.1)
    
    # Train
    scaled_data = scaler.fit_transform(normal_data)
    model.fit(scaled_data)
    
    return model, scaler

def monitor_for_anomalies():
    model, scaler = setup_anomaly_detection()
    
    while True:
        current_data = atmega.read_sensors()
        scaled_data = scaler.transform([current_data])
        
        if model.predict(scaled_data)[0] == -1:
            atmega.send_command("ANOMALY_DETECTED")
            alert_admin("Sensor anomaly detected!")
```

### Computer Vision Integration
```python
import cv2

def camera_controlled_atmega():
    cap = cv2.VideoCapture(0)
    
    while True:
        ret, frame = cap.read()
        
        # Object detection
        objects = detect_objects(frame)
        
        # Control ATmega128 based on vision
        if 'person' in objects:
            atmega.send_command("PERSON_DETECTED")
        elif 'vehicle' in objects:
            atmega.send_command("VEHICLE_DETECTED")
        else:
            atmega.send_command("AREA_CLEAR")
```

## 🔧 Troubleshooting

### Build Issues
```bash
# Clean build files
Remove-Item *.o,*.elf,*.hex,*.map -ErrorAction SilentlyContinue

# Check for missing dependencies
.\build.ps1  # Shows which modules are included
```

### Programming Issues
```bash
# List available COM ports
.\program.ps1 -ListPorts

# Try different programmer
.\program.ps1 -Programmer usbasp

# Verify connection
.\program.ps1 -ComPort COM3 -Programmer arduino
```

### Serial Communication Issues
```bash
# Test serial connection
.\serial.ps1 -Port COM3 -BaudRate 9600

# Check hardware loopback
# Connect TX to RX pins temporarily
```

## 📝 Development Workflow

### 1. **Example Development**
```bash
1. Edit config.h → Enable desired example
2. Press F7 → Build automatically
3. Press F8 → Program ATmega128
4. Press Ctrl+Shift+M → Monitor serial output
5. Test and iterate
```

### 2. **Python Integration**
```bash
1. Develop ATmega128 firmware
2. Create Python interface script
3. Test serial communication
4. Build high-level application
5. Deploy and monitor
```

### 3. **Web Dashboard**
```bash
1. Set up sensor data format in ATmega128
2. Configure Python web server
3. Design HTML/JavaScript interface  
4. Test real-time communication
5. Deploy to production server
```

## 🎯 Key Benefits

### Development Environment
- ✅ **Dual IDE Support** - VS Code + Microchip Studio
- ✅ **One-Click Building** - F7 keyboard shortcut
- ✅ **Direct Programming** - AVRDUDE integration
- ✅ **Real-time Monitoring** - Serial communication
- ✅ **Error Highlighting** - Integrated problem detection

### Scalability
- ✅ **Modular Design** - Easy to add new examples
- ✅ **Python Integration** - High-level processing
- ✅ **Web Interfaces** - Remote monitoring/control
- ✅ **Cloud Connectivity** - IoT platform integration
- ✅ **Machine Learning** - AI-powered applications

### Educational Value
- ✅ **50+ Examples** - Comprehensive learning resource
- ✅ **Progressive Complexity** - From basic I/O to IoT
- ✅ **Real-world Applications** - Practical project ideas
- ✅ **Professional Workflow** - Industry-standard tools

## 🚀 Future Enhancements

### Planned Features
- [ ] **Docker Integration** - Containerized development environment
- [ ] **Unit Testing** - Automated firmware testing
- [ ] **CI/CD Pipeline** - GitHub Actions integration
- [ ] **Mobile App** - React Native control interface
- [ ] **Voice Control** - Amazon Alexa/Google Assistant
- [ ] **Bluetooth/WiFi** - Wireless communication modules

### Community Contributions
- [ ] Additional sensor examples
- [ ] New game implementations
- [ ] Advanced graphics demos
- [ ] IoT protocol implementations
- [ ] Machine learning examples

## 📖 References

### Documentation
- [ATmega128 Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/doc2467.pdf)
- [AVRDUDE Documentation](https://avrdudes.github.io/avrdude/)
- [VS Code Tasks](https://code.visualstudio.com/docs/editor/tasks)

### Hardware Resources
- **Development Boards**: STK500, Arduino Mega (ATmega128 variant)
- **Programmers**: USBasp, AVRISP mkII, Arduino as ISP
- **Sensors**: Temperature, humidity, light, accelerometer, joystick

### Software Libraries
- **Python**: pySerial, Flask, matplotlib, pandas, scikit-learn
- **JavaScript**: Chart.js, Socket.IO
- **C Libraries**: AVR-LibC, util/delay.h

## 👥 Contributing

1. **Fork** the repository
2. **Create** feature branch (`git checkout -b feature/new-example`)
3. **Commit** changes (`git commit -am 'Add new sensor example'`)
4. **Push** to branch (`git push origin feature/new-example`)
5. **Create** Pull Request

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 🙏 Acknowledgments

- **Microchip Technology** - ATmega128 microcontroller and development tools
- **Atmel/Microchip Studio** - Integrated development environment
- **AVRDUDE Project** - Programming software
- **VS Code Team** - Excellent editor and extension ecosystem
- **Python Community** - Amazing libraries for embedded integration

---

**Happy Embedded Programming!** 🚀

*Transform your ATmega128 into a powerful IoT device with modern development tools.*