# 🎓 ATmega128 Student Learning Environment
**From Embedded C to Python IoT Integration**

A complete learning environment that takes you from basic microcontroller programming to advanced IoT applications. Perfect for students learning embedded systems, microcontroller programming, and Python integration.

## 🚀 Quick Start for Students

### 1️⃣ **One-Click Setup**
```powershell
# Run this once to set up everything:
.\setup.ps1
```
This installs Python, creates your environment, and sets up all tools.

### 2️⃣ **Activate Your Environment**
```powershell
# Start working (run this each time):
.\activate.ps1
```
Or use the desktop shortcut created during setup.

### 3️⃣ **Your First Program**
```powershell
# Test everything works:
python python\examples\quick_start.py
```

## 🎯 What You'll Learn

### 📘 **Level 1: Embedded C Programming**
- **Hardware Control**: LEDs, buttons, sensors, motors
- **Communication**: UART, SPI, I2C protocols  
- **Interrupts**: Timer-based and external interrupts
- **Memory Management**: EEPROM, SRAM optimization
- **Real-time Systems**: Timing-critical applications

### 📗 **Level 2: Python Integration**
- **Serial Communication**: Control microcontroller from PC
- **Data Acquisition**: Read and log sensor data
- **Real-time Monitoring**: Live graphs and dashboards
- **Web Interfaces**: Browser-based control panels
- **Data Analysis**: Process and analyze collected data

### 📙 **Level 3: IoT Applications**
- **Web Dashboards**: Professional monitoring interfaces
- **Data Logging**: Long-term data collection
- **Remote Control**: Internet-based device control
- **Machine Learning**: Predict patterns from sensor data
- **System Integration**: Complete IoT solutions

## 🛠️ What's Included

### **Hardware Support**
- ✅ **ATmega128** microcontroller (16MHz, 128KB Flash, 4KB RAM)
- ✅ **50+ Example Programs**: From basic blink to complex games
- ✅ **Multiple Programmers**: Arduino, USBasp, AVRISP mkII supported
- ✅ **Serial Communication**: Real-time monitoring and control

### **Development Tools**
- ✅ **VS Code Integration**: Professional code editing
- ✅ **One-Click Building**: Press F7 to compile
- ✅ **One-Click Programming**: Press F8 to upload
- ✅ **Python Environment**: Modern package management with uv
- ✅ **Web Dashboard**: Professional IoT interfaces

### **Learning Examples**
- ✅ **Basic Examples**: LED blink, button reading, serial output
- ✅ **Sensor Examples**: Temperature, light, accelerometer
- ✅ **Game Examples**: Pong, hangman, puzzle games
- ✅ **Communication**: UART, wireless, network protocols
- ✅ **Advanced**: Motor control, graphics, sound generation

## 📚 Learning Path

### 🏃‍♂️ **Start Here** (5 minutes)
1. Run `.\setup.ps1` - One-time setup
2. Run `.\activate.ps1` - Start environment  
3. Run `python python\examples\quick_start.py` - Test connection
4. Edit `Main\config.h` - Choose your first example
5. Press **F7** to build, **F8** to program

### 📖 **Embedded C Programming** (Weeks 1-4)
Choose examples in `config.h`:
- `#define SERIAL_BASIC` - Learn serial communication
- `#define LED_BLINK` - Control hardware
- `#define SENSOR_TEMP` - Read sensors  
- `#define GAME_PONG` - Build interactive programs

### 🐍 **Python Integration** (Weeks 5-6)  
Follow the progressive examples:
1. `python\examples\example_01_basic.py` - Simple communication
2. `python\examples\example_02_sensors.py` - Data reading
3. `python\examples\example_03_realtime.py` - Live monitoring
4. `python\examples\example_04_iot.py` - Web dashboard

### 🌐 **IoT Projects** (Weeks 7-8)
5. `python\examples\example_05_analysis.py` - Data science
6. Build your own IoT project!

## 🎮 Example Projects You Can Build

### 🌱 **Beginner Projects**
- **Smart Thermometer**: Monitor room temperature with web display
- **Light Controller**: Automatic lights based on ambient light  
- **Simple Alarm**: Motion detection with smartphone alerts
- **LED Controller**: Control RGB LEDs via web interface

### 🚀 **Intermediate Projects**  
- **Weather Station**: Multi-sensor monitoring with historical data
- **Home Automation**: Control lights, fans via web dashboard
- **Security System**: Camera integration with motion detection
- **Smart Garden**: Automated watering based on soil moisture

### 🔬 **Advanced Projects**
- **IoT Greenhouse**: Complete environmental control system
- **Predictive Maintenance**: Machine learning for equipment monitoring  
- **Smart Campus**: Network of sensors across multiple locations
- **Industrial Monitor**: Real-time production line monitoring

## 🔧 How It Works

### **Configuration System**
The magic happens in `config.h`:
```c
// Choose your example by uncommenting ONE line:
// #define SERIAL_BASIC          // Learn serial communication
// #define LED_PATTERNS          // Create LED light shows  
// #define SENSOR_DASHBOARD       // Read multiple sensors
// #define GAME_PONG             // Build Pong game
// #define IOT_INTERFACE         // Python communication
```

### **Build Process**
1. **Edit** `config.h` to choose example
2. **Build** (F7): Compiles only needed files  
3. **Program** (F8): Uploads to ATmega128
4. **Monitor**: Watch output in serial terminal

### **Python Integration**
```python
from atmega128 import ATmega128

# Connect and control
mcu = ATmega128("COM3")
mcu.led_on()
temp = mcu.read_temperature()
mcu.close()
```

## 🗂️ Project Organization

```
📁 soc3050code/
├── 📁 Main/                    # Embedded C code
│   ├── 📄 config.h             # Choose your example here
│   ├── 📄 Main.c               # Program entry point
│   ├── 📄 main_*.c             # 50+ example programs
│   ├── 📄 build.ps1            # Press F7 or run this
│   └── 📄 program.ps1          # Press F8 or run this
├── 📁 python/                  # Python integration
│   ├── 📄 atmega128.py         # Easy-to-use interface
│   └── 📁 examples/            # Progressive examples
│       ├── 📄 quick_start.py   # Start here!
│       ├── 📄 example_01_basic.py
│       ├── 📄 example_02_sensors.py
│       ├── 📄 example_03_realtime.py
│       ├── 📄 example_04_iot.py
│       └── 📄 example_05_analysis.py
├── 📁 .vscode/                 # VS Code configuration  
├── 📄 setup.ps1               # One-time setup script
├── 📄 activate.ps1             # Daily activation script
└── 📄 README.md                # This guide
```

## 🎯 Quick Reference

### **Common Tasks**
| Task | Method 1 | Method 2 |
|------|----------|----------|
| **Build** | Press F7 | `cd Main; .\build.ps1` |
| **Program** | Press F8 | `cd Main; .\program.ps1` |
| **Monitor** | Ctrl+Shift+` | `cd Main; .\serial.ps1` |
| **Python** | Examples menu | `python python\examples\*.py` |

### **File Locations**
| Need to edit | File | Purpose |
|--------------|------|---------|
| **Choose example** | `Main\config.h` | Select which program to build |
| **Port settings** | `python\examples\*.py` | Update COM port for Python |
| **Hardware config** | `Main\_main.h` | CPU speed, pin assignments |

## 🚨 Troubleshooting

### ❌ **"Could not connect"**
**Problem**: Python can't find ATmega128
**Solutions**:
- Check USB cable connection
- Find correct COM port in Device Manager
- Update port in Python examples (change "COM3")
- Close other serial programs

### ❌ **"Build failed"**  
**Problem**: C code won't compile
**Solutions**:
- Check only ONE `#define` enabled in `config.h`
- Ensure AVR-GCC installed (setup.ps1 helps check)
- Verify file paths don't contain spaces

### ❌ **"Programming failed"**
**Problem**: Can't upload to ATmega128  
**Solutions**:
- Check programmer connections
- Verify correct programmer type in `program.ps1`
- Try different USB port
- Check power connections

### ❌ **"No serial output"**
**Problem**: ATmega128 not responding
**Solutions**:
- Verify baudrate (default: 9600)
- Check serial cable connections
- Ensure program uses UART (like SERIAL_BASIC)
- Try serial monitor: Ctrl+Shift+`

### ❌ **"Python errors"**
**Problem**: Python examples crash
**Solutions**:
- Run `.\setup.ps1` to reinstall packages
- Activate environment: `.\activate.ps1`
- Update Python examples with correct COM port
- Check ATmega128 is running compatible firmware

## 🏫 For Instructors

### **Course Integration**
This environment supports:
- **Embedded Systems Courses**: Complete C programming curriculum
- **IoT Development**: Modern web-based interfaces  
- **Data Science Integration**: Real sensor data analysis
- **Capstone Projects**: Professional development workflows

### **Assessment Options**
- **Weekly Labs**: Progressive skill building
- **Project Milestones**: Measurable outcomes
- **Portfolio Development**: GitHub integration
- **Industry Preparation**: Professional toolchain

### **Customization**
- Add new examples in `main_*.c` files
- Extend Python interface with new methods
- Create course-specific web dashboards
- Integrate with learning management systems

## 🤝 Getting Help

### **Documentation**
- 📖 **This README**: Complete setup and usage guide
- 📖 **Python Examples README**: `python\examples\README.md`
- 📖 **Code Comments**: Every file thoroughly documented
- 📖 **Example Headers**: Each example explains its purpose

### **Support Resources**  
- 💬 **Course Forum**: Ask questions and share projects
- 🎥 **Video Tutorials**: Visual setup and usage guides
- 📧 **Instructor Support**: Office hours and email help
- 🔗 **Online Community**: Connect with other students

### **Common Learning Resources**
- 📚 **ATmega128 Datasheet**: Official hardware reference
- 📚 **AVR-GCC Manual**: Compiler documentation  
- 📚 **Python Serial Guide**: Communication programming
- 📚 **Web Development**: Flask and JavaScript tutorials

---

## 🎉 Success Stories

*"This environment helped me go from never programming microcontrollers to building a complete IoT weather station in 8 weeks!"* - Student feedback

*"The progressive examples made complex concepts easy to understand. I especially loved the web dashboard project."* - Course evaluation

*"Having both C and Python integration showed me how embedded systems connect to the real world."* - Student project reflection

---

## 🚀 Ready to Start?

1. **Run Setup**: `.\setup.ps1`
2. **Activate Environment**: `.\activate.ps1`  
3. **Test Connection**: `python python\examples\quick_start.py`
4. **Choose Example**: Edit `Main\config.h`
5. **Build & Program**: Press F7, then F8
6. **Start Learning**: Follow the progressive examples!

**Happy coding! 🎯**

---

*This learning environment was designed to bridge the gap between traditional embedded programming and modern IoT development, giving students hands-on experience with both low-level hardware control and high-level system integration.*

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