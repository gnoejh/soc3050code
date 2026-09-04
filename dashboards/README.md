# ATmega128 Educational Dashboard System

© 2025 Prof. Hong Jeaong, IUT (Inha University in Tashkent)  
All rights reserved for educational purposes.  
Contact: linkedin.com/in/gnoejh53

## 📊 Available Dashboards

### 1. **Launcher Dashboard** (`launcher.py`)
- **Port:** 5000
- **Purpose:** Main entry point to select and launch other dashboards
- **Features:** Dashboard selector, quick access to all tools

### 2. **Project Launcher Dashboard** (`project_launcher_dashboard.py`)
- **Port:** 5001
- **Purpose:** Educational project selection and deployment
- **Features:**
  - Browse 35+ ATmega128 projects
  - Select demo functions
  - Build and deploy to SimulIDE or physical hardware
  - Live monitoring of LED, buttons, ADC
  - Serial output viewer

### 3. **IoT Dashboard** (`iot_dashboard.py`)
- **Port:** 5002
- **Purpose:** IoT sensor monitoring and data logging
- **Features:**
  - Real-time sensor data visualization
  - Temperature, humidity, light sensors
  - Cloud integration (ThingSpeak, Firebase)
  - Historical data charts
  - Alert system

### 4. **Drone Dashboard** (`drone_dashboard.py`)
- **Port:** 5003
- **Purpose:** Drone/quadcopter telemetry and control
- **Features:**
  - Flight attitude indicators
  - GPS tracking
  - Battery monitoring
  - Motor status (4 ESCs)
  - Flight logs

### 5. **Car Dashboard** (`car_dashboard.py`)
- **Port:** 5004
- **Purpose:** Remote car control interface
- **Features:**
  - Speed and steering control
  - Ultrasonic obstacle detection
  - Camera feed (if available)
  - Path tracking
  - Gamepad support

### 6. **Robot Dashboard** (`robot_dashboard.py`)
- **Port:** 5005
- **Purpose:** Robot arm/gripper control
- **Features:**
  - Multi-servo control (6 DOF)
  - Inverse kinematics visualization
  - Position teaching/playback
  - 3D robot model
  - Sequence programming

### 7. **AI Dashboard** (`ai_dashboard.py`)
- **Port:** 5006
- **Purpose:** AI/ML integration with ATmega128
- **Features:**
  - Image classification (Vision Transformer)
  - Gesture recognition (accelerometer)
  - Voice command recognition
  - Anomaly detection
  - Model inference statistics

### 8. **Debug Dashboard** (`debug_dashboard.py`)
- **Port:** 5007
- **Purpose:** General serial debugging and monitoring
- **Features:**
  - Serial monitor
  - Command sender
  - Data logger
  - Port scanner

---

## 🚀 Quick Start

### Launch All Dashboards

```powershell
# Start the main launcher
python dashboards/launcher.py
```

Then open browser: `http://localhost:5000`

### Launch Individual Dashboard

```powershell
# Project launcher
python dashboards/project_launcher_dashboard.py

# IoT dashboard
python dashboards/iot_dashboard.py

# AI dashboard
python dashboards/ai_dashboard.py
```

---

## 📁 Directory Structure

```
dashboards/
├── __init__.py
├── launcher.py                      # Main launcher (port 5000)
│
├── common/                          # Shared components
│   ├── __init__.py
│   ├── base_dashboard.py           # Base class for all dashboards
│   ├── serial_handler.py           # Serial communication handler
│   └── data_parser.py              # Common data parsing utilities
│
├── project_launcher_dashboard.py   # Educational projects (port 5001)
├── iot_dashboard.py                # IoT monitoring (port 5002)
├── drone_dashboard.py              # Drone telemetry (port 5003)
├── car_dashboard.py                # Remote car (port 5004)
├── robot_dashboard.py              # Robot control (port 5005)
├── ai_dashboard.py                 # AI integration (port 5006)
├── debug_dashboard.py              # Debug tools (port 5007)
│
├── templates/                       # HTML templates
│   ├── base.html
│   ├── launcher.html
│   ├── project_launcher.html
│   ├── iot.html
│   ├── drone.html
│   ├── car.html
│   ├── robot.html
│   ├── ai.html
│   └── debug.html
│
├── static/                          # CSS, JS, images
│   ├── css/
│   │   ├── common.css
│   │   └── ...
│   ├── js/
│   │   ├── charts.js
│   │   ├── controls.js
│   │   └── serial_monitor.js
│   └── images/
│       └── ...
│
├── config/                          # Configuration files
│   ├── iot_config.json
│   ├── drone_config.json
│   ├── car_config.json
│   ├── robot_config.json
│   └── ai_config.json
│
└── README.md                        # This file
```

---

## 🔧 Configuration

Each dashboard has a JSON config file in `config/` directory:

```json
{
  "port": 5002,
  "baudrate": 9600,
  "auto_connect": true,
  "default_com_port": "COM3",
  "features": {
    "data_logging": true,
    "cloud_upload": false,
    "alerts": true
  }
}
```

---

## 🎓 Educational Use Cases

### **Semester 1: Fundamentals**
- Use **Project Launcher Dashboard** for basic embedded programming
- Students learn digital I/O, ADC, timers, UART

### **Semester 2: Integration**
- Use **IoT Dashboard** for sensor projects
- Learn serial protocols, data visualization

### **Semester 3: Advanced Applications**
- **Drone Dashboard** for flight control systems
- **Car Dashboard** for robotics projects
- **AI Dashboard** for edge AI applications

---

## 🌐 Network Access

Access dashboards from other devices:

```
http://<your-pc-ip>:5000    # Launcher
http://<your-pc-ip>:5001    # Project Launcher
http://<your-pc-ip>:5002    # IoT Dashboard
...
```

Find your IP:
```powershell
ipconfig
```

---

## 📦 Dependencies

Install required packages:

```powershell
pip install flask flask-socketio pyserial
```

Or use existing:
```powershell
pip install -r ../requirements.txt
```

---

## 🔗 Integration with SimulIDE

All dashboards support both:
- ✅ **SimulIDE Simulator** - Virtual COM port
- ✅ **Physical ATmega128** - USB-to-Serial adapter

Auto-detection finds the correct device automatically.

---

## 📝 Development Notes

### Adding a New Dashboard

1. Create `new_dashboard.py` in `dashboards/`
2. Inherit from `BaseDashboard` class
3. Create template in `templates/new_dashboard.html`
4. Add to launcher menu
5. Update this README

### Common Code Patterns

All dashboards follow this pattern:

```python
from common import BaseDashboard

class NewDashboard(BaseDashboard):
    def __init__(self):
        super().__init__(port=5008, name="New Dashboard")
        # Custom initialization
    
    def process_data(self, data):
        # Custom data processing
        pass
```

---

## 🐛 Troubleshooting

**Dashboard won't start:**
- Check if port is already in use
- Run `netstat -ano | findstr :5000` to check

**Can't connect to ATmega128:**
- Check COM port in Device Manager
- Verify baudrate (default 9600)
- Try manual port selection

**Serial data not showing:**
- Ensure ATmega128 firmware sends data via UART
- Check baud rate matches
- Verify TX/RX connections

---

## 📞 Support

For educational use and questions:
- **Author:** Prof. Hong Jeaong
- **Institution:** IUT (Inha University in Tashkent)
- **Contact:** linkedin.com/in/gnoejh53

---

## 📄 License

All rights reserved for educational purposes.  
© 2025 Prof. Hong Jeaong, IUT (Inha University in Tashkent)

---

**Happy Teaching! 🎓**
