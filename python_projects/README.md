# Python Projects for ATmega128

**Python companion projects for ATmega128 embedded systems programming**

© 2025 Prof. Hong Jeong, IUT (Inha University in Tashkent)  
SOC 3050 - Embedded Systems and Applications

---

## Overview

This directory contains Python projects that interface with ATmega128 C projects. Students learn the complete embedded systems stack: from bare-metal C programming to AI-powered Python applications.

## Learning Progression

```
C Programming (ATmega128) → Python Interface → Data Analysis → AI/ML
```

---

## Project Structure

Each Python project mirrors its C counterpart:

```
python_projects/
├── ProjectName/
│   ├── main.py              # Main demonstration program
│   ├── lab.py               # Student laboratory exercises
│   ├── config.py            # Project configuration
│   ├── requirements.txt     # Python dependencies
│   ├── run.bat              # Quick launcher (Windows)
│   └── README.md            # Project documentation
```

---

## Available Projects

### 📡 **Serial_Communications**
**Paired with:** `projects/Serial_Communications/`

- Interactive serial terminal
- Command-response protocols
- Data logging
- LED control from Python

**Skills:** Serial communication, protocol design

---

### 📊 **ADC_Basic**
**Paired with:** `projects/ADC_Basic/`

- Real-time ADC plotting
- Multi-channel visualization
- Statistical analysis
- Signal filtering
- FFT frequency analysis

**Skills:** Data visualization, signal processing

---

### 🤖 **Activity_Recognition** (AI Project)
**Paired with:** `projects/I2C_Sensors_Multi/` (MPU6050)

- Machine learning with TensorFlow
- Human activity recognition
- Training data collection
- Real-time AI inference
- Fall detection

**Skills:** AI/ML, feature extraction, neural networks

---

### 🌐 **LCD_Sensor_Dashboard** (Coming Soon)
**Paired with:** `projects/LCD_Sensor_Dashboard/`

- Web-based IoT dashboard
- Flask web server
- WebSocket real-time updates
- Multi-sensor monitoring
- Cloud integration

**Skills:** Web development, IoT, REST APIs

---

### 📡 **I2C_Sensors_Multi** (Coming Soon)
**Paired with:** `projects/I2C_Sensors_Multi/`

- Multi-sensor data fusion
- Sensor calibration
- Data export (CSV/JSON)
- Advanced visualization

**Skills:** Data fusion, sensor integration

---

## Shared Libraries

### `python_libs/`

Reusable modules for all projects:

#### **`atmega128/`** - ATmega128 Communication
- `serial_com.py` - Serial interface
- `protocol.py` - Communication protocols
- `auto_detect.py` - Device auto-detection

#### **`visualization/`** - Data Visualization
- `realtime_plot.py` - Real-time plotting
- `data_logger.py` - Data logging (CSV/JSON)

#### **`analysis/`** - Data Analysis
- `signal_processing.py` - Filtering, FFT, smoothing
- `statistics.py` - Statistical analysis, outlier detection

---

## Getting Started

### Quick Start (Windows)

```batch
cd python_projects/Serial_Communications
run.bat
```

### Manual Setup

```bash
# 1. Navigate to project
cd python_projects/Serial_Communications

# 2. Create virtual environment (recommended)
python -m venv venv

# 3. Activate virtual environment
# Windows:
venv\Scripts\activate
# Linux/Mac:
source venv/bin/activate

# 4. Install requirements
pip install -r requirements.txt

# 5. Run program
python main.py
```

---

## Requirements

### Software
- Python 3.8 or later
- pip (Python package manager)

### Hardware
- ATmega128 microcontroller
- USB-Serial connection OR SimulIDE simulator
- Sensors (depending on project)

### Python Packages
Common dependencies (installed via requirements.txt):
- `pyserial` - Serial communication
- `numpy` - Numerical computing
- `matplotlib` - Plotting and visualization
- `pandas` - Data analysis
- `tensorflow` - Machine learning (AI projects)
- `scikit-learn` - Data preprocessing
- `flask` - Web server (dashboard projects)

---

## Project Pairing Guide

| Python Project | C Project | Sensor/Hardware |
|----------------|-----------|-----------------|
| Serial_Communications | Serial_Communications | UART |
| ADC_Basic | ADC_Basic | ADC channels |
| Activity_Recognition | I2C_Sensors_Multi | MPU6050 |
| LCD_Sensor_Dashboard | LCD_Sensor_Dashboard | LCD + Sensors |

---

## Educational Benefits

### For Students

1. **Complete Stack Knowledge**
   - Embedded C (ATmega128)
   - Python programming
   - AI/ML integration
   - Web development

2. **Real-World Skills**
   - IoT system design
   - Data acquisition
   - Signal processing
   - Machine learning
   - Cloud integration

3. **Industry-Ready**
   - Modern embedded AI (TinyML concepts)
   - Edge computing
   - Sensor fusion
   - Full-stack IoT

### Learning Path

```
Semester 1: C + ATmega128
    ↓
Semester 2: Python + Sensors
    ↓
Semester 3: AI/ML + IoT
    ↓
Capstone: Full System Integration
```

---

## Communication Protocol

### Standard Commands

All Python projects use standardized commands:

```python
# System
'PING'              # Test connection
'STATUS'            # Get status
'VERSION'           # Get firmware version

# GPIO
'LED_ON <n>'        # Turn LED n on
'LED_OFF <n>'       # Turn LED n off
'LED_TOGGLE <n>'    # Toggle LED n

# Sensors
'READ_ADC <ch>'     # Read ADC channel
'READ_MPU6050'      # Read accelerometer/gyro
'READ_TEMP'         # Read temperature

# Response format
'OK:data'           # Success with data
'ERROR:message'     # Error occurred
```

---

## Troubleshooting

### "No COM port found"
- Check USB connection
- Verify ATmega128 is powered
- For SimulIDE: Configure serial port in SimulIDE settings

### "Module not found"
```bash
# Ensure virtual environment is activated
# Re-install requirements
pip install -r requirements.txt
```

### "ImportError: python_libs"
```bash
# Make sure you're running from project directory
cd python_projects/ProjectName
python main.py
```

### "No response from ATmega128"
- Verify C firmware is programmed
- Check baud rate (default: 9600)
- Ensure serial program is running on ATmega128

---

## Development Guidelines

### For Students Creating New Projects

1. **Use shared libraries** - Don't reinvent the wheel
2. **Follow project structure** - Consistent organization
3. **Add documentation** - README.md for each project
4. **Include examples** - Both main.py and lab.py
5. **Error handling** - Graceful failure and helpful messages

### Example Project Template

```python
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'python_libs'))

from atmega128 import ATmega128Serial
from config import *

def main():
    with ATmega128Serial(baudrate=BAUDRATE) as mcu:
        # Your code here
        pass

if __name__ == '__main__':
    main()
```

---

## Future Projects

Coming soon:
- **Motor_Control** - PWM motor control with GUI
- **Smart_Home** - IoT home automation
- **Weather_Station** - Multi-sensor weather monitoring
- **TinyML_OnDevice** - AI running ON ATmega128
- **MQTT_IoT** - Cloud IoT integration
- **Voice_Control** - Speech recognition interface

---

## Resources

### Documentation
- Individual project README files
- Python library docstrings
- C project documentation in `projects/`

### External Resources
- [Python Official Docs](https://docs.python.org/)
- [TensorFlow Tutorials](https://www.tensorflow.org/tutorials)
- [Matplotlib Documentation](https://matplotlib.org/)
- [PySerial Documentation](https://pyserial.readthedocs.io/)

---

## Support

For questions or issues:
1. Check project README.md
2. Review code comments
3. Consult course materials
4. Ask instructor

---

## License & Credits

© 2025 Prof. Hong Jeong  
Inha University in Tashkent (IUT)  
SOC 3050 - Embedded Systems and Applications

Educational use only.

---

**Master the full embedded systems + AI stack! 🚀**

From bare-metal C to cloud AI - this is modern embedded engineering.

