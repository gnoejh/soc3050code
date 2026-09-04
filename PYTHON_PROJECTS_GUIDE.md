# Python Projects for ATmega128 - Complete Guide

**Comprehensive guide to Python-ATmega128 integration**

© 2025 Prof. Hong Jeong, IUT (Inha University in Tashkent)  
SOC 3050 - Embedded Systems and Applications

---

## 🎓 Educational Vision

### Complete Learning Progression

```
┌──────────────────────┐
│  SEMESTER 1          │
│  C + ATmega128       │  ← Embedded Fundamentals
│  - GPIO, Timers      │
│  - ADC, UART         │
│  - I2C, SPI          │
└──────────┬───────────┘
           │
┌──────────▼───────────┐
│  SEMESTER 2          │
│  Python Interface    │  ← Host-Side Programming
│  - Serial Comm       │
│  - Data Viz          │
│  - Analysis          │
└──────────┬───────────┘
           │
┌──────────▼───────────┐
│  SEMESTER 3          │
│  AI/ML Integration   │  ← Modern Embedded AI
│  - TensorFlow        │
│  - Activity Recognition
│  - Edge Computing    │
└──────────┬───────────┘
           │
┌──────────▼───────────┐
│  CAPSTONE            │
│  Full System         │  ← Industry-Ready
│  ATmega128 → Python  │
│  → AI → Cloud        │
└──────────────────────┘
```

---

## 📁 Directory Structure

### Overview

```
soc3050code/
├── projects/                # C projects (ATmega128 firmware)
│   ├── Serial_Communications/
│   ├── ADC_Basic/
│   ├── I2C_Sensors_Multi/
│   └── ... (35+ projects)
│
├── python_projects/         # NEW: Python host applications
│   ├── Serial_Communications/
│   ├── ADC_Basic/
│   ├── Activity_Recognition/
│   └── README.md
│
├── shared_libs/             # C libraries (ATmega128)
│   ├── _uart.c/h
│   ├── _adc.c/h
│   └── ...
│
├── python_libs/             # NEW: Python shared libraries
│   ├── atmega128/           # Serial communication
│   ├── visualization/       # Real-time plotting
│   └── analysis/            # Signal processing
│
└── tools/
    └── avr-toolchain/       # AVR-GCC compiler
```

---

## 🚀 Python Projects

### 1. Serial_Communications

**Location:** `python_projects/Serial_Communications/`  
**Paired with:** `projects/Serial_Communications/`

#### Features
- Interactive serial terminal
- Command-response protocol
- LED control from Python
- Data logging (CSV/JSON)
- Real-time monitoring

#### Use Cases
- Learn serial communication
- Debug ATmega128 programs
- Remote device control
- Data acquisition basics

#### Quick Start
```bash
cd python_projects/Serial_Communications
python main.py
```

---

### 2. ADC_Basic

**Location:** `python_projects/ADC_Basic/`  
**Paired with:** `projects/ADC_Basic/`

#### Features
- **Real-time plotting** (matplotlib)
- Multi-channel visualization
- Statistical analysis
- Signal filtering (moving average, median, low-pass)
- FFT frequency analysis
- Noise measurement and SNR

#### Use Cases
- Sensor data visualization
- Signal processing learning
- ADC calibration
- Noise analysis

#### Demos
1. Simple ADC reading
2. Real-time single-channel plot
3. Multi-channel plot (3 channels)
4. Statistical analysis
5. Data logging

#### Quick Start
```bash
cd python_projects/ADC_Basic
python main.py
# Select Demo 2 for real-time plot
```

---

### 3. Activity_Recognition 🤖 (AI Project)

**Location:** `python_projects/Activity_Recognition/`  
**Paired with:** `projects/I2C_Sensors_Multi/` (MPU6050)

#### Features
- **Machine Learning** with TensorFlow
- Neural network training
- Feature extraction from MPU6050
- Real-time activity classification
- Fall detection with alerts

#### Recognized Activities
- Walking
- Running
- Sitting
- Standing
- Falling (with emergency alert)

#### Workflow
```
1. Collect Training Data
   ↓
2. Train AI Model (TensorFlow)
   ↓
3. Real-time Recognition
   ↓
4. Deploy & Monitor
```

#### Use Cases
- Health monitoring
- Elderly fall detection
- Fitness tracking
- Research projects

#### Quick Start
```bash
cd python_projects/Activity_Recognition
pip install -r requirements.txt
python main.py

# 1. Collect data (Demo 1)
# 2. Train model (Demo 2)
# 3. Test recognition (Demo 3)
```

---

## 🔧 Shared Python Libraries

### `python_libs/atmega128/`

#### `serial_com.py` - ATmega128Serial Class

```python
from atmega128 import ATmega128Serial

# Auto-detect and connect
with ATmega128Serial(baudrate=9600) as mcu:
    mcu.send_command('LED_ON 3')
    response = mcu.wait_for_response()
    print(response)
```

**Features:**
- Auto-detection of COM ports
- Context manager support
- Data buffering
- Callback support
- Logging capabilities

---

#### `protocol.py` - ATmega128Protocol Class

```python
from atmega128 import ATmega128Protocol

protocol = ATmega128Protocol()

# Build commands
cmd = protocol.build_command('READ_ADC', 0)

# Parse responses
response = protocol.parse_response("ADC:512")
# {'status': 'ADC', 'data': '512', 'raw': 'ADC:512'}

# Parse sensor data
data = protocol.parse_sensor_data("AX:100,AY:200,AZ:300")
# {'AX': 100, 'AY': 200, 'AZ': 300}
```

---

### `python_libs/visualization/`

#### `realtime_plot.py` - Real-time Plotting

```python
from visualization import RealtimePlotter

plotter = RealtimePlotter(
    max_points=200,
    title="Sensor Data",
    ylabel="Value",
    y_range=(0, 1023)
)

# Set data source
plotter.set_data_source(lambda: read_sensor())

# Run (opens plot window)
plotter.run()
```

#### `data_logger.py` - Data Logging

```python
from visualization import DataLogger

with DataLogger('sensor_data', format='csv',
               columns=['time', 'value']) as logger:
    logger.log({'time': 0, 'value': 512})
```

**Formats:** CSV, JSON, TXT

---

### `python_libs/analysis/`

#### `signal_processing.py` - Signal Processing

```python
from analysis import SignalProcessor

processor = SignalProcessor()

# Filtering
filtered = processor.moving_average(data, window_size=5)
filtered = processor.median_filter(data, window_size=5)
filtered = processor.low_pass_filter(data, alpha=0.3)

# Analysis
peaks = processor.find_peaks(data, threshold=500)
rms = processor.calculate_rms(data)
freqs, mags = processor.fft_analysis(data, sample_rate=50)
```

#### `statistics.py` - Statistical Analysis

```python
from analysis import StatisticsAnalyzer

analyzer = StatisticsAnalyzer()

# Comprehensive statistics
stats = analyzer.calculate_stats(data)
# Returns: mean, median, std, var, min, max, range, q25, q75, iqr

# Outlier detection
outliers = analyzer.detect_outliers(data, method='iqr')

# Trend analysis
trend = analyzer.calculate_trend(data)
# Returns: slope, direction, total_change
```

---

## 🎯 Project Pairing Matrix

| Python Project | C Project | Hardware | Concepts |
|----------------|-----------|----------|----------|
| **Serial_Communications** | Serial_Communications | UART | Protocols, Debugging |
| **ADC_Basic** | ADC_Basic | ADC channels | Data Viz, Analysis |
| **Activity_Recognition** | I2C_Sensors_Multi | MPU6050 | AI/ML, Edge Computing |

---

## 💡 Real-World Applications

### 1. IoT Sensor Networks
```
ATmega128 (Sensors) → Python (Gateway) → Cloud (Analytics)
```

### 2. Smart Manufacturing
```
ATmega128 (Machine Monitoring) → Python (Analysis) → AI (Predictive Maintenance)
```

### 3. Health Monitoring
```
ATmega128 (Wearable Sensors) → Python (Processing) → AI (Activity Recognition)
```

### 4. Environmental Monitoring
```
ATmega128 (Weather Station) → Python (Dashboard) → Database (Historical Data)
```

---

## 🛠️ Installation & Setup

### Prerequisites

1. **Python 3.8+**
   ```bash
   python --version
   ```

2. **Git** (optional)
   ```bash
   git --version
   ```

3. **ATmega128** with programmed firmware

### Installation Steps

#### Windows

```batch
# 1. Navigate to project
cd python_projects\Serial_Communications

# 2. Run launcher (automatic setup)
run.bat
```

#### Linux/Mac

```bash
# 1. Navigate to project
cd python_projects/Serial_Communications

# 2. Create virtual environment
python3 -m venv venv

# 3. Activate
source venv/bin/activate

# 4. Install requirements
pip install -r requirements.txt

# 5. Run
python main.py
```

---

## 📚 Dependencies

### Core (All Projects)
- `pyserial>=3.5` - Serial communication
- `numpy>=1.24.0` - Numerical computing

### Visualization Projects
- `matplotlib>=3.7.0` - Plotting
- `pandas>=2.0.0` - Data analysis

### AI Projects
- `tensorflow>=2.12.0` - Machine learning
- `scikit-learn>=1.3.0` - Data preprocessing

### Web Projects (Future)
- `flask>=2.3.0` - Web server
- `flask-socketio>=5.3.0` - Real-time updates

---

## 🎓 For Students

### Learning Path

#### Beginner (Semester 1-2)
1. Start with **Serial_Communications**
   - Learn basic communication
   - Understand protocols
   - Debug techniques

2. Move to **ADC_Basic**
   - Real-time visualization
   - Data analysis basics
   - Signal processing intro

#### Intermediate (Semester 3)
3. Explore **Activity_Recognition**
   - Feature engineering
   - ML training pipeline
   - Model evaluation

#### Advanced (Capstone)
4. Create custom projects
   - Combine multiple sensors
   - Implement complex AI
   - Deploy to cloud

### Skills Gained

✅ **Programming**
- Python (beginner to advanced)
- Embedded C (ATmega128)
- Object-oriented design

✅ **Data Science**
- NumPy, Pandas
- Matplotlib visualization
- Statistical analysis

✅ **AI/Machine Learning**
- TensorFlow/Keras
- Feature extraction
- Model training & deployment

✅ **Engineering**
- Serial communication protocols
- Real-time systems
- Sensor integration

✅ **Industry Tools**
- Git version control
- Virtual environments
- Documentation

---

## 🔍 Troubleshooting

### Common Issues

#### 1. "No module named 'atmega128'"

**Solution:**
```bash
# Ensure correct working directory
cd python_projects/ProjectName
python main.py
```

The path `../../python_libs` is relative to project directory.

---

#### 2. "No COM port found"

**Solutions:**
- Check USB cable connection
- Verify ATmega128 is powered
- For SimulIDE: Configure serial port
- Try manual port selection:
  ```python
  mcu = ATmega128Serial(port='COM3', baudrate=9600)
  ```

---

#### 3. "TensorFlow not installed"

**Solution:**
```bash
pip install tensorflow scikit-learn
```

Or use CPU-only version:
```bash
pip install tensorflow-cpu
```

---

#### 4. "Matplotlib backend error"

**Solution (Windows):**
```bash
pip install pyqt5
```

**Solution (Linux):**
```bash
sudo apt-get install python3-tk
```

---

## 📖 Teaching Guide

### For Instructors

#### Week-by-Week Plan

**Weeks 1-4: Foundations**
- Python basics review
- Serial communication concepts
- `Serial_Communications` project
- LAB: Interactive terminal

**Weeks 5-8: Data Acquisition**
- Data visualization with matplotlib
- Signal processing fundamentals
- `ADC_Basic` project
- LAB: Filter comparison

**Weeks 9-12: AI Integration**
- Machine learning introduction
- Feature engineering
- `Activity_Recognition` project
- LAB: Train custom model

**Weeks 13-16: Capstone**
- Student project proposals
- Implementation
- Testing & debugging
- Final presentations

---

### Assessment Ideas

1. **Lab Reports**
   - Complete exercises in `lab.py`
   - Document results
   - Answer analysis questions

2. **Mini Projects**
   - Modify existing projects
   - Add new features
   - Improve accuracy

3. **Final Project**
   - Design complete system
   - ATmega128 + Python + AI
   - Real-world application

---

## 🌟 Future Enhancements

### Planned Projects

1. **LCD_Sensor_Dashboard**
   - Flask web interface
   - WebSocket real-time updates
   - Multi-user support

2. **Motor_Control_GUI**
   - PyQt5 GUI
   - PWM control
   - Position feedback

3. **TinyML_OnDevice**
   - TensorFlow Lite Micro
   - Run AI on ATmega128
   - Extremely low latency

4. **MQTT_IoT_Gateway**
   - MQTT protocol
   - Cloud integration
   - AWS/Azure support

---

## 📞 Support & Resources

### Documentation
- Project README files
- Python docstrings
- C project documentation

### External Resources
- [Python Docs](https://docs.python.org/)
- [NumPy Guide](https://numpy.org/doc/)
- [Matplotlib Tutorials](https://matplotlib.org/stable/tutorials/index.html)
- [TensorFlow Learn](https://www.tensorflow.org/learn)
- [PySerial Docs](https://pyserial.readthedocs.io/)

### Community
- Course forum
- Office hours
- Study groups

---

## 🏆 Success Stories

*Space for student projects and achievements*

---

## 📝 Contributing

Students are encouraged to:
- Improve existing projects
- Add new features
- Fix bugs
- Create new projects
- Write documentation

---

## ⚖️ License

© 2025 Prof. Hong Jeong  
Inha University in Tashkent (IUT)  
SOC 3050 - Embedded Systems and Applications

For educational purposes only.

---

## 🎉 Conclusion

You now have a **complete embedded systems + AI curriculum**!

```
ATmega128 (C) + Python + AI = Modern Embedded Engineering
```

### What Students Learn:
✅ Bare-metal embedded programming  
✅ Host-side application development  
✅ Real-time data visualization  
✅ Machine learning & AI  
✅ IoT system integration  
✅ Industry-standard tools  

### Career Preparation:
🚀 Embedded Systems Engineer  
🚀 IoT Developer  
🚀 AI/ML Engineer  
🚀 Full-Stack Developer  
🚀 Robotics Engineer  

---

**Welcome to modern embedded engineering education! 🎓**

From transistors to neural networks - this is the future of embedded systems.

---

*Last Updated: October 2025*  
*Version: 1.0.0*

