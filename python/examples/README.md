# 🚀 ATmega128 Python Examples

This directory contains progressive examples to help you learn ATmega128 programming with Python. Start with `quick_start.py` and work your way through the numbered examples.

## 📚 Learning Progression

### 🏃‍♂️ **Start Here: `quick_start.py`**
Your first step! This script will:
- Auto-detect your ATmega128
- Test basic communication
- Blink the LED
- Read a sensor
- Guide you to the next steps

**Run it:** `python quick_start.py`

---

### 📖 **Example 1: `example_01_basic.py`**
**Learn:** Basic communication and LED control

**Concepts:**
- Connecting to ATmega128
- Sending commands
- Controlling LEDs
- Reading responses
- Proper connection management

**Skills gained:**
- ✅ Serial communication basics
- ✅ Command sending
- ✅ Error handling

---

### 📖 **Example 2: `example_02_sensors.py`**
**Learn:** Reading sensor data

**Concepts:**
- Temperature sensor reading
- Multi-sensor data collection
- Data logging
- Context managers (automatic cleanup)

**Skills gained:**
- ✅ Sensor data acquisition
- ✅ Data storage (JSON)
- ✅ Automatic logging
- ✅ Clean coding practices

---

### 📖 **Example 3: `example_03_realtime.py`**
**Learn:** Real-time monitoring with graphs

**Concepts:**
- Live data plotting
- Background data collection
- Real-time visualization
- Threading for concurrent operations

**Skills gained:**
- ✅ Real-time data display
- ✅ Graph creation
- ✅ Concurrent programming
- ✅ CSV data logging

**Requirements:** `matplotlib` for plotting

---

### 📖 **Example 4: `example_04_iot.py`**
**Learn:** IoT web dashboard

**Concepts:**
- Web interface creation
- REST API design
- Real-time web updates
- Remote device control
- Modern web technologies

**Skills gained:**
- ✅ Web development
- ✅ API creation
- ✅ Real-time web apps
- ✅ Remote control
- ✅ Professional dashboards

**Features:**
- Live sensor graphs
- LED control buttons
- Real-time updates
- Mobile-friendly interface

**Access:** Open browser to `http://localhost:5000`

---

### 📖 **Example 5: `example_05_analysis.py`**
**Learn:** Data analysis and machine learning

**Concepts:**
- Data collection strategies
- Statistical analysis
- Trend detection
- Simple prediction modeling
- Data visualization

**Skills gained:**
- ✅ Data science techniques
- ✅ Statistical analysis
- ✅ Machine learning basics
- ✅ Predictive modeling
- ✅ Advanced plotting

**Output:**
- Statistical reports
- Correlation analysis
- Trend predictions
- Professional plots

---

## 🛠️ Setup Instructions

### 1. Install Dependencies
```bash
# Using uv (recommended)
uv sync

# Or using pip
pip install -r requirements.txt
```

### 2. Find Your Port
Run this to see available ports:
```python
from atmega128 import list_ports
list_ports()
```

### 3. Update Port in Examples
Change `"COM3"` to your actual port in each example file.

### 4. Run Examples
```bash
python examples/quick_start.py
python examples/example_01_basic.py
# ... and so on
```

## 🎯 Learning Path

**Beginner Path:**
1. `quick_start.py` → Get familiar with the setup
2. `example_01_basic.py` → Learn basic communication
3. `example_02_sensors.py` → Understand sensor reading

**Intermediate Path:**
4. `example_03_realtime.py` → Add real-time visualization
5. `example_04_iot.py` → Build web interfaces

**Advanced Path:**
6. `example_05_analysis.py` → Apply data science techniques
7. Create your own projects!

## 🚨 Troubleshooting

### ❌ "Could not connect"
- Check USB cable connection
- Verify correct COM port
- Close other serial programs
- Check device drivers

### ❌ "Port not found"
- Run `list_ports()` to see available ports
- Update port name in example code
- Try different USB ports

### ❌ "No data received"
- Check ATmega128 code configuration
- Verify baudrate settings (default: 9600)
- Ensure ATmega128 is sending data

### ❌ "Module not found"
- Install dependencies: `uv sync` or `pip install -r requirements.txt`
- Activate virtual environment if using one

## 🎓 Project Ideas

After completing the examples, try these projects:

### 🌱 **Beginner Projects**
- **Smart Thermometer:** Log temperature and send alerts
- **Light Controller:** Automatic lights based on ambient light
- **Simple Alarm:** Motion detection with buzzer

### 🚀 **Intermediate Projects**
- **Weather Station:** Multi-sensor data with web dashboard
- **Home Automation:** Control multiple devices via web interface
- **Data Logger:** Long-term environmental monitoring

### 🔬 **Advanced Projects**
- **IoT Greenhouse:** Automated plant care system
- **Machine Learning Predictor:** Predict environmental changes
- **Multi-Device Network:** Connect multiple ATmega128 units

## 📖 Additional Resources

- **ATmega128 Datasheet:** Technical specifications
- **Python Serial Documentation:** Advanced communication techniques
- **Flask Documentation:** Web development
- **Matplotlib Guide:** Data visualization techniques

## 🤝 Need Help?

- Check the main README.md for detailed setup
- Review comments in each example file
- Start with simpler examples if you're stuck
- Practice with `quick_start.py` first

**Happy coding! 🎉**