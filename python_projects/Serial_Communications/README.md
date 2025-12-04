# Serial Communications - Python Interface

**Python companion project for ATmega128 Serial Communications**

© 2025 Prof. Hong Jeong, IUT (Inha University in Tashkent)  
SOC 3050 - Embedded Systems and Applications

---

## Overview

This Python project provides a host-side interface for the ATmega128 Serial Communications C project. It demonstrates how to create PC applications that communicate with embedded systems.

## Paired C Project

**Location:** `projects/Serial_Communications/`

This Python project is designed to work with the ATmega128 serial communication firmware. Make sure the C project is programmed to your ATmega128 before running these Python scripts.

---

## Features

### Main Program (`main.py`)

1. **Simple Communication** - Test basic command-response patterns
2. **LED Control** - Control ATmega128 LEDs from Python
3. **Sensor Reading** - Read and display sensor values
4. **Continuous Monitoring** - Real-time sensor data display
5. **Data Logging** - Save sensor data to files
6. **Interactive Terminal** - Send custom commands

### Laboratory Exercises (`lab.py`)

1. **Echo Server Test** - Verify communication reliability
2. **LED Pattern Controller** - Create custom LED patterns
3. **Multi-Sensor Dashboard** - Real-time multi-channel monitoring
4. **Command-Line Interface** - Build a CLI application
5. **Data Analysis Tool** - Statistical analysis of sensor data

---

## Requirements

- Python 3.8 or later
- pyserial library
- ATmega128 with Serial_Communications firmware
- USB-Serial connection or SimulIDE

---

## Quick Start

### Windows

```batch
run.bat
```

### Linux/Mac

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python main.py
```

---

## Manual Installation

```bash
# Create virtual environment (recommended)
python -m venv venv

# Activate virtual environment
# Windows:
venv\Scripts\activate
# Linux/Mac:
source venv/bin/activate

# Install requirements
pip install -r requirements.txt

# Run main program
python main.py

# Or run lab exercises
python lab.py
```

---

## Configuration

Edit `config.py` to customize:

```python
BAUDRATE = 9600          # Serial baud rate
AUTO_DETECT = True       # Auto-detect COM port
LOG_TO_FILE = True       # Enable data logging
LOG_FORMAT = "csv"       # Log file format (csv/json/txt)
```

---

## Usage Examples

### Basic Communication

```python
from atmega128 import ATmega128Serial

# Connect to ATmega128
with ATmega128Serial(baudrate=9600) as mcu:
    # Send command
    mcu.send_command('PING')
    
    # Wait for response
    response = mcu.wait_for_response(timeout=2.0)
    print(response)
```

### LED Control

```python
# Turn LED 3 on
mcu.send_command('LED_ON 3')

# Turn LED 3 off
mcu.send_command('LED_OFF 3')
```

### Sensor Reading

```python
from atmega128 import ATmega128Protocol

protocol = ATmega128Protocol()

# Read ADC channel 0
mcu.send_command('READ_ADC 0')
response = mcu.wait_for_response()

# Parse value
adc_value = protocol.parse_adc_value(response)
print(f"ADC Value: {adc_value}")
```

### Data Logging

```python
from visualization import DataLogger

with DataLogger('sensor_data', format='csv', 
               columns=['time', 'value']) as logger:
    logger.log({'time': 0, 'value': 512})
```

---

## Communication Protocol

### Standard Commands

| Command | Description | Example |
|---------|-------------|---------|
| `PING` | Test connection | `PING` |
| `ECHO <text>` | Echo back text | `ECHO Hello` |
| `LED_ON <n>` | Turn LED on | `LED_ON 3` |
| `LED_OFF <n>` | Turn LED off | `LED_OFF 3` |
| `READ_ADC <ch>` | Read ADC channel | `READ_ADC 0` |
| `STATUS` | Get system status | `STATUS` |

### Response Format

```
STATUS:DATA
```

Examples:
- `OK:Connected`
- `ADC:512`
- `ERROR:Invalid command`

---

## Troubleshooting

### No COM Port Found

- Check USB cable connection
- Verify ATmega128 is powered
- For SimulIDE: Ensure serial port is configured in SimulIDE

### No Response from ATmega128

- Verify C firmware is programmed correctly
- Check baud rate matches (default: 9600)
- Ensure ATmega128 serial program is running

### Import Errors

```bash
# Make sure you're in the project directory and python_libs is accessible
cd python_projects/Serial_Communications
python main.py
```

---

## Learning Objectives

By completing this project, students will learn:

1. ✅ Python serial communication (`pyserial`)
2. ✅ Request-response protocols
3. ✅ Data parsing and validation
4. ✅ Real-time data visualization
5. ✅ File I/O and data logging
6. ✅ Command-line interface design
7. ✅ Error handling and debugging

---

## Related Projects

- **C Project:** `projects/Serial_Communications/`
- **Python Libraries:** `python_libs/atmega128/`
- **Next Project:** `python_projects/ADC_Basic/` (with real-time plotting)

---

## Support

For questions or issues:
- Check project documentation
- Review C project code
- Consult course materials

---

**Happy Coding! 🚀**

