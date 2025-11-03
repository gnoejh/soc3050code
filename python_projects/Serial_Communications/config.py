"""
Configuration for Serial_Communications Python Project
Pairs with ATmega128 Serial_Communications C project

© 2025 Prof. Hong Jeong, IUT
"""

# Serial Communication Settings
BAUDRATE = 9600
TIMEOUT = 1.0
AUTO_DETECT = True
DEFAULT_PORT = None  # Auto-detect if None

# Data Logging
LOG_TO_FILE = True
LOG_DIRECTORY = "data_logs"
LOG_FORMAT = "csv"  # csv, json, txt

# Display Settings
ENABLE_COLORS = True
SHOW_TIMESTAMPS = True

