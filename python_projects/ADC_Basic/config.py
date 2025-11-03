"""
Configuration for ADC_Basic Python Project
Real-time ADC data visualization

© 2025 Prof. Hong Jeong, IUT
"""

# Serial Communication Settings
BAUDRATE = 9600
TIMEOUT = 1.0

# ADC Settings
ADC_RESOLUTION = 1023  # 10-bit ADC
ADC_REFERENCE_V = 5.0  # 5V reference
NUM_ADC_CHANNELS = 8   # ATmega128 has 8 ADC channels

# Plotting Settings
PLOT_UPDATE_INTERVAL = 50  # milliseconds
MAX_PLOT_POINTS = 200     # Number of points to display
AUTO_SCALE_Y = False      # Auto-scale Y axis
Y_MIN = 0                 # Fixed Y min
Y_MAX = 1023              # Fixed Y max

# Data Logging
LOG_TO_FILE = True
LOG_FORMAT = "csv"        # csv, json
LOG_DIRECTORY = "adc_data"

