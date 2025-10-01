"""
ATmega128 Educational Framework - Python Integration
Complete learning progression: Assembly → C → Python → IoT
"""

__version__ = "1.0.0"
__author__ = "SOC3050 Educational Framework"

from .educational_interface import ATmega128Educational
from .sensor_monitoring import SensorMonitor
from .data_visualization import DataVisualizer

__all__ = [
    'ATmega128Educational',
    'SensorMonitor', 
    'DataVisualizer'
]