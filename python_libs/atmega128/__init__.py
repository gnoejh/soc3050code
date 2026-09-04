"""
ATmega128 Communication Module
Handles serial communication with ATmega128 microcontroller
"""

from .serial_com import ATmega128Serial
from .protocol import ATmega128Protocol
from .auto_detect import detect_atmega128, scan_ports

__all__ = [
    'ATmega128Serial',
    'ATmega128Protocol',
    'detect_atmega128',
    'scan_ports'
]

