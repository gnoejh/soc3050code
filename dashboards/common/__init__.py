"""
Common shared modules for all dashboards

© 2025 Prof. Hong Jeaong, IUT (Inha University in Tashkent)
All rights reserved for educational purposes.
Contact: linkedin.com/in/gnoejh53
"""

from .base_dashboard import BaseDashboard
from .serial_handler import SerialHandler
from .data_parser import DataParser

__all__ = ['BaseDashboard', 'SerialHandler', 'DataParser']
