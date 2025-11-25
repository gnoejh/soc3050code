"""
Visualization Module
Real-time plotting and data visualization for ATmega128 projects
"""

from .realtime_plot import RealtimePlotter, MultiChannelPlotter
from .data_logger import DataLogger

__all__ = [
    'RealtimePlotter',
    'MultiChannelPlotter',
    'DataLogger'
]

