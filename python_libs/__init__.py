"""
ATmega128 Python Interface Libraries
Educational Python modules for ATmega128 microcontroller projects

© 2025 Prof. Hong Jeong, IUT (Inha University in Tashkent)
SOC 3050 - Embedded Systems and Applications

Modules:
- atmega128: Serial communication and device interface
- visualization: Real-time plotting and data visualization
- analysis: Signal processing and data analysis
- web: Web dashboard and IoT interfaces
"""

__version__ = "1.0.0"
__author__ = "Prof. Hong Jeong"
__course__ = "SOC 3050 - Embedded Systems and Applications"

# Package information
PACKAGE_INFO = {
    'name': 'atmega128-python-libs',
    'version': __version__,
    'author': __author__,
    'course': __course__,
    'year': 2025,
    'institution': 'Inha University in Tashkent'
}

def get_info():
    """Print package information"""
    print("=" * 70)
    print(f"ATmega128 Python Libraries v{__version__}")
    print(f"Course: {__course__}")
    print(f"Author: {__author__}")
    print("=" * 70)
    return PACKAGE_INFO

