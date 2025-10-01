#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Installation and setup script for ATmega128 Educational Framework
"""

import subprocess
import sys
import os

# Hand    pr    print("\n4. [VISUALIZE] Or visualize existing data:")nt("\n2. [PYTHON] Run Python integration:")e console encoding for Windows
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding='utf-8')
        sys.stderr.reconfigure(encoding='utf-8')
    except:
        pass

def install_dependencies():
    """Install required Python packages"""
    requirements = [
        'pyserial>=3.5',
        'matplotlib>=3.5.0',
        'numpy>=1.21.0',
        'pandas>=1.3.0',
        'seaborn>=0.11.0'
    ]
    
    print("Installing Python dependencies...")
    print("=" * 40)
    
    for package in requirements:
        print(f"Installing {package}...")
        try:
            subprocess.check_call([sys.executable, '-m', 'pip', 'install', package])
            print(f"[OK] {package} installed successfully")
        except subprocess.CalledProcessError as e:
            print(f"[ERROR] Failed to install {package}: {e}")
            return False
    
    print("\n[OK] All dependencies installed successfully!")
    return True

def test_imports():
    """Test if all modules can be imported"""
    print("\n[TEST] Testing module imports...")
    print("-" * 25)
    
    modules = {
        'serial': 'PySerial',
        'matplotlib.pyplot': 'Matplotlib',
        'numpy': 'NumPy', 
        'pandas': 'Pandas',
        'seaborn': 'Seaborn'
    }
    
    all_success = True
    
    for module, name in modules.items():
        try:
            __import__(module)
            print(f"[OK] {name} - OK")
        except ImportError as e:
            print(f"[ERROR] {name} - FAILED: {e}")
            all_success = False
    
    return all_success

def check_hardware_connection():
    """Check for available serial ports"""
    print("\n[PORTS] Checking serial port availability...")
    print("-" * 35)
    
    try:
        import serial.tools.list_ports
        
        ports = list(serial.tools.list_ports.comports())
        
        if not ports:
            print("[WARNING] No serial ports detected")
            print("   - Make sure ATmega128 is connected")
            print("   - Check USB drivers are installed")
            return False
        
        print("[PORTS] Available serial ports:")
        for port in ports:
            print(f"   - {port.device}: {port.description}")
        
        # Check if COM3 (default) is available
        com3_available = any(port.device == 'COM3' for port in ports)
        if com3_available:
            print(f"[OK] Default port COM3 is available")
        else:
            print(f"[WARNING] Default port COM3 not found")
            print(f"   Update main.py or use --port argument")
        
        return True
        
    except ImportError:
        print("[ERROR] PySerial not installed - cannot check ports")
        return False

def create_test_data():
    """Create sample test data for demonstration"""
    print("\n[DATA] Creating sample test data...")
    print("-" * 30)
    
    try:
        import csv
        from datetime import datetime, timedelta
        import random
        
        # Create sample CSV data
        filename = "data.csv"
        
        start_time = datetime.now()
        
        with open(filename, 'w', newline='', encoding='utf-8') as csvfile:
            fieldnames = ['timestamp', 'temperature', 'light', 'sound', 'raw']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            
            # Generate 50 sample data points
            for i in range(50):
                timestamp = start_time + timedelta(seconds=i*2)
                temp = 20 + random.gauss(0, 3)  # Temperature around 20°C
                light = random.randint(200, 800)  # Light sensor ADC
                sound = random.randint(100, 900)  # Sound sensor ADC
                
                writer.writerow({
                    'timestamp': timestamp.isoformat(),
                    'temperature': round(temp, 1),
                    'light': light,
                    'sound': sound,
                    'raw': f"Temp: {temp:.1f}°C, Light: {light}, Sound: {sound}"
                })
        
        print(f"[OK] Created sample data: {filename}")
        print(f"   Test with: python main.py visualize {filename}")
        return True
        
    except Exception as e:
        print(f"[ERROR] Failed to create test data: {e}")
        return False

def main():
    """Main setup function"""
    print("ATmega128 Educational Framework Setup")
    print("=" * 45)
    print("Setting up Python integration environment...")
    print()
    
    # Step 1: Install dependencies
    if not install_dependencies():
        print("\n[ERROR] Setup failed at dependency installation")
        return False
    
    # Step 2: Test imports
    if not test_imports():
        print("\n[ERROR] Setup failed at import testing")
        return False
    
    # Step 3: Check hardware
    check_hardware_connection()  # Non-critical
    
    # Step 4: Create test data
    create_test_data()  # Non-critical
    
    print("\n" + "=" * 45)
    print("[OK] SETUP COMPLETE!")
    print("=" * 45)
    print("\n[READY] Ready to use ATmega128 Educational Framework!")
    print("\n[STEPS] Next steps:")
    print("1. [BUILD] Program ATmega128 with EDUCATIONAL_DEMO:")
    print("   cd ..")
    print("   .\\build.ps1")
    print("   .\\program.ps1 COM3")
    print()
    print("2. [PYTHON] Run Python integration:")
    print("   python main.py demo")
    print()
    print("3. [MONITOR] Or monitor sensors:")
    print("   python main.py monitor --duration 60")
    print()
    print("4. [VISUALIZE] Or visualize existing data:")
    print("   python main.py visualize data.csv")
    
    return True

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)