#!/usr/bin/env python3
"""
Quick Start - Project Launcher Dashboard
© 2025 Prof. Hong Jeaong, IUT (Inha University in Tashkent)

This script checks dependencies and starts the dashboard.
"""

import sys
import subprocess
import importlib.util

def check_python_version():
    """Check if Python version is 3.8+"""
    if sys.version_info < (3, 8):
        print("[ERROR] Python 3.8+ required. Current:", sys.version)
        return False
    print(f"[OK] Python {sys.version.split()[0]}")
    return True

def check_package(package_name, import_name=None):
    """Check if a package is installed"""
    if import_name is None:
        import_name = package_name.replace('-', '_')
    
    spec = importlib.util.find_spec(import_name)
    if spec is None:
        return False
    return True

def install_package(package_name):
    """Install a package using pip"""
    print(f"📦 Installing {package_name}...")
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", package_name])
        return True
    except subprocess.CalledProcessError:
        return False

def main():
    print("=" * 70)
    print("🎓 ATmega128 Project Launcher Dashboard")
    print("   Inha University in Tashkent")
    print("=" * 70)
    print()
    
    # Check Python version
    if not check_python_version():
        sys.exit(1)
    
    # Required packages
    packages = {
        'flask': 'flask',
        'flask-socketio': 'flask_socketio',
        'pyserial': 'serial'
    }
    
    print("\n📋 Checking dependencies...")
    
    missing = []
    for package, import_name in packages.items():
        if check_package(package, import_name):
            print(f"[OK] {package}")
        else:
            print(f"[ERROR] {package} - Missing")
            missing.append(package)
    
    # Install missing packages
    if missing:
        print(f"\n[INSTALL] Installing {len(missing)} missing package(s)...")
        for package in missing:
            if install_package(package):
                print(f"[OK] {package} installed")
            else:
                print(f"[ERROR] Failed to install {package}")
                sys.exit(1)
    
    print("\n" + "=" * 70)
    print("[START] Starting Dashboard...")
    print("=" * 70)
    print("🌐 Dashboard URL: http://localhost:5001")
    print("📱 Network access: http://<your-ip>:5001")
    print("=" * 70)
    print("Press Ctrl+C to stop")
    print("=" * 70)
    print()
    
    # Import and run dashboard
    try:
        from project_launcher_dashboard import dashboard
        dashboard.run(host='0.0.0.0', debug=True)
    except KeyboardInterrupt:
        print("\n\n[STOP] Dashboard stopped")
    except Exception as e:
        print(f"\n[ERROR] Error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
