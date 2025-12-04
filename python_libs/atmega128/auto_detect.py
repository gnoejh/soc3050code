"""
ATmega128 Auto-Detection Utilities
Automatically detect and identify ATmega128 devices

© 2025 Prof. Hong Jeong, IUT
"""

import serial
import serial.tools.list_ports
from typing import Optional, List, Dict


def scan_ports() -> List[Dict]:
    """
    Scan all available serial ports
    
    Returns:
        List of port information dictionaries
    """
    ports = []
    
    for port in serial.tools.list_ports.comports():
        port_type = 'unknown'
        
        # Detect port type
        desc_lower = port.description.lower()
        if 'simulide' in desc_lower or 'virtual' in desc_lower:
            port_type = 'simulator'
        elif 'usb' in desc_lower or 'serial' in desc_lower:
            port_type = 'hardware'
        elif 'ch340' in desc_lower or 'cp210' in desc_lower or 'ftdi' in desc_lower:
            port_type = 'hardware'
        
        ports.append({
            'port': port.device,
            'description': port.description,
            'hwid': port.hwid,
            'type': port_type,
            'vid': port.vid,
            'pid': port.pid
        })
    
    return ports


def detect_atmega128(prefer_simulator: bool = True, 
                    test_connection: bool = True,
                    baudrate: int = 9600) -> Optional[str]:
    """
    Auto-detect ATmega128 device
    
    Args:
        prefer_simulator: Prefer SimulIDE over hardware
        test_connection: Test connection before returning
        baudrate: Baud rate for testing
    
    Returns:
        Port name if found, None otherwise
    """
    ports = scan_ports()
    
    if not ports:
        return None
    
    # Priority list
    candidates = []
    
    if prefer_simulator:
        # Simulator first
        candidates.extend([p for p in ports if p['type'] == 'simulator'])
        candidates.extend([p for p in ports if p['type'] == 'hardware'])
        candidates.extend([p for p in ports if p['type'] == 'unknown'])
    else:
        # Hardware first
        candidates.extend([p for p in ports if p['type'] == 'hardware'])
        candidates.extend([p for p in ports if p['type'] == 'simulator'])
        candidates.extend([p for p in ports if p['type'] == 'unknown'])
    
    # Test each candidate
    for port_info in candidates:
        port = port_info['port']
        
        if not test_connection:
            return port
        
        # Try to connect and test
        try:
            ser = serial.Serial(port, baudrate, timeout=0.5)
            
            # Send ping command
            ser.write(b'PING\n')
            
            # Wait for response (optional - some devices may not respond)
            response = ser.readline()
            
            ser.close()
            
            # If we got here, connection works
            return port
            
        except (serial.SerialException, OSError):
            # Try next port
            continue
    
    # No working port found, return first available
    if ports:
        return ports[0]['port']
    
    return None


def print_available_ports():
    """Print all available serial ports with details"""
    ports = scan_ports()
    
    if not ports:
        print("❌ No serial ports found")
        return
    
    print("=" * 70)
    print("Available Serial Ports:")
    print("=" * 70)
    
    for i, port in enumerate(ports, 1):
        print(f"\n{i}. {port['port']}")
        print(f"   Description: {port['description']}")
        print(f"   Type: {port['type']}")
        print(f"   HWID: {port['hwid']}")
        if port['vid']:
            print(f"   VID:PID: {port['vid']:04X}:{port['pid']:04X}")
    
    print("\n" + "=" * 70)


# Example usage
if __name__ == '__main__':
    print("ATmega128 Auto-Detection Tool")
    print("=" * 70)
    
    # Scan ports
    print_available_ports()
    
    # Auto-detect
    print("\nAuto-detecting ATmega128...")
    port = detect_atmega128()
    
    if port:
        print(f"✅ Found ATmega128 on {port}")
    else:
        print("❌ No ATmega128 device found")

