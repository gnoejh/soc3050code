#!/usr/bin/env python3
"""
Virtual Serial Port Bridge - Replaces com0com
Creates a clean bridge between two COM ports without data corruption
"""

import serial
import threading
import time
import sys

def forward_data(src_port, dst_port, direction):
    """Forward data from source to destination port"""
    try:
        while True:
            if src_port.in_waiting > 0:
                data = src_port.read(src_port.in_waiting)
                dst_port.write(data)
                print(f"{direction}: {data.decode('ascii', errors='replace')}", end='', flush=True)
            time.sleep(0.001)  # Small delay to prevent CPU hogging
    except Exception as e:
        print(f"\nError in {direction}: {e}")

def main():
    if len(sys.argv) != 3:
        print("Usage: python virtual-serial-port.py COM_PORT1 COM_PORT2")
        print("Example: python virtual-serial-port.py COM4 COM5")
        sys.exit(1)
    
    port1_name = sys.argv[1]
    port2_name = sys.argv[2]
    
    print(f"=== Virtual Serial Port Bridge ===")
    print(f"Bridging {port1_name} <-> {port2_name}")
    print(f"Press Ctrl+C to stop\n")
    
    try:
        # Open both ports
        port1 = serial.Serial(
            port=port1_name,
            baudrate=19200,  # Will be overridden by actual communication
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )
        
        port2 = serial.Serial(
            port=port2_name,
            baudrate=19200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )
        
        print(f"✓ {port1_name} opened")
        print(f"✓ {port2_name} opened")
        print(f"Bridge active!\n")
        
        # Create bidirectional forwarding threads
        thread1 = threading.Thread(
            target=forward_data,
            args=(port1, port2, f"{port1_name}→{port2_name}"),
            daemon=True
        )
        thread2 = threading.Thread(
            target=forward_data,
            args=(port2, port1, f"{port2_name}→{port1_name}"),
            daemon=True
        )
        
        thread1.start()
        thread2.start()
        
        # Keep running until Ctrl+C
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n\nBridge stopped by user")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        try:
            port1.close()
            port2.close()
            print("Ports closed")
        except:
            pass

if __name__ == "__main__":
    main()
