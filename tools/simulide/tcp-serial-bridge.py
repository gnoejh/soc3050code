#!/usr/bin/env python3
"""
TCP to Serial Bridge for SimulIDE
Connects VS Code Serial Monitor to SimulIDE via TCP

Usage: python tcp-serial-bridge.py [--port COM4] [--baud 9600] [--tcp 1234]
"""

import socket
import serial
import threading
import sys
import argparse
import time

def tcp_to_serial(tcp_conn, ser):
    """Forward data from TCP to Serial"""
    try:
        while True:
            data = tcp_conn.recv(1024)
            if not data:
                break
            ser.write(data)
            print(f"TCP→Serial: {data.decode('utf-8', errors='replace')}", end='', flush=True)
    except Exception as e:
        print(f"\nTCP→Serial error: {e}")

def serial_to_tcp(tcp_conn, ser):
    """Forward data from Serial to TCP"""
    try:
        while True:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                tcp_conn.sendall(data)
                print(f"Serial→TCP: {data.decode('utf-8', errors='replace')}", end='', flush=True)
            time.sleep(0.01)
    except Exception as e:
        print(f"\nSerial→TCP error: {e}")

def main():
    parser = argparse.ArgumentParser(description='TCP to Serial Bridge')
    parser.add_argument('--port', default='COM4', help='Serial port (default: COM4)')
    parser.add_argument('--baud', type=int, default=9600, help='Baud rate (default: 9600)')
    parser.add_argument('--tcp', type=int, default=1234, help='TCP port (default: 1234)')
    args = parser.parse_args()

    # Open serial port
    print(f"Opening serial port {args.port} at {args.baud} baud...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
        print(f"✓ Serial port opened: {args.port}")
    except Exception as e:
        print(f"✗ Failed to open serial port: {e}")
        return

    # Connect to SimulIDE TCP server
    print(f"Connecting to SimulIDE at localhost:{args.tcp}...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(('localhost', args.tcp))
        print(f"✓ Connected to TCP port {args.tcp}")
    except Exception as e:
        print(f"✗ Failed to connect to TCP: {e}")
        ser.close()
        return

    print("\n=== Bridge Running ===")
    print("Press Ctrl+C to stop\n")

    # Start forwarding threads
    t1 = threading.Thread(target=tcp_to_serial, args=(sock, ser), daemon=True)
    t2 = threading.Thread(target=serial_to_tcp, args=(sock, ser), daemon=True)
    
    t1.start()
    t2.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        sock.close()
        ser.close()
        print("Bridge stopped.")

if __name__ == '__main__':
    main()
