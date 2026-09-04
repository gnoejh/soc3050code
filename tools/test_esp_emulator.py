#!/usr/bin/env python3
"""
ESP-01 AT Command Emulator
Simulates ESP-01 responses for testing ATmega128 code without real hardware
"""

import serial
import time
import sys
import argparse

def main():
    parser = argparse.ArgumentParser(description='ESP-01 AT Command Emulator')
    parser.add_argument('--port', default='COM10', help='Serial port (default: COM10)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--verbose', action='store_true', help='Show all traffic')
    args = parser.parse_args()
    
    print("=" * 60)
    print("  ESP-01 AT Command Emulator")
    print("=" * 60)
    print(f"Port: {args.port} @ {args.baud} baud")
    print("Waiting for commands from ATmega128...")
    print("Press Ctrl+C to exit")
    print("=" * 60)
    print()
    
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
        print(f"✓ Connected to {args.port}\n")
    except Exception as e:
        print(f"✗ ERROR: Cannot open {args.port}")
        print(f"  {e}")
        print("\nTroubleshooting:")
        print("  1. Check SimulIDE is running")
        print("  2. Verify SerialPort component port setting")
        print("  3. Try different COM port (--port COM11)")
        print("  4. Install pyserial: pip install pyserial")
        sys.exit(1)
    
    command_count = 0
    
    while True:
        try:
            line = ser.readline()
            if not line:
                continue
            
            text = line.decode('utf-8', errors='ignore').strip()
            if not text:
                continue
            
            command_count += 1
            timestamp = time.strftime("%H:%M:%S")
            
            print(f"[{timestamp}] RX #{command_count}: {text}")
            
            # Simulate ESP-01 responses
            if text == 'AT':
                response = b'OK\r\n'
                ser.write(response)
                print(f"            TX: OK")
                
            elif text.startswith('AT+GMR'):
                responses = [
                    b'AT version:1.7.4.0(May 11 2020 19:13:04)\r\n',
                    b'SDK version:3.0.4(9532cba)\r\n',
                    b'compile time:May 27 2020 10:12:17\r\n',
                    b'Bin version(Wroom 02):1.7.4\r\n',
                    b'OK\r\n'
                ]
                for resp in responses:
                    ser.write(resp)
                    if args.verbose:
                        print(f"            TX: {resp.decode().strip()}")
                if not args.verbose:
                    print(f"            TX: [Version info] OK")
                
            elif text.startswith('AT+CWMODE'):
                if '?' in text:
                    # Query mode
                    ser.write(b'+CWMODE:1\r\n')
                    ser.write(b'OK\r\n')
                    print(f"            TX: +CWMODE:1 OK")
                else:
                    # Set mode
                    ser.write(b'OK\r\n')
                    print(f"            TX: OK")
                    
            elif text.startswith('AT+CWJAP'):
                # Simulate WiFi connection with realistic delay
                print(f"            TX: [Connecting to WiFi...]")
                time.sleep(1.0)
                ser.write(b'WIFI CONNECTED\r\n')
                time.sleep(0.5)
                ser.write(b'WIFI GOT IP\r\n')
                ser.write(b'\r\n')
                ser.write(b'OK\r\n')
                print(f"            TX: WIFI CONNECTED, WIFI GOT IP, OK")
                
            elif text.startswith('AT+CWQAP'):
                ser.write(b'OK\r\n')
                ser.write(b'WIFI DISCONNECT\r\n')
                print(f"            TX: OK, WIFI DISCONNECT")
                
            elif text.startswith('AT+CIFSR'):
                responses = [
                    b'+CIFSR:STAIP,"192.168.1.100"\r\n',
                    b'+CIFSR:STAMAC,"18:fe:34:a1:23:45"\r\n',
                    b'OK\r\n'
                ]
                for resp in responses:
                    ser.write(resp)
                print(f"            TX: [IP info] OK")
                
            elif text.startswith('AT+CWLAP'):
                # List access points
                print(f"            TX: [Scanning APs...]")
                time.sleep(0.5)
                ser.write(b'+CWLAP:(3,"MyWiFi",-45,"aa:bb:cc:dd:ee:ff",1)\r\n')
                ser.write(b'+CWLAP:(4,"Guest_Net",-62,"11:22:33:44:55:66",6)\r\n')
                ser.write(b'OK\r\n')
                print(f"            TX: [2 APs found] OK")
                
            elif text.startswith('AT+CIPSTART'):
                # TCP connection
                print(f"            TX: [Opening TCP connection...]")
                time.sleep(0.3)
                ser.write(b'CONNECT\r\n')
                ser.write(b'\r\n')
                ser.write(b'OK\r\n')
                print(f"            TX: CONNECT, OK")
                
            elif text.startswith('AT+CIPSEND'):
                # Ready to send data
                ser.write(b'OK\r\n')
                ser.write(b'\r\n>')  # Prompt for data
                print(f"            TX: OK, > [Ready for data]")
                
            elif text.startswith('AT+CIPCLOSE'):
                ser.write(b'CLOSED\r\n')
                ser.write(b'OK\r\n')
                print(f"            TX: CLOSED, OK")
                
            elif text.startswith('AT+RST'):
                ser.write(b'OK\r\n')
                print(f"            TX: OK [Resetting...]")
                time.sleep(1.5)
                ser.write(b'\r\nready\r\n')
                print(f"            TX: ready")
                
            elif text.startswith('AT+CIPSTATUS'):
                ser.write(b'STATUS:2\r\n')
                ser.write(b'OK\r\n')
                print(f"            TX: STATUS:2 (Got IP), OK")
                
            elif text.startswith('AT+CIPMUX'):
                # Multiple connections setting
                ser.write(b'OK\r\n')
                print(f"            TX: OK")
                
            else:
                # Unknown command
                ser.write(b'ERROR\r\n')
                print(f"            TX: ERROR [Unknown: {text}]")
                
        except KeyboardInterrupt:
            print("\n\n✓ Emulator stopped")
            print(f"Total commands processed: {command_count}")
            break
        except Exception as e:
            print(f"\n✗ ERROR: {e}")
            break
    
    ser.close()
    print("Port closed. Goodbye!")

if __name__ == '__main__':
    main()
