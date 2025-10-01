#!/usr/bin/env python3
"""
ATmega128 Educational Framework - Main Application
Complete Python integration for Assembly → C → Python → IoT learning progression
"""

import sys
import os
import argparse
from datetime import datetime

# Add the src directory to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from atmega128_educational import ATmega128Educational, SensorMonitor, DataVisualizer

def print_banner():
    """Print application banner"""
    print("=" * 70)
    print("🎓 ATmega128 EDUCATIONAL FRAMEWORK - Python Integration")
    print("🔄 Assembly → C → Python → IoT Learning Progression")
    print("=" * 70)
    print(f"⏰ Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print()

def interactive_demo(port='COM3'):
    """Run interactive educational demonstration"""
    print("🎮 INTERACTIVE EDUCATIONAL DEMONSTRATION")
    print("-" * 40)
    print("This demonstrates the complete learning progression:")
    print("1. 🔧 Assembly register manipulation")
    print("2. 🔗 Communication protocols")
    print("3. 📊 Sensor data collection")
    print("4. 🐍 Python integration")
    print()
    
    try:
        atmega = ATmega128Educational(port)
        atmega.run_complete_demonstration()
    except Exception as e:
        print(f"❌ Demo failed: {e}")
        return False
    
    return True

def sensor_monitoring(port='COM3', duration=60):
    """Run sensor monitoring session"""
    print("🔍 SENSOR MONITORING SESSION")
    print("-" * 30)
    print(f"📡 Port: {port}")
    print(f"⏱️  Duration: {duration} seconds")
    print()
    
    try:
        monitor = SensorMonitor(port)
        
        if monitor.start_monitoring(duration):
            monitor.print_summary()
            
            # Offer visualization
            choice = input("\n📈 Visualize collected data? (y/n): ")
            if choice.lower().startswith('y'):
                viz = DataVisualizer(monitor.get_all_data())
                viz.create_dashboard()
                
                # Save dashboard
                save_choice = input("💾 Save dashboard to file? (y/n): ")
                if save_choice.lower().startswith('y'):
                    filename = f"sensor_dashboard_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
                    viz.create_dashboard(filename)
            
            return True
    except Exception as e:
        print(f"❌ Monitoring failed: {e}")
        return False

def visualize_data(csv_file):
    """Visualize data from CSV file"""
    print("📈 DATA VISUALIZATION")
    print("-" * 20)
    print(f"📁 File: {csv_file}")
    print()
    
    try:
        viz = DataVisualizer()
        
        if viz.load_csv_data(csv_file):
            print("Creating comprehensive dashboard...")
            viz.create_dashboard()
            return True
        else:
            print(f"❌ Failed to load data from {csv_file}")
            return False
            
    except Exception as e:
        print(f"❌ Visualization failed: {e}")
        return False

def check_dependencies():
    """Check if all required dependencies are installed"""
    required_modules = [
        'serial',
        'matplotlib', 
        'numpy',
        'pandas',
        'seaborn'
    ]
    
    missing = []
    
    for module in required_modules:
        try:
            __import__(module)
        except ImportError:
            missing.append(module)
    
    if missing:
        print("❌ Missing required dependencies:")
        for module in missing:
            print(f"   - {module}")
        print("\n💡 Install with: pip install pyserial matplotlib numpy pandas seaborn")
        return False
    
    print("✅ All dependencies installed")
    return True

def main():
    """Main application entry point"""
    parser = argparse.ArgumentParser(
        description="ATmega128 Educational Framework - Python Integration",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python main.py demo                                 # Run interactive demo
  python main.py monitor --port COM3 --duration 120  # Monitor sensors for 2 minutes
  python main.py visualize data.csv                  # Visualize CSV data
  python main.py check                               # Check dependencies
        """
    )
    
    subparsers = parser.add_subparsers(dest='command', help='Available commands')
    
    # Demo command
    demo_parser = subparsers.add_parser('demo', help='Run interactive educational demonstration')
    demo_parser.add_argument('--port', default='COM3', help='Serial port (default: COM3)')
    
    # Monitor command
    monitor_parser = subparsers.add_parser('monitor', help='Run sensor monitoring session')
    monitor_parser.add_argument('--port', default='COM3', help='Serial port (default: COM3)')
    monitor_parser.add_argument('--duration', type=int, default=60, help='Monitoring duration in seconds (default: 60)')
    
    # Visualize command
    viz_parser = subparsers.add_parser('visualize', help='Visualize data from CSV file')
    viz_parser.add_argument('csv_file', help='Path to CSV file containing sensor data')
    
    # Check command
    subparsers.add_parser('check', help='Check dependencies and system status')
    
    args = parser.parse_args()
    
    print_banner()
    
    # Check dependencies first
    if not check_dependencies():
        return 1
    
    if args.command == 'demo':
        print("⚠️  PREREQUISITES:")
        print("   1. ATmega128 connected to", args.port)
        print("   2. EDUCATIONAL_DEMO compiled and programmed")
        print("   3. Serial terminal closed")
        print()
        
        input("Press Enter when ready...")
        success = interactive_demo(args.port)
        
    elif args.command == 'monitor':
        print("⚠️  PREREQUISITES:")
        print("   1. ATmega128 connected to", args.port)
        print("   2. EDUCATIONAL_DEMO compiled and programmed")
        print("   3. Serial terminal closed")
        print()
        
        input("Press Enter to start monitoring...")
        success = sensor_monitoring(args.port, args.duration)
        
    elif args.command == 'visualize':
        if not os.path.exists(args.csv_file):
            print(f"❌ File not found: {args.csv_file}")
            return 1
        
        success = visualize_data(args.csv_file)
        
    elif args.command == 'check':
        print("🔍 System Check Complete")
        success = True
        
    else:
        parser.print_help()
        print("\n💡 Start with: python main.py demo")
        return 0
    
    if success:
        print("\n✅ Operation completed successfully!")
        return 0
    else:
        print("\n❌ Operation failed!")
        return 1

if __name__ == "__main__":
    sys.exit(main())