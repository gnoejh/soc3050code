"""
ADC_Basic - Real-time ADC Data Visualization
Reads ATmega128 ADC and plots data in real-time

PROJECT: ADC_Basic (Python)
COURSE: SOC 3050 - Embedded Systems and Applications
YEAR: 2025
AUTHOR: Professor Hong Jeong

PURPOSE:
Demonstrates real-time visualization of ATmega128 ADC data using Python.
Shows how to create professional data acquisition and visualization tools.

FEATURES:
- Real-time ADC plotting
- Multi-channel data acquisition
- Statistical analysis
- Data logging and export
- Voltage conversion

PAIRED C PROJECT:
projects/ADC_Basic/Main.c

© 2025 Prof. Hong Jeong, IUT
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'python_libs'))

from atmega128 import ATmega128Serial, ATmega128Protocol
from visualization import RealtimePlotter, MultiChannelPlotter, DataLogger
from analysis import StatisticsAnalyzer, SignalProcessor
import time
from config import *


def demo_1_simple_adc_reading():
    """
    Demo 1: Simple ADC Reading
    Read and display ADC values
    """
    print("\n" + "=" * 70)
    print("DEMO 1: Simple ADC Reading")
    print("=" * 70)
    print()
    
    with ATmega128Serial(baudrate=BAUDRATE) as mcu:
        protocol = ATmega128Protocol()
        
        print(f"{'Sample':<10} {'ADC':<10} {'Voltage (V)':<15} {'Percentage':<15}")
        print("-" * 50)
        
        for i in range(20):
            mcu.send_command('READ_ADC 0')
            response = mcu.wait_for_response(timeout=1.0)
            
            if response:
                adc_value = protocol.parse_adc_value(response)
                if adc_value is not None:
                    voltage = (adc_value / ADC_RESOLUTION) * ADC_REFERENCE_V
                    percent = (adc_value / ADC_RESOLUTION) * 100
                    
                    print(f"{i+1:<10} {adc_value:<10} {voltage:<15.3f} {percent:<15.1f}%")
            
            time.sleep(0.3)


def demo_2_realtime_plot():
    """
    Demo 2: Real-time ADC Plotting
    Visualize ADC data with matplotlib
    """
    print("\n" + "=" * 70)
    print("DEMO 2: Real-time ADC Plotting")
    print("=" * 70)
    print("Close plot window to stop")
    print()
    
    # Create serial connection
    mcu = ATmega128Serial(baudrate=BAUDRATE, auto_connect=True)
    protocol = ATmega128Protocol()
    
    # Create plotter
    plotter = RealtimePlotter(
        max_points=MAX_PLOT_POINTS,
        title="Real-time ADC Data (Channel 0)",
        ylabel="ADC Value (0-1023)",
        xlabel="Sample",
        y_range=None if AUTO_SCALE_Y else (Y_MIN, Y_MAX),
        update_interval=PLOT_UPDATE_INTERVAL
    )
    
    # Define data source
    def get_adc_data():
        """Get next ADC value"""
        try:
            mcu.send_command('READ_ADC 0')
            response = mcu.wait_for_response(timeout=0.5)
            
            if response:
                value = protocol.parse_adc_value(response)
                return value
        except:
            pass
        return None
    
    # Set data source
    plotter.set_data_source(get_adc_data)
    
    # Run plotter (blocking)
    try:
        plotter.run()
    finally:
        mcu.disconnect()


def demo_3_multi_channel_plot():
    """
    Demo 3: Multi-Channel ADC Plotting
    Plot multiple ADC channels simultaneously
    """
    print("\n" + "=" * 70)
    print("DEMO 3: Multi-Channel ADC Plotting")
    print("=" * 70)
    print("Plotting 3 ADC channels simultaneously")
    print("Close plot window to stop")
    print()
    
    # Create serial connection
    mcu = ATmega128Serial(baudrate=BAUDRATE, auto_connect=True)
    protocol = ATmega128Protocol()
    
    # Create multi-channel plotter
    plotter = MultiChannelPlotter(
        num_channels=3,
        channel_names=['ADC Channel 0', 'ADC Channel 1', 'ADC Channel 2'],
        max_points=MAX_PLOT_POINTS,
        title="Multi-Channel ADC Monitoring",
        update_interval=PLOT_UPDATE_INTERVAL
    )
    
    # Define data source
    def get_multi_channel_data():
        """Get data from all channels"""
        try:
            values = []
            for ch in range(3):
                mcu.send_command(f'READ_ADC {ch}')
                response = mcu.wait_for_response(timeout=0.5)
                
                if response:
                    value = protocol.parse_adc_value(response)
                    values.append(value if value is not None else 0)
                else:
                    values.append(0)
            
            return values if len(values) == 3 else None
        except:
            return None
    
    # Set data source
    plotter.set_data_source(get_multi_channel_data)
    
    # Run plotter (blocking)
    try:
        plotter.run()
    finally:
        mcu.disconnect()


def demo_4_data_analysis():
    """
    Demo 4: Statistical Data Analysis
    Collect data and perform analysis
    """
    print("\n" + "=" * 70)
    print("DEMO 4: Statistical Data Analysis")
    print("=" * 70)
    print()
    
    with ATmega128Serial(baudrate=BAUDRATE) as mcu:
        protocol = ATmega128Protocol()
        analyzer = StatisticsAnalyzer()
        processor = SignalProcessor()
        
        # Collect data
        num_samples = 100
        print(f"Collecting {num_samples} samples...")
        
        data_raw = []
        for i in range(num_samples):
            mcu.send_command('READ_ADC 0')
            response = mcu.wait_for_response(timeout=1.0)
            
            if response:
                value = protocol.parse_adc_value(response)
                if value is not None:
                    data_raw.append(value)
                    print(f"\rProgress: {len(data_raw)}/{num_samples}", end='', flush=True)
            
            time.sleep(0.05)
        
        print(f"\n\n✅ Collected {len(data_raw)} samples\n")
        
        # Perform analysis
        if data_raw:
            # Basic statistics
            stats = analyzer.calculate_stats(data_raw)
            
            print("=" * 50)
            print("STATISTICAL ANALYSIS")
            print("=" * 50)
            print(f"Samples:          {stats['count']}")
            print(f"Mean:             {stats['mean']:.2f}")
            print(f"Median:           {stats['median']:.2f}")
            print(f"Std Deviation:    {stats['std']:.2f}")
            print(f"Variance:         {stats['var']:.2f}")
            print(f"Min:              {stats['min']:.2f}")
            print(f"Max:              {stats['max']:.2f}")
            print(f"Range:            {stats['range']:.2f}")
            print(f"Q25 (25%):        {stats['q25']:.2f}")
            print(f"Q75 (75%):        {stats['q75']:.2f}")
            print(f"IQR:              {stats['iqr']:.2f}")
            
            # Outlier detection
            print("\n" + "=" * 50)
            print("OUTLIER DETECTION")
            print("=" * 50)
            outliers = analyzer.detect_outliers(data_raw, method='iqr')
            print(f"Outliers found:   {len(outliers)}")
            if outliers:
                print(f"Outlier indices:  {outliers[:10]}" + ("..." if len(outliers) > 10 else ""))
            
            # Trend analysis
            print("\n" + "=" * 50)
            print("TREND ANALYSIS")
            print("=" * 50)
            trend = analyzer.calculate_trend(data_raw)
            print(f"Trend direction:  {trend['direction']}")
            print(f"Slope:            {trend['slope']:.4f}")
            print(f"Change/sample:    {trend['change_per_sample']:.4f}")
            print(f"Total change:     {trend['total_change']:.2f}")
            
            # Signal processing
            print("\n" + "=" * 50)
            print("SIGNAL PROCESSING")
            print("=" * 50)
            
            # Apply filters
            ma_filtered = processor.moving_average(data_raw, window_size=5)
            lp_filtered = processor.low_pass_filter(data_raw, alpha=0.3)
            
            stats_ma = analyzer.calculate_stats(ma_filtered)
            stats_lp = analyzer.calculate_stats(lp_filtered)
            
            print(f"Raw data noise (std):       {stats['std']:.2f}")
            print(f"After moving average:       {stats_ma['std']:.2f}")
            print(f"After low-pass filter:      {stats_lp['std']:.2f}")
            
            # Data quality
            print("\n" + "=" * 50)
            print("DATA QUALITY ASSESSMENT")
            print("=" * 50)
            quality = analyzer.data_quality_score(data_raw)
            print(f"Quality score:    {quality['score']}/100")
            print(f"Issues:           {', '.join(quality['issues'])}")


def demo_5_data_logging():
    """
    Demo 5: Data Logging to File
    Save ADC data for later analysis
    """
    print("\n" + "=" * 70)
    print("DEMO 5: Data Logging")
    print("=" * 70)
    print()
    
    # Create log directory
    os.makedirs(LOG_DIRECTORY, exist_ok=True)
    
    # Create filename with timestamp
    from datetime import datetime
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = os.path.join(LOG_DIRECTORY, f"adc_data_{timestamp}")
    
    with ATmega128Serial(baudrate=BAUDRATE) as mcu:
        with DataLogger(log_file, format=LOG_FORMAT,
                       columns=['sample', 'adc_value', 'voltage', 'percentage']) as logger:
            
            protocol = ATmega128Protocol()
            num_samples = 50
            
            print(f"Logging {num_samples} samples to {logger.filename}...")
            print()
            
            for i in range(num_samples):
                mcu.send_command('READ_ADC 0')
                response = mcu.wait_for_response(timeout=1.0)
                
                if response:
                    adc_value = protocol.parse_adc_value(response)
                    if adc_value is not None:
                        voltage = (adc_value / ADC_RESOLUTION) * ADC_REFERENCE_V
                        percentage = (adc_value / ADC_RESOLUTION) * 100
                        
                        # Log data
                        logger.log({
                            'sample': i + 1,
                            'adc_value': adc_value,
                            'voltage': voltage,
                            'percentage': percentage
                        })
                        
                        # Display progress
                        print(f"\rSample {i+1:3d}: ADC={adc_value:4d}, V={voltage:.3f}, {percentage:5.1f}%", 
                              end='', flush=True)
                
                time.sleep(0.1)
            
            print(f"\n\n✅ Data saved to {logger.filename}")


def main_menu():
    """Main menu for demo selection"""
    while True:
        print("\n" + "=" * 70)
        print("ADC_BASIC - REAL-TIME DATA VISUALIZATION")
        print("=" * 70)
        print("Select demonstration:")
        print()
        print("  1. Simple ADC Reading")
        print("  2. Real-time ADC Plotting (Single Channel)")
        print("  3. Multi-Channel Plotting (3 Channels)")
        print("  4. Statistical Data Analysis")
        print("  5. Data Logging to File")
        print("  0. Exit")
        print()
        
        choice = input("Enter choice (0-5): ").strip()
        
        if choice == '1':
            demo_1_simple_adc_reading()
        elif choice == '2':
            demo_2_realtime_plot()
        elif choice == '3':
            demo_3_multi_channel_plot()
        elif choice == '4':
            demo_4_data_analysis()
        elif choice == '5':
            demo_5_data_logging()
        elif choice == '0':
            print("\n👋 Goodbye!")
            break
        else:
            print("❌ Invalid choice")


if __name__ == '__main__':
    print("=" * 70)
    print("SOC 3050 - ADC Basic Python Interface")
    print("© 2025 Prof. Hong Jeong, IUT")
    print("=" * 70)
    
    try:
        main_menu()
    except KeyboardInterrupt:
        print("\n\n👋 Program interrupted")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()

