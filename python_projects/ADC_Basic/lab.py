"""
ADC_Basic - Laboratory Exercises
Hands-on exercises for ADC data acquisition and visualization

© 2025 Prof. Hong Jeong, IUT
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'python_libs'))

from atmega128 import ATmega128Serial, ATmega128Protocol
from visualization import RealtimePlotter, DataLogger
from analysis import StatisticsAnalyzer, SignalProcessor
import time
import matplotlib.pyplot as plt
from config import *


def exercise_1_voltage_calibration():
    """
    Exercise 1: ADC Calibration
    
    TASK:
    Measure known voltages and create calibration curve
    """
    print("\n" + "=" * 70)
    print("EXERCISE 1: ADC Calibration")
    print("=" * 70)
    print("Apply known voltages and record ADC readings")
    print()
    
    with ATmega128Serial(baudrate=BAUDRATE) as mcu:
        protocol = ATmega128Protocol()
        
        print("Enter voltage values (0-5V) or 'done' to finish:")
        
        voltages = []
        adc_values = []
        
        while True:
            voltage_input = input("\nApply voltage (V): ").strip()
            
            if voltage_input.lower() == 'done':
                break
            
            try:
                voltage_expected = float(voltage_input)
                
                # Read ADC
                print("Reading ADC...")
                samples = []
                for _ in range(10):
                    mcu.send_command('READ_ADC 0')
                    response = mcu.wait_for_response(timeout=1.0)
                    if response:
                        value = protocol.parse_adc_value(response)
                        if value is not None:
                            samples.append(value)
                    time.sleep(0.1)
                
                if samples:
                    adc_avg = sum(samples) / len(samples)
                    voltages.append(voltage_expected)
                    adc_values.append(adc_avg)
                    print(f"✅ Recorded: {voltage_expected}V -> {adc_avg:.1f} ADC")
            
            except ValueError:
                print("❌ Invalid input")
        
        # Plot calibration curve
        if len(voltages) >= 2:
            plt.figure(figsize=(10, 6))
            plt.scatter(voltages, adc_values, s=100, c='blue', label='Measured')
            
            # Linear fit
            import numpy as np
            coeffs = np.polyfit(voltages, adc_values, 1)
            fit_line = np.poly1d(coeffs)
            v_range = np.linspace(min(voltages), max(voltages), 100)
            plt.plot(v_range, fit_line(v_range), 'r--', label=f'Fit: ADC = {coeffs[0]:.1f}*V + {coeffs[1]:.1f}')
            
            plt.xlabel('Voltage (V)')
            plt.ylabel('ADC Value')
            plt.title('ADC Calibration Curve')
            plt.legend()
            plt.grid(True)
            plt.show()


def exercise_2_noise_measurement():
    """
    Exercise 2: Noise Analysis
    
    TASK:
    Measure ADC noise and calculate SNR
    """
    print("\n" + "=" * 70)
    print("EXERCISE 2: Noise Measurement and SNR")
    print("=" * 70)
    print()
    
    with ATmega128Serial(baudrate=BAUDRATE) as mcu:
        protocol = ATmega128Protocol()
        analyzer = StatisticsAnalyzer()
        processor = SignalProcessor()
        
        # Collect data
        print("Collecting 200 samples...")
        data = []
        for i in range(200):
            mcu.send_command('READ_ADC 0')
            response = mcu.wait_for_response(timeout=1.0)
            if response:
                value = protocol.parse_adc_value(response)
                if value is not None:
                    data.append(value)
            print(f"\rProgress: {len(data)}/200", end='', flush=True)
            time.sleep(0.02)
        
        print("\n")
        
        if data:
            stats = analyzer.calculate_stats(data)
            rms = processor.calculate_rms(data)
            
            # Calculate SNR (Signal-to-Noise Ratio)
            signal_power = stats['mean'] ** 2
            noise_power = stats['var']
            snr = 10 * np.log10(signal_power / noise_power) if noise_power > 0 else float('inf')
            
            print("Noise Analysis Results:")
            print("=" * 40)
            print(f"Mean (Signal):    {stats['mean']:.2f}")
            print(f"Std Dev (Noise):  {stats['std']:.2f}")
            print(f"Peak-to-Peak:     {stats['range']:.2f}")
            print(f"RMS:              {rms:.2f}")
            print(f"SNR:              {snr:.2f} dB")
            
            # Plot histogram
            plt.figure(figsize=(12, 5))
            
            plt.subplot(1, 2, 1)
            plt.plot(data)
            plt.title('ADC Readings Over Time')
            plt.xlabel('Sample')
            plt.ylabel('ADC Value')
            plt.grid(True)
            
            plt.subplot(1, 2, 2)
            plt.hist(data, bins=30, edgecolor='black')
            plt.title('ADC Value Distribution')
            plt.xlabel('ADC Value')
            plt.ylabel('Frequency')
            plt.grid(True, alpha=0.3)
            
            plt.tight_layout()
            plt.show()


def exercise_3_signal_filtering():
    """
    Exercise 3: Signal Filtering Comparison
    
    TASK:
    Compare different filtering techniques
    """
    print("\n" + "=" * 70)
    print("EXERCISE 3: Signal Filtering")
    print("=" * 70)
    print("Comparing filter performance")
    print()
    
    with ATmega128Serial(baudrate=BAUDRATE) as mcu:
        protocol = ATmega128Protocol()
        processor = SignalProcessor()
        
        # Collect noisy data
        print("Collecting data...")
        data_raw = []
        for i in range(150):
            mcu.send_command('READ_ADC 0')
            response = mcu.wait_for_response(timeout=1.0)
            if response:
                value = protocol.parse_adc_value(response)
                if value is not None:
                    data_raw.append(value)
            print(f"\rProgress: {len(data_raw)}/150", end='', flush=True)
            time.sleep(0.02)
        
        print("\n")
        
        if data_raw:
            # Apply different filters
            ma_filter = processor.moving_average(data_raw, window_size=5)
            median_filter = processor.median_filter(data_raw, window_size=5)
            lp_filter = processor.low_pass_filter(data_raw, alpha=0.2)
            
            # Plot comparison
            plt.figure(figsize=(14, 10))
            
            plt.subplot(2, 2, 1)
            plt.plot(data_raw, 'gray', alpha=0.7, label='Raw')
            plt.title('Raw ADC Data')
            plt.xlabel('Sample')
            plt.ylabel('ADC Value')
            plt.legend()
            plt.grid(True)
            
            plt.subplot(2, 2, 2)
            plt.plot(data_raw, 'gray', alpha=0.3, label='Raw')
            plt.plot(ma_filter, 'blue', linewidth=2, label='Moving Average')
            plt.title('Moving Average Filter (window=5)')
            plt.xlabel('Sample')
            plt.ylabel('ADC Value')
            plt.legend()
            plt.grid(True)
            
            plt.subplot(2, 2, 3)
            plt.plot(data_raw, 'gray', alpha=0.3, label='Raw')
            plt.plot(median_filter, 'green', linewidth=2, label='Median')
            plt.title('Median Filter (window=5)')
            plt.xlabel('Sample')
            plt.ylabel('ADC Value')
            plt.legend()
            plt.grid(True)
            
            plt.subplot(2, 2, 4)
            plt.plot(data_raw, 'gray', alpha=0.3, label='Raw')
            plt.plot(lp_filter, 'red', linewidth=2, label='Low-pass')
            plt.title('Low-pass Filter (alpha=0.2)')
            plt.xlabel('Sample')
            plt.ylabel('ADC Value')
            plt.legend()
            plt.grid(True)
            
            plt.tight_layout()
            plt.show()


def exercise_4_frequency_analysis():
    """
    Exercise 4: Frequency Analysis (FFT)
    
    TASK:
    Perform FFT analysis to identify frequency components
    """
    print("\n" + "=" * 70)
    print("EXERCISE 4: Frequency Analysis")
    print("=" * 70)
    print()
    
    with ATmega128Serial(baudrate=BAUDRATE) as mcu:
        protocol = ATmega128Protocol()
        processor = SignalProcessor()
        
        # Collect data at known rate
        print("Collecting data at 50Hz...")
        data = []
        sample_rate = 50  # Hz
        
        for i in range(256):  # Power of 2 for FFT
            mcu.send_command('READ_ADC 0')
            response = mcu.wait_for_response(timeout=1.0)
            if response:
                value = protocol.parse_adc_value(response)
                if value is not None:
                    data.append(value)
            print(f"\rProgress: {len(data)}/256", end='', flush=True)
            time.sleep(1.0 / sample_rate)
        
        print("\n")
        
        if len(data) >= 64:
            # Perform FFT
            freqs, magnitudes = processor.fft_analysis(data, sample_rate)
            
            # Plot results
            plt.figure(figsize=(14, 5))
            
            plt.subplot(1, 2, 1)
            plt.plot(data)
            plt.title('Time Domain Signal')
            plt.xlabel('Sample')
            plt.ylabel('ADC Value')
            plt.grid(True)
            
            plt.subplot(1, 2, 2)
            plt.plot(freqs[:len(freqs)//2], magnitudes[:len(magnitudes)//2])
            plt.title('Frequency Spectrum')
            plt.xlabel('Frequency (Hz)')
            plt.ylabel('Magnitude')
            plt.grid(True)
            
            plt.tight_layout()
            plt.show()
            
            # Find dominant frequency
            if len(magnitudes) > 1:
                dominant_idx = np.argmax(magnitudes[1:len(magnitudes)//2]) + 1
                dominant_freq = freqs[dominant_idx]
                print(f"\nDominant frequency: {dominant_freq:.2f} Hz")


def lab_menu():
    """Laboratory exercises menu"""
    while True:
        print("\n" + "=" * 70)
        print("ADC_BASIC - LABORATORY EXERCISES")
        print("=" * 70)
        print("Select exercise:")
        print()
        print("  1. ADC Calibration")
        print("  2. Noise Measurement and SNR")
        print("  3. Signal Filtering Comparison")
        print("  4. Frequency Analysis (FFT)")
        print("  0. Exit")
        print()
        
        choice = input("Enter choice (0-4): ").strip()
        
        if choice == '1':
            exercise_1_voltage_calibration()
        elif choice == '2':
            exercise_2_noise_measurement()
        elif choice == '3':
            exercise_3_signal_filtering()
        elif choice == '4':
            exercise_4_frequency_analysis()
        elif choice == '0':
            print("\n👋 Lab session ended!")
            break
        else:
            print("❌ Invalid choice")


if __name__ == '__main__':
    import numpy as np  # Import numpy for exercises
    
    print("=" * 70)
    print("SOC 3050 - ADC Basic Laboratory Exercises")
    print("© 2025 Prof. Hong Jeong, IUT")
    print("=" * 70)
    
    try:
        lab_menu()
    except KeyboardInterrupt:
        print("\n\n👋 Lab interrupted")

