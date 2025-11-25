"""
Signal Processing for ATmega128 Sensor Data
Filtering, smoothing, and feature extraction

© 2025 Prof. Hong Jeong, IUT
"""

import numpy as np
from typing import List, Optional
from collections import deque


class SignalProcessor:
    """
    Signal processing utilities for sensor data
    
    Features:
    - Moving average filter
    - Median filter
    - Low-pass filter
    - High-pass filter
    - Peak detection
    - FFT analysis
    """
    
    @staticmethod
    def moving_average(data: List[float], window_size: int) -> List[float]:
        """
        Apply moving average filter
        
        Args:
            data: Input data
            window_size: Window size for averaging
        
        Returns:
            Filtered data
        """
        if len(data) < window_size:
            return data
        
        result = []
        for i in range(len(data)):
            if i < window_size - 1:
                # Not enough data yet, use available data
                window = data[:i+1]
            else:
                window = data[i-window_size+1:i+1]
            result.append(sum(window) / len(window))
        
        return result
    
    @staticmethod
    def median_filter(data: List[float], window_size: int) -> List[float]:
        """
        Apply median filter (good for removing spikes)
        
        Args:
            data: Input data
            window_size: Window size
        
        Returns:
            Filtered data
        """
        if len(data) < window_size:
            return data
        
        result = []
        for i in range(len(data)):
            if i < window_size // 2:
                window = data[:i + window_size // 2 + 1]
            elif i >= len(data) - window_size // 2:
                window = data[i - window_size // 2:]
            else:
                window = data[i - window_size // 2:i + window_size // 2 + 1]
            result.append(sorted(window)[len(window) // 2])
        
        return result
    
    @staticmethod
    def low_pass_filter(data: List[float], alpha: float = 0.3) -> List[float]:
        """
        Simple exponential low-pass filter
        
        Args:
            data: Input data
            alpha: Smoothing factor (0-1, lower = more smoothing)
        
        Returns:
            Filtered data
        """
        if not data:
            return []
        
        result = [data[0]]
        for i in range(1, len(data)):
            filtered = alpha * data[i] + (1 - alpha) * result[-1]
            result.append(filtered)
        
        return result
    
    @staticmethod
    def find_peaks(data: List[float], threshold: Optional[float] = None,
                   min_distance: int = 1) -> List[int]:
        """
        Find peaks in signal
        
        Args:
            data: Input data
            threshold: Minimum peak height
            min_distance: Minimum distance between peaks
        
        Returns:
            List of peak indices
        """
        if len(data) < 3:
            return []
        
        peaks = []
        
        for i in range(1, len(data) - 1):
            # Check if this is a local maximum
            if data[i] > data[i-1] and data[i] > data[i+1]:
                # Check threshold
                if threshold is None or data[i] >= threshold:
                    # Check minimum distance
                    if not peaks or i - peaks[-1] >= min_distance:
                        peaks.append(i)
        
        return peaks
    
    @staticmethod
    def calculate_rms(data: List[float]) -> float:
        """
        Calculate RMS (Root Mean Square) value
        
        Args:
            data: Input data
        
        Returns:
            RMS value
        """
        if not data:
            return 0.0
        return np.sqrt(np.mean(np.array(data) ** 2))
    
    @staticmethod
    def fft_analysis(data: List[float], sample_rate: float) -> tuple:
        """
        Perform FFT analysis
        
        Args:
            data: Input data
            sample_rate: Sampling rate (Hz)
        
        Returns:
            (frequencies, magnitudes) tuple
        """
        n = len(data)
        fft_result = np.fft.fft(data)
        frequencies = np.fft.fftfreq(n, 1/sample_rate)
        magnitudes = np.abs(fft_result)
        
        # Return only positive frequencies
        positive_freq_idx = frequencies >= 0
        return frequencies[positive_freq_idx], magnitudes[positive_freq_idx]
    
    @staticmethod
    def normalize(data: List[float], min_val: float = 0.0, 
                 max_val: float = 1.0) -> List[float]:
        """
        Normalize data to specified range
        
        Args:
            data: Input data
            min_val: Minimum value of output range
            max_val: Maximum value of output range
        
        Returns:
            Normalized data
        """
        if not data:
            return []
        
        data_min = min(data)
        data_max = max(data)
        
        if data_max == data_min:
            return [min_val] * len(data)
        
        scale = (max_val - min_val) / (data_max - data_min)
        return [(x - data_min) * scale + min_val for x in data]


class MovingAverageFilter:
    """Real-time moving average filter using circular buffer"""
    
    def __init__(self, window_size: int):
        """
        Initialize filter
        
        Args:
            window_size: Size of moving average window
        """
        self.window_size = window_size
        self.buffer = deque(maxlen=window_size)
        self.sum = 0.0
    
    def add_sample(self, value: float) -> float:
        """
        Add sample and get filtered output
        
        Args:
            value: New sample value
        
        Returns:
            Filtered value
        """
        # Remove oldest value from sum if buffer is full
        if len(self.buffer) == self.window_size:
            self.sum -= self.buffer[0]
        
        # Add new value
        self.buffer.append(value)
        self.sum += value
        
        # Calculate average
        return self.sum / len(self.buffer)
    
    def reset(self):
        """Reset filter state"""
        self.buffer.clear()
        self.sum = 0.0


# Example usage
if __name__ == '__main__':
    # Generate test signal
    import matplotlib.pyplot as plt
    
    # Noisy sine wave
    t = np.linspace(0, 2*np.pi, 100)
    clean_signal = np.sin(t)
    noise = np.random.normal(0, 0.2, len(t))
    noisy_signal = clean_signal + noise
    
    # Apply filters
    processor = SignalProcessor()
    
    ma_filtered = processor.moving_average(noisy_signal.tolist(), window_size=5)
    median_filtered = processor.median_filter(noisy_signal.tolist(), window_size=5)
    lp_filtered = processor.low_pass_filter(noisy_signal.tolist(), alpha=0.3)
    
    # Plot results
    plt.figure(figsize=(12, 8))
    
    plt.subplot(2, 2, 1)
    plt.plot(t, noisy_signal, 'gray', alpha=0.5, label='Noisy')
    plt.plot(t, clean_signal, 'b', label='Original')
    plt.title('Original Signal')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(2, 2, 2)
    plt.plot(t, noisy_signal, 'gray', alpha=0.5, label='Noisy')
    plt.plot(t, ma_filtered, 'r', label='Moving Average')
    plt.title('Moving Average Filter')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(2, 2, 3)
    plt.plot(t, noisy_signal, 'gray', alpha=0.5, label='Noisy')
    plt.plot(t, median_filtered, 'g', label='Median')
    plt.title('Median Filter')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(2, 2, 4)
    plt.plot(t, noisy_signal, 'gray', alpha=0.5, label='Noisy')
    plt.plot(t, lp_filtered, 'orange', label='Low-pass')
    plt.title('Low-pass Filter')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

