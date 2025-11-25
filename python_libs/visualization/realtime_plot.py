"""
Real-time Plotting for ATmega128 Sensor Data
Matplotlib-based real-time visualization

© 2025 Prof. Hong Jeong, IUT
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
from typing import Optional, Callable, List
import numpy as np


class RealtimePlotter:
    """
    Real-time single-channel data plotter
    
    Features:
    - Smooth scrolling plot
    - Configurable window size
    - Auto-scaling or fixed range
    - Grid and labels
    """
    
    def __init__(self, max_points: int = 200, title: str = "Real-time Data",
                 ylabel: str = "Value", xlabel: str = "Sample",
                 y_range: Optional[tuple] = None, update_interval: int = 50):
        """
        Initialize real-time plotter
        
        Args:
            max_points: Maximum number of points to display
            title: Plot title
            ylabel: Y-axis label
            xlabel: X-axis label
            y_range: Fixed Y-axis range (min, max) or None for auto-scale
            update_interval: Update interval in milliseconds
        """
        self.max_points = max_points
        self.update_interval = update_interval
        
        # Data buffers
        self.time_data = deque(maxlen=max_points)
        self.value_data = deque(maxlen=max_points)
        self.sample_count = 0
        
        # Setup plot
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.line, = self.ax.plot([], [], 'b-', linewidth=2)
        
        # Configure axes
        self.ax.set_title(title, fontsize=14, fontweight='bold')
        self.ax.set_xlabel(xlabel, fontsize=12)
        self.ax.set_ylabel(ylabel, fontsize=12)
        self.ax.grid(True, alpha=0.3)
        
        # Set Y range
        if y_range:
            self.ax.set_ylim(y_range)
            self.auto_scale = False
        else:
            self.auto_scale = True
        
        # Data source callback
        self.data_callback: Optional[Callable] = None
    
    def set_data_source(self, callback: Callable):
        """
        Set callback function for data acquisition
        
        Args:
            callback: Function that returns next data value
        """
        self.data_callback = callback
    
    def add_point(self, value: float):
        """
        Add data point manually
        
        Args:
            value: Data value to add
        """
        self.time_data.append(self.sample_count)
        self.value_data.append(value)
        self.sample_count += 1
    
    def init_plot(self):
        """Initialize plot (for animation)"""
        self.line.set_data([], [])
        return self.line,
    
    def update_plot(self, frame):
        """Update plot with new data (for animation)"""
        # Get new data from callback
        if self.data_callback:
            try:
                value = self.data_callback()
                if value is not None:
                    self.add_point(value)
            except Exception as e:
                print(f"⚠️  Data callback error: {e}")
        
        # Update plot
        if len(self.time_data) > 0:
            self.line.set_data(list(self.time_data), list(self.value_data))
            
            # Update X axis
            self.ax.set_xlim(
                max(0, self.sample_count - self.max_points),
                max(self.max_points, self.sample_count)
            )
            
            # Auto-scale Y axis if enabled
            if self.auto_scale and len(self.value_data) > 0:
                y_min = min(self.value_data)
                y_max = max(self.value_data)
                margin = (y_max - y_min) * 0.1 if y_max != y_min else 1
                self.ax.set_ylim(y_min - margin, y_max + margin)
        
        return self.line,
    
    def run(self):
        """Start real-time plotting"""
        ani = animation.FuncAnimation(
            self.fig,
            self.update_plot,
            init_func=self.init_plot,
            interval=self.update_interval,
            blit=True,
            cache_frame_data=False
        )
        plt.show()
        return ani


class MultiChannelPlotter:
    """
    Real-time multi-channel data plotter
    
    Supports multiple data streams in separate subplots
    """
    
    def __init__(self, num_channels: int, channel_names: Optional[List[str]] = None,
                 max_points: int = 200, title: str = "Multi-Channel Data",
                 update_interval: int = 50):
        """
        Initialize multi-channel plotter
        
        Args:
            num_channels: Number of channels to plot
            channel_names: Names for each channel
            max_points: Maximum points per channel
            title: Overall plot title
            update_interval: Update interval in milliseconds
        """
        self.num_channels = num_channels
        self.max_points = max_points
        self.update_interval = update_interval
        self.sample_count = 0
        
        # Channel names
        if channel_names:
            self.channel_names = channel_names
        else:
            self.channel_names = [f"Channel {i+1}" for i in range(num_channels)]
        
        # Data buffers for each channel
        self.time_data = deque(maxlen=max_points)
        self.channel_data = [deque(maxlen=max_points) for _ in range(num_channels)]
        
        # Setup subplots
        self.fig, self.axes = plt.subplots(num_channels, 1, figsize=(10, 3*num_channels))
        if num_channels == 1:
            self.axes = [self.axes]
        
        self.fig.suptitle(title, fontsize=14, fontweight='bold')
        
        # Create lines for each channel
        self.lines = []
        colors = ['b', 'r', 'g', 'orange', 'purple', 'brown']
        
        for i in range(num_channels):
            line, = self.axes[i].plot([], [], color=colors[i % len(colors)], linewidth=2)
            self.lines.append(line)
            
            self.axes[i].set_ylabel(self.channel_names[i])
            self.axes[i].grid(True, alpha=0.3)
        
        self.axes[-1].set_xlabel("Sample")
        plt.tight_layout()
        
        # Data source callback
        self.data_callback: Optional[Callable] = None
    
    def set_data_source(self, callback: Callable):
        """
        Set callback function for data acquisition
        
        Args:
            callback: Function that returns list of channel values
        """
        self.data_callback = callback
    
    def add_points(self, values: List[float]):
        """
        Add data points for all channels
        
        Args:
            values: List of values (one per channel)
        """
        if len(values) != self.num_channels:
            raise ValueError(f"Expected {self.num_channels} values, got {len(values)}")
        
        self.time_data.append(self.sample_count)
        for i, value in enumerate(values):
            self.channel_data[i].append(value)
        self.sample_count += 1
    
    def init_plot(self):
        """Initialize plot (for animation)"""
        for line in self.lines:
            line.set_data([], [])
        return self.lines
    
    def update_plot(self, frame):
        """Update plot with new data (for animation)"""
        # Get new data from callback
        if self.data_callback:
            try:
                values = self.data_callback()
                if values is not None:
                    self.add_points(values)
            except Exception as e:
                print(f"⚠️  Data callback error: {e}")
        
        # Update all channels
        if len(self.time_data) > 0:
            time_list = list(self.time_data)
            
            for i in range(self.num_channels):
                # Update line data
                self.lines[i].set_data(time_list, list(self.channel_data[i]))
                
                # Update X axis
                self.axes[i].set_xlim(
                    max(0, self.sample_count - self.max_points),
                    max(self.max_points, self.sample_count)
                )
                
                # Auto-scale Y axis
                if len(self.channel_data[i]) > 0:
                    y_min = min(self.channel_data[i])
                    y_max = max(self.channel_data[i])
                    margin = (y_max - y_min) * 0.1 if y_max != y_min else 1
                    self.axes[i].set_ylim(y_min - margin, y_max + margin)
        
        return self.lines
    
    def run(self):
        """Start real-time plotting"""
        ani = animation.FuncAnimation(
            self.fig,
            self.update_plot,
            init_func=self.init_plot,
            interval=self.update_interval,
            blit=True,
            cache_frame_data=False
        )
        plt.show()
        return ani


# Example usage
if __name__ == '__main__':
    import random
    
    # Single channel example
    print("Testing single-channel plotter...")
    plotter = RealtimePlotter(
        max_points=100,
        title="Random Data Test",
        ylabel="Value",
        y_range=(0, 100)
    )
    
    # Simulate data source
    plotter.set_data_source(lambda: random.randint(0, 100))
    
    # Run (press Ctrl+C to stop)
    plotter.run()

