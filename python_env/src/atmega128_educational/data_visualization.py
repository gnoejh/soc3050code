#!/usr/bin/env python3
"""
Data Visualization for ATmega128 Educational Framework
Creates real-time and static plots of sensor data
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.dates import DateFormatter
import numpy as np
import pandas as pd
from datetime import datetime, timedelta
import seaborn as sns

# Set style for better looking plots
plt.style.use('seaborn-v0_8')
sns.set_palette("husl")

class DataVisualizer:
    def __init__(self, data=None):
        """Initialize data visualizer"""
        self.data = data if data else []
        self.fig = None
        self.axes = None
        
    def load_csv_data(self, filename):
        """Load sensor data from CSV file"""
        try:
            df = pd.read_csv(filename)
            # Convert timestamp column to datetime with flexible parsing
            df['timestamp'] = pd.to_datetime(df['timestamp'], format='mixed', errors='coerce')
            
            # Convert to list of dictionaries for compatibility
            self.data = df.to_dict('records')
            print(f"✅ Loaded {len(self.data)} data points from {filename}")
            return True
        except Exception as e:
            print(f"❌ Error loading CSV: {e}")
            return False
    
    def plot_sensor_data(self, save_filename=None):
        """Create static plots of all sensor data"""
        if not self.data:
            print("❌ No data to visualize")
            return
        
        # Extract data for plotting
        timestamps = [d['timestamp'] for d in self.data if isinstance(d['timestamp'], datetime)]
        temperatures = [d['temperature'] for d in self.data if d['temperature'] is not None]
        light_values = [d['light'] for d in self.data if d['light'] is not None]
        sound_values = [d['sound'] for d in self.data if d['sound'] is not None]
        
        # Create subplots
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        fig.suptitle('ATmega128 Sensor Data Analysis', fontsize=16, fontweight='bold')
        
        # Temperature plot
        if temperatures and timestamps:
            temp_times = [timestamps[i] for i in range(len(temperatures))]
            axes[0].plot(temp_times, temperatures, 'r-', linewidth=2, marker='o', markersize=4)
            axes[0].set_title('🌡️ Temperature Sensor', fontweight='bold')
            axes[0].set_ylabel('Temperature (°C)')
            axes[0].grid(True, alpha=0.3)
            axes[0].tick_params(axis='x', rotation=45)
            
            # Add statistics
            avg_temp = sum(temperatures) / len(temperatures)
            axes[0].axhline(y=avg_temp, color='r', linestyle='--', alpha=0.7, 
                           label=f'Average: {avg_temp:.1f}°C')
            axes[0].legend()
        
        # Light sensor plot
        if light_values and timestamps:
            light_times = [timestamps[i] for i in range(len(light_values))]
            axes[1].plot(light_times, light_values, 'g-', linewidth=2, marker='s', markersize=4)
            axes[1].set_title('💡 Light Sensor (ADC Value)', fontweight='bold')
            axes[1].set_ylabel('ADC Value (0-1023)')
            axes[1].grid(True, alpha=0.3)
            axes[1].tick_params(axis='x', rotation=45)
            
            # Add statistics
            avg_light = sum(light_values) / len(light_values)
            axes[1].axhline(y=avg_light, color='g', linestyle='--', alpha=0.7,
                           label=f'Average: {avg_light:.0f}')
            axes[1].legend()
        
        # Sound sensor plot
        if sound_values and timestamps:
            sound_times = [timestamps[i] for i in range(len(sound_values))]
            axes[2].plot(sound_times, sound_values, 'b-', linewidth=2, marker='^', markersize=4)
            axes[2].set_title('🔊 Sound Sensor (ADC Value)', fontweight='bold')
            axes[2].set_ylabel('ADC Value (0-1023)')
            axes[2].set_xlabel('Time')
            axes[2].grid(True, alpha=0.3)
            axes[2].tick_params(axis='x', rotation=45)
            
            # Add statistics
            avg_sound = sum(sound_values) / len(sound_values)
            axes[2].axhline(y=avg_sound, color='b', linestyle='--', alpha=0.7,
                           label=f'Average: {avg_sound:.0f}')
            axes[2].legend()
        
        # Format x-axis for all subplots
        for ax in axes:
            ax.xaxis.set_major_formatter(DateFormatter('%H:%M:%S'))
            ax.tick_params(axis='x', rotation=45)
        
        plt.tight_layout()
        
        # Save if filename provided
        if save_filename:
            plt.savefig(save_filename, dpi=300, bbox_inches='tight')
            print(f"💾 Plot saved as: {save_filename}")
        
        plt.show()
    
    def plot_correlation_matrix(self):
        """Plot correlation matrix between sensors"""
        if not self.data:
            print("❌ No data to visualize")
            return
        
        # Create DataFrame
        df_data = {
            'Temperature': [d['temperature'] for d in self.data if d['temperature'] is not None],
            'Light': [d['light'] for d in self.data if d['light'] is not None],
            'Sound': [d['sound'] for d in self.data if d['sound'] is not None]
        }
        
        # Find minimum length
        min_length = min(len(v) for v in df_data.values()) if df_data else 0
        
        if min_length < 2:
            print("❌ Not enough data for correlation analysis")
            return
        
        # Truncate to same length
        for key in df_data:
            df_data[key] = df_data[key][:min_length]
        
        df = pd.DataFrame(df_data)
        
        # Create correlation matrix
        plt.figure(figsize=(8, 6))
        correlation_matrix = df.corr()
        
        sns.heatmap(correlation_matrix, annot=True, cmap='coolwarm', center=0,
                    square=True, fmt='.3f', cbar_kws={'label': 'Correlation Coefficient'})
        
        plt.title('Sensor Data Correlation Matrix', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.show()
    
    def plot_distribution_analysis(self):
        """Plot distribution analysis of sensor data"""
        if not self.data:
            print("❌ No data to visualize")
            return
        
        # Extract data
        temperatures = [d['temperature'] for d in self.data if d['temperature'] is not None]
        light_values = [d['light'] for d in self.data if d['light'] is not None]
        sound_values = [d['sound'] for d in self.data if d['sound'] is not None]
        
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        fig.suptitle('Sensor Data Distribution Analysis', fontsize=16, fontweight='bold')
        
        # Temperature distribution
        if temperatures:
            axes[0].hist(temperatures, bins=20, alpha=0.7, color='red', edgecolor='black')
            axes[0].set_title('🌡️ Temperature Distribution')
            axes[0].set_xlabel('Temperature (°C)')
            axes[0].set_ylabel('Frequency')
            axes[0].grid(True, alpha=0.3)
        
        # Light distribution
        if light_values:
            axes[1].hist(light_values, bins=20, alpha=0.7, color='green', edgecolor='black')
            axes[1].set_title('💡 Light Distribution')
            axes[1].set_xlabel('ADC Value')
            axes[1].set_ylabel('Frequency')
            axes[1].grid(True, alpha=0.3)
        
        # Sound distribution
        if sound_values:
            axes[2].hist(sound_values, bins=20, alpha=0.7, color='blue', edgecolor='black')
            axes[2].set_title('🔊 Sound Distribution')
            axes[2].set_xlabel('ADC Value')
            axes[2].set_ylabel('Frequency')
            axes[2].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
    
    def create_dashboard(self, save_filename=None):
        """Create comprehensive dashboard with all visualizations"""
        if not self.data:
            print("❌ No data to visualize")
            return
        
        # Create figure with subplots
        fig = plt.figure(figsize=(16, 12))
        gs = fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)
        
        # Title
        fig.suptitle('ATmega128 Educational Framework - Complete Data Analysis Dashboard', 
                     fontsize=18, fontweight='bold', y=0.95)
        
        # Extract data
        timestamps = [d['timestamp'] for d in self.data if isinstance(d['timestamp'], datetime)]
        temperatures = [d['temperature'] for d in self.data if d['temperature'] is not None]
        light_values = [d['light'] for d in self.data if d['light'] is not None]
        sound_values = [d['sound'] for d in self.data if d['sound'] is not None]
        
        # Time series plots (top row)
        if temperatures and timestamps:
            ax1 = fig.add_subplot(gs[0, 0])
            temp_times = timestamps[:len(temperatures)]
            ax1.plot(temp_times, temperatures, 'r-', linewidth=2)
            ax1.set_title('🌡️ Temperature Over Time')
            ax1.set_ylabel('°C')
            ax1.grid(True, alpha=0.3)
        
        if light_values and timestamps:
            ax2 = fig.add_subplot(gs[0, 1])
            light_times = timestamps[:len(light_values)]
            ax2.plot(light_times, light_values, 'g-', linewidth=2)
            ax2.set_title('💡 Light Over Time')
            ax2.set_ylabel('ADC Value')
            ax2.grid(True, alpha=0.3)
        
        if sound_values and timestamps:
            ax3 = fig.add_subplot(gs[0, 2])
            sound_times = timestamps[:len(sound_values)]
            ax3.plot(sound_times, sound_values, 'b-', linewidth=2)
            ax3.set_title('🔊 Sound Over Time')
            ax3.set_ylabel('ADC Value')
            ax3.grid(True, alpha=0.3)
        
        # Distribution plots (middle row)
        if temperatures:
            ax4 = fig.add_subplot(gs[1, 0])
            ax4.hist(temperatures, bins=15, alpha=0.7, color='red', edgecolor='black')
            ax4.set_title('Temperature Distribution')
            ax4.set_xlabel('°C')
            ax4.set_ylabel('Count')
        
        if light_values:
            ax5 = fig.add_subplot(gs[1, 1])
            ax5.hist(light_values, bins=15, alpha=0.7, color='green', edgecolor='black')
            ax5.set_title('Light Distribution')
            ax5.set_xlabel('ADC Value')
            ax5.set_ylabel('Count')
        
        if sound_values:
            ax6 = fig.add_subplot(gs[1, 2])
            ax6.hist(sound_values, bins=15, alpha=0.7, color='blue', edgecolor='black')
            ax6.set_title('Sound Distribution')
            ax6.set_xlabel('ADC Value')
            ax6.set_ylabel('Count')
        
        # Summary statistics (bottom row)
        ax7 = fig.add_subplot(gs[2, :])
        ax7.axis('off')
        
        # Calculate statistics
        stats_text = "📊 SENSOR STATISTICS SUMMARY\n\n"
        
        if temperatures:
            stats_text += f"🌡️ Temperature: {min(temperatures):.1f}°C - {max(temperatures):.1f}°C "
            stats_text += f"(avg: {sum(temperatures)/len(temperatures):.1f}°C, std: {np.std(temperatures):.1f}°C)\n"
        
        if light_values:
            stats_text += f"💡 Light: {min(light_values)} - {max(light_values)} ADC "
            stats_text += f"(avg: {sum(light_values)//len(light_values)}, std: {np.std(light_values):.0f})\n"
        
        if sound_values:
            stats_text += f"🔊 Sound: {min(sound_values)} - {max(sound_values)} ADC "
            stats_text += f"(avg: {sum(sound_values)//len(sound_values)}, std: {np.std(sound_values):.0f})\n"
        
        stats_text += f"\n🔢 Total samples: {len(self.data)}"
        stats_text += f"\n⏰ Collection time: {timestamps[0].strftime('%H:%M:%S')} - {timestamps[-1].strftime('%H:%M:%S')}" if timestamps else ""
        
        ax7.text(0.5, 0.5, stats_text, transform=ax7.transAxes, fontsize=12,
                ha='center', va='center', bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.8))
        
        # Save if requested
        if save_filename:
            plt.savefig(save_filename, dpi=300, bbox_inches='tight')
            print(f"💾 Dashboard saved as: {save_filename}")
        
        plt.show()

def main():
    """Main function for standalone visualization"""
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python data_visualization.py <csv_filename>")
        print("Example: python data_visualization.py sensor_data_20241201_143022.csv")
        return
    
    filename = sys.argv[1]
    
    print("📈 ATmega128 Data Visualizer")
    print("=" * 30)
    print(f"📁 Loading data from: {filename}")
    
    viz = DataVisualizer()
    
    if viz.load_csv_data(filename):
        print("\n📊 Available visualizations:")
        print("1. Time series plots")
        print("2. Distribution analysis") 
        print("3. Correlation matrix")
        print("4. Complete dashboard")
        
        try:
            choice = input("\nSelect visualization (1-4, or 'all'): ").strip()
            
            if choice == '1':
                viz.plot_sensor_data()
            elif choice == '2':
                viz.plot_distribution_analysis()
            elif choice == '3':
                viz.plot_correlation_matrix()
            elif choice == '4':
                viz.create_dashboard()
            elif choice.lower() == 'all':
                viz.plot_sensor_data()
                viz.plot_distribution_analysis()
                viz.plot_correlation_matrix()
                viz.create_dashboard()
            else:
                print("Invalid choice, showing dashboard...")
                viz.create_dashboard()
                
        except KeyboardInterrupt:
            print("\n👋 Visualization cancelled")

if __name__ == "__main__":
    main()