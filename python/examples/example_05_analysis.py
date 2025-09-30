# Example 5: Data Analysis and Machine Learning
# Learn how to analyze sensor data and create simple predictions

from atmega128 import ATmega128
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime, timedelta
import time

print("🤖 Example 5: Data Analysis & ML")
print("=" * 40)

def collect_training_data(mcu, duration_minutes=5):
    """Collect sensor data for training."""
    print(f"📊 Collecting {duration_minutes} minutes of training data...")
    
    data = []
    start_time = datetime.now()
    end_time = start_time + timedelta(minutes=duration_minutes)
    
    while datetime.now() < end_time:
        sensors = mcu.read_sensors()
        if sensors:
            record = {
                'timestamp': datetime.now(),
                'temperature': sensors.get('TEMP', 0),
                'humidity': sensors.get('HUMID', 0),
                'light': sensors.get('LIGHT', 0)
            }
            data.append(record)
            
            # Show progress
            elapsed = (datetime.now() - start_time).seconds
            remaining = duration_minutes * 60 - elapsed
            print(f"⏱️  {remaining}s remaining | Temp: {record['temperature']:.1f}°C", end='\r')
            
        time.sleep(1)
    
    print(f"\n✅ Collected {len(data)} data points!")
    return pd.DataFrame(data)

def analyze_data(df):
    """Perform basic data analysis."""
    print("\n📈 Data Analysis Results:")
    print("=" * 30)
    
    # Basic statistics
    print("📊 Basic Statistics:")
    for column in ['temperature', 'humidity', 'light']:
        if column in df.columns:
            mean_val = df[column].mean()
            std_val = df[column].std()
            min_val = df[column].min()
            max_val = df[column].max()
            print(f"  {column.title()}:")
            print(f"    Mean: {mean_val:.2f}")
            print(f"    Std:  {std_val:.2f}")
            print(f"    Range: {min_val:.2f} - {max_val:.2f}")
    
    # Correlations
    print("\n🔗 Correlations:")
    numeric_cols = ['temperature', 'humidity', 'light']
    numeric_cols = [col for col in numeric_cols if col in df.columns]
    correlations = df[numeric_cols].corr()
    print(correlations)
    
    # Trends
    print("\n📈 Trends (per minute):")
    df['minute'] = df['timestamp'].dt.floor('min')
    trends = df.groupby('minute')[numeric_cols].mean().diff().mean()
    for col, trend in trends.items():
        direction = "↗️" if trend > 0 else "↘️" if trend < 0 else "➡️"
        print(f"  {col.title()}: {direction} {trend:.3f}/min")

def create_simple_predictor(df):
    """Create a simple linear predictor."""
    print("\n🤖 Creating Simple Predictor:")
    print("=" * 30)
    
    if len(df) < 10:
        print("❌ Need more data for prediction!")
        return None
    
    # Simple linear regression for temperature prediction
    # Using time elapsed (in minutes) to predict temperature
    df['elapsed_minutes'] = (df['timestamp'] - df['timestamp'].min()).dt.total_seconds() / 60
    
    # Simple slope calculation
    x = df['elapsed_minutes'].values
    y = df['temperature'].values
    
    # Linear regression: y = mx + b
    m = np.polyfit(x, y, 1)[0]  # slope
    b = np.polyfit(x, y, 1)[1]  # intercept
    
    print(f"📐 Temperature trend: {m:.3f}°C per minute")
    
    # Predict next 5 minutes
    current_time = df['elapsed_minutes'].max()
    predictions = []
    for i in range(1, 6):
        future_time = current_time + i
        predicted_temp = m * future_time + b
        predictions.append({
            'minutes_ahead': i,
            'predicted_temp': predicted_temp
        })
        print(f"🔮 +{i} min: {predicted_temp:.1f}°C")
    
    return predictions

def plot_analysis(df, predictions=None):
    """Create analysis plots."""
    print("\n📊 Creating plots...")
    
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle('ATmega128 Sensor Data Analysis')
    
    # Time series plot
    ax1.plot(df['timestamp'], df['temperature'], 'r-', label='Temperature')
    ax1.set_title('Temperature Over Time')
    ax1.set_ylabel('Temperature (°C)')
    ax1.grid(True)
    ax1.legend()
    
    # Distribution histogram
    ax2.hist(df['temperature'], bins=20, alpha=0.7, color='red')
    ax2.set_title('Temperature Distribution')
    ax2.set_xlabel('Temperature (°C)')
    ax2.set_ylabel('Frequency')
    ax2.grid(True)
    
    # Correlation plot
    if 'humidity' in df.columns and 'light' in df.columns:
        ax3.scatter(df['temperature'], df['humidity'], alpha=0.6)
        ax3.set_title('Temperature vs Humidity')
        ax3.set_xlabel('Temperature (°C)')
        ax3.set_ylabel('Humidity (%)')
        ax3.grid(True)
        
        # Multi-sensor plot
        ax4.plot(df['timestamp'], df['temperature'], 'r-', label='Temp')
        ax4_twin = ax4.twinx()
        ax4_twin.plot(df['timestamp'], df['light'], 'g-', label='Light')
        ax4.set_title('Multi-sensor View')
        ax4.set_ylabel('Temperature (°C)', color='red')
        ax4_twin.set_ylabel('Light Level', color='green')
        ax4.grid(True)
    else:
        # Prediction plot if available
        if predictions:
            current_temp = df['temperature'].iloc[-1]
            future_temps = [p['predicted_temp'] for p in predictions]
            times = list(range(1, 6))
            
            ax3.plot([0] + times, [current_temp] + future_temps, 'b-o')
            ax3.set_title('Temperature Prediction')
            ax3.set_xlabel('Minutes Ahead')
            ax3.set_ylabel('Predicted Temperature (°C)')
            ax3.grid(True)
    
    plt.tight_layout()
    
    # Save plot
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"sensor_analysis_{timestamp}.png"
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    print(f"💾 Plot saved as: {filename}")
    
    plt.show()

def main():
    """Main analysis workflow."""
    # Connect to ATmega128
    mcu = ATmega128("COM3")  # Change COM3 to your port
    
    if not mcu.connected:
        print("❌ Could not connect. Check your port!")
        return
    
    print("✅ Connected successfully!")
    
    try:
        # Collect data
        df = collect_training_data(mcu, duration_minutes=2)  # Short demo
        
        # Save raw data
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = f"sensor_data_{timestamp}.csv"
        df.to_csv(csv_filename, index=False)
        print(f"💾 Raw data saved as: {csv_filename}")
        
        # Analyze data
        analyze_data(df)
        
        # Create predictor
        predictions = create_simple_predictor(df)
        
        # Create plots
        plot_analysis(df, predictions)
        
        print("\n🎉 Analysis complete!")
        print("\n📚 What you learned:")
        print("  ✅ Data collection and storage")
        print("  ✅ Statistical analysis")
        print("  ✅ Trend detection")
        print("  ✅ Simple prediction modeling")
        print("  ✅ Data visualization")
        
        print("\n🚀 Next steps:")
        print("  - Try longer data collection periods")
        print("  - Experiment with different sensors")
        print("  - Learn more advanced ML techniques")
        print("  - Create automated alert systems")
        
    except KeyboardInterrupt:
        print("\n⏹️ Analysis stopped by user")
    finally:
        mcu.close()
        print("🔌 Disconnected")

if __name__ == "__main__":
    main()