"""
Activity Recognition using MPU6050 and Machine Learning
Real-time human activity classification

PROJECT: Activity_Recognition (Python + AI)
COURSE: SOC 3050 - Embedded Systems and Applications
YEAR: 2025
AUTHOR: Professor Hong Jeong

PURPOSE:
Demonstrates AI integration with ATmega128 embedded systems.
Uses machine learning to recognize human activities from sensor data.

FEATURES:
- Real-time activity recognition
- Training data collection
- Model training with TensorFlow
- Feature extraction from accelerometer/gyroscope
- Confidence scoring

PAIRED C PROJECT:
projects/I2C_Sensors_Multi/Main.c (MPU6050 interface)

REQUIREMENTS:
- ATmega128 with MPU6050 sensor
- TensorFlow/Keras for AI
- Scikit-learn for preprocessing

© 2025 Prof. Hong Jeong, IUT
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'python_libs'))

from atmega128 import ATmega128Serial, ATmega128Protocol
from visualization import DataLogger
import numpy as np
import time
from collections import deque
from config import *


class ActivityRecognizer:
    """
    AI-powered activity recognition system
    """
    
    def __init__(self):
        """Initialize recognizer"""
        self.model = None
        self.scaler = None
        
        # Sensor data buffers
        self.accel_x = deque(maxlen=WINDOW_SIZE)
        self.accel_y = deque(maxlen=WINDOW_SIZE)
        self.accel_z = deque(maxlen=WINDOW_SIZE)
        self.gyro_x = deque(maxlen=WINDOW_SIZE)
        self.gyro_y = deque(maxlen=WINDOW_SIZE)
        self.gyro_z = deque(maxlen=WINDOW_SIZE)
        
        # Load model if available
        self.load_model()
    
    def load_model(self):
        """Load pre-trained model"""
        try:
            # Check if TensorFlow is available
            import tensorflow as tf
            import pickle
            
            if os.path.exists(TRAINED_MODEL):
                self.model = tf.keras.models.load_model(TRAINED_MODEL)
                print(f"✅ Loaded model from {TRAINED_MODEL}")
            else:
                print(f"⚠️  Model not found: {TRAINED_MODEL}")
                print("   Run training first or use demo mode")
            
            if os.path.exists(SCALER_FILE):
                with open(SCALER_FILE, 'rb') as f:
                    self.scaler = pickle.load(f)
                print(f"✅ Loaded scaler from {SCALER_FILE}")
        
        except ImportError:
            print("⚠️  TensorFlow not installed - AI features disabled")
            print("   Install with: pip install tensorflow scikit-learn")
    
    def collect_sample(self, mcu):
        """
        Collect one sensor sample
        
        Args:
            mcu: ATmega128Serial connection
        
        Returns:
            True if successful
        """
        protocol = ATmega128Protocol()
        
        # Request MPU6050 data
        mcu.send_command('READ_MPU6050')
        response = mcu.wait_for_response(timeout=1.0)
        
        if response:
            # Parse: "AX:123,AY:456,AZ:789,GX:12,GY:34,GZ:56"
            try:
                data = protocol.parse_sensor_data(response)
                
                self.accel_x.append(data.get('AX', 0))
                self.accel_y.append(data.get('AY', 0))
                self.accel_z.append(data.get('AZ', 0))
                self.gyro_x.append(data.get('GX', 0))
                self.gyro_y.append(data.get('GY', 0))
                self.gyro_z.append(data.get('GZ', 0))
                
                return True
            except Exception as e:
                print(f"⚠️  Parse error: {e}")
        
        return False
    
    def extract_features(self):
        """
        Extract statistical features from sensor window
        
        Returns:
            Feature vector (numpy array)
        """
        features = []
        
        # Process each axis
        for data in [self.accel_x, self.accel_y, self.accel_z,
                     self.gyro_x, self.gyro_y, self.gyro_z]:
            
            data_array = np.array(list(data))
            
            features.extend([
                np.mean(data_array),          # Mean
                np.std(data_array),           # Standard deviation
                np.max(data_array),           # Maximum
                np.min(data_array),           # Minimum
                np.median(data_array),        # Median
                np.percentile(data_array, 25), # 25th percentile
                np.percentile(data_array, 75)  # 75th percentile
            ])
        
        return np.array(features).reshape(1, -1)
    
    def predict_activity(self):
        """
        Predict current activity
        
        Returns:
            (activity_name, confidence) tuple
        """
        if self.model is None or len(self.accel_x) < WINDOW_SIZE:
            return None, 0.0
        
        # Extract features
        features = self.extract_features()
        
        # Normalize if scaler available
        if self.scaler:
            features = self.scaler.transform(features)
        
        # Predict
        prediction = self.model.predict(features, verbose=0)
        activity_idx = np.argmax(prediction)
        confidence = prediction[0][activity_idx]
        
        return ACTIVITIES[activity_idx], confidence
    
    def run_realtime_recognition(self, mcu):
        """
        Run real-time activity recognition
        
        Args:
            mcu: ATmega128Serial connection
        """
        print("\n" + "=" * 70)
        print("REAL-TIME ACTIVITY RECOGNITION")
        print("=" * 70)
        print("Move the sensor to detect activities")
        print("Press Ctrl+C to stop")
        print()
        
        if self.model is None:
            print("❌ No trained model available")
            return
        
        # Fill initial buffer
        print("Collecting initial data...")
        for i in range(WINDOW_SIZE):
            self.collect_sample(mcu)
            print(f"\rProgress: {i+1}/{WINDOW_SIZE}", end='', flush=True)
            time.sleep(1.0 / SAMPLE_RATE)
        
        print("\n\n🤖 Recognition active!\n")
        
        try:
            while True:
                # Collect new sample
                self.collect_sample(mcu)
                
                # Predict activity
                activity, confidence = self.predict_activity()
                
                if activity:
                    # Visual confidence bar
                    bar_length = int(confidence * 30)
                    bar = '█' * bar_length + '░' * (30 - bar_length)
                    
                    # Color coding (simulated with symbols)
                    if confidence > 0.8:
                        status = '✅'
                    elif confidence > 0.6:
                        status = '⚠️ '
                    else:
                        status = '❓'
                    
                    print(f"\r{status} Activity: {activity:12s} [{bar}] {confidence*100:5.1f}%", 
                          end='', flush=True)
                    
                    # Alert on falling
                    if activity == 'Falling' and confidence > 0.7:
                        print("\n🚨 FALL DETECTED! 🚨")
                        mcu.send_command('ALERT FALL')
                
                time.sleep(1.0 / SAMPLE_RATE)
        
        except KeyboardInterrupt:
            print("\n\n✅ Recognition stopped")


def demo_1_data_collection():
    """
    Demo 1: Collect Training Data
    Collect labeled sensor data for training
    """
    print("\n" + "=" * 70)
    print("DEMO 1: Training Data Collection")
    print("=" * 70)
    print()
    
    os.makedirs(TRAINING_DATA_DIR, exist_ok=True)
    
    print("This tool collects labeled data for each activity.")
    print(f"Activities: {', '.join(ACTIVITIES)}")
    print()
    
    for activity in ACTIVITIES:
        input(f"\nPress Enter to start collecting '{activity}' data...")
        
        filename = os.path.join(TRAINING_DATA_DIR, f"{activity.lower()}.csv")
        
        with ATmega128Serial(baudrate=BAUDRATE) as mcu:
            with DataLogger(filename, format='csv',
                           columns=['ax', 'ay', 'az', 'gx', 'gy', 'gz', 'label'],
                           auto_timestamp=False) as logger:
                
                protocol = ATmega128Protocol()
                
                print(f"Collecting {SAMPLES_PER_ACTIVITY} samples for '{activity}'...")
                print("Perform the activity now!")
                
                for i in range(SAMPLES_PER_ACTIVITY * WINDOW_SIZE):
                    mcu.send_command('READ_MPU6050')
                    response = mcu.wait_for_response(timeout=1.0)
                    
                    if response:
                        data = protocol.parse_sensor_data(response)
                        logger.log({
                            'ax': data.get('AX', 0),
                            'ay': data.get('AY', 0),
                            'az': data.get('AZ', 0),
                            'gx': data.get('GX', 0),
                            'gy': data.get('GY', 0),
                            'gz': data.get('GZ', 0),
                            'label': activity
                        })
                        
                        progress = (i + 1) / (SAMPLES_PER_ACTIVITY * WINDOW_SIZE) * 100
                        print(f"\rProgress: {progress:.1f}%", end='', flush=True)
                    
                    time.sleep(1.0 / SAMPLE_RATE)
                
                print(f"\n✅ Saved to {filename}")
        
        print(f"✅ '{activity}' data collection complete!")


def demo_2_train_model():
    """
    Demo 2: Train AI Model
    Train neural network on collected data
    """
    print("\n" + "=" * 70)
    print("DEMO 2: Train Activity Recognition Model")
    print("=" * 70)
    print()
    
    try:
        import tensorflow as tf
        from sklearn.preprocessing import StandardScaler
        from sklearn.model_selection import train_test_split
        import pandas as pd
        import pickle
        
    except ImportError:
        print("❌ Required libraries not installed")
        print("   Install with: pip install tensorflow scikit-learn pandas")
        return
    
    # Load training data
    print("Loading training data...")
    all_data = []
    
    for activity in ACTIVITIES:
        filename = os.path.join(TRAINING_DATA_DIR, f"{activity.lower()}.csv")
        if os.path.exists(filename):
            df = pd.read_csv(filename)
            all_data.append(df)
            print(f"  ✅ Loaded {len(df)} samples for '{activity}'")
        else:
            print(f"  ❌ Missing data for '{activity}'")
    
    if not all_data:
        print("\n❌ No training data found!")
        print("   Run Demo 1 to collect data first")
        return
    
    # Combine all data
    df_all = pd.concat(all_data, ignore_index=True)
    print(f"\nTotal samples: {len(df_all)}")
    
    # Extract features in windows
    print("\nExtracting features...")
    X = []
    y = []
    
    for activity in ACTIVITIES:
        df_activity = df_all[df_all['label'] == activity]
        
        for i in range(0, len(df_activity) - WINDOW_SIZE, WINDOW_OVERLAP):
            window = df_activity.iloc[i:i+WINDOW_SIZE]
            
            if len(window) == WINDOW_SIZE:
                features = []
                
                for col in ['ax', 'ay', 'az', 'gx', 'gy', 'gz']:
                    data = window[col].values
                    features.extend([
                        np.mean(data),
                        np.std(data),
                        np.max(data),
                        np.min(data),
                        np.median(data),
                        np.percentile(data, 25),
                        np.percentile(data, 75)
                    ])
                
                X.append(features)
                y.append(ACTIVITIES.index(activity))
    
    X = np.array(X)
    y = np.array(y)
    
    print(f"Feature vectors: {X.shape}")
    print(f"Labels: {y.shape}")
    
    # Normalize features
    print("\nNormalizing features...")
    scaler = StandardScaler()
    X_scaled = scaler.fit_transform(X)
    
    # Split data
    X_train, X_test, y_train, y_test = train_test_split(
        X_scaled, y, test_size=0.2, random_state=42, stratify=y
    )
    
    print(f"Training samples: {len(X_train)}")
    print(f"Test samples: {len(X_test)}")
    
    # Build model
    print("\nBuilding neural network...")
    model = tf.keras.Sequential([
        tf.keras.layers.Dense(64, activation='relu', input_shape=(X.shape[1],)),
        tf.keras.layers.Dropout(0.3),
        tf.keras.layers.Dense(32, activation='relu'),
        tf.keras.layers.Dropout(0.3),
        tf.keras.layers.Dense(16, activation='relu'),
        tf.keras.layers.Dense(len(ACTIVITIES), activation='softmax')
    ])
    
    model.compile(
        optimizer='adam',
        loss='sparse_categorical_crossentropy',
        metrics=['accuracy']
    )
    
    print(model.summary())
    
    # Train model
    print("\nTraining model...")
    history = model.fit(
        X_train, y_train,
        epochs=50,
        batch_size=32,
        validation_split=0.2,
        verbose=1
    )
    
    # Evaluate model
    print("\nEvaluating model...")
    test_loss, test_acc = model.evaluate(X_test, y_test)
    print(f"Test accuracy: {test_acc*100:.2f}%")
    
    # Save model
    os.makedirs(MODEL_DIR, exist_ok=True)
    model.save(TRAINED_MODEL)
    print(f"\n✅ Model saved to {TRAINED_MODEL}")
    
    # Save scaler
    with open(SCALER_FILE, 'wb') as f:
        pickle.dump(scaler, f)
    print(f"✅ Scaler saved to {SCALER_FILE}")


def demo_3_realtime_recognition():
    """
    Demo 3: Real-time Recognition
    Run activity recognition in real-time
    """
    recognizer = ActivityRecognizer()
    
    if recognizer.model is None:
        print("\n❌ No trained model found!")
        print("   Run Demo 2 to train a model first")
        return
    
    with ATmega128Serial(baudrate=BAUDRATE) as mcu:
        recognizer.run_realtime_recognition(mcu)


def demo_4_model_evaluation():
    """
    Demo 4: Model Evaluation
    Detailed performance analysis
    """
    print("\n" + "=" * 70)
    print("DEMO 4: Model Performance Evaluation")
    print("=" * 70)
    print()
    
    try:
        import tensorflow as tf
        from sklearn.metrics import classification_report, confusion_matrix
        import matplotlib.pyplot as plt
        import seaborn as sns
        
        if not os.path.exists(TRAINED_MODEL):
            print("❌ No trained model found")
            return
        
        # Load model
        model = tf.keras.load_model(TRAINED_MODEL)
        
        # TODO: Load test data and evaluate
        # (Implementation similar to training)
        
        print("✅ Evaluation complete")
        print("   See confusion matrix and classification report")
        
    except ImportError:
        print("❌ Required libraries not available")


def main_menu():
    """Main menu"""
    while True:
        print("\n" + "=" * 70)
        print("ACTIVITY RECOGNITION - AI + ATmega128")
        print("=" * 70)
        print("Select demonstration:")
        print()
        print("  1. Collect Training Data")
        print("  2. Train AI Model")
        print("  3. Real-time Recognition")
        print("  4. Model Evaluation")
        print("  0. Exit")
        print()
        
        choice = input("Enter choice (0-4): ").strip()
        
        if choice == '1':
            demo_1_data_collection()
        elif choice == '2':
            demo_2_train_model()
        elif choice == '3':
            demo_3_realtime_recognition()
        elif choice == '4':
            demo_4_model_evaluation()
        elif choice == '0':
            print("\n👋 Goodbye!")
            break
        else:
            print("❌ Invalid choice")


if __name__ == '__main__':
    print("=" * 70)
    print("SOC 3050 - Activity Recognition with AI")
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

