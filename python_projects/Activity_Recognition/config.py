"""
Configuration for Activity_Recognition Python Project
AI-powered activity recognition using MPU6050 sensor data

© 2025 Prof. Hong Jeong, IUT
"""

# Serial Communication Settings
BAUDRATE = 9600
TIMEOUT = 1.0

# MPU6050 Settings
SAMPLE_RATE = 50  # Hz (50 samples per second)
WINDOW_SIZE = 50  # Number of samples per classification window
WINDOW_OVERLAP = 25  # Overlap between windows

# Activity Labels
ACTIVITIES = [
    'Walking',
    'Running',
    'Sitting',
    'Standing',
    'Falling'
]

# Model Paths
MODEL_DIR = "models"
TRAINED_MODEL = "models/activity_model.h5"
SCALER_FILE = "models/scaler.pkl"

# Data Collection
TRAINING_DATA_DIR = "training_data"
SAMPLES_PER_ACTIVITY = 100  # Windows to collect per activity

# Feature Extraction
FEATURES_PER_AXIS = 7  # mean, std, max, min, median, q25, q75
TOTAL_FEATURES = FEATURES_PER_AXIS * 6  # 6 axes (3 accel + 3 gyro)

