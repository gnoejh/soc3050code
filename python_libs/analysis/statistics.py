"""
Statistical Analysis for ATmega128 Sensor Data
Calculate statistics, detect anomalies, analyze trends

© 2025 Prof. Hong Jeong, IUT
"""

import numpy as np
from typing import List, Dict, Any, Optional
import statistics


class StatisticsAnalyzer:
    """
    Statistical analysis tools for sensor data
    
    Features:
    - Descriptive statistics
    - Outlier detection
    - Trend analysis
    - Data quality metrics
    """
    
    @staticmethod
    def calculate_stats(data: List[float]) -> Dict[str, float]:
        """
        Calculate comprehensive statistics
        
        Args:
            data: Input data
        
        Returns:
            Dictionary of statistical measures
        """
        if not data:
            return {}
        
        np_data = np.array(data)
        
        return {
            'count': len(data),
            'mean': float(np.mean(np_data)),
            'median': float(np.median(np_data)),
            'std': float(np.std(np_data)),
            'var': float(np.var(np_data)),
            'min': float(np.min(np_data)),
            'max': float(np.max(np_data)),
            'range': float(np.max(np_data) - np.min(np_data)),
            'q25': float(np.percentile(np_data, 25)),
            'q75': float(np.percentile(np_data, 75)),
            'iqr': float(np.percentile(np_data, 75) - np.percentile(np_data, 25))
        }
    
    @staticmethod
    def detect_outliers(data: List[float], method: str = 'iqr', 
                       threshold: float = 1.5) -> List[int]:
        """
        Detect outliers in data
        
        Args:
            data: Input data
            method: Detection method ('iqr', 'zscore')
            threshold: Outlier threshold
        
        Returns:
            List of outlier indices
        """
        if len(data) < 3:
            return []
        
        np_data = np.array(data)
        outliers = []
        
        if method == 'iqr':
            # Interquartile range method
            q25 = np.percentile(np_data, 25)
            q75 = np.percentile(np_data, 75)
            iqr = q75 - q25
            
            lower_bound = q25 - threshold * iqr
            upper_bound = q75 + threshold * iqr
            
            for i, value in enumerate(data):
                if value < lower_bound or value > upper_bound:
                    outliers.append(i)
        
        elif method == 'zscore':
            # Z-score method
            mean = np.mean(np_data)
            std = np.std(np_data)
            
            if std > 0:
                for i, value in enumerate(data):
                    z_score = abs((value - mean) / std)
                    if z_score > threshold:
                        outliers.append(i)
        
        return outliers
    
    @staticmethod
    def calculate_trend(data: List[float]) -> Dict[str, Any]:
        """
        Calculate linear trend
        
        Args:
            data: Input data
        
        Returns:
            Dictionary with trend information
        """
        if len(data) < 2:
            return {'slope': 0, 'direction': 'flat'}
        
        x = np.arange(len(data))
        y = np.array(data)
        
        # Linear regression
        coefficients = np.polyfit(x, y, 1)
        slope = coefficients[0]
        intercept = coefficients[1]
        
        # Determine trend direction
        if abs(slope) < 0.001:
            direction = 'flat'
        elif slope > 0:
            direction = 'increasing'
        else:
            direction = 'decreasing'
        
        return {
            'slope': float(slope),
            'intercept': float(intercept),
            'direction': direction,
            'change_per_sample': float(slope),
            'total_change': float(slope * len(data))
        }
    
    @staticmethod
    def calculate_correlation(data1: List[float], data2: List[float]) -> float:
        """
        Calculate correlation between two data series
        
        Args:
            data1: First data series
            data2: Second data series
        
        Returns:
            Correlation coefficient (-1 to 1)
        """
        if len(data1) != len(data2) or len(data1) < 2:
            return 0.0
        
        return float(np.corrcoef(data1, data2)[0, 1])
    
    @staticmethod
    def data_quality_score(data: List[float]) -> Dict[str, Any]:
        """
        Assess data quality
        
        Args:
            data: Input data
        
        Returns:
            Quality metrics
        """
        if not data:
            return {'score': 0, 'issues': ['No data']}
        
        issues = []
        score = 100
        
        # Check for missing/invalid values
        if None in data or np.nan in data:
            issues.append('Contains invalid values')
            score -= 30
        
        # Check for constant data
        if len(set(data)) == 1:
            issues.append('Data is constant')
            score -= 20
        
        # Check for excessive noise
        std = np.std(data)
        mean = np.mean(data)
        if mean != 0:
            cv = std / abs(mean)  # Coefficient of variation
            if cv > 0.5:
                issues.append('High noise level')
                score -= 15
        
        # Check for outliers
        outliers = StatisticsAnalyzer.detect_outliers(data)
        outlier_percent = len(outliers) / len(data) * 100
        if outlier_percent > 10:
            issues.append(f'{outlier_percent:.1f}% outliers')
            score -= 10
        
        # Check sample size
        if len(data) < 10:
            issues.append('Small sample size')
            score -= 10
        
        return {
            'score': max(0, score),
            'issues': issues if issues else ['Good quality'],
            'sample_size': len(data),
            'outlier_percent': outlier_percent
        }


# Example usage
if __name__ == '__main__':
    # Generate test data
    np.random.seed(42)
    test_data = [20 + i*0.1 + np.random.normal(0, 0.5) for i in range(100)]
    
    # Add some outliers
    test_data[50] = 50  # Outlier
    test_data[75] = 5   # Outlier
    
    analyzer = StatisticsAnalyzer()
    
    # Calculate statistics
    print("=" * 70)
    print("STATISTICAL ANALYSIS")
    print("=" * 70)
    
    stats = analyzer.calculate_stats(test_data)
    print("\nDescriptive Statistics:")
    for key, value in stats.items():
        print(f"  {key:12s}: {value:.3f}")
    
    # Detect outliers
    print("\nOutlier Detection:")
    outliers = analyzer.detect_outliers(test_data, method='iqr')
    print(f"  Found {len(outliers)} outliers at indices: {outliers}")
    
    # Trend analysis
    print("\nTrend Analysis:")
    trend = analyzer.calculate_trend(test_data)
    for key, value in trend.items():
        print(f"  {key:18s}: {value}")
    
    # Data quality
    print("\nData Quality Assessment:")
    quality = analyzer.data_quality_score(test_data)
    print(f"  Quality Score: {quality['score']}/100")
    print(f"  Issues: {', '.join(quality['issues'])}")

