import math

def calibrate_gyro_realtime(ax, ay, az, gx, gy, gz, biases):
    # Constants
    # Gravity componenet has already been removed from all the three axis
    G = 9.81  # Standard gravity in m/s²
    GYRO_THRESHOLD = 0.5  # Gyro threshold in °/s
    ALPHA = 0.01  # Low-pass filter coefficient
    
    # Calculate acceleration magnitude
    acc_magnitude = math.sqrt(ax**2 + ay**2 + az**2)
    
    # Calculate gyroscope magnitude
    gyro_magnitude = math.sqrt(gx**2 + gy**2 + gz**2)
    
    # Check if CanSat is stationary
    is_stationary = (abs(acc_magnitude) < 0.1) and (gyro_magnitude < GYRO_THRESHOLD)
    
    # Update biases if stationary
    if is_stationary:
        bias_gx = 0.99 * biases[0] + ALPHA * gx
        bias_gy = 0.99 * biases[1] + ALPHA * gy
        bias_gz = 0.99 * biases[2] + ALPHA * gz
        biases = [bias_gx, bias_gy, bias_gz]
    else:
        bias_gx, bias_gy, bias_gz = biases
    
    #
    
    return biases