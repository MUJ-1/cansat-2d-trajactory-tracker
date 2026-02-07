import math
import time
import numpy as np

class KalmanAngle:
    def __init__(self, Q_angle=0.001, Q_bias=0.003, R_measure=0.03):
        """
        Q_angle: Process noise variance for the angle
        Q_bias: Process noise variance for the gyro bias
        R_measure: Measurement noise variance for the accelerometer
        """
        self.Q_angle = Q_angle
        self.Q_bias = Q_bias
        self.R_measure = R_measure
        self.angle = 0.0
        self.bias = 0.0
        self.P00 = 0.0
        self.P01 = 0.0
        self.P10 = 0.0
        self.P11 = 0.0

    def update(self, new_rate, dt, new_angle):
        # Check for NaN or inf in inputs
        if any(np.isnan(v) or np.isinf(v) for v in [new_rate, dt, new_angle]):
            print(f"Warning: Invalid input in KalmanAngle.update: new_rate={new_rate}, dt={dt}, new_angle={new_angle}")
            return self.angle  # Return last valid angle

        rate = new_rate - self.bias
        self.angle += dt * rate

        self.P00 += dt * (dt * self.P11 - self.P01 - self.P10 + self.Q_angle)
        self.P01 -= dt * self.P11
        self.P10 -= dt * self.P11
        self.P11 += self.Q_bias * dt

        y = new_angle - self.angle
        S = self.P00 + self.R_measure
        # Prevent division by near-zero
        if abs(S) < 1e-6:
            print(f"Warning: Small S={S} in KalmanAngle.update, skipping update")
            return self.angle

        K0 = self.P00 / S
        K1 = self.P10 / S

        self.angle += K0 * y
        self.bias += K1 * y

        P00_temp = self.P00
        P01_temp = self.P01
        self.P00 -= K0 * P00_temp
        self.P01 -= K0 * P01_temp
        self.P10 -= K1 * P00_temp
        self.P11 -= K1 * P01_temp

        # Check for NaN or inf in output angle
        if np.isnan(self.angle) or np.isinf(self.angle):
            print(f"Warning: NaN or inf in KalmanAngle angle: resetting to 0")
            self.angle = 0.0
            self.bias = 0.0
            self.P00 = self.P01 = self.P10 = self.P11 = 0.0  # Reset covariance

        return self.angle

class KalmanFilter:
    def __init__(self, Q_angle=0.01, Q_bias=0.01, R_measure=0.1):
        """
        Initializes separate Kalman filters for roll and pitch.
        """
        self.roll_kalman = KalmanAngle(Q_angle, Q_bias, R_measure)
        self.pitch_kalman = KalmanAngle(Q_angle, Q_bias, R_measure)
        self.last_time = time.time()

    def update(self, ax, ay, az, gx, gy, gz):
        """
        ax, ay, az: accelerometer readings (m/s²)
        gx, gy, gz: gyroscope readings (rad/s ya deg/s) -> yahan deg/s assume karte hain
        return: (roll, pitch) in degrees
        """
        # Fixed time step
        dt = 0.01

        # Check for NaN or inf in inputs
        if any(np.isnan(v) or np.isinf(v) for v in [ax, ay, az, gx, gy, gz]):
            print(f"Warning: Invalid input in KalmanFilter.update: ax={ax}, ay={ay}, az={az}, gx={gx}, gy={gy}, gz={gz}")
            return self.roll_kalman.angle, self.pitch_kalman.angle

        # --- Step 1: roll/pitch from accelerometer ---
        # atan2 formula (gravity-based estimate)
        acc_roll = math.degrees(math.atan2(ay, az))
        denominator = math.sqrt(ay*ay + az*az)
        if abs(denominator) < 1e-6:
            print(f"Warning: Small denominator={denominator} in acc_pitch, skipping update")
            return self.roll_kalman.angle, self.pitch_kalman.angle
        acc_pitch = math.degrees(math.atan2(-ax, denominator))

        # Check for NaN or inf in computed angles
        if np.isnan(acc_roll) or np.isinf(acc_roll) or np.isnan(acc_pitch) or np.isinf(acc_pitch):
            print(f"Warning: Invalid angles: acc_roll={acc_roll}, acc_pitch={acc_pitch}")
            return self.roll_kalman.angle, self.pitch_kalman.angle

        # --- Step 2: Kalman update for roll and pitch ---
        roll = self.roll_kalman.update(gx, dt, acc_roll)
        pitch = self.pitch_kalman.update(gy, dt, acc_pitch)

        return roll, pitch