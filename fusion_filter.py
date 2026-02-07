# fusion_filter.py
import math
import time

class ComplementaryFilter:
    def __init__(self, alpha=0.9):
        """
        alpha ~0.98 ka matlab:
        - 98% trust gyro pe (short-term smoothness)
        - 2% trust accelerometer pe (long-term correction)
        """
        self.alpha = alpha
        self.roll = 0.0
        self.pitch = 0.0
        self.last_time = time.time()

    def update(self, ax, ay, az, gx, gy, gz):
        """
        ax, ay, az: accelerometer readings (m/s²)
        gx, gy, gz: gyroscope readings (rad/s ya deg/s) -> yahan deg/s assume karte hain
        return: (roll, pitch) in degrees
        """

        # timestep
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # --- Step 1: roll/pitch from accelerometer ---
        # atan2 formula (gravity-based estimate)
        acc_roll = math.degrees(math.atan2(ay, az))
        acc_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))

        # --- Step 2: integrate gyro for roll/pitch change ---
        # gyro values usually in deg/s → multiply with dt
        gyro_roll = self.roll + gx * dt
        gyro_pitch = self.pitch + gy * dt

        # --- Step 3: complementary fusion ---
        self.roll = self.alpha * gyro_roll + (1 - self.alpha) * acc_roll
        self.pitch = self.alpha * gyro_pitch + (1 - self.alpha) * acc_pitch

        return self.roll, self.pitch
