import time

class TimestampProcessor:
    def __init__(self):
        self.python_start_ms = time.monotonic_ns() // 1_000_000  # Python relative ms at start
        self.previous_hardware_ms = 0  # Store previous hardware timestamp
        self.min_diff = float('inf')  # Track min difference for offset estimation

    def process_timestamp(self, hardware_ms):
        """
        Process hardware timestamp and return estimated lag.
        
        Args:
            hardware_ms (float): Hardware timestamp in milliseconds (from CanSat's millis()).
        
        Returns:
            tuple: (dt, estimated_lag_ms)
                - dt (float): Time delta in seconds since last hardware timestamp.
                - estimated_lag_ms (float): Estimated lag in milliseconds (Python receive time - hardware time, offset-corrected).
        """
        # Current Python relative timestamp in ms
        python_ms = (time.monotonic_ns() // 1_000_000) - self.python_start_ms

        # Compute dt from hardware timestamps (convert to seconds)
        dt = 0.01 if self.previous_hardware_ms == 0 else (hardware_ms - self.previous_hardware_ms) / 1000.0
        self.previous_hardware_ms = hardware_ms

        # Compute lag (raw difference and estimated true lag)
        raw_diff_ms = python_ms - hardware_ms
        self.min_diff = min(self.min_diff, raw_diff_ms)  # Update offset estimate
        estimated_lag_ms = raw_diff_ms - self.min_diff

        return dt, estimated_lag_ms