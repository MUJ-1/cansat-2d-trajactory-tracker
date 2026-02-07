from fusion_filter import ComplementaryFilter
from serial_reader import SerialReader
import numpy as np
import queue
from collections import deque
import sys
from serial_port_button import select_com_port
import cv2
import time
import math

out = cv2.VideoWriter('2dTrajactory.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 20, (1280, 720))

# ------------------------
# Initialization
# ------------------------
white_img = np.ones((720, 1280, 3), dtype=np.uint8) * 255
x_init, y_init = 1280 // 2, 720 // 2
raw_data = []
biases = [0, 0, 0, 0, 0, 0]

# ------------------------
# Calibration
# ------------------------
def callibrate_data(data):
    global biases
    biases[0] = np.mean([d['ax'] for d in data])
    biases[1] = np.mean([d['ay'] for d in data])
    biases[2] = np.mean([d['az'] for d in data]) - 9.87  # gravity correction

    biases[3] = np.mean([d['gx'] for d in data])
    biases[4] = np.mean([d['gy'] for d in data])
    biases[5] = np.mean([d['gz'] for d in data])

    data.clear()
    print("Calibration Complete")

# ------------------------
# Filter & parameters
# ------------------------
filter = ComplementaryFilter(alpha=0.98)
previous_time = 0
MAX_POINTS = 200
BUFFER_SIZE = 3

# ------------------------
# Position, velocity, buffers
# ------------------------
x_pos, y_pos = [0], [0]
vx, vy = 0, 0
ax_buffer = deque(maxlen=BUFFER_SIZE)
ay_buffer = deque(maxlen=BUFFER_SIZE)

# ------------------------
# Thread-safe queue
# ------------------------
data_queue = queue.Queue()

# Gravity low-pass filter
g_est = np.zeros(3)

# High-pass filter
hp_alpha = 0.9
prev_a = np.zeros(3)
hp_a = np.zeros(3)

# ------------------------
# Serial callback
# ------------------------
def handle_packet(data):
    global i
    if i < 300:
        raw_data.append(data)
        i += 1
        if i == 300:
            callibrate_data(raw_data)
    else:
        data_queue.put(data)

# ------------------------
# Update function
# ------------------------
def update():
    global vx, vy, white_img, x_init, y_init, prev_a, hp_a, g_est, previous_time

    while not data_queue.empty():
        black_img_data = np.zeros((720, 1280, 3), dtype=np.uint8)
        data = data_queue.get()

        # ------------------------
        # Time delta
        # ------------------------
        current_time = data['HardwareTimestamp'] / 1000
        dt = 0.01 if previous_time == 0 else current_time - previous_time
        previous_time = current_time

        # ------------------------
        # Bias correction
        # ------------------------
        ax_raw = data['ax'] - biases[0]
        ay_raw = data['ay'] - biases[1]
        az_raw = data['az'] - biases[2]

        gx = data['gx'] - biases[3]
        gy = data['gy'] - biases[4]
        gz = data['gz'] - biases[5]

        # ------------------------
        # Orientation
        # ------------------------
        roll, pitch = filter.update(ax_raw, ay_raw, az_raw, gx, gy, gz)
        roll_rad, pitch_rad = np.deg2rad([roll, pitch])

        # ------------------------
        # Gravity estimation using low-pass
        # ------------------------
        tau = 0.5   # recommended; try 0.5 or 1.0 etc.
        alpha = math.exp(-dt / tau)
        g_est[0] = alpha * g_est[0] + (1-alpha) * ax_raw
        g_est[1] = alpha * g_est[1] + (1-alpha) * ay_raw
        g_est[2] = alpha * g_est[2] + (1-alpha) * az_raw

        # Linear acceleration (gravity removed)
        ax, ay, az = ax_raw - g_est[0], ay_raw - g_est[1], az_raw - g_est[2]

        # ------------------------
        # High-pass filter for noise removal
        # ------------------------
        hp_a[0] = hp_alpha * (hp_a[0] + ax - prev_a[0])
        hp_a[1] = hp_alpha * (hp_a[1] + ay - prev_a[1])
        hp_a[2] = hp_alpha * (hp_a[2] + az - prev_a[2])
        prev_a[:] = [ax, ay, az]

        # Threshold small noise
        hp_a[0] = 0 if abs(hp_a[0]) < 0.03 else hp_a[0]
        hp_a[1] = 0 if abs(hp_a[1]) < 0.03 else hp_a[1]

        # ------------------------
        # Buffers for smoothing
        # ------------------------
        ax_buffer.append(hp_a[0])
        ay_buffer.append(hp_a[1])
        ax_smooth = np.mean(ax_buffer) if len(ax_buffer) == BUFFER_SIZE else hp_a[0]
        ay_smooth = np.mean(ay_buffer) if len(ay_buffer) == BUFFER_SIZE else hp_a[1]

        # ------------------------
        # Integration: velocity -> position
        # ------------------------
        vx += ax_smooth * dt
        vy += ay_smooth * dt

        x_current = vx * dt
        y_current = vy * dt

        # x_pos.append(x_current if abs(x_current - x_pos[-1]) > 0.01 else x_pos[-1])
        # y_pos.append(y_pos[-1] + vy*dt if abs(y_current - y_pos[-1]) > 0.01 else y_pos[-1])

        x_pos.append(x_current if abs(x_current - x_pos[-1]) > 0.001 else 0)
        y_pos.append(y_current if abs(y_current - y_pos[-1]) > 0.001 else 0)

        # ------------------------
        # OpenCV plotting
        # ------------------------
        x_f, y_f = int(x_init + x_pos[-1]*10), int(y_init - y_pos[-1]*10)
        cv2.line(white_img, (x_init, y_init), (x_f, y_f), (255,0,0), 1)
        cv2.circle(white_img, (x_f, y_f), 2, (0,0,255), -1)
        x_init, y_init = x_f, y_f

        # Display text info
        text_data = [
            f"Raw: ax={data['ax']:.3f}, ay={data['ay']:.3f}, az={data['az']:.3f}",
            f"Calibrated: ax={ax_raw:.3f}, ay={ay_raw:.3f}, az={az_raw:.3f}",
            f"Gyro: gx={data['gx']:.3f}, gy={data['gy']:.3f}, gz={data['gz']:.3f}",
            f"Accel World: ax={ax_smooth:.3f}, ay={ay_smooth:.3f}",
            f"Angles: pitch={pitch:.2f}, roll={roll:.2f}",
            f"Position: x={x_pos[-1]:.3f}, y={y_pos[-1]:.3f}"
        ]
        colors = [(0,30,255),(255,23,140),(0,255,70),(0,0,255),(0,255,12),(255,255,0)]

        for i, text in enumerate(text_data):
            cv2.putText(black_img_data, text, (50,30 + i*25), cv2.FONT_HERSHEY_PLAIN, 1, colors[i], 2)

        img = cv2.bitwise_and(white_img, cv2.bitwise_not(black_img_data))
        cv2.imshow('Plot', img)
        out.write(img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            out.release()
            cv2.destroyAllWindows()
            exit()

        # Limit stored positions
        if len(x_pos) > MAX_POINTS:
            x_pos.pop(0)
            y_pos.pop(0)


# ------------------------
# Main
# ------------------------
if __name__ == "__main__":
    Serial_port = select_com_port()
    i = 0
    reader = SerialReader(port=Serial_port, baudrate=115200, callback=handle_packet)
    reader.start()
    print("Listening for packets... Press Ctrl+C to stop.")

    try:
        while True:
            update()
    except KeyboardInterrupt:
        print("\nStopping...")
        reader.stop()
    