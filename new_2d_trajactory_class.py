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
import gui
from data_logger import Logger
import subprocess
import platform
from kalman_filter import KalmanFilter
from callibrate_gyro_real_time import calibrate_gyro_realtime
from live_matlib_plot import update_plot
from python_timeStamp import TimestampProcessor


class TrajectoryProcessor:
    def __init__(self):
        # Video writer
        self.out = cv2.VideoWriter('2dTrajactory.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 20, (1280, 720))

        # Start time in ms
        self.start_time_ms =   time.time_ns()

        # Initialization
        self.white_img = np.ones((720, 1280, 3), dtype=np.uint8) * 255
        self.x_init, self.y_init = 1280 // 2, 720 // 2
        self.raw_data = []
        self.biases = [0, 0, 0, 0, 0, 0]

        # Scaling Factor in Acceleration
        self.ax_scale = 1
        self.ay_scale = 1
        self.az_scale = 1

        # Filter & parameters
        self.filter = ComplementaryFilter(alpha=0.98)
        self.previous_time = 0
        self.MAX_POINTS = 200
        self.BUFFER_SIZE = 5

        # Kalman filter
        self.kf = KalmanFilter()

        # Position, velocity, buffers
        self.x_pos, self.y_pos = [0], [0]
        self.x_new, self.y_new = 0, 0
        self.vx, self.vy = 0, 0
        self.x_current, self.y_current = 0, 0
        self.ax_buffer = deque(maxlen=self.BUFFER_SIZE)
        self.ay_buffer = deque(maxlen=self.BUFFER_SIZE)
        self.az_buffer = deque(maxlen=self.BUFFER_SIZE)

        # Thread-safe queue
        self.data_queue = queue.Queue()

        # Gravity low-pass filter
        self.g_est = np.zeros(3)

        # High-pass filter
        self.hp_alpha = 0.9
        self.prev_a = np.zeros(3)
        self.hp_a = np.zeros(3)

        # Gravity
        self.g = 9.87

        # Quadrant angles
        self.quadAngle = [0, 90, 180, 270 ,360]

        # Calibration counter
        self.i = 0 # Collect initial 300 packets for callibration
#########################################
#########################################
        # Debugging varibales
        # self.Logger = Logger('dt.csv')
        

    def display_debug_gui(self, s):
        display.update(s)
        display.root.update()
        # time.sleep(0.01)

########################################
    def calibrate_data(self, data):
        self.biases[0] = np.mean([d['ax'] for d in data])
        self.biases[1] = np.mean([d['ay'] for d in data])
        self.biases[2] = np.mean([d['az'] for d in data]) - 9.87  # gravity correction

        self.biases[3] = np.mean([d['gx'] for d in data])
        self.biases[4] = np.mean([d['gy'] for d in data])
        self.biases[5] = np.mean([d['gz'] for d in data])

        data.clear()
        print("Calibration Complete")
        # Removed display.run() here to avoid blocking early; GUI will update in loop

    def apply_bias_correction(self, data):
        ax_raw = data['ax'] - self.biases[0]
        ay_raw = data['ay'] - self.biases[1]
        az_raw = data['az'] - self.biases[2]

        gx = data['gx'] - self.biases[3]
        gy = data['gy'] - self.biases[4]
        gz = data['gz'] - self.biases[5]

        return ax_raw, ay_raw, az_raw, gx, gy, gz

    def get_orientation(self, ax_raw, ay_raw, az_raw, gx, gy, gz):
        roll, pitch = self.filter.update(ax_raw, ay_raw, az_raw, gx, gy, gz)
        return roll, pitch

    def estimate_gravity(self, dt, ax_raw, ay_raw, az_raw):
        tau = 0.5
        alpha = math.exp(-dt / tau)
        self.g_est[0] = alpha * self.g_est[0] + (1 - alpha) * ax_raw
        self.g_est[1] = alpha * self.g_est[1] + (1 - alpha) * ay_raw
        self.g_est[2] = alpha * self.g_est[2] + (1 - alpha) * az_raw
        return self.g_est.copy()

    def remove_gravity(self, ax_raw, ay_raw, az_raw):
        ax = ax_raw - self.g_est[0]
        ay = ay_raw - self.g_est[1]
        az = az_raw - self.g_est[2]
        return ax, ay, az

    def rotate_to_world(self, ax, ay, az, roll, pitch):
        roll_rad = np.deg2rad(roll)
        pitch_rad = np.deg2rad(pitch)
        cr = np.cos(roll_rad)
        sr = np.sin(roll_rad)
        cp = np.cos(pitch_rad)
        sp = np.sin(pitch_rad)
        # Rotation matrix R = Ry(pitch) @ Rx(roll)
        R = np.array([
            [cp, sp * sr, sp * cr],
            [0, cr, -sr],
            [-sp, -cp * sr, cp * cr]
        ])
        a_sensor = np.array([ax, ay, az])
        a_world = R @ a_sensor
        return a_world[0], a_world[1]

    def apply_highpass_filter(self, ax, ay, az):
        new_hp_a = np.zeros(3)
        new_hp_a[0] = self.hp_alpha * (self.hp_a[0] + ax - self.prev_a[0])
        new_hp_a[1] = self.hp_alpha * (self.hp_a[1] + ay - self.prev_a[1])
        new_hp_a[2] = self.hp_alpha * (self.hp_a[2] + az - self.prev_a[2])
        self.prev_a[:] = [ax, ay, az]
        self.hp_a[:] = new_hp_a

        hp_ax = 0 if abs(new_hp_a[0]) < 0.03 else new_hp_a[0]
        hp_ay = 0 if abs(new_hp_a[1]) < 0.03 else new_hp_a[1]
        hp_az = 0 if abs(new_hp_a[2]) < 0.03 else new_hp_a[2]
        return hp_ax, hp_ay, hp_az

    def smooth_acceleration(self, hp_ax, hp_ay, hp_az):
        self.ax_buffer.append(hp_ax)
        self.ay_buffer.append(hp_ay)
        self.az_buffer.append(hp_az)
        ax_smooth = np.mean(self.ax_buffer) if len(self.ax_buffer) == self.BUFFER_SIZE else hp_ax
        ay_smooth = np.mean(self.ay_buffer) if len(self.ay_buffer) == self.BUFFER_SIZE else hp_ay
        az_smooth = np.mean(self.az_buffer) if len(self.az_buffer) == self.BUFFER_SIZE else hp_az
        return ax_smooth, ay_smooth, az_smooth

    def integrate_position(self, dt, ax_smooth, ay_smooth):
        self.vx += ax_smooth * dt
        self.vy += ay_smooth * dt

        # Calculate the change in position 
        self.x_current = self.vx * dt
        self.y_current = self.vy * dt
        
        # Use cumulative positions with threshold on delta
        if abs(self.x_current) > 0.001:
            self.x_new= self.x_pos[-1] + self.x_current
        else:
            self.y_new = 0
        if abs(self.y_current) > 0.001:
            self.y_new = self.y_pos[-1] + self.y_current
        else:
            self.y_new = 0
        self.x_pos.append(self.x_new)
        self.y_pos.append(self.y_new)

    def limit_history(self):
        if len(self.x_pos) > self.MAX_POINTS:
            self.x_pos.pop(0)
            self.y_pos.pop(0)

    def plot_trajectory(self, x_delta, y_delta):
        x_f = int(self.x_init + x_delta * 10)
        y_f = int(self.y_init - y_delta * 10)
        cv2.line(self.white_img, (self.x_init, self.y_init), (x_f, y_f), (255, 0, 0), 1)
        cv2.circle(self.white_img, (x_f, y_f), 2, (0, 0, 255), -1)
        self.x_init, self.y_init = x_f, y_f

    def overlay_data(self, data, ax_raw, ay_raw, az_raw, ax_smooth, ay_smooth, pitch, roll):
        black_img_data = np.zeros((720, 1280, 3), dtype=np.uint8)
        text_data = [
            f"Raw: ax={data['ax']:.3f}, ay={data['ay']:.3f}, az={data['az']:.3f}",
            f"Calibrated: ax={ax_raw:.3f}, ay={ay_raw:.3f}, az={az_raw:.3f}",
            f"Gyro: gx={data['gx']:.3f}, gy={data['gy']:.3f}, gz={data['gz']:.3f}",
            f"Accel World: ax={ax_smooth:.3f}, ay={ay_smooth:.3f}",
            f"Angles: pitch={pitch:.2f}, roll={roll:.2f}",
            f"Position: x={self.x_pos[-1]:.3f}, y={self.y_pos[-1]:.3f}"
        ]
        colors = [(0, 30, 255), (255, 23, 140), (0, 255, 70), (0, 0, 255), (0, 255, 12), (255, 255, 0)]
        for i, text in enumerate(text_data):
            cv2.putText(black_img_data, text, (50, 30 + i * 25), cv2.FONT_HERSHEY_PLAIN, 1, colors[i], 2)
        img = cv2.bitwise_and(self.white_img, cv2.bitwise_not(black_img_data))
        return img

    def handle_packet(self, data):
        if self.i < 300:
            print(self.i)
            self.raw_data.append(data)
            self.i += 1
            if self.i == 300:
                self.calibrate_data(self.raw_data)
        else:
            self.data_queue.put(data)

    def update(self):
        while not self.data_queue.empty():
            data = self.data_queue.get()

            # # Time delta
            # current_time = data['HardwareTimestamp'] / 1000
            # dt = 0.01 if self.previous_time == 0 else current_time - self.previous_time
            # self.previous_time = current_time
            # self.Logger.write(dt)

            # Using fixed dt as 10ms
            dt = 0.01


            # Bias correction
            ax, ay, az, gx, gy, gz = self.apply_bias_correction(data)

            # Sclaing factor removed
            # ax = (data['ax']-ax)*self.ax_scale 
            # ay = (data['ay']-ay)*self.ay_scale
            # az = (data['az'])*self.az_scale
            ax = ax*self.ax_scale 
            ay = ay*self.ay_scale
            az = az*self.az_scale
            


            # Acceleration Debugging
            # s = f"ax_raw = {ax}, ay_raw = {ay}, az_raw = {az}, gx = {data['gx']}, gy = {data['gy']}, gz = {data['gz']}"

            # # Orientation
            # roll_c, pitch_c = self.get_orientation(ax, ay, az, gx, gy, gz) # Complementry filter

            roll_k, pitch_k = self.kf.update(ax, ay, az, gx, gy, gz)    # Kalman Filter

            # To avoid division by undefined/infinity while using sin or cos
            if roll_k in self.quadAngle:
                roll_k += 0.001
            if pitch_k in self.quadAngle:
                pitch_k += 0.001

            # Selected Kalman Filter for gravity component removal
            ax = ax + self.g*np.sin(np.deg2rad(pitch_k))
            ay = ay - (self.g*np.sin(np.deg2rad(roll_k))*np.cos(np.deg2rad(pitch_k)))
            az = az - (self.g*np.cos(np.deg2rad(roll_k))*np.cos(np.deg2rad(pitch_k)))
            


            # # Gravity estimation and removal
            g_est = self.estimate_gravity(dt, ax, ay, az)
            ax, ay, az = self.remove_gravity(ax, ay, az)


            # # High-pass filter
            hp_ax, hp_ay, hp_az = self.apply_highpass_filter(ax, ay, az)

            # s = f"ax = {ax}, ay = {ay}, hp_ax = {hp_ax}, hp_ay = {hp_ay}, hp_az = {hp_az}, roll = {roll_k}, pitch = {pitch_k}"
            # display.update(s)
            # display.root.update()
            # time.sleep(0.01)

            # # Smoothing
            ax_smooth, ay_smooth, az_smooth = self.smooth_acceleration(hp_ax, hp_ay, hp_az)
            #--------------------------#    
            # Callibrate Gyro Real Time#
            #--------------------------#    
            self.biases[3], self.biases[4], self.biases[5] = calibrate_gyro_realtime(ax_smooth, ay_smooth, az_smooth, 
                                                       data['gx'], data['gy'], data['gz'], 
                                                       [self.biases[3], self.biases[4], self.biases[5]])
            # # Conversion to world frame
            ax_world, ay_world = self.rotate_to_world(ax_smooth, ay_smooth, az_smooth, roll_k, pitch_k)
            ax_smooth, ay_smooth = ax_world, ay_world

            

            # # Position estimation (integration)
            self.integrate_position(dt, ax_smooth, ay_smooth)

            # # Limit history
            # self.limit_history()

            # # Compute deltas for plotting
            x_delta = self.x_pos[-1] - self.x_pos[-2] if len(self.x_pos) > 1 else 0
            # y_delta = self.y_pos[-1] - self.y_pos[-2] if len(self.y_pos) > 1 else 0

            # # OpenCV plotting
            # self.plot_trajectory(x_delta, y_delta)

            # # Overlay data
            # img = self.overlay_data(data, ax_raw, ay_raw, az_raw, ax_smooth, ay_smooth, pitch, roll)


            # Graph plotting live matplotlib
            update_plot(self.x_pos[-1], self.y_pos[-1])        
            # Pause to simulate real-time data and make the animation visible
            # time.sleep(0.01)
            python_timestamp =  (time.time_ns() - self.start_time_ms)/1e6
            lag_ms = (python_timestamp / 1e6) - (data['HardwareTimestamp'] / 1e3)

            s = f"Hardware_timeStamp = {data['HardwareTimestamp']}, python_timeStamp = {python_timestamp}, lag = {lag_ms} ,ax = {ax_smooth:.3f}, ay = {ay_smooth:.3f}, az = {az_smooth:.3f}, ptich = {pitch_k}, roll = {roll_k}, gx = {gx:.3f}, gy= {gy:.3f}, gz = {gz:.3f}, x_pos = {self.x_new:.3f}, y_pos = {self.y_new:.3f}"
            self.display_debug_gui(s)


            # cv2.imshow('Plot', img)
            # self.out.write(img)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     self.out.release()
            #     cv2.destroyAllWindows()
            #     sys.exit()

# ------------------------
# Main
# ------------------------
if __name__ == "__main__":
    display = gui.LivePrintGUI()
    processor = TrajectoryProcessor()
    Serial_port = select_com_port()
    reader = SerialReader(port=Serial_port, baudrate=115200, callback=processor.handle_packet)
    reader.start()
    print("Listening for packets... Press Ctrl+C to stop.")

    try:
        while True:
            processor.update()
    except KeyboardInterrupt:
        print("\nStopping...")
        reader.stop()
        # Keep GUI window open briefly after stop for final view
        display.root.update()
        display.run()  # Starts mainloop to handle window close