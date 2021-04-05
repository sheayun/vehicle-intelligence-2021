import numpy as np
from kalman_filter import KalmanFilter

class EKF:
    def __init__(self):
        self.is_initialized = False
        self.previous_timestamp = 0
        # Initialize measurement covariance matrix - laser
        self.R_laser = np.array([
            [0.0225, 0.0],
            [0.0, 0.0225]
        ], dtype=np.float32)
        # Initialize measurement covariance matrix - radar
        self.R_radar = np.array([
            [0.09, 0.0, 0.0],
            [0.0, 0.0009, 0.0],
            [0.0, 0.0, 0.09]
        ], dtype=np.float32)
        # Initialize state variable
        x_init = np.zeros(4, dtype=np.float32)
        # Initialize state covariance matrix
        P_init = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float32)
        # System model - dt not yet taken into account
        F_init = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float32)
        # Transformation from state variable to measurement
        H_init = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float32)
        # Covariance matrix of process noise - dt not yet taken into account
        Q_init = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [1, 0, 1, 0],
            [0, 1, 0, 1],
        ], dtype=np.float32)
        # Initialize our Kalman filter
        self.ekf = KalmanFilter(
            x_init, P_init, F_init, H_init, self.R_laser, Q_init
        )

        # Set the process noise constants
        self.noise_ax = 9.0
        self.noise_ay = 9.0

    def process_measurement(self, m):
        if not self.is_initialized:
            # First measurement
            if m['sensor_type'] == 'R':
                # Convert radar data in polar to cartesian coordinates
                # (Note that rho_dot from the very first measurement is
                #  not used))
                px = m['rho'] * np.cos(m['phi'])
                py = m['rho'] * np.sin(m['phi'])
                self.ekf.x = np.array([px, py, 0.0, 0.0])
                self.previous_timestamp = m['timestamp']
            elif m['sensor_type'] == 'L':
                # Initialize state variable
                px, py = m['x'], m['y']
                self.ekf.x = np.array([px, py, 0, 0])
                self.previous_timestamp = m['timestamp']
            self.is_initialized = True
            return

        # Prediction

        # Update state transition matrix (time measured in seconds)
        dt = (m['timestamp'] - self.previous_timestamp) / 1000000.0
        self.previous_timestamp = m['timestamp']
        self.ekf.F[0][2] = dt
        self.ekf.F[1][3] = dt
        # Set the process noise covariance
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt
        self.ekf.Q = np.array([
            [dt4 * self.noise_ax / 4.0, 0.0, dt3 * self.noise_ax / 2.0, 0.0],
            [0.0, dt4 * self.noise_ay / 4.0, 0.0, dt3 * self.noise_ay / 2.0],
            [dt3 * self.noise_ax / 2.0, 0.0, dt2 * self.noise_ax, 0.0],
            [0.0, dt3 * self.noise_ay / 2.0, 0.0, dt2 * self.noise_ay]
        ], dtype=np.float32)
        self.ekf.predict()

        # Measurement update

        if m['sensor_type'] == 'R':
            # Radar updates
            z = np.array([
                m['rho'], m['phi'], m['rho_dot']
            ], dtype=np.float32)
            self.ekf.R = self.R_radar
            self.ekf.update_ekf(z)
        elif m['sensor_type'] == 'L':
            # Laser updates
            z = np.array([
                m['x'], m['y']
            ], dtype=np.float32)
            self.ekf.R = self.R_laser
            self.ekf.update(z)

        # Print the output
        # print("x = ")
        # print(self.ekf.x)
        # print("P = ")
        # print(self.ekf.P)
