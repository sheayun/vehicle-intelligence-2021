import numpy as np
from math import sqrt
from math import atan2
from tools import Jacobian

class KalmanFilter:
    def __init__(self, x_in, P_in, F_in, H_in, R_in, Q_in):
        self.x = x_in
        self.P = P_in
        self.F = F_in
        self.H = H_in
        self.R = R_in
        self.Q = Q_in

    def predict(self):
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, z):
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        # Calculate new estimates
        self.x = self.x + np.dot(K, z - np.dot(self.H, self.x))
        self.P = self.P - np.dot(np.dot(K, self.H), self.P)

    def update_ekf(self, z):
        # TODO: Implement EKF update for radar measurements
        # 1. Compute Jacobian Matrix H_j
        # 2. Calculate S = H_j * P' * H_j^T + R
        # 3. Calculate Kalman gain K = H_j * P' * Hj^T + R
        # 4. Estimate y = z - h(x')
        # 5. Normalize phi so that it is between -PI and +PI
        # 6. Calculate new estimates
        #    x = x' + K * y
        #    P = (I - K * H_j) * P

        '''
        Hj = Jacobian(self.x)
        S = np.dot(np.dot(Hj, self.P), Hj.T) + self.R
        K = np.dot(np.dot(self.P, Hj.T), np.linalg.inv(S))
        # Estimate y = z - h(x')
        px = self.x[0]
        py = self.x[1]
        vx = self.x[2]
        vy = self.x[3]
        rho = sqrt(px * px + py * py)
        phi = atan2(py, px)
        rho_dot = (px * vx + py * vy) / rho
        hx = np.array([rho, phi, rho_dot])
        y = z - hx
        # Normalize phi so that it is between -PI and PI
        phi = y[1]
        sign = 1.0 if phi > 0 else -1.0
        mag = abs(phi)
        while mag > np.pi:
            mag -= np.pi
        y[1] = sign * mag
        # Calculate new estimates
        self.x = self.x + np.dot(K, y)
        self.P = self.P - np.dot(np.dot(K, Hj), self.P)
        '''
