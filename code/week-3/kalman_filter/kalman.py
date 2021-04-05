import numpy as np

class KalmanFilter:
    def __init__(self, A, C, R, Q):
        # Initialize system model matrices
        self.A = A
        self.C = C
        self.R = R
        self.Q = Q
        # Initialize state variable and its covariance (with zeros)
        self.x = np.zeros(self.A.shape[1])
        self.P = np.zeros(self.R.shape)

    def filter(self, z, dt):
        # One step of Kalman filter operation consists of:
        # (1) prediction
        # (2) estimation(measurement update)
        x_p, P_p = self.predict(dt)
        self.update(x_p, P_p, z)
        return self.x

    def predict(self, dt):
        # Set dt to the appropriate position in maxtrix A
        self.A[0][1] = dt
        # Predict state variable and its covariance
        x_p = np.dot(self.A, self.x)
        P_p = np.dot(np.dot(self.A, self.P), self.A.T) + self.R
        return x_p, P_p

    def update(self, x_p, P_p, z):
        # Compute Kalman gain
        PpCT = np.dot(P_p, self.C.T)
        invterm = np.linalg.inv(np.dot(self.C, PpCT) + self.Q)
        K = np.dot(PpCT, invterm)
        # Update state with current measurement
        uterm = np.dot(K, z - np.dot(self.C, x_p))
        self.x = x_p + uterm
        I = np.eye(self.P.shape[0], self.P.shape[1])
        self.P = np.dot(I - np.dot(K, self.C), P_p)
