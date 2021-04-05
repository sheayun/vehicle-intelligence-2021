import numpy as np
from data import measurements
from data import true_velocity
from kalman import KalmanFilter
from plot import plot_graphs

dt = 0.1

A = np.array([
    [1, dt],
    [0, 1]
])

C = np.array([
    [1, 0]
])

R = np.array([
    [1, 0],
    [0, 3]
])

Q = np.array([
    10
])

KF = KalmanFilter(A, C, R, Q)
filtered_positions = []
filtered_velocity = []
for m in measurements:
    x= KF.filter(m, dt)
    filtered_positions.append(x[0])
    filtered_velocity.append(x[1])

plot_graphs(measurements, filtered_positions,
            true_velocity, filtered_velocity)
