import numpy as np
import matplotlib.pyplot as plt

plt.rcParams["figure.figsize"] = (12, 5)

def plot_graphs(measurements, filtered_positions,
                true_velocity, filtered_velocity):
    dt = 0.1
    N = len(measurements)
    t = np.arange(0, N * dt, dt)
    plt.cla()
    pos_plot = plt.subplot(121)
    vel_plot = plt.subplot(122)
    pos_plot.scatter(t, measurements,
                     s=5, c="blue", label="Measurements")
    pos_plot.plot(t, filtered_positions, c="red",
                  label="Kalman Filter")
    pos_plot.set_xlabel('Time [sec]')
    pos_plot.set_ylabel('Position [m]')
    pos_plot.legend(loc='upper left')
    #########################################################################
    vel_plot.scatter(t, true_velocity, s=5, c="cyan",
                     label='True speed')
    vel_plot.plot(t, filtered_velocity, c="magenta",
                  label='Kalman Filter')
    vel_plot.set_xlabel('Time [sec]')
    vel_plot.set_ylabel('Speed [m/s]')
    vel_plot.legend(loc='lower right')

    plt.gcf().canvas.set_window_title('Kalman Filter Example')

    plt.show()
