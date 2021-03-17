from particle_filter import ParticleFilter
from plot import plot_2D

# Time elapsed between measurements (sec)
delta_t = 0.1
# Sensor range (m)
sensor_range = 50
# GNSS measurement uncertainty [x (m), y (m), theta (rad)]
pos_std = [0.3, 0.3, 0.01]
# Landmark measurement uncertainty [x (m), y (m)]
landmark_std = [0.3, 0.3]

# Read in the map data comprising
#   (landmark x, landmark y, id).
def read_map_from_file(filename):
    f = open(filename, 'r')
    lines = f.readlines()
    f.close()
    landmarks = {}
    for line in lines:
        tokens = line.split()
        landmarks[int(tokens[2])] = {
            'x': eval(tokens[0]),
            'y': eval(tokens[1]),
        }
    return landmarks

# Read in the measurement (lidar assumed) data line by line and
#   prepare them to be fed to the filter.
def read_measurements_from_file(filename):
    f = open(filename, 'r')
    lines = f.readlines()
    f.close()
    i = 0
    while i < len(lines):
        x, y, theta, velocity, yawrate = map(float, lines[i].split())
        measurement_x = list(map(float, lines[i + 1].split()))
        measurement_y = list(map(float, lines[i + 2].split()))
        measurement = {
            'gnss_x': x,
            'gnss_y': y,
            'gnss_theta': theta,
            'previous_velocity': velocity,
            'previous_yawrate': yawrate,
            'measurement_x': measurement_x,
            'measurement_y': measurement_y,
        }
        i += 3
        yield measurement

# Main driver code
if __name__ == '__main__':
    Map = read_map_from_file('map_data.txt')
    measurements = read_measurements_from_file('measurements.txt')
    # Number of particles in the set is an important parameter.
    pf = ParticleFilter(30)
    # Fill in the graph data as defined below so that
    #   a 2-D plot can be drawn.
    graph = []
    count = 0
    for m in measurements:
        count += 1
        if not pf.initialized:
            # Initialize the particle set using GNSS measurement.
            pf.initialize(m['gnss_x'], m['gnss_y'], m['gnss_theta'], *pos_std)
        else:
            # Prediction step
            pf.predict(delta_t, m['previous_velocity'],
                       m['previous_yawrate'], *pos_std)
        # Feed the particle with observations to update weights.
        observations = [{'x': x, 'y': y} for (x, y) in \
                        zip (m['measurement_x'], m['measurement_y'])]
        pf.update_weights(sensor_range, *landmark_std, observations, Map)
        # Resample particles to capture posterior belief distribution.
        pf.resample()
        # Select the best (highest weight) particle;
        #   we will assume that this represents the vehicle's position.
        particle = pf.get_best_particle()
        # Print for debugging purposes.
        print("[%d] %f, %f" % (count, particle['x'], particle['y']))
        # Collect data for post-mortem plotting.
        graph.append({
            'position': (particle['x'], particle['y']),
            'particles': [(p['x'], p['y']) for p in pf.particles],
            'landmarks': [(Map[l]['x'], Map[l]['y']) \
                          for l in particle['assoc']],
        })
    # Go plot the results.
    plot_2D(graph)
