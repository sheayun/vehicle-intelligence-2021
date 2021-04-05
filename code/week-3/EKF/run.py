from sensor_fusion import EKF
from plot import plot_2d

def testEKF(inputfilename):
    try:
        f = open(inputfilename, 'r')
    except:
        print("Failed to open file %s" % inputfilename)
        return
    lines = f.readlines()
    f.close()

    # Instantiate an extended Kalman filter
    fusion_EKF = EKF()
    gt_values = []
    estimations = []
    for i, line in enumerate(lines):
        words = line.split('\t')
        sensor_type = words[0]
        measurement = {}
        measurement['sensor_type'] = sensor_type
        if sensor_type == 'L':
            measurement['x'] = float(words[1])
            measurement['y'] = float(words[2])
            pos = 3
        elif sensor_type == 'R':
            measurement['rho'] = float(words[1])
            measurement['phi'] = float(words[2])
            measurement['rho_dot'] = float(words[3])
            pos = 4
        measurement['timestamp'] = int(words[pos])
        # Accumulate ground truth values
        gt_values.append({
            'px': float(words[pos + 1]),
            'py': float(words[pos + 2]),
            'vx': float(words[pos + 3]),
            'vy': float(words[pos + 4]),
        })
        # Process the measurement
        fusion_EKF.process_measurement(measurement)
        estimations.append({
            'px': fusion_EKF.ekf.x[0],
            'py': fusion_EKF.ekf.x[1],
            'vx': fusion_EKF.ekf.x[2],
            'vy': fusion_EKF.ekf.x[3],
        })
    # Visualize the result
    plot_2d(estimations, gt_values)

if __name__ == '__main__':
    testEKF('data.txt')
