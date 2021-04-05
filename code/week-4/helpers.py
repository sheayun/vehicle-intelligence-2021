import numpy as np

# Calculate Euclidean distance between two 2-D points.
def distance(p1, p2):
    dx = p1['x'] - p2['x']
    dy = p1['y'] - p2['y']
    d = np.sqrt(dx ** 2 + dy ** 2)
    return d
