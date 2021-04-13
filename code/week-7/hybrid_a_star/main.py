import numpy as np
from hybrid_astar import HybridAStar
from plot import GridPlot

# The grid is of shape 16 x 16.
# Note that we use (x, y) instead of (y, x) coordinates,
# since that makes application of trigonometric functions easier.
grid = np.array([
    [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0],
    [0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
    [0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0],
    [0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0],
    [0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0],
    [0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0],
    [0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1],
    [0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1],
    [0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1],
    [0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1],
    [0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
], dtype=np.int)

# The coordinates are represented by (x, y, theta).
# Note that this order does not agree with numpy array indices.
start = (0.0, 0.0, 0.0)
# Note that the goal is specified by (x, y)
goal = (grid.shape[0] - 1, grid.shape[1] - 1)

if __name__ == '__main__':
    has = HybridAStar((HybridAStar.NUM_THETA_CELLS,) + grid.shape)
    success, expanded = has.search(grid, start, goal)
    if success:
        print("Found path to goal in %d expansions" % expanded)
    else:
        print("No valid path found after %d expansions" % expanded)
    path = has.reconstruct_path(start, goal)

    '''
    for i, p in enumerate(path, start=1):
        print("##### step %d #####" % i)
        print("x %f" % p['x'])
        print("y %f" % p['y'])
        print("theta %f" % p['t'])
    '''

    plot = GridPlot(grid.shape)
    plot.plot_grid(grid)
    if success:
        plot.plot_path(path, start, goal)
    plot.show()
