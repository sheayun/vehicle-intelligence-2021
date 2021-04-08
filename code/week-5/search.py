import numpy as np

grid = np.array([
    [0, 0, 1, 0, 0, 0],
    [0, 0, 1, 0, 0, 0],
    [0, 0, 0, 0, 1, 0],
    [0, 0, 1, 1, 1, 0],
    [0, 0, 0, 0, 1, 0]
])
'''
grid = np.array([
    [0, 0, 1, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 1, 0, 1, 0],
    [0, 0, 1, 0, 1, 0],
    [0, 0, 1, 0, 1, 0]
])
'''

init = (0, 0)
goal = (grid.shape[0] - 1, grid.shape[1] - 1)
cost = 1
delta = [
    (-1,  0),   # Up
    ( 0, -1),   # Left
    ( 1,  0),   # Down
    ( 0,  1),   # Right
]

def search(grid, init, goal, cost):
    closed = np.zeros(grid.shape, dtype=np.int32)
    path = None
    queue = [(init, 0)]
    while len(queue) > 0:
        c, t = queue.pop(0)
        for x in delta:
            if 0 <= c[0] + x[0] < grid.shape[0] \
               and 0 <= c[1] + x[1] < grid.shape[1]:
                n = (c[0] + x[0], c[1] + x[1])
                if closed[n] != 0 or grid[n] != 0:
                    continue
                if n == goal:
                    path = [t + cost, n]
                    break
                queue.append((n, t + cost))
                closed[n] = -1
    if not path:
        path = 'fail'

    return path

path = search(grid, init, goal, cost)
print(path)
