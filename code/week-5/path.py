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

delta_name = ['^', '<', 'v', '>']

def search(grid, init, goal, cost):
    closed = np.zeros(grid.shape, dtype=np.int32)
    expand = np.full(grid.shape, -1, dtype=np.int32)
    path = np.full(grid.shape, ' ')
    queue = [(init, 0)]
    closed[init] = -1
    i = 1
    while len(queue) > 0:
        c, t = queue.pop(0)
        for x in delta:
            if 0 <= c[0] + x[0] < grid.shape[0] \
               and 0 <= c[1] + x[1] < grid.shape[1]:
                n = (c[0] + x[0], c[1] + x[1])
                if closed[n] != 0 or grid[n] != 0:
                    continue
                expand[n] = i
                i += 1
                if n == goal:
                    break
                queue.append((n, t + cost))
                closed[n] = -1

    pos = init
    while pos != goal:
        n = expand[pos]
        p = -1
        for i, d in enumerate(delta):
            zy, zx = pos[0] + d[0], pos[1] + d[1]
            if 0 <= zy < grid.shape[0] and 0 <= zx < grid.shape[1] \
               and expand[(zy, zx)] > n:
                n = expand[(zy, zx)]
                p = i
        path[pos] = delta_name[p]
        pos = pos[0] + delta[p][0], pos[1] + delta[p][1]
    path[pos] = '*'
    return path

path = search(grid, init, goal, cost)
print(path)
