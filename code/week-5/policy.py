import numpy as np

grid = np.array([
    [0, 0, 0, 0, 0, 0],
    [0, 0, 1, 1, 1, 1],
    [0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 0]
])

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

def policy(grid, goal, cost):
    value = np.full(grid.shape, 99, dtype=np.int32)
    policy = np.full(grid.shape, ' ')
    change = True

    while change:
        change = False
        for x in range(grid.shape[1]):
            for y in range(grid.shape[0]):
                if (y, x) == goal and value[(y, x)] > 0:
                        value[(y, x)] = 0
                        policy[(y, x)] = '*'
                        change = True
                elif grid[(y, x)] == 0:
                    for d, a in zip(delta, delta_name):
                        y2, x2 = y + d[0], x + d[1]
                        if 0 <= y2 < grid.shape[0] \
                            and 0 <= x2 < grid.shape[1] \
                            and grid[(y2, x2)] == 0:
                            v2 = value[(y2, x2)] + cost
                            if v2 < value[(y, x)]:
                                change = True
                                value[(y, x)] = v2
                                policy[(y, x)] = a
    print(value)
    return policy

print(policy(grid, goal, cost))
