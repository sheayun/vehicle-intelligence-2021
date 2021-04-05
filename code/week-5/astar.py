import numpy as np

grid = np.array([
    [0, 1, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 1, 0, 1, 0, 0],
    [0, 0, 0, 1, 0, 0]
])
'''
grid = np.array([
    [0, 1, 0, 1, 0, 0],
    [0, 1, 0, 1, 0, 0],
    [0, 1, 0, 1, 0, 0],
    [0, 1, 0, 1, 0, 0],
    [0, 0, 0, 1, 1, 0]
])
'''
heuristic = np.array([
    [9, 8, 7, 6, 5, 4],
    [8, 7, 6, 5, 4, 3],
    [7, 6, 5, 4, 3, 2],
    [6, 5, 4, 3, 2, 1],
    [5, 4, 3, 2, 1, 0]
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

def search(grid, init, goal, cost):
    closed = np.zeros(grid.shape, dtype=np.int32)
    expand = np.full(grid.shape, -1, dtype=np.int32)
    path = np.full(grid.shape, ' ')
    closed[init] = -1

    g = 0
    f = g + heuristic[init]
    open_list = [(f, g, init)]

    found = False
    count = 0
    while not found:
        if len(open_list) == 0:
            break
        else:
            open_list.sort(reverse=True)
            n = open_list.pop()
            f, g = n[0], n[1]
            y, x = n[2]
            expand[(y, x)] = count
            count += 1

            if (y, x) == goal:
                found = True
            else:
                for d in delta:
                    y2, x2 = y + d[0], x + d[1]
                    if 0 <= y2 < grid.shape[0] and 0 <= x2 < grid.shape[1] \
                       and closed[(y2, x2)] == 0 and grid[(y2, x2)] == 0:
                        g2 = g + cost
                        f2 = g2 + heuristic[(y2, x2)]
                        open_list.append((f2, g2, (y2, x2)))
                        closed[(y2, x2)] = -1
    print(expand)
    if found:
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
    else:
        path = 'fail'

    return path

path = search(grid, init, goal, cost)
print(path)
