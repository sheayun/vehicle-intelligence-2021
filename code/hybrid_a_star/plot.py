import numpy as np
import cv2

class GridPlot:

    # Horizontal and vertical size (in pixels) for one grid cell.
    dx = 50
    dy = 50
    # Horizontal (left & right) and vertical (top & bottom) margins.
    mx = 30
    my = 30

    def __init__(self, dim):
        self.dim = dim
        img_dim = (
            dim[0] * self.dx + 2 * self.mx,
            dim[1] * self.dy + 2 * self.my,
            3   # 3-channel color image
        )
        self.img = np.full(img_dim, 255, dtype=np.uint8)

    def plot_grid(self, grid):
        # Draw grid lines.
        for i in range(self.dim[1] + 1):
            begin, end = self.conv(i, 0), self.conv(i, self.dim[0])
            cv2.line(self.img, begin, end, (255, 0, 0), 1)
        for i in range(self.dim[0] + 1):
            begin, end = self.conv(0, i), self.conv(self.dim[1], i)
            cv2.line(self.img, begin, end, (255, 0, 0), 1)
        # Fill in obstacles.
        for x in range(self.dim[0]):
            for y in range(self.dim[1]):
                if grid[x][y] == 1:
                    ul = self.conv(x, y)
                    ul = (ul[0] + 1, ul[1] + 1)
                    lr = self.conv(x + 1, y + 1)
                    lr = (lr[0] - 1, lr[1] - 1)
                    cv2.rectangle(self.img, ul, lr, (120, 150, 150), -1)

    def plot_path(self, path, start, goal):
        prev = self.conv(path[0]['x'], path[0]['y'])
        cv2.circle(self.img, prev, 5, (0, 0, 255), -1)
        for p in path[1:]:
            x, y = p['x'], p['y']
            curr = self.conv(x, y)
            cv2.circle(self.img, curr, 5, (0, 0, 255), -1)
            cv2.line(self.img, prev, curr, (0, 0, 255), 2)
            prev = curr

    def show(self):
        cv2.imshow('grid plot', self.img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def conv(self, x, y):
        px = int(y * self.dx + self.mx)
        py = int(x * self.dy + self.my)
        return (px, py)
