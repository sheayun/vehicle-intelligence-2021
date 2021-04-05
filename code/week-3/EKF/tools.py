import numpy as np
from math import sqrt, floor, ceil

def Jacobian(x):
    px, py, vx, vy = x
    if px == 0 and py == 0:
        print("Error: both px and py are zero while trying to")
        print("       calculate the Jacobian.")
        return np.zeros((3, 4))
    # Prepare calculation
    c1 = px * px + py * py
    c2 = sqrt(c1)
    c3 = c1 * c2
    # Fill in the matrix
    Hj = np.array([
        [px / c2, py / c2, 0.0, 0.0],
        [-py / c1, px / c1, 0.0, 0.0],
        [py * (vx * py - vy * px) / c3,
         px * (vy * px - vx * py) / c3,
         px / c2,
         py / c2]
    ])
    return Hj


def normalize(num, lower, upper, b=False):
    res = num
    if not b:
        if lower >= upper:
            raise ValueError("Invalid lower and upper limits: (%s, %s)" %
                             (lower, upper))

        res = num
        if num > upper or num == lower:
            num = lower + abs(num + upper) % (abs(lower) + abs(upper))
        if num < lower or num == upper:
            num = upper - abs(num - lower) % (abs(lower) + abs(upper))

        res = lower if res == upper else num
    else:
        total_length = abs(lower) + abs(upper)
        if num < -total_length:
            num += ceil(num / (-2 * total_length)) * 2 * total_length
        if num > total_length:
            num -= floor(num / (2 * total_length)) * 2 * total_length
        if num > upper:
            num = total_length - num
        if num < lower:
            num = -total_length - num

        res = num * 1.0  # Make all numbers float, to be consistent

    return res