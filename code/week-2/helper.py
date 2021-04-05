from math import sqrt
from math import pi
from math import exp

# Calculate normal (Gaussian) distribution probability.
# norm_pdf(x, m, s) returns the probability of a random variable
# havnig the value of x, assuming that the variable follows
# the Gaussian probability distribution with mean of m and
# standard deviation of s.
def norm_pdf(x, m, s):
    one_over_sqrt_2pi = 1 / sqrt(2 * pi)
    return (one_over_sqrt_2pi / s) * exp(-0.5 * ((x - m) / s) ** 2)

# A helper class to visualize pdf evolution.
class GraphAnimator:
    data = []
    graph = None

    def __init__(self, graph, data):
        self.graph = graph
        self.data = data

    def animate(self, frameno):
        if frameno == 0:
            return self.graph
        if len(self.data) > 0:
            d = self.data.pop(0)
            for b, h in zip(self.graph, d):
                b.set_height(h)
        return self.graph
