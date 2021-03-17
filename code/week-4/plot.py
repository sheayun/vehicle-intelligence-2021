import matplotlib.pyplot as plt
import matplotlib.animation as animation

class GraphAnimator:
    data = []
    particle_graph = None
    landmark_graph = None
    pos_graph = None

    def __init__(self, particle_graph, landmark_graph, pos_graph, data):
        self.particle_graph = particle_graph
        self.landmark_graph = landmark_graph
        self.pos_graph = pos_graph
        self.data = data

    def animate(self, frameno):
        if frameno > 0 and len(self.data) > 0:
            d = self.data.pop(0)
            self.pos_graph.set_offsets([d['position']])
            self.particle_graph.set_offsets(d['particles'])
            if d['landmarks']:
                self.landmark_graph.set_offsets(d['landmarks'])
        return (self.particle_graph, self.landmark_graph, self.pos_graph)

def plot_2D(graph):
    # Now we generate an animated plot with the saved data.
    fig, ax = plt.subplots(figsize=(14.0, 6.4), num='Particle Filter')
    plt.xlim(-50, 300)
    plt.ylim(-120, 40)
    particle_scatter = ax.scatter([], [], s=20, c='red')
    landmark_scatter = ax.scatter([], [], s=10, c='black')
    pos_scatter = ax.scatter([], [], s=10, c='blue')
    graph_animator = GraphAnimator(
        particle_scatter, landmark_scatter, pos_scatter, graph
    )
    _ = animation.FuncAnimation(
        fig, graph_animator.animate, blit=True, interval=50, repeat=False,
        frames=len(graph)
    )
    plt.show()
