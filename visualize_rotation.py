import matplotlib.pyplot as plt
from math import pi

class RotationVisualization:
        
    def map_pos(self, rot):
        r, p, y = rot
        return (r + pi, p + pi, y + pi)

    def _init_figure(self):
        plt.ion()
        fig, ax = plt.subplots()
        names = ['roll', 'pitch', 'yaw']
        angles = [1.0, 1.0, 1.0]

        self._bars = ax.bar(names, angles)
        ax.set_ylim(0, 2.5 * pi)
        plt.draw()

        return fig, ax
    
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self._fig, self._axes = self._init_figure()

    def update_rotation(self, rotations):
        pos_rot = self.map_pos(rotations)
        for i, bar in enumerate(self._bars):
            bar.set_height(pos_rot[i])
        self.redraw_figure()

    def redraw_figure(self):
        self._fig.canvas.flush_events()
        plt.show(block=False)