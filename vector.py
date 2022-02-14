import numpy as np

class Vector2D:
    x: float
    y: float

    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.normalize()

    def normalize(self):
        v = np.array([self.x, self.y])
        norm = np.linalg.norm(v)
        if not norm == 0:
            self.x, self.y = v[0] / norm, v[1] / norm

    def get_xy_vector(self):
        return np.array([self.x, self.y])

    def get_np_vector(self):
        return np.array([self.y, self.x])

    def get_m(self):
        return self.y

    def get_n(self):
        return self.x
