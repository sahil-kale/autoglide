import numpy as np

class WorldFrameCoordinate:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def distance_to(self, other):
        assert isinstance(other, WorldFrameCoordinate), "Other must be a WorldFrameCoordinate"
        return np.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)