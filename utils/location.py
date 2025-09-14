import numpy as np
from utils.vector import Vector2D


class WorldFrameCoordinate(Vector2D):
    def __init__(self, x, y):
        super().__init__(x, y)

    def distance_to(self, other):
        assert isinstance(
            other, WorldFrameCoordinate
        ), "Other must be a WorldFrameCoordinate"
        return (self - other).norm()
