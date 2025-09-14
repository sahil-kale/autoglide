import numpy as np


class Vector2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, other):
        if isinstance(other, Vector2D):
            return Vector2D(self.x + other.x, self.y + other.y)
        else:
            raise ValueError("Can only add Vector2D to Vector2D")

    def __sub__(self, other):
        if isinstance(other, Vector2D):
            return Vector2D(self.x - other.x, self.y - other.y)
        else:
            raise ValueError("Can only subtract Vector2D from Vector2D")

    def __mul__(self, scalar):
        if isinstance(scalar, (int, float)):
            return Vector2D(self.x * scalar, self.y * scalar)
        else:
            raise ValueError("Can only multiply Vector2D by a scalar")

    def __truediv__(self, scalar):
        if isinstance(scalar, (int, float)):
            if scalar != 0:
                return Vector2D(self.x / scalar, self.y / scalar)
            else:
                raise ValueError("Division by zero")
        else:
            raise ValueError("Can only divide Vector2D by a scalar")

    def norm(self):
        return np.sqrt(self.x**2 + self.y**2)

    def normalize(self):
        n = self.norm()
        if n > 0:
            return self / n
        else:
            return Vector2D(0.0, 0.0)

    def dot(self, other):
        if isinstance(other, Vector2D):
            return self.x * other.x + self.y * other.y
        else:
            raise ValueError("Can only compute dot product with another Vector2D")

    def to_array(self):
        return np.array([self.x, self.y])

    def angle_between(self, other):
        if isinstance(other, Vector2D):
            dot_prod = self.dot(other)
            norms = self.norm() * other.norm()
            if norms == 0:
                return 0.0
            cos_angle = np.clip(dot_prod / norms, -1.0, 1.0)
            return np.arccos(cos_angle)
        else:
            raise ValueError("Can only compute angle with another Vector2D")

    def cross_product(self, other):
        if isinstance(other, Vector2D):
            return self.x * other.y - self.y * other.x
        else:
            raise ValueError("Can only compute cross product with another Vector2D")

    def signed_angle_between(self, other):
        """
        Returns the signed angle in radians that rotates this vector (self) to align with the other vector.
        """
        if isinstance(other, Vector2D):
            angle = self.angle_between(other)
            cross = self.cross_product(other)
            if cross < 0:
                return -angle
            else:
                return angle
        else:
            raise ValueError("Can only compute signed angle with another Vector2D")

    @staticmethod
    def from_array(arr):
        if len(arr) == 2:
            return Vector2D(arr[0], arr[1])
        else:
            raise ValueError("Array must have exactly two elements")
