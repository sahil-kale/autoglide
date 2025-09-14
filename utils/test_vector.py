def test_cross_product():
    v1 = Vector2D(1, 0)
    v2 = Vector2D(0, 1)
    v3 = Vector2D(1, 1)
    assert v1.cross_product(v2) == 1
    assert v2.cross_product(v1) == -1
    assert v1.cross_product(v3) == 1
    assert v3.cross_product(v1) == -1
    assert v1.cross_product(v1) == 0


def test_signed_angle_between():
    v1 = Vector2D(1, 0)
    v2 = Vector2D(0, 1)
    v3 = Vector2D(-1, 0)
    v4 = Vector2D(0, -1)
    # v1 to v2: +pi/2
    assert np.isclose(v1.signed_angle_between(v2), np.pi / 2)
    # v2 to v1: -pi/2
    assert np.isclose(v2.signed_angle_between(v1), -np.pi / 2)
    # v1 to v3: pi
    assert np.isclose(v1.signed_angle_between(v3), np.pi)
    # v1 to v4: -pi/2
    assert np.isclose(v1.signed_angle_between(v4), -np.pi / 2)
    # v1 to v1: 0
    assert np.isclose(v1.signed_angle_between(v1), 0.0)


import pytest
import numpy as np
from utils.vector import Vector2D


def test_addition():
    v1 = Vector2D(1, 2)
    v2 = Vector2D(3, 4)
    v3 = v1 + v2
    assert v3.x == 4 and v3.y == 6


def test_subtraction():
    v1 = Vector2D(5, 7)
    v2 = Vector2D(2, 3)
    v3 = v1 - v2
    assert v3.x == 3 and v3.y == 4


def test_scalar_multiplication():
    v = Vector2D(2, -3)
    v2 = v * 4
    assert v2.x == 8 and v2.y == -12


def test_scalar_division():
    v = Vector2D(8, 4)
    v2 = v / 2
    assert v2.x == 4 and v2.y == 2
    with pytest.raises(ValueError):
        v / 0


def test_norm_and_normalize():
    v = Vector2D(3, 4)
    assert np.isclose(v.norm(), 5)
    vn = v.normalize()
    assert np.isclose(vn.norm(), 1)
    v0 = Vector2D(0, 0)
    vn0 = v0.normalize()
    assert vn0.x == 0 and vn0.y == 0


def test_dot_product():
    v1 = Vector2D(1, 2)
    v2 = Vector2D(3, 4)
    assert v1.dot(v2) == 11


def test_to_array_and_from_array():
    v = Vector2D(5, -6)
    arr = v.to_array()
    assert np.array_equal(arr, np.array([5, -6]))
    v2 = Vector2D.from_array([7, 8])
    assert v2.x == 7 and v2.y == 8
    with pytest.raises(ValueError):
        Vector2D.from_array([1])


def test_angle_between_vectors():
    def angle(v1, v2):
        dot = v1.dot(v2)
        n1 = v1.norm()
        n2 = v2.norm()
        if n1 == 0 or n2 == 0:
            return 0.0
        cos_theta = np.clip(dot / (n1 * n2), -1.0, 1.0)
        return np.arccos(cos_theta)

    v1 = Vector2D(1, 0)
    v2 = Vector2D(0, 1)
    v3 = Vector2D(1, 1)
    assert np.isclose(angle(v1, v2), np.pi / 2)
    assert np.isclose(angle(v1, v3), np.pi / 4)
    assert np.isclose(angle(v1, v1), 0.0)
