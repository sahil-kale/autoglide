import numpy as np
from dataclasses import dataclass


@dataclass
class Quaternion:
    w: float
    x: float
    y: float
    z: float

    def as_np_array(self) -> np.ndarray:
        return np.array([self.w, self.x, self.y, self.z])

    @staticmethod
    def from_np_array(arr: np.ndarray) -> "Quaternion":
        if arr.shape != (4,):
            raise ValueError("Array must be of shape (4,)")
        return Quaternion(w=arr[0], x=arr[1], y=arr[2], z=arr[3])

    @staticmethod
    def from_euler(roll: float, pitch: float, yaw: float) -> "Quaternion":
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return Quaternion(w, x, y, z)

    def normalize(self) -> "Quaternion":
        norm = np.linalg.norm(self.as_np_array())
        if norm == 0:
            raise ValueError("Cannot normalize a zero-length quaternion")
        return Quaternion(
            w=self.w / norm, x=self.x / norm, y=self.y / norm, z=self.z / norm
        )

    def conjugate(self) -> "Quaternion":
        return Quaternion(w=self.w, x=-self.x, y=-self.y, z=-self.z)

    def multiply(self, other: "Quaternion") -> "Quaternion":
        w1, x1, y1, z1 = self.w, self.x, self.y, self.z
        w2, x2, y2, z2 = other.w, other.x, other.y, other.z
        return Quaternion(
            w=w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            x=w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            y=w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            z=w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        )

    def rotate_vector(self, v: np.ndarray) -> np.ndarray:
        if v.shape != (3,):
            raise ValueError("Vector must be of shape (3,)")
        q_vec = Quaternion(0, v[0], v[1], v[2])
        q_res = self.multiply(q_vec).multiply(self.conjugate())
        return np.array([q_res.x, q_res.y, q_res.z])
