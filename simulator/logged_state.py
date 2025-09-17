from dataclasses import dataclass
import json


@dataclass
class LoggedState:
    time: float
    glider_x: float
    glider_y: float
    glider_h: float
    glider_V: float
    glider_phi: float
    glider_psi: float
    control_phi: float
    control_V: float
    disturbance_w: float
    estimator_confidence: float
    guidance_state: str
    est_thermal_x: float = 0.0
    est_thermal_y: float = 0.0
    est_thermal_strength: float = 0.0
    est_thermal_radius: float = 0.0
    actual_thermal_x: float = 0.0
    actual_thermal_y: float = 0.0
    actual_thermal_radius: float = 0.0

    @staticmethod
    def from_json(d):
        return LoggedState(
            time=d.get("time", 0.0),
            glider_x=d.get("glider_x", 0.0),
            glider_y=d.get("glider_y", 0.0),
            glider_h=d.get("glider_h", 0.0),
            glider_V=d.get("glider_V", 0.0),
            glider_phi=d.get("glider_phi", 0.0),
            glider_psi=d.get("glider_psi", 0.0),
            control_phi=d.get("control_phi", 0.0),
            control_V=d.get("control_V", 0.0),
            disturbance_w=d.get("disturbance_w", 0.0),
            estimator_confidence=d.get("estimator_confidence", 0.0),
            guidance_state=d.get("guidance_state", ""),
            est_thermal_x=d.get("est_thermal_x", 0.0),
            est_thermal_y=d.get("est_thermal_y", 0.0),
            est_thermal_strength=d.get("est_thermal_strength", 0.0),
            est_thermal_radius=d.get("est_thermal_radius", 0.0),
            actual_thermal_x=d.get("actual_thermal_x", 0.0),
            actual_thermal_y=d.get("actual_thermal_y", 0.0),
            actual_thermal_radius=d.get("actual_thermal_radius", 0.0),
        )

    def to_json(self):
        d = self.__dict__.copy()
        return json.dumps(d)
