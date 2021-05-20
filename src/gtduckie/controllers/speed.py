from typing import Optional, Tuple
import numpy as np
from dataclasses import dataclass

__all__ = ["SpeedController"]

from geometry import xytheta_from_SE2


@dataclass
class SpeedControllerParam:
    kP: float = 0.1
    kI: float = 0.0
    antiwindup: Tuple[float, float] = (-0.5, 0.5)
    setpoint_minmax: Tuple[float, float] = (-1, 1)


class SpeedController:

    def __init__(self):
        self.params = SpeedControllerParam()
        self.current_speed: float = 0
        self.desired_speed: float = 0
        self.last_request_at: Optional[float] = None
        self.last_integral_error: float = 0

    def update_observations(self, current_velocity: np.ndarray):
        self.current_speed = xytheta_from_SE2(current_velocity)[0]

    def update_reference(self, desired_speed: float):
        if not self.params.setpoint_minmax[0] <= desired_speed <= self.params.setpoint_minmax[1]:
            raise RuntimeWarning("Attempting to set a desired speed out of range. I'll clip the value.")
        self.desired_speed = np.clip(desired_speed, self.params.setpoint_minmax[0], self.params.setpoint_minmax[1])

    def get_control(self, at: float) -> float:
        "A simple PI"
        dt = 0 if self.last_request_at is None else at - self.last_request_at
        self.last_request_at = at
        p_error = self.desired_speed - self.current_speed
        self.last_integral_error += self.params.kI * p_error * dt
        self.last_integral_error = np.clip(self.last_integral_error,
                                           self.params.antiwindup[0],
                                           self.params.antiwindup[1])
        return self.params.kP * p_error + self.last_integral_error
