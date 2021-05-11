from typing import Optional, Tuple
import numpy as np
from dataclasses import dataclass

__all__ = ["SpeedController"]


@dataclass
class SpeedControllerParam:
    kP: float = 0.1
    kI: float = 0.1
    antiwindup: Tuple[float, float] = (-0.5, 0.5)


class SpeedController:

    def __init__(self):
        self.params = SpeedControllerParam()
        self.current_speed: float = 0
        self.desired_speed: float = 0
        self.last_request_at: Optional[float] = None
        self.last_integral_error: float = 0

    def update_observations(self, current_velocity: np.ndarray):
        self.current_speed = current_velocity[0]

    def update_reference(self, desired_speed: float):
        self.desired_speed = desired_speed

    def get_control(self, at: float) -> float:
        dt = 0 if self.last_request_at is None else at - self.last_request_at
        self.last_request_at = at

        p_error = self.desired_speed - self.current_speed
        self.last_integral_error += self.params.kI * p_error * dt
        self.last_integral_error = np.clip(self.last_integral_error, self.params.antiwindup[0],
                                           self.params.antiwindup[1])
        return self.params.kP * p_error + self.last_integral_error
