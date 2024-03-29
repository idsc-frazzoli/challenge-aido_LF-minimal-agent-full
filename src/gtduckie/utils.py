from geometry import SE2value, translation_angle_from_SE2
import numpy as np


def euclidean_between_SE2value(p0: SE2value, p1: SE2value) -> float:
    t0, _ = translation_angle_from_SE2(p0)
    t1, _ = translation_angle_from_SE2(p1)
    return np.linalg.norm(t0 - t1)
