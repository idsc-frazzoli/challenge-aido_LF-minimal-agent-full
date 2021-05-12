from math import sin
from typing import Optional, Tuple

import numpy as np
from dataclasses import dataclass

import scipy.optimize
from duckietown_world import LaneSegment
from geometry import SE2value, translation_angle_from_SE2

__all__ = ["PurePursuit"]

from gtduckie.utils import euclidean_between_SE2value


@dataclass
class PurePursuitParam:
    look_ahead: float = 0.3
    min_distance: float = 0.1
    max_extra_distance: float = 0.5


class PurePursuit:
    "https://ethz.ch/content/dam/ethz/special-interest/mavt/dynamic-systems-n-control/idsc-dam/Lectures/amod/AMOD_2020/20201019-05%20-%20ETHZ%20-%20Control%20in%20Duckietown%20(PID).pdf"

    def __init__(self):
        """
        initialise pure_pursuit control loop
        :param
        """
        self.path: Optional[LaneSegment] = None
        self.pose: Optional[SE2value] = None
        self.speed: float = 0
        self.param: PurePursuitParam = PurePursuitParam()

    def update_path(self, path: LaneSegment):
        assert isinstance(path, LaneSegment)
        self.path = path

    def update_pose(self, pose: SE2value):
        assert isinstance(pose, SE2value)
        self.pose = pose

    def update_speed(self, speed: float):
        self.speed = speed

    def find_goal_point(self) -> Tuple[float, SE2value]:
        """
        find goal point on path
        :return: along_lane, SE2value
        """
        # todo test this function

        def goal_point_error(along_lane: float) -> float:
            """
            :param along_lane:
            :return: euclidean distance between self.pose and a point along_lane
            """
            beta = self.path.beta_from_along_lane(along_lane)
            cp = self.path.center_point(beta)
            dist = euclidean_between_SE2value(self.pose, cp)
            return np.linalg.norm(dist - self.param.look_ahead)

        min_along_lane = self.path.lane_pose_from_SE2_generic(self.pose).along_lane + self.param.min_distance

        bounds = [min_along_lane,
                  min_along_lane + self.param.look_ahead + self.param.max_extra_distance]
        res = scipy.optimize.minimize_scalar(fun=goal_point_error, bounds=bounds, tol=0.001)
        goal_point = self.path.center_point(self.path.beta_from_along_lane(res.x))
        return res.x, goal_point

    def find_goal_point_approx(self) -> Tuple[float, SE2value]:
        """
        Approximate goal point on the path, from your projection on the lane advance by lookahead
        :return:
        """
        # todo test this function
        along_lane = self.path.lane_pose_from_SE2_generic(self.pose).along_lane
        along_lane_goal_point = along_lane + self.param.look_ahead
        beta = self.path.beta_from_along_lane(along_lane_goal_point)
        return along_lane_goal_point, self.path.center_point(beta)

    def get_turn_factor(self) -> float:
        """
        gives "rotational velocity"
        :return: float
        """
        if any([_ is None for _ in [self.pose, self.path]]):
            raise RuntimeError("Attempting to use pure pursuit before having ady observations/path")
        p, theta = translation_angle_from_SE2(self.pose)
        _, goal_point = self.find_goal_point()
        p_goal, theta_goal = translation_angle_from_SE2(goal_point)
        alpha = theta - np.arctan2(p_goal[1] - p[1], p_goal[0] - p[0])
        radius = self.param.look_ahead / 2 * sin(alpha)
        return self.speed / radius
