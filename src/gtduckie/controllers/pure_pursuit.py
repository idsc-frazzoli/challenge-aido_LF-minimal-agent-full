from collections import Sequence
from typing import Optional
import numpy as np
from dataclasses import dataclass
from geometry import SE2value

__all__ = ["PurePursuit"]


@dataclass
class PurePursuitParam:
    look_ahead: float = 0.1
    min_distance: float = 0.1
    steering_factor: float = 0.1


class PurePursuit:

    def __init__(self):
        """
        initialise pure_pursuit control loop
        :param
        """
        self.path: Optional[Sequence[SE2value]] = None
        self.pose: SE2value
        self.look_ahead_point: SE2value
        self.start_index: int = 0
        self.speed: float = 0
        self.near_point_index: int = 0
        self.curvature: float = 0
        self.length: float = 1.19
        self.steering_angle: float = 0
        self.param: PurePursuitParam = PurePursuitParam()

    def update_path(self, path: Sequence[SE2value]):
        self.path = path

    def update_state(self, pose: SE2value, velocity: SE2value):
        """
        update current location
        :param pose: SE2value
        :return:
        """
        self.pose = pose
        self.speed = velocity.x

        if self.look_ahead_point is None and self.is_start_controller is True:
            self.look_ahead_point = SE2value()
            angle = self.pose.theta
            rotation = np.matmul(
                np.array([[np.round(np.cos(angle), 1), -np.round(np.sin(angle), 1)],
                          [np.round(np.sin(angle), 1), np.round(np.cos(angle), 1)]]),
                np.array([[self.param.look_ahead], [0.0]]))
            self.look_ahead_point.x = float(rotation[0]) + self.pose.x
            self.look_ahead_point.y = float(rotation[1]) + self.pose.y

    def find_nearest_point_index_on_path(self) -> int:
        """
        find nearest point on path to gokart
        :return:
        """
        near_point_dist = float('inf')
        near_point_index = None
        for i in range(len(self.path)):
            dist = np.sqrt((self.pose.x - self.path[i].x) ** 2 + (self.pose.y - self.path[i].y) ** 2)
            if dist < near_point_dist:
                near_point_dist = dist
                near_point_index = i
        # error if none
        return near_point_index

    def find_goal_point(self, path: Sequence[SE2value]) -> SE2value:
        """
        find goal point on path
        :return: SE2value
        """
        dist_array = []
        pose = []

        dist = np.linalg.norm(self.look_ahead_point[:2] - self.pose[:2])
        min_dist = self.param.min_distance

        if dist < min_dist:
            for i in range(self.near_point_index, (self.near_point_index + 20) % len(path)):
                dist = np.sqrt((self.pose.x - path[i].x) ** 2 + (self.pose.y - path[i].y) ** 2)

                dist_array.append(abs(dist - self.param.look_ahead))
                pose.append(path[i])

            if len(dist_array) != 0:
                print('dist', min(dist_array))
                look_ahead_point_index = dist_array.index(min(dist_array))
                self.look_ahead_point.x = pose[look_ahead_point_index].x
                self.look_ahead_point.y = pose[look_ahead_point_index].y

        return self.look_ahead_point

    def get_curvature(self) -> float:
        """
        find curvature
        :return: float
        """
        self.goal_point_in_vehicle_frame.x = self.look_ahead_point.x - self.pose.x
        self.goal_point_in_vehicle_frame.y = self.look_ahead_point.y - self.pose.y
        self.goal_point_in_vehicle_frame.theta = self.pose.theta

        angle = -self.pose.theta
        rotation = np.matmul(
            np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]),
            np.array([[self.goal_point_in_vehicle_frame.x], [self.goal_point_in_vehicle_frame.y]]))
        self.goal_point_in_vehicle_frame.y = float(rotation[1])
        perpendicular_dist = self.goal_point_in_vehicle_frame.y

        self.curvature = (2 * perpendicular_dist / self.look_ahead_distance ** 2)

        return self.curvature

    def compute_steering_angle(self) -> float:
        """
        gives steering angle
        :return: float
        """
        # put here the logic
        self.steering_angle = np.arctan(self.length * self.curvature)
        return self.param.steering_factor * self.steering_angle
