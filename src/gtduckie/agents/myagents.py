import time
from typing import Optional, Dict, cast
import numpy as np
from aido_schemas import DB20ObservationsPlusState, Context, DTSimRobotInfo, GetCommands, PWMCommands, DB20Commands
from duckietown_world import get_lane_poses, GetLanePoseResult, relative_pose
from geometry import SE2value

from gtduckie.agents.base import FullAgentBase
from gtduckie.controllers import SpeedController, LedsController
from gtduckie.controllers.pure_pursuit import PurePursuit

__all__ = ["MyFullAgent"]


class MyFullAgent(FullAgentBase):
    duckiebots: Dict[str, DTSimRobotInfo]
    speed_controller: SpeedController = SpeedController()
    pure_pursuit: PurePursuit = PurePursuit()
    leds_controller: LedsController = LedsController()
    myglpr: Optional[GetLanePoseResult] = None

    def on_received_observations(self, context: Context, data: DB20ObservationsPlusState):
        if self.is_first_callback:
            self.init_observations(context=context, data=data)
        self.duckiebots: Dict[str, DTSimRobotInfo] = data.state.duckiebots
        self.mypose = self.duckiebots[self.myname].pose

        self.speed_controller.update_observations(
            current_velocity=self.duckiebots[self.myname].velocity)

        # update lane position
        self.update_get_lane_pose_result()
        if self.myglpr is not None:
            self.pure_pursuit.update_path(self.myglpr.lane_segment)
            rel = relative_pose(self.myglpr.lane_segment_transform.asmatrix2d().m, self.mypose)
            self.pure_pursuit.update_pose(
                rel,
                self.myglpr.lane_pose.along_lane)

    def on_received_get_commands(self, context: Context, data: GetCommands):
        t0 = time.time()
        self.speed_controller.update_reference(0.2)
        speed = self.speed_controller.get_control(at=data.at_time)
        context.debug(f"speed: {speed}")
        self.pure_pursuit.update_speed(speed)
        if self.myglpr is not None:
            # start debug
            # context.debug(f"mypose: \n{self.mypose}")
            # _, goal_point = self.pure_pursuit.find_goal_point()
            # context.debug(f"goal_point: \n{_},{goal_point}")
            # _, goal_point_approx = self.pure_pursuit.find_goal_point_approx()
            # context.debug(f"goal_point_approx: \n{_},{goal_point_approx}")
            # end debug
            k = 0.1  # fixme need to check signs and teh missing factor for the transformation rad/s -> pwm
            turn = self.pure_pursuit.get_turn_factor()
        else:
            turn = 0.05  # fixme totally random fallback
        context.debug(f"turn: {turn}")

        pwm_left = float(np.clip(speed - turn, self.pwm_limits[0], self.pwm_limits[1]))
        pwm_right = float(np.clip(speed + turn, self.pwm_limits[0], self.pwm_limits[1]))

        pwm_commands = PWMCommands(motor_left=pwm_left, motor_right=pwm_right)
        led_commands = self.leds_controller.get_led_lights(new_speed=speed, new_turn=turn, t=data.at_time)
        commands = DB20Commands(pwm_commands, led_commands)

        dt = time.time() - t0
        context.write("commands", commands)
        context.info(f"commands computed in {dt:.3f} seconds")

    def update_get_lane_pose_result(self):
        possibilities = list(get_lane_poses(self.dtmap, self.mypose))
        if not possibilities:
            self.myglpr = None  # outside of lane:
        else:
            s = sorted(possibilities, key=lambda _: np.abs(_.lane_pose.relative_heading))
            res = {}
            for i, _ in enumerate(s):
                res[i] = _
            self.myglpr = res[0]
        return
