import time
from typing import Optional, Dict
import numpy as np
from aido_schemas import DB20ObservationsPlusState, Context, DTSimRobotInfo, GetCommands, PWMCommands, DB20Commands
from duckietown_world import get_lane_poses, GetLanePoseResult, relative_pose
from geometry import translation_angle_from_SE2

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
        self.pure_pursuit.update_pose(self.mypose)
        # update lane position
        possibilities = list(get_lane_poses(self.dtmap, self.mypose))
        if not possibilities:
            self.myglpr = None  # outside of lane:
        else:
            self.myglpr = possibilities[0]
            self.pure_pursuit.update_path(self.myglpr.lane_segment)

    def on_received_get_commands(self, context: Context, data: GetCommands):
        t0 = time.time()
        self.speed_controller.update_reference(0.2)
        speed = self.speed_controller.get_control(at=data.at_time)
        context.debug(f"speed: {speed}")
        self.pure_pursuit.update_speed(speed)
        if self.myglpr is not None:
            # start debug
            context.debug(f"mypose: \n{self.mypose}")
            goal_point = self.pure_pursuit.find_goal_point()
            context.debug(f"goal_point: \n{goal_point}")
            # end debug
            k = 0.1 # fixme need to check signs
            turn = self.pure_pursuit.get_turn_factor()
        else:
            turn = 0.1  # fixme totally random fallback
        context.debug(f"turn: {turn}")

        # turn = self.pure_pursuit.compute_steering_angle(at=data.at_time)

        pwm_left = speed - turn
        pwm_right = speed + turn

        pwm_commands = PWMCommands(motor_left=pwm_left, motor_right=pwm_right)
        led_commands = self.leds_controller.get_led_lights(new_speed=speed, new_turn=turn, t=data.at_time)
        commands = DB20Commands(pwm_commands, led_commands)

        dt = time.time() - t0
        context.write("commands", commands)
        context.info(f"commands computed in {dt:.3f} seconds")
