import time
from typing import Optional, Dict
import numpy as np
from aido_schemas import DB20ObservationsPlusState, Context, DTSimRobotInfo, GetCommands, PWMCommands, DB20Commands
from duckietown_world import get_lane_poses, GetLanePoseResult, relative_pose, SE2Transform

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
        speed_ref: float = .2
        self.speed_controller.update_reference(speed_ref)
        speed = speed_ref + self.speed_controller.get_control(at=data.at_time)
        context.debug(f"speed: {speed}")
        self.pure_pursuit.update_speed(speed_ref)
        if self.myglpr is not None:
            turn = self.pure_pursuit.get_pwmturn_factor()
        else:
            # totally random fallback
            turn = 0.1

        context.debug(f"turn: {turn}")
        pwm_left = float(np.clip(speed - turn / 2, self.pwm_limits[0], self.pwm_limits[1]))
        pwm_right = float(np.clip(speed + turn / 2, self.pwm_limits[0], self.pwm_limits[1]))

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
