import time
from typing import Optional, Dict

from aido_schemas import DB20ObservationsPlusState, Context, DTSimRobotInfo, GetCommands, PWMCommands, DB20Commands
from duckietown_world import get_lane_poses, GetLanePoseResult
from geometry import angle_from_SE2

from gtduckie.agents.base import FullAgentBase
from gtduckie.controllers import SpeedController, LedsController
from gtduckie.controllers.pure_pursuit import PurePursuitParam

__all__ = ["MyFullAgent"]


class MyFullAgent(FullAgentBase):
    duckiebots: Dict[str, DTSimRobotInfo]
    speed_controller: SpeedController = SpeedController()
    # pure_pursuit: PurePursuit = PurePursuit()
    pure_pursuit: PurePursuitParam = PurePursuitParam()
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
        possibilities = list(get_lane_poses(self.dtmap, self.mypose))
        if not possibilities:
            self.myglpr = None  # outside of lane:
        else:
            self.myglpr = possibilities[0]

    def on_received_get_commands(self, context: Context, data: GetCommands):
        t0 = time.time()
        self.speed_controller.update_reference(0.2)  # fixme maybe this can be done when we receive the observations
        speed = self.speed_controller.get_control(at=data.at_time)
        context.debug(f"speed: {speed}")

        if self.myglpr is not None:
            next_along_lane = self.myglpr.lane_pose.along_lane + self.pure_pursuit.look_ahead
            beta = self.myglpr.lane_segment.beta_from_along_lane(next_along_lane)
            _, goal_point = self.myglpr.lane_segment.find_along_lane_closest_point(beta)

            relative_heading = angle_from_SE2(goal_point) - angle_from_SE2(self.mypose)
            context.debug(f"relative heading: {relative_heading}")
            k = 0.0
            turn = -k * relative_heading
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
