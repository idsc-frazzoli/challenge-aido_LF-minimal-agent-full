import time
from abc import ABC, abstractmethod
from typing import cast, Optional, Dict

import numpy as np
import yaml

from aido_agents import get_blinking_LEDs_left, get_blinking_LEDs_right, get_braking_LEDs, jpg2rgb
from aido_schemas import (
    Context,
    DB20Commands,
    DB20ObservationsPlusState,
    EpisodeStart,
    GetCommands,
    JPGImage,
    PWMCommands,
    DTSimRobotInfo
)
from duckietown_world import construct_map, DuckietownMap, get_lane_poses, GetLanePoseResult
from gtduckie.controllers import SpeedController, PurePursuit, LedsController

__all__ = ["FullAgentBase", "FullAgent", "MyFullAgent"]


class FullAgentBase(ABC):
    dtmap: Optional[DuckietownMap]
    mypose: np.ndarray
    myname: Optional[str]
    is_first_callback: bool = True

    def init(self, context: Context):
        context.info("FullAgent init()")
        self.dtmap = None

    def _init_map(self, map_data: dict):
        self.dtmap = construct_map(map_data)
        print(self.dtmap)

    def on_received_seed(self, data: int):
        np.random.seed(data)

    def on_received_episode_start(self, context: Context, data: EpisodeStart):
        # This is called at the beginning of episode.
        context.info(f'Starting episode "{data.episode_name}".')

    @abstractmethod
    def on_received_observations(self, context: Context, data: DB20ObservationsPlusState):
        pass

    def init_observations(self, context: Context, data: DB20ObservationsPlusState):
        if self.myname is None:
            self.myname = data.your_name
            context.info(f'Myname is {self.myname}')
        if self.dtmap is None:
            context.info("Loading map")
            yaml_str = cast(str, data.map_data)
            map_data = yaml.load(yaml_str, Loader=yaml.SafeLoader)
            self._init_map(map_data)
            context.info("Loading map done")
        self.is_first_callback = False

    @abstractmethod
    def on_received_get_commands(self, context: Context, data: GetCommands):
        pass

    def finish(self, context: Context):
        context.info("finish()")


class FullAgent(FullAgentBase):

    def on_received_observations(self, context: Context, data: DB20ObservationsPlusState):
        if self.is_first_callback:
            self.init_observations(context=context, data=data)

        mystate = data.state.duckiebots[self.myname]
        self.mypose = mystate.pose

        # context.info(f'state {state}')
        # Get the JPG image
        # camera: JPGImage = data.camera
        # Convert to numpy array
        # _rgb = jpg2rgb(camera.jpg_data)

    def on_received_get_commands(self, context: Context, data: GetCommands):
        pose: np.array = self.mypose

        # context.info('Which lane am I in?')

        t0 = time.time()
        possibilities = list(get_lane_poses(self.dtmap, pose))
        if not possibilities:  # outside of lane:
            speed = 0.05
            turn = 0.1

            led_commands = get_braking_LEDs(data.at_time)

        else:
            glpr: GetLanePoseResult = possibilities[0]
            lane_pose = glpr.lane_pose
            # context.info(debug_print(lane_pose))
            #
            k = 0.1
            speed = 0.1
            turn = -k * lane_pose.relative_heading

            if turn > 0:
                led_commands = get_blinking_LEDs_left(data.at_time)
            else:
                led_commands = get_blinking_LEDs_right(data.at_time)

        pwm_left = speed - turn
        pwm_right = speed + turn

        pwm_commands = PWMCommands(motor_left=pwm_left, motor_right=pwm_right)

        commands = DB20Commands(pwm_commands, led_commands)
        dt = time.time() - t0
        context.write("commands", commands)
        context.info(f"commands computed in {dt:.3f} seconds")


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
        possibilities = list(get_lane_poses(self.dtmap, self.mypose))
        if not possibilities:
            self.myglpr = None  # outside of lane:
        else:
            self.myglpr = possibilities[0]

    def on_received_get_commands(self, context: Context, data: GetCommands):
        t0 = time.time()
        self.speed_controller.update_reference(0.2)  # fixme maybe this can be done when we receive the observations
        speed = self.speed_controller.get_control(at=data.at_time)
        context.info(f"turn: {speed}")

        if self.myglpr is not None:
            next_along_lane = self.myglpr.lane_pose.along_lane + self.pure_pursuit.param.look_ahead
            beta = self.myglpr.lane_segment.beta_from_along_lane(next_along_lane)
            _, goal_point = self.myglpr.lane_segment.find_along_lane_closest_point(beta)

            relative_heading = goal_point[2] - self.mypose[2]
            context.info(f"relative heading: {relative_heading}")
            k = 0.1
            turn = -k * relative_heading
        else:
            turn = 0.1  # fixme totally random fallback
        context.info(f"turn: {turn}")

        # turn = self.pure_pursuit.compute_steering_angle(at=data.at_time)

        pwm_left = speed - turn
        pwm_right = speed + turn

        pwm_commands = PWMCommands(motor_left=pwm_left, motor_right=pwm_right)
        led_commands = self.leds_controller.get_led_lights(new_speed=speed, new_turn=turn, t=data.at_time)
        commands = DB20Commands(pwm_commands, led_commands)

        dt = time.time() - t0
        context.write("commands", commands)
        context.info(f"commands computed in {dt:.3f} seconds")
