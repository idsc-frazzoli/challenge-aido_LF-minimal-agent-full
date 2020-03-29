#!/usr/bin/env python3

from dataclasses import dataclass
from typing import cast, Optional

import numpy as np
import yaml

from aido_agents import get_blinking_LEDs_left, get_blinking_LEDs_right, jpg2rgb, get_braking_LEDs
from aido_schemas import (Duckiebot1Commands, Duckiebot1ObservationsPlusState, EpisodeStart, GetCommands, JPGImage,
                          protocol_agent_duckiebot1_fullstate, PWMCommands)
from duckietown_world import construct_map, DuckietownMap, GetLanePoseResult
from duckietown_world.world_duckietown import get_lane_poses
from zuper_nodes_wrapper import Context, wrap_direct
from zuper_typing import debug_print


@dataclass
class FullAgentConfig:
    pass


class FullAgent:
    config: FullAgentConfig = FullAgentConfig()
    dtmap: Optional[DuckietownMap]

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

    def on_received_observations(self, context: Context, data: Duckiebot1ObservationsPlusState):
        myname = data.your_name
        # context.info(f'myname {myname}')
        # state = data.state.duckiebots

        if self.dtmap is None:
            context.info('Loading map')
            yaml_str = cast(str, data.map_data)
            map_data = yaml.load(yaml_str, Loader=yaml.SafeLoader)
            self._init_map(map_data)
            context.info('Loading map done')

        mystate = data.state.duckiebots[myname]
        self.pose = mystate.pose

        # context.info(f'state {state}')
        # Get the JPG image
        camera: JPGImage = data.camera
        # Convert to numpy array
        _rgb = jpg2rgb(camera.jpg_data)

    def on_received_get_commands(self, context: Context, data: GetCommands):
        pose: np.array = self.pose

        # context.info('Which lane am I in?')

        possibilities = list(get_lane_poses(self.dtmap, pose))
        if not possibilities: # outside of lane:
            speed = 0
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

        # commands = PWM + LED
        commands = Duckiebot1Commands(pwm_commands, led_commands)
        # write them out
        context.write("commands", commands)
        context.info('commands computed')

    def finish(self, context: Context):
        context.info("finish()")


def main() -> None:
    node = FullAgent()
    protocol = protocol_agent_duckiebot1_fullstate
    wrap_direct(node=node, protocol=protocol)


if __name__ == "__main__":
    main()
