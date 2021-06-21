import time

import numpy as np

from aido_agents import get_blinking_LEDs_left, get_blinking_LEDs_right, get_braking_LEDs
from aido_schemas import Context, DB20Commands, DB20ObservationsPlusState, GetCommands, PWMCommands
from duckietown_world import get_lane_poses, GetLanePoseResult
from .base import FullAgentBase


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
