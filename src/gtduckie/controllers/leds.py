from dataclasses import dataclass

from aido_agents import get_braking_LEDs, get_blinking_LEDs_left, get_blinking_LEDs_right, get_normal_LEDs
from aido_schemas import LEDSCommands

__all__ = ["LedsController"]


@dataclass
class LedsController:
    last_speed_cmd: float = 0
    last_turn_cmd: float = 0
    turn_light_thresh: float = 0.1

    def get_led_lights(self, new_speed: float, new_turn: float, t: float) -> LEDSCommands:
        acc = new_speed - self.last_speed_cmd
        self.last_speed_cmd = new_speed
        self.last_turn_cmd = new_turn
        if acc < 0:
            led_commands = get_braking_LEDs(t)
        else:
            if new_turn > self.turn_light_thresh:
                led_commands = get_blinking_LEDs_left(t)
            elif new_turn < -self.turn_light_thresh:
                led_commands = get_blinking_LEDs_right(t)
            else:
                led_commands = get_normal_LEDs(t)
        return led_commands
