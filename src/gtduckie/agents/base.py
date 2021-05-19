from abc import ABC, abstractmethod
from aido_schemas import Context, EpisodeStart, DB20ObservationsPlusState, GetCommands
from typing import Optional, cast, Tuple

from duckietown_world import DuckietownMap, construct_map, GetLanePoseResult, get_lane_poses
import numpy as np
import yaml

__all__ = ["FullAgentBase"]


class FullAgentBase(ABC):
    myname: str
    mypose: np.ndarray
    pwm_limits: Tuple[float, float] = (-1, 1)
    dtmap: Optional[DuckietownMap] = None
    is_first_callback: bool = True

    def init(self, context: Context):
        context.info("FullAgent init()")
        self.dtmap = None

    def _init_map(self, map_data: dict):
        self.dtmap = construct_map(map_data)
        # print(self.dtmap)

    def on_received_seed(self, data: int):
        np.random.seed(data)

    def on_received_episode_start(self, context: Context, data: EpisodeStart):
        # This is called at the beginning of episode.
        context.info(f'Starting episode "{data.episode_name}".')

    @abstractmethod
    def on_received_observations(self, context: Context, data: DB20ObservationsPlusState):
        pass

    def init_observations(self, context: Context, data: DB20ObservationsPlusState):
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

    @staticmethod
    def finish(context: Context):
        context.info("finish()")

