import random

import contracts
import numpy as np
from duckietown_world import get_lane_poses, SE2Transform, relative_pose

from gtduckie import MyFullAgent

contracts.disable_all()
import duckietown_world as dw
from duckietown_world.world_duckietown.sampling_poses import sample_good_starting_pose


def test_agent():
    # init agent
    agent = MyFullAgent()
    agent.pure_pursuit.update_speed(0.2)

    m = dw.load_map("robotarium2")
    random.seed(34)
    q = sample_good_starting_pose(m, only_straight=False, along_lane=0.02)

    possibilities = list(get_lane_poses(m, q))
    s = sorted(possibilities, key=lambda _: np.abs(_.lane_pose.relative_heading))
    res = {}
    for i, _ in enumerate(s):
        res[i] = _
    myglpr = res[0]

    agent.myglpr = myglpr
    agent.pure_pursuit.update_path(agent.myglpr.lane_segment)
    agent.mypose = q
    print("Lane segment control points: ", agent.pure_pursuit.path.control_points)
    print("Agent mypose: ", SE2Transform.from_SE2(agent.mypose))

    rel = relative_pose(agent.myglpr.lane_segment_transform.asmatrix2d().m, q)
    print("Agent relative pose [rel]: ", SE2Transform.from_SE2(rel))

    agent.pure_pursuit.update_pose(rel, agent.myglpr.lane_pose.along_lane)
    along_lane, goal_point = agent.pure_pursuit.find_goal_point()

    print("Goal point:", along_lane,SE2Transform.from_SE2(goal_point))
    turn = agent.pure_pursuit.get_turn_factor()
    print("Turn [rel pose]: ", turn)

