from typing import Tuple, Union

import contracts
from geometry import SE2_from_translation_angle, SE2value
from math import cos, pi, sin

from gtduckie.utils import euclidean_between_SE2value

contracts.disable_all()
import matplotlib.pyplot as plt
from duckietown_world.world_duckietown.sampling import sample_many_good_starting_poses
import numpy as np
import duckietown_world as dw
from duckietown_world import get_lane_poses, SE2Transform, relative_pose

from gtduckie import MyFullAgent


def arrow_from_SE2(p: Union[SE2Transform, SE2value]) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    if isinstance(p, SE2value):
        p = SE2Transform.from_SE2(p)
    x, y = p.p
    l = .1
    dx = cos(p.theta) * l
    dy = sin(p.theta) * l
    return (x, y), (dx, dy)


def test_agent():
    # init agent
    agent = MyFullAgent()
    agent.pure_pursuit.update_speed(0.2)

    m = dw.load_map("robotarium1")
    # random.seed(9)
    # q = sample_good_starting_pose(m, only_straight=False, along_lane=0.9)
    poses = sample_many_good_starting_poses(m,
                                            nrobots=10,
                                            only_straight=False,
                                            min_dist=1.0,
                                            delta_theta_rad=0.5,
                                            delta_y_m=0.1,
                                            )
    print([SE2Transform.from_SE2(p) for p in poses])
    for q in poses:
        ax = plt.axes()
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
        for ctr_point in agent.pure_pursuit.path.control_points:
            tail, head = arrow_from_SE2(ctr_point)
            ax.arrow(tail[0], tail[1], head[0], head[1], head_width=0.01, head_length=0.05, fc='k', ec='k')

        print("Agent mypose: ", SE2Transform.from_SE2(agent.mypose))

        rel = relative_pose(agent.myglpr.lane_segment_transform.asmatrix2d().m, q)
        print("Agent relative pose [rel]: ", SE2Transform.from_SE2(rel))
        tail, head = arrow_from_SE2(rel)
        ax.arrow(tail[0], tail[1], head[0], head[1], head_width=0.01, head_length=0.05, fc='r', ec='r')

        agent.pure_pursuit.update_pose(rel, agent.myglpr.lane_pose.along_lane)
        along_lane, goal_point = agent.pure_pursuit.find_goal_point()
        g_point = SE2Transform.from_SE2(goal_point)
        ax.plot(g_point.p[0], g_point.p[1], "x")

        print("Goal point:", along_lane, SE2Transform.from_SE2(goal_point))
        turn = agent.pure_pursuit.get_pwmturn_factor()
        print("Turn [rel pose]: ", turn)
        side = "left" if turn > 0 else "right"
        ax.text(0, 0, f"turn: {turn:.3f}({side})")
        plt.xlim(-.7, .7)
        plt.ylim(-.7, .7)
        plt.show()

        # ipython_draw_html(m);


def test_speed_behavior():
    ax = plt.axes()
    p1 = SE2_from_translation_angle([0, 0], 0)
    p2 = SE2_from_translation_angle([1, -1], pi / 4)
    p3 = SE2_from_translation_angle([0.3, .2], - pi)
    for p in [p1, p2, p3]:
        tail, head = arrow_from_SE2(p)
        ax.arrow(tail[0], tail[1], head[0], head[1], head_width=0.03, head_length=0.06, fc='r', ec='r')

    print("Distance: ", euclidean_between_SE2value(p1, p2))
    print("Rel pose: ", SE2Transform.from_SE2(relative_pose(p1, p2)))
    print("Rel pose: ", SE2Transform.from_SE2(relative_pose(p1, p3)))

    plt.xlim(-1.5, 1.5)
    plt.ylim(-1.5, 1.5)
    plt.show()
