from random import choice

import contracts

from duckietown_world.world_duckietown.sampling_poses import sample_good_starting_pose

contracts.disable_all()

import numpy as np

from duckietown_world import SE2Transform, get_lane_poses
import duckietown_world as dw
from duckietown_world.svg_drawing.ipython_utils import ipython_draw_html

dw.logger.setLevel(50)


def test_lane_to_follow():
    available_map = dw.list_maps()
    print(available_map)
    map_name = choice(available_map)
    print(f"Loading {map_name}")
    m = dw.load_map("robotarium2")
    sk = dw.get_skeleton_graph(m)
    ipython_draw_html(m);

    q = sample_good_starting_pose(m, only_straight=False, along_lane=0.02)
    print(SE2Transform.from_SE2(q))
    possibilities = list(get_lane_poses(m, q))
    print(f"There are {len(possibilities)} possibilities")
    glpr = possibilities[0]

    ctr_points = glpr.lane_segment.control_points
    abs_ctr_points = []
    for ctr_point in ctr_points:
        abs_point = SE2Transform.from_SE2(
            glpr.lane_segment_transform.asmatrix2d().m @ ctr_point.as_SE2())
        abs_ctr_points.append(
            abs_point
        )

    print(ctr_points)
    print(abs_ctr_points)
    ctr_point_id = None
    print("looking for a match of ", abs_ctr_points[-1])
    for node_id, node in sk.G.nodes.items():
        print("candidate: ", node["point"])
        if np.allclose(node["point"].p, abs_ctr_points[-1].p, rtol=1.e-3):
            ctr_point_id = node_id
            print(node["point"].theta)
            print(abs_ctr_points[-1].theta)
            break

    print("ctr_point: ", ctr_point_id)
    succ = sk.G[ctr_point_id]
