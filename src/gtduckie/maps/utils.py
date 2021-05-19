from typing import List, cast
import itertools as it

import duckietown_world as dw
from duckietown_world import LaneSegment, DuckietownMap, get_skeleton_graph

LaneName = str


def merge_lanes(lanes: List[LaneSegment]) -> LaneSegment:
    """
    Merges a list of consecutive lane segments to one single unified lane segment
    :param lanes: List of consecutive lane segments from a duckietown map.
    :return: One single lane segment
    """
    width = lanes[0].width
    # Make a list of all the control points, while making sure that the points that overlap are only taken once
    contr_points_lanes = list(
        it.chain(
            *[ls.control_points[:-1] if ls is not lanes[-1]
              else ls.control_points for ls in lanes]
        )
    )

    # Creating a unified lane segment
    merged_lane_segments = dw.LaneSegment(
        width=width, control_points=contr_points_lanes
    )
    return merged_lane_segments


def get_lane_segments(duckie_map: DuckietownMap, lane_names: List[LaneName]) -> List[LaneSegment]:
    """
    Given a list of names of consecutive lane segments in a duckietown map (seen in the map network
    found in the maps folder) it returns a list of the corresponding lane segments.
    :param duckie_map: A duckietown map containing the lanes which should be extracted
    :param lane_names: A list of the names of the lane segments as indicated in the map network found in the maps folder
    :return: The list of the lane segments in the same order as the list of names.
    """
    sk = get_skeleton_graph(duckie_map)  # get the skeleton graph
    map_lane_segments = sk.root2  # get the map with all the lane segments
    lane_segments = [cast(LaneSegment, map_lane_segments.children[lane_name]) for lane_name in lane_names]
    return lane_segments
