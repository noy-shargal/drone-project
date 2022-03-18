from typing import List, Dict
from utils import get_point_in_polar_coords
from shapely.geometry import Point

from LTG import LTG, TGEdge, TGVertex

class TGSegment:

    def __init__(self,  is_blocked):
        self._points = list()
        self._is_blocked = is_blocked

    def add_point(self, point:Point):
        self._points.append(point)

class TangentBug:

    def __init__(self):
        self._segments = list()
        self._ltg = LTG()
        self._current_position = None
        self._target = None

        self._current_position_vertex = None
        self._target_vertex = None

        self._lidar_points = list()
        self._lidar_polar_points = dict()

        self._obstacles = list()

    def set_current_position(self, position: Point):
        self._current_position = position
        self._current_position_vertex = TGVertex(position)
        self._ltg.add_vertex(self._current_position_vertex)

    def set_target(self, target: Point):
        self._target = target
        self._target_vertex = TGVertex(target)
        self._ltg.add_vertex(self._target_vertex)

    def connect_current_position_to_target(self):
        edge = TGEdge(self._current_position, self._target)
        self._ltg.add_edge(edge)

    def add_points(self, points: List):

        self._lidar_points.extend(points)
        for point, is_blocked in points:
            x, y = point.x, point.y
            r, theta = get_point_in_polar_coords(x, y)
            self._lidar_polar_points[(r, theta)] = is_blocked

    def _split_to_segments(self, points: Dict):

        points_list = points.items()
        last_blocked_state = None
        segment = None
        for i in range(len(points_list)):
            point, is_blocked = points_list[i]
            if last_blocked_state != is_blocked:
                last_blocked_state = is_blocked
                segment = TGSegment(is_blocked)
                self._segments.append(segment)
            segment.add_point(point)

    def build_obstacles(self):
        self._split_to_segments()

    def build_ltg(self):
        pass

    def build_sub_graph(self):
        pass



    def clear(self):
        self._lidar_polar_points.clear()
        self._lidar_points.clear()
        self._segments.clear()

