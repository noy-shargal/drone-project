from typing import List, Dict
from utils import get_point_in_polar_degrees_coords
from shapely.geometry import Point


from LTG import LTG, TGEdge, TGVertex, SubGraph


class PointInfo:

    def __init__(self, r, theta, point_lidar_frame, point_world_frame, is_blocked):
        self.r = r
        self.theta = theta
        self.point_lidar_frame = point_lidar_frame
        self.point_world_frame = point_world_frame
        self.is_blocked = is_blocked

    def __lt__(self, other):
        return self.theta < other.theta


class TGSegment:

    def __init__(self,  is_blocked):
        self._points = list()
        self._is_blocked = is_blocked
        self._vertices = list()

    def is_blocked(self):
        return self._is_blocked

    def add_point(self, point_info: PointInfo):
        self._points.append(point_info)

    def get_endpoints(self):

        min_theta = 10000
        max_theta = -10000
        min_theta_point = None
        max_theta_point = None
        for p_info in  self._points:
            if p_info.r < min_theta:
                min_theta = p_info.r
                min_theta_point = p_info.point_world_frame

            if p_info.r > max_theta:
                max_theta = p_info.r
                max_theta_point = p_info.point_world_frame

        return min_theta_point, max_theta_point



class TangentBug:

    def __init__(self):
        self._segments = list()
        self._ltg = LTG()
        self._sub_graph = None
        self._current_position = None
        self._target = None

        self._current_position_vertex = None
        self._target_vertex = None

        self._lidar_points = list()
        self._lidar_polar_points = list()

        self._obstacles = list()
        self._min_r = 10000

        self._is_obstacle_on_90_to_135_degrees = False
        self._is_obstacle_on_45_to_90_degrees = False

        self._vl = PointInfo( 1, 0, 0, 0, True)
        self._vr = PointInfo( 1, 0, 0, 0, True)

    def set_current_position(self, position: Point):
        self._current_position = position
        self._current_position_vertex = TGVertex(position)
        self._ltg.set_source_vertex(self._current_position_vertex)

    def set_target(self, target: Point):
        self._target = target
        self._target_vertex = TGVertex(target)
        self._ltg.set_target_vertex(self._target_vertex)

    def connect_current_position_to_target(self):
        edge = TGEdge(self._current_position, self._target)
        self._ltg.add_edge(edge)

    def is_way_blocked(self):
        return self._is_obstacle_on_90_to_135_degrees and \
               self._is_obstacle_on_45_to_90_degrees


    def add_point(self, point: Point, world_frame_point: Point, is_blocked = True):
            x, y = point.x, point.y
            r, theta = get_point_in_polar_degrees_coords(x, y)

            if -45 <= theta <= 0:
                self._is_obstacle_on_90_to_135_degrees = True

            if 0 <= theta <= 45:
                self._is_obstacle_on_45_to_90_degrees = True

            if r < self._min_r:
                self._min_r = r

            point_info = PointInfo(r, theta, point, world_frame_point, is_blocked)
            if point_info.theta > self._vr.theta:
                self._vr = point_info

            if point_info.theta < self._vl.theta:
                self._vl = point_info

            self._lidar_polar_points.append(PointInfo(r, theta, point, world_frame_point, is_blocked))

    def get_closest_endpoint_to_target(self,curr_poss: Point):

        dx_vr = curr_poss.distance(self._vr.point_world_frame)
        dh_vr = self._vr.point_world_frame.distance(self._target_vertex.point())
        d_vr = dx_vr + dh_vr

        dx_vl = curr_poss.distance(self._vl.point_world_frame)
        dh_vl = self._vl.point_world_frame.distance(self._target_vertex.point())
        d_vl = dx_vl + dh_vl

        if d_vl < d_vr:



            return self._vl.point_world_frame
        return self._vr.point_world_frame

    def get_num_of_points(self):
        return len(self._lidar_polar_points)

    def get_min_r_distance(self):
        return self._min_r

    def _split_to_segments(self):

        last_blocked_state = None
        segment = None

        for point_info in self._lidar_polar_points:
            if not -45 <=point_info.theta <=45:
                continue
            if last_blocked_state != point_info.is_blocked:
                last_blocked_state = point_info.is_blocked
                segment = TGSegment(point_info.is_blocked)
                self._segments.append(segment)

            segment.add_point(point_info)

    def _build_obstacles(self):
        self._split_to_segments()

    def _get_endpoints_vertices(self, segment: TGSegment):
        p1, p2 = segment.get_endpoints()
        v1 = TGVertex(p1)
        v2 = TGVertex(p2)
        return v1, v2

    def build_ltg(self):
        self._lidar_polar_points.sort()
        self._split_to_segments()
        for segment in self._segments:
            v1, v2 = self._get_endpoints_vertices(segment)
            edge = TGEdge(v1, v2)
            v1.add_edge(edge)
            v2.add_edge(edge)
            self._ltg.add_vertex(v1)
            self._ltg.add_vertex(v2)
            self._ltg.add_edge(edge)

    def build_sub_graph(self):
        self._sub_graph = SubGraph(self._ltg)
        return self._sub_graph

    def clear(self):
        self._lidar_polar_points.clear()
        self._lidar_points.clear()
        self._segments.clear()

        self._segments = list()
        self._ltg = LTG()
        self._sub_graph = None
        self._current_position = None
        self._target = None

        self._current_position_vertex = None
        self._target_vertex = None

        self._obstacles = list()
        self._min_r = 10000

        self._is_obstacle_on_90_to_135_degrees = False
        self._is_obstacle_on_45_to_90_degrees = False

