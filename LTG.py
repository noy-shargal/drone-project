from typing import List
from Obstacle import ThinWallObstacle
import numpy as np
from shapely.geometry import Point, LineString


class TGEdge:
    def __init__(self, vertex1, vertex2, virtual_edge, distance=None):
        self._vertex1 = vertex1
        self._vertex2 = vertex2
        self.is_virtual_edge = virtual_edge
        if distance is None:
            distance = self._vertex1.distance(self._vertex2)
        self.distance = distance

    def get_distance(self):
        return self._vertex1.distance(self._vertex2)


class TGVertex:

    def __init__(self, point: Point, admissible=False, vtype="INNER"):
        super().__init__()
        self._edges = list()
        self._point = point
        self.vtype = vtype
        self._compare_epsilon = 0.5
        self._handled_status = False
        self.is_admissible = admissible
        self._distance_to_target = None

    def __eq__(self, other):
        return self.distance(other) < self._compare_epsilon

    def add_edge(self, edge):
        self._edges.append(edge)

    def get_edges(self):
        return self._edges

    def get_coordinates(self):
        return self._point.x, self._point.y

    def point(self) -> Point:
        return self._point

    def distance(self, vertex):
        return self._point.distance(vertex.point())

    def set_status(self, status, distance_to_target):
        self._handled_status = status
        self._distance_to_target = distance_to_target

    def get_status(self):
        return self._handled_status

    def get_distance_to_target(self):
        return self._distance_to_target


class AugmentedSubGraph:
    LIDAR_RANGE = 35

    def __init__(self, current_location: Point, target: Point):
        self._current_location = current_location
        self._target = target

        self._source_vertex = None
        self._target_vertex = None

        self._vertices = list()
        self._edges = list()

        self._t_blocking_obstacle = None
        self._blocking_obstacle = None

        self._build_obstacle_vertices()
        self._add_source_vertex()
        self._try_to_add_target_vertex()

        self._add_edges()

    def remove_duplicate_vertices(self):
        self._vertices = list(dict.fromkeys(self._vertices))

    def get_vertices(self):
        return self._vertices

    def _get_t_node_position(self, curr_position: Point, target: Point):
        theta = np.atan2(target.y - curr_position.y, target.x - curr_position.x)
        r = np.min(self.LIDAR_RANGE, curr_position.distance(target))
        delta_x = r * np.cos(theta)
        delta_y = r * np.sin(theta)
        pos = Point(curr_position.x + delta_x, curr_position.y + delta_y)
        return pos

    def try_to_add_T_node(self, curr_pos: Point, target, obstacles):
        line_to_target = LineString([curr_pos, target])
        for obs in obstacles:
            obstacle_line = LineString([obs._first_endpoint, obs._second_endpoint])
            if line_to_target.intersects(obstacle_line):
                self._t_blocking_obstacle = obs
                return

        t_node = TGVertex(self._get_t_node_position(curr_pos, target), admissible=True, vtype="T_NODE")
        self.add_vertex(t_node)

    def add_vertex(self, vertex: TGVertex):
        self._vertices.append(vertex)

        if vertex.vtype == "START":
            self._start_vertex = vertex

        if vertex.vtype == "TARGET":
            self._target_vertex = vertex

    def get_start(self):
        return self._start_vertex

    def get_target(self):
        return self._target_vertex

    def add_edge(self, edge: TGEdge):
        self._edges.append(edge)

    def get_closet_point_to_target(self):
        min_distance = np.float(np.inf)
        point = None
        for v in self._vertices:
            if v.is_admissible:
                distance = self._current_location.distance(v.point()) + \
                           v.get_distance_to_target()
                if min_distance > distance:
                    min_distance = distance
                    point = v.point()

        assert point
        return point, min_distance

    def get_blocking_obstacle(self):
        if self._blocking_obstacle is None:
            return self._t_blocking_obstacle
        return self._blocking_obstacle

    def calculate_d_min(self):
        d_min, _ = self._t_blocking_obstacle.get_closest_point_to_target()
        return d_min

    def update_blocking_obstacle(self, curr_pos: Point, obstacles: List[ThinWallObstacle]):
        min_dist = np.float(np.inf)
        relevant_obs = None
        for obs in obstacles:
            distance = obs._line.project(curr_pos)
            if distance < min_dist:
                min_dist = distance
                relevant_obs = obs
        self._blocking_obstacle = relevant_obs

    def is_g2_empty(self, d_min):
        _, min_distance = self.get_closet_point_to_target()
        return min_distance >= d_min
