from typing import List

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

    def __init__(self, point: Point, admissible = False, vtype="INNER"):
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


class LTG:

    def __init__(self):
        self._edges = list()
        self._vertices = list()
        self._source_vertex = None
        self._target_vertex = None


    def add_edges(self, edges: List):
        self._edges.extend(edges)

    def add_edge(self, edge: TGEdge):
        self._edges.append(edge)

    def add_vertices(self, vertices: List):
        self._vertices.extend(vertices)

    def add_vertex(self, vertex: TGVertex):

        self._vertices.append(vertex)

    def set_source_vertex(self, source: TGVertex):
        self._source_vertex = source
        self._source_vertex.vtype = "SOURCE"
        self.add_vertex(source)

    def set_target_vertex(self, target: TGVertex):
        self._target_vertex = target
        self._target_vertex.vtype = "TARGET"
        self.add_vertex(target)

    def get_source(self) -> TGVertex:
        return self._source_vertex

    def get_target(self) -> TGVertex:
        return self._target_vertex

    def get_vertices(self) -> List:
        return self._vertices


class SubGraph(LTG):

    def __init__(self, ltg: LTG):
        super(SubGraph, self).__init__()
        self._ltg = ltg
        self.source_target_distance = None
        self._closet_vertex_to_target_distance = 10000

        source = ltg.get_source()
        target = ltg.get_target()

        source_vertex = TGVertex(source.point(), "SOURCE")
        self.set_source_vertex(source_vertex)

        target_vertex = TGVertex(target.point(), "TARGET")
        self.set_target_vertex(target_vertex)

        vertices = ltg.get_vertices()

        self.source_target_distance = source_vertex.point().distance(target_vertex.point())
        # add admisible vertices
        for vertex in vertices:
            if vertex.vtype == "INNER":
                if vertex.point().distance(
                        self._target_vertex.point()) < self.source_target_distance:  # then admisible Vertex
                    sub_graph_vertex = TGVertex(vertex.point())
                    self.add_vertex(sub_graph_vertex)

        # edges

        self._closet_vertex_to_target = None

        for vertex in self._vertices:
            # connecting source to vertex
            if vertex.vtype != "SOURCE":
                edge = TGEdge(self._source_vertex, vertex)
                self.add_edge(edge)
                vertex.add_edge(edge)
                self._source_vertex.add_edge(edge)

            # connecting vertex to target
            if vertex.vtype != "TARGET" and vertex.vtype != "SOURCE":
                edge = TGEdge(vertex, self._target_vertex)
                self.add_edge(edge)
                vertex.add_edge(edge)
                self._target_vertex.add_edge(edge)
                distance = self._get_distance_to_target(vertex)
                if distance < self._closet_vertex_to_target_distance:
                    self._closet_vertex_to_target = vertex
                    self._closet_vertex_to_target_distance = distance

    def _get_distance_to_target(self, vertex):

        dx = self._source_vertex.point().distance(vertex.point())
        dh = vertex.point().distance(self._target_vertex.point())
        return dx + dh

    def _get_next_boundry_walk_point(self):
        # self._closet_vertex_to_target
        pass

    def get_closet_point_to_target(self):
        return self._closet_vertex_to_target.point()

    def is_source_local_minima(self):
        return self.source_target_distance < self._closet_vertex_to_target_distance


class AugmentedSubGraph:

    LIDAR_RANGE = 35

    def __init__(self, current_location: Point, target: Point):
        self._current_location = current_location
        self._target = target

        self._source_vertex = None
        self._target_vertex = None

        self._vertices = list()
        self._edges = list()

        self._blocking_obstacle = None

        self._build_obstacle_vertices()
        self._add_source_vertex()
        self._try_to_add_target_vertex()

        self._add_edges()

    def remove_duplicate_vertices(self):
        self._vertices = list(dict.fromkeys(self._vertices))

    def get_vertices(self):
        return self._vertices

    def _get_t_node_position(self, curr_position:  Point, target: Point):
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
                self._blocking_obstacle = obs
                return

        t_node = TGVertex(self._get_t_node_position(curr_pos,target),admissible=True,vtype= "T_NODE")
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

    def is_source_local_minima(self):


    def get_closet_point_to_target(self):

        min_distance = np.float(np.inf)
        point = None
        for v in self._vertices:
            if v.is_admissible:
                distance = self._current_location.distance(v.point()) +\
                           v.get_distance_to_target()
                if min_distance > distance:
                    min_distance = distance
                    point = v.point()

        assert point
        return point, min_distance

    def get_blocking_obstacle(self):
        return self._blocking_obstacle

    def calculate_d_min(self, blocking_obstacle=None):
        d_min, _ = blocking_obstacle.get_closest_point_to_target()
        return d_min

    def find_following_direction(self):
