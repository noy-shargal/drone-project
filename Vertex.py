from shapely.geometry import Point
import Edge


class Vertex:

    def __init__(self, point: Point):
        self._point = point
        self._distance_from_source = 1000000.0  # Infinity
        self._h_for_distance_from_goal = None
        self._edges = list()
        self._is_visited = False
        self._previous_vertex = None

    def __eq__(self, other):
        return self._point == other.point()

    def __lt__(self, other):
        return self._distance_from_source < other.get_distance()

    def point(self):
        return self._point

    def add_edge(self, edge: Edge):
        self._edges.append(edge)

    def get_edges(self):
        return self._edges

    def is_visited(self):
        return self._is_visited

    def visit(self):
        self._is_visited = True

    def get_distance(self):
        return self._distance_from_source

    def set_distance(self, distance):
        self._distance_from_source = distance

    def get_h(self):
        return self._h_for_distance_from_goal

    def set_h(self, distance):
        self._h_for_distance_from_goal = distance

    def get_prev_vertex(self):
        return self._previous_vertex

    def set_prev_vertex(self, prev_vertex):
        self._previous_vertex = prev_vertex
