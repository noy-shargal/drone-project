from shapely.geometry import Point, Polygon
import Edge, math


class Vertex:

    def __init__(self, point: Point, polygon: Polygon, goal=None ):
        self._settled = False
        self._point = point
        self._distance_from_source = 1000000.0  # Infinity

        self._edges = list()
        self._is_visited = False
        self._previous_vertex = None
        self._polygon = polygon

        if goal is not None:
            self._h_for_distance_from_goal = math.dist([point.x, point.y], [goal.x, goal.y])

    def __eq__(self, other):
        return self._point == other.point()

    def __lt__(self, other):
        return self._distance_from_source < other.get_distance()

    def polygon(self):
        return self._polygon

    def point(self):
        return self._point

    def add_edge(self, edge: Edge):
        self._edges.append(edge)

    def get_edges(self):
        return self._edges

    def is_settled(self):
        return self._settled

    def settle(self):
        self._settled = True

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
