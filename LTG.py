from typing import List
from shapely.geometry import Point


class TGEdge:
    def __init__(self, vertex1: TGVertex, vertex2: TGVertex):
        self._vertex1 = vertex1
        self._vertex2 = vertex2

    def get_distance(self):
        return self._vertex1.distance(self._vertex2)


class TGVertex:

    def __init__(self,point: Point):
        self._edges = list()
        self._point = point

    def add_edge(self, edge):
        self._edges.append(edge)

    def get_edges(self):
        return self._edges

    def get_coordinates(self):
        return self._point.x, self._point.y

    def distance(self, vertex: TGVertex):
        return self._point.distance(vertex._point)


class LTG:

    def __init__(self):
        self._edges = list()
        self._vertices = list()

    def add_edges(self, edges: List):
        self._edges.extend(edges)

    def add_edge(self, edge: TGEdge):
        self._edges.append(edge)

    def add_vertices(self, vertices: List):
        self._vertices.extend(vertices)

    def add_vertex(self, vertex: TGVertex):
        self._vertices.append(vertex)












