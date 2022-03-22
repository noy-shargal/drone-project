from typing import List
from shapely.geometry import Point




class TGEdge:
    def __init__(self, vertex1, vertex2):
        self._vertex1 = vertex1
        self._vertex2 = vertex2

    def get_distance(self):
        return self._vertex1.distance(self._vertex2)


class TGVertex:


    def __init__(self,point: Point, vtype = "INNER"):
        super().__init__()
        self._edges = list()
        self._point = point
        self.vtype = vtype

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

    def  __init__(self, ltg: LTG):
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

        self.source_target_distance = source_vertex.point().distance( target_vertex.point())
        # add admisible vertices
        for vertex in vertices:
           if vertex.vtype == "INNER":
               if vertex.point().distance( self._target_vertex.point()) < self.source_target_distance: # then admisible Vertex
                sub_graph_vertex = TGVertex( vertex.point())
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
        #self._closet_vertex_to_target
        pass

    def get_closet_point_to_target(self):
        return self._closet_vertex_to_target.point()

    def is_source_local_minima(self):
        return self.source_target_distance < self._closet_vertex_to_target_distance

















