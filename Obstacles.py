import csv
from typing import List

import Edge
import Vertex
from shapely.geometry import Polygon, Point, box, LineString
from Vertex import Vertex
from Edge import Edge


class Graph:
    def __init__(self):
        self._vertices = None
        self._edges = None

    def set(self, edges: List, vertices: List):
        self._edges = edges
        self._vertices = vertices

    def save(self):
        pass


class Obstacles:

    def __init__(self):
        self._points_map = dict()
        self._polygons_map = dict()
        self._is_map_inited = False
        self._source = None
        self._destination = None
        self._edges = list()
        self._vertices = dict()
        self._source_vertex = None
        self._destination_vertex = None

        self._graph = Graph()

    def get_obstacle_points_list(self, id):

        if id not in self._points_map.keys():
            self._points_map[id] = list()
        return self._points_map[id]

    @staticmethod
    def _get_coord(row):
        x_str = row[1]
        y_str = row[2]
        x = float(x_str)
        y = float(y_str)
        point = Point(x, y)
        return point

    @staticmethod
    def _coords_2_points(coords):
        x, y = coords.xy
        new_pts = []
        for i in range(len(x)):
            px = x[i]
            py = y[i]
            point = Point(px, py)
            if point not in new_pts:
                new_pts.append(point)

        return new_pts

    def read_csv(self):
        with open('obstacles_100m_above_sea_level.csv', newline='') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            next(reader, None)  # skip the headers
            for row in reader:
                obs_id = row[4]
                obs_list = self.get_obstacle_points_list(obs_id)
                point = Obstacles._get_coord(row)
                obs_list.append(point)

            for polygon_id, points in self._points_map.items():
                polygon = Polygon(points)
                bbox = polygon.bounds
                padding = 4
                box_polygon = box(bbox[0] - padding, bbox[1] - padding, bbox[2] + padding, bbox[3] + padding)
                coords = box_polygon.exterior.coords
                new_pts = self._coords_2_points(coords)
                self._points_map[polygon_id] = new_pts
                self._polygons_map[polygon_id] = box_polygon

            self._is_map_inited = True

    def is_position_blocked(self, x, y, z):
        key = x, y, z
        return self._points_map[key] is not None

    def get_polygons(self):
        return self._polygons_map

    def get_polygons_vertices(self):

        vertices = list()
        for points in self._points_map.values():
            vertices.extend(points)

        return vertices

    # def is_poin
    @staticmethod
    def _is_line_tangent(p1: Point, p2: Point, p1_vertex: Vertex):

        if p1.x == p2.x:  # TODO: for a = 0
            return False

        polygon = p1_vertex.polygon()
        a = (p1.y - p2.y) / (p1.x - p2.x)
        b = p1.y - a * p1.x

        p_near_p1_x = p1.x + 1
        p_near_p1_y = a * p_near_p1_x + b
        p_near_p1 = Point(p_near_p1_x, p_near_p1_y)

        if polygon.contains(p_near_p1):
            return False

        p_near_p1_x = p1.x - 1
        p_near_p1_y = a * p_near_p1_x + b
        p_near_p1 = Point(p_near_p1_x, p_near_p1_y)

        if polygon.contains(p_near_p1):
            return False

        return True

    @staticmethod
    def _is_edge_tangent(edge: Edge):
        p1 = edge.one.point()
        p2 = edge.two.point()
        if not Obstacles._is_line_tangent(p1, p2, edge.one):
            return False

        if not Obstacles._is_line_tangent(p2, p1, edge.two):
            return False

        return True

    def is_valid_edge(self, edge: Edge):
        p1 = edge.one.point()
        p2 = edge.two.point()
        if type(p1) != Point:
            print("type mismatch")
        line = LineString([p1, p2])

        for poly in self._polygons_map.values():
            ints = line.intersection(poly)
            if len(ints.coords) > 1:
                return False

            if not self._is_edge_tangent(edge):
                return False

        return True

    @staticmethod
    def _get_combinations(vertices):

        combs = []
        vertices_list = list(vertices.values())
        for i in range(len(vertices_list)):
            for j in range(i + 1, len(vertices_list)):
                pair = vertices_list[i], vertices_list[j]
                if vertices_list[i] == vertices_list[j]:
                    i = 9
                combs.append(pair)
        return combs

    def _add_polygon_edges(self, polygon: Polygon):
        points = self._coords_2_points(polygon.exterior.coords)

        for i in range(len(points) - 1):
            x1, y1 = points[i].x, points[i].y
            v1 = self._vertices[x1, y1]

            x2, y2 = points[i + 1].x, points[i + 1].y
            v2 = self._vertices[x2, y2]
            edge = Edge(v1, v2)
            self._edges.append(edge)

        x1, y1 = points[len(points) - 1].x, points[len(points) - 1].y
        v1 = self._vertices[x1, y1]

        x2, y2 = points[0].x, points[0].y
        v2 = self._vertices[x2, y2]

        edge = Edge(v1, v2)
        self._edges.append(edge)

    def _add_polygons_edges(self):

        for poly in self._polygons_map.values():
            self._add_polygon_edges(poly)

    def _build_vertices(self):



        for poly_id, points in self._points_map.items():
            for point in points:
                polygon = self._polygons_map[poly_id]

                vertex = Vertex(point, polygon, self._destination_vertex.point())

                if poly_id == 'start' and self._source_vertex is None:
                    self._source_vertex = vertex
                if poly_id == 'destination' and self._destination_vertex is None:
                    self._destination_vertex = vertex


                x, y = point.x, point.y
                key = x, y
                self._vertices[key] = vertex

    def build_visibility_graph(self):

        self._build_vertices()

        combinations = Obstacles._get_combinations(self._vertices)
        for one, two in combinations:
            edge = Edge(one, two)
            if self.is_valid_edge(edge):
                one.add_edge(edge)
                two.add_edge(edge)
                self._edges.append(edge)

        self._add_polygons_edges()
        print("Number of edges = " + str(len(self._edges)))
        self._graph.set(self._edges, list(self._vertices.values()))


    def get_edges(self):
        return self._edges

    def set_start(self, start):
        self._source = start
        self._points_map['start'] = [start]

    def set_destination(self, destination):
        self._destination = destination
        self._points_map['destination'] = [destination]

    def set_start_polygon(self, source):
        self._polygons_map['start'] = source
        pts = self._coords_2_points(source.exterior.coords)
        self._points_map['start'] = pts

    def set_destination_polygon(self, destination):
        self._polygons_map['destination'] = destination
        pts = self._coords_2_points(destination.exterior.coords)
        self._points_map['destination'] = pts

    def get_source_vertex(self):
        return self._source_vertex

    def get_destination_vertex(self):
        return self._destination_vertex
