import csv
from datetime import datetime
from typing import List, Dict, Tuple

import Edge
import Vertex
from shapely.geometry import Polygon, Point, box, LineString
from Vertex import Vertex
from Edge import Edge
import json


class PersistentPolygons:
    def __init__(self, polygons: Dict):
        self._polygons = list()
        for poly_id, polygon in polygons.items():
            xs, ys = polygon.exterior.coords.xy
            polygon_list = [poly_id]
            for i in range(len(xs)):
                px = xs[i]
                py = ys[i]
                polygon_list.append([px, py])
            self._polygons.append(polygon_list)

    def to_json(self):
        str_json = json.dumps(self, default=lambda o: o.__dict__, sort_keys=True, indent=4)
        with open('polygons.json', 'w') as f:
            f.write(str_json)

    @staticmethod
    def from_json():
        polygons_map = dict()
        with open('polygons.json', 'r') as f:
            json_object = json.load(f)
            poly_list = json_object["_polygons"]

            for poly in poly_list:
                id = poly[0]
                points_list = list()
                for i in range(1, len(poly)):
                    coords = poly[i]
                    point = Point(coords)
                    points_list.append(point)
                polygon = Polygon(points_list)
                polygons_map[id] = polygon
        return polygons_map

    def get(self):
        return self._polygons


class PersistentGraph:
    def __init__(self):
        self._edges = list()

    def set(self, edges: List):
        for edge in edges:
            e = [edge.one.point().x, edge.one.point().y, edge.two.point().x, edge.two.point().y]
            self._edges.append(e)

        # for vertex in vertices:
        #     # v = PersistentVertex(vertex.point().x, vertex.point().y)
        #     v = [vertex.point().x, vertex.point().y]
        #     self._vertices.append(v)

    def get(self):
        return self._edges

    def to_json(self):
        str_json = json.dumps(self, default=lambda o: o.__dict__, sort_keys=True, indent=4)
        with open('obs.json', 'w') as f:
            f.write(str_json)

    def from_json(self):
        with open('obs.json', 'r') as f:
            json_object = json.load(f)
            self._edges = json_object["_edges"]
            # self._vertices = json_object["_vertices"]
            return self._edges


class Obstacles:

    def __init__(self):
        self._points_map = dict()
        self._polygons_map = dict()
        self._is_map_inited = False
        self._source = None
        self._destination = None
        self._edges_list = list()
        self._vertices = dict()
        self._vertices_list = list()
        self._destination_point = None
        self._graph_is_built = False

    def get_obstacle_points_list(self, id):

        if id not in self._points_map.keys():
            self._points_map[id] = list()
        return self._points_map[id]

    def is_point_in_obstacles_map(self, point: Point):
        for obstacle in self._polygons_map.values():
            if obstacle.contains(point):
                return True
        return False

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

    #########################Read from JSON METHODS: use 'load_from_json' ###########################

    def _add_polygon_vertices(self, points_list: List, polygon: Polygon):

        assert self._destination_point
        for p in points_list:
            vertex = Vertex(p, polygon, self._destination_point)
            #vertex = Vertex(p, polygon, None)
            x, y = p.x, p.y
            key = x, y
            self._vertices[key] = vertex
            self._vertices_list.append(vertex)

    def _add_polygon(self, poly_record):
        poly_id = poly_record[0]
        points_list = list()

        for i in range(1, len(poly_record)):
            coords = poly_record[i]
            point = Point(coords)
            points_list.append(point)

        polygon = Polygon(points_list)
        self._add_polygon_vertices(points_list, polygon)
        self._polygons_map[poly_id] = polygon

    def set_destination_point(self, dest: Point):
        self._destination_point = dest

    def read_polygons_json(self):
        with open('polygons.json', 'r') as f:
            json_object = json.load(f)
            poly_list = json_object["_polygons"]
            for poly_record in poly_list:
                self._add_polygon(poly_record)

    def read_edges(self):
        pp = PersistentGraph()
        es = pp.from_json()
        for e in es:
            key1 = e[0], e[1]
            v1 = self._vertices[key1]
            key2 = e[2], e[3]
            v2 = self._vertices[key2]
            new_edge = Edge(v1, v2)
            self._edges_list.append(new_edge)
            v1.add_edge(new_edge)
            v2.add_edge(new_edge)

    def load_from_json(self):
        self.read_polygons_json()
        self.read_edges()
        self._graph_is_built = True

    def save_to_json(self):
        persist_poly = PersistentPolygons(self._polygons_map)
        persist_poly.to_json()
        graph = PersistentGraph()
        graph.set(self._edges_list)
        graph.to_json()

    #######################################################################################################################

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

            poly_json = PersistentPolygons(self._polygons_map)
            poly_json.to_json()

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


        if not self._is_edge_tangent(edge):
            return False

        p1 = edge.one.point()
        p2 = edge.two.point()
        line = LineString([p1, p2])

        for poly in self._polygons_map.values():
            ints = line.intersection(poly)
            if len(ints.coords) > 1:
                return False

        return True

    def is_valid_edge_lite(self, edge: Edge):

        p1 = edge.one.point()
        p2 = edge.two.point()
        line = LineString([p1, p2])

        for poly in self._polygons_map.values():
            ints = line.intersection(poly)
            if len(ints.coords) > 1:
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
            self._edges_list.append(edge)

        x1, y1 = points[len(points) - 1].x, points[len(points) - 1].y
        v1 = self._vertices[x1, y1]

        x2, y2 = points[0].x, points[0].y
        v2 = self._vertices[x2, y2]

        edge = Edge(v1, v2)
        self._edges_list.append(edge)

    def _add_polygons_edges(self):

        for poly in self._polygons_map.values():
            self._add_polygon_edges(poly)

    def _build_vertices(self):

        for poly_id, points in self._points_map.items():
            for point in points:
                polygon = self._polygons_map[poly_id]
                vertex = Vertex(point, polygon, None)
                x, y = point.x, point.y
                key = x, y
                self._vertices[key] = vertex

    def set_vertices(self, vertices: List):
        self._vertices = vertices

    def set_edges(self, edges: List):
        self._edges_list = edges

    def get_edges(self):
        return self._edges_list

    def get_vertices(self):
        return self._vertices_list

    def build_visibility_graph(self):

        self._build_vertices()
        self._vertices_list.extend(list(self._vertices.values()))
        combinations = Obstacles._get_combinations(self._vertices)
        for one, two in combinations:
            edge = Edge(one, two)
            if self.is_valid_edge(edge):
                one.add_edge(edge)
                two.add_edge(edge)
                self._edges_list.append(edge)

        self._add_polygons_edges()
        print("Number of edges = " + str(len(self._edges_list)))
        self._graph_is_built = True

    def _attach_vertex_to_graph(self, vertex: Vertex):
        assert self._graph_is_built
        t1 = datetime.now()
        for i in range(len(self._vertices_list)):
            edge = Edge(self._vertices_list[i], vertex)
            if self.is_valid_edge_lite(edge):
                self._vertices_list[i].add_edge(edge)
                vertex.add_edge(edge)
                self._edges_list.append(edge)
        self._vertices[vertex.point().x, vertex.point().y] = vertex
        self._vertices_list.append(vertex)

        t2 = datetime.now()
        print("_attach_vertex_to_graph: " + str(t2 - t1))
        print(type(t1 - t2))

    def set_source(self, source: Point):
        poly = Polygon([source, source, source, source])
        self._source = Vertex(source, poly)
        self._attach_vertex_to_graph(self._source)

    def set_destination(self, destination: Point):
        poly = Polygon([destination, destination, destination, destination])
        self._destination = Vertex(destination, poly)
        self._attach_vertex_to_graph(self._destination)

    def add_new_obstacle(self, point1: Point, point2: Point, point3: Point):
        points = [point1, point2, point3]
        polygon = Polygon(points)
        bbox = polygon.bounds
        padding = 4
        box_polygon = box(bbox[0] - padding, bbox[1] - padding, bbox[2] + padding, bbox[3] + padding)
        points_list = box_polygon.exterior.coords
        p1, p2, p3, p4 = points_list[0], points_list[1], points_list[2], points_list[3]
        vertex1 = Vertex(Point(p1[0], p1[1]), box_polygon)
        vertex2 = Vertex(Point(p2[0], p2[1]), box_polygon)
        vertex3 = Vertex(Point(p3[0], p3[1]), box_polygon)
        vertex4 = Vertex(Point(p4[0], p4[1]), box_polygon)
        self._attach_vertex_to_graph(vertex1)
        vertex1.set_h(10000)
        self._attach_vertex_to_graph(vertex2)
        vertex2.set_h(10000)
        self._attach_vertex_to_graph(vertex3)
        vertex3.set_h(10000)
        self._attach_vertex_to_graph(vertex4)
        vertex4.set_h(10000)

    def get_source_vertex(self):
        return self._source

    def get_destination_vertex(self):
        return self._destination

    def reset_vertices_data_for_search(self):
        for v in self._vertices_list:
            v.set_distance(0.0)

