from shapely.geometry import Point
from Dijkstra import Dijkstra
from MapDrawer import MapDrawer
from Obstacles import Obstacles
from enum import Enum, unique
from Config import config



class ASTARPathPlanner:

    def __init__(self):

        self._astar = Dijkstra()
        self._start_point = config.source
        self._destination_point = config.destination
        self._astar_path = list()
        self._obs = Obstacles()
        self._obs.set_destination_point(config.destination)
        self._load_map()
        self._prepare_astar_path()

    def get_obstacles_object(self):
        return self._obs

    def _load_map(self):

        if config.load_from_json:
            self._obs.load_from_json()
        else:
            self._obs.read_csv()
            self._obs.build_visibility_graph()

        self._obs.set_source(config.source)
        self._obs.set_destination(config.destination)

    def _prepare_astar_path(self):
        src = self._obs.get_source_vertex()
        dst = self._obs.get_destination_vertex()
        found = self._astar.search(src, dst)
        if found:
            self._astar_path = self._astar.get_path()
        else: assert False

        if config.show_map:
            md = MapDrawer()
            md.set_source( self._start_point)
            md.set_destination( self._destination_point)
            polygons = self._obs.get_polygons()
            md.add_polygons(polygons)
            print("Number of polygons is: " + str(len(polygons)))
            edges = self._obs.get_edges()
            #md.add_edges(edges)
            src = self._obs.get_source_vertex()
            dst = self._obs.get_destination_vertex()
            md.set_path(self._astar_path)
            print("Total Distance: " + str(dst.get_distance()))
            print("Number Of vertices visited :" + str(self._astar.get_num_of_vertices_visited()))
            md.show()
            print("Total Distance: " + str(dst.get_distance()))
            print("Number Of vertices visited :" + str(self._astar.get_num_of_vertices_visited()))

    def is_point_in_obstacles_map(self, point: Point):
        return self._obs.is_point_in_obstacles_map(point)

    def get_start(self):
        return config.source

    def get_destination(self):
        return config.destination

    def get_height(self):
        return config.height

    def get_path(self):
        assert self._astar_path is not None
        return self._astar_path