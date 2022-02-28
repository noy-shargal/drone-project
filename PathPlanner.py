from typing import Tuple

import numpy

from ObstaclesCSVReader import ObstaclesCSVReader
from LatticeMap import RepulsionMap, AttractionMap


class PathPlanner:

    def __init__(self, start: Tuple, goal: Tuple, d=30.0, k=1.0, grid_unit_size=10.0, q_star=30.0, s=1.0):
        self._obstacles_reader = ObstaclesCSVReader()
        self._polygons_map = self._obstacles_reader.polygons_map

        min_x, max_x, min_y, max_y = self._obstacles_reader.get_boundaries()

        self._obstacles_map = None
        self._repulsion_map = RepulsionMap(min_x, max_x, min_y, max_y, grid_unit_size, numpy.float32,
                                           self._polygons_map, q_star, s)
        self._attraction_map = AttractionMap(min_x, max_x, min_y, max_y, grid_unit_size,  numpy.float32, start, goal, d, k)

    def make_path(self):
        pass

    def get_path_commands(self):
        pass

    def next_step(self, curr_position: Tuple):
        pass

    def reached_goal(self):
        pass

