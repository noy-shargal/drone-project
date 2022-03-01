import math
from typing import Tuple
from copy import deepcopy
import numpy
import numpy as np

from ObstaclesCSVReader import ObstaclesCSVReader
from LatticeMap import RepulsionMap, AttractionMap, ObstacleMap


class PathPlanner:

    def __init__(self, start: Tuple, goal: Tuple, d=30.0, k=1.0, grid_unit_size=10.0, q_star=30.0, s=1.0):
        self._obstacles_reader = ObstaclesCSVReader()
        self._polygons_map = self._obstacles_reader.polygons_map
        self._goal = goal
        self._start = start
        min_x, max_x, min_y, max_y = self._obstacles_reader.get_boundaries()

        self._obstacles_map = ObstacleMap(self._polygons_map, min_x, max_x, min_y, max_y, grid_unit_size, numpy.int8)
        self._repulsion_map = RepulsionMap(min_x, max_x, min_y, max_y, grid_unit_size, numpy.float32,
                                           self._polygons_map, q_star, s)
        self._attraction_map = AttractionMap(min_x, max_x, min_y, max_y, grid_unit_size, numpy.float32, start,
                                             goal, d, k)

    def get_boundaries(self):
        return self._obstacles_reader.get_boundaries()

    def _get_local_potential_map(self, position: Tuple):
        local_attraction_map_value = self._attraction_map.get_local_values(*position)
        local_repulsion_map_value = self._repulsion_map.get_local_values(*position)
        return local_attraction_map_value + local_repulsion_map_value

    def make_path(self):
        pass

    def get_path_commands(self):
        pass

    def next_step(self, curr_position: Tuple):
        potential_map = self._get_local_potential_map(curr_position)
        step_indices = np.argmin(potential_map)
        curr_index = self._attraction_map.coord_to_index(*curr_position)
        next_indices = (curr_index[0] + step_indices[0] - 1, curr_index[1] + step_indices[1] - 1)
        next_position = self._attraction_map.index_to_coord(*next_indices)
        return next_position

    def reached_goal(self, curr_position: Tuple):
        diff_x = self._goal[0] - curr_position[0]
        diff_y = self._goal[1] - curr_position[1]
        dist = math.sqrt(diff_x * diff_x + diff_y * diff_y)
        return dist < 11.0

    @property
    def polygons_map(self):
        return deepcopy(self._polygons_map)
