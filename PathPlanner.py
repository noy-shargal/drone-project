import math
from typing import Tuple
from copy import deepcopy
import numpy
import numpy as np

from ObstaclesCSVReader import ObstaclesCSVReader
from LatticeMap import RepulsionMap, AttractionMap, ObstacleMap


class PathPlanner:

    def __init__(self, start: Tuple, goal: Tuple, d=30.0, k=0.5, grid_unit_size=5.0, q_star=120.0, s=10.0):
        self._obstacles_reader = ObstaclesCSVReader()

        self._polygons_map = self._obstacles_reader.polygons_map
        self._goal = goal
        self._start = start
        self._grid_unit_size = grid_unit_size
        min_x, max_x, min_y, max_y = self._obstacles_reader.get_boundaries()

        self._obstacles_map = ObstacleMap(self._polygons_map, min_x, max_x, min_y, max_y, grid_unit_size, numpy.int8)
        self._repulsion_map = RepulsionMap(min_x, max_x, min_y, max_y, grid_unit_size, numpy.float32,
                                           self._polygons_map, q_star)
        self._attraction_map = AttractionMap(min_x, max_x, min_y, max_y, grid_unit_size, numpy.float32, start,
                                             goal, d)

        self._k = k
        self._s = s

    @property
    def s(self):
        return self._s

    @property
    def k(self):
        return self._k

    @s.setter
    def s(self, new_val):
        self._s = new_val

    @k.setter
    def k(self, new_val):
        self._k = new_val

    def get_boundaries(self):
        return self._obstacles_reader.get_boundaries()

    def _get_local_potential_map(self, position: Tuple):
        local_attraction_map_value = self._attraction_map.get_local_values(*position)
        local_repulsion_map_value = self._repulsion_map.get_local_values(*position)
        return self._k * local_attraction_map_value + self._s * local_repulsion_map_value

    def make_path(self):
        pass

    def get_path_commands(self):
        pass

    @staticmethod
    def _argmin_2D_array(array: numpy.ndarray):
        min = numpy.float(numpy.inf)
        min_i, min_j = -1, -1
        for i in range(3):
            for j in range(3):
                if array[i, j] < min:
                    min = array[i, j]
                    min_i = i
                    min_j = j
        return min_i, min_j

    def next_step(self, curr_position: Tuple):
        potential_map = self._get_local_potential_map(curr_position)
        step_indices = self._argmin_2D_array(potential_map)

        curr_index = self._attraction_map.coord_to_index(*curr_position)
        next_indices = (curr_index[0] + step_indices[0] - 1, curr_index[1] + step_indices[1] - 1)
        next_position = self._attraction_map.index_to_coord(*next_indices)

        return next_position

    def reached_location(self, curr_position: Tuple, target_position: Tuple):
        diff_x = target_position[0] - curr_position[0]
        diff_y = target_position[1] - curr_position[1]
        dist = math.sqrt(diff_x * diff_x + diff_y * diff_y)
        return dist < self._grid_unit_size

    def reached_goal(self, curr_position: Tuple):
        self.reached_location(curr_position, self._goal)

    @property
    def polygons_map(self):
        return deepcopy(self._polygons_map)

    def update_obstacle(self, obstacle_position):
        self._obstacles_map.update_map(*obstacle_position, 1)
