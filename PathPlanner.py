import math
from typing import Tuple, Set
from copy import deepcopy
import numpy

from ObstaclesCSVReader import ObstaclesCSVReader
from LatticeMap import RepulsionMap, AttractionMap, ObstacleMap
from config import current_config


class PathPlanner:

    def __init__(self):
        self._obstacles_reader = ObstaclesCSVReader()

        self._polygons_map = self._obstacles_reader.polygons_map
        self._goal = current_config.end_position
        self._start = current_config.start_position
        self._grid_unit_size = current_config.grid_size
        min_x, max_x, min_y, max_y = self._obstacles_reader.get_boundaries()

        self._obstacles_map = ObstacleMap(self._polygons_map, min_x, max_x, min_y, max_y)
        self._repulsion_map = RepulsionMap(min_x, max_x, min_y, max_y, self._polygons_map)
        self._attraction_map = AttractionMap(min_x, max_x, min_y, max_y)

        self._k = current_config.k
        self._s = current_config.s
        self._q_star = current_config.q_star
        self._unknown_amplification = current_config.unknown_amplification

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

    def next_step(self, curr_position: Tuple, lidar_points=set()):
        potential_map = self._get_local_potential_map(curr_position)
        potential_map += self._unknown_amplification*self._calculate_unknown_environment_potential(curr_position, lidar_points)
        step_indices = self._argmin_2D_array(potential_map)

        curr_index = self._attraction_map.coord_to_index(*curr_position)
        next_indices = (curr_index[0] + step_indices[0] - 1, curr_index[1] + step_indices[1] - 1)
        next_position = self._attraction_map.index_to_coord(*next_indices)

        return next_position

    def reached_location(self, curr_position: Tuple, target_position: Tuple):
        diff_x = target_position[0] - curr_position[0]
        diff_y = target_position[1] - curr_position[1]
        dist = math.sqrt(diff_x * diff_x + diff_y * diff_y)
        return dist < current_config.reach_dist

    def reached_goal(self, curr_position: Tuple):
        self.reached_location(curr_position, self._goal)

    @property
    def polygons_map(self):
        return deepcopy(self._polygons_map)

    def _update_obstacle(self, obstacle_position):
        self._obstacles_map.update_map(*obstacle_position, 1)

    def process_lidar_data(self, lidar_sample):
        if self._obstacles_map.new_obstacle(*lidar_sample):
            print(f"new obstacle", lidar_sample)
            self._update_obstacle(lidar_sample)
            self._update_repulsion(lidar_sample)

    def _update_repulsion(self, lidar_sample):
        self._repulsion_map.update_map(*lidar_sample)

    def new_obstacle(self, lidar_sample):
        return self._obstacles_map.new_obstacle(*lidar_sample)

    def _calculate_unknown_environment_potential(self, curr_position, lidar_points: Set):
        unknown_environment_potential = numpy.zeros((3, 3))
        for i in range(3):
            for j in range(3):
                world_i, world_j = self._attraction_map.coord_to_index(*curr_position)
                x, y = self._attraction_map.index_to_coord(i+world_i-1, j+world_j-1)
                distance = self._calculate_distance(x, y, lidar_points)
                unknown_environment_potential[i, j] = self._repulsion_map.repulsion_by_distance(distance)
        return unknown_environment_potential

    @staticmethod
    def _calculate_distance(x, y, lidar_points):
        min_distance = numpy.inf
        for lidar_point in lidar_points:
            distance = math.sqrt((x-lidar_point[0])**2 + (y-lidar_point[1])**2)
            if distance < min_distance:
                min_distance = distance
        return min_distance
