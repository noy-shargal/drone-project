import math
from typing import Tuple, Set
from copy import deepcopy
import numpy
import numpy as np
from apf.ObstaclesCSVReader import ObstaclesCSVReader
from apf.LatticeMap import RepulsionMap, AttractionMap, ObstacleMap
from apf.config import current_config
from Config import config


class APFPathPlanner:

    def __init__(self, start_position, end_position):
        self._obstacles_reader = ObstaclesCSVReader()

        self._polygons_map = self._obstacles_reader.polygons_map
        self._goal = end_position
        self._real_goal = end_position
        self._start = start_position

        self._grid_unit_size = current_config.grid_size
        # min_x, max_x, min_y, max_y = self._obstacles_reader.get_boundaries()
        min_x, max_x, min_y, max_y = self.calculate_boundaries(start_position, end_position)
        self._obstacles_map = ObstacleMap(self._polygons_map, min_x, max_x, min_y, max_y)
        self._repulsion_map = RepulsionMap(min_x, max_x, min_y, max_y, self._polygons_map)
        self._attraction_map = AttractionMap(start_position, end_position, min_x, max_x, min_y, max_y)

        self._k = current_config.k
        self._s = current_config.s
        self._q_star = current_config.q_star
        self._unknown_amplification = current_config.unknown_amplification

    def set_goal(self, new_goal):
        self._goal = new_goal
        self._attraction_map.set_goal(new_goal)
        return

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

    def _get_local_potential_map(self, position: Tuple, use_repulsion = True):
        local_attraction_map_value = self._attraction_map.get_local_values(*position)
        local_repulsion_map_value = self._repulsion_map.get_local_values(*position)
        if use_repulsion:
            return self.weighted_average_of_dicts(local_attraction_map_value, self._k, local_repulsion_map_value, self._s)
        return local_attraction_map_value

    @staticmethod
    def weighted_average_of_dicts(dict1, weight1, dict2, weight2):
        output = dict()
        for position in dict1.keys():
            output[position] = weight1 * dict1[position] + weight2 * dict2[position]
        return output

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

    def next_step(self, curr_position: Tuple, lidar_points=set):
        distance_to_nearest_obstacle = self._calculate_distance(curr_position[0], curr_position[1], lidar_points)
        current_config.window_size = self._compute_window_size(distance_to_nearest_obstacle)
        potential_map = self._get_local_potential_map(curr_position)
        unknown_potential_map = self._calculate_unknown_environment_potential(curr_position, lidar_points)
        total_potential = self.weighted_average_of_dicts(potential_map, 1, unknown_potential_map,
                                                         self._unknown_amplification)

        step_indices = min(total_potential, key=total_potential.get)

        next_position = self._attraction_map.index_to_coord(*step_indices)

        return next_position

    def next_step_no_attraction(self, curr_position: Tuple, lidar_points=set):
        distance_to_nearest_obstacle = self._calculate_distance(curr_position[0], curr_position[1], lidar_points)
        current_config.window_size = self._compute_window_size(distance_to_nearest_obstacle)
        potential_map = self._get_local_potential_map(curr_position, False)
        unknown_potential_map = self._calculate_unknown_environment_potential(curr_position, lidar_points)
        total_potential = self.weighted_average_of_dicts(potential_map, 1, unknown_potential_map,
                                                         self._unknown_amplification)

        step_indices = min(total_potential, key=total_potential.get)

        next_position = self._attraction_map.index_to_coord(*step_indices)

        return next_position

    def next_step_no_repulsion(self, curr_position: Tuple, lidar_points=set):
        distance_to_nearest_obstacle = self._calculate_distance(curr_position[0], curr_position[1], lidar_points)
        current_config.window_size = self._compute_window_size(distance_to_nearest_obstacle)
        potential_map = self._get_local_potential_map(curr_position, False)
        unknown_potential_map = self._calculate_unknown_environment_potential(curr_position, lidar_points)
        total_potential = self.weighted_average_of_dicts(potential_map, 1, unknown_potential_map,
                                                         self._unknown_amplification)

        step_indices = min(total_potential, key=total_potential.get)

        next_position = self._attraction_map.index_to_coord(*step_indices)

        return next_position


    def reached_location(self, curr_position: Tuple, target_position: Tuple):
        diff_x = target_position[0] - curr_position[0]
        diff_y = target_position[1] - curr_position[1]
        dist = math.sqrt(diff_x * diff_x + diff_y * diff_y)
        return dist < current_config.reach_dist

    def reached_goal(self, curr_position: Tuple):
        return self.reached_location(curr_position, self._real_goal)

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

    def add_new_obstacle(self, lidar_sample):
        return self._obstacles_map.update_map(*lidar_sample, 2)

    def _calculate_unknown_environment_potential(self, curr_position, lidar_points: Set):
        output = dict()
        world_i, world_j = self._attraction_map.coord_to_index(*curr_position)
        window_size = current_config.window_size

        # Top
        for x_iterator in range(-(window_size - 1) // 2, window_size // 2 + 1, 1):
            position = (world_i + x_iterator, world_j - window_size // 2)
            x, y = self._attraction_map.index_to_coord(position[0], position[1])
            distance = self._calculate_distance(x, y, lidar_points)
            output[position] = self._repulsion_map.repulsion_by_distance(distance)
        # Bottom
        for x_iterator in range(-(window_size - 1) // 2, window_size // 2 + 1, 1):
            position = (world_i + x_iterator, world_j + window_size // 2)
            x, y = self._attraction_map.index_to_coord(position[0], position[1])
            distance = self._calculate_distance(x, y, lidar_points)
            output[position] = self._repulsion_map.repulsion_by_distance(distance)
        # Left
        for y_iterator in range(-(window_size - 1) // 2 + 1, window_size // 2, 1):
            position = (world_i - window_size // 2, world_j + y_iterator)
            x, y = self._attraction_map.index_to_coord(position[0], position[1])
            distance = self._calculate_distance(x, y, lidar_points)
            output[position] = self._repulsion_map.repulsion_by_distance(distance)
        # Right
        for y_iterator in range(-(window_size - 1) // 2 + 1, window_size // 2, 1):
            position = (world_i + window_size // 2, world_j + y_iterator)
            x, y = self._attraction_map.index_to_coord(position[0], position[1])
            distance = self._calculate_distance(x, y, lidar_points)
            output[position] = self._repulsion_map.repulsion_by_distance(distance)
        return output

    @staticmethod
    def _calculate_distance(x, y, lidar_points):
        min_distance = numpy.inf
        for lidar_point in lidar_points:
            distance = math.sqrt((x - lidar_point[0]) ** 2 + (y - lidar_point[1]) ** 2)
            if distance < min_distance:
                min_distance = distance
        return np.max((min_distance - current_config.lidar_padding, current_config.lidar_padding))

    def _compute_window_size(self, distance_to_nearest_obstacle):
        window_size = int(min(30, distance_to_nearest_obstacle) / self._grid_unit_size)

        if window_size % 2 == 0:
            window_size -= 1

        window_size -= 2
        return max(3, window_size)

    def calculate_boundaries(self, start_position, end_position):
        padding = 60
        min_x = np.min((start_position[0], end_position[0])) - padding
        min_y = np.min((start_position[1], end_position[1])) - padding
        max_x = np.max((start_position[0], end_position[0])) + padding
        max_y = np.max((start_position[1], end_position[1])) + padding
        return min_x, max_x, min_y, max_y
